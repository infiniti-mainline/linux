// SPDX-License-Identifier: GPL-2.0-only
/*
 *  Driver for Synaptics TCM Oncell Touchscreens
 *
 *  Copyright (c) 2024 Frieder Hannenheim <frieder.hannenheim@proton.me>
 *  Copyright (c) 2024 Caleb Connolly <caleb@postmarketos.org>
 */

#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/interrupt.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>

/*
 * The TCM oncell interface uses a command byte followed by a 16-bit LE
 * payload length (see tcm_send_cmd_noargs() below).
 *
 * The following list only defines commands that are used in this driver (and their
 * counterparts for context). Vendor reference implementations can be found at
 * https://github.com/LineageOS/android_kernel_oneplus_sm8250/tree/ee0a7ee1939ffd53000e42051caf8f0800defb27/drivers/input/touchscreen/synaptics_tcm
 */

/*
 * Request information about the chip. We don't send this command explicitly as
 * the controller automatically sends this information when starting up.
 */
#define TCM_IDENTIFY				0x02

/* Enable/disable reporting touch inputs */
#define TCM_ENABLE_REPORT			0x05
#define TCM_DISABLE_REPORT			0x06

/*
 * After powering on, we send this to exit the bootloader mode and run the main
 * firmware.
 */
#define TCM_RUN_APPLICATION_FIRMWARE		0x14

/*
 * Reports information about the vendor provided application firmware. This is
 * also used to determine when the firmware has finished booting.
 */
#define TCM_GET_APPLICATION_INFO		0x20

#define MODE_APPLICATION			0x01

#define APP_STATUS_OK				0x00
#define APP_STATUS_BOOTING			0x01
#define APP_STATUS_UPDATING			0x02

/* status codes */
#define REPORT_IDLE				0x00
#define REPORT_OK				0x01
#define REPORT_BUSY				0x02
#define REPORT_CONTINUED_READ			0x03
#define REPORT_RECEIVE_BUFFER_OVERFLOW		0x0c
#define REPORT_PREVIOUS_COMMAND_PENDING		0x0d
#define REPORT_NOT_IMPLEMENTED			0x0e
#define REPORT_ERROR				0x0f

/* report types */
#define REPORT_IDENTIFY				0x10
#define REPORT_TOUCH				0x11
#define REPORT_DELTA				0x12
#define REPORT_RAW				0x13
#define REPORT_DEBUG				0x14
#define REPORT_LOG				0x1d
#define REPORT_TOUCH_HOLD			0x20
#define REPORT_INVALID				0xff

struct tcm_message_header {
	u8 marker;
	u8 code;
	__le16 length;
} __packed;

struct tcm_identification {
	struct tcm_message_header header;
	u8 version;
	u8 mode;
	char part_number[16];
	u8 build_id[4];
	u8 max_write_size[2];
} __packed;

struct tcm_app_info {
	struct tcm_message_header header;
	u8 version[2];
	__le16 status;
	u8 static_config_size[2];
	u8 dynamic_config_size[2];
	u8 app_config_start_write_block[2];
	u8 app_config_size[2];
	u8 max_touch_report_config_size[2];
	u8 max_touch_report_payload_size[2];
	char customer_config_id[16];
	__le16 max_x;
	__le16 max_y;
	u8 max_objects[2];
	u8 num_of_buttons[2];
	u8 num_of_image_rows[2];
	u8 num_of_image_cols[2];
	u8 has_hybrid_data[2];
} __packed;

#define TCM_BUF_SIZE	256

struct tcm_data {
	struct spi_device *spi;
	struct input_dev *input;
	struct gpio_desc *reset_gpio;
	struct completion response;
	struct touchscreen_properties props;
	struct regulator_bulk_data supplies[2];

	/* annoying state */
	u16 buf_size;
	char buf[TCM_BUF_SIZE];

	/*
	 * DMA-safe transfer buffers (the QUP this sits on can do GPI DMA,
	 * so SPI buffers must not live on the stack or mid-struct).
	 * tx_fill is clocked out on MOSI during reads: the vendor SPI
	 * transport (synaptics_tcm_spi.c in the tree referenced above)
	 * fills the TX buffer with 0xff for reads, so do the same rather
	 * than let the controller shift out zeros. rx_buf is only touched
	 * from the IRQ thread. tx_cmd is protected by the response
	 * completion flow (one command in flight at a time).
	 */
	u8 *tx_fill;
	u8 *rx_buf;
	u8 *tx_cmd;
};

/*
 * A TCM command packet is a command byte followed by a 16-bit LE payload
 * length. The original (I2C, OnePlus 8T) version of this driver sent only
 * the bare command byte for zero-length commands; the vendor SPI transport
 * always writes the full 3-byte header, so do the same here.
 *
 * TODO(hw-verify): confirm on the S3910 that the two length bytes are
 * required (or at least harmless) on SPI.
 */
static int tcm_send_cmd_noargs(struct tcm_data *tcm, u8 cmd)
{
	dev_dbg(&tcm->spi->dev, "sending command %#x\n", cmd);

	tcm->tx_cmd[0] = cmd;
	tcm->tx_cmd[1] = 0;
	tcm->tx_cmd[2] = 0;

	return spi_write(tcm->spi, tcm->tx_cmd, 3);
}

static int tcm_recv_report(struct tcm_data *tcm,
			   void *buf, size_t length)
{
	struct spi_transfer xfer = {
		.tx_buf = tcm->tx_fill,
		.rx_buf = buf,
		.len = length,
	};

	if (WARN_ON(length > TCM_BUF_SIZE))
		return -EINVAL;

	/*
	 * TODO(hw-verify): the vendor driver optionally inserts a per-byte
	 * delay (synaptics,byte-delay-us) between single-byte transfers.
	 * The downstream infiniti DT was not seen setting one, so a single
	 * transfer is used here. Revisit if reads come back corrupted.
	 */
	return spi_sync_transfer(tcm->spi, &xfer, 1);
}

static int tcm_read_message(struct tcm_data *tcm, u8 cmd, void *buf, size_t length)
{
	int ret;

	reinit_completion(&tcm->response);
	ret = tcm_send_cmd_noargs(tcm, cmd);
	if (ret)
		return ret;

	ret = wait_for_completion_timeout(&tcm->response, msecs_to_jiffies(1000));
	if (ret == 0)
		return -ETIMEDOUT;

	if (buf) {
		if (length > tcm->buf_size) {
			dev_warn(&tcm->spi->dev, "expected %zu bytes, got %u\n",
				 length, tcm->buf_size);
		}
		length = min(tcm->buf_size, length);
		memcpy(buf, tcm->buf, length);
	}

	return 0;
}

static void tcm_power_off(void *data)
{
	struct tcm_data *tcm = data;

	disable_irq(tcm->spi->irq);
	regulator_bulk_disable(ARRAY_SIZE(tcm->supplies), tcm->supplies);
}

static int tcm_input_open(struct input_dev *dev)
{
	struct tcm_data *tcm = input_get_drvdata(dev);

	return tcm_send_cmd_noargs(tcm, TCM_ENABLE_REPORT);
}

static void tcm_input_close(struct input_dev *dev)
{
	struct tcm_data *tcm = input_get_drvdata(dev);
	int ret;

	ret = tcm_send_cmd_noargs(tcm, TCM_DISABLE_REPORT);
	if (ret)
		dev_err(&tcm->spi->dev, "failed to turn off sensing\n");
}

/*
 * REPORT_TOUCH (0x11) payload layout as observed on the S3910 (OnePlus 15),
 * reverse-engineered from live single-finger captures (2026-07-21):
 *
 *   a5 11 2b 00                                  header, payload length 43
 *   00 x 31                                      frame-level data, offset 0..30
 *                                                (all zero in the captures;
 *                                                per the TCM report config
 *                                                these are frame fields such
 *                                                as gesture data / timestamp
 *                                                that precede the object loop)
 *   10 | c5 15 | 7d 56 | 09 | 90 | 00 x 5        one 12-byte object record
 *
 * Object record fields, matching the vendor TCM parser's field codes
 * (TOUCH_OBJECT_N_INDEX 4 bits, then _CLASSIFICATION 4 bits, LSB-first,
 * so index sits in the low nibble):
 *
 *   +0       index : 4, classification : 4   (0x10 = index 0, FINGER)
 *   +1 .. 2  X, little-endian 16             (0 .. max_x, 12719 on this unit)
 *   +3 .. 4  Y, little-endian 16             (0 .. max_y, 27719 on this unit)
 *   +5       Z                               (0x07..0x0a observed)
 *   +6       width-related                   (0x70..0xa0 observed; exact
 *                                            sub-layout not yet known)
 *   +7 .. 11 zero in all captures            (presumably more width/tx/rx)
 *
 * Observed payload lengths: 31 (no active object), 43 (one), 55 (two) --
 * i.e. 31 frame bytes plus 12 bytes per active object. This is consistent
 * with a report config using TOUCH_FOREACH_ACTIVE_OBJECT (the config itself
 * has not been read out of the chip): only active objects are present and
 * the object count follows from the payload length. Only the one-object
 * form has been captured raw so far; the 55 = two-object reading is an
 * arithmetic inference from the length histogram.
 *
 * This is hardcoded for the report config the S3910 ships with. To support
 * other configs we would need to read the config and parse dynamically.
 */

#define TCM_TOUCH_FRAME_BYTES		31

/* classification (HIGH nibble of an object record's first byte) values */
#define TCM_OBJ_CLASS_NO_OBJECT		0

/* must match input_mt_init_slots() in tcm_probe() */
#define TCM_MAX_OBJECTS			10

struct tcm_report_object {
	u8 desc; /* index : 4, classification : 4 */
	__le16 x;
	__le16 y;
	u8 z;
	u8 width; /* sub-layout unknown, see comment above */
	u8 unknown[5];
} __packed;

static_assert(sizeof(struct tcm_report_object) == 12);

static int tcm_handle_touch_report(struct tcm_data *tcm, const char *buf, size_t len)
{
	const struct tcm_report_object *obj;
	size_t num_objects;

	/* If the input device hasn't registered yet then we can't do anything */
	if (!tcm->input)
		return 0;

	if (len < sizeof(struct tcm_message_header))
		return 0;

	buf += sizeof(struct tcm_message_header);
	len -= sizeof(struct tcm_message_header);

	dev_dbg(&tcm->spi->dev, "touch report len %zu\n", len);
	if (len < TCM_TOUCH_FRAME_BYTES ||
	    (len - TCM_TOUCH_FRAME_BYTES) % sizeof(*obj)) {
		dev_err_ratelimited(&tcm->spi->dev,
				    "invalid touch report length %zu\n", len);
		return 0;
	}

	num_objects = (len - TCM_TOUCH_FRAME_BYTES) / sizeof(*obj);
	buf += TCM_TOUCH_FRAME_BYTES;

	/* We don't need to report releases because we have INPUT_MT_DROP_UNUSED */
	for (size_t i = 0; i < num_objects; i++) {
		u8 idx, classification;
		u16 x, y;

		obj = (const struct tcm_report_object *)buf;
		buf += sizeof(*obj);

		idx = obj->desc & 0xf;
		classification = obj->desc >> 4;
		x = le16_to_cpu(obj->x);
		y = le16_to_cpu(obj->y);

		dev_dbg(&tcm->spi->dev,
			"touch report: idx %u class %u x %u y %u z %u w %#x\n",
			idx, classification, x, y, obj->z, obj->width);

		if (classification == TCM_OBJ_CLASS_NO_OBJECT)
			continue;

		/*
		 * The index field is 4 bits but only TCM_MAX_OBJECTS slots
		 * exist; an out-of-range slot would silently land events on
		 * the previously addressed slot.
		 */
		if (idx >= TCM_MAX_OBJECTS)
			continue;

		input_mt_slot(tcm->input, idx);
		input_mt_report_slot_state(tcm->input, MT_TOOL_FINGER, true);

		touchscreen_report_pos(tcm->input, &tcm->props, x, y, true);

		/*
		 * The width byte's sub-layout is not understood yet, so
		 * ABS_MT_TOUCH_MAJOR/MINOR are not reported; the debug print
		 * above keeps collecting evidence.
		 */
		input_report_abs(tcm->input, ABS_MT_PRESSURE, obj->z);
	}

	input_mt_sync_frame(tcm->input);
	input_sync(tcm->input);

	return 0;
}

static irqreturn_t tcm_report_irq(int irq, void *data)
{
	struct tcm_data *tcm = data;
	struct tcm_message_header *header;
	u8 *buf = tcm->rx_buf;
	u16 len;
	int ret;

	header = (struct tcm_message_header *)buf;
	ret = tcm_recv_report(tcm, buf, TCM_BUF_SIZE);
	if (ret) {
		dev_err(&tcm->spi->dev, "failed to read report: %d\n", ret);
		return IRQ_HANDLED;
	}

	switch (header->code) {
	case REPORT_OK:
	case REPORT_IDENTIFY:
	case REPORT_TOUCH:
	case REPORT_DELTA:
	case REPORT_RAW:
	case REPORT_DEBUG:
	case REPORT_TOUCH_HOLD:
		break;
	default:
		dev_dbg(&tcm->spi->dev, "Ignoring report %#x\n", header->code);
		return IRQ_HANDLED;
	}

	len = le16_to_cpu(header->length);

	dev_dbg(&tcm->spi->dev, "report %#x len %u\n", header->code, len);
	print_hex_dump_bytes("report: ", DUMP_PREFIX_OFFSET, buf,
			     min_t(size_t, TCM_BUF_SIZE, len + sizeof(*header)));

	if (len > TCM_BUF_SIZE - sizeof(*header)) {
		dev_err(&tcm->spi->dev, "report too long\n");
		return IRQ_HANDLED;
	}

	/* Check if this is a read response or an indication. For indications
	 * (user touched the screen) we just parse the report directly.
	 */
	if (completion_done(&tcm->response) && header->code == REPORT_TOUCH) {
		tcm_handle_touch_report(tcm, (const char *)buf, len + sizeof(*header));
		return IRQ_HANDLED;
	}

	tcm->buf_size = len + sizeof(*header);
	memcpy(tcm->buf, buf, len + sizeof(*header));
	complete(&tcm->response);

	return IRQ_HANDLED;
}

static int tcm_hw_init(struct tcm_data *tcm, u16 *max_x, u16 *max_y)
{
	int ret;
	struct tcm_identification id = { 0 };
	struct tcm_app_info app_info = { 0 };
	u16 status;

	/*
	 * Tell the firmware to start up. After starting it sends an IDENTIFY report, which
	 * we treat like a response to this message even though it's technically a new report.
	 */
	ret = tcm_read_message(tcm, TCM_RUN_APPLICATION_FIRMWARE, &id, sizeof(id));
	if (ret) {
		dev_err(&tcm->spi->dev, "failed to identify device: %d\n", ret);
		return ret;
	}

	dev_dbg(&tcm->spi->dev, "Synaptics TCM %s v%d mode %d\n",
		id.part_number, id.version, id.mode);
	if (id.mode != MODE_APPLICATION) {
		/* We don't support firmware updates or anything else */
		dev_err(&tcm->spi->dev, "Device is not in application mode\n");
		return -ENODEV;
	}

	do {
		msleep(20);
		ret = tcm_read_message(tcm, TCM_GET_APPLICATION_INFO, &app_info, sizeof(app_info));
		if (ret) {
			dev_err(&tcm->spi->dev, "failed to get application info: %d\n", ret);
			return ret;
		}
		status = le16_to_cpu(app_info.status);
	} while (status == APP_STATUS_BOOTING || status == APP_STATUS_UPDATING);

	dev_dbg(&tcm->spi->dev, "Application firmware v%d.%d (customer '%s') status %d\n",
		app_info.version[0], app_info.version[1], app_info.customer_config_id,
		status);

	*max_x = le16_to_cpu(app_info.max_x);
	*max_y = le16_to_cpu(app_info.max_y);

	return 0;
}

static int tcm_power_on(struct tcm_data *tcm)
{
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(tcm->supplies),
				    tcm->supplies);
	if (ret)
		return ret;

	gpiod_set_value_cansleep(tcm->reset_gpio, 1);
	usleep_range(10000, 11000);
	gpiod_set_value_cansleep(tcm->reset_gpio, 0);
	usleep_range(80000, 81000);

	return 0;
}

static int tcm_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct tcm_data *tcm;
	u16 max_x, max_y;
	int ret;

	tcm = devm_kzalloc(dev, sizeof(struct tcm_data), GFP_KERNEL);
	if (!tcm)
		return -ENOMEM;

	spi_set_drvdata(spi, tcm);
	tcm->spi = spi;

	tcm->tx_fill = devm_kmalloc(dev, TCM_BUF_SIZE, GFP_KERNEL);
	tcm->rx_buf = devm_kmalloc(dev, TCM_BUF_SIZE, GFP_KERNEL);
	tcm->tx_cmd = devm_kmalloc(dev, 3, GFP_KERNEL);
	if (!tcm->tx_fill || !tcm->rx_buf || !tcm->tx_cmd)
		return -ENOMEM;
	memset(tcm->tx_fill, 0xff, TCM_BUF_SIZE);

	init_completion(&tcm->response);

	tcm->supplies[0].supply = "vdd";
	tcm->supplies[1].supply = "vcc";
	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(tcm->supplies),
				      tcm->supplies);
	if (ret)
		return ret;

	tcm->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(tcm->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(tcm->reset_gpio),
				     "failed to get reset gpio\n");

	ret = devm_add_action_or_reset(dev, tcm_power_off,
				       tcm);
	if (ret)
		return ret;

	ret = tcm_power_on(tcm);
	if (ret)
		return ret;

	ret = devm_request_threaded_irq(dev, spi->irq, NULL,
					tcm_report_irq,
					IRQF_ONESHOT,
					"synaptics_tcm_report", tcm);
	if (ret < 0)
		return ret;

	ret = tcm_hw_init(tcm, &max_x, &max_y);
	if (ret) {
		dev_err(dev, "failed to initialize hardware\n");
		return ret;
	}

	tcm->input = devm_input_allocate_device(dev);
	if (!tcm->input)
		return -ENOMEM;

	tcm->input->name = "Synaptics TCM Oncell Touchscreen";
	tcm->input->id.bustype = BUS_SPI;
	tcm->input->open = tcm_input_open;
	tcm->input->close = tcm_input_close;

	input_set_abs_params(tcm->input, ABS_MT_POSITION_X, 0, max_x, 0, 0);
	input_set_abs_params(tcm->input, ABS_MT_POSITION_Y, 0, max_y, 0, 0);
	/*
	 * MAJOR/MINOR are declared but not currently reported: the S3910
	 * record's width sub-layout is not understood yet (see the report
	 * layout comment above tcm_handle_touch_report()).
	 */
	input_set_abs_params(tcm->input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(tcm->input, ABS_MT_TOUCH_MINOR, 0, 255, 0, 0);
	input_set_abs_params(tcm->input, ABS_MT_PRESSURE, 0, 255, 0, 0);

	touchscreen_parse_properties(tcm->input, true, &tcm->props);

	ret = input_mt_init_slots(tcm->input, TCM_MAX_OBJECTS,
				  INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
	if (ret)
		return ret;

	input_set_drvdata(tcm->input, tcm);

	ret = input_register_device(tcm->input);
	if (ret)
		return ret;

	return 0;
}

static const struct of_device_id syna_driver_ids[] = {
	/*
	 * Note: the S3908 (OnePlus 8T) sits on I2C; the upstream v2 series
	 * this driver derives from is an I2C driver. This tree carries an
	 * SPI conversion for the S3910 (OnePlus 15), so the s3908 entry is
	 * only kept for reference until the transport is abstracted (e.g.
	 * via regmap) to support both buses.
	 */
	{
		.compatible = "syna,s3908",
	},
	{
		.compatible = "syna,s3910",
	},
	{}
};
MODULE_DEVICE_TABLE(of, syna_driver_ids);

static const struct spi_device_id syna_spi_ids[] = {
	{ "s3908" },
	{ "s3910" },
	{ }
};
MODULE_DEVICE_TABLE(spi, syna_spi_ids);

static struct spi_driver syna_spi_driver = {
	.probe		= tcm_probe,
	.id_table	= syna_spi_ids,
	.driver		= {
		.name		= "synaptics-tcm",
		.of_match_table	= syna_driver_ids,
	},
};

module_spi_driver(syna_spi_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Frieder Hannenheim <frieder.hannenheim@proton.me>");
MODULE_AUTHOR("Caleb Connolly <caleb@postmarketos.org>");
MODULE_DESCRIPTION("A driver for Synaptics TCM Oncell Touchpanels");
