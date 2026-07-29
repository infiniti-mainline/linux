// SPDX-License-Identifier: GPL-2.0
/*
 * Synaptics TouchComm touchscreen controllers on SPI
 *
 * Copyright (C) 2017-2020 Synaptics Incorporated.
 * Copyright (C) 2026 Luka Panio <lukapanio@gmail.com>
 * Copyright (C) 2026 Victor Fuentes <victor@vlinkz.dev>
 */

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
#include <linux/unaligned.h>

#define TCM_MARKER			0xa5
#define TCM_BUF_SIZE			1024

#define TCM_CMD_IDENTIFY		0x02
#define TCM_CMD_ENABLE_REPORT		0x05
#define TCM_CMD_DISABLE_REPORT		0x06
#define TCM_CMD_GET_APP_INFO		0x20
#define TCM_CMD_GET_REPORT_CONFIG	0x25

#define TCM_REPORT_TOUCH		0x11

#define TCM_APP_INFO_MAX_X		32
#define TCM_APP_INFO_MAX_Y		34
#define TCM_APP_INFO_MAX_OBJECTS	36

#define TCM_RC_END			0x00
#define TCM_RC_FOREACH_ACTIVE_OBJECT	0x01
#define TCM_RC_FOREACH_OBJECT		0x02
#define TCM_RC_FOREACH_END		0x03
#define TCM_RC_PAD_TO_NEXT_BYTE		0x04
#define TCM_RC_OBJECT_INDEX		0x06
#define TCM_RC_OBJECT_CLASSIFICATION	0x07
#define TCM_RC_OBJECT_X			0x08
#define TCM_RC_OBJECT_Y			0x09
#define TCM_RC_NUM_ACTIVE_OBJECTS	0x18

#define TCM_OBJECT_FINGER		1
#define TCM_OBJECT_GLOVED_FINGER	2

#define TCM_MAX_OBJECTS			20

struct tcm_point {
	u16 x;
	u16 y;
	bool active;
};

struct tcm {
	struct spi_device *spi;
	struct input_dev *input;
	struct touchscreen_properties props;
	struct regulator_bulk_data supplies[2];
	struct gpio_desc *reset_gpio;
	unsigned int num_objects;
	u16 max_x;
	u16 max_y;
	u8 report_cfg[TCM_BUF_SIZE];
	unsigned int report_cfg_len;
	u8 *tx;
	u8 *rx;
};

static int tcm_read(struct tcm *tcm, size_t len)
{
	struct spi_transfer xfer = {
		.tx_buf = tcm->tx,
		.rx_buf = tcm->rx,
		.len = len,
	};

	return spi_sync_transfer(tcm->spi, &xfer, 1);
}

/* Returns the payload length, the payload is at tcm->rx + 2 */
static int tcm_read_message(struct tcm *tcm, u8 *code)
{
	unsigned int len;
	int ret;

	ret = tcm_read(tcm, 4);
	if (ret)
		return ret;

	if (tcm->rx[0] != TCM_MARKER)
		return -EIO;

	*code = tcm->rx[1];
	len = get_unaligned_le16(&tcm->rx[2]);
	if (!len)
		return 0;

	if (len + 3 > TCM_BUF_SIZE)
		return -EOVERFLOW;

	ret = tcm_read(tcm, len + 3);

	return ret ?: len;
}

static int tcm_write(struct tcm *tcm, u8 cmd, u8 *payload, size_t len)
{
	u8 buf[4] = { cmd, len };

	if (len)
		buf[3] = *payload;

	return spi_write_then_read(tcm->spi, buf, 3 + len, NULL, 0);
}

static int tcm_command(struct tcm *tcm, u8 cmd)
{
	u8 code;
	int ret;

	ret = tcm_write(tcm, cmd, NULL, 0);
	if (ret)
		return ret;

	msleep(40);

	return tcm_read_message(tcm, &code);
}

static int tcm_init(struct tcm *tcm)
{
	u8 code;
	int ret;

	/* Identifying the controller makes it enter application firmware */
	ret = tcm_write(tcm, TCM_CMD_IDENTIFY, NULL, 0);
	if (ret)
		return ret;

	tcm_read_message(tcm, &code);

	ret = tcm_command(tcm, TCM_CMD_GET_APP_INFO);
	if (ret < 0)
		return ret;
	if (ret < TCM_APP_INFO_MAX_OBJECTS + 2)
		return -EPROTO;

	tcm->max_x = get_unaligned_le16(&tcm->rx[2 + TCM_APP_INFO_MAX_X]);
	tcm->max_y = get_unaligned_le16(&tcm->rx[2 + TCM_APP_INFO_MAX_Y]);
	tcm->num_objects = min(get_unaligned_le16(&tcm->rx[2 + TCM_APP_INFO_MAX_OBJECTS]),
			       TCM_MAX_OBJECTS);

	ret = tcm_command(tcm, TCM_CMD_GET_REPORT_CONFIG);
	if (ret < 0)
		return ret;
	if (!ret)
		return -EPROTO;

	memcpy(tcm->report_cfg, &tcm->rx[2], ret);
	tcm->report_cfg_len = ret;

	return 0;
}

static unsigned int tcm_get_bits(const u8 *buf, unsigned int len,
				 unsigned int *pos, unsigned int bits)
{
	unsigned int val = 0, i;

	if (*pos + bits > len * 8) {
		*pos += bits;
		return 0;
	}

	for (i = 0; i < bits; i++, (*pos)++)
		val |= ((buf[*pos / 8] >> (*pos % 8)) & 1) << i;

	return val;
}

static void tcm_report_touch(struct tcm *tcm, const u8 *data, unsigned int len)
{
	struct tcm_point points[TCM_MAX_OBJECTS] = { };
	const u8 *cfg = tcm->report_cfg;
	unsigned int i = 0, pos = 0, loop = 0, count = 0, obj = 0;
	unsigned int num_active = 0, val;
	bool active_only = false, have_num_active = false, more;
	u8 code;

	while (i < tcm->report_cfg_len) {
		code = cfg[i++];

		switch (code) {
		case TCM_RC_END:
			goto out;
		case TCM_RC_FOREACH_ACTIVE_OBJECT:
			if (have_num_active && !num_active)
				goto out;
			fallthrough;
		case TCM_RC_FOREACH_OBJECT:
			active_only = code == TCM_RC_FOREACH_ACTIVE_OBJECT;
			loop = i;
			count = 0;
			continue;
		case TCM_RC_FOREACH_END:
			count++;
			if (!active_only)
				more = count < tcm->num_objects;
			else if (have_num_active)
				more = count < num_active;
			else
				more = pos < len * 8;
			if (more)
				i = loop;
			continue;
		case TCM_RC_PAD_TO_NEXT_BYTE:
			pos = ALIGN(pos, 8);
			continue;
		}

		if (i >= tcm->report_cfg_len)
			break;

		val = tcm_get_bits(data, len, &pos, cfg[i++]);

		switch (code) {
		case TCM_RC_NUM_ACTIVE_OBJECTS:
			num_active = val;
			have_num_active = true;
			break;
		case TCM_RC_OBJECT_INDEX:
			obj = min(val, TCM_MAX_OBJECTS - 1);
			points[obj].active = true;
			break;
		case TCM_RC_OBJECT_CLASSIFICATION:
			points[obj].active = val == TCM_OBJECT_FINGER ||
					     val == TCM_OBJECT_GLOVED_FINGER;
			break;
		case TCM_RC_OBJECT_X:
			points[obj].x = val;
			break;
		case TCM_RC_OBJECT_Y:
			points[obj].y = val;
			break;
		}
	}

out:
	for (i = 0; i < tcm->num_objects; i++) {
		if (!points[i].active)
			continue;

		input_mt_slot(tcm->input, i);
		input_mt_report_slot_state(tcm->input, MT_TOOL_FINGER, true);
		touchscreen_report_pos(tcm->input, &tcm->props,
				       points[i].x, points[i].y, true);
	}

	input_mt_sync_frame(tcm->input);
	input_sync(tcm->input);
}

static irqreturn_t tcm_irq(int irq, void *data)
{
	struct tcm *tcm = data;
	u8 code;
	int ret;

	ret = tcm_read_message(tcm, &code);
	if (ret < 0)
		return IRQ_NONE;

	if (code == TCM_REPORT_TOUCH && ret)
		tcm_report_touch(tcm, &tcm->rx[2], ret);

	return IRQ_HANDLED;
}

static int tcm_enable_report(struct tcm *tcm, bool enable)
{
	u8 report = TCM_REPORT_TOUCH;

	return tcm_write(tcm, enable ? TCM_CMD_ENABLE_REPORT :
			 TCM_CMD_DISABLE_REPORT, &report, 1);
}

static int tcm_power_on(struct tcm *tcm)
{
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(tcm->supplies), tcm->supplies);
	if (ret)
		return ret;

	msleep(200);
	gpiod_set_value_cansleep(tcm->reset_gpio, 1);
	usleep_range(10000, 11000);
	gpiod_set_value_cansleep(tcm->reset_gpio, 0);
	msleep(80);

	return 0;
}

static void tcm_power_off(void *data)
{
	struct tcm *tcm = data;

	regulator_bulk_disable(ARRAY_SIZE(tcm->supplies), tcm->supplies);
}

static int tcm_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct tcm *tcm;
	int ret;

	tcm = devm_kzalloc(dev, sizeof(*tcm), GFP_KERNEL);
	if (!tcm)
		return -ENOMEM;

	tcm->spi = spi;
	spi_set_drvdata(spi, tcm);

	tcm->tx = devm_kmalloc(dev, TCM_BUF_SIZE, GFP_KERNEL);
	tcm->rx = devm_kmalloc(dev, TCM_BUF_SIZE, GFP_KERNEL);
	if (!tcm->tx || !tcm->rx)
		return -ENOMEM;

	memset(tcm->tx, 0xff, TCM_BUF_SIZE);

	tcm->supplies[0].supply = "vdd";
	tcm->supplies[1].supply = "avdd";
	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(tcm->supplies), tcm->supplies);
	if (ret)
		return ret;

	tcm->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(tcm->reset_gpio))
		return PTR_ERR(tcm->reset_gpio);

	ret = tcm_power_on(tcm);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(dev, tcm_power_off, tcm);
	if (ret)
		return ret;

	ret = tcm_init(tcm);
	if (ret)
		return dev_err_probe(dev, ret, "failed to initialize\n");

	tcm->input = devm_input_allocate_device(dev);
	if (!tcm->input)
		return -ENOMEM;

	tcm->input->name = "Synaptics TCM Touchscreen";
	tcm->input->id.bustype = BUS_SPI;

	input_set_abs_params(tcm->input, ABS_MT_POSITION_X, 0, tcm->max_x, 0, 0);
	input_set_abs_params(tcm->input, ABS_MT_POSITION_Y, 0, tcm->max_y, 0, 0);
	touchscreen_parse_properties(tcm->input, true, &tcm->props);

	ret = input_mt_init_slots(tcm->input, tcm->num_objects,
				  INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
	if (ret)
		return ret;

	ret = input_register_device(tcm->input);
	if (ret)
		return ret;

	ret = devm_request_threaded_irq(dev, spi->irq, NULL, tcm_irq,
					IRQF_ONESHOT, "synaptics-tcm", tcm);
	if (ret)
		return ret;

	return tcm_enable_report(tcm, true);
}

static int tcm_suspend(struct device *dev)
{
	struct tcm *tcm = dev_get_drvdata(dev);

	disable_irq(tcm->spi->irq);
	tcm_enable_report(tcm, false);
	tcm_power_off(tcm);

	return 0;
}

static int tcm_resume(struct device *dev)
{
	struct tcm *tcm = dev_get_drvdata(dev);
	int ret;

	ret = tcm_power_on(tcm);
	if (ret)
		return ret;

	ret = tcm_init(tcm);
	if (ret)
		dev_err(dev, "failed to initialize: %d\n", ret);

	enable_irq(tcm->spi->irq);

	return tcm_enable_report(tcm, true);
}

static DEFINE_SIMPLE_DEV_PM_OPS(tcm_pm_ops, tcm_suspend, tcm_resume);

static const struct of_device_id tcm_of_match[] = {
	{ .compatible = "synaptics,s3910" },
	{ }
};
MODULE_DEVICE_TABLE(of, tcm_of_match);

static const struct spi_device_id tcm_spi_ids[] = {
	{ "s3910" },
	{ }
};
MODULE_DEVICE_TABLE(spi, tcm_spi_ids);

static struct spi_driver tcm_driver = {
	.driver = {
		.name = "synaptics-tcm-spi",
		.of_match_table = tcm_of_match,
		.pm = pm_sleep_ptr(&tcm_pm_ops),
	},
	.probe = tcm_probe,
	.id_table = tcm_spi_ids,
};
module_spi_driver(tcm_driver);

MODULE_DESCRIPTION("Synaptics TouchComm SPI touchscreen driver");
MODULE_LICENSE("GPL");
