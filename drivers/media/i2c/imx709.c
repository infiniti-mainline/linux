// SPDX-License-Identifier: GPL-2.0
/*
 * Sony IMX709 32 MP 10-bit RAW MIPI CSI-2 sensor driver
 *
 * Register sequences taken from the OnePlus 15 (infiniti) front camera
 * module.
 *
 * Copyright (C) 2026 Victor Fuentes <victor@vlinkz.dev>
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#define IMX709_REG_CHIP_ID		CCI_REG16(0x0016)
#define IMX709_CHIP_ID			0x0709

#define IMX709_REG_MODE_SELECT		CCI_REG8(0x0100)
#define IMX709_MODE_STANDBY		0x00
#define IMX709_MODE_STREAMING		0x01

#define IMX709_REG_EXPOSURE		CCI_REG16(0x0202)
#define IMX709_EXPOSURE_MIN		8
#define IMX709_EXPOSURE_MARGIN		24

/* Analogue gain: code = 16384 - 16384 / gain, 1x to 16x */
#define IMX709_REG_AGAIN		CCI_REG16(0x0204)
#define IMX709_AGAIN_MIN		0
#define IMX709_AGAIN_MAX		0x3c00

/* Digital gain: 8.8 fixed point */
#define IMX709_REG_DGAIN		CCI_REG16(0x020e)
#define IMX709_DGAIN_MIN		0x0100
#define IMX709_DGAIN_MAX		0x0fff

#define IMX709_REG_FLL			CCI_REG16(0x0340)
#define IMX709_FLL_MAX			0xffff

#define IMX709_REG_TEST_PATTERN		CCI_REG16(0x0600)

#define IMX709_XCLK_RATE		19200000
#define IMX709_DATA_LANES		4
#define IMX709_BPP			10

#define IMX709_NATIVE_WIDTH		6560
#define IMX709_NATIVE_HEIGHT		4928

/* 19.2 MHz / 8 * 1253 / 4 = 751.8 Mbps per lane */
#define IMX709_LINK_FREQ_376MHZ		375900000LL

static const s64 imx709_link_freqs[] = {
	IMX709_LINK_FREQ_376MHZ,
};

#define IMX709_PIXEL_RATE \
	(IMX709_LINK_FREQ_376MHZ * 2 * IMX709_DATA_LANES / IMX709_BPP)

static const char * const imx709_supply_names[] = {
	"dovdd",
	"avdd",
	"dvdd",
};

struct imx709_mode {
	u32 width;
	u32 height;
	u32 hts;
	u32 vts;
	const struct cci_reg_sequence *regs;
	unsigned int num_regs;
};

static const struct cci_reg_sequence imx709_init_regs[] = {
	{ CCI_REG8(0x0136), 0x13 },
	{ CCI_REG8(0x0137), 0x33 },
	{ CCI_REG8(0x3304), 0x00 },
	{ CCI_REG8(0x33f0), 0x10 },
	{ CCI_REG8(0x33f1), 0x07 },
	{ CCI_REG8(0x0111), 0x02 },
	{ CCI_REG8(0x3379), 0x00 },
	{ CCI_REG8(0x3724), 0x03 },
	{ CCI_REG8(0x3732), 0x05 },
	{ CCI_REG8(0x3733), 0xdc },
	{ CCI_REG8(0x3736), 0x09 },
	{ CCI_REG8(0x3737), 0xc4 },
	{ CCI_REG8(0x373a), 0x07 },
	{ CCI_REG8(0x373b), 0xd0 },
	{ CCI_REG8(0x373e), 0x03 },
	{ CCI_REG8(0x373f), 0xe8 },
	{ CCI_REG8(0x37a2), 0x8c },
	{ CCI_REG8(0x37b4), 0x03 },
	{ CCI_REG8(0x37ba), 0x0f },
	{ CCI_REG8(0x37bb), 0xa0 },
	{ CCI_REG8(0x37be), 0x0d },
	{ CCI_REG8(0x37bf), 0xac },
	{ CCI_REG8(0x37c2), 0x04 },
	{ CCI_REG8(0x37c3), 0xb0 },
	{ CCI_REG8(0x37c6), 0x3e },
	{ CCI_REG8(0x37c7), 0x80 },
	{ CCI_REG8(0x37ca), 0x3a },
	{ CCI_REG8(0x37cb), 0x20 },
	{ CCI_REG8(0x37ce), 0x03 },
	{ CCI_REG8(0x37cf), 0x84 },
	{ CCI_REG8(0x37d6), 0x09 },
	{ CCI_REG8(0x37db), 0x0f },
	{ CCI_REG8(0x37dc), 0x0f },
	{ CCI_REG8(0x37dd), 0x01 },
	{ CCI_REG8(0x385e), 0x05 },
	{ CCI_REG8(0x385f), 0xdc },
	{ CCI_REG8(0x3862), 0x09 },
	{ CCI_REG8(0x3863), 0xc4 },
	{ CCI_REG8(0x3866), 0x07 },
	{ CCI_REG8(0x3867), 0xd0 },
	{ CCI_REG8(0x386a), 0x03 },
	{ CCI_REG8(0x386b), 0xe8 },
	{ CCI_REG8(0x3a1b), 0x40 },
	{ CCI_REG8(0x3a1d), 0x40 },
	{ CCI_REG8(0x3a1e), 0x01 },
	{ CCI_REG8(0x3a92), 0x01 },
	{ CCI_REG8(0x3aa2), 0x01 },
	{ CCI_REG8(0x640c), 0xff },
	{ CCI_REG8(0x640d), 0xff },
	{ CCI_REG8(0x6414), 0x00 },
	{ CCI_REG8(0x6415), 0x9d },
	{ CCI_REG8(0x6418), 0xff },
	{ CCI_REG8(0x6419), 0xff },
	{ CCI_REG8(0x6420), 0x00 },
	{ CCI_REG8(0x6421), 0x9d },
	{ CCI_REG8(0x6424), 0xff },
	{ CCI_REG8(0x6425), 0xff },
	{ CCI_REG8(0x642c), 0x00 },
	{ CCI_REG8(0x642d), 0x9d },
	{ CCI_REG8(0x6430), 0xff },
	{ CCI_REG8(0x6431), 0xff },
	{ CCI_REG8(0x6438), 0x02 },
	{ CCI_REG8(0x6439), 0x24 },
	{ CCI_REG8(0x643c), 0xff },
	{ CCI_REG8(0x643d), 0xff },
	{ CCI_REG8(0x6444), 0x02 },
	{ CCI_REG8(0x6445), 0x24 },
	{ CCI_REG8(0x6454), 0xff },
	{ CCI_REG8(0x6455), 0xff },
	{ CCI_REG8(0x645c), 0x00 },
	{ CCI_REG8(0x645d), 0x9d },
	{ CCI_REG8(0x646c), 0xff },
	{ CCI_REG8(0x646d), 0xff },
	{ CCI_REG8(0x6474), 0x00 },
	{ CCI_REG8(0x6475), 0x9d },
	{ CCI_REG8(0x68b0), 0x00 },
	{ CCI_REG8(0x68b2), 0x1a },
	{ CCI_REG8(0x68b5), 0x01 },
	{ CCI_REG8(0x68b6), 0x00 },
	{ CCI_REG8(0x68b8), 0x1a },
	{ CCI_REG8(0x69d3), 0x27 },
	{ CCI_REG8(0x69df), 0x27 },
	{ CCI_REG8(0x6fd1), 0x09 },
	{ CCI_REG8(0x6fd3), 0x09 },
	{ CCI_REG8(0x6fd5), 0x09 },
	{ CCI_REG8(0x6fd7), 0x09 },
	{ CCI_REG8(0x6fd9), 0x09 },
	{ CCI_REG8(0x701e), 0x03 },
	{ CCI_REG8(0x7020), 0x03 },
	{ CCI_REG8(0x7022), 0x03 },
	{ CCI_REG8(0x7024), 0x03 },
	{ CCI_REG8(0x7026), 0x03 },
	{ CCI_REG8(0x710a), 0x17 },
	{ CCI_REG8(0x710c), 0x18 },
	{ CCI_REG8(0x73e9), 0x53 },
	{ CCI_REG8(0x73ed), 0x50 },
	{ CCI_REG8(0x73f1), 0x4b },
	{ CCI_REG8(0x73f5), 0x4d },
	{ CCI_REG8(0x73f9), 0x4e },
	{ CCI_REG8(0x741d), 0x56 },
	{ CCI_REG8(0x7421), 0x55 },
	{ CCI_REG8(0x7425), 0x53 },
	{ CCI_REG8(0x7429), 0x56 },
	{ CCI_REG8(0x742d), 0x56 },
	{ CCI_REG8(0x7451), 0x58 },
	{ CCI_REG8(0x7455), 0x56 },
	{ CCI_REG8(0x7459), 0x57 },
	{ CCI_REG8(0x745d), 0x58 },
	{ CCI_REG8(0x7461), 0x58 },
	{ CCI_REG8(0x7485), 0x5a },
	{ CCI_REG8(0x7489), 0x58 },
	{ CCI_REG8(0x748d), 0x58 },
	{ CCI_REG8(0x7491), 0x5b },
	{ CCI_REG8(0x7495), 0x5b },
	{ CCI_REG8(0x74b9), 0x5d },
	{ CCI_REG8(0x74bb), 0x5e },
	{ CCI_REG8(0x74bd), 0x5f },
	{ CCI_REG8(0x74bf), 0x60 },
	{ CCI_REG8(0x74e0), 0x16 },
	{ CCI_REG8(0x74e2), 0x19 },
	{ CCI_REG8(0x74e4), 0x16 },
	{ CCI_REG8(0x74e6), 0x17 },
	{ CCI_REG8(0x74e8), 0x17 },
	{ CCI_REG8(0x74fa), 0x20 },
	{ CCI_REG8(0x74fc), 0x24 },
	{ CCI_REG8(0x74fe), 0x1e },
	{ CCI_REG8(0x7500), 0x1f },
	{ CCI_REG8(0x7502), 0x30 },
	{ CCI_REG8(0x7514), 0x3f },
	{ CCI_REG8(0x7516), 0x1e },
	{ CCI_REG8(0x7518), 0x19 },
	{ CCI_REG8(0x751a), 0x1e },
	{ CCI_REG8(0x751c), 0x1e },
	{ CCI_REG8(0x752e), 0x57 },
	{ CCI_REG8(0x7530), 0x26 },
	{ CCI_REG8(0x7532), 0x14 },
	{ CCI_REG8(0x7534), 0x1d },
	{ CCI_REG8(0x7536), 0x1d },
	{ CCI_REG8(0x7548), 0x53 },
	{ CCI_REG8(0x7549), 0x19 },
	{ CCI_REG8(0x754a), 0x52 },
	{ CCI_REG8(0x754b), 0x53 },
	{ CCI_REG8(0x757e), 0x07 },
	{ CCI_REG8(0x7590), 0x0a },
	{ CCI_REG8(0x75aa), 0x0a },
	{ CCI_REG8(0x75ac), 0x05 },
	{ CCI_REG8(0x75c4), 0x0a },
	{ CCI_REG8(0x75c5), 0x0d },
	{ CCI_REG8(0x75c6), 0x0a },
	{ CCI_REG8(0x75c7), 0x0a },
	{ CCI_REG8(0x7656), 0x28 },
	{ CCI_REG8(0x765a), 0x14 },
	{ CCI_REG8(0x765e), 0x05 },
	{ CCI_REG8(0x7660), 0x05 },
	{ CCI_REG8(0x7678), 0x0a },
	{ CCI_REG8(0x767a), 0x0a },
	{ CCI_REG8(0x7692), 0x14 },
	{ CCI_REG8(0x7694), 0x14 },
	{ CCI_REG8(0x76ea), 0x01 },
	{ CCI_REG8(0x76ec), 0x01 },
	{ CCI_REG8(0x76ee), 0x01 },
	{ CCI_REG8(0x76f0), 0x01 },
	{ CCI_REG8(0x76f2), 0x01 },
	{ CCI_REG8(0x787a), 0x05 },
	{ CCI_REG8(0x787c), 0x05 },
	{ CCI_REG8(0x787e), 0x05 },
	{ CCI_REG8(0x7880), 0x05 },
	{ CCI_REG8(0x7b52), 0x14 },
	{ CCI_REG8(0x7b53), 0x14 },
	{ CCI_REG8(0x7b54), 0x14 },
	{ CCI_REG8(0x7b55), 0x14 },
	{ CCI_REG8(0x7b56), 0x14 },
	{ CCI_REG8(0x7b57), 0x14 },
	{ CCI_REG8(0x7b58), 0x14 },
	{ CCI_REG8(0x7b59), 0x14 },
	{ CCI_REG8(0x7b5a), 0x14 },
	{ CCI_REG8(0x7b5c), 0x05 },
	{ CCI_REG8(0x7b5e), 0x05 },
	{ CCI_REG8(0x7b60), 0x14 },
	{ CCI_REG8(0x7b61), 0x14 },
	{ CCI_REG8(0x7b62), 0x14 },
	{ CCI_REG8(0x7b63), 0x14 },
	{ CCI_REG8(0x7b64), 0x14 },
	{ CCI_REG8(0x7b65), 0x14 },
	{ CCI_REG8(0x7b66), 0x14 },
	{ CCI_REG8(0x7b67), 0x14 },
	{ CCI_REG8(0x7b68), 0x14 },
	{ CCI_REG8(0x7b69), 0x14 },
	{ CCI_REG8(0x7b6a), 0x14 },
	{ CCI_REG8(0x7b6b), 0x14 },
	{ CCI_REG8(0x7b6c), 0x14 },
	{ CCI_REG8(0x7b6d), 0x14 },
	{ CCI_REG8(0x7b6e), 0x14 },
	{ CCI_REG8(0x7b6f), 0x14 },
	{ CCI_REG8(0x7b70), 0x14 },
	{ CCI_REG8(0x7b71), 0x14 },
	{ CCI_REG8(0x7b72), 0x14 },
	{ CCI_REG8(0x7b73), 0x14 },
	{ CCI_REG8(0x7b74), 0x14 },
	{ CCI_REG8(0x7b76), 0x14 },
	{ CCI_REG8(0x7b78), 0x14 },
	{ CCI_REG8(0x7b7a), 0x14 },
	{ CCI_REG8(0x7b7b), 0x14 },
	{ CCI_REG8(0x7b7c), 0x14 },
	{ CCI_REG8(0x7b7d), 0x14 },
	{ CCI_REG8(0x7b7e), 0x14 },
	{ CCI_REG8(0x7b7f), 0x14 },
	{ CCI_REG8(0x7b80), 0x14 },
	{ CCI_REG8(0x7b81), 0x14 },
	{ CCI_REG8(0x7b82), 0x14 },
	{ CCI_REG8(0x7b83), 0x14 },
	{ CCI_REG8(0x7b84), 0x14 },
	{ CCI_REG8(0x7b85), 0x14 },
	{ CCI_REG8(0x7b86), 0x14 },
	{ CCI_REG8(0x7b87), 0x14 },
	{ CCI_REG8(0x7b88), 0x14 },
	{ CCI_REG8(0x7b89), 0x14 },
	{ CCI_REG8(0x7b8a), 0x14 },
	{ CCI_REG8(0x7b8b), 0x14 },
	{ CCI_REG8(0x7b8c), 0x14 },
	{ CCI_REG8(0x7b8d), 0x14 },
	{ CCI_REG8(0x7b8e), 0x14 },
	{ CCI_REG8(0x7b90), 0x14 },
	{ CCI_REG8(0x7b92), 0x14 },
	{ CCI_REG8(0x7b94), 0x14 },
	{ CCI_REG8(0x7b95), 0x14 },
	{ CCI_REG8(0x7b96), 0x14 },
	{ CCI_REG8(0x7b97), 0x14 },
	{ CCI_REG8(0x7b98), 0x14 },
	{ CCI_REG8(0x7b99), 0x14 },
	{ CCI_REG8(0x7b9a), 0x14 },
	{ CCI_REG8(0x7b9b), 0x14 },
	{ CCI_REG8(0x7b9c), 0x14 },
	{ CCI_REG8(0x7b9d), 0x14 },
	{ CCI_REG8(0x7b9e), 0x14 },
	{ CCI_REG8(0x7b9f), 0x14 },
	{ CCI_REG8(0x7ba0), 0x14 },
	{ CCI_REG8(0x7ba1), 0x14 },
	{ CCI_REG8(0x7ba2), 0x14 },
	{ CCI_REG8(0x7ba3), 0x14 },
	{ CCI_REG8(0x7ba4), 0x14 },
	{ CCI_REG8(0x7ba5), 0x14 },
	{ CCI_REG8(0x7ba6), 0x14 },
	{ CCI_REG8(0x7ba7), 0x14 },
	{ CCI_REG8(0x7ba8), 0x14 },
	{ CCI_REG8(0x7baa), 0x14 },
	{ CCI_REG8(0x7bac), 0x14 },
	{ CCI_REG8(0x7bae), 0x14 },
	{ CCI_REG8(0x7baf), 0x14 },
	{ CCI_REG8(0x7bb0), 0x14 },
	{ CCI_REG8(0x7bb1), 0x14 },
	{ CCI_REG8(0x7bb2), 0x14 },
	{ CCI_REG8(0x7bb3), 0x14 },
	{ CCI_REG8(0x7bb4), 0x14 },
	{ CCI_REG8(0x7bb5), 0x14 },
	{ CCI_REG8(0x7bb6), 0x14 },
	{ CCI_REG8(0x7bb7), 0x14 },
	{ CCI_REG8(0x7bb8), 0x14 },
	{ CCI_REG8(0x7bb9), 0x14 },
	{ CCI_REG8(0x7bba), 0x14 },
	{ CCI_REG8(0x7bbb), 0x14 },
	{ CCI_REG8(0x7bbc), 0x14 },
	{ CCI_REG8(0x7bbd), 0x14 },
	{ CCI_REG8(0x7bbe), 0x14 },
	{ CCI_REG8(0x7bc0), 0x14 },
	{ CCI_REG8(0x7bc1), 0x14 },
	{ CCI_REG8(0x7bc2), 0x14 },
	{ CCI_REG8(0x7bc3), 0x14 },
	{ CCI_REG8(0x7bc4), 0x14 },
	{ CCI_REG8(0x7bc5), 0x14 },
	{ CCI_REG8(0x7bc7), 0x14 },
	{ CCI_REG8(0x7bc8), 0x14 },
	{ CCI_REG8(0x7bc9), 0x14 },
	{ CCI_REG8(0x7bca), 0x14 },
	{ CCI_REG8(0x7bcb), 0x14 },
	{ CCI_REG8(0x9002), 0x0a },
	{ CCI_REG8(0x9003), 0x0a },
	{ CCI_REG8(0x9004), 0x0a },
	{ CCI_REG8(0x90e4), 0x01 },
	{ CCI_REG8(0x90e7), 0x01 },
	{ CCI_REG8(0x9200), 0x65 },
	{ CCI_REG8(0x9201), 0xce },
	{ CCI_REG8(0x9202), 0x65 },
	{ CCI_REG8(0x9203), 0xc4 },
	{ CCI_REG8(0x9204), 0x00 },
	{ CCI_REG8(0x9205), 0x00 },
	{ CCI_REG8(0x9206), 0x00 },
	{ CCI_REG8(0x9207), 0x00 },
	{ CCI_REG8(0x9208), 0x00 },
	{ CCI_REG8(0x9209), 0x00 },
	{ CCI_REG8(0x920a), 0x00 },
	{ CCI_REG8(0x920b), 0x00 },
	{ CCI_REG8(0x920c), 0x00 },
	{ CCI_REG8(0x920d), 0x00 },
	{ CCI_REG8(0x920e), 0x00 },
	{ CCI_REG8(0x920f), 0x00 },
	{ CCI_REG8(0x9210), 0x00 },
	{ CCI_REG8(0x9211), 0x00 },
	{ CCI_REG8(0x9212), 0x00 },
	{ CCI_REG8(0x9213), 0x00 },
	{ CCI_REG8(0x9214), 0x00 },
	{ CCI_REG8(0x9215), 0x00 },
	{ CCI_REG8(0x9216), 0x00 },
	{ CCI_REG8(0x9217), 0x00 },
	{ CCI_REG8(0x9218), 0x00 },
	{ CCI_REG8(0x9219), 0x00 },
	{ CCI_REG8(0x921a), 0x00 },
	{ CCI_REG8(0x921b), 0x00 },
	{ CCI_REG8(0x921c), 0x00 },
	{ CCI_REG8(0x921d), 0x00 },
	{ CCI_REG8(0x921e), 0x00 },
	{ CCI_REG8(0x921f), 0x00 },
	{ CCI_REG8(0x9220), 0x00 },
	{ CCI_REG8(0x9221), 0x00 },
	{ CCI_REG8(0x9222), 0x00 },
	{ CCI_REG8(0x9223), 0x00 },
	{ CCI_REG8(0x9224), 0x00 },
	{ CCI_REG8(0x9225), 0x00 },
	{ CCI_REG8(0x9226), 0x00 },
	{ CCI_REG8(0x9227), 0x00 },
	{ CCI_REG8(0x9228), 0x00 },
	{ CCI_REG8(0x9229), 0x00 },
	{ CCI_REG8(0x922a), 0x00 },
	{ CCI_REG8(0x922b), 0x00 },
	{ CCI_REG8(0x922c), 0x00 },
	{ CCI_REG8(0x922d), 0x00 },
	{ CCI_REG8(0x922e), 0x00 },
	{ CCI_REG8(0x922f), 0x00 },
	{ CCI_REG8(0x9230), 0x00 },
	{ CCI_REG8(0x9231), 0x00 },
	{ CCI_REG8(0x9232), 0x00 },
	{ CCI_REG8(0x9233), 0x00 },
	{ CCI_REG8(0x9234), 0xd7 },
	{ CCI_REG8(0x9235), 0xe4 },
	{ CCI_REG8(0x9236), 0xd7 },
	{ CCI_REG8(0x9237), 0xe5 },
	{ CCI_REG8(0x9238), 0xd7 },
	{ CCI_REG8(0x9239), 0xe6 },
	{ CCI_REG8(0x923a), 0xd7 },
	{ CCI_REG8(0x923b), 0xe7 },
	{ CCI_REG8(0x923c), 0x15 },
	{ CCI_REG8(0x923d), 0x67 },
	{ CCI_REG8(0xb0f9), 0x06 },
	{ CCI_REG8(0xbc51), 0xc8 },
	{ CCI_REG8(0xbcaf), 0x01 },
	{ CCI_REG8(0xbd27), 0x28 },
	{ CCI_REG8(0xbdc6), 0x00 },
	{ CCI_REG8(0xbdc7), 0x00 },
	{ CCI_REG8(0xbdc8), 0x00 },
	{ CCI_REG8(0xbdc9), 0x00 },
	{ CCI_REG8(0xbdca), 0x00 },
	{ CCI_REG8(0xbdcb), 0x00 },
	{ CCI_REG8(0xbdcc), 0x00 },
	{ CCI_REG8(0xbdcd), 0x00 },
	{ CCI_REG8(0xbdce), 0x00 },
	{ CCI_REG8(0xbdcf), 0x00 },
	{ CCI_REG8(0xbdd0), 0x10 },
	{ CCI_REG8(0xbdd1), 0xdc },
	{ CCI_REG8(0xbdd2), 0x08 },
	{ CCI_REG8(0xbdd3), 0x58 },
	{ CCI_REG8(0xbdd4), 0x17 },
	{ CCI_REG8(0xbdd5), 0xdc },
	{ CCI_REG8(0xbdd6), 0x0b },
	{ CCI_REG8(0xbdd7), 0x80 },
	{ CCI_REG8(0xbde0), 0x00 },
	{ CCI_REG8(0xbde1), 0x00 },
	{ CCI_REG8(0xbde2), 0x00 },
	{ CCI_REG8(0xbde3), 0x00 },
	{ CCI_REG8(0xbde4), 0x00 },
	{ CCI_REG8(0xbde5), 0xe0 },
	{ CCI_REG8(0xbde7), 0xc8 },
	{ CCI_REG8(0xbde8), 0x00 },
	{ CCI_REG8(0xbdea), 0x01 },
	{ CCI_REG8(0xbdeb), 0xc8 },
	{ CCI_REG8(0xc0d9), 0x12 },
	{ CCI_REG8(0xc0db), 0x28 },
	{ CCI_REG8(0xc0de), 0x28 },
	{ CCI_REG8(0xc0e1), 0x2d },
	{ CCI_REG8(0xc0e3), 0x00 },
	{ CCI_REG8(0xc0f2), 0x18 },
	{ CCI_REG8(0xc0f3), 0x18 },
	{ CCI_REG8(0xc546), 0x0a },
	{ CCI_REG8(0xc547), 0x08 },
	{ CCI_REG8(0xc548), 0x28 },
	{ CCI_REG8(0xc54b), 0x28 },
	{ CCI_REG8(0xc54e), 0x3c },
	{ CCI_REG8(0xc551), 0x01 },
	{ CCI_REG8(0xc55e), 0x18 },
	{ CCI_REG8(0xc55f), 0x18 },
	{ CCI_REG8(0xe981), 0x02 },
	{ CCI_REG8(0xe982), 0x05 },
	{ CCI_REG8(0xe983), 0x33 },
	{ CCI_REG8(0xe984), 0x19 },
	{ CCI_REG8(0xe985), 0x13 },
	{ CCI_REG8(0xe986), 0x11 },
	{ CCI_REG8(0xe987), 0x10 },
	{ CCI_REG8(0xe98a), 0x01 },
	{ CCI_REG8(0xeb00), 0x18 },
	{ CCI_REG8(0xeb01), 0x0b },
	{ CCI_REG8(0xeb02), 0x04 },
	{ CCI_REG8(0x9412), 0x03 },
	{ CCI_REG8(0x9413), 0x02 },
	{ CCI_REG8(0x9414), 0x02 },
	{ CCI_REG8(0x950f), 0x8f },
	{ CCI_REG8(0x9533), 0x5f },
	{ CCI_REG8(0x953d), 0x9f },
	{ CCI_REG8(0x953f), 0xbf },
	{ CCI_REG8(0x9541), 0xff },
	{ CCI_REG8(0x9ae7), 0x11 },
	{ CCI_REG8(0x9ae9), 0x0e },
	{ CCI_REG8(0x9aed), 0x02 },
	{ CCI_REG8(0x9aef), 0x02 },
	{ CCI_REG8(0x9af9), 0x05 },
	{ CCI_REG8(0x9afb), 0x04 },
	{ CCI_REG8(0x9b0b), 0x02 },
	{ CCI_REG8(0x9b10), 0x02 },
	{ CCI_REG8(0x9b15), 0x02 },
	{ CCI_REG8(0x9b1a), 0x02 },
	{ CCI_REG8(0x9b84), 0x1c },
	{ CCI_REG8(0x9b86), 0x06 },
	{ CCI_REG8(0x9b87), 0x30 },
	{ CCI_REG8(0x9b89), 0x0c },
	{ CCI_REG8(0x9b8a), 0x20 },
	{ CCI_REG8(0x9b8c), 0x06 },
	{ CCI_REG8(0x9b90), 0x24 },
	{ CCI_REG8(0x9c01), 0x10 },
	{ CCI_REG8(0x9c05), 0x1c },
	{ CCI_REG8(0x9c07), 0x06 },
	{ CCI_REG8(0x9c0a), 0x0c },
	{ CCI_REG8(0x9c0e), 0x01 },
	{ CCI_REG8(0x9c24), 0x00 },
	{ CCI_REG8(0x9c3a), 0x00 },
	{ CCI_REG8(0x9c50), 0x00 },
	{ CCI_REG8(0x9c66), 0x00 },
	{ CCI_REG8(0x9f00), 0xef },
	{ CCI_REG8(0x9f01), 0xef },
	{ CCI_REG8(0x9f02), 0xef },
	{ CCI_REG8(0x9f03), 0xbf },
	{ CCI_REG8(0x9f04), 0xbf },
	{ CCI_REG8(0x9f05), 0xbf },
	{ CCI_REG8(0x9f06), 0xef },
	{ CCI_REG8(0x9f07), 0xef },
	{ CCI_REG8(0x9f08), 0xef },
	{ CCI_REG8(0x9f09), 0xbf },
	{ CCI_REG8(0x9f0a), 0xbf },
	{ CCI_REG8(0x9f0b), 0xbf },
	{ CCI_REG8(0xa512), 0x2f },
	{ CCI_REG8(0xa513), 0x14 },
	{ CCI_REG8(0xa514), 0x07 },
	{ CCI_REG8(0xa518), 0x32 },
	{ CCI_REG8(0xa519), 0x20 },
	{ CCI_REG8(0xa51a), 0x0c },
	{ CCI_REG8(0xa521), 0x04 },
	{ CCI_REG8(0xa527), 0x04 },
	{ CCI_REG8(0xa528), 0x04 },
	{ CCI_REG8(0xa529), 0x02 },
	{ CCI_REG8(0xa52d), 0x2f },
	{ CCI_REG8(0xa52f), 0x06 },
	{ CCI_REG8(0xa533), 0x31 },
	{ CCI_REG8(0xa534), 0x15 },
	{ CCI_REG8(0xa535), 0x08 },
	{ CCI_REG8(0xa53c), 0x04 },
	{ CCI_REG8(0xa53f), 0x04 },
	{ CCI_REG8(0xa542), 0x04 },
	{ CCI_REG8(0xa543), 0x04 },
	{ CCI_REG8(0xa544), 0x02 },
	{ CCI_REG8(0xa545), 0x04 },
	{ CCI_REG8(0xa546), 0x04 },
	{ CCI_REG8(0xa547), 0x02 },
	{ CCI_REG8(0xa548), 0x2f },
	{ CCI_REG8(0xa54a), 0x06 },
	{ CCI_REG8(0xa54b), 0x2f },
	{ CCI_REG8(0xa54d), 0x06 },
	{ CCI_REG8(0xa54e), 0x31 },
	{ CCI_REG8(0xa54f), 0x15 },
	{ CCI_REG8(0xa550), 0x08 },
	{ CCI_REG8(0xa557), 0x04 },
	{ CCI_REG8(0xa55d), 0x04 },
	{ CCI_REG8(0xa55e), 0x04 },
	{ CCI_REG8(0xa55f), 0x02 },
	{ CCI_REG8(0xa563), 0x1a },
	{ CCI_REG8(0xa565), 0x03 },
	{ CCI_REG8(0xa569), 0x1b },
	{ CCI_REG8(0xa56a), 0x15 },
	{ CCI_REG8(0xa56b), 0x05 },
	{ CCI_REG8(0xa572), 0x04 },
	{ CCI_REG8(0xa575), 0x04 },
	{ CCI_REG8(0xa578), 0x04 },
	{ CCI_REG8(0xa579), 0x04 },
	{ CCI_REG8(0xa57a), 0x02 },
	{ CCI_REG8(0xa57b), 0x04 },
	{ CCI_REG8(0xa57c), 0x04 },
	{ CCI_REG8(0xa57d), 0x02 },
	{ CCI_REG8(0xa57e), 0x1a },
	{ CCI_REG8(0xa580), 0x03 },
	{ CCI_REG8(0xa581), 0x1a },
	{ CCI_REG8(0xa583), 0x03 },
	{ CCI_REG8(0xa584), 0x1b },
	{ CCI_REG8(0xa585), 0x15 },
	{ CCI_REG8(0xa586), 0x05 },
	{ CCI_REG8(0xa60d), 0x14 },
	{ CCI_REG8(0xa60f), 0x08 },
	{ CCI_REG8(0xa611), 0x01 },
	{ CCI_REG8(0xa70a), 0x10 },
	{ CCI_REG8(0xa70b), 0x04 },
	{ CCI_REG8(0xa70f), 0x28 },
	{ CCI_REG8(0xa710), 0x28 },
	{ CCI_REG8(0xa711), 0x10 },
	{ CCI_REG8(0xa716), 0x02 },
	{ CCI_REG8(0xa717), 0x02 },
	{ CCI_REG8(0xa71b), 0x31 },
	{ CCI_REG8(0xa71c), 0x15 },
	{ CCI_REG8(0xa71d), 0x08 },
	{ CCI_REG8(0xa736), 0x31 },
	{ CCI_REG8(0xa737), 0x15 },
	{ CCI_REG8(0xa738), 0x08 },
	{ CCI_REG8(0xa751), 0x1b },
	{ CCI_REG8(0xa752), 0x15 },
	{ CCI_REG8(0xa753), 0x05 },
	{ CCI_REG8(0xa76c), 0x1b },
	{ CCI_REG8(0xa76d), 0x15 },
	{ CCI_REG8(0xa76e), 0x05 },
	{ CCI_REG8(0xa801), 0x17 },
	{ CCI_REG8(0xa803), 0x0c },
	{ CCI_REG8(0xa805), 0x00 },
	{ CCI_REG8(0xa811), 0x08 },
	{ CCI_REG8(0xa900), 0x02 },
	{ CCI_REG8(0xa902), 0x02 },
	{ CCI_REG8(0xa907), 0x03 },
	{ CCI_REG8(0xa908), 0x03 },
	{ CCI_REG8(0xa90d), 0x02 },
	{ CCI_REG8(0xa90e), 0x02 },
	{ CCI_REG8(0xa912), 0x00 },
	{ CCI_REG8(0xa918), 0x38 },
	{ CCI_REG8(0xa919), 0x38 },
	{ CCI_REG8(0xa91a), 0x38 },
	{ CCI_REG8(0xaa01), 0x18 },
	{ CCI_REG8(0xaa03), 0x18 },
	{ CCI_REG8(0xaa0f), 0x00 },
	{ CCI_REG8(0xaa11), 0x00 },
	{ CCI_REG8(0xaf01), 0x00 },
	{ CCI_REG8(0xaf03), 0x00 },
	{ CCI_REG8(0xaf05), 0x00 },
	{ CCI_REG8(0xaf0d), 0xaf },
	{ CCI_REG8(0xaf0f), 0x9f },
	{ CCI_REG8(0xe80f), 0x93 },
	{ CCI_REG8(0xe811), 0xfc },
	{ CCI_REG8(0xe827), 0x93 },
	{ CCI_REG8(0xe829), 0xfc },
	{ CCI_REG8(0xe83f), 0x93 },
	{ CCI_REG8(0xe841), 0xfc },
	{ CCI_REG8(0xe845), 0x93 },
	{ CCI_REG8(0xe847), 0xfc },
	{ CCI_REG8(0xe857), 0x34 },
	{ CCI_REG8(0xe859), 0xe9 },
	{ CCI_REG8(0xe86f), 0x34 },
	{ CCI_REG8(0xe871), 0xe9 },
	{ CCI_REG8(0xe875), 0x34 },
	{ CCI_REG8(0xe877), 0xe9 },
};

static const struct cci_reg_sequence imx709_3280x2464_regs[] = {
	{ CCI_REG8(0x0112), 0x0a },
	{ CCI_REG8(0x0113), 0x0a },
	{ CCI_REG8(0x0114), 0x03 },
	{ CCI_REG8(0x0342), 0x2d },
	{ CCI_REG8(0x0343), 0xf0 },
	{ CCI_REG8(0x0344), 0x00 },
	{ CCI_REG8(0x0345), 0x00 },
	{ CCI_REG8(0x0346), 0x00 },
	{ CCI_REG8(0x0347), 0x00 },
	{ CCI_REG8(0x0348), 0x19 },
	{ CCI_REG8(0x0349), 0x9f },
	{ CCI_REG8(0x034a), 0x13 },
	{ CCI_REG8(0x034b), 0x3f },
	{ CCI_REG8(0x0900), 0x01 },
	{ CCI_REG8(0x0901), 0x12 },
	{ CCI_REG8(0x0902), 0x00 },
	{ CCI_REG8(0x3148), 0x04 },
	{ CCI_REG8(0x3150), 0x00 },
	{ CCI_REG8(0x31d0), 0x40 },
	{ CCI_REG8(0x31d1), 0x41 },
	{ CCI_REG8(0x0408), 0x00 },
	{ CCI_REG8(0x0409), 0x00 },
	{ CCI_REG8(0x040a), 0x00 },
	{ CCI_REG8(0x040b), 0x00 },
	{ CCI_REG8(0x040c), 0x0c },
	{ CCI_REG8(0x040d), 0xd0 },
	{ CCI_REG8(0x040e), 0x09 },
	{ CCI_REG8(0x040f), 0xa0 },
	{ CCI_REG8(0x034c), 0x0c },
	{ CCI_REG8(0x034d), 0xd0 },
	{ CCI_REG8(0x034e), 0x09 },
	{ CCI_REG8(0x034f), 0xa0 },
	{ CCI_REG8(0x0303), 0x02 },
	{ CCI_REG8(0x0305), 0x01 },
	{ CCI_REG8(0x0306), 0x00 },
	{ CCI_REG8(0x0307), 0x70 },
	{ CCI_REG8(0x030b), 0x04 },
	{ CCI_REG8(0x030d), 0x08 },
	{ CCI_REG8(0x030e), 0x04 },
	{ CCI_REG8(0x030f), 0xe5 },
	{ CCI_REG8(0x310c), 0x00 },
	{ CCI_REG8(0x3207), 0x00 },
	{ CCI_REG8(0x3214), 0x01 },
	{ CCI_REG8(0x3a00), 0x40 },
	{ CCI_REG8(0x3a01), 0x40 },
	{ CCI_REG8(0x3a90), 0x01 },
	{ CCI_REG8(0x3a91), 0x83 },
	{ CCI_REG8(0x3a94), 0x01 },
	{ CCI_REG8(0x3a95), 0x83 },
	{ CCI_REG8(0x3a96), 0x00 },
	{ CCI_REG8(0x3a97), 0x00 },
	{ CCI_REG8(0x3a9a), 0x00 },
	{ CCI_REG8(0x3a9b), 0x00 },
	{ CCI_REG8(0x3aa0), 0x00 },
	{ CCI_REG8(0x3aa1), 0x00 },
	{ CCI_REG8(0x3aa4), 0x00 },
	{ CCI_REG8(0x3aa5), 0x00 },
	{ CCI_REG8(0x3ab0), 0x00 },
	{ CCI_REG8(0x3ab1), 0x00 },
	{ CCI_REG8(0x3ab2), 0x00 },
	{ CCI_REG8(0x3ab3), 0x00 },
	{ CCI_REG8(0x3ab4), 0x00 },
	{ CCI_REG8(0x3ab5), 0x00 },
	{ CCI_REG8(0x3ab6), 0x00 },
	{ CCI_REG8(0x3ab7), 0x00 },
	{ CCI_REG8(0x3ac8), 0x05 },
	{ CCI_REG8(0x3ac9), 0x6a },
	{ CCI_REG8(0x3aca), 0x05 },
	{ CCI_REG8(0x3acb), 0x6a },
	{ CCI_REG8(0x3ad0), 0x01 },
	{ CCI_REG8(0x3ad1), 0xe7 },
	{ CCI_REG8(0x3ad2), 0x01 },
	{ CCI_REG8(0x3ad3), 0xe7 },
	{ CCI_REG8(0x3ad8), 0x01 },
	{ CCI_REG8(0x3ad9), 0xe7 },
	{ CCI_REG8(0x3adc), 0x01 },
	{ CCI_REG8(0x3add), 0xe7 },
	{ CCI_REG8(0x3c70), 0x00 },
	{ CCI_REG8(0x3c71), 0x00 },
	{ CCI_REG8(0x3c72), 0x00 },
	{ CCI_REG8(0x3c73), 0x00 },
	{ CCI_REG8(0x3c84), 0x00 },
	{ CCI_REG8(0x3c85), 0x00 },
	{ CCI_REG8(0x3c88), 0x00 },
	{ CCI_REG8(0x3c89), 0x00 },
	{ CCI_REG8(0x3c8a), 0x00 },
	{ CCI_REG8(0x3c8b), 0x00 },
	{ CCI_REG8(0x3c9c), 0x00 },
	{ CCI_REG8(0x3c9d), 0x00 },
	{ CCI_REG8(0x3ca0), 0x00 },
	{ CCI_REG8(0x3ca1), 0x00 },
	{ CCI_REG8(0x3ca2), 0x00 },
	{ CCI_REG8(0x3ca3), 0x00 },
	{ CCI_REG8(0x3cb4), 0x00 },
	{ CCI_REG8(0x3cb5), 0x00 },
	{ CCI_REG8(0x3cb8), 0x00 },
	{ CCI_REG8(0x3cb9), 0x00 },
	{ CCI_REG8(0x3cba), 0x00 },
	{ CCI_REG8(0x3cbb), 0x00 },
	{ CCI_REG8(0x3ccc), 0x00 },
	{ CCI_REG8(0x3ccd), 0x00 },
	{ CCI_REG8(0x4f14), 0x01 },
	{ CCI_REG8(0x910e), 0x00 },
	{ CCI_REG8(0x910f), 0x00 },
	{ CCI_REG8(0x9110), 0x00 },
	{ CCI_REG8(0x9154), 0x00 },
	{ CCI_REG8(0x0224), 0x01 },
	{ CCI_REG8(0x0225), 0xf4 },
	{ CCI_REG8(0x0216), 0x00 },
	{ CCI_REG8(0x0217), 0x00 },
	{ CCI_REG8(0x0218), 0x01 },
	{ CCI_REG8(0x0219), 0x00 },
	{ CCI_REG8(0x30c0), 0x00 },
	{ CCI_REG8(0x3080), 0x00 },
	{ CCI_REG8(0x3081), 0x30 },
	{ CCI_REG8(0x3102), 0x01 },
	{ CCI_REG8(0x347a), 0x03 },
	{ CCI_REG8(0x347b), 0x2c },
	{ CCI_REG8(0x3170), 0x00 },
	{ CCI_REG8(0x317c), 0x0a },
	{ CCI_REG8(0x317d), 0x0a },
};

static const struct imx709_mode imx709_modes[] = {
	{
		/*
		 * 2x2 binned; 11760 VT clocks per line at 859.4208 MHz,
		 * 13.68 us, expressed in output pixel clocks.
		 */
		.width = 3280,
		.height = 2464,
		.hts = 4115,
		.vts = 2520,
		.regs = imx709_3280x2464_regs,
		.num_regs = ARRAY_SIZE(imx709_3280x2464_regs),
	},
};

struct imx709 {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct regmap *regmap;
	struct clk *xclk;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[ARRAY_SIZE(imx709_supply_names)];
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *exposure;
};

static inline struct imx709 *to_imx709(struct v4l2_subdev *sd)
{
	return container_of(sd, struct imx709, sd);
}

static const char * const imx709_test_pattern_menu[] = {
	"Disabled",
	"Solid Color",
	"Color Bars",
	"Fade To Grey Color Bars",
	"PN9",
};

static int imx709_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct imx709 *imx709 = to_imx709(sd);
	int i, ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(imx709->supplies),
				    imx709->supplies);
	if (ret) {
		dev_err(dev, "failed to enable regulators: %d\n", ret);
		return ret;
	}

	/* The MCLK sometimes fails to start on the first try after boot */
	for (i = 0; i < 3; i++) {
		ret = clk_prepare_enable(imx709->xclk);
		if (ret != -EBUSY)
			break;
		dev_warn(dev, "clock did not start, retrying\n");
		usleep_range(1000, 2000);
	}
	if (ret) {
		dev_err(dev, "failed to enable clock: %d\n", ret);
		regulator_bulk_disable(ARRAY_SIZE(imx709->supplies),
				       imx709->supplies);
		return ret;
	}

	usleep_range(1000, 1500);
	gpiod_set_value_cansleep(imx709->reset_gpio, 0);
	usleep_range(10000, 12000);

	return 0;
}

static int imx709_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct imx709 *imx709 = to_imx709(sd);

	gpiod_set_value_cansleep(imx709->reset_gpio, 1);
	clk_disable_unprepare(imx709->xclk);
	regulator_bulk_disable(ARRAY_SIZE(imx709->supplies), imx709->supplies);

	return 0;
}

static int imx709_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx709 *imx709 = container_of(ctrl->handler, struct imx709,
					     ctrl_handler);
	const struct imx709_mode *mode = &imx709_modes[0];
	struct device *dev = imx709->sd.dev;
	int ret;

	if (ctrl->id == V4L2_CID_VBLANK) {
		u32 exposure_max = mode->height + ctrl->val -
				   IMX709_EXPOSURE_MARGIN;

		ret = __v4l2_ctrl_modify_range(imx709->exposure,
					       imx709->exposure->minimum,
					       exposure_max,
					       imx709->exposure->step,
					       min(imx709->exposure->default_value,
						   (s64)exposure_max));
		if (ret)
			return ret;
	}

	if (!pm_runtime_get_if_active(dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		ret = cci_write(imx709->regmap, IMX709_REG_EXPOSURE,
				ctrl->val, NULL);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ret = cci_write(imx709->regmap, IMX709_REG_AGAIN,
				ctrl->val, NULL);
		break;
	case V4L2_CID_DIGITAL_GAIN:
		ret = cci_write(imx709->regmap, IMX709_REG_DGAIN,
				ctrl->val, NULL);
		break;
	case V4L2_CID_VBLANK:
		ret = cci_write(imx709->regmap, IMX709_REG_FLL,
				mode->height + ctrl->val, NULL);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = cci_write(imx709->regmap, IMX709_REG_TEST_PATTERN,
				ctrl->val, NULL);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put_autosuspend(dev);

	return ret;
}

static const struct v4l2_ctrl_ops imx709_ctrl_ops = {
	.s_ctrl = imx709_s_ctrl,
};

static int imx709_init_controls(struct imx709 *imx709)
{
	const struct imx709_mode *mode = &imx709_modes[0];
	struct v4l2_ctrl_handler *hdl = &imx709->ctrl_handler;
	struct v4l2_fwnode_device_properties props;
	u32 hblank;
	int ret;

	ret = v4l2_fwnode_device_parse(imx709->sd.dev, &props);
	if (ret)
		return ret;

	v4l2_ctrl_handler_init(hdl, 10);

	imx709->link_freq = v4l2_ctrl_new_int_menu(hdl, NULL,
						   V4L2_CID_LINK_FREQ,
						   ARRAY_SIZE(imx709_link_freqs) - 1,
						   0, imx709_link_freqs);

	v4l2_ctrl_new_std(hdl, NULL, V4L2_CID_PIXEL_RATE, IMX709_PIXEL_RATE,
			  IMX709_PIXEL_RATE, 1, IMX709_PIXEL_RATE);

	v4l2_ctrl_new_std(hdl, &imx709_ctrl_ops, V4L2_CID_VBLANK,
			  mode->vts - mode->height,
			  IMX709_FLL_MAX - mode->height, 1,
			  mode->vts - mode->height);

	hblank = mode->hts - mode->width;
	imx709->hblank = v4l2_ctrl_new_std(hdl, NULL, V4L2_CID_HBLANK,
					   hblank, hblank, 1, hblank);

	imx709->exposure = v4l2_ctrl_new_std(hdl, &imx709_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     IMX709_EXPOSURE_MIN,
					     mode->vts - IMX709_EXPOSURE_MARGIN,
					     1,
					     mode->vts - IMX709_EXPOSURE_MARGIN);

	v4l2_ctrl_new_std(hdl, &imx709_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  IMX709_AGAIN_MIN, IMX709_AGAIN_MAX, 1,
			  IMX709_AGAIN_MIN);

	v4l2_ctrl_new_std(hdl, &imx709_ctrl_ops, V4L2_CID_DIGITAL_GAIN,
			  IMX709_DGAIN_MIN, IMX709_DGAIN_MAX, 1,
			  IMX709_DGAIN_MIN);

	v4l2_ctrl_new_std_menu_items(hdl, &imx709_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(imx709_test_pattern_menu) - 1,
				     0, 0, imx709_test_pattern_menu);

	v4l2_ctrl_new_fwnode_properties(hdl, &imx709_ctrl_ops, &props);

	if (hdl->error) {
		ret = hdl->error;
		v4l2_ctrl_handler_free(hdl);
		return ret;
	}

	imx709->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	imx709->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	imx709->sd.ctrl_handler = hdl;

	return 0;
}

static int imx709_enable_streams(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state, u32 pad,
				 u64 streams_mask)
{
	struct imx709 *imx709 = to_imx709(sd);
	const struct imx709_mode *mode = &imx709_modes[0];
	struct device *dev = sd->dev;
	int ret;

	ret = pm_runtime_resume_and_get(dev);
	if (ret < 0)
		return ret;

	ret = cci_multi_reg_write(imx709->regmap, imx709_init_regs,
				  ARRAY_SIZE(imx709_init_regs), NULL);
	if (ret)
		goto err_pm;

	ret = cci_multi_reg_write(imx709->regmap, mode->regs, mode->num_regs,
				  NULL);
	if (ret)
		goto err_pm;

	ret = __v4l2_ctrl_handler_setup(&imx709->ctrl_handler);
	if (ret)
		goto err_pm;

	ret = cci_write(imx709->regmap, IMX709_REG_MODE_SELECT,
			IMX709_MODE_STREAMING, NULL);
	if (ret)
		goto err_pm;

	return 0;

err_pm:
	dev_err(dev, "failed to start streaming: %d\n", ret);
	pm_runtime_put_autosuspend(dev);
	return ret;
}

static int imx709_disable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state, u32 pad,
				  u64 streams_mask)
{
	struct imx709 *imx709 = to_imx709(sd);
	int ret;

	ret = cci_write(imx709->regmap, IMX709_REG_MODE_SELECT,
			IMX709_MODE_STANDBY, NULL);
	if (ret)
		dev_warn(sd->dev, "failed to stop streaming: %d\n", ret);

	pm_runtime_put_autosuspend(sd->dev);

	return 0;
}

static void imx709_fill_format(const struct imx709_mode *mode,
			       struct v4l2_mbus_framefmt *fmt)
{
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->code = MEDIA_BUS_FMT_SRGGB10_1X10;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_NONE;
}

static int imx709_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_SRGGB10_1X10;

	return 0;
}

static int imx709_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->code != MEDIA_BUS_FMT_SRGGB10_1X10 ||
	    fse->index >= ARRAY_SIZE(imx709_modes))
		return -EINVAL;

	fse->min_width = imx709_modes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = imx709_modes[fse->index].height;
	fse->max_height = fse->min_height;

	return 0;
}

static int imx709_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *state,
			  struct v4l2_subdev_format *fmt)
{
	imx709_fill_format(&imx709_modes[0], &fmt->format);
	*v4l2_subdev_state_get_format(state, 0) = fmt->format;

	return 0;
}

static int imx709_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *state,
				struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = IMX709_NATIVE_WIDTH;
		sel->r.height = IMX709_NATIVE_HEIGHT;
		return 0;
	default:
		return -EINVAL;
	}
}

static int imx709_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	imx709_fill_format(&imx709_modes[0],
			   v4l2_subdev_state_get_format(state, 0));

	return 0;
}

static const struct v4l2_subdev_video_ops imx709_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops imx709_pad_ops = {
	.enum_mbus_code = imx709_enum_mbus_code,
	.enum_frame_size = imx709_enum_frame_size,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = imx709_set_fmt,
	.get_selection = imx709_get_selection,
	.enable_streams = imx709_enable_streams,
	.disable_streams = imx709_disable_streams,
};

static const struct v4l2_subdev_ops imx709_subdev_ops = {
	.video = &imx709_video_ops,
	.pad = &imx709_pad_ops,
};

static const struct v4l2_subdev_internal_ops imx709_internal_ops = {
	.init_state = imx709_init_state,
};

static int imx709_check_hwcfg(struct device *dev)
{
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	struct fwnode_handle *ep;
	unsigned long freq_bitmap;
	int ret;

	ep = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!ep)
		return dev_err_probe(dev, -EINVAL, "no endpoint found\n");

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &bus_cfg);
	fwnode_handle_put(ep);
	if (ret)
		return dev_err_probe(dev, ret, "failed to parse endpoint\n");

	if (bus_cfg.bus.mipi_csi2.num_data_lanes != IMX709_DATA_LANES) {
		ret = dev_err_probe(dev, -EINVAL, "only %u data lanes supported\n",
				    IMX709_DATA_LANES);
		goto out;
	}

	ret = v4l2_link_freq_to_bitmap(dev, bus_cfg.link_frequencies,
				       bus_cfg.nr_of_link_frequencies,
				       imx709_link_freqs,
				       ARRAY_SIZE(imx709_link_freqs),
				       &freq_bitmap);
out:
	v4l2_fwnode_endpoint_free(&bus_cfg);
	return ret;
}

static int imx709_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct imx709 *imx709;
	unsigned int i;
	u64 id = 0;
	int ret;

	imx709 = devm_kzalloc(dev, sizeof(*imx709), GFP_KERNEL);
	if (!imx709)
		return -ENOMEM;

	ret = imx709_check_hwcfg(dev);
	if (ret)
		return ret;

	v4l2_i2c_subdev_init(&imx709->sd, client, &imx709_subdev_ops);

	imx709->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(imx709->regmap))
		return dev_err_probe(dev, PTR_ERR(imx709->regmap),
				     "failed to init CCI regmap\n");

	imx709->xclk = devm_v4l2_sensor_clk_get(dev, NULL);
	if (IS_ERR(imx709->xclk))
		return dev_err_probe(dev, PTR_ERR(imx709->xclk),
				     "failed to get clock\n");

	if (clk_get_rate(imx709->xclk) != IMX709_XCLK_RATE)
		return dev_err_probe(dev, -EINVAL,
				     "unsupported clock rate %lu Hz\n",
				     clk_get_rate(imx709->xclk));

	imx709->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						     GPIOD_OUT_HIGH);
	if (IS_ERR(imx709->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(imx709->reset_gpio),
				     "failed to get reset GPIO\n");

	for (i = 0; i < ARRAY_SIZE(imx709_supply_names); i++)
		imx709->supplies[i].supply = imx709_supply_names[i];

	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(imx709->supplies),
				      imx709->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get regulators\n");

	ret = imx709_power_on(dev);
	if (ret)
		return ret;

	ret = cci_read(imx709->regmap, IMX709_REG_CHIP_ID, &id, NULL);
	if (ret || id != IMX709_CHIP_ID) {
		/* The first power-up after boot occasionally reads zeroes */
		dev_warn(dev, "chip ID 0x%04llx (%d), power cycling\n", id, ret);
		imx709_power_off(dev);
		usleep_range(10000, 11000);
		ret = imx709_power_on(dev);
		if (ret)
			return ret;
		ret = cci_read(imx709->regmap, IMX709_REG_CHIP_ID, &id, NULL);
	}
	if (ret) {
		dev_err_probe(dev, ret, "failed to read chip ID\n");
		goto err_power_off;
	}
	if (id != IMX709_CHIP_ID) {
		ret = dev_err_probe(dev, -ENODEV, "unexpected chip ID 0x%04llx\n",
				    id);
		goto err_power_off;
	}

	ret = imx709_init_controls(imx709);
	if (ret)
		goto err_power_off;

	imx709->sd.internal_ops = &imx709_internal_ops;
	imx709->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	imx709->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	imx709->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&imx709->sd.entity, 1, &imx709->pad);
	if (ret)
		goto err_controls;

	imx709->sd.state_lock = imx709->ctrl_handler.lock;
	ret = v4l2_subdev_init_finalize(&imx709->sd);
	if (ret)
		goto err_entity;

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	ret = v4l2_async_register_subdev_sensor(&imx709->sd);
	if (ret)
		goto err_pm;

	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_idle(dev);

	return 0;

err_pm:
	pm_runtime_disable(dev);
	pm_runtime_set_suspended(dev);
	v4l2_subdev_cleanup(&imx709->sd);
err_entity:
	media_entity_cleanup(&imx709->sd.entity);
err_controls:
	v4l2_ctrl_handler_free(&imx709->ctrl_handler);
err_power_off:
	imx709_power_off(dev);
	return ret;
}

static void imx709_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct device *dev = &client->dev;

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(sd->ctrl_handler);

	pm_runtime_disable(dev);
	if (!pm_runtime_status_suspended(dev)) {
		imx709_power_off(dev);
		pm_runtime_set_suspended(dev);
	}
}

static DEFINE_RUNTIME_DEV_PM_OPS(imx709_pm_ops, imx709_power_off,
				 imx709_power_on, NULL);

static const struct of_device_id imx709_of_match[] = {
	{ .compatible = "sony,imx709" },
	{ }
};
MODULE_DEVICE_TABLE(of, imx709_of_match);

static struct i2c_driver imx709_i2c_driver = {
	.driver = {
		.name = "imx709",
		.pm = pm_ptr(&imx709_pm_ops),
		.of_match_table = imx709_of_match,
	},
	.probe = imx709_probe,
	.remove = imx709_remove,
};
module_i2c_driver(imx709_i2c_driver);

MODULE_DESCRIPTION("Sony IMX709 camera sensor driver");
MODULE_LICENSE("GPL");
