// SPDX-License-Identifier: GPL-2.0
/*
 * Sony IMX906 50 MP 10-bit RAW MIPI CSI-2 C-PHY sensor driver
 *
 * Register sequences taken from the OnePlus 15 (infiniti) main camera
 * module.
 *
 * Copyright (C) 2026 Victor Fuentes <victor@vlinkz.dev>
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#define IMX906_REG_CHIP_ID		CCI_REG16(0x0016)
#define IMX906_CHIP_ID			0x0906

#define IMX906_REG_MODE_SELECT		CCI_REG8(0x0100)
#define IMX906_MODE_STANDBY		0x00
#define IMX906_MODE_STREAMING		0x01

#define IMX906_REG_EXPOSURE		CCI_REG16(0x0202)
#define IMX906_EXPOSURE_MIN		8
#define IMX906_EXPOSURE_MARGIN		48

/* Analogue gain: code = 16384 - 16384 / gain, 1x to 16x */
#define IMX906_REG_AGAIN		CCI_REG16(0x0204)
#define IMX906_AGAIN_MIN		0
#define IMX906_AGAIN_MAX		0x3c00

/* Digital gain: 8.8 fixed point */
#define IMX906_REG_DGAIN		CCI_REG16(0x020e)
#define IMX906_DGAIN_MIN		0x0100
#define IMX906_DGAIN_MAX		0x0fff

#define IMX906_REG_FLL			CCI_REG16(0x0340)
#define IMX906_FLL_MAX			0xffff

#define IMX906_REG_TEST_PATTERN		CCI_REG16(0x0600)

#define IMX906_XCLK_RATE		19200000
#define IMX906_TRIOS			3
#define IMX906_BPP			10

#define IMX906_NATIVE_WIDTH		8192
#define IMX906_NATIVE_HEIGHT		6144

/*
 * Half the symbol rate per trio: 19.2 MHz / 8 * 1362 = 3268.8 Msps and
 * 19.2 MHz / 8 * 850 = 2040 Msps.
 */
static const s64 imx906_link_freqs[] = {
	1634400000,
	1020000000,
};

/* A C-PHY symbol carries 16/7 bits */
static u64 imx906_pixel_rate(unsigned int link_freq_index)
{
	return div_u64(imx906_link_freqs[link_freq_index] * 2 * IMX906_TRIOS *
		       16, 7 * IMX906_BPP);
}

static const char * const imx906_supply_names[] = {
	"dovdd",
	"avdd",
	"dvdd",
	"avdd1p8",
};

struct imx906_mode {
	u32 width;
	u32 height;
	u32 hts;
	u32 vts;
	unsigned int link_freq_index;
	const struct cci_reg_sequence *regs;
	unsigned int num_regs;
};

static const struct cci_reg_sequence imx906_init_regs[] = {
	{ CCI_REG8(0x0136), 0x13 },
	{ CCI_REG8(0x0138), 0x01 },
	{ CCI_REG8(0x0137), 0x33 },
	{ CCI_REG8(0x3304), 0x00 },
	{ CCI_REG8(0x33f0), 0x04 },
	{ CCI_REG8(0x33f1), 0x06 },
	{ CCI_REG8(0x0111), 0x03 },
	{ CCI_REG8(0x4815), 0x0f },
	{ CCI_REG8(0x614d), 0x00 },
	{ CCI_REG8(0x614f), 0x00 },
	{ CCI_REG8(0x6188), 0x09 },
	{ CCI_REG8(0x6189), 0x09 },
	{ CCI_REG8(0x6190), 0x09 },
	{ CCI_REG8(0x6191), 0x09 },
	{ CCI_REG8(0x6356), 0x13 },
	{ CCI_REG8(0x6358), 0x13 },
	{ CCI_REG8(0x6366), 0x13 },
	{ CCI_REG8(0x6367), 0x13 },
	{ CCI_REG8(0x90e7), 0x01 },
	{ CCI_REG8(0x9200), 0x90 },
	{ CCI_REG8(0x9202), 0xd4 },
	{ CCI_REG8(0x9203), 0xda },
	{ CCI_REG8(0x9204), 0x90 },
	{ CCI_REG8(0x9206), 0xd4 },
	{ CCI_REG8(0x9207), 0xd7 },
	{ CCI_REG8(0x9208), 0x90 },
	{ CCI_REG8(0x920a), 0x7a },
	{ CCI_REG8(0x920b), 0x96 },
	{ CCI_REG8(0xa3f8), 0x0c },
	{ CCI_REG8(0xa429), 0x40 },
	{ CCI_REG8(0xb148), 0x01 },
	{ CCI_REG8(0xb149), 0x61 },
	{ CCI_REG8(0xb14a), 0x01 },
	{ CCI_REG8(0xb14b), 0xdf },
	{ CCI_REG8(0xb14c), 0x02 },
	{ CCI_REG8(0xb14d), 0xd0 },
	{ CCI_REG8(0xb14e), 0x01 },
	{ CCI_REG8(0xb14f), 0x61 },
	{ CCI_REG8(0xb150), 0x01 },
	{ CCI_REG8(0xb151), 0xdf },
	{ CCI_REG8(0xb152), 0x02 },
	{ CCI_REG8(0xb153), 0xd0 },
	{ CCI_REG8(0xb154), 0x01 },
	{ CCI_REG8(0xb155), 0x61 },
	{ CCI_REG8(0xb156), 0x01 },
	{ CCI_REG8(0xb157), 0xdf },
	{ CCI_REG8(0xb158), 0x02 },
	{ CCI_REG8(0xb159), 0xd0 },
	{ CCI_REG8(0xb15a), 0x01 },
	{ CCI_REG8(0xb15b), 0x61 },
	{ CCI_REG8(0xb15c), 0x01 },
	{ CCI_REG8(0xb15d), 0xdf },
	{ CCI_REG8(0xb15e), 0x02 },
	{ CCI_REG8(0xb15f), 0xd0 },
	{ CCI_REG8(0x2433), 0x01 },
	{ CCI_REG8(0xd566), 0x13 },
	{ CCI_REG8(0xd567), 0x13 },
	{ CCI_REG8(0xd556), 0x13 },
	{ CCI_REG8(0xd558), 0x13 },
	{ CCI_REG8(0x2433), 0x00 },
	{ CCI_REG8(0x31a7), 0x06 },
	{ CCI_REG8(0x0101), 0x00 },
	{ CCI_REG8(0x0106), 0x01 },
	{ CCI_REG8(0x3ac0), 0xe6 },
	{ CCI_REG8(0x3ac4), 0xe6 },
	{ CCI_REG8(0x3ac8), 0xe6 },
	{ CCI_REG8(0x3acc), 0xe6 },
	{ CCI_REG8(0x3b80), 0x00 },
	{ CCI_REG8(0x3b81), 0x00 },
	{ CCI_REG8(0x3b82), 0x00 },
	{ CCI_REG8(0x3b83), 0x00 },
	{ CCI_REG8(0x3b8c), 0x00 },
	{ CCI_REG8(0x3b8d), 0x00 },
	{ CCI_REG8(0x3b8e), 0x00 },
	{ CCI_REG8(0x3b8f), 0x00 },
	{ CCI_REG8(0x9bc8), 0x00 },
	{ CCI_REG8(0x9c20), 0x00 },
	{ CCI_REG8(0x9c21), 0x00 },
	{ CCI_REG8(0x9c22), 0x00 },
	{ CCI_REG8(0x9c23), 0x00 },
	{ CCI_REG8(0x9c24), 0x00 },
	{ CCI_REG8(0x9c25), 0x00 },
};

static const struct cci_reg_sequence imx906_4096x3072_regs[] = {
	{ CCI_REG8(0x0112), 0x0a },
	{ CCI_REG8(0x0113), 0x0a },
	{ CCI_REG8(0x0114), 0x02 },
	{ CCI_REG8(0x3239), 0x00 },
	{ CCI_REG8(0x0342), 0x25 },
	{ CCI_REG8(0x0343), 0xd0 },
	{ CCI_REG8(0x3750), 0x00 },
	{ CCI_REG8(0x3751), 0x55 },
	{ CCI_REG8(0x0344), 0x00 },
	{ CCI_REG8(0x0345), 0x00 },
	{ CCI_REG8(0x0346), 0x00 },
	{ CCI_REG8(0x0347), 0x00 },
	{ CCI_REG8(0x0348), 0x1f },
	{ CCI_REG8(0x0349), 0xff },
	{ CCI_REG8(0x034a), 0x17 },
	{ CCI_REG8(0x034b), 0xff },
	{ CCI_REG8(0x0900), 0x01 },
	{ CCI_REG8(0x0901), 0x22 },
	{ CCI_REG8(0x0902), 0x00 },
	{ CCI_REG8(0x3005), 0x02 },
	{ CCI_REG8(0x3144), 0x00 },
	{ CCI_REG8(0x3148), 0x04 },
	{ CCI_REG8(0x3149), 0x01 },
	{ CCI_REG8(0x31c0), 0x41 },
	{ CCI_REG8(0x31c1), 0x41 },
	{ CCI_REG8(0x0408), 0x00 },
	{ CCI_REG8(0x0409), 0x00 },
	{ CCI_REG8(0x040a), 0x00 },
	{ CCI_REG8(0x040b), 0x00 },
	{ CCI_REG8(0x040c), 0x10 },
	{ CCI_REG8(0x040d), 0x00 },
	{ CCI_REG8(0x040e), 0x0c },
	{ CCI_REG8(0x040f), 0x00 },
	{ CCI_REG8(0x034c), 0x10 },
	{ CCI_REG8(0x034d), 0x00 },
	{ CCI_REG8(0x034e), 0x0c },
	{ CCI_REG8(0x034f), 0x00 },
	{ CCI_REG8(0x0301), 0x08 },
	{ CCI_REG8(0x0303), 0x02 },
	{ CCI_REG8(0x0305), 0x03 },
	{ CCI_REG8(0x0306), 0x01 },
	{ CCI_REG8(0x0307), 0x57 },
	{ CCI_REG8(0x030b), 0x01 },
	{ CCI_REG8(0x030d), 0x08 },
	{ CCI_REG8(0x030e), 0x05 },
	{ CCI_REG8(0x030f), 0x52 },
	{ CCI_REG8(0x3205), 0x00 },
	{ CCI_REG8(0x3206), 0x00 },
	{ CCI_REG8(0x3213), 0x01 },
	{ CCI_REG8(0x324c), 0x01 },
	{ CCI_REG8(0x3700), 0x01 },
	{ CCI_REG8(0x3701), 0x02 },
	{ CCI_REG8(0x3702), 0x01 },
	{ CCI_REG8(0x37a0), 0x00 },
	{ CCI_REG8(0x37a1), 0x96 },
	{ CCI_REG8(0x37a8), 0x00 },
	{ CCI_REG8(0x37a9), 0x00 },
	{ CCI_REG8(0x37a2), 0x00 },
	{ CCI_REG8(0x37a3), 0x0e },
	{ CCI_REG8(0x37aa), 0x00 },
	{ CCI_REG8(0x37ab), 0x00 },
	{ CCI_REG8(0x37d0), 0x02 },
	{ CCI_REG8(0x37d1), 0x5c },
	{ CCI_REG8(0x37d2), 0x02 },
	{ CCI_REG8(0x37d3), 0x6c },
	{ CCI_REG8(0x3a00), 0x03 },
	{ CCI_REG8(0x3a01), 0x28 },
	{ CCI_REG8(0x3a04), 0x00 },
	{ CCI_REG8(0x3a05), 0x00 },
	{ CCI_REG8(0x3a06), 0x00 },
	{ CCI_REG8(0x3a07), 0x00 },
	{ CCI_REG8(0x3ac0), 0xcd },
	{ CCI_REG8(0x3ac4), 0xcd },
	{ CCI_REG8(0x3ac8), 0xcd },
	{ CCI_REG8(0x3acc), 0xcd },
	{ CCI_REG8(0x3b80), 0x00 },
	{ CCI_REG8(0x3b81), 0x00 },
	{ CCI_REG8(0x3b82), 0x00 },
	{ CCI_REG8(0x3b83), 0x00 },
	{ CCI_REG8(0x3b8c), 0x00 },
	{ CCI_REG8(0x3b8d), 0x00 },
	{ CCI_REG8(0x3b8e), 0x00 },
	{ CCI_REG8(0x3b8f), 0x00 },
	{ CCI_REG8(0x9bc8), 0x00 },
	{ CCI_REG8(0x9c20), 0x00 },
	{ CCI_REG8(0x9c21), 0x00 },
	{ CCI_REG8(0x9c22), 0x00 },
	{ CCI_REG8(0x9c23), 0x00 },
	{ CCI_REG8(0x9c24), 0x00 },
	{ CCI_REG8(0x9c25), 0x00 },
	{ CCI_REG8(0x0224), 0x01 },
	{ CCI_REG8(0x0225), 0xf4 },
	{ CCI_REG8(0x3172), 0x01 },
	{ CCI_REG8(0x3173), 0xf4 },
	{ CCI_REG8(0x317a), 0x01 },
	{ CCI_REG8(0x317b), 0xf4 },
	{ CCI_REG8(0x0216), 0x00 },
	{ CCI_REG8(0x0217), 0x00 },
	{ CCI_REG8(0x0218), 0x01 },
	{ CCI_REG8(0x0219), 0x00 },
	{ CCI_REG8(0x3174), 0x00 },
	{ CCI_REG8(0x3175), 0x00 },
	{ CCI_REG8(0x3176), 0x01 },
	{ CCI_REG8(0x3177), 0x00 },
	{ CCI_REG8(0x317c), 0x00 },
	{ CCI_REG8(0x317d), 0x00 },
	{ CCI_REG8(0x317e), 0x01 },
	{ CCI_REG8(0x317f), 0x00 },
	{ CCI_REG8(0x3184), 0x00 },
	{ CCI_REG8(0x3186), 0x00 },
	{ CCI_REG8(0x320b), 0x00 },
	{ CCI_REG8(0x4400), 0x00 },
	{ CCI_REG8(0x4401), 0x00 },
	{ CCI_REG8(0x310b), 0x00 },
	{ CCI_REG8(0x3107), 0x00 },
	{ CCI_REG8(0x3268), 0x00 },
	{ CCI_REG8(0x3104), 0x01 },
	{ CCI_REG8(0x3879), 0x00 },
	{ CCI_REG8(0x3103), 0x00 },
	{ CCI_REG8(0x3422), 0x00 },
	{ CCI_REG8(0x3423), 0x00 },
	{ CCI_REG8(0x3878), 0x00 },
	{ CCI_REG8(0x3190), 0x00 },
	{ CCI_REG8(0x3191), 0x00 },
	{ CCI_REG8(0x3880), 0x00 },
	{ CCI_REG8(0x3884), 0x00 },
	{ CCI_REG8(0x3886), 0x00 },
	{ CCI_REG8(0x0e00), 0x00 },
	{ CCI_REG8(0x0808), 0x02 },
	{ CCI_REG8(0x084e), 0x00 },
	{ CCI_REG8(0x084f), 0x29 },
	{ CCI_REG8(0x0850), 0x00 },
	{ CCI_REG8(0x0851), 0x19 },
	{ CCI_REG8(0x0852), 0x00 },
	{ CCI_REG8(0x0853), 0x2f },
	{ CCI_REG8(0x0854), 0x00 },
	{ CCI_REG8(0x0855), 0x29 },
	{ CCI_REG8(0x0858), 0x00 },
	{ CCI_REG8(0x0859), 0x19 },
};

static const struct cci_reg_sequence imx906_4096x2304_regs[] = {
	{ CCI_REG8(0x0112), 0x0a },
	{ CCI_REG8(0x0113), 0x0a },
	{ CCI_REG8(0x0114), 0x02 },
	{ CCI_REG8(0x3239), 0x00 },
	{ CCI_REG8(0x0342), 0x25 },
	{ CCI_REG8(0x0343), 0xd0 },
	{ CCI_REG8(0x3750), 0x00 },
	{ CCI_REG8(0x3751), 0x55 },
	{ CCI_REG8(0x0344), 0x00 },
	{ CCI_REG8(0x0345), 0x00 },
	{ CCI_REG8(0x0346), 0x03 },
	{ CCI_REG8(0x0347), 0x00 },
	{ CCI_REG8(0x0348), 0x1f },
	{ CCI_REG8(0x0349), 0xff },
	{ CCI_REG8(0x034a), 0x14 },
	{ CCI_REG8(0x034b), 0xff },
	{ CCI_REG8(0x0900), 0x01 },
	{ CCI_REG8(0x0901), 0x22 },
	{ CCI_REG8(0x0902), 0x00 },
	{ CCI_REG8(0x3005), 0x02 },
	{ CCI_REG8(0x3144), 0x00 },
	{ CCI_REG8(0x3148), 0x04 },
	{ CCI_REG8(0x3149), 0x01 },
	{ CCI_REG8(0x31c0), 0x41 },
	{ CCI_REG8(0x31c1), 0x41 },
	{ CCI_REG8(0x0408), 0x00 },
	{ CCI_REG8(0x0409), 0x00 },
	{ CCI_REG8(0x040a), 0x00 },
	{ CCI_REG8(0x040b), 0x00 },
	{ CCI_REG8(0x040c), 0x10 },
	{ CCI_REG8(0x040d), 0x00 },
	{ CCI_REG8(0x040e), 0x09 },
	{ CCI_REG8(0x040f), 0x00 },
	{ CCI_REG8(0x034c), 0x10 },
	{ CCI_REG8(0x034d), 0x00 },
	{ CCI_REG8(0x034e), 0x09 },
	{ CCI_REG8(0x034f), 0x00 },
	{ CCI_REG8(0x0301), 0x08 },
	{ CCI_REG8(0x0303), 0x02 },
	{ CCI_REG8(0x0305), 0x03 },
	{ CCI_REG8(0x0306), 0x01 },
	{ CCI_REG8(0x0307), 0x57 },
	{ CCI_REG8(0x030b), 0x01 },
	{ CCI_REG8(0x030d), 0x08 },
	{ CCI_REG8(0x030e), 0x03 },
	{ CCI_REG8(0x030f), 0x52 },
	{ CCI_REG8(0x3205), 0x00 },
	{ CCI_REG8(0x3206), 0x00 },
	{ CCI_REG8(0x3213), 0x01 },
	{ CCI_REG8(0x324c), 0x01 },
	{ CCI_REG8(0x3700), 0x01 },
	{ CCI_REG8(0x3701), 0x02 },
	{ CCI_REG8(0x3702), 0x01 },
	{ CCI_REG8(0x37a0), 0x00 },
	{ CCI_REG8(0x37a1), 0x96 },
	{ CCI_REG8(0x37a8), 0x00 },
	{ CCI_REG8(0x37a9), 0x00 },
	{ CCI_REG8(0x37a2), 0x00 },
	{ CCI_REG8(0x37a3), 0x0e },
	{ CCI_REG8(0x37aa), 0x00 },
	{ CCI_REG8(0x37ab), 0x00 },
	{ CCI_REG8(0x37d0), 0x02 },
	{ CCI_REG8(0x37d1), 0x5c },
	{ CCI_REG8(0x37d2), 0x02 },
	{ CCI_REG8(0x37d3), 0x6c },
	{ CCI_REG8(0x3a00), 0x03 },
	{ CCI_REG8(0x3a01), 0x28 },
	{ CCI_REG8(0x3a06), 0x00 },
	{ CCI_REG8(0x3a07), 0x00 },
	{ CCI_REG8(0x3a04), 0x00 },
	{ CCI_REG8(0x3a05), 0x00 },
	{ CCI_REG8(0x3ac0), 0xcd },
	{ CCI_REG8(0x3ac4), 0xcd },
	{ CCI_REG8(0x3ac8), 0xcd },
	{ CCI_REG8(0x3acc), 0xcd },
	{ CCI_REG8(0x3b80), 0x00 },
	{ CCI_REG8(0x3b81), 0x00 },
	{ CCI_REG8(0x3b82), 0x00 },
	{ CCI_REG8(0x3b83), 0x00 },
	{ CCI_REG8(0x3b8c), 0x00 },
	{ CCI_REG8(0x3b8d), 0x00 },
	{ CCI_REG8(0x3b8e), 0x00 },
	{ CCI_REG8(0x3b8f), 0x00 },
	{ CCI_REG8(0x9bc8), 0x00 },
	{ CCI_REG8(0x9c20), 0x00 },
	{ CCI_REG8(0x9c21), 0x00 },
	{ CCI_REG8(0x9c22), 0x00 },
	{ CCI_REG8(0x9c23), 0x00 },
	{ CCI_REG8(0x9c24), 0x00 },
	{ CCI_REG8(0x9c25), 0x00 },
	{ CCI_REG8(0x0224), 0x01 },
	{ CCI_REG8(0x0225), 0xf4 },
	{ CCI_REG8(0x3172), 0x01 },
	{ CCI_REG8(0x3173), 0xf4 },
	{ CCI_REG8(0x317a), 0x01 },
	{ CCI_REG8(0x317b), 0xf4 },
	{ CCI_REG8(0x0216), 0x00 },
	{ CCI_REG8(0x0217), 0x00 },
	{ CCI_REG8(0x0218), 0x01 },
	{ CCI_REG8(0x0219), 0x00 },
	{ CCI_REG8(0x3174), 0x00 },
	{ CCI_REG8(0x3175), 0x00 },
	{ CCI_REG8(0x3176), 0x01 },
	{ CCI_REG8(0x3177), 0x00 },
	{ CCI_REG8(0x317c), 0x00 },
	{ CCI_REG8(0x317d), 0x00 },
	{ CCI_REG8(0x317e), 0x01 },
	{ CCI_REG8(0x317f), 0x00 },
	{ CCI_REG8(0x3184), 0x00 },
	{ CCI_REG8(0x3186), 0x00 },
	{ CCI_REG8(0x320b), 0x00 },
	{ CCI_REG8(0x4400), 0x00 },
	{ CCI_REG8(0x4401), 0x00 },
	{ CCI_REG8(0x310b), 0x00 },
	{ CCI_REG8(0x3107), 0x00 },
	{ CCI_REG8(0x3268), 0x00 },
	{ CCI_REG8(0x3104), 0x01 },
	{ CCI_REG8(0x3879), 0x00 },
	{ CCI_REG8(0x3103), 0x00 },
	{ CCI_REG8(0x3422), 0x00 },
	{ CCI_REG8(0x3423), 0x00 },
	{ CCI_REG8(0x3878), 0x00 },
	{ CCI_REG8(0x3190), 0x00 },
	{ CCI_REG8(0x3191), 0x00 },
	{ CCI_REG8(0x3880), 0x00 },
	{ CCI_REG8(0x3884), 0x00 },
	{ CCI_REG8(0x3886), 0x00 },
	{ CCI_REG8(0x0e00), 0x00 },
	{ CCI_REG8(0x0808), 0x02 },
	{ CCI_REG8(0x084e), 0x00 },
	{ CCI_REG8(0x084f), 0x19 },
	{ CCI_REG8(0x0850), 0x00 },
	{ CCI_REG8(0x0851), 0x11 },
	{ CCI_REG8(0x0852), 0x00 },
	{ CCI_REG8(0x0853), 0x1f },
	{ CCI_REG8(0x0854), 0x00 },
	{ CCI_REG8(0x0855), 0x29 },
	{ CCI_REG8(0x0858), 0x00 },
	{ CCI_REG8(0x0859), 0x19 },
};

static const struct imx906_mode imx906_modes[] = {
	{
		/*
		 * 2x2 binned; 9680 VT clocks per line at 2195.2 MHz,
		 * 4.41 us, expressed in output pixel clocks.
		 */
		.width = 4096,
		.height = 3072,
		.hts = 9885,
		.vts = 7528,
		.link_freq_index = 0,
		.regs = imx906_4096x3072_regs,
		.num_regs = ARRAY_SIZE(imx906_4096x3072_regs),
	},
	{
		/* 2x2 binned, 16:9, 60 fps */
		.width = 4096,
		.height = 2304,
		.hts = 6169,
		.vts = 3764,
		.link_freq_index = 1,
		.regs = imx906_4096x2304_regs,
		.num_regs = ARRAY_SIZE(imx906_4096x2304_regs),
	},
};

struct imx906 {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct regmap *regmap;
	struct clk *xclk;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[ARRAY_SIZE(imx906_supply_names)];
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *exposure;
	const struct imx906_mode *mode;
};

static inline struct imx906 *to_imx906(struct v4l2_subdev *sd)
{
	return container_of(sd, struct imx906, sd);
}

static const char * const imx906_test_pattern_menu[] = {
	"Disabled",
	"Solid Color",
	"Color Bars",
	"Fade To Grey Color Bars",
	"PN9",
};

static int imx906_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct imx906 *imx906 = to_imx906(sd);
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(imx906->supplies),
				    imx906->supplies);
	if (ret) {
		dev_err(dev, "failed to enable regulators: %d\n", ret);
		return ret;
	}

	ret = clk_prepare_enable(imx906->xclk);
	if (ret) {
		dev_err(dev, "failed to enable clock: %d\n", ret);
		regulator_bulk_disable(ARRAY_SIZE(imx906->supplies),
				       imx906->supplies);
		return ret;
	}

	usleep_range(1000, 1500);
	gpiod_set_value_cansleep(imx906->reset_gpio, 0);
	usleep_range(10000, 12000);

	return 0;
}

static int imx906_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct imx906 *imx906 = to_imx906(sd);

	gpiod_set_value_cansleep(imx906->reset_gpio, 1);
	clk_disable_unprepare(imx906->xclk);
	regulator_bulk_disable(ARRAY_SIZE(imx906->supplies), imx906->supplies);

	return 0;
}

static int imx906_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx906 *imx906 = container_of(ctrl->handler, struct imx906,
					     ctrl_handler);
	const struct imx906_mode *mode = imx906->mode;
	struct device *dev = imx906->sd.dev;
	int ret;

	if (ctrl->id == V4L2_CID_VBLANK) {
		u32 exposure_max = mode->height + ctrl->val -
				   IMX906_EXPOSURE_MARGIN;

		ret = __v4l2_ctrl_modify_range(imx906->exposure,
					       imx906->exposure->minimum,
					       exposure_max,
					       imx906->exposure->step,
					       min(imx906->exposure->default_value,
						   (s64)exposure_max));
		if (ret)
			return ret;
	}

	if (!pm_runtime_get_if_active(dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		ret = cci_write(imx906->regmap, IMX906_REG_EXPOSURE,
				ctrl->val, NULL);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ret = cci_write(imx906->regmap, IMX906_REG_AGAIN,
				ctrl->val, NULL);
		break;
	case V4L2_CID_DIGITAL_GAIN:
		ret = cci_write(imx906->regmap, IMX906_REG_DGAIN,
				ctrl->val, NULL);
		break;
	case V4L2_CID_VBLANK:
		ret = cci_write(imx906->regmap, IMX906_REG_FLL,
				mode->height + ctrl->val, NULL);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = cci_write(imx906->regmap, IMX906_REG_TEST_PATTERN,
				ctrl->val, NULL);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put_autosuspend(dev);

	return ret;
}

static const struct v4l2_ctrl_ops imx906_ctrl_ops = {
	.s_ctrl = imx906_s_ctrl,
};

static int imx906_init_controls(struct imx906 *imx906)
{
	const struct imx906_mode *mode = imx906->mode;
	struct v4l2_ctrl_handler *hdl = &imx906->ctrl_handler;
	struct v4l2_fwnode_device_properties props;
	u32 hblank;
	int ret;

	ret = v4l2_fwnode_device_parse(imx906->sd.dev, &props);
	if (ret)
		return ret;

	v4l2_ctrl_handler_init(hdl, 10);

	imx906->link_freq = v4l2_ctrl_new_int_menu(hdl, NULL,
						   V4L2_CID_LINK_FREQ,
						   ARRAY_SIZE(imx906_link_freqs) - 1,
						   mode->link_freq_index,
						   imx906_link_freqs);

	imx906->pixel_rate = v4l2_ctrl_new_std(hdl, NULL, V4L2_CID_PIXEL_RATE,
					       imx906_pixel_rate(1),
					       imx906_pixel_rate(0), 1,
					       imx906_pixel_rate(mode->link_freq_index));

	imx906->vblank = v4l2_ctrl_new_std(hdl, &imx906_ctrl_ops, V4L2_CID_VBLANK,
					   mode->vts - mode->height,
					   IMX906_FLL_MAX - mode->height, 1,
					   mode->vts - mode->height);

	hblank = mode->hts - mode->width;
	imx906->hblank = v4l2_ctrl_new_std(hdl, NULL, V4L2_CID_HBLANK,
					   hblank, hblank, 1, hblank);

	imx906->exposure = v4l2_ctrl_new_std(hdl, &imx906_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     IMX906_EXPOSURE_MIN,
					     mode->vts - IMX906_EXPOSURE_MARGIN,
					     1,
					     mode->vts - IMX906_EXPOSURE_MARGIN);

	v4l2_ctrl_new_std(hdl, &imx906_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  IMX906_AGAIN_MIN, IMX906_AGAIN_MAX, 1,
			  IMX906_AGAIN_MIN);

	v4l2_ctrl_new_std(hdl, &imx906_ctrl_ops, V4L2_CID_DIGITAL_GAIN,
			  IMX906_DGAIN_MIN, IMX906_DGAIN_MAX, 1,
			  IMX906_DGAIN_MIN);

	v4l2_ctrl_new_std_menu_items(hdl, &imx906_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(imx906_test_pattern_menu) - 1,
				     0, 0, imx906_test_pattern_menu);

	v4l2_ctrl_new_fwnode_properties(hdl, &imx906_ctrl_ops, &props);

	if (hdl->error) {
		ret = hdl->error;
		v4l2_ctrl_handler_free(hdl);
		return ret;
	}

	imx906->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	imx906->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	imx906->sd.ctrl_handler = hdl;

	return 0;
}

static int imx906_enable_streams(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state, u32 pad,
				 u64 streams_mask)
{
	struct imx906 *imx906 = to_imx906(sd);
	const struct imx906_mode *mode = imx906->mode;
	struct device *dev = sd->dev;
	int ret;

	ret = pm_runtime_resume_and_get(dev);
	if (ret < 0)
		return ret;

	ret = cci_multi_reg_write(imx906->regmap, imx906_init_regs,
				  ARRAY_SIZE(imx906_init_regs), NULL);
	if (ret)
		goto err_pm;

	ret = cci_multi_reg_write(imx906->regmap, mode->regs, mode->num_regs,
				  NULL);
	if (ret)
		goto err_pm;

	ret = __v4l2_ctrl_handler_setup(&imx906->ctrl_handler);
	if (ret)
		goto err_pm;

	ret = cci_write(imx906->regmap, IMX906_REG_MODE_SELECT,
			IMX906_MODE_STREAMING, NULL);
	if (ret)
		goto err_pm;

	return 0;

err_pm:
	dev_err(dev, "failed to start streaming: %d\n", ret);
	pm_runtime_put_autosuspend(dev);
	return ret;
}

static int imx906_disable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state, u32 pad,
				  u64 streams_mask)
{
	struct imx906 *imx906 = to_imx906(sd);
	int ret;

	ret = cci_write(imx906->regmap, IMX906_REG_MODE_SELECT,
			IMX906_MODE_STANDBY, NULL);
	if (ret)
		dev_warn(sd->dev, "failed to stop streaming: %d\n", ret);

	pm_runtime_put_autosuspend(sd->dev);

	return 0;
}

static void imx906_fill_format(const struct imx906_mode *mode,
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

static int imx906_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_SRGGB10_1X10;

	return 0;
}

static int imx906_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->code != MEDIA_BUS_FMT_SRGGB10_1X10 ||
	    fse->index >= ARRAY_SIZE(imx906_modes))
		return -EINVAL;

	fse->min_width = imx906_modes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = imx906_modes[fse->index].height;
	fse->max_height = fse->min_height;

	return 0;
}

static int imx906_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *state,
			  struct v4l2_subdev_format *fmt)
{
	struct imx906 *imx906 = to_imx906(sd);
	const struct imx906_mode *mode;
	u32 hblank, vblank;
	int ret;

	mode = v4l2_find_nearest_size(imx906_modes, ARRAY_SIZE(imx906_modes),
				      width, height, fmt->format.width,
				      fmt->format.height);
	imx906_fill_format(mode, &fmt->format);
	*v4l2_subdev_state_get_format(state, 0) = fmt->format;

	if (fmt->which != V4L2_SUBDEV_FORMAT_ACTIVE)
		return 0;

	imx906->mode = mode;

	ret = __v4l2_ctrl_s_ctrl(imx906->link_freq, mode->link_freq_index);
	if (ret)
		return ret;

	ret = __v4l2_ctrl_s_ctrl_int64(imx906->pixel_rate,
				       imx906_pixel_rate(mode->link_freq_index));
	if (ret)
		return ret;

	hblank = mode->hts - mode->width;
	ret = __v4l2_ctrl_modify_range(imx906->hblank, hblank, hblank, 1,
				       hblank);
	if (ret)
		return ret;

	vblank = mode->vts - mode->height;
	ret = __v4l2_ctrl_modify_range(imx906->vblank, vblank,
				       IMX906_FLL_MAX - mode->height, 1,
				       vblank);
	if (ret)
		return ret;

	return __v4l2_ctrl_s_ctrl(imx906->vblank, vblank);
}

static int imx906_get_selection(struct v4l2_subdev *sd,
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
		sel->r.width = IMX906_NATIVE_WIDTH;
		sel->r.height = IMX906_NATIVE_HEIGHT;
		return 0;
	default:
		return -EINVAL;
	}
}

static int imx906_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	imx906_fill_format(&imx906_modes[0],
			   v4l2_subdev_state_get_format(state, 0));

	return 0;
}

static const struct v4l2_subdev_video_ops imx906_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops imx906_pad_ops = {
	.enum_mbus_code = imx906_enum_mbus_code,
	.enum_frame_size = imx906_enum_frame_size,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = imx906_set_fmt,
	.get_selection = imx906_get_selection,
	.enable_streams = imx906_enable_streams,
	.disable_streams = imx906_disable_streams,
};

static const struct v4l2_subdev_ops imx906_subdev_ops = {
	.video = &imx906_video_ops,
	.pad = &imx906_pad_ops,
};

static const struct v4l2_subdev_internal_ops imx906_internal_ops = {
	.init_state = imx906_init_state,
};

static int imx906_check_hwcfg(struct device *dev)
{
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_CPHY,
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

	if (bus_cfg.bus.mipi_csi2.num_data_lanes != IMX906_TRIOS) {
		ret = dev_err_probe(dev, -EINVAL, "only %u trios supported\n",
				    IMX906_TRIOS);
		goto out;
	}

	ret = v4l2_link_freq_to_bitmap(dev, bus_cfg.link_frequencies,
				       bus_cfg.nr_of_link_frequencies,
				       imx906_link_freqs,
				       ARRAY_SIZE(imx906_link_freqs),
				       &freq_bitmap);
out:
	v4l2_fwnode_endpoint_free(&bus_cfg);
	return ret;
}

static int imx906_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct imx906 *imx906;
	unsigned int i;
	u64 id;
	int ret;

	imx906 = devm_kzalloc(dev, sizeof(*imx906), GFP_KERNEL);
	if (!imx906)
		return -ENOMEM;

	ret = imx906_check_hwcfg(dev);
	if (ret)
		return ret;

	v4l2_i2c_subdev_init(&imx906->sd, client, &imx906_subdev_ops);

	imx906->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(imx906->regmap))
		return dev_err_probe(dev, PTR_ERR(imx906->regmap),
				     "failed to init CCI regmap\n");

	imx906->xclk = devm_v4l2_sensor_clk_get(dev, NULL);
	if (IS_ERR(imx906->xclk))
		return dev_err_probe(dev, PTR_ERR(imx906->xclk),
				     "failed to get clock\n");

	if (clk_get_rate(imx906->xclk) != IMX906_XCLK_RATE)
		return dev_err_probe(dev, -EINVAL,
				     "unsupported clock rate %lu Hz\n",
				     clk_get_rate(imx906->xclk));

	imx906->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						     GPIOD_OUT_HIGH);
	if (IS_ERR(imx906->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(imx906->reset_gpio),
				     "failed to get reset GPIO\n");

	for (i = 0; i < ARRAY_SIZE(imx906_supply_names); i++)
		imx906->supplies[i].supply = imx906_supply_names[i];

	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(imx906->supplies),
				      imx906->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get regulators\n");

	ret = imx906_power_on(dev);
	if (ret)
		return ret;

	ret = cci_read(imx906->regmap, IMX906_REG_CHIP_ID, &id, NULL);
	if (ret) {
		dev_err_probe(dev, ret, "failed to read chip ID\n");
		goto err_power_off;
	}
	if (id != IMX906_CHIP_ID) {
		ret = dev_err_probe(dev, -ENODEV, "unexpected chip ID 0x%04llx\n",
				    id);
		goto err_power_off;
	}

	imx906->mode = &imx906_modes[0];

	ret = imx906_init_controls(imx906);
	if (ret)
		goto err_power_off;

	imx906->sd.internal_ops = &imx906_internal_ops;
	imx906->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	imx906->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	imx906->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&imx906->sd.entity, 1, &imx906->pad);
	if (ret)
		goto err_controls;

	imx906->sd.state_lock = imx906->ctrl_handler.lock;
	ret = v4l2_subdev_init_finalize(&imx906->sd);
	if (ret)
		goto err_entity;

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	ret = v4l2_async_register_subdev_sensor(&imx906->sd);
	if (ret)
		goto err_pm;

	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_idle(dev);

	return 0;

err_pm:
	pm_runtime_disable(dev);
	pm_runtime_set_suspended(dev);
	v4l2_subdev_cleanup(&imx906->sd);
err_entity:
	media_entity_cleanup(&imx906->sd.entity);
err_controls:
	v4l2_ctrl_handler_free(&imx906->ctrl_handler);
err_power_off:
	imx906_power_off(dev);
	return ret;
}

static void imx906_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct device *dev = &client->dev;

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(sd->ctrl_handler);

	pm_runtime_disable(dev);
	if (!pm_runtime_status_suspended(dev)) {
		imx906_power_off(dev);
		pm_runtime_set_suspended(dev);
	}
}

static DEFINE_RUNTIME_DEV_PM_OPS(imx906_pm_ops, imx906_power_off,
				 imx906_power_on, NULL);

static const struct of_device_id imx906_of_match[] = {
	{ .compatible = "sony,imx906" },
	{ }
};
MODULE_DEVICE_TABLE(of, imx906_of_match);

static struct i2c_driver imx906_i2c_driver = {
	.driver = {
		.name = "imx906",
		.pm = pm_ptr(&imx906_pm_ops),
		.of_match_table = imx906_of_match,
	},
	.probe = imx906_probe,
	.remove = imx906_remove,
};
module_i2c_driver(imx906_i2c_driver);

MODULE_DESCRIPTION("Sony IMX906 camera sensor driver");
MODULE_LICENSE("GPL");
