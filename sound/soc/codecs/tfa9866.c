// SPDX-License-Identifier: GPL-2.0-only
/*
 * NXP TFA9866 amplifier driver
 *
 * Register sequences taken from the vendor tfa98xx kernel driver:
 * Copyright (C) 2014-2020 NXP Semiconductors, All Rights Reserved.
 *
 * Copyright (C) 2026 Victor Fuentes <victor@vlinkz.dev>
 */

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <sound/soc.h>

#define TFA9866_SYS_CTRL0		0x00
#define TFA9866_SYS_CTRL0_PWDN		0	/* power down */
#define TFA9866_SYS_CTRL0_I2CR		1	/* I2C reset */
#define TFA9866_SYS_CTRL0_AMPE		3	/* enable amplifier */
#define TFA9866_REVISIONNUMBER		0x03
#define TFA9866_REVISIONNUMBER_REV_MSK	GENMASK(7, 0)
#define TFA9866_CLOCK_CTRL		0x04
#define TFA9866_HIDE_UNHIDE_KEY		0x0f
#define TFA9866_STATUS2			0x12
#define TFA9866_STATUS2_MANSTATE_MSK	GENMASK(3, 0)	/* manager state */
#define TFA9866_TDM_CFG1		0x21
#define TFA9866_TDM_CFG1_SPKS		8	/* speaker input slot */

#define TFA9866_REVISION		0x66

static const struct regmap_config tfa9866_regmap = {
	.reg_bits = 8,
	.val_bits = 16,
};

static const char * const spks_text[] = { "Left", "Right" };
static SOC_ENUM_SINGLE_DECL(spks_enum, TFA9866_TDM_CFG1, TFA9866_TDM_CFG1_SPKS, spks_text);
static const struct snd_kcontrol_new spks_mux = SOC_DAPM_ENUM("Amp Input", spks_enum);

static const struct snd_soc_dapm_widget tfa9866_dapm_widgets[] = {
	SND_SOC_DAPM_OUTPUT("OUT"),
	SND_SOC_DAPM_SUPPLY("POWER", TFA9866_SYS_CTRL0, TFA9866_SYS_CTRL0_PWDN, 1, NULL, 0),
	SND_SOC_DAPM_OUT_DRV("AMPE", TFA9866_SYS_CTRL0, TFA9866_SYS_CTRL0_AMPE, 0, NULL, 0),

	SND_SOC_DAPM_MUX("Amp Input", SND_SOC_NOPM, 0, 0, &spks_mux),
	SND_SOC_DAPM_AIF_IN("AIFINL", "HiFi Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("AIFINR", "HiFi Playback", 1, SND_SOC_NOPM, 0, 0),
};

static const struct snd_soc_dapm_route tfa9866_dapm_routes[] = {
	{"OUT", NULL, "AMPE"},
	{"AMPE", NULL, "POWER"},
	{"AMPE", NULL, "Amp Input"},
	{"Amp Input", "Left", "AIFINL"},
	{"Amp Input", "Right", "AIFINR"},
};

static const struct snd_soc_component_driver tfa9866_component = {
	.dapm_widgets		= tfa9866_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(tfa9866_dapm_widgets),
	.dapm_routes		= tfa9866_dapm_routes,
	.num_dapm_routes	= ARRAY_SIZE(tfa9866_dapm_routes),
	.use_pmdown_time	= 1,
	.endianness		= 1,
};

static struct snd_soc_dai_driver tfa9866_dai = {
	.name = "tfa9866-hifi",
	.playback = {
		.stream_name	= "HiFi Playback",
		.formats	= SNDRV_PCM_FMTBIT_S32_LE,
		.rates		= SNDRV_PCM_RATE_48000,
		.channels_min	= 1,
		.channels_max	= 2,
	},
};

/* Speaker tuning for 48 kHz I2S, with the amplifier powered down */
static const struct reg_sequence tfa9866_reg_init[] = {
	{ TFA9866_HIDE_UNHIDE_KEY, 0x5a6b },
	{ 0x02, 0x0c28 },
	{ 0x50, 0xc000 },
	{ 0x5a, 0x0d28 },
	{ 0x5b, 0x6089 },
	{ 0x5c, 0x330b },
	{ 0x5f, 0x00a0 },
	{ 0x62, 0x05c6 },
	{ 0x63, 0x90d4 },
	{ 0x67, 0x0621 },
	{ 0x68, 0x0914 },
	{ 0x74, 0x6120 },
	{ 0x75, 0x1600 },
	{ 0x76, 0xc000 },
	{ 0x78, 0x0001 },
	{ 0x7c, 0x50f2 },
	{ 0xcf, 0x2cd7 },
	{ 0xd7, 0x1000 },
	{ 0xdd, 0x0036 },
	{ TFA9866_CLOCK_CTRL, 0x0340 },
	{ 0x08, 0x0042 },
	{ TFA9866_TDM_CFG1, 0x0007 },
	{ 0x87, 0x0000 },
	{ 0xc6, 0x0032 },
	{ 0x60, 0x4040 },
	{ TFA9866_SYS_CTRL0, 0xf651 },
	{ 0x01, 0x0084 },
};

static int tfa9866_init(struct regmap *regmap)
{
	unsigned int val;
	int ret;

	ret = regmap_write(regmap, TFA9866_SYS_CTRL0, BIT(TFA9866_SYS_CTRL0_I2CR));
	if (ret)
		return ret;

	/* Power up once to load the OTP shadow registers */
	ret = regmap_write(regmap, 0x90, 0x0001);
	if (ret)
		return ret;

	ret = regmap_write(regmap, TFA9866_SYS_CTRL0, 0xf260);
	if (ret)
		return ret;

	ret = regmap_write(regmap, TFA9866_CLOCK_CTRL, 0x0300);
	if (ret)
		return ret;

	ret = regmap_read_poll_timeout(regmap, TFA9866_STATUS2, val,
				       FIELD_GET(TFA9866_STATUS2_MANSTATE_MSK, val) == 1,
				       1000, 50000);
	if (ret)
		return ret;

	return regmap_multi_reg_write(regmap, tfa9866_reg_init,
				      ARRAY_SIZE(tfa9866_reg_init));
}

static int tfa9866_i2c_probe(struct i2c_client *i2c)
{
	struct device *dev = &i2c->dev;
	struct gpio_desc *reset_gpiod;
	struct regmap *regmap;
	unsigned int val;
	int ret;

	ret = devm_regulator_get_enable(dev, "vddd");
	if (ret)
		return dev_err_probe(dev, ret, "Failed to enable vddd regulator\n");

	reset_gpiod = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(reset_gpiod))
		return PTR_ERR(reset_gpiod);

	if (reset_gpiod) {
		usleep_range(1000, 2000);
		gpiod_set_value_cansleep(reset_gpiod, 0);
		usleep_range(1000, 2000);
	}

	regmap = devm_regmap_init_i2c(i2c, &tfa9866_regmap);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	ret = regmap_read(regmap, TFA9866_REVISIONNUMBER, &val);
	if (ret)
		return dev_err_probe(dev, ret, "failed to read revision number\n");

	val &= TFA9866_REVISIONNUMBER_REV_MSK;
	if (val != TFA9866_REVISION)
		return dev_err_probe(dev, -ENODEV, "invalid revision number %#x\n", val);

	ret = tfa9866_init(regmap);
	if (ret)
		return dev_err_probe(dev, ret, "failed to initialize registers\n");

	return devm_snd_soc_register_component(dev, &tfa9866_component,
					       &tfa9866_dai, 1);
}

static const struct of_device_id tfa9866_of_match[] = {
	{ .compatible = "nxp,tfa9866" },
	{ }
};
MODULE_DEVICE_TABLE(of, tfa9866_of_match);

static struct i2c_driver tfa9866_i2c_driver = {
	.driver = {
		.name = "tfa9866",
		.of_match_table = tfa9866_of_match,
	},
	.probe = tfa9866_i2c_probe,
};
module_i2c_driver(tfa9866_i2c_driver);

MODULE_DESCRIPTION("ASoC NXP/Goodix TFA9866 driver");
MODULE_LICENSE("GPL");
