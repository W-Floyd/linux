// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2026 William Floyd <git@notmy.space>
//
// Driver for the Sony IMX320 8MP image sensor.
//
// The register sequences and mode timings were recovered from the Qualcomm
// CHI sensor module description shipped with the vendor camera stack
// (com.qti.sensormodule.1_imx320.bin). Link frequencies are derived from
// each mode's PLL dividers rather than taken from a datasheet.

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/unaligned.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#define IMX320_REG_MODE_SELECT		0x0100
#define IMX320_MODE_STANDBY		0x00
#define IMX320_MODE_STREAMING		0x01

#define IMX320_REG_CHIP_ID		0x0016
#define IMX320_CHIP_ID			0x0320

/* V-timing */
#define IMX320_REG_FLL			0x0340
#define IMX320_FLL_MAX			0xffff

/* H-timing */
#define IMX320_REG_LLP			0x0342

#define IMX320_REG_EXPOSURE		0x0202
#define IMX320_EXPOSURE_MIN		1
#define IMX320_EXPOSURE_STEP		1
#define IMX320_EXPOSURE_DEFAULT		0x0282
/* Frame length lines minus this is the maximum coarse integration time */
#define IMX320_EXPOSURE_OFFSET		10

/* Analogue gain is gain = 1024 / (1024 - code) */
#define IMX320_REG_ANALOG_GAIN		0x0204
#define IMX320_ANA_GAIN_MIN		0
#define IMX320_ANA_GAIN_MAX		960
#define IMX320_ANA_GAIN_STEP		1
#define IMX320_ANA_GAIN_DEFAULT		0

/* Digital gain is a 8.8 fixed point multiplier, 0x0100 being unity */
#define IMX320_REG_DIG_GAIN		0x020e
#define IMX320_DGTL_GAIN_MIN		256
#define IMX320_DGTL_GAIN_MAX		4095
#define IMX320_DGTL_GAIN_STEP		1
#define IMX320_DGTL_GAIN_DEFAULT	256

#define IMX320_REG_TEST_PATTERN		0x0600
#define IMX320_TEST_PATTERN_DISABLE	0

#define IMX320_XCLK_FREQ		24000000
#define IMX320_DATA_LANES		2
#define IMX320_RGB_DEPTH		10

/* Time from XCLR release to the sensor accepting I2C, T7 in Sony datasheets */
#define IMX320_XCLR_MIN_DELAY_US	8000
#define IMX320_XCLR_DELAY_RANGE_US	1000

struct imx320_reg {
	u16 address;
	u8 val;
};

struct imx320_reg_list {
	u32 num_of_regs;
	const struct imx320_reg *regs;
};

struct imx320_mode {
	u32 width;
	u32 height;

	/* Frame length lines, as programmed by reg_list */
	u32 fll_def;
	u32 fll_min;

	/* Line length pixels, as programmed by reg_list */
	u32 llp;

	/* Index into imx320_link_freq_menu[] */
	u32 link_freq_index;

	struct imx320_reg_list reg_list;
};

/* Link frequencies, computed from each mode's PLL dividers */
static const s64 imx320_link_freq_menu[] = {
	172500000,
	681000000,
	738000000,
};

/* 2640x1980 RAW10, LLP 3872, FLL 2594, binning 1x1, link 681 MHz */
static const struct imx320_reg mode_2640x1980_regs[] = {
	{ 0x0112, 0x0a },
	{ 0x0113, 0x0a },
	{ 0x0114, 0x01 },
	{ 0x0220, 0x61 },
	{ 0x0221, 0x11 },
	{ 0x0340, 0x0a },
	{ 0x0341, 0x22 },
	{ 0x0342, 0x0f },
	{ 0x0343, 0x20 },
	{ 0x0381, 0x01 },
	{ 0x0383, 0x01 },
	{ 0x0385, 0x01 },
	{ 0x0387, 0x01 },
	{ 0x0900, 0x00 },
	{ 0x0901, 0x11 },
	{ 0x0902, 0x00 },
	{ 0x3e6a, 0x01 },
	{ 0x3f4c, 0x01 },
	{ 0x3f4d, 0x01 },
	{ 0x3f7c, 0x00 },
	{ 0x3f7d, 0x28 },
	{ 0x5c8e, 0x0e },
	{ 0x5ddd, 0x0e },
	{ 0x5e0e, 0x0e },
	{ 0x3420, 0x02 },
	{ 0x0344, 0x00 },
	{ 0x0345, 0x00 },
	{ 0x0346, 0x00 },
	{ 0x0347, 0x00 },
	{ 0x0348, 0x0c },
	{ 0x0349, 0xcf },
	{ 0x034a, 0x09 },
	{ 0x034b, 0x9f },
	{ 0x034c, 0x0a },
	{ 0x034d, 0x50 },
	{ 0x034e, 0x07 },
	{ 0x034f, 0xbc },
	{ 0x0408, 0x01 },
	{ 0x0409, 0x40 },
	{ 0x040a, 0x00 },
	{ 0x040b, 0xf0 },
	{ 0x040c, 0x0a },
	{ 0x040d, 0x50 },
	{ 0x040e, 0x07 },
	{ 0x040f, 0xbc },
	{ 0x0301, 0x09 },
	{ 0x0303, 0x04 },
	{ 0x0305, 0x02 },
	{ 0x0306, 0x00 },
	{ 0x0307, 0xe3 },
	{ 0x030b, 0x02 },
	{ 0x030d, 0x04 },
	{ 0x030e, 0x01 },
	{ 0x030f, 0xc6 },
	{ 0x0310, 0x01 },
	{ 0x080a, 0x00 },
	{ 0x080b, 0x8f },
	{ 0x080c, 0x00 },
	{ 0x080d, 0x57 },
	{ 0x080e, 0x00 },
	{ 0x080f, 0x8f },
	{ 0x0810, 0x00 },
	{ 0x0811, 0x67 },
	{ 0x0812, 0x00 },
	{ 0x0813, 0x67 },
	{ 0x0814, 0x00 },
	{ 0x0815, 0x57 },
	{ 0x0816, 0x01 },
	{ 0x0817, 0x5f },
	{ 0x0818, 0x00 },
	{ 0x0819, 0x47 },
	{ 0xe04c, 0x00 },
	{ 0xe04d, 0x8f },
	{ 0xe04e, 0x00 },
	{ 0xe04f, 0x1f },
	{ 0x0202, 0x0a },
	{ 0x0203, 0x18 },
	{ 0x0224, 0x00 },
	{ 0x0225, 0xa1 },
	{ 0x0204, 0x00 },
	{ 0x0205, 0x00 },
	{ 0x020e, 0x01 },
	{ 0x020f, 0x00 },
	{ 0x0218, 0x01 },
	{ 0x0219, 0x00 },
};

/* 3264x2448 RAW10, LLP 3872, FLL 2605, binning 1x1, link 738 MHz */
static const struct imx320_reg mode_3264x2448_regs[] = {
	{ 0x0112, 0x0a },
	{ 0x0113, 0x0a },
	{ 0x0114, 0x01 },
	{ 0x0220, 0x61 },
	{ 0x0221, 0x11 },
	{ 0x0340, 0x0a },
	{ 0x0341, 0x22 },
	{ 0x0342, 0x0f },
	{ 0x0343, 0x20 },
	{ 0x0381, 0x01 },
	{ 0x0383, 0x01 },
	{ 0x0385, 0x01 },
	{ 0x0387, 0x01 },
	{ 0x0900, 0x00 },
	{ 0x0901, 0x11 },
	{ 0x0902, 0x00 },
	{ 0x3e6a, 0x01 },
	{ 0x3f4c, 0x01 },
	{ 0x3f4d, 0x01 },
	{ 0x3f7c, 0x00 },
	{ 0x3f7d, 0x28 },
	{ 0x5c8e, 0x0e },
	{ 0x5ddd, 0x0e },
	{ 0x5e0e, 0x0e },
	{ 0x3420, 0x02 },
	{ 0x0344, 0x00 },
	{ 0x0345, 0x00 },
	{ 0x0346, 0x00 },
	{ 0x0347, 0x08 },
	{ 0x0348, 0x0c },
	{ 0x0349, 0xcf },
	{ 0x034a, 0x09 },
	{ 0x034b, 0x97 },
	{ 0x034c, 0x0c },
	{ 0x034d, 0xc0 },
	{ 0x034e, 0x09 },
	{ 0x034f, 0x90 },
	{ 0x0408, 0x00 },
	{ 0x0409, 0x08 },
	{ 0x040a, 0x00 },
	{ 0x040b, 0x00 },
	{ 0x040c, 0x0c },
	{ 0x040d, 0xc0 },
	{ 0x040e, 0x09 },
	{ 0x040f, 0x90 },
	{ 0x0301, 0x09 },
	{ 0x0303, 0x04 },
	{ 0x0305, 0x02 },
	{ 0x0306, 0x00 },
	{ 0x0307, 0xe3 },
	{ 0x030b, 0x02 },
	{ 0x030d, 0x04 },
	{ 0x030e, 0x01 },
	{ 0x030f, 0xec },
	{ 0x0310, 0x01 },
	{ 0x080a, 0x00 },
	{ 0x080b, 0x97 },
	{ 0x080c, 0x00 },
	{ 0x080d, 0x5f },
	{ 0x080e, 0x00 },
	{ 0x080f, 0x9f },
	{ 0x0810, 0x00 },
	{ 0x0811, 0x6f },
	{ 0x0812, 0x00 },
	{ 0x0813, 0x6f },
	{ 0x0814, 0x00 },
	{ 0x0815, 0x57 },
	{ 0x0816, 0x01 },
	{ 0x0817, 0x87 },
	{ 0x0818, 0x00 },
	{ 0x0819, 0x4f },
	{ 0xe04c, 0x00 },
	{ 0xe04d, 0x9f },
	{ 0xe04e, 0x00 },
	{ 0xe04f, 0x1f },
	{ 0x0202, 0x0a },
	{ 0x0203, 0x18 },
	{ 0x0224, 0x00 },
	{ 0x0225, 0xa1 },
	{ 0x0204, 0x00 },
	{ 0x0205, 0x00 },
	{ 0x020e, 0x01 },
	{ 0x020f, 0x00 },
	{ 0x0218, 0x01 },
	{ 0x0219, 0x00 },
};

/* 816x1456 RAW10, LLP 3872, FLL 2594, binning 0x0, link 681 MHz */
static const struct imx320_reg mode_816x1456_regs[] = {
	{ 0x0112, 0x0a },
	{ 0x0113, 0x0a },
	{ 0x0114, 0x01 },
	{ 0x0220, 0x00 },
	{ 0x0221, 0x11 },
	{ 0x0340, 0x0a },
	{ 0x0341, 0x22 },
	{ 0x0342, 0x0f },
	{ 0x0343, 0x20 },
	{ 0x0381, 0x01 },
	{ 0x0383, 0x01 },
	{ 0x0385, 0x01 },
	{ 0x0387, 0x01 },
	{ 0x0900, 0x00 },
	{ 0x0901, 0x11 },
	{ 0x0902, 0x00 },
	{ 0x3e6a, 0x01 },
	{ 0x3f4c, 0x01 },
	{ 0x3f4d, 0x01 },
	{ 0x3f7c, 0x00 },
	{ 0x3f7d, 0x00 },
	{ 0x5c8e, 0x0e },
	{ 0x5ddd, 0x0e },
	{ 0x5e0e, 0x0e },
	{ 0x3420, 0x02 },
	{ 0x0344, 0x04 },
	{ 0x0345, 0xd0 },
	{ 0x0346, 0x01 },
	{ 0x0347, 0xcc },
	{ 0x0348, 0x07 },
	{ 0x0349, 0xff },
	{ 0x034a, 0x07 },
	{ 0x034b, 0x7b },
	{ 0x034c, 0x03 },
	{ 0x034d, 0x30 },
	{ 0x034e, 0x05 },
	{ 0x034f, 0xb0 },
	{ 0x0408, 0x00 },
	{ 0x0409, 0x00 },
	{ 0x040a, 0x00 },
	{ 0x040b, 0x00 },
	{ 0x040c, 0x03 },
	{ 0x040d, 0x30 },
	{ 0x040e, 0x05 },
	{ 0x040f, 0xb0 },
	{ 0x0301, 0x09 },
	{ 0x0303, 0x04 },
	{ 0x0305, 0x02 },
	{ 0x0306, 0x00 },
	{ 0x0307, 0xe3 },
	{ 0x030b, 0x02 },
	{ 0x030d, 0x04 },
	{ 0x030e, 0x01 },
	{ 0x030f, 0xc6 },
	{ 0x0310, 0x01 },
	{ 0x080a, 0x00 },
	{ 0x080b, 0x8f },
	{ 0x080c, 0x00 },
	{ 0x080d, 0x57 },
	{ 0x080e, 0x00 },
	{ 0x080f, 0x8f },
	{ 0x0810, 0x00 },
	{ 0x0811, 0x67 },
	{ 0x0812, 0x00 },
	{ 0x0813, 0x67 },
	{ 0x0814, 0x00 },
	{ 0x0815, 0x57 },
	{ 0x0816, 0x01 },
	{ 0x0817, 0x5f },
	{ 0x0818, 0x00 },
	{ 0x0819, 0x47 },
	{ 0xe04c, 0x00 },
	{ 0xe04d, 0x8f },
	{ 0xe04e, 0x00 },
	{ 0xe04f, 0x1f },
	{ 0x0202, 0x0a },
	{ 0x0203, 0x18 },
	{ 0x0224, 0x01 },
	{ 0x0225, 0xf4 },
	{ 0x0204, 0x00 },
	{ 0x0205, 0x00 },
	{ 0x020e, 0x01 },
	{ 0x020f, 0x00 },
	{ 0x0218, 0x01 },
	{ 0x0219, 0x00 },
};

/* 800x600 RAW10, LLP 4080, FLL 652, binning 4x4, link 172.5 MHz */
static const struct imx320_reg mode_800x600_regs[] = {
	{ 0x0112, 0x0a },
	{ 0x0113, 0x0a },
	{ 0x0114, 0x01 },
	{ 0x0220, 0x00 },
	{ 0x0221, 0x11 },
	{ 0x0340, 0x02 },
	{ 0x0341, 0x8c },
	{ 0x0342, 0x0f },
	{ 0x0343, 0xf0 },
	{ 0x0381, 0x01 },
	{ 0x0383, 0x01 },
	{ 0x0385, 0x01 },
	{ 0x0387, 0x01 },
	{ 0x0900, 0x01 },
	{ 0x0901, 0x44 },
	{ 0x0902, 0x00 },
	{ 0x3e6a, 0x01 },
	{ 0x3f4c, 0x01 },
	{ 0x3f4d, 0x03 },
	{ 0x3f7c, 0x00 },
	{ 0x3f7d, 0x01 },
	{ 0x5c8e, 0x19 },
	{ 0x5ddd, 0x19 },
	{ 0x5e0e, 0x19 },
	{ 0x3420, 0x02 },
	{ 0x0344, 0x00 },
	{ 0x0345, 0x00 },
	{ 0x0346, 0x00 },
	{ 0x0347, 0x20 },
	{ 0x0348, 0x0c },
	{ 0x0349, 0xcf },
	{ 0x034a, 0x09 },
	{ 0x034b, 0x7f },
	{ 0x034c, 0x03 },
	{ 0x034d, 0x20 },
	{ 0x034e, 0x02 },
	{ 0x034f, 0x58 },
	{ 0x0408, 0x00 },
	{ 0x0409, 0x0a },
	{ 0x040a, 0x00 },
	{ 0x040b, 0x00 },
	{ 0x040c, 0x03 },
	{ 0x040d, 0x20 },
	{ 0x040e, 0x02 },
	{ 0x040f, 0x58 },
	{ 0x0301, 0x09 },
	{ 0x0303, 0x04 },
	{ 0x0305, 0x04 },
	{ 0x0306, 0x01 },
	{ 0x0307, 0xe0 },
	{ 0x030b, 0x04 },
	{ 0x030d, 0x04 },
	{ 0x030e, 0x00 },
	{ 0x030f, 0xe6 },
	{ 0x0310, 0x00 },
	{ 0x080a, 0x00 },
	{ 0x080b, 0x5f },
	{ 0x080c, 0x00 },
	{ 0x080d, 0x2f },
	{ 0x080e, 0x00 },
	{ 0x080f, 0x47 },
	{ 0x0810, 0x00 },
	{ 0x0811, 0x3f },
	{ 0x0812, 0x00 },
	{ 0x0813, 0x3f },
	{ 0x0814, 0x00 },
	{ 0x0815, 0x2f },
	{ 0x0816, 0x00 },
	{ 0x0817, 0xaf },
	{ 0x0818, 0x00 },
	{ 0x0819, 0x27 },
	{ 0xe04c, 0x00 },
	{ 0xe04d, 0x4f },
	{ 0xe04e, 0x00 },
	{ 0xe04f, 0x1f },
	{ 0x0202, 0x02 },
	{ 0x0203, 0x82 },
	{ 0x0224, 0x01 },
	{ 0x0225, 0xf4 },
	{ 0x0204, 0x00 },
	{ 0x0205, 0x00 },
	{ 0x020e, 0x01 },
	{ 0x020f, 0x00 },
	{ 0x0218, 0x01 },
	{ 0x0219, 0x00 },
};

static const char * const imx320_test_pattern_menu[] = {
	"Disabled",
	"Solid Colour",
	"Eight Vertical Colour Bars",
	"Colour Bars With Fade to Grey",
	"Pseudorandom Sequence (PN9)",
};

static const char * const imx320_supply_name[] = {
	/* Enabled in this order by the vendor power sequence */
	"vana",		/* Analog (2.8V) supply */
	"vio",		/* Interface (1.8V) supply */
	"vcore",	/* Digital core (1.05V) supply */
};

/*
 * The Bayer order has not been confirmed against hardware. If captured
 * frames come out with red and blue swapped, this is the value to change.
 */
#define IMX320_MBUS_CODE MEDIA_BUS_FMT_SRGGB10_1X10

static const struct imx320_mode supported_modes[] = {
	{
		.width = 3264,
		.height = 2448,
		.fll_def = 2605,
		.fll_min = 2605,
		.llp = 3872,
		.link_freq_index = 2,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_3264x2448_regs),
			.regs = mode_3264x2448_regs,
		},
	},
	{
		.width = 2640,
		.height = 1980,
		.fll_def = 2594,
		.fll_min = 2594,
		.llp = 3872,
		.link_freq_index = 1,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_2640x1980_regs),
			.regs = mode_2640x1980_regs,
		},
	},
	{
		.width = 816,
		.height = 1456,
		.fll_def = 2594,
		.fll_min = 2594,
		.llp = 3872,
		.link_freq_index = 1,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_816x1456_regs),
			.regs = mode_816x1456_regs,
		},
	},
	{
		.width = 800,
		.height = 600,
		.fll_def = 652,
		.fll_min = 652,
		.llp = 4080,
		.link_freq_index = 0,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_800x600_regs),
			.regs = mode_800x600_regs,
		},
	},
};

struct imx320 {
	struct v4l2_subdev sd;
	struct media_pad pad;

	struct clk *xclk;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[ARRAY_SIZE(imx320_supply_name)];

	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *exposure;

	const struct imx320_mode *cur_mode;

	/* Serialise access to the controls and the streaming state */
	struct mutex mutex;
};

static inline struct imx320 *to_imx320(struct v4l2_subdev *_sd)
{
	return container_of(_sd, struct imx320, sd);
}

static u64 imx320_pixel_rate(const struct imx320_mode *mode)
{
	return div_u64(imx320_link_freq_menu[mode->link_freq_index] * 2 *
		       IMX320_DATA_LANES, IMX320_RGB_DEPTH);
}

static int imx320_read_reg(struct imx320 *imx320, u16 reg, u32 len, u32 *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx320->sd);
	struct i2c_msg msgs[2];
	u8 addr_buf[2];
	u8 data_buf[4] = { 0 };
	int ret;

	if (len > 4)
		return -EINVAL;

	put_unaligned_be16(reg, addr_buf);
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = sizeof(addr_buf);
	msgs[0].buf = addr_buf;

	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = &data_buf[4 - len];

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return ret < 0 ? ret : -EIO;

	*val = get_unaligned_be32(data_buf);

	return 0;
}

static int imx320_write_reg(struct imx320 *imx320, u16 reg, u32 len, u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx320->sd);
	u8 buf[6];
	int ret;

	if (len > 4)
		return -EINVAL;

	put_unaligned_be16(reg, buf);
	put_unaligned_be32(val << (8 * (4 - len)), buf + 2);
	ret = i2c_master_send(client, buf, len + 2);
	if (ret != len + 2)
		return ret < 0 ? ret : -EIO;

	return 0;
}

static int imx320_write_regs(struct imx320 *imx320,
			     const struct imx320_reg *regs, u32 len)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx320->sd);
	int ret;
	u32 i;

	for (i = 0; i < len; i++) {
		ret = imx320_write_reg(imx320, regs[i].address, 1, regs[i].val);
		if (ret) {
			dev_err_ratelimited(&client->dev,
					    "write reg 0x%4.4x return err %d\n",
					    regs[i].address, ret);
			return ret;
		}
	}

	return 0;
}

static int imx320_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct imx320 *imx320 = to_imx320(sd);
	struct v4l2_mbus_framefmt *fmt;

	mutex_lock(&imx320->mutex);

	fmt = v4l2_subdev_state_get_format(fh->state, 0);
	fmt->width = supported_modes[0].width;
	fmt->height = supported_modes[0].height;
	fmt->code = IMX320_MBUS_CODE;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;

	mutex_unlock(&imx320->mutex);

	return 0;
}

static int imx320_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx320 *imx320 =
		container_of(ctrl->handler, struct imx320, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&imx320->sd);
	s64 max;
	int ret = 0;

	if (ctrl->id == V4L2_CID_VBLANK) {
		max = imx320->cur_mode->height + ctrl->val -
		      IMX320_EXPOSURE_OFFSET;
		ret = __v4l2_ctrl_modify_range(imx320->exposure,
					       imx320->exposure->minimum, max,
					       imx320->exposure->step, max);
		if (ret)
			return ret;
	}

	/*
	 * Applying the control while powered down is pointless; the register
	 * is reprogrammed on the next power up.
	 */
	if (!pm_runtime_get_if_in_use(&client->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_ANALOGUE_GAIN:
		ret = imx320_write_reg(imx320, IMX320_REG_ANALOG_GAIN, 2,
				       ctrl->val);
		break;
	case V4L2_CID_DIGITAL_GAIN:
		ret = imx320_write_reg(imx320, IMX320_REG_DIG_GAIN, 2,
				       ctrl->val);
		break;
	case V4L2_CID_EXPOSURE:
		ret = imx320_write_reg(imx320, IMX320_REG_EXPOSURE, 2,
				       ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		ret = imx320_write_reg(imx320, IMX320_REG_FLL, 2,
				       imx320->cur_mode->height + ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = imx320_write_reg(imx320, IMX320_REG_TEST_PATTERN, 2,
				       ctrl->val);
		break;
	default:
		dev_info(&client->dev, "ctrl(id:0x%x, val:0x%x) is not handled\n",
			 ctrl->id, ctrl->val);
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops imx320_ctrl_ops = {
	.s_ctrl = imx320_set_ctrl,
};

static int imx320_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index > 0)
		return -EINVAL;

	code->code = IMX320_MBUS_CODE;

	return 0;
}

static int imx320_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	if (fse->code != IMX320_MBUS_CODE)
		return -EINVAL;

	fse->min_width = supported_modes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = supported_modes[fse->index].height;
	fse->max_height = fse->min_height;

	return 0;
}

static void imx320_update_pad_format(const struct imx320_mode *mode,
				     struct v4l2_mbus_framefmt *fmt)
{
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->code = IMX320_MBUS_CODE;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
}

static int imx320_get_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct imx320 *imx320 = to_imx320(sd);

	mutex_lock(&imx320->mutex);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		fmt->format = *v4l2_subdev_state_get_format(sd_state, 0);
	else
		imx320_update_pad_format(imx320->cur_mode, &fmt->format);

	mutex_unlock(&imx320->mutex);

	return 0;
}

static int imx320_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct imx320 *imx320 = to_imx320(sd);
	const struct imx320_mode *mode;
	s32 vblank_def, vblank_min;
	s64 h_blank;

	mutex_lock(&imx320->mutex);

	fmt->format.code = IMX320_MBUS_CODE;

	mode = v4l2_find_nearest_size(supported_modes,
				      ARRAY_SIZE(supported_modes),
				      width, height,
				      fmt->format.width, fmt->format.height);
	imx320_update_pad_format(mode, &fmt->format);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		*v4l2_subdev_state_get_format(sd_state, 0) = fmt->format;
	} else {
		imx320->cur_mode = mode;
		__v4l2_ctrl_s_ctrl(imx320->link_freq, mode->link_freq_index);
		__v4l2_ctrl_s_ctrl_int64(imx320->pixel_rate,
					 imx320_pixel_rate(mode));

		/* Update limits and set FPS to default */
		vblank_def = mode->fll_def - mode->height;
		vblank_min = mode->fll_min - mode->height;
		__v4l2_ctrl_modify_range(imx320->vblank, vblank_min,
					 IMX320_FLL_MAX - mode->height, 1,
					 vblank_def);
		__v4l2_ctrl_s_ctrl(imx320->vblank, vblank_def);

		h_blank = mode->llp - mode->width;
		__v4l2_ctrl_modify_range(imx320->hblank, h_blank, h_blank, 1,
					 h_blank);
	}

	mutex_unlock(&imx320->mutex);

	return 0;
}

static int imx320_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *sel)
{
	struct imx320 *imx320 = to_imx320(sd);

	if (sel->which != V4L2_SUBDEV_FORMAT_ACTIVE)
		return -EINVAL;

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_CROP_DEFAULT:
		sel->r.top = 0;
		sel->r.left = 0;
		sel->r.width = imx320->cur_mode->width;
		sel->r.height = imx320->cur_mode->height;
		return 0;
	default:
		return -EINVAL;
	}
}

static int imx320_start_streaming(struct imx320 *imx320)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx320->sd);
	const struct imx320_reg_list *reg_list;
	int ret;

	reg_list = &imx320->cur_mode->reg_list;
	ret = imx320_write_regs(imx320, reg_list->regs, reg_list->num_of_regs);
	if (ret) {
		dev_err(&client->dev, "failed to set mode\n");
		return ret;
	}

	ret = __v4l2_ctrl_handler_setup(imx320->sd.ctrl_handler);
	if (ret)
		return ret;

	return imx320_write_reg(imx320, IMX320_REG_MODE_SELECT, 1,
				IMX320_MODE_STREAMING);
}

static int imx320_stop_streaming(struct imx320 *imx320)
{
	return imx320_write_reg(imx320, IMX320_REG_MODE_SELECT, 1,
				IMX320_MODE_STANDBY);
}

static int imx320_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct imx320 *imx320 = to_imx320(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;

	mutex_lock(&imx320->mutex);

	if (enable) {
		ret = pm_runtime_resume_and_get(&client->dev);
		if (ret < 0)
			goto err_unlock;

		ret = imx320_start_streaming(imx320);
		if (ret)
			goto err_rpm_put;
	} else {
		imx320_stop_streaming(imx320);
		pm_runtime_put(&client->dev);
	}

	mutex_unlock(&imx320->mutex);

	return ret;

err_rpm_put:
	pm_runtime_put(&client->dev);
err_unlock:
	mutex_unlock(&imx320->mutex);

	return ret;
}

static int imx320_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct imx320 *imx320 = to_imx320(sd);
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(imx320_supply_name),
				    imx320->supplies);
	if (ret) {
		dev_err(dev, "failed to enable regulators\n");
		return ret;
	}

	ret = clk_prepare_enable(imx320->xclk);
	if (ret) {
		dev_err(dev, "failed to enable clock\n");
		goto reg_off;
	}

	gpiod_set_value_cansleep(imx320->reset_gpio, 0);
	usleep_range(IMX320_XCLR_MIN_DELAY_US,
		     IMX320_XCLR_MIN_DELAY_US + IMX320_XCLR_DELAY_RANGE_US);

	return 0;

reg_off:
	regulator_bulk_disable(ARRAY_SIZE(imx320_supply_name), imx320->supplies);

	return ret;
}

static int imx320_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct imx320 *imx320 = to_imx320(sd);

	gpiod_set_value_cansleep(imx320->reset_gpio, 1);
	clk_disable_unprepare(imx320->xclk);
	regulator_bulk_disable(ARRAY_SIZE(imx320_supply_name), imx320->supplies);

	return 0;
}

static int imx320_identify_module(struct imx320 *imx320)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx320->sd);
	int ret;
	u32 val;

	ret = imx320_read_reg(imx320, IMX320_REG_CHIP_ID, 2, &val);
	if (ret)
		return dev_err_probe(&client->dev, ret,
				     "failed to read chip id\n");

	if (val != IMX320_CHIP_ID)
		return dev_err_probe(&client->dev, -ENXIO,
				     "chip id mismatch: %x!=%x\n",
				     IMX320_CHIP_ID, val);

	return 0;
}

static const struct v4l2_subdev_video_ops imx320_video_ops = {
	.s_stream = imx320_set_stream,
};

static const struct v4l2_subdev_pad_ops imx320_pad_ops = {
	.enum_mbus_code = imx320_enum_mbus_code,
	.get_fmt = imx320_get_pad_format,
	.set_fmt = imx320_set_pad_format,
	.get_selection = imx320_get_selection,
	.enum_frame_size = imx320_enum_frame_size,
};

static const struct v4l2_subdev_ops imx320_subdev_ops = {
	.video = &imx320_video_ops,
	.pad = &imx320_pad_ops,
};

static const struct media_entity_operations imx320_subdev_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static const struct v4l2_subdev_internal_ops imx320_internal_ops = {
	.open = imx320_open,
};

static int imx320_init_controls(struct imx320 *imx320)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx320->sd);
	struct v4l2_ctrl_handler *ctrl_hdlr = &imx320->ctrl_handler;
	const struct imx320_mode *mode = imx320->cur_mode;
	s64 exposure_max, vblank_def, vblank_min, hblank;
	int ret;

	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 10);
	if (ret)
		return ret;

	ctrl_hdlr->lock = &imx320->mutex;
	imx320->link_freq =
		v4l2_ctrl_new_int_menu(ctrl_hdlr, &imx320_ctrl_ops,
				       V4L2_CID_LINK_FREQ,
				       ARRAY_SIZE(imx320_link_freq_menu) - 1,
				       mode->link_freq_index,
				       imx320_link_freq_menu);
	if (imx320->link_freq)
		imx320->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	imx320->pixel_rate =
		v4l2_ctrl_new_std(ctrl_hdlr, &imx320_ctrl_ops,
				  V4L2_CID_PIXEL_RATE, 1,
				  imx320_pixel_rate(&supported_modes[0]), 1,
				  imx320_pixel_rate(mode));
	if (imx320->pixel_rate)
		imx320->pixel_rate->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank_def = mode->fll_def - mode->height;
	vblank_min = mode->fll_min - mode->height;
	imx320->vblank =
		v4l2_ctrl_new_std(ctrl_hdlr, &imx320_ctrl_ops, V4L2_CID_VBLANK,
				  vblank_min, IMX320_FLL_MAX - mode->height, 1,
				  vblank_def);

	hblank = mode->llp - mode->width;
	imx320->hblank =
		v4l2_ctrl_new_std(ctrl_hdlr, &imx320_ctrl_ops, V4L2_CID_HBLANK,
				  hblank, hblank, 1, hblank);
	if (imx320->hblank)
		imx320->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	exposure_max = mode->fll_def - IMX320_EXPOSURE_OFFSET;
	imx320->exposure =
		v4l2_ctrl_new_std(ctrl_hdlr, &imx320_ctrl_ops,
				  V4L2_CID_EXPOSURE, IMX320_EXPOSURE_MIN,
				  exposure_max, IMX320_EXPOSURE_STEP,
				  IMX320_EXPOSURE_DEFAULT);

	v4l2_ctrl_new_std(ctrl_hdlr, &imx320_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  IMX320_ANA_GAIN_MIN, IMX320_ANA_GAIN_MAX,
			  IMX320_ANA_GAIN_STEP, IMX320_ANA_GAIN_DEFAULT);

	v4l2_ctrl_new_std(ctrl_hdlr, &imx320_ctrl_ops, V4L2_CID_DIGITAL_GAIN,
			  IMX320_DGTL_GAIN_MIN, IMX320_DGTL_GAIN_MAX,
			  IMX320_DGTL_GAIN_STEP, IMX320_DGTL_GAIN_DEFAULT);

	v4l2_ctrl_new_std_menu_items(ctrl_hdlr, &imx320_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(imx320_test_pattern_menu) - 1,
				     0, 0, imx320_test_pattern_menu);

	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(&client->dev, "control init failed: %d\n", ret);
		goto error;
	}

	imx320->sd.ctrl_handler = ctrl_hdlr;

	return 0;

error:
	v4l2_ctrl_handler_free(ctrl_hdlr);

	return ret;
}

static int imx320_check_hwcfg(struct device *dev)
{
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	struct fwnode_handle *ep, *fwnode = dev_fwnode(dev);
	unsigned int i, j;
	int ret;

	ep = fwnode_graph_get_next_endpoint(fwnode, NULL);
	if (!ep)
		return dev_err_probe(dev, -ENXIO, "missing endpoint node\n");

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &bus_cfg);
	fwnode_handle_put(ep);
	if (ret)
		return dev_err_probe(dev, ret, "parsing endpoint failed\n");

	if (bus_cfg.bus.mipi_csi2.num_data_lanes != IMX320_DATA_LANES) {
		ret = dev_err_probe(dev, -EINVAL,
				    "number of CSI2 data lanes %d is not supported\n",
				    bus_cfg.bus.mipi_csi2.num_data_lanes);
		goto out_err;
	}

	if (!bus_cfg.nr_of_link_frequencies) {
		ret = dev_err_probe(dev, -EINVAL,
				    "no link frequencies defined\n");
		goto out_err;
	}

	for (i = 0; i < ARRAY_SIZE(imx320_link_freq_menu); i++) {
		for (j = 0; j < bus_cfg.nr_of_link_frequencies; j++) {
			if (imx320_link_freq_menu[i] ==
			    bus_cfg.link_frequencies[j])
				break;
		}

		if (j == bus_cfg.nr_of_link_frequencies) {
			ret = dev_err_probe(dev, -EINVAL,
					    "no link frequency %lld supported\n",
					    imx320_link_freq_menu[i]);
			goto out_err;
		}
	}

out_err:
	v4l2_fwnode_endpoint_free(&bus_cfg);

	return ret;
}

static int imx320_get_resources(struct imx320 *imx320, struct device *dev)
{
	unsigned int i;
	int ret;

	imx320->xclk = devm_clk_get(dev, NULL);
	if (IS_ERR(imx320->xclk))
		return dev_err_probe(dev, PTR_ERR(imx320->xclk),
				     "failed to get xclk\n");

	imx320->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						     GPIOD_OUT_HIGH);
	if (IS_ERR(imx320->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(imx320->reset_gpio),
				     "failed to get reset gpio\n");

	for (i = 0; i < ARRAY_SIZE(imx320_supply_name); i++)
		imx320->supplies[i].supply = imx320_supply_name[i];

	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(imx320_supply_name),
				      imx320->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get supplies\n");

	return 0;
}

static int imx320_probe(struct i2c_client *client)
{
	struct imx320 *imx320;
	u32 xclk_freq;
	int ret;

	ret = imx320_check_hwcfg(&client->dev);
	if (ret)
		return ret;

	imx320 = devm_kzalloc(&client->dev, sizeof(*imx320), GFP_KERNEL);
	if (!imx320)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&imx320->sd, client, &imx320_subdev_ops);

	ret = imx320_get_resources(imx320, &client->dev);
	if (ret)
		return ret;

	xclk_freq = clk_get_rate(imx320->xclk);
	if (xclk_freq != IMX320_XCLK_FREQ)
		return dev_err_probe(&client->dev, -EINVAL,
				     "xclk frequency %u is not %u\n",
				     xclk_freq, IMX320_XCLK_FREQ);

	ret = imx320_power_on(&client->dev);
	if (ret)
		return ret;

	ret = imx320_identify_module(imx320);
	if (ret)
		goto error_power_off;

	mutex_init(&imx320->mutex);
	imx320->cur_mode = &supported_modes[0];

	ret = imx320_init_controls(imx320);
	if (ret)
		goto error_mutex_destroy;

	imx320->sd.internal_ops = &imx320_internal_ops;
	imx320->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	imx320->sd.entity.ops = &imx320_subdev_entity_ops;
	imx320->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	imx320->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&imx320->sd.entity, 1, &imx320->pad);
	if (ret)
		goto error_handler_free;

	ret = v4l2_async_register_subdev_sensor(&imx320->sd);
	if (ret < 0)
		goto error_media_entity;

	pm_runtime_set_active(&client->dev);
	pm_runtime_enable(&client->dev);
	pm_runtime_idle(&client->dev);

	return 0;

error_media_entity:
	media_entity_cleanup(&imx320->sd.entity);

error_handler_free:
	v4l2_ctrl_handler_free(imx320->sd.ctrl_handler);

error_mutex_destroy:
	mutex_destroy(&imx320->mutex);

error_power_off:
	imx320_power_off(&client->dev);

	return ret;
}

static void imx320_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx320 *imx320 = to_imx320(sd);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(sd->ctrl_handler);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		imx320_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);

	mutex_destroy(&imx320->mutex);
}

static DEFINE_RUNTIME_DEV_PM_OPS(imx320_pm_ops, imx320_power_off,
				 imx320_power_on, NULL);

static const struct of_device_id imx320_of_match[] = {
	{ .compatible = "sony,imx320" },
	{ }
};
MODULE_DEVICE_TABLE(of, imx320_of_match);

static struct i2c_driver imx320_i2c_driver = {
	.driver = {
		.name = "imx320",
		.pm = pm_ptr(&imx320_pm_ops),
		.of_match_table = imx320_of_match,
	},
	.probe = imx320_probe,
	.remove = imx320_remove,
};
module_i2c_driver(imx320_i2c_driver);

MODULE_AUTHOR("William Floyd <git@notmy.space>");
MODULE_DESCRIPTION("Sony IMX320 sensor driver");
MODULE_LICENSE("GPL");
