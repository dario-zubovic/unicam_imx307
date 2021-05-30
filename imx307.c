// SPDX-License-Identifier: GPL-2.0
/*
 * A V4L2 driver for Sony imx307 cameras.
 * Copyright (C) 2021, Dario Zubovic
 *
 * Based on Sony imx219 camera driver
 * Copyright (C) 2019, Raspberry Pi (Trading) Ltd
 *
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mediabus.h>
#include <asm/unaligned.h>

#define imx307_REG_VALUE_08BIT		1
#define imx307_REG_VALUE_16BIT		2

#define imx307_REG_MODE_SELECT		0x0100
#define imx307_MODE_STANDBY		    0x00
#define imx307_MODE_STREAMING		0x01

/* Chip ID */
#define imx307_REG_CHIP_ID		0x0000
#define imx307_CHIP_ID			0x0219

/* External clock frequency is 24.0M */
#define imx307_XCLK_FREQ		24000000

/* Pixel rate is fixed at 182.4M for all the modes */
#define imx307_PIXEL_RATE		182400000

#define imx307_DEFAULT_LINK_FREQ	456000000

/* V_TIMING internal */
#define imx307_REG_VTS			    0x0160
#define imx307_VTS_15FPS		    0x0dc6
#define imx307_VTS_30FPS_1080P		0x06e3
#define imx307_VTS_30FPS_BINNED		0x06e3
#define imx307_VTS_30FPS_640x480	0x06e3
#define imx307_VTS_MAX			    0xffff

#define imx307_VBLANK_MIN		4

/*Frame Length Line*/
#define imx307_FLL_MIN			0x08a6
#define imx307_FLL_MAX			0xffff
#define imx307_FLL_STEP			1
#define imx307_FLL_DEFAULT		0x0c98

/* HBLANK control - read only */
#define imx307_PPL_DEFAULT		3448

/* Exposure control */
#define imx307_REG_EXPOSURE		    0x015a
#define imx307_EXPOSURE_MIN		    4
#define imx307_EXPOSURE_STEP		1
#define imx307_EXPOSURE_DEFAULT		0x640
#define imx307_EXPOSURE_MAX		    65535

/* Analog gain control */
#define imx307_REG_ANALOG_GAIN		0x0157
#define imx307_ANA_GAIN_MIN		    0
#define imx307_ANA_GAIN_MAX		    232
#define imx307_ANA_GAIN_STEP		1
#define imx307_ANA_GAIN_DEFAULT		0x0

/* Digital gain control */
#define imx307_REG_DIGITAL_GAIN		0x0158
#define imx307_DGTL_GAIN_MIN		0x0100
#define imx307_DGTL_GAIN_MAX		0x0fff
#define imx307_DGTL_GAIN_DEFAULT	0x0100
#define imx307_DGTL_GAIN_STEP		1

#define imx307_REG_ORIENTATION		0x0172

/* Test Pattern Control */
#define imx307_REG_TEST_PATTERN		    0x0600
#define imx307_TEST_PATTERN_DISABLE	    0
#define imx307_TEST_PATTERN_SOLID_COLOR	1
#define imx307_TEST_PATTERN_COLOR_BARS	2
#define imx307_TEST_PATTERN_GREY_COLOR	3
#define imx307_TEST_PATTERN_PN9		    4

/* Test pattern colour components */
#define imx307_REG_TESTP_RED		0x0602
#define imx307_REG_TESTP_GREENR		0x0604
#define imx307_REG_TESTP_BLUE		0x0606
#define imx307_REG_TESTP_GREENB		0x0608
#define imx307_TESTP_COLOUR_MIN		0
#define imx307_TESTP_COLOUR_MAX		0x03ff
#define imx307_TESTP_COLOUR_STEP	1
#define imx307_TESTP_RED_DEFAULT	imx307_TESTP_COLOUR_MAX
#define imx307_TESTP_GREENR_DEFAULT	0
#define imx307_TESTP_BLUE_DEFAULT	0
#define imx307_TESTP_GREENB_DEFAULT	0

/* imx307 native and active pixel array size. */
#define imx307_NATIVE_WIDTH		    3296U
#define imx307_NATIVE_HEIGHT		2480U
#define imx307_PIXEL_ARRAY_LEFT		8U
#define imx307_PIXEL_ARRAY_TOP		8U
#define imx307_PIXEL_ARRAY_WIDTH	3280U
#define imx307_PIXEL_ARRAY_HEIGHT	2464U

/* Embedded metadata stream structure */
#define imx307_EMBEDDED_LINE_WIDTH 16384
#define imx307_NUM_EMBEDDED_LINES  1

enum pad_types {
	IMAGE_PAD,
	METADATA_PAD,
	NUM_PADS
};

struct imx307_reg {
	u16 address;
	u8 val;
};

struct imx307_reg_list {
	unsigned int num_of_regs;
	const struct imx307_reg *regs;
};

/* Mode : resolution and related config&values */
struct imx307_mode {
	/* Frame width */
	unsigned int width;
	/* Frame height */
	unsigned int height;

	/* Analog crop rectangle. */
	struct v4l2_rect crop;

	/* V-timing */
	unsigned int vts_def;

	/* Default register values */
	struct imx307_reg_list reg_list;
};

/*
 * Register sets lifted off the i2C interface from the Raspberry Pi firmware
 * driver.
 * 3280x2464 = mode 2, 1920x1080 = mode 1, 1640x1232 = mode 4, 640x480 = mode 7.
 */
static const struct imx307_reg mode_3280x2464_regs[] = {
	{0x0100, 0x00},
	{0x30eb, 0x0c},
	{0x30eb, 0x05},
	{0x300a, 0xff},
	{0x300b, 0xff},
	{0x30eb, 0x05},
	{0x30eb, 0x09},
	{0x0114, 0x01},
	{0x0128, 0x00},
	{0x012a, 0x18},
	{0x012b, 0x00},
	{0x0164, 0x00},
	{0x0165, 0x00},
	{0x0166, 0x0c},
	{0x0167, 0xcf},
	{0x0168, 0x00},
	{0x0169, 0x00},
	{0x016a, 0x09},
	{0x016b, 0x9f},
	{0x016c, 0x0c},
	{0x016d, 0xd0},
	{0x016e, 0x09},
	{0x016f, 0xa0},
	{0x0170, 0x01},
	{0x0171, 0x01},
	{0x0174, 0x00},
	{0x0175, 0x00},
	{0x0301, 0x05},
	{0x0303, 0x01},
	{0x0304, 0x03},
	{0x0305, 0x03},
	{0x0306, 0x00},
	{0x0307, 0x39},
	{0x030b, 0x01},
	{0x030c, 0x00},
	{0x030d, 0x72},
	{0x0624, 0x0c},
	{0x0625, 0xd0},
	{0x0626, 0x09},
	{0x0627, 0xa0},
	{0x455e, 0x00},
	{0x471e, 0x4b},
	{0x4767, 0x0f},
	{0x4750, 0x14},
	{0x4540, 0x00},
	{0x47b4, 0x14},
	{0x4713, 0x30},
	{0x478b, 0x10},
	{0x478f, 0x10},
	{0x4793, 0x10},
	{0x4797, 0x0e},
	{0x479b, 0x0e},
	{0x0162, 0x0d},
	{0x0163, 0x78},
};

static const struct imx307_reg mode_1920_1080_regs[] = {
	{0x0100, 0x00},
	{0x30eb, 0x05},
	{0x30eb, 0x0c},
	{0x300a, 0xff},
	{0x300b, 0xff},
	{0x30eb, 0x05},
	{0x30eb, 0x09},
	{0x0114, 0x01},
	{0x0128, 0x00},
	{0x012a, 0x18},
	{0x012b, 0x00},
	{0x0162, 0x0d},
	{0x0163, 0x78},
	{0x0164, 0x02},
	{0x0165, 0xa8},
	{0x0166, 0x0a},
	{0x0167, 0x27},
	{0x0168, 0x02},
	{0x0169, 0xb4},
	{0x016a, 0x06},
	{0x016b, 0xeb},
	{0x016c, 0x07},
	{0x016d, 0x80},
	{0x016e, 0x04},
	{0x016f, 0x38},
	{0x0170, 0x01},
	{0x0171, 0x01},
	{0x0174, 0x00},
	{0x0175, 0x00},
	{0x0301, 0x05},
	{0x0303, 0x01},
	{0x0304, 0x03},
	{0x0305, 0x03},
	{0x0306, 0x00},
	{0x0307, 0x39},
	{0x030b, 0x01},
	{0x030c, 0x00},
	{0x030d, 0x72},
	{0x0624, 0x07},
	{0x0625, 0x80},
	{0x0626, 0x04},
	{0x0627, 0x38},
	{0x455e, 0x00},
	{0x471e, 0x4b},
	{0x4767, 0x0f},
	{0x4750, 0x14},
	{0x4540, 0x00},
	{0x47b4, 0x14},
	{0x4713, 0x30},
	{0x478b, 0x10},
	{0x478f, 0x10},
	{0x4793, 0x10},
	{0x4797, 0x0e},
	{0x479b, 0x0e},
	{0x0162, 0x0d},
	{0x0163, 0x78},
};

static const struct imx307_reg mode_1640_1232_regs[] = {
	{0x0100, 0x00},
	{0x30eb, 0x0c},
	{0x30eb, 0x05},
	{0x300a, 0xff},
	{0x300b, 0xff},
	{0x30eb, 0x05},
	{0x30eb, 0x09},
	{0x0114, 0x01},
	{0x0128, 0x00},
	{0x012a, 0x18},
	{0x012b, 0x00},
	{0x0164, 0x00},
	{0x0165, 0x00},
	{0x0166, 0x0c},
	{0x0167, 0xcf},
	{0x0168, 0x00},
	{0x0169, 0x00},
	{0x016a, 0x09},
	{0x016b, 0x9f},
	{0x016c, 0x06},
	{0x016d, 0x68},
	{0x016e, 0x04},
	{0x016f, 0xd0},
	{0x0170, 0x01},
	{0x0171, 0x01},
	{0x0174, 0x01},
	{0x0175, 0x01},
	{0x0301, 0x05},
	{0x0303, 0x01},
	{0x0304, 0x03},
	{0x0305, 0x03},
	{0x0306, 0x00},
	{0x0307, 0x39},
	{0x030b, 0x01},
	{0x030c, 0x00},
	{0x030d, 0x72},
	{0x0624, 0x06},
	{0x0625, 0x68},
	{0x0626, 0x04},
	{0x0627, 0xd0},
	{0x455e, 0x00},
	{0x471e, 0x4b},
	{0x4767, 0x0f},
	{0x4750, 0x14},
	{0x4540, 0x00},
	{0x47b4, 0x14},
	{0x4713, 0x30},
	{0x478b, 0x10},
	{0x478f, 0x10},
	{0x4793, 0x10},
	{0x4797, 0x0e},
	{0x479b, 0x0e},
	{0x0162, 0x0d},
	{0x0163, 0x78},
};

static const struct imx307_reg mode_640_480_regs[] = {
	{0x0100, 0x00},
	{0x30eb, 0x05},
	{0x30eb, 0x0c},
	{0x300a, 0xff},
	{0x300b, 0xff},
	{0x30eb, 0x05},
	{0x30eb, 0x09},
	{0x0114, 0x01},
	{0x0128, 0x00},
	{0x012a, 0x18},
	{0x012b, 0x00},
	{0x0162, 0x0d},
	{0x0163, 0x78},
	{0x0164, 0x03},
	{0x0165, 0xe8},
	{0x0166, 0x08},
	{0x0167, 0xe7},
	{0x0168, 0x02},
	{0x0169, 0xf0},
	{0x016a, 0x06},
	{0x016b, 0xaf},
	{0x016c, 0x02},
	{0x016d, 0x80},
	{0x016e, 0x01},
	{0x016f, 0xe0},
	{0x0170, 0x01},
	{0x0171, 0x01},
	{0x0174, 0x03},
	{0x0175, 0x03},
	{0x0301, 0x05},
	{0x0303, 0x01},
	{0x0304, 0x03},
	{0x0305, 0x03},
	{0x0306, 0x00},
	{0x0307, 0x39},
	{0x030b, 0x01},
	{0x030c, 0x00},
	{0x030d, 0x72},
	{0x0624, 0x06},
	{0x0625, 0x68},
	{0x0626, 0x04},
	{0x0627, 0xd0},
	{0x455e, 0x00},
	{0x471e, 0x4b},
	{0x4767, 0x0f},
	{0x4750, 0x14},
	{0x4540, 0x00},
	{0x47b4, 0x14},
	{0x4713, 0x30},
	{0x478b, 0x10},
	{0x478f, 0x10},
	{0x4793, 0x10},
	{0x4797, 0x0e},
	{0x479b, 0x0e},
};

static const struct imx307_reg raw8_framefmt_regs[] = {
	{0x018c, 0x08},
	{0x018d, 0x08},
	{0x0309, 0x08},
};

static const struct imx307_reg raw10_framefmt_regs[] = {
	{0x018c, 0x0a},
	{0x018d, 0x0a},
	{0x0309, 0x0a},
};

static const char * const imx307_test_pattern_menu[] = {
	"Disabled",
	"Color Bars",
	"Solid Color",
	"Grey Color Bars",
	"PN9"
};

static const int imx307_test_pattern_val[] = {
	imx307_TEST_PATTERN_DISABLE,
	imx307_TEST_PATTERN_COLOR_BARS,
	imx307_TEST_PATTERN_SOLID_COLOR,
	imx307_TEST_PATTERN_GREY_COLOR,
	imx307_TEST_PATTERN_PN9,
};

/* regulator supplies */
static const char * const imx307_supply_name[] = {
	/* Supplies can be enabled in any order */
	"VANA",  /* Analog (2.8V) supply */
	"VDIG",  /* Digital Core (1.8V) supply */
	"VDDL",  /* IF (1.2V) supply */
};

#define imx307_NUM_SUPPLIES ARRAY_SIZE(imx307_supply_name)

/*
 * The supported formats.
 * This table MUST contain 4 entries per format, to cover the various flip
 * combinations in the order
 * - no flip
 * - h flip
 * - v flip
 * - h&v flips
 */
static const u32 codes[] = {
	MEDIA_BUS_FMT_SRGGB10_1X10,
	MEDIA_BUS_FMT_SGRBG10_1X10,
	MEDIA_BUS_FMT_SGBRG10_1X10,
	MEDIA_BUS_FMT_SBGGR10_1X10,

	MEDIA_BUS_FMT_SRGGB8_1X8,
	MEDIA_BUS_FMT_SGRBG8_1X8,
	MEDIA_BUS_FMT_SGBRG8_1X8,
	MEDIA_BUS_FMT_SBGGR8_1X8,
};

/*
 * Initialisation delay between XCLR low->high and the moment when the sensor
 * can start capture (i.e. can leave software stanby) must be not less than:
 *   t4 + max(t5, t6 + <time to initialize the sensor register over I2C>)
 * where
 *   t4 is fixed, and is max 200uS,
 *   t5 is fixed, and is 6000uS,
 *   t6 depends on the sensor external clock, and is max 32000 clock periods.
 * As per sensor datasheet, the external clock must be from 6MHz to 27MHz.
 * So for any acceptable external clock t6 is always within the range of
 * 1185 to 5333 uS, and is always less than t5.
 * For this reason this is always safe to wait (t4 + t5) = 6200 uS, then
 * initialize the sensor over I2C, and then exit the software standby.
 *
 * This start-up time can be optimized a bit more, if we start the writes
 * over I2C after (t4+t6), but before (t4+t5) expires. But then sensor
 * initialization over I2C may complete before (t4+t5) expires, and we must
 * ensure that capture is not started before (t4+t5).
 *
 * This delay doesn't account for the power supply startup time. If needed,
 * this should be taken care of via the regulator framework. E.g. in the
 * case of DT for regulator-fixed one should define the startup-delay-us
 * property.
 */
#define imx307_XCLR_MIN_DELAY_US	6200
#define imx307_XCLR_DELAY_RANGE_US	1000

/* Mode configs */
static const struct imx307_mode supported_modes[] = {
	{
		/* 8MPix 15fps mode */
		.width = 3280,
		.height = 2464,
		.crop = {
			.left = imx307_PIXEL_ARRAY_LEFT,
			.top = imx307_PIXEL_ARRAY_TOP,
			.width = 3280,
			.height = 2464
		},
		.vts_def = imx307_VTS_15FPS,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_3280x2464_regs),
			.regs = mode_3280x2464_regs,
		},
	},
	{
		/* 1080P 30fps cropped */
		.width = 1920,
		.height = 1080,
		.crop = {
			.left = 688,
			.top = 700,
			.width = 1920,
			.height = 1080
		},
		.vts_def = imx307_VTS_30FPS_1080P,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_1920_1080_regs),
			.regs = mode_1920_1080_regs,
		},
	},
	{
		/* 2x2 binned 30fps mode */
		.width = 1640,
		.height = 1232,
		.crop = {
			.left = imx307_PIXEL_ARRAY_LEFT,
			.top = imx307_PIXEL_ARRAY_TOP,
			.width = 3280,
			.height = 2464
		},
		.vts_def = imx307_VTS_30FPS_BINNED,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_1640_1232_regs),
			.regs = mode_1640_1232_regs,
		},
	},
	{
		/* 640x480 30fps mode */
		.width = 640,
		.height = 480,
		.crop = {
			.left = 1008,
			.top = 760,
			.width = 1280,
			.height = 960
		},
		.vts_def = imx307_VTS_30FPS_640x480,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_640_480_regs),
			.regs = mode_640_480_regs,
		},
	},
};

struct imx307 {
	struct v4l2_subdev sd;
	struct media_pad pad[NUM_PADS];

	struct v4l2_mbus_framefmt fmt;

	struct clk *xclk; /* system clock to imx307 */
	u32 xclk_freq;

	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[imx307_NUM_SUPPLIES];

	struct v4l2_ctrl_handler ctrl_handler;
	/* V4L2 Controls */
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *vflip;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;

	/* Current mode */
	const struct imx307_mode *mode;

	/*
	 * Mutex for serialized access:
	 * Protect sensor module set pad format and start/stop streaming safely.
	 */
	struct mutex mutex;

	/* Streaming on/off */
	bool streaming;
};

static inline struct imx307 *to_imx307(struct v4l2_subdev *_sd)
{
	return container_of(_sd, struct imx307, sd);
}

/* Read registers up to 2 at a time */
static int imx307_read_reg(struct imx307 *imx307, u16 reg, u32 len, u32 *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx307->sd);
	struct i2c_msg msgs[2];
	u8 addr_buf[2] = { reg >> 8, reg & 0xff };
	u8 data_buf[4] = { 0, };
	int ret;

	if (len > 4)
		return -EINVAL;

	/* Write register address */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = ARRAY_SIZE(addr_buf);
	msgs[0].buf = addr_buf;

	/* Read data from register */
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = &data_buf[4 - len];

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	*val = get_unaligned_be32(data_buf);

	return 0;
}

/* Write registers up to 2 at a time */
static int imx307_write_reg(struct imx307 *imx307, u16 reg, u32 len, u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx307->sd);
	u8 buf[6];

	if (len > 4)
		return -EINVAL;

	put_unaligned_be16(reg, buf);
	put_unaligned_be32(val << (8 * (4 - len)), buf + 2);
	if (i2c_master_send(client, buf, len + 2) != len + 2)
		return -EIO;

	return 0;
}

/* Write a list of registers */
static int imx307_write_regs(struct imx307 *imx307,
			     const struct imx307_reg *regs, u32 len)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx307->sd);
	unsigned int i;
	int ret;

	for (i = 0; i < len; i++) {
		ret = imx307_write_reg(imx307, regs[i].address, 1, regs[i].val);
		if (ret) {
			dev_err_ratelimited(&client->dev,
					    "Failed to write reg 0x%4.4x. error = %d\n",
					    regs[i].address, ret);

			return ret;
		}
	}

	return 0;
}

/* Get bayer order based on flip setting. */
static u32 imx307_get_format_code(struct imx307 *imx307, u32 code)
{
	unsigned int i;

	lockdep_assert_held(&imx307->mutex);

	for (i = 0; i < ARRAY_SIZE(codes); i++)
		if (codes[i] == code)
			break;

	if (i >= ARRAY_SIZE(codes))
		i = 0;

	i = (i & ~3) | (imx307->vflip->val ? 2 : 0) |
	    (imx307->hflip->val ? 1 : 0);

	return codes[i];
}

static void imx307_set_default_format(struct imx307 *imx307)
{
	struct v4l2_mbus_framefmt *fmt;

	fmt = &imx307->fmt;
	fmt->code = MEDIA_BUS_FMT_SRGGB10_1X10;
	fmt->colorspace = V4L2_COLORSPACE_SRGB;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_MAP_QUANTIZATION_DEFAULT(true,
							  fmt->colorspace,
							  fmt->ycbcr_enc);
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
	fmt->width = supported_modes[0].width;
	fmt->height = supported_modes[0].height;
	fmt->field = V4L2_FIELD_NONE;
}

static int imx307_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct imx307 *imx307 = to_imx307(sd);
	struct v4l2_mbus_framefmt *try_fmt_img =
		v4l2_subdev_get_try_format(sd, fh->pad, IMAGE_PAD);
	struct v4l2_mbus_framefmt *try_fmt_meta =
		v4l2_subdev_get_try_format(sd, fh->pad, METADATA_PAD);
	struct v4l2_rect *try_crop;

	mutex_lock(&imx307->mutex);

	/* Initialize try_fmt for the image pad */
	try_fmt_img->width = supported_modes[0].width;
	try_fmt_img->height = supported_modes[0].height;
	try_fmt_img->code = imx307_get_format_code(imx307,
						   MEDIA_BUS_FMT_SRGGB10_1X10);
	try_fmt_img->field = V4L2_FIELD_NONE;

	/* Initialize try_fmt for the embedded metadata pad */
	try_fmt_meta->width = imx307_EMBEDDED_LINE_WIDTH;
	try_fmt_meta->height = imx307_NUM_EMBEDDED_LINES;
	try_fmt_meta->code = MEDIA_BUS_FMT_SENSOR_DATA;
	try_fmt_meta->field = V4L2_FIELD_NONE;

	/* Initialize try_crop rectangle. */
	try_crop = v4l2_subdev_get_try_crop(sd, fh->pad, 0);
	try_crop->top = imx307_PIXEL_ARRAY_TOP;
	try_crop->left = imx307_PIXEL_ARRAY_LEFT;
	try_crop->width = imx307_PIXEL_ARRAY_WIDTH;
	try_crop->height = imx307_PIXEL_ARRAY_HEIGHT;

	mutex_unlock(&imx307->mutex);

	return 0;
}

static int imx307_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx307 *imx307 =
		container_of(ctrl->handler, struct imx307, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&imx307->sd);
	int ret;

	if (ctrl->id == V4L2_CID_VBLANK) {
		int exposure_max, exposure_def;

		/* Update max exposure while meeting expected vblanking */
		exposure_max = imx307->mode->height + ctrl->val - 4;
		exposure_def = (exposure_max < imx307_EXPOSURE_DEFAULT) ?
			exposure_max : imx307_EXPOSURE_DEFAULT;
		__v4l2_ctrl_modify_range(imx307->exposure,
					 imx307->exposure->minimum,
					 exposure_max, imx307->exposure->step,
					 exposure_def);
	}

	/*
	 * Applying V4L2 control value only happens
	 * when power is up for streaming
	 */
	if (pm_runtime_get_if_in_use(&client->dev) == 0)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_ANALOGUE_GAIN:
		ret = imx307_write_reg(imx307, imx307_REG_ANALOG_GAIN,
				       imx307_REG_VALUE_08BIT, ctrl->val);
		break;
	case V4L2_CID_EXPOSURE:
		ret = imx307_write_reg(imx307, imx307_REG_EXPOSURE,
				       imx307_REG_VALUE_16BIT, ctrl->val);
		break;
	case V4L2_CID_DIGITAL_GAIN:
		ret = imx307_write_reg(imx307, imx307_REG_DIGITAL_GAIN,
				       imx307_REG_VALUE_16BIT, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = imx307_write_reg(imx307, imx307_REG_TEST_PATTERN,
				       imx307_REG_VALUE_16BIT,
				       imx307_test_pattern_val[ctrl->val]);
		break;
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
		ret = imx307_write_reg(imx307, imx307_REG_ORIENTATION, 1,
				       imx307->hflip->val |
				       imx307->vflip->val << 1);
		break;
	case V4L2_CID_VBLANK:
		ret = imx307_write_reg(imx307, imx307_REG_VTS,
				       imx307_REG_VALUE_16BIT,
				       imx307->mode->height + ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN_RED:
		ret = imx307_write_reg(imx307, imx307_REG_TESTP_RED,
				       imx307_REG_VALUE_16BIT, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN_GREENR:
		ret = imx307_write_reg(imx307, imx307_REG_TESTP_GREENR,
				       imx307_REG_VALUE_16BIT, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN_BLUE:
		ret = imx307_write_reg(imx307, imx307_REG_TESTP_BLUE,
				       imx307_REG_VALUE_16BIT, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN_GREENB:
		ret = imx307_write_reg(imx307, imx307_REG_TESTP_GREENB,
				       imx307_REG_VALUE_16BIT, ctrl->val);
		break;
	default:
		dev_info(&client->dev,
			 "ctrl(id:0x%x,val:0x%x) is not handled\n",
			 ctrl->id, ctrl->val);
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops imx307_ctrl_ops = {
	.s_ctrl = imx307_set_ctrl,
};

static int imx307_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct imx307 *imx307 = to_imx307(sd);

	if (code->pad >= NUM_PADS)
		return -EINVAL;

	if (code->pad == IMAGE_PAD) {
		if (code->index >= (ARRAY_SIZE(codes) / 4))
			return -EINVAL;

		code->code = imx307_get_format_code(imx307,
						    codes[code->index * 4]);
	} else {
		if (code->index > 0)
			return -EINVAL;

		code->code = MEDIA_BUS_FMT_SENSOR_DATA;
	}

	return 0;
}

static int imx307_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct imx307 *imx307 = to_imx307(sd);

	if (fse->pad >= NUM_PADS)
		return -EINVAL;

	if (fse->pad == IMAGE_PAD) {
		if (fse->index >= ARRAY_SIZE(supported_modes))
			return -EINVAL;

		if (fse->code != imx307_get_format_code(imx307, fse->code))
			return -EINVAL;

		fse->min_width = supported_modes[fse->index].width;
		fse->max_width = fse->min_width;
		fse->min_height = supported_modes[fse->index].height;
		fse->max_height = fse->min_height;
	} else {
		if (fse->code != MEDIA_BUS_FMT_SENSOR_DATA || fse->index > 0)
			return -EINVAL;

		fse->min_width = imx307_EMBEDDED_LINE_WIDTH;
		fse->max_width = fse->min_width;
		fse->min_height = imx307_NUM_EMBEDDED_LINES;
		fse->max_height = fse->min_height;
	}

	return 0;
}

static void imx307_reset_colorspace(struct v4l2_mbus_framefmt *fmt)
{
	fmt->colorspace = V4L2_COLORSPACE_SRGB;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_MAP_QUANTIZATION_DEFAULT(true,
							  fmt->colorspace,
							  fmt->ycbcr_enc);
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
}

static void imx307_update_image_pad_format(struct imx307 *imx307,
					   const struct imx307_mode *mode,
					   struct v4l2_subdev_format *fmt)
{
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	imx307_reset_colorspace(&fmt->format);
}

static void imx307_update_metadata_pad_format(struct v4l2_subdev_format *fmt)
{
	fmt->format.width = imx307_EMBEDDED_LINE_WIDTH;
	fmt->format.height = imx307_NUM_EMBEDDED_LINES;
	fmt->format.code = MEDIA_BUS_FMT_SENSOR_DATA;
	fmt->format.field = V4L2_FIELD_NONE;
}

static int __imx307_get_pad_format(struct imx307 *imx307,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_format *fmt)
{
	if (fmt->pad >= NUM_PADS)
		return -EINVAL;

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		struct v4l2_mbus_framefmt *try_fmt =
			v4l2_subdev_get_try_format(&imx307->sd, cfg, fmt->pad);
		/* update the code which could change due to vflip or hflip: */
		try_fmt->code = fmt->pad == IMAGE_PAD ?
				imx307_get_format_code(imx307, try_fmt->code) :
				MEDIA_BUS_FMT_SENSOR_DATA;
		fmt->format = *try_fmt;
	} else {
		if (fmt->pad == IMAGE_PAD) {
			imx307_update_image_pad_format(imx307, imx307->mode,
						       fmt);
			fmt->format.code = imx307_get_format_code(imx307,
							      imx307->fmt.code);
		} else {
			imx307_update_metadata_pad_format(fmt);
		}
	}

	return 0;
}

static int imx307_get_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_format *fmt)
{
	struct imx307 *imx307 = to_imx307(sd);
	int ret;

	mutex_lock(&imx307->mutex);
	ret = __imx307_get_pad_format(imx307, cfg, fmt);
	mutex_unlock(&imx307->mutex);

	return ret;
}

static int imx307_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_format *fmt)
{
	struct imx307 *imx307 = to_imx307(sd);
	const struct imx307_mode *mode;
	struct v4l2_mbus_framefmt *framefmt;
	int exposure_max, exposure_def, hblank;
	unsigned int i;

	if (fmt->pad >= NUM_PADS)
		return -EINVAL;

	mutex_lock(&imx307->mutex);

	if (fmt->pad == IMAGE_PAD) {
		for (i = 0; i < ARRAY_SIZE(codes); i++)
			if (codes[i] == fmt->format.code)
				break;
		if (i >= ARRAY_SIZE(codes))
			i = 0;

		/* Bayer order varies with flips */
		fmt->format.code = imx307_get_format_code(imx307, codes[i]);

		mode = v4l2_find_nearest_size(supported_modes,
					      ARRAY_SIZE(supported_modes),
					      width, height,
					      fmt->format.width,
					      fmt->format.height);
		imx307_update_image_pad_format(imx307, mode, fmt);
		if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
			framefmt = v4l2_subdev_get_try_format(sd, cfg,
							      fmt->pad);
			*framefmt = fmt->format;
		} else if (imx307->mode != mode ||
			imx307->fmt.code != fmt->format.code) {
			imx307->fmt = fmt->format;
			imx307->mode = mode;
			/* Update limits and set FPS to default */
			__v4l2_ctrl_modify_range(imx307->vblank,
						 imx307_VBLANK_MIN,
						 imx307_VTS_MAX - mode->height,
						 1,
						 mode->vts_def - mode->height);
			__v4l2_ctrl_s_ctrl(imx307->vblank,
					   mode->vts_def - mode->height);
			/*
			 * Update max exposure while meeting
			 * expected vblanking
			 */
			exposure_max = mode->vts_def - 4;
			exposure_def =
				(exposure_max < imx307_EXPOSURE_DEFAULT) ?
					exposure_max : imx307_EXPOSURE_DEFAULT;
			__v4l2_ctrl_modify_range(imx307->exposure,
						 imx307->exposure->minimum,
						 exposure_max,
						 imx307->exposure->step,
						 exposure_def);
			/*
			 * Currently PPL is fixed to imx307_PPL_DEFAULT, so
			 * hblank depends on mode->width only, and is not
			 * changeble in any way other than changing the mode.
			 */
			hblank = imx307_PPL_DEFAULT - mode->width;
			__v4l2_ctrl_modify_range(imx307->hblank, hblank, hblank,
						 1, hblank);
		}
	} else {
		if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
			framefmt = v4l2_subdev_get_try_format(sd, cfg,
							      fmt->pad);
			*framefmt = fmt->format;
		} else {
			/* Only one embedded data mode is supported */
			imx307_update_metadata_pad_format(fmt);
		}
	}

	mutex_unlock(&imx307->mutex);

	return 0;
}

static int imx307_set_framefmt(struct imx307 *imx307)
{
	switch (imx307->fmt.code) {
	case MEDIA_BUS_FMT_SRGGB8_1X8:
	case MEDIA_BUS_FMT_SGRBG8_1X8:
	case MEDIA_BUS_FMT_SGBRG8_1X8:
	case MEDIA_BUS_FMT_SBGGR8_1X8:
		return imx307_write_regs(imx307, raw8_framefmt_regs,
					ARRAY_SIZE(raw8_framefmt_regs));

	case MEDIA_BUS_FMT_SRGGB10_1X10:
	case MEDIA_BUS_FMT_SGRBG10_1X10:
	case MEDIA_BUS_FMT_SGBRG10_1X10:
	case MEDIA_BUS_FMT_SBGGR10_1X10:
		return imx307_write_regs(imx307, raw10_framefmt_regs,
					ARRAY_SIZE(raw10_framefmt_regs));
	}

	return -EINVAL;
}

static const struct v4l2_rect *
__imx307_get_pad_crop(struct imx307 *imx307, struct v4l2_subdev_pad_config *cfg,
		      unsigned int pad, enum v4l2_subdev_format_whence which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_crop(&imx307->sd, cfg, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &imx307->mode->crop;
	}

	return NULL;
}

static int imx307_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP: {
		struct imx307 *imx307 = to_imx307(sd);

		mutex_lock(&imx307->mutex);
		sel->r = *__imx307_get_pad_crop(imx307, cfg, sel->pad,
						sel->which);
		mutex_unlock(&imx307->mutex);

		return 0;
	}

	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.top = 0;
		sel->r.left = 0;
		sel->r.width = imx307_NATIVE_WIDTH;
		sel->r.height = imx307_NATIVE_HEIGHT;

		return 0;

	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.top = imx307_PIXEL_ARRAY_TOP;
		sel->r.left = imx307_PIXEL_ARRAY_LEFT;
		sel->r.width = imx307_PIXEL_ARRAY_WIDTH;
		sel->r.height = imx307_PIXEL_ARRAY_HEIGHT;

		return 0;
	}

	return -EINVAL;
}

static int imx307_start_streaming(struct imx307 *imx307)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx307->sd);
	const struct imx307_reg_list *reg_list;
	int ret;

	ret = pm_runtime_get_sync(&client->dev);
	if (ret < 0) {
		pm_runtime_put_noidle(&client->dev);
		return ret;
	}

	/* Apply default values of current mode */
	reg_list = &imx307->mode->reg_list;
	ret = imx307_write_regs(imx307, reg_list->regs, reg_list->num_of_regs);
	if (ret) {
		dev_err(&client->dev, "%s failed to set mode\n", __func__);
		goto err_rpm_put;
	}

	ret = imx307_set_framefmt(imx307);
	if (ret) {
		dev_err(&client->dev, "%s failed to set frame format: %d\n",
			__func__, ret);
		goto err_rpm_put;
	}

	/* Apply customized values from user */
	ret =  __v4l2_ctrl_handler_setup(imx307->sd.ctrl_handler);
	if (ret)
		goto err_rpm_put;

	/* set stream on register */
	ret = imx307_write_reg(imx307, imx307_REG_MODE_SELECT,
			       imx307_REG_VALUE_08BIT, imx307_MODE_STREAMING);
	if (ret)
		goto err_rpm_put;

	/* vflip and hflip cannot change during streaming */
	__v4l2_ctrl_grab(imx307->vflip, true);
	__v4l2_ctrl_grab(imx307->hflip, true);

	return 0;

err_rpm_put:
	pm_runtime_put(&client->dev);
	return ret;
}

static void imx307_stop_streaming(struct imx307 *imx307)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx307->sd);
	int ret;

	/* set stream off register */
	ret = imx307_write_reg(imx307, imx307_REG_MODE_SELECT,
			       imx307_REG_VALUE_08BIT, imx307_MODE_STANDBY);
	if (ret)
		dev_err(&client->dev, "%s failed to set stream\n", __func__);

	__v4l2_ctrl_grab(imx307->vflip, false);
	__v4l2_ctrl_grab(imx307->hflip, false);

	pm_runtime_put(&client->dev);
}

static int imx307_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct imx307 *imx307 = to_imx307(sd);
	int ret = 0;

	mutex_lock(&imx307->mutex);
	if (imx307->streaming == enable) {
		mutex_unlock(&imx307->mutex);
		return 0;
	}

	if (enable) {
		/*
		 * Apply default & customized values
		 * and then start streaming.
		 */
		ret = imx307_start_streaming(imx307);
		if (ret)
			goto err_unlock;
	} else {
		imx307_stop_streaming(imx307);
	}

	imx307->streaming = enable;

	mutex_unlock(&imx307->mutex);

	return ret;

err_unlock:
	mutex_unlock(&imx307->mutex);

	return ret;
}

/* Power/clock management functions */
static int imx307_power_on(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx307 *imx307 = to_imx307(sd);
	int ret;

	ret = regulator_bulk_enable(imx307_NUM_SUPPLIES,
				    imx307->supplies);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable regulators\n",
			__func__);
		return ret;
	}

	ret = clk_prepare_enable(imx307->xclk);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable clock\n",
			__func__);
		goto reg_off;
	}

	gpiod_set_value_cansleep(imx307->reset_gpio, 1);
	usleep_range(imx307_XCLR_MIN_DELAY_US,
		     imx307_XCLR_MIN_DELAY_US + imx307_XCLR_DELAY_RANGE_US);

	return 0;

reg_off:
	regulator_bulk_disable(imx307_NUM_SUPPLIES, imx307->supplies);

	return ret;
}

static int imx307_power_off(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx307 *imx307 = to_imx307(sd);

	gpiod_set_value_cansleep(imx307->reset_gpio, 0);
	regulator_bulk_disable(imx307_NUM_SUPPLIES, imx307->supplies);
	clk_disable_unprepare(imx307->xclk);

	return 0;
}

static int __maybe_unused imx307_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx307 *imx307 = to_imx307(sd);

	if (imx307->streaming)
		imx307_stop_streaming(imx307);

	return 0;
}

static int __maybe_unused imx307_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx307 *imx307 = to_imx307(sd);
	int ret;

	if (imx307->streaming) {
		ret = imx307_start_streaming(imx307);
		if (ret)
			goto error;
	}

	return 0;

error:
	imx307_stop_streaming(imx307);
	imx307->streaming = false;

	return ret;
}

static int imx307_get_regulators(struct imx307 *imx307)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx307->sd);
	unsigned int i;

	for (i = 0; i < imx307_NUM_SUPPLIES; i++)
		imx307->supplies[i].supply = imx307_supply_name[i];

	return devm_regulator_bulk_get(&client->dev,
				       imx307_NUM_SUPPLIES,
				       imx307->supplies);
}

/* Verify chip ID */
static int imx307_identify_module(struct imx307 *imx307)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx307->sd);
	int ret;
	u32 val;

	ret = imx307_read_reg(imx307, imx307_REG_CHIP_ID,
			      imx307_REG_VALUE_16BIT, &val);
	if (ret) {
		dev_err(&client->dev, "failed to read chip id %x\n",
			imx307_CHIP_ID);
		return ret;
	}

	if (val != imx307_CHIP_ID) {
		dev_err(&client->dev, "chip id mismatch: %x!=%x\n",
			imx307_CHIP_ID, val);
		return -EIO;
	}

	return 0;
}

static const struct v4l2_subdev_core_ops imx307_core_ops = {
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_video_ops imx307_video_ops = {
	.s_stream = imx307_set_stream,
};

static const struct v4l2_subdev_pad_ops imx307_pad_ops = {
	.enum_mbus_code = imx307_enum_mbus_code,
	.get_fmt = imx307_get_pad_format,
	.set_fmt = imx307_set_pad_format,
	.get_selection = imx307_get_selection,
	.enum_frame_size = imx307_enum_frame_size,
};

static const struct v4l2_subdev_ops imx307_subdev_ops = {
	.core = &imx307_core_ops,
	.video = &imx307_video_ops,
	.pad = &imx307_pad_ops,
};

static const struct v4l2_subdev_internal_ops imx307_internal_ops = {
	.open = imx307_open,
};

/* Initialize control handlers */
static int imx307_init_controls(struct imx307 *imx307)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx307->sd);
	struct v4l2_ctrl_handler *ctrl_hdlr;
	unsigned int height = imx307->mode->height;
	struct v4l2_fwnode_device_properties props;
	int exposure_max, exposure_def, hblank;
	int i, ret;

	ctrl_hdlr = &imx307->ctrl_handler;
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 11);
	if (ret)
		return ret;

	mutex_init(&imx307->mutex);
	ctrl_hdlr->lock = &imx307->mutex;

	/* By default, PIXEL_RATE is read only */
	imx307->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &imx307_ctrl_ops,
					       V4L2_CID_PIXEL_RATE,
					       imx307_PIXEL_RATE,
					       imx307_PIXEL_RATE, 1,
					       imx307_PIXEL_RATE);

	/* Initial vblank/hblank/exposure parameters based on current mode */
	imx307->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &imx307_ctrl_ops,
					   V4L2_CID_VBLANK, imx307_VBLANK_MIN,
					   imx307_VTS_MAX - height, 1,
					   imx307->mode->vts_def - height);
	hblank = imx307_PPL_DEFAULT - imx307->mode->width;
	imx307->hblank = v4l2_ctrl_new_std(ctrl_hdlr, &imx307_ctrl_ops,
					   V4L2_CID_HBLANK, hblank, hblank,
					   1, hblank);
	if (imx307->hblank)
		imx307->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	exposure_max = imx307->mode->vts_def - 4;
	exposure_def = (exposure_max < imx307_EXPOSURE_DEFAULT) ?
		exposure_max : imx307_EXPOSURE_DEFAULT;
	imx307->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &imx307_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     imx307_EXPOSURE_MIN, exposure_max,
					     imx307_EXPOSURE_STEP,
					     exposure_def);

	v4l2_ctrl_new_std(ctrl_hdlr, &imx307_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  imx307_ANA_GAIN_MIN, imx307_ANA_GAIN_MAX,
			  imx307_ANA_GAIN_STEP, imx307_ANA_GAIN_DEFAULT);

	v4l2_ctrl_new_std(ctrl_hdlr, &imx307_ctrl_ops, V4L2_CID_DIGITAL_GAIN,
			  imx307_DGTL_GAIN_MIN, imx307_DGTL_GAIN_MAX,
			  imx307_DGTL_GAIN_STEP, imx307_DGTL_GAIN_DEFAULT);

	imx307->hflip = v4l2_ctrl_new_std(ctrl_hdlr, &imx307_ctrl_ops,
					  V4L2_CID_HFLIP, 0, 1, 1, 0);
	if (imx307->hflip)
		imx307->hflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	imx307->vflip = v4l2_ctrl_new_std(ctrl_hdlr, &imx307_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 0);
	if (imx307->vflip)
		imx307->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	v4l2_ctrl_new_std_menu_items(ctrl_hdlr, &imx307_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(imx307_test_pattern_menu) - 1,
				     0, 0, imx307_test_pattern_menu);
	for (i = 0; i < 4; i++) {
		/*
		 * The assumption is that
		 * V4L2_CID_TEST_PATTERN_GREENR == V4L2_CID_TEST_PATTERN_RED + 1
		 * V4L2_CID_TEST_PATTERN_BLUE   == V4L2_CID_TEST_PATTERN_RED + 2
		 * V4L2_CID_TEST_PATTERN_GREENB == V4L2_CID_TEST_PATTERN_RED + 3
		 */
		v4l2_ctrl_new_std(ctrl_hdlr, &imx307_ctrl_ops,
				  V4L2_CID_TEST_PATTERN_RED + i,
				  imx307_TESTP_COLOUR_MIN,
				  imx307_TESTP_COLOUR_MAX,
				  imx307_TESTP_COLOUR_STEP,
				  imx307_TESTP_COLOUR_MAX);
		/* The "Solid color" pattern is white by default */
	}

	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(&client->dev, "%s control init failed (%d)\n",
			__func__, ret);
		goto error;
	}

	ret = v4l2_fwnode_device_parse(&client->dev, &props);
	if (ret)
		goto error;

	ret = v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &imx307_ctrl_ops,
					      &props);
	if (ret)
		goto error;

	imx307->sd.ctrl_handler = ctrl_hdlr;

	return 0;

error:
	v4l2_ctrl_handler_free(ctrl_hdlr);
	mutex_destroy(&imx307->mutex);

	return ret;
}

static void imx307_free_controls(struct imx307 *imx307)
{
	v4l2_ctrl_handler_free(imx307->sd.ctrl_handler);
	mutex_destroy(&imx307->mutex);
}

static int imx307_check_hwcfg(struct device *dev)
{
	struct fwnode_handle *endpoint;
	struct v4l2_fwnode_endpoint ep_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY
	};
	int ret = -EINVAL;

	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!endpoint) {
		dev_err(dev, "endpoint node not found\n");
		return -EINVAL;
	}

	if (v4l2_fwnode_endpoint_alloc_parse(endpoint, &ep_cfg)) {
		dev_err(dev, "could not parse endpoint\n");
		goto error_out;
	}

	/* Check the number of MIPI CSI2 data lanes */
	if (ep_cfg.bus.mipi_csi2.num_data_lanes != 2) {
		dev_err(dev, "only 2 data lanes are currently supported\n");
		goto error_out;
	}

	/* Check the link frequency set in device tree */
	if (!ep_cfg.nr_of_link_frequencies) {
		dev_err(dev, "link-frequency property not found in DT\n");
		goto error_out;
	}

	if (ep_cfg.nr_of_link_frequencies != 1 ||
	    ep_cfg.link_frequencies[0] != imx307_DEFAULT_LINK_FREQ) {
		dev_err(dev, "Link frequency not supported: %lld\n",
			ep_cfg.link_frequencies[0]);
		goto error_out;
	}

	ret = 0;

error_out:
	v4l2_fwnode_endpoint_free(&ep_cfg);
	fwnode_handle_put(endpoint);

	return ret;
}

static int imx307_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct imx307 *imx307;
	int ret;

	imx307 = devm_kzalloc(&client->dev, sizeof(*imx307), GFP_KERNEL);
	if (!imx307)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&imx307->sd, client, &imx307_subdev_ops);

	/* Check the hardware configuration in device tree */
	if (imx307_check_hwcfg(dev))
		return -EINVAL;

	/* Get system clock (xclk) */
	imx307->xclk = devm_clk_get(dev, NULL);
	if (IS_ERR(imx307->xclk)) {
		dev_err(dev, "failed to get xclk\n");
		return PTR_ERR(imx307->xclk);
	}

	imx307->xclk_freq = clk_get_rate(imx307->xclk);
	if (imx307->xclk_freq != imx307_XCLK_FREQ) {
		dev_err(dev, "xclk frequency not supported: %d Hz\n",
			imx307->xclk_freq);
		return -EINVAL;
	}

	ret = imx307_get_regulators(imx307);
	if (ret) {
		dev_err(dev, "failed to get regulators\n");
		return ret;
	}

	/* Request optional enable pin */
	imx307->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						     GPIOD_OUT_HIGH);

	/*
	 * The sensor must be powered for imx307_identify_module()
	 * to be able to read the CHIP_ID register
	 */
	ret = imx307_power_on(dev);
	if (ret)
		return ret;

	ret = imx307_identify_module(imx307);
	if (ret)
		goto error_power_off;

	/* Set default mode to max resolution */
	imx307->mode = &supported_modes[0];

	/* sensor doesn't enter LP-11 state upon power up until and unless
	 * streaming is started, so upon power up switch the modes to:
	 * streaming -> standby
	 */
	ret = imx307_write_reg(imx307, imx307_REG_MODE_SELECT,
			       imx307_REG_VALUE_08BIT, imx307_MODE_STREAMING);
	if (ret < 0)
		goto error_power_off;
	usleep_range(100, 110);

	/* put sensor back to standby mode */
	ret = imx307_write_reg(imx307, imx307_REG_MODE_SELECT,
			       imx307_REG_VALUE_08BIT, imx307_MODE_STANDBY);
	if (ret < 0)
		goto error_power_off;
	usleep_range(100, 110);

	ret = imx307_init_controls(imx307);
	if (ret)
		goto error_power_off;

	/* Initialize subdev */
	imx307->sd.internal_ops = &imx307_internal_ops;
	imx307->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
			    V4L2_SUBDEV_FL_HAS_EVENTS;
	imx307->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	/* Initialize source pads */
	imx307->pad[IMAGE_PAD].flags = MEDIA_PAD_FL_SOURCE;
	imx307->pad[METADATA_PAD].flags = MEDIA_PAD_FL_SOURCE;

	/* Initialize default format */
	imx307_set_default_format(imx307);

	ret = media_entity_pads_init(&imx307->sd.entity, NUM_PADS, imx307->pad);
	if (ret) {
		dev_err(dev, "failed to init entity pads: %d\n", ret);
		goto error_handler_free;
	}

	ret = v4l2_async_register_subdev_sensor_common(&imx307->sd);
	if (ret < 0) {
		dev_err(dev, "failed to register sensor sub-device: %d\n", ret);
		goto error_media_entity;
	}

	/* Enable runtime PM and turn off the device */
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	return 0;

error_media_entity:
	media_entity_cleanup(&imx307->sd.entity);

error_handler_free:
	imx307_free_controls(imx307);

error_power_off:
	imx307_power_off(dev);

	return ret;
}

static int imx307_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx307 *imx307 = to_imx307(sd);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	imx307_free_controls(imx307);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		imx307_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

static const struct of_device_id imx307_dt_ids[] = {
	{ .compatible = "sony,imx307" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx307_dt_ids);

static const struct dev_pm_ops imx307_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(imx307_suspend, imx307_resume)
	SET_RUNTIME_PM_OPS(imx307_power_off, imx307_power_on, NULL)
};

static struct i2c_driver imx307_i2c_driver = {
	.driver = {
		.name = "imx307",
		.of_match_table	= imx307_dt_ids,
		.pm = &imx307_pm_ops,
	},
	.probe_new = imx307_probe,
	.remove = imx307_remove,
};

module_i2c_driver(imx307_i2c_driver);

MODULE_AUTHOR("Dario Zubovic <dario@zubovic.email");
MODULE_DESCRIPTION("Sony IMX307 sensor driver");
MODULE_LICENSE("GPL v2");
