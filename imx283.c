// SPDX-License-Identifier: GPL-2.0
/*
 * A V4L2 driver for Sony imx283 cameras.
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd
 *
 * Based on Sony imx219 camera driver
 * Copyright (C) 2019-2020 Raspberry Pi (Trading) Ltd
 */
#include <asm/unaligned.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mediabus.h>

/* Chip ID */
#define IMX283_REG_CHIP_ID		0x3000
#define IMX283_CHIP_ID			0x0000

#define IMX283_REG_MODE_SELECT		0x3000
#define IMX283_MODE_STANDBY		0x01
#define IMX283_MODE_STREAMING		0x00

#define IMX283_XCLK_FREQ		24000000

#define IMX283_DEFAULT_LINK_FREQ	450000000

/* Pixel rate is fixed at 840MHz for all the modes */
#define IMX283_PIXEL_RATE		840000000

/* V_TIMING internal */
#define IMX283_REG_FRAME_LENGTH		0x0340
#define IMX283_FRAME_LENGTH_MAX		0xffff

/* H_TIMING internal */
#define IMX283_REG_LINE_LENGTH		0x0342
#define IMX283_LINE_LENGTH_MAX		0xffff

/* SHR internal */
#define IMX283_REG_SHR		0x303B
#define IMX283_SHR_MIN		11

/* Exposure control */
#define IMX283_REG_EXPOSURE		0x0202
#define IMX283_EXPOSURE_MIN			52
#define IMX283_EXPOSURE_STEP		1
#define IMX283_EXPOSURE_DEFAULT		1000
#define IMX283_EXPOSURE_MAX		49865

/* Analog gain control */
#define IMX283_REG_ANALOG_GAIN		0x3042
#define IMX283_ANA_GAIN_MIN		0
#define IMX283_ANA_GAIN_MAX		1957
#define IMX283_ANA_GAIN_STEP		1
#define IMX283_ANA_GAIN_DEFAULT		0x0

/* Digital gain control */
#define IMX283_REG_DIGITAL_GAIN		0x3044
#define IMX283_DGTL_GAIN_MIN		0x0000
#define IMX283_DGTL_GAIN_MAX		0x0011
#define IMX283_DGTL_GAIN_DEFAULT	0x0000
#define IMX283_DGTL_GAIN_STEP		1

#define IMX283_REG_VFLIP		0x300B

/* Embedded metadata stream structure */
#define IMX283_EMBEDDED_LINE_WIDTH 16384
#define IMX283_NUM_EMBEDDED_LINES 1

enum pad_types {
	IMAGE_PAD,
	METADATA_PAD,
	NUM_PADS
};

/* imx283 native and active pixel array size. */
#define imx283_NATIVE_WIDTH		5592U
#define imx283_NATIVE_HEIGHT		3694U
#define imx283_PIXEL_ARRAY_LEFT		0U
#define imx283_PIXEL_ARRAY_TOP		0U
#define imx283_PIXEL_ARRAY_WIDTH	5592U
#define imx283_PIXEL_ARRAY_HEIGHT	3694U

struct imx283_reg {
	u16 address;
	u8 val;
};

struct IMX283_reg_list {
	unsigned int num_of_regs;
	const struct imx283_reg *regs;
};

/* Mode : resolution and related config&values */
struct imx283_mode {
	/* Frame width */
	unsigned int width;

	/* Frame height */
	unsigned int height;

	/* H-timing in pixels */
	unsigned int line_length_pix;

	/* Analog crop rectangle. */
	struct v4l2_rect crop;

	/* Highest possible framerate. */
	struct v4l2_fract timeperframe_min;

	/* Default framerate. */
	struct v4l2_fract timeperframe_default;

	/* Default register values */
	struct IMX283_reg_list reg_list;
};

static const struct imx283_reg mode_common_regs[] = {
    {0x3000, 0x0A},

    {0x36C1, 0x02},
    {0x36C2, 0xF0},
    {0x36C3, 0x00},
    {0x36F7, 0x02},
    {0x36F8, 0xC0},

    {0x3003, 0x77},
    {0x36AA, 0x00},

    {0x320B, 0x00},
        //delay 1ms
    {0xFFFE, 0x01},

    
    //{0x3105, 0x00},
    {0x3000, 0x00},
        //delay 19ms
    {0xFFFE, 0x14},

    {0x3001, 0x10},
    {0x3105, 0x00},
    {0x3107, 0xA2},
};

/* 12 mpix 10fps */
static const struct imx283_reg mode_4056x3040_regs[] = {
    {0x3004, 0x04}, //MDSEL1
    {0x3005, 0x03}, //MDSEL2
    {0x3006, 0x10}, //MDSEL3
    {0x3007, 0x00}, //MDSEL4

    {0x3009, 0x00}, //SVR[7:0]    *
    {0x300A, 0x00}, //SVR[15:8]   *
    {0x300B, 0x30}, // 1[5] + HTRIMMING_EN[4]=1 + MDVREV[0]=0

    {0x300F, 0x00}, //VWINPOS[7:0] = 0
    {0x3010, 0x00}, //VWINPOS[3:0] = 0

    {0x3011, 0x00}, //VWIDCUT[7:0] = 0
    {0x3012, 0x00}, //VWIDCUT[2:0] = 0
    {0x3013, 0x00}, //MDSEL7 = 0
    {0x3014, 0x00}, //MDSEL7 = 0
    {0x302F, 0x6E}, //Y_OUT_SIZE = 0E6E    1110 011 X   0 1110
    {0x3030, 0x0E},  

    {0x3031, 0x7E}, //WRITE_VSIZE = 0E7E
    {0x3032, 0x0E},

    {0x3033, 0x10}, //OB_SIZE_V = 10


    {0x3036, 0x84},  //HMAX = 900
    {0x3037, 0x03},

    {0x3038, 0xA0},  //VMAX = 4000
    {0x3039, 0x2F}, 
    {0x303A, 0x00}, 
    


    {0x303B, 0xF0}, //SHR [7:0]
    {0x303C, 0x0F}, //SHR [15:8]

    {0x3058, 0x78}, //HTRIMMING_START = 0x0078 
    {0x3059, 0x00}, 


    {0x305A, 0xF0}, // HTRIMMING_END = 0x15F0
    {0x305B, 0x15},
};

/* Mode configs */
static const struct imx283_mode supported_modes_12bit[] = {
	{
		/* 12MPix 10fps mode */
		.width = 5592,
		.height = 3694,
		.line_length_pix = 0x5dc0,
		.crop = {
			.left = imx283_PIXEL_ARRAY_LEFT,
			.top = imx283_PIXEL_ARRAY_TOP,
			.width = 5592,
			.height = 3694,
		},
		.timeperframe_min = {
			.numerator = 100,
			.denominator = 1000
		},
		.timeperframe_default = {
			.numerator = 100,
			.denominator = 1000
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_4056x3040_regs),
			.regs = mode_4056x3040_regs,
		},
	},
};

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
	/* 12-bit modes. */
	MEDIA_BUS_FMT_SRGGB12_1X12,
	MEDIA_BUS_FMT_SGRBG12_1X12,
	MEDIA_BUS_FMT_SGBRG12_1X12,
	MEDIA_BUS_FMT_SBGGR12_1X12,
};

/* regulator supplies */
static const char * const imx283_supply_name[] = {
	/* Supplies can be enabled in any order */
	"VANA",  /* Analog (2.8V) supply */
	"VDIG",  /* Digital Core (1.05V) supply */
	"VDDL",  /* IF (1.8V) supply */
};

#define imx283_NUM_SUPPLIES ARRAY_SIZE(imx283_supply_name)

/*
 * Initialisation delay between XCLR low->high and the moment when the sensor
 * can start capture (i.e. can leave software standby), given by T7 in the
 * datasheet is 8ms.  This does include I2C setup time as well.
 *
 * Note, that delay between XCLR low->high and reading the CCI ID register (T6
 * in the datasheet) is much smaller - 600us.
 */
#define imx283_XCLR_MIN_DELAY_US	100000
#define imx283_XCLR_DELAY_RANGE_US	1000

struct imx283_compatible_data {
	unsigned int chip_id;
	struct IMX283_reg_list extra_regs;
};

struct imx283 {
	struct v4l2_subdev sd;
	struct media_pad pad[NUM_PADS];

	unsigned int fmt_code;

	struct clk *xclk;
	u32 xclk_freq;

	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[imx283_NUM_SUPPLIES];

	struct v4l2_ctrl_handler ctrl_handler;
	/* V4L2 Controls */
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *vflip;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;

	/* Current mode */
	const struct imx283_mode *mode;

	/*
	 * Mutex for serialized access:
	 * Protect sensor module set pad format and start/stop streaming safely.
	 */
	struct mutex mutex;

	/* Streaming on/off */
	bool streaming;

	/* Rewrite common registers on stream on? */
	bool common_regs_written;

	/* Any extra information related to different compatible sensors */
	const struct imx283_compatible_data *compatible_data;
};

static inline struct imx283 *to_imx283(struct v4l2_subdev *_sd)
{
	return container_of(_sd, struct imx283, sd);
}

static inline void get_mode_table(unsigned int code,
				  const struct imx283_mode **mode_list,
				  unsigned int *num_modes)
{
	switch (code) {
	/* 12-bit */
	case MEDIA_BUS_FMT_SRGGB12_1X12:
	case MEDIA_BUS_FMT_SGRBG12_1X12:
	case MEDIA_BUS_FMT_SGBRG12_1X12:
	case MEDIA_BUS_FMT_SBGGR12_1X12:
		*mode_list = supported_modes_12bit;
		*num_modes = ARRAY_SIZE(supported_modes_12bit);
		break;
	default:
		*mode_list = NULL;
		*num_modes = 0;
	}
}

/* Read registers up to 2 at a time */
static int imx283_read_reg(struct imx283 *imx283, u16 reg, u32 len, u32 *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx283->sd);
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
static int imx283_write_reg_1byte(struct imx283 *imx283, u16 reg, u8 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx283->sd);
	u8 buf[3];

	put_unaligned_be16(reg, buf);
	buf[2]  = val;
	if (i2c_master_send(client, buf, 3) != 3)
		return -EIO;

	return 0;
}

/* Write registers up to 2 at a time */
static int imx283_write_reg_2byte(struct imx283 *imx283, u16 reg, u16 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx283->sd);
	u8 buf[4];

	put_unaligned_be16(reg, buf);
	buf[2]  = val;
	buf[3]  = val>>8;
	if (i2c_master_send(client, buf, 4) != 4)
		return -EIO;

	return 0;
}

/* Write a list of registers */
static int imx283_write_regs(struct imx283 *imx283,
			     const struct imx283_reg *regs, u32 len)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx283->sd);
	unsigned int i;
	int ret;

	for (i = 0; i < len; i++) {
		if (regs[i].address == 0xFFFE) {
			usleep_range(regs[i].val*1000,(regs[i].val+1)*1000);
		}
		else{
			ret = imx283_write_reg_1byte(imx283, regs[i].address, regs[i].val);
			if (ret) {
				dev_err_ratelimited(&client->dev,
						    "Failed to write reg 0x%4.4x. error = %d\n",
						    regs[i].address, ret);

				return ret;
			}
		}
	}

	return 0;
}

/* Get bayer order based on flip setting. */
static u32 imx283_get_format_code(struct imx283 *imx283, u32 code)
{
	unsigned int i;

	lockdep_assert_held(&imx283->mutex);


	return codes[0];
}

static void imx283_set_default_format(struct imx283 *imx283)
{
	/* Set default mode to max resolution */
	imx283->mode = &supported_modes_12bit[0];
	imx283->fmt_code = MEDIA_BUS_FMT_SRGGB12_1X12;
}

static int imx283_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct imx283 *imx283 = to_imx283(sd);
	struct v4l2_mbus_framefmt *try_fmt_img =
		v4l2_subdev_get_try_format(sd, fh->state, IMAGE_PAD);
	struct v4l2_mbus_framefmt *try_fmt_meta =
		v4l2_subdev_get_try_format(sd, fh->state, METADATA_PAD);
	struct v4l2_rect *try_crop;

	mutex_lock(&imx283->mutex);

	/* Initialize try_fmt for the image pad */
	try_fmt_img->width = supported_modes_12bit[0].width;
	try_fmt_img->height = supported_modes_12bit[0].height;
	try_fmt_img->code = imx283_get_format_code(imx283,
						   MEDIA_BUS_FMT_SRGGB12_1X12);
	try_fmt_img->field = V4L2_FIELD_NONE;

	/* Initialize try_fmt for the embedded metadata pad */
	try_fmt_meta->width = IMX283_EMBEDDED_LINE_WIDTH;
	try_fmt_meta->height = IMX283_NUM_EMBEDDED_LINES;
	try_fmt_meta->code = MEDIA_BUS_FMT_SENSOR_DATA;
	try_fmt_meta->field = V4L2_FIELD_NONE;

	/* Initialize try_crop */
	try_crop = v4l2_subdev_get_try_crop(sd, fh->state, IMAGE_PAD);
	try_crop->left = imx283_PIXEL_ARRAY_LEFT;
	try_crop->top = imx283_PIXEL_ARRAY_TOP;
	try_crop->width = imx283_PIXEL_ARRAY_WIDTH;
	try_crop->height = imx283_PIXEL_ARRAY_HEIGHT;

	mutex_unlock(&imx283->mutex);

	return 0;
}


/* TODO */
static int imx283_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx283 *imx283 =
		container_of(ctrl->handler, struct imx283, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&imx283->sd);
	int ret = 0;

	/*
	 * The VBLANK control may change the limits of usable exposure, so check
	 * and adjust if necessary.
	 */
	if (ctrl->id == V4L2_CID_VBLANK){
		int exposure_max, exposure_def, exposure_min;

		/* Honour the VBLANK limits when setting exposure. */
		exposure_max = ((imx283->vblank->val - 11) * imx283->hblank->val + 209 )/72;
		exposure_min = (4 * imx283->hblank->val + 209 )/72;
		exposure_def = min(exposure_max, imx283->exposure->val);
		exposure_def = max(exposure_min, imx283->exposure->val);

		dev_info(&client->dev,"exposure_max:%d, exposure_min:%d\n",exposure_max, exposure_min);
		__v4l2_ctrl_modify_range(imx283->exposure, exposure_min,
					 exposure_max, 1,
					 exposure_def);
	}

	/*
	 * Applying V4L2 control value only happens
	 * when power is up for streaming
	 */
	if (pm_runtime_get_if_in_use(&client->dev) == 0)
		return 0;

	
	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		{
		uint16_t shr = imx283->vblank->val - ((ctrl->val * 72 ) - 209 )/imx283->hblank->val;
		dev_info(&client->dev,"shr:%d <- %d\n",shr, ctrl->val);
		dev_info(&client->dev,"vblank:%d, hblank%d\n",imx283->vblank->val, imx283->hblank->val);
		ret = imx283_write_reg_2byte(imx283, IMX283_REG_SHR, shr);
		}
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		dev_info(&client->dev,"V4L2_CID_ANALOGUE_GAIN : %d\n",ctrl->val);
		ret = imx283_write_reg_2byte(imx283, IMX283_REG_ANALOG_GAIN, ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		ret = imx283_write_reg_2byte(imx283, IMX283_REG_FRAME_LENGTH, ctrl->val);
		break;
	case V4L2_CID_HBLANK:
		ret = imx283_write_reg_2byte(imx283, IMX283_REG_LINE_LENGTH, ctrl->val);
		break;
	case V4L2_CID_DIGITAL_GAIN:
		ret = imx283_write_reg_1byte(imx283, IMX283_REG_DIGITAL_GAIN, ctrl->val);
		break;
	case V4L2_CID_VFLIP:
		ret = imx283_write_reg_1byte(imx283, IMX283_REG_VFLIP, ctrl->val);
		break;
	default:
		dev_info(&client->dev,
			 "ctrl(id:0x%x,val:0x%x) is not handled\n",
			 ctrl->id, ctrl->val);
		//ret = -EINVAL;
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops imx283_ctrl_ops = {
	.s_ctrl = imx283_set_ctrl,
};

static int imx283_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct imx283 *imx283 = to_imx283(sd);

	if (code->pad >= NUM_PADS)
		return -EINVAL;

	if (code->pad == IMAGE_PAD) {
		if (code->index >= (ARRAY_SIZE(codes) / 4))
			return -EINVAL;

		code->code = imx283_get_format_code(imx283,
						    codes[code->index * 4]);
	} else {
		if (code->index > 0)
			return -EINVAL;

		code->code = MEDIA_BUS_FMT_SENSOR_DATA;
	}

	return 0;
}

static int imx283_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct imx283 *imx283 = to_imx283(sd);

	if (fse->pad >= NUM_PADS)
		return -EINVAL;

	if (fse->pad == IMAGE_PAD) {
		const struct imx283_mode *mode_list;
		unsigned int num_modes;

		get_mode_table(fse->code, &mode_list, &num_modes);

		if (fse->index >= num_modes)
			return -EINVAL;

		if (fse->code != imx283_get_format_code(imx283, fse->code))
			return -EINVAL;

		fse->min_width = mode_list[fse->index].width;
		fse->max_width = fse->min_width;
		fse->min_height = mode_list[fse->index].height;
		fse->max_height = fse->min_height;
	} else {
		if (fse->code != MEDIA_BUS_FMT_SENSOR_DATA || fse->index > 0)
			return -EINVAL;

		fse->min_width = IMX283_EMBEDDED_LINE_WIDTH;
		fse->max_width = fse->min_width;
		fse->min_height = IMX283_NUM_EMBEDDED_LINES;
		fse->max_height = fse->min_height;
	}

	return 0;
}

static void imx283_reset_colorspace(struct v4l2_mbus_framefmt *fmt)
{
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_MAP_QUANTIZATION_DEFAULT(true,
							  fmt->colorspace,
							  fmt->ycbcr_enc);
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
}

static void imx283_update_image_pad_format(struct imx283 *imx283,
					   const struct imx283_mode *mode,
					   struct v4l2_subdev_format *fmt)
{
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	imx283_reset_colorspace(&fmt->format);
}

static void imx283_update_metadata_pad_format(struct v4l2_subdev_format *fmt)
{
	fmt->format.width = IMX283_EMBEDDED_LINE_WIDTH;
	fmt->format.height = IMX283_NUM_EMBEDDED_LINES;
	fmt->format.code = MEDIA_BUS_FMT_SENSOR_DATA;
	fmt->format.field = V4L2_FIELD_NONE;
}

static int imx283_get_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct imx283 *imx283 = to_imx283(sd);

	if (fmt->pad >= NUM_PADS)
		return -EINVAL;

	mutex_lock(&imx283->mutex);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		struct v4l2_mbus_framefmt *try_fmt =
			v4l2_subdev_get_try_format(&imx283->sd, sd_state,
						   fmt->pad);
		/* update the code which could change due to vflip or hflip: */
		try_fmt->code = fmt->pad == IMAGE_PAD ?
				imx283_get_format_code(imx283, try_fmt->code) :
				MEDIA_BUS_FMT_SENSOR_DATA;
		fmt->format = *try_fmt;
	} else {
		if (fmt->pad == IMAGE_PAD) {
			imx283_update_image_pad_format(imx283, imx283->mode,
						       fmt);
			fmt->format.code =
			       imx283_get_format_code(imx283, imx283->fmt_code);
		} else {
			imx283_update_metadata_pad_format(fmt);
		}
	}

	mutex_unlock(&imx283->mutex);
	return 0;
}
/* TODO */
static
unsigned int imx283_get_frame_length(const struct imx283_mode *mode,
				     const struct v4l2_fract *timeperframe)
{
	u64 frame_length;

	frame_length = (u64)timeperframe->numerator * IMX283_PIXEL_RATE;
	do_div(frame_length,
	       (u64)timeperframe->denominator * mode->line_length_pix);

	if (WARN_ON(frame_length > IMX283_FRAME_LENGTH_MAX))
		frame_length = IMX283_FRAME_LENGTH_MAX;

	return max_t(unsigned int, frame_length, mode->height);
}
/* TODO */
static void imx283_set_framing_limits(struct imx283 *imx283)
{
	unsigned int frm_length_min, frm_length_default, hblank_min;
	const struct imx283_mode *mode = imx283->mode;

	frm_length_min = imx283_get_frame_length(mode, &mode->timeperframe_min);
	frm_length_default =
		     imx283_get_frame_length(mode, &mode->timeperframe_default);

	/* Update limits and set FPS to default */
	__v4l2_ctrl_modify_range(imx283->vblank, 3793,
				 (IMX283_FRAME_LENGTH_MAX),
				 1, 9000);

	/* Setting this will adjust the exposure limits as well. */
	__v4l2_ctrl_s_ctrl(imx283->vblank, frm_length_default - mode->height);

	__v4l2_ctrl_modify_range(imx283->hblank, 887,
				 IMX283_LINE_LENGTH_MAX, 1, 900);
	__v4l2_ctrl_s_ctrl(imx283->hblank, 900);

}
/* TODO */
static int imx283_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt;
	const struct imx283_mode *mode;
	struct imx283 *imx283 = to_imx283(sd);

	if (fmt->pad >= NUM_PADS)
		return -EINVAL;

	mutex_lock(&imx283->mutex);

	if (fmt->pad == IMAGE_PAD) {
		const struct imx283_mode *mode_list;
		unsigned int num_modes;

		/* Bayer order varies with flips */
		fmt->format.code = imx283_get_format_code(imx283,
							  fmt->format.code);

		get_mode_table(fmt->format.code, &mode_list, &num_modes);

		mode = v4l2_find_nearest_size(mode_list,
					      num_modes,
					      width, height,
					      fmt->format.width,
					      fmt->format.height);
		imx283_update_image_pad_format(imx283, mode, fmt);
		if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
			framefmt = v4l2_subdev_get_try_format(sd, sd_state,
							      fmt->pad);
			*framefmt = fmt->format;
		} else if (imx283->mode != mode) {
			imx283->mode = mode;
			imx283->fmt_code = fmt->format.code;
			imx283_set_framing_limits(imx283);
		}
	} else {
		if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
			framefmt = v4l2_subdev_get_try_format(sd, sd_state,
							      fmt->pad);
			*framefmt = fmt->format;
		} else {
			/* Only one embedded data mode is supported */
			imx283_update_metadata_pad_format(fmt);
		}
	}

	mutex_unlock(&imx283->mutex);

	return 0;
}
/* TODO */
static const struct v4l2_rect *
__imx283_get_pad_crop(struct imx283 *imx283,
		      struct v4l2_subdev_state *sd_state,
		      unsigned int pad, enum v4l2_subdev_format_whence which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_crop(&imx283->sd, sd_state, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &imx283->mode->crop;
	}

	return NULL;
}

/* Start streaming */
static int imx283_start_streaming(struct imx283 *imx283)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx283->sd);
	const struct IMX283_reg_list *reg_list;
	int ret;

	if (!imx283->common_regs_written) {
		ret = imx283_write_regs(imx283, mode_common_regs,
					ARRAY_SIZE(mode_common_regs));
		if (ret) {
			dev_err(&client->dev, "%s failed to set common settings\n",
				__func__);
			return ret;
		}
		imx283->common_regs_written = true;
	}

	/* Apply default values of current mode */
	reg_list = &imx283->mode->reg_list;
	ret = imx283_write_regs(imx283, reg_list->regs, reg_list->num_of_regs);
	if (ret) {
		dev_err(&client->dev, "%s failed to set mode\n", __func__);
		return ret;
	}

	/* Apply customized values from user */
	ret =  __v4l2_ctrl_handler_setup(imx283->sd.ctrl_handler);

	return ret;
}

/* Stop streaming */
static void imx283_stop_streaming(struct imx283 *imx283)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx283->sd);
	int ret;

	/* set stream off register */
	ret = imx283_write_reg_1byte(imx283, IMX283_REG_MODE_SELECT, IMX283_MODE_STANDBY);
	if (ret)
		dev_err(&client->dev, "%s failed to set stream\n", __func__);
}

static int imx283_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct imx283 *imx283 = to_imx283(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;

	mutex_lock(&imx283->mutex);
	if (imx283->streaming == enable) {
		mutex_unlock(&imx283->mutex);
		return 0;
	}

	if (enable) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto err_unlock;
		}

		/*
		 * Apply default & customized values
		 * and then start streaming.
		 */
		ret = imx283_start_streaming(imx283);
		if (ret)
			goto err_rpm_put;
	} else {
		imx283_stop_streaming(imx283);
		pm_runtime_put(&client->dev);
	}

	imx283->streaming = enable;

	/* vflip and hflip cannot change during streaming */
	__v4l2_ctrl_grab(imx283->vflip, enable);
	__v4l2_ctrl_grab(imx283->hflip, enable);

	mutex_unlock(&imx283->mutex);

	return ret;

err_rpm_put:
	pm_runtime_put(&client->dev);
err_unlock:
	mutex_unlock(&imx283->mutex);

	return ret;
}

/* Power/clock management functions */
static int imx283_power_on(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx283 *imx283 = to_imx283(sd);
	int ret;

	ret = regulator_bulk_enable(imx283_NUM_SUPPLIES,
				    imx283->supplies);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable regulators\n",
			__func__);
		return ret;
	}

	ret = clk_prepare_enable(imx283->xclk);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable clock\n",
			__func__);
		goto reg_off;
	}

	gpiod_set_value_cansleep(imx283->reset_gpio, 1);
	usleep_range(imx283_XCLR_MIN_DELAY_US,
		     imx283_XCLR_MIN_DELAY_US + imx283_XCLR_DELAY_RANGE_US);

	return 0;

reg_off:
	regulator_bulk_disable(imx283_NUM_SUPPLIES, imx283->supplies);
	return ret;
}

static int imx283_power_off(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx283 *imx283 = to_imx283(sd);

	gpiod_set_value_cansleep(imx283->reset_gpio, 0);
	regulator_bulk_disable(imx283_NUM_SUPPLIES, imx283->supplies);
	clk_disable_unprepare(imx283->xclk);

	/* Force reprogramming of the common registers when powered up again. */
	imx283->common_regs_written = false;

	return 0;
}

static int __maybe_unused imx283_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx283 *imx283 = to_imx283(sd);

	if (imx283->streaming)
		imx283_stop_streaming(imx283);

	return 0;
}

static int __maybe_unused imx283_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx283 *imx283 = to_imx283(sd);
	int ret;

	if (imx283->streaming) {
		ret = imx283_start_streaming(imx283);
		if (ret)
			goto error;
	}

	return 0;

error:
	imx283_stop_streaming(imx283);
	imx283->streaming = 0;
	return ret;
}

static int imx283_get_regulators(struct imx283 *imx283)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx283->sd);
	unsigned int i;

	for (i = 0; i < imx283_NUM_SUPPLIES; i++)
		imx283->supplies[i].supply = imx283_supply_name[i];

	return devm_regulator_bulk_get(&client->dev,
				       imx283_NUM_SUPPLIES,
				       imx283->supplies);
}

/* Verify chip ID */
static int imx283_identify_module(struct imx283 *imx283, u32 expected_id)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx283->sd);
	int ret;
	u32 val;

	ret = imx283_read_reg(imx283, IMX283_REG_CHIP_ID,
			      1, &val);
	if (ret) {
		dev_err(&client->dev, "failed to read chip id %x, with error %d\n",
			expected_id, ret);
		return ret;
	}

	dev_info(&client->dev, "Device found\n");

	return 0;
}

static int imx283_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP: {
		struct imx283 *imx283 = to_imx283(sd);

		mutex_lock(&imx283->mutex);
		sel->r = *__imx283_get_pad_crop(imx283, sd_state, sel->pad,
						sel->which);
		mutex_unlock(&imx283->mutex);

		return 0;
	}

	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = imx283_NATIVE_WIDTH;
		sel->r.height = imx283_NATIVE_HEIGHT;

		return 0;

	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.left = imx283_PIXEL_ARRAY_LEFT;
		sel->r.top = imx283_PIXEL_ARRAY_TOP;
		sel->r.width = imx283_PIXEL_ARRAY_WIDTH;
		sel->r.height = imx283_PIXEL_ARRAY_HEIGHT;

		return 0;
	}

	return -EINVAL;
}


static const struct v4l2_subdev_core_ops imx283_core_ops = {
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_video_ops imx283_video_ops = {
	.s_stream = imx283_set_stream,
};

static const struct v4l2_subdev_pad_ops imx283_pad_ops = {
	.enum_mbus_code = imx283_enum_mbus_code,
	.get_fmt = imx283_get_pad_format,
	.set_fmt = imx283_set_pad_format,
	.get_selection = imx283_get_selection,
	.enum_frame_size = imx283_enum_frame_size,
};

static const struct v4l2_subdev_ops imx283_subdev_ops = {
	.core = &imx283_core_ops,
	.video = &imx283_video_ops,
	.pad = &imx283_pad_ops,
};

static const struct v4l2_subdev_internal_ops imx283_internal_ops = {
	.open = imx283_open,
};




/* Initialize control handlers */
static int imx283_init_controls(struct imx283 *imx283)
{
	struct v4l2_ctrl_handler *ctrl_hdlr;
	struct i2c_client *client = v4l2_get_subdevdata(&imx283->sd);
	struct v4l2_fwnode_device_properties props;
	unsigned int i;
	int ret;

	ctrl_hdlr = &imx283->ctrl_handler;
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 16);
	if (ret)
		return ret;

	mutex_init(&imx283->mutex);
	ctrl_hdlr->lock = &imx283->mutex;

	/* By default, PIXEL_RATE is read only */
	imx283->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &imx283_ctrl_ops,
					       V4L2_CID_PIXEL_RATE,
					       IMX283_PIXEL_RATE,
					       IMX283_PIXEL_RATE, 1,
					       IMX283_PIXEL_RATE);

	/*
	 * Create the controls here, but mode specific limits are setup
	 * in the imx283_set_framing_limits() call below.
	 */
	imx283->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &imx283_ctrl_ops,
					   V4L2_CID_VBLANK, 0, 0xffff, 1, 0);
	imx283->hblank = v4l2_ctrl_new_std(ctrl_hdlr, &imx283_ctrl_ops,
					   V4L2_CID_HBLANK, 0, 0xffff, 1, 0);

	imx283->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &imx283_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     IMX283_EXPOSURE_MIN,
					     IMX283_EXPOSURE_MAX,
					     IMX283_EXPOSURE_STEP,
					     IMX283_EXPOSURE_DEFAULT);

	v4l2_ctrl_new_std(ctrl_hdlr, &imx283_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  IMX283_ANA_GAIN_MIN, IMX283_ANA_GAIN_MAX,
			  IMX283_ANA_GAIN_STEP, IMX283_ANA_GAIN_DEFAULT);

	v4l2_ctrl_new_std(ctrl_hdlr, &imx283_ctrl_ops, V4L2_CID_DIGITAL_GAIN,
			  IMX283_DGTL_GAIN_MIN, IMX283_DGTL_GAIN_MAX,
			  IMX283_DGTL_GAIN_STEP, IMX283_DGTL_GAIN_DEFAULT);

	imx283->vflip = v4l2_ctrl_new_std(ctrl_hdlr, &imx283_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 0);
	if (imx283->vflip)
		imx283->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(&client->dev, "%s control init failed (%d)\n",
			__func__, ret);
		goto error;
	}

	ret = v4l2_fwnode_device_parse(&client->dev, &props);
	if (ret)
		goto error;

	ret = v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &imx283_ctrl_ops,
					      &props);
	if (ret)
		goto error;

	imx283->sd.ctrl_handler = ctrl_hdlr;

	/* Setup exposure and frame/line length limits. */
	imx283_set_framing_limits(imx283);

	return 0;

error:
	v4l2_ctrl_handler_free(ctrl_hdlr);
	mutex_destroy(&imx283->mutex);

	return ret;
}

static void imx283_free_controls(struct imx283 *imx283)
{
	v4l2_ctrl_handler_free(imx283->sd.ctrl_handler);
	mutex_destroy(&imx283->mutex);
}


static const struct imx283_compatible_data imx283_compatible = {
	.chip_id = IMX283_CHIP_ID,
	.extra_regs = {
		.num_of_regs = 0,
		.regs = NULL
	}
};

static const struct of_device_id imx283_dt_ids[] = {
	{ .compatible = "sony,imx283", .data = &imx283_compatible },
	{ /* sentinel */ }
};

static int imx283_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct imx283 *imx283;
	const struct of_device_id *match;
	int ret;

	imx283 = devm_kzalloc(&client->dev, sizeof(*imx283), GFP_KERNEL);
	if (!imx283)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&imx283->sd, client, &imx283_subdev_ops);

	match = of_match_device(imx283_dt_ids, dev);
	if (!match)
		return -ENODEV;
	imx283->compatible_data =
		(const struct imx283_compatible_data *)match->data;

	/* Get system clock (xclk) */
	imx283->xclk = devm_clk_get(dev, NULL);
	if (IS_ERR(imx283->xclk)) {
		dev_err(dev, "failed to get xclk\n");
		return PTR_ERR(imx283->xclk);
	}

	imx283->xclk_freq = clk_get_rate(imx283->xclk);
	if (imx283->xclk_freq != IMX283_XCLK_FREQ) {
		dev_err(dev, "xclk frequency not supported: %d Hz\n",
			imx283->xclk_freq);
		return -EINVAL;
	}

	ret = imx283_get_regulators(imx283);
	if (ret) {
		dev_err(dev, "failed to get regulators\n");
		return ret;
	}

	/* Request optional enable pin */
	imx283->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						     GPIOD_OUT_HIGH);
	
	/*
	 * The sensor must be powered for imx283_identify_module()
	 * to be able to read the CHIP_ID register
	 */
	ret = imx283_power_on(dev);
	if (ret)
		return ret;

	ret = imx283_identify_module(imx283, imx283->compatible_data->chip_id);
	if (ret)
		goto error_power_off;

	/* Initialize default format */
	imx283_set_default_format(imx283);

	/* Enable runtime PM and turn off the device */
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	/* This needs the pm runtime to be registered. */
	ret = imx283_init_controls(imx283);
	if (ret)
		goto error_power_off;

	/* Initialize subdev */
	imx283->sd.internal_ops = &imx283_internal_ops;
	imx283->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
			    V4L2_SUBDEV_FL_HAS_EVENTS;
	imx283->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	/* Initialize source pads */
	imx283->pad[IMAGE_PAD].flags = MEDIA_PAD_FL_SOURCE;
	imx283->pad[METADATA_PAD].flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&imx283->sd.entity, NUM_PADS, imx283->pad);
	if (ret) {
		dev_err(dev, "failed to init entity pads: %d\n", ret);
		goto error_handler_free;
	}

	ret = v4l2_async_register_subdev_sensor(&imx283->sd);
	if (ret < 0) {
		dev_err(dev, "failed to register sensor sub-device: %d\n", ret);
		goto error_media_entity;
	}

	return 0;

error_media_entity:
	media_entity_cleanup(&imx283->sd.entity);

error_handler_free:
	imx283_free_controls(imx283);

error_power_off:
	pm_runtime_disable(&client->dev);
	pm_runtime_set_suspended(&client->dev);
	imx283_power_off(&client->dev);

	return ret;
}

static int imx283_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx283 *imx283 = to_imx283(sd);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	imx283_free_controls(imx283);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		imx283_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

MODULE_DEVICE_TABLE(of, imx283_dt_ids);

static const struct dev_pm_ops imx283_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(imx283_suspend, imx283_resume)
	SET_RUNTIME_PM_OPS(imx283_power_off, imx283_power_on, NULL)
};

static struct i2c_driver imx283_i2c_driver = {
	.driver = {
		.name = "imx283",
		.of_match_table	= imx283_dt_ids,
		.pm = &imx283_pm_ops,
	},
	.probe_new = imx283_probe,
	.remove = imx283_remove,
};

module_i2c_driver(imx283_i2c_driver);

MODULE_AUTHOR("Naushir Patuck <naush@raspberrypi.com>");
MODULE_DESCRIPTION("Sony imx283 sensor driver");
MODULE_LICENSE("GPL v2");
