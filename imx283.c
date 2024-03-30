// SPDX-License-Identifier: GPL-2.0
/*
 * V4L2 Support for the IMX283
 *
 * The IMX283 has BigEndian register addresses
 * and uses little-endian value.
 *
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


struct cci_reg_sequence {
	u32 reg;
	u64 val;
};

#define CCI_REG_ADDR_MASK		GENMASK(15, 0)
#define CCI_REG_WIDTH_SHIFT		16
#define CCI_REG_WIDTH_MASK		GENMASK(19, 16)
#define CCI_REG_LE                     BIT(20)


#define CCI_REG8(x)			((1 << CCI_REG_WIDTH_SHIFT) | (x))
#define CCI_REG16(x)			((2 << CCI_REG_WIDTH_SHIFT) | (x))
#define CCI_REG24(x)			((3 << CCI_REG_WIDTH_SHIFT) | (x))
#define CCI_REG32(x)			((4 << CCI_REG_WIDTH_SHIFT) | (x))
#define CCI_REG64(x)			((8 << CCI_REG_WIDTH_SHIFT) | (x))
#define CCI_REG16_LE(x)         (CCI_REG_LE | (2U << CCI_REG_WIDTH_SHIFT) | (x))
#define CCI_REG24_LE(x)         (CCI_REG_LE | (3U << CCI_REG_WIDTH_SHIFT) | (x))
#define CCI_REG32_LE(x)         (CCI_REG_LE | (4U << CCI_REG_WIDTH_SHIFT) | (x))
#define CCI_REG64_LE(x)         (CCI_REG_LE | (8U << CCI_REG_WIDTH_SHIFT) | (x))




/*
 * TODOs
 *  - Move to active state api
 *  - Add 720 MBps speed mode to link_freq
 *    - HMAX/VMAX must be calculated based on link-freq to support this.
 *  - Support arbitrary cropping
 * 
 *  - account for the VOB
 *  - Identify where the HOB is coming from.
 * 
 *  - Remove 'events' that are not used.
 *  - Fix/remove HFLIP/VFLIP which aren't well supported at all.
 *  - Fix exposure and blanking calculations
 */

/* Chip ID */
#define IMX283_REG_CHIP_ID		CCI_REG8(0x3000)
#define IMX283_CHIP_ID			0x0b	// Default power on state

#define IMX283_REG_STANDBY		CCI_REG8(0x3000)
#define   IMX283_ACTIVE			0
#define   IMX283_STANDBY		BIT(0)
#define   IMX283_STBLOGIC		BIT(1)
#define   IMX283_STBMIPI		BIT(2)
#define   IMX283_STBDV			BIT(3)
#define   IMX283_SLEEP			BIT(4)

#define IMX283_REG_CLAMP		CCI_REG8(0x3001)
#define   IMX283_CLPSQRST		BIT(4)

#define IMX283_REG_PLSTMG08		CCI_REG8(0x3003)
#define   IMX283_PLSTMG08_VAL		0x77

#define IMX283_REG_MDSEL1		CCI_REG8(0x3004)
#define IMX283_REG_MDSEL2		CCI_REG8(0x3005)
#define IMX283_REG_MDSEL3		CCI_REG8(0x3006)
#define IMX283_REG_MDSEL4		CCI_REG8(0x3007)

#define IMX283_REG_SVR			CCI_REG16_LE(0x3009)

#define IMX283_REG_HTRIMMING		CCI_REG8(0x300b)
#define   IMX283_MDVREV			BIT(0) // VFLIP
#define   IMX283_HTRIMMING_EN		BIT(4)
#define   IMX283_HTRIMMING_RESERVED	BIT(5)

#define IMX283_REG_VWINPOS		CCI_REG16_LE(0x300f)
#define IMX283_REG_VWIDCUT		CCI_REG16_LE(0x3011)

#define IMX283_REG_MDSEL7		CCI_REG16_LE(0x3013)

/* CSI Clock Configuration */
#define IMX283_REG_TCLKPOST		CCI_REG8(0x3018)
#define IMX283_REG_THSPREPARE		CCI_REG8(0x301a)
#define IMX283_REG_THSZERO		CCI_REG8(0x301c)
#define IMX283_REG_THSTRAIL		CCI_REG8(0x3020)
#define IMX283_REG_TCLKPREPARE		CCI_REG8(0x3022)
#define IMX283_REG_TCLKZERO		CCI_REG16_LE(0x3024)
#define IMX283_REG_TLPX			CCI_REG8(0x3026)
#define IMX283_REG_THSEXIT		CCI_REG8(0x3028)
#define IMX283_REG_TCLKPRE		CCI_REG8(0x302a)

#define IMX283_REG_Y_OUT_SIZE		CCI_REG16_LE(0x302f)
#define IMX283_REG_WRITE_VSIZE		CCI_REG16_LE(0x3031)
#define IMX283_REG_OB_SIZE_V		CCI_REG8(0x3033)

/* HMAX internal HBLANK*/
#define IMX283_REG_HMAX			CCI_REG16_LE(0x3036)
#define IMX283_HMAX_MAX			0xffff

/* VMAX internal VBLANK */
#define IMX283_REG_VMAX			CCI_REG24_LE(0x3038)
#define   IMX283_VMAX_MAX		0xfffff

/* SHR internal */
#define IMX283_REG_SHR			CCI_REG16_LE(0x303b)
#define   IMX283_SHR_MIN		11

/*
 * Analog gain control
 *  Gain [dB] = –20log{(2048 – value [10:0]) /2048}
 *  Range: 0dB to approximately +27dB
 */
#define IMX283_REG_ANALOG_GAIN		CCI_REG16_LE(0x3042)
#define   IMX283_ANA_GAIN_MIN		0
#define   IMX283_ANA_GAIN_MAX		1957
#define   IMX283_ANA_GAIN_STEP		1
#define   IMX283_ANA_GAIN_DEFAULT	0x0

/*
 * Digital gain control
 *  Gain [dB] = value * 6
 *  Range: 0dB to +18db
 */
#define IMX283_REG_DIGITAL_GAIN		CCI_REG8(0x3044)
#define IMX283_DGTL_GAIN_MIN		0
#define IMX283_DGTL_GAIN_MAX		3
#define IMX283_DGTL_GAIN_DEFAULT	0
#define IMX283_DGTL_GAIN_STEP		1

#define IMX283_REG_HTRIMMING_START	CCI_REG16_LE(0x3058)
#define IMX283_REG_HTRIMMING_END	CCI_REG16_LE(0x305a)

#define IMX283_REG_MDSEL18		CCI_REG16_LE(0x30f6)

/* Master Mode Operation Control */
#define IMX283_REG_XMSTA		CCI_REG8(0x3105)
#define   IMX283_XMSTA			BIT(0)

#define IMX283_REG_SYNCDRV		CCI_REG8(0x3107)
#define   IMX283_SYNCDRV_XHS_XVS	(0xa0 | 0x02)
#define   IMX283_SYNCDRV_HIZ		(0xa0 | 0x03)

/* PLL Standby */
#define IMX283_REG_STBPL		CCI_REG8(0x320b)
#define  IMX283_STBPL_NORMAL		0x00
#define  IMX283_STBPL_STANDBY		0x03

/* Input Frequency Setting */
#define IMX283_REG_PLRD1		CCI_REG8(0x36c1)
#define IMX283_REG_PLRD2		CCI_REG16_LE(0x36c2)
#define IMX283_REG_PLRD3		CCI_REG8(0x36f7)
#define IMX283_REG_PLRD4		CCI_REG8(0x36f8)

#define IMX283_REG_PLSTMG02		CCI_REG8(0x36aa)
#define   IMX283_PLSTMG02_VAL		0x00

#define IMX283_REG_EBD_X_OUT_SIZE	CCI_REG16_LE(0x3a54)

/* Test pattern generator */
#define IMX283_REG_TPG_CTRL		CCI_REG8(0x3156)
#define   IMX283_TPG_CTRL_CLKEN		BIT(0)
#define   IMX283_TPG_CTRL_PATEN		BIT(4)

#define IMX283_REG_TPG_PAT		CCI_REG8(0x3157)
#define   IMX283_TPG_PAT_ALL_000	0x00
#define   IMX283_TPG_PAT_ALL_FFF	0x01
#define   IMX283_TPG_PAT_ALL_555	0x02
#define   IMX283_TPG_PAT_ALL_AAA	0x03
#define   IMX283_TPG_PAT_H_COLOR_BARS	0x0a
#define   IMX283_TPG_PAT_V_COLOR_BARS	0x0b

#define MHZ(x)				((x) * 1000 * 1000)

/* MIPI link speed is fixed at 1.44Gbps for all the modes */
#define IMX283_DEFAULT_LINK_FREQ	MHZ(720)

/* Exposure control */
#define IMX283_EXPOSURE_MIN		52
#define IMX283_EXPOSURE_STEP		1
#define IMX283_EXPOSURE_DEFAULT		1000
#define IMX283_EXPOSURE_MAX		49865

/* Embedded metadata stream structure */
#define IMX283_EMBEDDED_LINE_WIDTH 16384
#define IMX283_NUM_EMBEDDED_LINES 1

#define IMAGE_PAD			0

/* imx283 native and active pixel array size. */
static const struct v4l2_rect imx283_native_area = {
	.top = 0,
	.left = 0,
	.width = 5592,
	.height = 3710,
};

static const struct v4l2_rect imx283_active_area = {
	.top = 108,
	.left = 40,
	.width = 5472,
	.height = 3648,
};

struct IMX283_reg_list {
	unsigned int num_of_regs;
	const struct cci_reg_sequence *regs;
};

/* Mode : resolution and related config&values */
struct imx283_mode {
	unsigned int mode;

	/* Bits per pixel */
	unsigned int bpp;

	/* Frame width */
	unsigned int width;

	/* Frame height */
	unsigned int height;

	/* minimum H-timing */
	u64 min_HMAX;

	/* minimum V-timing */
	u64 min_VMAX;

	/* default H-timing */
	u64 default_HMAX;

	/* default V-timing */
	u64 default_VMAX;

	/* minimum SHR */
	u64 min_SHR;

	/* Optical Blanking */
	u32 horizontal_ob;
	u32 vertical_ob;

	/* Analog crop rectangle. */
	struct v4l2_rect crop;
};

struct imx283_input_frequency {
	unsigned int mhz;
	unsigned int reg_count;
	struct cci_reg_sequence regs[4];
};

static const struct imx283_input_frequency imx283_frequencies[] = {
	{
		.mhz = MHZ(6),
		.reg_count = 4,
		.regs = {
			{ IMX283_REG_PLRD1, 0x00 },
			{ IMX283_REG_PLRD2, 0x00f0 },
			{ IMX283_REG_PLRD3, 0x00 },
			{ IMX283_REG_PLRD4, 0xc0 },
		},
	},
	{
		.mhz = MHZ(12),
		.reg_count = 4,
		.regs = {
			{ IMX283_REG_PLRD1, 0x01 },
			{ IMX283_REG_PLRD2, 0x00f0 },
			{ IMX283_REG_PLRD3, 0x01 },
			{ IMX283_REG_PLRD4, 0xc0 },
		},
	},
	{
		.mhz = MHZ(18),
		.reg_count = 4,
		.regs = {
			{ IMX283_REG_PLRD1, 0x01 },
			{ IMX283_REG_PLRD2, 0x00a0 },
			{ IMX283_REG_PLRD3, 0x01 },
			{ IMX283_REG_PLRD4, 0x80 },
		},
	},
	{
		.mhz = MHZ(24),
		.reg_count = 4,
		.regs = {
			{ IMX283_REG_PLRD1, 0x02 },
			{ IMX283_REG_PLRD2, 0x00f0 },
			{ IMX283_REG_PLRD3, 0x02 },
			{ IMX283_REG_PLRD4, 0xc0 },
		},
	},
};

enum imx283_modes {
	IMX283_MODE_0,
	IMX283_MODE_1,
	IMX283_MODE_1A,
	IMX283_MODE_1S,
	IMX283_MODE_2,
	IMX283_MODE_2A,
	IMX283_MODE_3,
	IMX283_MODE_4,
	IMX283_MODE_5,
	IMX283_MODE_6,
};

struct imx283_readout_mode {
	u64 mdsel1;
	u64 mdsel2;
	u64 mdsel3;
	u64 mdsel4;
};

static const struct imx283_readout_mode imx283_readout_modes[] = {
	/* All pixel scan modes */
	[IMX283_MODE_0] = { 0x04, 0x03, 0x10, 0x00 }, /* 12 bit */
	[IMX283_MODE_1] = { 0x04, 0x01, 0x00, 0x00 }, /* 10 bit */
	[IMX283_MODE_1A] = { 0x04, 0x01, 0x20, 0x50 }, /* 10 bit */
	[IMX283_MODE_1S] = { 0x04, 0x41, 0x20, 0x50 }, /* 10 bit */

	/* Horizontal / Vertical 2/2-line binning */
	[IMX283_MODE_2] = { 0x0d, 0x11, 0x50, 0x00 }, /* 12 bit */
	[IMX283_MODE_2A] = { 0x0d, 0x11, 0x70, 0x50 }, /* 12 bit */

	/* Horizontal / Vertical 3/3-line binning */
	[IMX283_MODE_3] = { 0x1e, 0x18, 0x10, 0x00 }, /* 12 bit */

	/* Veritcal 2/9 subsampling, horizontal 3 binning cropping */
	[IMX283_MODE_4] = { 0x29, 0x18, 0x30, 0x50 }, /* 12 bit */

	/* Vertical 2/19 subsampling binning, horizontal 3 binning */
	[IMX283_MODE_5] = { 0x2d, 0x18, 0x10, 0x00 }, /* 12 bit */

	/* Vertical 2 binning horizontal 2/4, subsampling 16:9 cropping */
	[IMX283_MODE_6] = { 0x18, 0x21, 0x00, 0x09 }, /* 10 bit */
};

static const struct cci_reg_sequence mipi_data_rate_1440Mbps[] = {
	/* The default register settings provide the 1440Mbps rate */
#if 0
	{ CCI_REG8(0x36c5), 0x00 }, /* Undocumented */
	{ CCI_REG8(0x3ac4), 0x00 }, /* Undocumented */

	{ CCI_REG8(0x320B), 0x00 }, /* STBPL */
	{ CCI_REG8(0x3018), 0xa7 }, /* TCLKPOST */
	{ CCI_REG8(0x301A), 0x6f }, /* THSPREPARE */
	{ CCI_REG8(0x301C), 0x9f }, /* THSZERO */
	{ CCI_REG8(0x301E), 0x5f }, /* THSTRAIL */
	{ CCI_REG8(0x3020), 0x5f }, /* TCLKTRAIL */
	{ CCI_REG8(0x3022), 0x6f }, /* TCLKPREPARE */
	{ CCI_REG8(0x3024), 0x7f }, /* TCLKZERO[7:0] */
	{ CCI_REG8(0x3025), 0x01 }, /* TCLKZERO[8] */
	{ CCI_REG8(0x3026), 0x4f }, /* TLPX*/
	{ CCI_REG8(0x3028), 0x47 }, /* THSEXIT */
	{ CCI_REG8(0x302A), 0x07 }, /* TCKLPRE */
	{ CCI_REG8(0x3104), 0x02 }, /* SYSMODE */

#endif
};

static const struct cci_reg_sequence mipi_data_rate_720Mbps[] = {
	/* Undocumented Arducam Additions "For 720MBps" Setting */
	{ CCI_REG8(0x36c5), 0x01 }, /* Undocumented */
	{ CCI_REG8(0x3ac4), 0x01 }, /* Undocumented */

	{ CCI_REG8(0x320B), 0x00 }, /* STBPL */
	{ CCI_REG8(0x3018), 0x77 }, /* TCLKPOST */
	{ CCI_REG8(0x301A), 0x37 }, /* THSPREPARE */
	{ CCI_REG8(0x301C), 0x67 }, /* THSZERO */
	{ CCI_REG8(0x301E), 0x37 }, /* THSTRAIL */
	{ CCI_REG8(0x3020), 0x37 }, /* TCLKTRAIL */
	{ CCI_REG8(0x3022), 0x37 }, /* TCLKPREPARE */
	{ CCI_REG8(0x3024), 0xDF }, /* TCLKZERO[7:0] */
	{ CCI_REG8(0x3025), 0x00 }, /* TCLKZERO[8] */
	{ CCI_REG8(0x3026), 0x2F }, /* TLPX*/
	{ CCI_REG8(0x3028), 0x47 }, /* THSEXIT */
	{ CCI_REG8(0x302A), 0x0F }, /* TCKLPRE */
	{ CCI_REG8(0x3104), 0x02 }, /* SYSMODE */
};

static const s64 link_frequencies[] = {
	MHZ(720), /* 1440 Mbps lane data rate */
	MHZ(360), /* 720 Mbps data lane rate */
};

static const struct IMX283_reg_list link_freq_reglist[] = {
	{ /* MHZ(720)*/
		.num_of_regs = ARRAY_SIZE(mipi_data_rate_1440Mbps),
		.regs = mipi_data_rate_1440Mbps,
	},
	{ /* MHZ(360) */
		.num_of_regs = ARRAY_SIZE(mipi_data_rate_720Mbps),
		.regs = mipi_data_rate_720Mbps,
	},
};

#define CENTERED_RECTANGLE(rect, _width, _height) \
	{ \
		.left = rect.left + ((rect.width - (_width)) / 2), \
		.top = rect.top + ((rect.height - (_height)) / 2), \
		.width = (_width), \
		.height = (_height), \
	}

/* Mode configs */
static const struct imx283_mode supported_modes_12bit[] = {
	{
		/* 5568x3664 21.40fps readout mode 0 */
		.mode = IMX283_MODE_0,
		.bpp = 12,
		.width = 5472 + 96,
		.height = 3648 + 16,
		.min_HMAX = 887,
		.min_VMAX = 3793,
		.default_HMAX = 900,
		.default_VMAX = 4000,
		.min_SHR = 12,
		.horizontal_ob = 96,
		.vertical_ob = 16,
		.crop = CENTERED_RECTANGLE(imx283_active_area, 5472, 3648),
	},
	{
		/* 2784x1828 51.80fps readout mode 2 */
		.mode = IMX283_MODE_2,
		.bpp = 12,
		.width = (5472 + 96)/2,
		.height = (3648 + 8)/2,
		.min_HMAX = 362,
		.min_VMAX = 3840,
		.default_HMAX = 375,
		.default_VMAX = 3840,
		.min_SHR = 12,
		.horizontal_ob = 96/2,
		.vertical_ob = 8/2,
		.crop = CENTERED_RECTANGLE(imx283_active_area, 5472, 3648),
	},
};

static const struct imx283_mode supported_modes_10bit[] = {
	{
		/* 5568x3664 25.48fps readout mode 1 */
		.mode = IMX283_MODE_1,
		.bpp = 10,
		.width = 5472 + 96,
		.height = 3648 + 16,
		.min_HMAX = 745,
		.min_VMAX = 3793,
		.default_HMAX = 750,
		.default_VMAX = 3840,
		.min_SHR = 12,
		.horizontal_ob = 96,
		.vertical_ob = 16,
		.crop = CENTERED_RECTANGLE(imx283_active_area, 5472, 3648),
	},
	{
		/* 5568x3094 30.17fps readout mode 1A */
		.mode = IMX283_MODE_1A,
		.bpp = 10,
		.width = 5472 + 96,
		.height = 3078 + 16,
		.min_HMAX = 745,
		.min_VMAX = 3203,
		.default_HMAX = 750,
		.default_VMAX = 3840,
		.min_SHR = 12,
		.horizontal_ob = 96,
		.vertical_ob = 16,
		.crop = CENTERED_RECTANGLE(imx283_active_area, 5472, 3078),
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
	/* 10-bit modes. */
	MEDIA_BUS_FMT_SRGGB10_1X10,
	MEDIA_BUS_FMT_SGRBG10_1X10,
	MEDIA_BUS_FMT_SGBRG10_1X10,
	MEDIA_BUS_FMT_SBGGR10_1X10,
};

/* regulator supplies */
static const char * const imx283_supply_name[] = {
	/* Supplies can be enabled in any order */
	"VANA",  /* Analog (2.8V) supply */
	"VDIG",  /* Digital Core (1.1V) supply */
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

struct imx283 {
	struct device *dev;

	const struct imx283_input_frequency *freq;

	/* Selected link_frequency */
	unsigned int link_freq_idx;

	struct v4l2_subdev sd;
	struct media_pad pad;

	unsigned int fmt_code;

	struct clk *xclk;

	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[imx283_NUM_SUPPLIES];

	struct v4l2_ctrl_handler ctrl_handler;
	/* V4L2 Controls */
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *vflip;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;

	/* Current mode */
	const struct imx283_mode *mode;

	u16 hmax;
	u32 vmax;

	/*
	 * Mutex for serialized access:
	 * Protect sensor module set pad format and start/stop streaming safely.
	 */
	struct mutex mutex;

	/* Streaming on/off */
	bool streaming;
};



int cci_read(struct imx283 *imx283, u32 reg, u64 *val, int *err) {
    if (err && *err)
        return *err;

    struct i2c_client *client = v4l2_get_subdevdata(&imx283->sd);
    u32 reg_addr = reg & CCI_REG_ADDR_MASK;
    u32 width = (reg & CCI_REG_WIDTH_MASK) >> CCI_REG_WIDTH_SHIFT;
    u8 addr_buf[2] = { reg_addr >> 8, reg_addr & 0xff };
    u8 data_buf[8] = { 0 };  // Max 8 bytes for 64-bit data
    struct i2c_msg msgs[2];
    int ret;

    if (width == 0 || width > 8) {
        if (err) *err = -EINVAL;
        return -EINVAL;
    }

    // Setup I2C message to write the register address
    msgs[0].addr = client->addr;
    msgs[0].flags = 0;
    msgs[0].len = sizeof(addr_buf);
    msgs[0].buf = addr_buf;

    // Setup I2C message to read data from the register
    msgs[1].addr = client->addr;
    msgs[1].flags = I2C_M_RD;
    msgs[1].len = width;
    msgs[1].buf = data_buf;

    ret = i2c_transfer(client->adapter, msgs, 2);
    if (ret != 2) {
        if (err) *err = -EIO;
        return -EIO;
    }

    // Assuming big-endian register format
    *val = 0;
    for (int i = 0; i < width; i++) {
        *val = (*val << 8) | data_buf[i];
    }

    return 0;
}


int cci_write(struct imx283 *imx283, u32 reg, u64 val, int *err) {

    struct i2c_client *client = v4l2_get_subdevdata(&imx283->sd);
    u32 reg_addr = reg & CCI_REG_ADDR_MASK;
    u32 width = (reg & CCI_REG_WIDTH_MASK) >> CCI_REG_WIDTH_SHIFT;
    bool is_le = reg & CCI_REG_LE;
    u8 buf[10]; // Maximum size needed: 2 bytes for address + 8 bytes for data
    int ret, i;

    // Set the register address (big-endian)
    buf[0] = (reg_addr >> 8) & 0xff;
    buf[1] = reg_addr & 0xff;

    // Set the data
    for (i = 0; i < width; i++) {
        if (is_le) {
            // Little-endian: lower address bytes have lower value bytes
            buf[2 + i] = (val >> (8 * i)) & 0xff;
        } else {
            // Big-endian: lower address bytes have higher value bytes
            buf[2 + width - 1 - i] = (val >> (8 * i)) & 0xff;
        }
    }

    ret = i2c_master_send(client, buf, 2 + width);
    if (ret < 0) {
        if (err) *err = ret;
        return ret;
    }

    return 0;
}


int cci_multi_reg_write(struct imx283 *imx283, const struct cci_reg_sequence *regs, unsigned int num_regs, int *err) {

    for (unsigned int i = 0; i < num_regs; i++) {
        *err = cci_write(imx283, regs[i].reg, regs[i].val, err);
        if (*err)
            return *err;
    }

    return 0;
}



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
	/* 10-bit */
	case MEDIA_BUS_FMT_SRGGB10_1X10:
	case MEDIA_BUS_FMT_SGRBG10_1X10:
	case MEDIA_BUS_FMT_SGBRG10_1X10:
	case MEDIA_BUS_FMT_SBGGR10_1X10:
		*mode_list = supported_modes_10bit;
		*num_modes = ARRAY_SIZE(supported_modes_10bit);
		break;
	default:
		*mode_list = NULL;
		*num_modes = 0;
	}
}

/* Get bayer order based on flip setting. */
static u32 imx283_get_format_code(struct imx283 *imx283, u32 code)
{
	unsigned int i;
	lockdep_assert_held(&imx283->mutex);
	for (i = 0; i < ARRAY_SIZE(codes); i++)
		if (codes[i] == code)
			break;

	return codes[i];
}

static void imx283_set_default_format(struct imx283 *imx283)
{
	/* Set default mode to max resolution */
	imx283->mode = &supported_modes_12bit[0];
	imx283->fmt_code = MEDIA_BUS_FMT_SRGGB12_1X12;
}

// Move this to .init_cfg
static int imx283_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct imx283 *imx283 = to_imx283(sd);
	struct v4l2_mbus_framefmt *try_fmt_img =
		v4l2_subdev_get_try_format(sd, fh->state, IMAGE_PAD);

	struct v4l2_rect *try_crop;

	mutex_lock(&imx283->mutex);

	/* Initialize try_fmt for the image pad */
	try_fmt_img->width = supported_modes_12bit[0].width;
	try_fmt_img->height = supported_modes_12bit[0].height;
	try_fmt_img->code = imx283_get_format_code(imx283,
						   MEDIA_BUS_FMT_SRGGB12_1X12);
	try_fmt_img->field = V4L2_FIELD_NONE;

	/* Initialize try_crop */
	try_crop = v4l2_subdev_get_try_crop(sd, fh->state, IMAGE_PAD);
	*try_crop = imx283_active_area;

	mutex_unlock(&imx283->mutex);

	return 0;
}

static u64 calculate_v4l2_cid_exposure(u64 hmax, u64 vmax, u64 shr, u64 svr, u64 offset) {
    u64 numerator;
    numerator = (vmax * (svr + 1) - shr) * hmax + offset;

    do_div(numerator, hmax);
    numerator = clamp_t(uint32_t, numerator, 0, 0xFFFFFFFF);
    return numerator;
}

static void calculate_min_max_v4l2_cid_exposure(u64 hmax, u64 vmax, u64 min_shr, u64 svr, u64 offset, u64 *min_exposure, u64 *max_exposure) {
    u64 max_shr = (svr + 1) * vmax - 4;
    max_shr = min_t(uint64_t, max_shr, 0xFFFF);

    *min_exposure = calculate_v4l2_cid_exposure(hmax, vmax, max_shr, svr, offset);
    *max_exposure = calculate_v4l2_cid_exposure(hmax, vmax, min_shr, svr, offset);
}


/*
Integration Time [s] = [{VMAX × (SVR + 1) – (SHR)}
 × HMAX + offset] / (72 × 10^6)

Integration Time [s] = exposure * HMAX / (72 × 10^6)
*/

static uint32_t calculate_shr(uint32_t exposure, uint32_t hmax, uint64_t vmax, uint32_t svr, uint32_t offset) {
    uint64_t temp;
    uint32_t shr;

    temp = ((uint64_t)exposure * hmax - offset);
    do_div(temp, hmax);
    shr = (uint32_t)(vmax * (svr + 1) - temp);

    return shr;
}

static const char * const imx283_tpg_menu[] = {
	"Disabled",
	"All 000h",
	"All FFFh",
	"All 555h",
	"All AAAh",
	"Horizontal color bars",
	"Vertical color bars",
};

static const int imx283_tpg_val[] = {
	IMX283_TPG_PAT_ALL_000,
	IMX283_TPG_PAT_ALL_000,
	IMX283_TPG_PAT_ALL_FFF,
	IMX283_TPG_PAT_ALL_555,
	IMX283_TPG_PAT_ALL_AAA,
	IMX283_TPG_PAT_H_COLOR_BARS,
	IMX283_TPG_PAT_V_COLOR_BARS,
};

static int imx283_update_test_pattern(struct imx283 *imx283, u32 pattern_index)
{
	int ret;

	if (pattern_index >= ARRAY_SIZE(imx283_tpg_val))
		return -EINVAL;

	if (pattern_index) {
		ret = cci_write(imx283, IMX283_REG_TPG_PAT,
				imx283_tpg_val[pattern_index], NULL);
		if (ret)
			return ret;

		ret = cci_write(imx283, IMX283_REG_TPG_CTRL,
				IMX283_TPG_CTRL_CLKEN | IMX283_TPG_CTRL_PATEN, NULL);
	} else {
		ret = cci_write(imx283, IMX283_REG_TPG_CTRL, 0x00, NULL);
	}

	return ret;
}

static int imx283_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx283 *imx283 =
		container_of(ctrl->handler, struct imx283, ctrl_handler);
	const struct imx283_mode *mode = imx283->mode;
	u64 shr, pixel_rate, hmax = 0;
	int ret = 0;

	//state = v4l2_subdev_get_locked_active_state(&imx283->sd);
	//format = v4l2_subdev_get_pad_format(&imx283->sd, state, 0);

	/*
	 * The VBLANK control may change the limits of usable exposure, so check
	 * and adjust if necessary.
	 */
	if (ctrl->id == V4L2_CID_VBLANK){
		/* Honour the VBLANK limits when setting exposure. */
		u64 current_exposure, max_exposure, min_exposure, vmax;
		vmax = ((u64)mode->height + ctrl->val) ;
		imx283->vmax = vmax;

		calculate_min_max_v4l2_cid_exposure(imx283->hmax, imx283->vmax,
						    (u64)mode->min_SHR, 0, 209,
						    &min_exposure, &max_exposure);

		current_exposure = clamp_t(uint32_t, current_exposure, min_exposure, max_exposure);

		dev_info(imx283->dev,"exposure_max:%lld, exposure_min:%lld, current_exposure:%lld\n",max_exposure, min_exposure, current_exposure);
		dev_info(imx283->dev, "\tVMAX:%d, HMAX:%d\n", imx283->vmax, imx283->hmax);
		__v4l2_ctrl_modify_range(imx283->exposure, min_exposure,max_exposure, 1,current_exposure);
	}

	/*
	 * Applying V4L2 control value only happens
	 * when power is up for streaming
	 */
	if (pm_runtime_get_if_in_use(imx283->dev) == 0)
		return 0;

	
	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		{
		dev_info(imx283->dev,"V4L2_CID_EXPOSURE : %d\n",ctrl->val);
		dev_info(imx283->dev,"\tvblank:%d, hblank:%d\n",imx283->vblank->val, imx283->hblank->val);
		dev_info(imx283->dev, "\tVMAX:%d, HMAX:%d\n", imx283->vmax, imx283->hmax);
		shr = calculate_shr(ctrl->val, imx283->hmax, imx283->vmax, 0, 209);
		dev_info(imx283->dev,"\tSHR:%lld\n",shr);
		ret = cci_write(imx283, IMX283_REG_SHR, shr, NULL);

		}
		break;

	case V4L2_CID_HBLANK:
		{
		dev_info(imx283->dev, "V4L2_CID_HBLANK : %d\n", ctrl->val);
		//int hmax = (IMX283_NATIVE_WIDTH + ctrl->val) * 72000000; / IMX283_PIXEL_RATE;
		pixel_rate = (u64)mode->width * 72000000;
		do_div(pixel_rate, mode->min_HMAX);
		hmax = (u64)(mode->width + ctrl->val) * 72000000;
		do_div(hmax, pixel_rate);
		imx283->hmax = hmax;
		dev_info(imx283->dev, "\tHMAX : %d\n", imx283->hmax);
		ret = cci_write(imx283, IMX283_REG_HMAX, hmax, NULL);
		}
		break;

	case V4L2_CID_VBLANK:
		{
		dev_info(imx283->dev,"V4L2_CID_VBLANK : %d\n",ctrl->val);
		imx283->vmax = ((u64)mode->height + ctrl->val);
		dev_info(imx283->dev, "\tVMAX : %d\n", imx283->vmax);
		ret = cci_write(imx283, IMX283_REG_VMAX, imx283->vmax, NULL);
		}
		break;

	case V4L2_CID_ANALOGUE_GAIN:
		dev_info(imx283->dev, "V4L2_CID_ANALOGUE_GAIN : %d\n", ctrl->val);
		ret = cci_write(imx283, IMX283_REG_ANALOG_GAIN, ctrl->val, NULL);
		break;

	case V4L2_CID_DIGITAL_GAIN:
		dev_info(imx283->dev, "V4L2_CID_DIGITAL_GAIN : %d\n", ctrl->val);
		ret = cci_write(imx283, IMX283_REG_DIGITAL_GAIN, ctrl->val, NULL);
		break;

	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
		//dev_info(imx283->dev,"V4L2_CID_HFLIP : %d\n",imx283->hflip->val);
		//dev_info(imx283->dev,"V4L2_CID_VFLIP : %d\n",imx283->vflip->val);
		//ret = imx283_write_reg_1byte(imx283, IMX283_REG_VFLIP, imx283->vflip->val);
		break;

	case V4L2_CID_TEST_PATTERN:
		ret = imx283_update_test_pattern(imx283, ctrl->val);
		break;

	default:
		dev_info(imx283->dev,
			 "ctrl(id:0x%x,val:0x%x) is not handled\n",
			 ctrl->id, ctrl->val);
		//ret = -EINVAL;
		break;
	}

	pm_runtime_put(imx283->dev);

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

	if (code->index >= (ARRAY_SIZE(codes) / 4))
		return -EINVAL;

	code->code = imx283_get_format_code(imx283, codes[code->index * 4]);

	return 0;
}

static int imx283_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct imx283 *imx283 = to_imx283(sd);

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

static int imx283_get_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct imx283 *imx283 = to_imx283(sd);

	mutex_lock(&imx283->mutex);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		struct v4l2_mbus_framefmt *try_fmt =
			v4l2_subdev_get_try_format(&imx283->sd, sd_state,
						   fmt->pad);
		/* update the code which could change due to vflip or hflip: */
		try_fmt->code = imx283_get_format_code(imx283, try_fmt->code);
		fmt->format = *try_fmt;
	} else {
		imx283_update_image_pad_format(imx283, imx283->mode, fmt);
		fmt->format.code = imx283_get_format_code(imx283, imx283->fmt_code);
	}

	mutex_unlock(&imx283->mutex);
	return 0;
}

/* TODO */
static void imx283_set_framing_limits(struct imx283 *imx283)
{
	const struct imx283_mode *mode = imx283->mode;
	u64 def_hblank;
	u64 pixel_rate;


	imx283->vmax = mode->default_VMAX;
	imx283->hmax = mode->default_HMAX;

	pixel_rate = (u64)mode->width * 72000000;
	do_div(pixel_rate,mode->min_HMAX);
	dev_info(imx283->dev,"Pixel Rate : %lld\n",pixel_rate);


	//int def_hblank = mode->default_HMAX * IMX283_PIXEL_RATE / 72000000 - IMX283_NATIVE_WIDTH;
	def_hblank = mode->default_HMAX * pixel_rate;
	do_div(def_hblank, 72000000);
	def_hblank = def_hblank - mode->width;
	__v4l2_ctrl_modify_range(imx283->hblank, 0,
				 IMX283_HMAX_MAX, 1, def_hblank);
	__v4l2_ctrl_s_ctrl(imx283->hblank, def_hblank);

	/* Update limits and set FPS to default */
	__v4l2_ctrl_modify_range(imx283->vblank, mode->min_VMAX - mode->height,
				 IMX283_VMAX_MAX - mode->height,
				 1, mode->default_VMAX - mode->height);
	__v4l2_ctrl_s_ctrl(imx283->vblank, mode->default_VMAX - mode->height);

	/* Setting this will adjust the exposure limits as well. */

	__v4l2_ctrl_modify_range(imx283->pixel_rate, pixel_rate, pixel_rate, 1, pixel_rate);

	dev_info(imx283->dev,"Setting default HBLANK : %lld, VBLANK : %lld with PixelRate: %lld\n",def_hblank,mode->default_VMAX - mode->height, pixel_rate);

}
/* TODO */
static int imx283_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt;
	const struct imx283_mode *mode;
	struct imx283 *imx283 = to_imx283(sd);
	const struct imx283_mode *mode_list;
	unsigned int num_modes;

	mutex_lock(&imx283->mutex);

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

static int imx283_standby_cancel(struct imx283 *imx283)
{
	int ret = 0;

	cci_write(imx283, IMX283_REG_STANDBY,
		  IMX283_STBLOGIC | IMX283_STBDV, &ret);

	/* Configure PLL clocks based on the xclk */
	cci_multi_reg_write(imx283, imx283->freq->regs,
			    imx283->freq->reg_count, &ret);

	dev_err(imx283->dev, "Using clk freq %d MHz", imx283->freq->mhz / MHZ(1));

	/* Initialise communication */
	cci_write(imx283, IMX283_REG_PLSTMG08, IMX283_PLSTMG08_VAL, &ret);
	cci_write(imx283, IMX283_REG_PLSTMG02, IMX283_PLSTMG02_VAL, &ret);

	/* Enable PLL */
	cci_write(imx283, IMX283_REG_STBPL, IMX283_STBPL_NORMAL, &ret);

	/* Configure the MIPI link speed */
	cci_multi_reg_write(imx283,
			    link_freq_reglist[imx283->link_freq_idx].regs,
			    link_freq_reglist[imx283->link_freq_idx].num_of_regs,
			    &ret);

	usleep_range(1000, 2000); /* 1st Stabilisation period of 1 ms or more */

	/* Activate */
	cci_write(imx283, IMX283_REG_STANDBY, IMX283_ACTIVE, &ret);
	usleep_range(19000, 20000); /* 2nd Stabilisation period of 19ms or more */

	cci_write(imx283, IMX283_REG_CLAMP, IMX283_CLPSQRST, &ret);
	cci_write(imx283, IMX283_REG_XMSTA, 0, &ret);
	cci_write(imx283, IMX283_REG_SYNCDRV, IMX283_SYNCDRV_XHS_XVS, &ret);

	return ret;
}

/* Start streaming */
static int imx283_start_streaming(struct imx283 *imx283)
{
	const struct imx283_readout_mode *readout;
	const struct imx283_mode *mode = imx283->mode;
	int ret;

	ret = imx283_standby_cancel(imx283);
	if (ret) {
		dev_err(imx283->dev, "failed to cancel standby\n");
		return ret;
	}

	/* Set the readout mode registers */
	readout = &imx283_readout_modes[imx283->mode->mode];
	cci_write(imx283, IMX283_REG_MDSEL1, readout->mdsel1, &ret);
	cci_write(imx283, IMX283_REG_MDSEL2, readout->mdsel2, &ret);
	cci_write(imx283, IMX283_REG_MDSEL3, readout->mdsel3, &ret);
	cci_write(imx283, IMX283_REG_MDSEL4, readout->mdsel4, &ret);

	/* Mode 1S specific entries from the Readout Drive Mode Tables */
	if (mode->mode == IMX283_MODE_1S) {
		cci_write(imx283, IMX283_REG_MDSEL7, 0x01, &ret);
		cci_write(imx283, IMX283_REG_MDSEL18, 0x1098, &ret);
	}

	if (ret) {
		dev_err(imx283->dev, "%s failed to set readout\n", __func__);
		return ret;
	}

	/* Initialise SVR. Unsupported for now - Always 0 */
	cci_write(imx283, IMX283_REG_SVR, 0x00, &ret);

	dev_err(imx283->dev, "Mode: Size %d x %d\n", mode->width, mode->height);

	dev_err(imx283->dev, "Analogue Crop (in the mode) %d,%d %dx%d\n",
		mode->crop.left,
		mode->crop.top,
		mode->crop.width,
		mode->crop.height);

	/* Todo: Update for arbitrary vertical cropping */
	cci_write(imx283, IMX283_REG_Y_OUT_SIZE,
		  mode->height - mode->vertical_ob, &ret);
	cci_write(imx283, IMX283_REG_WRITE_VSIZE, mode->height, &ret);
	cci_write(imx283, IMX283_REG_OB_SIZE_V, mode->vertical_ob, &ret);

	/* Configure cropping */
	cci_write(imx283, IMX283_REG_HTRIMMING,
		  IMX283_HTRIMMING_EN | IMX283_HTRIMMING_RESERVED, &ret);

	/* Todo: Validate mode->crop is fully contained within imx283_native_area */
	/* Todo: Validate with an adjustable crop */
	cci_write(imx283, IMX283_REG_HTRIMMING_START, mode->crop.left, &ret);
	cci_write(imx283, IMX283_REG_HTRIMMING_END,
		  mode->crop.left + mode->crop.width + 1, &ret);

	/* Todo: These must be calculated based on the link-freq and mode */
	cci_write(imx283, IMX283_REG_HMAX, mode->default_HMAX, &ret);
	cci_write(imx283, IMX283_REG_VMAX, mode->default_VMAX, &ret);
	cci_write(imx283, IMX283_REG_SHR, mode->min_SHR, &ret);

	/* Disable embedded data */
	cci_write(imx283, IMX283_REG_EBD_X_OUT_SIZE, 0, &ret);

	/* Apply customized values from user */
	ret =  __v4l2_ctrl_handler_setup(imx283->sd.ctrl_handler);

	return ret;
}

/* Stop streaming */
static void imx283_stop_streaming(struct imx283 *imx283)
{
	int ret;

	ret = cci_write(imx283, IMX283_REG_STANDBY, IMX283_STBLOGIC, NULL);
	if (ret)
		dev_err(imx283->dev, "%s failed to set stream\n", __func__);
}

static int imx283_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct imx283 *imx283 = to_imx283(sd);
	int ret = 0;

	mutex_lock(&imx283->mutex);
	if (imx283->streaming == enable) {
		mutex_unlock(&imx283->mutex);
		return 0;
	}

	if (enable) {
		ret = pm_runtime_get_sync(imx283->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(imx283->dev);
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
		pm_runtime_put(imx283->dev);
	}

	imx283->streaming = enable;

	/* vflip and hflip cannot change during streaming */
	__v4l2_ctrl_grab(imx283->vflip, enable);
	__v4l2_ctrl_grab(imx283->hflip, enable);

	mutex_unlock(&imx283->mutex);

	return ret;

err_rpm_put:
	pm_runtime_put(imx283->dev);
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
		dev_err(imx283->dev, "%s: failed to enable regulators\n",
			__func__);
		return ret;
	}

	ret = clk_prepare_enable(imx283->xclk);
	if (ret) {
		dev_err(imx283->dev, "%s: failed to enable clock\n",
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
	unsigned int i;

	for (i = 0; i < imx283_NUM_SUPPLIES; i++)
		imx283->supplies[i].supply = imx283_supply_name[i];

	return devm_regulator_bulk_get(imx283->dev,
				       imx283_NUM_SUPPLIES,
				       imx283->supplies);
}

/* Verify chip ID */
static int imx283_identify_module(struct imx283 *imx283)
{
	int ret;
	u64 val;

	ret = cci_read(imx283, IMX283_REG_CHIP_ID, &val, NULL);
	if (ret) {
		dev_err(imx283->dev, "failed to read chip id %x, with error %d\n",
			IMX283_CHIP_ID, ret);
		return ret;
	}

	if (val != IMX283_CHIP_ID) {
		dev_err(imx283->dev, "chip id mismatch: %x!=%llx\n",
			IMX283_CHIP_ID, val);
		return -EIO;
	}

	dev_info(imx283->dev, "Device found\n");

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
		sel->r = imx283_native_area;

		return 0;

	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r = imx283_active_area;

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
	const struct imx283_mode *mode = imx283->mode;
	int ret;

	ctrl_hdlr = &imx283->ctrl_handler;
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 16);
	if (ret)
		return ret;

	mutex_init(&imx283->mutex);
	ctrl_hdlr->lock = &imx283->mutex;


	/*
	 * Create the controls here, but mode specific limits are setup
	 * in the imx283_set_framing_limits() call below.
	 */
	/* By default, PIXEL_RATE is read only */
	imx283->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &imx283_ctrl_ops,
					       V4L2_CID_PIXEL_RATE,
					       0xffff,
					       0xffff, 1,
					       0xffff);

	imx283->link_freq = v4l2_ctrl_new_int_menu(ctrl_hdlr,
						   &imx283_ctrl_ops,
						   V4L2_CID_LINK_FREQ,
						   ARRAY_SIZE(link_frequencies) - 1,
						   0, link_frequencies);
	if (imx283->link_freq)
		imx283->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/* Initial vblank/hblank/exposure based on the current mode. */
	imx283->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &imx283_ctrl_ops,
					   V4L2_CID_VBLANK,
					   mode->min_VMAX - mode->height,
					   IMX283_VMAX_MAX, 1,
					   mode->default_VMAX - mode->height);

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

	imx283->hflip = v4l2_ctrl_new_std(ctrl_hdlr, &imx283_ctrl_ops,
					  V4L2_CID_HFLIP, 0, 1, 1, 0);
	if (imx283->hflip)
		imx283->hflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	imx283->vflip = v4l2_ctrl_new_std(ctrl_hdlr, &imx283_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 0);
	if (imx283->vflip)
		imx283->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	v4l2_ctrl_new_std_menu_items(ctrl_hdlr, &imx283_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(imx283_tpg_menu) - 1,
				     0, 0, imx283_tpg_menu);

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

static const struct of_device_id imx283_dt_ids[] = {
	{ .compatible = "sony,imx283", },
	{ /* sentinel */ }
};

static int imx283_parse_endpoint(struct imx283 *imx283)
{
	struct fwnode_handle *fwnode = dev_fwnode(imx283->dev);
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY
	};
	struct fwnode_handle *ep;
	int ret;
	int i, j;

	if (!fwnode)
		return -ENXIO;

	ep = fwnode_graph_get_next_endpoint(fwnode, NULL);
	if (!ep) {
		dev_err(imx283->dev, "Failed to get next endpoint\n");
		return -ENXIO;
	}

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &bus_cfg);
	fwnode_handle_put(ep);
	if (ret)
		return ret;

	if (bus_cfg.bus.mipi_csi2.num_data_lanes != 4) {
		dev_err(imx283->dev,
			"number of CSI2 data lanes %d is not supported\n",
			bus_cfg.bus.mipi_csi2.num_data_lanes);
		ret = -EINVAL;
		goto done_endpoint_free;
	}

	if (!bus_cfg.nr_of_link_frequencies) {
		dev_err(imx283->dev, "no link frequencies defined\n");
		ret = -EINVAL;
		goto done_endpoint_free;
	}

	for (i = 0; i < bus_cfg.nr_of_link_frequencies; i++) {
		for (j = 0; j < ARRAY_SIZE(link_frequencies); j++) {
			if (bus_cfg.link_frequencies[i] == link_frequencies[j]) {
				imx283->link_freq_idx = j;
				break;
			}
		}

		if (j == ARRAY_SIZE(link_frequencies)) {
			ret = dev_err_probe(imx283->dev, -EINVAL,
					    "no supported link freq found\n");
			goto done_endpoint_free;
		}
	}

done_endpoint_free:
	v4l2_fwnode_endpoint_free(&bus_cfg);

	return ret;
};

static int imx283_probe(struct i2c_client *client)
{
	struct imx283 *imx283;
	int ret;
	unsigned int i;
	unsigned int xclk_freq;

	imx283 = devm_kzalloc(&client->dev, sizeof(*imx283), GFP_KERNEL);
	if (!imx283)
		return -ENOMEM;

	imx283->dev = &client->dev;

	struct device *dev = &client->dev;

	v4l2_i2c_subdev_init(&imx283->sd, client, &imx283_subdev_ops);

	/*
	imx283 = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(imx283)) {
		ret = PTR_ERR(imx283);
		dev_err(imx283->dev, "failed to initialize CCI: %d\n", ret);
		return ret;
	}
	*/

	/* Get system clock (xclk) */
	imx283->xclk = devm_clk_get(imx283->dev, NULL);
	if (IS_ERR(imx283->xclk)) {
		dev_err(imx283->dev, "failed to get xclk\n");
		return PTR_ERR(imx283->xclk);
	}

	xclk_freq = clk_get_rate(imx283->xclk);
	for (i = 0; i < ARRAY_SIZE(imx283_frequencies); i++) {
		if (xclk_freq == imx283_frequencies[i].mhz) {
			imx283->freq = &imx283_frequencies[i];
			break;
		}
	}
	if (!imx283->freq) {
		dev_err(imx283->dev, "xclk frequency unsupported: %d Hz\n", xclk_freq);
		return -EINVAL;
	}

	ret = imx283_get_regulators(imx283);
	if (ret) {
		dev_err(imx283->dev, "failed to get regulators\n");
		return ret;
	}

	ret = imx283_parse_endpoint(imx283);
	if (ret) {
		dev_err(imx283->dev, "failed to parse endpoint configuration\n");
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

	ret = imx283_identify_module(imx283);
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
		goto error_pm;

	/* Initialize subdev */
	imx283->sd.internal_ops = &imx283_internal_ops;
	imx283->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
			    V4L2_SUBDEV_FL_HAS_EVENTS;
	imx283->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	/* Initialize source pads */
	imx283->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&imx283->sd.entity, 1, &imx283->pad);
	if (ret) {
		dev_err(imx283->dev, "failed to init entity pads: %d\n", ret);
		goto error_handler_free;
	}

	ret = v4l2_async_register_subdev_sensor(&imx283->sd);
	if (ret < 0) {
		dev_err(imx283->dev, "failed to register sensor sub-device: %d\n", ret);
		goto error_media_entity;
	}

	return 0;

error_media_entity:
	media_entity_cleanup(&imx283->sd.entity);

error_handler_free:
	imx283_free_controls(imx283);

error_pm:
	pm_runtime_disable(imx283->dev);
	pm_runtime_set_suspended(imx283->dev);
error_power_off:
	imx283_power_off(imx283->dev);

	return ret;
}

static void imx283_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx283 *imx283 = to_imx283(sd);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	imx283_free_controls(imx283);

	pm_runtime_disable(imx283->dev);
	if (!pm_runtime_status_suspended(imx283->dev))
		imx283_power_off(imx283->dev);
	pm_runtime_set_suspended(imx283->dev);

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
	.probe = imx283_probe,
	.remove = imx283_remove,
};

module_i2c_driver(imx283_i2c_driver);

MODULE_DESCRIPTION("Sony IMX283 Sensor Driver");
MODULE_LICENSE("GPL v2");
