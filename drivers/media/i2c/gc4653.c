// SPDX-License-Identifier: GPL-2.0
/*
 * GalaxyCore GC4653 4MP RAW10 MIPI CSI-2 sensor driver.
 *
 * Single mode: 2560x1440p30, 2 data lanes, 10-bit Bayer. The register
 * program is transcribed from Sipeed's LicheeRV-Nano beta-board SDK
 * (middleware/v2/component/isp/sensor/cv182x/gcore_gc4653/
 * gc4653_sensor_ctl.c, gc4653_linear_1440p30_init), which drives the
 * sensor from userspace; no vendor kernel driver exists.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#define GC4653_XVCLK_FREQ	24000000

#define GC4653_REG_CHIP_ID_H	CCI_REG8(0x03f0)
#define GC4653_REG_CHIP_ID_L	CCI_REG8(0x03f1)
#define GC4653_CHIP_ID		0x4653

#define GC4653_REG_STREAMING	CCI_REG8(0x0100)

#define GC4653_NATIVE_WIDTH	2560
#define GC4653_NATIVE_HEIGHT	1440
#define GC4653_DEFAULT_VTS	0x05dc
#define GC4653_EXPOSURE_DEFAULT	0x05d0
#define GC4653_VBLANK_DEFAULT	60
#define GC4653_VTS_MAX		0x3fff

#define GC4653_REG_EXPOSURE_H	CCI_REG8(0x0202)
#define GC4653_REG_EXPOSURE_L	CCI_REG8(0x0203)
#define GC4653_REG_DGAIN_H	CCI_REG8(0x020e)
#define GC4653_REG_DGAIN_L	CCI_REG8(0x020f)
#define GC4653_REG_VTS_H		CCI_REG8(0x0340)
#define GC4653_REG_VTS_L		CCI_REG8(0x0341)

static const struct cci_reg_sequence gc4653_init_regs[] = {
	{ CCI_REG8(0x03fe), 0xf0 },
	{ CCI_REG8(0x03fe), 0x00 },
	{ CCI_REG8(0x0317), 0x00 },
	{ CCI_REG8(0x0320), 0x77 },
	{ CCI_REG8(0x0324), 0xc8 },
	{ CCI_REG8(0x0325), 0x06 },
	{ CCI_REG8(0x0326), 0x6c },
	{ CCI_REG8(0x0327), 0x03 },
	{ CCI_REG8(0x0334), 0x40 },
	{ CCI_REG8(0x0336), 0x6c },
	{ CCI_REG8(0x0337), 0x82 },
	{ CCI_REG8(0x0315), 0x25 },
	{ CCI_REG8(0x031c), 0xc6 },
	{ CCI_REG8(0x0287), 0x18 },
	{ CCI_REG8(0x0084), 0x00 },
	{ CCI_REG8(0x0087), 0x50 },
	{ CCI_REG8(0x029d), 0x08 },
	{ CCI_REG8(0x0290), 0x00 },
	{ CCI_REG8(0x0340), 0x05 },
	{ CCI_REG8(0x0341), 0xdc },
	{ CCI_REG8(0x0345), 0x06 },
	{ CCI_REG8(0x034b), 0xb0 },
	{ CCI_REG8(0x0352), 0x06 },
	{ CCI_REG8(0x0354), 0x07 },
	{ CCI_REG8(0x02d1), 0xe0 },
	{ CCI_REG8(0x0223), 0xf2 },
	{ CCI_REG8(0x0238), 0xa4 },
	{ CCI_REG8(0x02ce), 0x7f },
	{ CCI_REG8(0x0232), 0xc4 },
	{ CCI_REG8(0x02d3), 0x05 },
	{ CCI_REG8(0x0243), 0x06 },
	{ CCI_REG8(0x02ee), 0x30 },
	{ CCI_REG8(0x026f), 0x70 },
	{ CCI_REG8(0x0257), 0x09 },
	{ CCI_REG8(0x0211), 0x02 },
	{ CCI_REG8(0x0219), 0x09 },
	{ CCI_REG8(0x023f), 0x2d },
	{ CCI_REG8(0x0518), 0x00 },
	{ CCI_REG8(0x0519), 0x1e },
	{ CCI_REG8(0x0515), 0x30 },
	{ CCI_REG8(0x02d9), 0x5c },
	{ CCI_REG8(0x02da), 0x02 },
	{ CCI_REG8(0x02db), 0xe8 },
	{ CCI_REG8(0x02e6), 0x20 },
	{ CCI_REG8(0x021b), 0x10 },
	{ CCI_REG8(0x0252), 0x22 },
	{ CCI_REG8(0x024e), 0x22 },
	{ CCI_REG8(0x02c4), 0x01 },
	{ CCI_REG8(0x021d), 0x17 },
	{ CCI_REG8(0x024a), 0x01 },
	{ CCI_REG8(0x02ca), 0x02 },
	{ CCI_REG8(0x0262), 0x10 },
	{ CCI_REG8(0x029a), 0x20 },
	{ CCI_REG8(0x021c), 0x0e },
	{ CCI_REG8(0x0298), 0x03 },
	{ CCI_REG8(0x029c), 0x00 },
	{ CCI_REG8(0x027e), 0x14 },
	{ CCI_REG8(0x02c2), 0x10 },
	{ CCI_REG8(0x0540), 0x20 },
	{ CCI_REG8(0x0546), 0x01 },
	{ CCI_REG8(0x0548), 0x01 },
	{ CCI_REG8(0x0544), 0x01 },
	{ CCI_REG8(0x0242), 0x1b },
	{ CCI_REG8(0x02c0), 0x1b },
	{ CCI_REG8(0x02c3), 0x20 },
	{ CCI_REG8(0x02e4), 0x10 },
	{ CCI_REG8(0x022e), 0x00 },
	{ CCI_REG8(0x027b), 0x3f },
	{ CCI_REG8(0x0269), 0x0f },
	{ CCI_REG8(0x02d2), 0x40 },
	{ CCI_REG8(0x027c), 0x08 },
	{ CCI_REG8(0x023a), 0x2e },
	{ CCI_REG8(0x0245), 0xce },
	{ CCI_REG8(0x0530), 0x20 },
	{ CCI_REG8(0x0531), 0x02 },
	{ CCI_REG8(0x0228), 0x50 },
	{ CCI_REG8(0x02ab), 0x00 },
	{ CCI_REG8(0x0250), 0x00 },
	{ CCI_REG8(0x0221), 0x50 },
	{ CCI_REG8(0x02ac), 0x00 },
	{ CCI_REG8(0x02a5), 0x02 },
	{ CCI_REG8(0x0260), 0x0b },
	{ CCI_REG8(0x0216), 0x04 },
	{ CCI_REG8(0x0299), 0x1c },
	{ CCI_REG8(0x02bb), 0x0d },
	{ CCI_REG8(0x02a3), 0x02 },
	{ CCI_REG8(0x02a4), 0x02 },
	{ CCI_REG8(0x021e), 0x02 },
	{ CCI_REG8(0x024f), 0x08 },
	{ CCI_REG8(0x028c), 0x08 },
	{ CCI_REG8(0x0532), 0x3f },
	{ CCI_REG8(0x0533), 0x02 },
	{ CCI_REG8(0x0277), 0xc0 },
	{ CCI_REG8(0x0276), 0xc0 },
	{ CCI_REG8(0x0239), 0xc0 },
	{ CCI_REG8(0x0202), 0x05 },
	{ CCI_REG8(0x0203), 0xd0 },
	{ CCI_REG8(0x0205), 0xc0 },
	{ CCI_REG8(0x02b0), 0x68 },
	{ CCI_REG8(0x0002), 0xa9 },
	{ CCI_REG8(0x0004), 0x01 },
	{ CCI_REG8(0x021a), 0x98 },
	{ CCI_REG8(0x0266), 0xa0 },
	{ CCI_REG8(0x0020), 0x01 },
	{ CCI_REG8(0x0021), 0x03 },
	{ CCI_REG8(0x0022), 0x00 },
	{ CCI_REG8(0x0023), 0x04 },
	{ CCI_REG8(0x0342), 0x06 },
	{ CCI_REG8(0x0343), 0x40 },
	{ CCI_REG8(0x03fe), 0x10 },
	{ CCI_REG8(0x03fe), 0x00 },
	{ CCI_REG8(0x0106), 0x78 },
	{ CCI_REG8(0x0108), 0x0c },
	{ CCI_REG8(0x0114), 0x01 },
	{ CCI_REG8(0x0115), 0x10 },
	{ CCI_REG8(0x0180), 0x46 },
	{ CCI_REG8(0x0181), 0x30 },
	{ CCI_REG8(0x0182), 0x05 },
	{ CCI_REG8(0x0185), 0x01 },
	{ CCI_REG8(0x03fe), 0x10 },
	{ CCI_REG8(0x03fe), 0x00 },
	{ CCI_REG8(0x000f), 0x00 },
	{ CCI_REG8(0x0100), 0x00 },
	{ CCI_REG8(0x0080), 0x02 },
	{ CCI_REG8(0x0097), 0x0a },
	{ CCI_REG8(0x0098), 0x10 },
	{ CCI_REG8(0x0099), 0x05 },
	{ CCI_REG8(0x009a), 0xb0 },
	{ CCI_REG8(0x0317), 0x08 },
	{ CCI_REG8(0x0a67), 0x80 },
	{ CCI_REG8(0x0a70), 0x03 },
	{ CCI_REG8(0x0a82), 0x00 },
	{ CCI_REG8(0x0a83), 0x10 },
	{ CCI_REG8(0x0a80), 0x2b },
	{ CCI_REG8(0x05be), 0x00 },
	{ CCI_REG8(0x05a9), 0x01 },
	{ CCI_REG8(0x0313), 0x80 },
	{ CCI_REG8(0x05be), 0x01 },
	{ CCI_REG8(0x0317), 0x00 },
	{ CCI_REG8(0x0a67), 0x00 },
};

/*
 * Vendor gain table from gc4653_cmos.c.  The seven values in each row
 * are written to 0x02b3, 0x02b4, 0x02b8, 0x02b9, 0x0515, 0x0519,
 * and 0x02d9 respectively.
 */
static const u32 gc4653_gain_table[] = {
	1024, 1200, 1424, 1696, 2048, 2416, 2864, 3392, 4096, 4848, 5728,
	6784, 8192, 9696, 11456, 13584, 16384, 19408, 22928, 27168, 32768,
	38816, 45872, 54352, 65536, 77648,
};

static const u8 gc4653_gain_regs[][7] = {
	{ 0x00, 0x00, 0x01, 0x00, 0x30, 0x1e, 0x5c },
	{ 0x20, 0x00, 0x01, 0x0b, 0x30, 0x1e, 0x5c },
	{ 0x01, 0x00, 0x01, 0x19, 0x30, 0x1d, 0x5b },
	{ 0x21, 0x00, 0x01, 0x2a, 0x30, 0x1e, 0x5c },
	{ 0x02, 0x00, 0x02, 0x00, 0x30, 0x1e, 0x5c },
	{ 0x22, 0x00, 0x02, 0x17, 0x30, 0x1d, 0x5b },
	{ 0x03, 0x00, 0x02, 0x33, 0x20, 0x16, 0x54 },
	{ 0x23, 0x00, 0x03, 0x14, 0x20, 0x17, 0x55 },
	{ 0x04, 0x00, 0x04, 0x00, 0x20, 0x17, 0x55 },
	{ 0x24, 0x00, 0x04, 0x2f, 0x20, 0x19, 0x57 },
	{ 0x05, 0x00, 0x05, 0x26, 0x20, 0x19, 0x57 },
	{ 0x25, 0x00, 0x06, 0x28, 0x20, 0x1b, 0x59 },
	{ 0x0c, 0x00, 0x08, 0x00, 0x20, 0x1d, 0x5b },
	{ 0x2c, 0x00, 0x09, 0x1e, 0x20, 0x1f, 0x5d },
	{ 0x0d, 0x00, 0x0b, 0x0c, 0x20, 0x21, 0x5f },
	{ 0x2d, 0x00, 0x0d, 0x11, 0x20, 0x24, 0x62 },
	{ 0x1c, 0x00, 0x10, 0x00, 0x20, 0x26, 0x64 },
	{ 0x3c, 0x00, 0x12, 0x3d, 0x18, 0x2a, 0x68 },
	{ 0x5c, 0x00, 0x16, 0x19, 0x18, 0x2c, 0x6a },
	{ 0x7c, 0x00, 0x1a, 0x22, 0x18, 0x2e, 0x6c },
	{ 0x9c, 0x00, 0x20, 0x00, 0x18, 0x32, 0x70 },
	{ 0xbc, 0x00, 0x25, 0x3a, 0x18, 0x35, 0x73 },
	{ 0xdc, 0x00, 0x2c, 0x33, 0x10, 0x36, 0x74 },
	{ 0xfc, 0x00, 0x35, 0x05, 0x10, 0x38, 0x76 },
	{ 0x1c, 0x01, 0x40, 0x00, 0x10, 0x3c, 0x7a },
	{ 0x3c, 0x01, 0x4b, 0x35, 0x10, 0x42, 0x80 },
};

struct gc4653 {
	struct i2c_client *client;
	struct regmap *regmap;
	struct clk *xvclk;
	struct gpio_desc *reset;

	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *analogue_gain;
	struct mutex mutex;
	bool streaming;
};

static inline struct gc4653 *to_gc4653(struct v4l2_subdev *sd)
{
	return container_of(sd, struct gc4653, sd);
}

static const u32 reg_gc4653_gain[] = {
	CCI_REG8(0x02b3), CCI_REG8(0x02b4), CCI_REG8(0x02b8),
	CCI_REG8(0x02b9), CCI_REG8(0x0515), CCI_REG8(0x0519),
	CCI_REG8(0x02d9),
};

static int gc4653_write_gain(struct gc4653 *gc4653, u32 gain)
{
	u32 dgain;
	unsigned int i, j;
	int ret;

	for (i = 1; i < ARRAY_SIZE(gc4653_gain_table); i++)
		if (gain < gc4653_gain_table[i])
			break;
	i--;
	dgain = gain * 64 / gc4653_gain_table[i];

	for (j = 0; j < ARRAY_SIZE(reg_gc4653_gain); j++) {
		ret = cci_write(gc4653->regmap, reg_gc4653_gain[j],
				gc4653_gain_regs[i][j], NULL);
		if (ret)
			return ret;
	}

	ret = cci_write(gc4653->regmap, GC4653_REG_DGAIN_H,
			dgain >> 6, NULL);
	if (ret)
		return ret;

	return cci_write(gc4653->regmap, GC4653_REG_DGAIN_L,
			(dgain & 0x3f) << 2, NULL);
}

static int gc4653_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct gc4653 *gc4653 =
		container_of(ctrl->handler, struct gc4653, ctrl_handler);
	u32 vts;
	int ret;

	if (ctrl->id == V4L2_CID_VBLANK) {
		vts = GC4653_NATIVE_HEIGHT + ctrl->val;
		ret = __v4l2_ctrl_modify_range(gc4653->exposure, 1, vts - 8,
						1, GC4653_EXPOSURE_DEFAULT);
		if (ret)
			return ret;
	}

	/*
	 * Power is managed explicitly by s_stream(). Controls changed while
	 * stopped remain cached and are applied by v4l2_ctrl_handler_setup().
	 */
	if (!gc4653->streaming)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		ret = cci_write(gc4653->regmap, GC4653_REG_EXPOSURE_H,
				ctrl->val >> 8, NULL);
		if (ret)
			return ret;
		return cci_write(gc4653->regmap, GC4653_REG_EXPOSURE_L,
				ctrl->val & 0xff, NULL);

	case V4L2_CID_VBLANK:
		vts = GC4653_NATIVE_HEIGHT + ctrl->val;
		ret = cci_write(gc4653->regmap, GC4653_REG_VTS_H,
				(vts >> 8) & 0x3f, NULL);
		if (ret)
			return ret;
		ret = cci_write(gc4653->regmap, GC4653_REG_VTS_L,
				vts & 0xff, NULL);
		if (ret)
			return ret;

		return 0;

	case V4L2_CID_ANALOGUE_GAIN:
		return gc4653_write_gain(gc4653, ctrl->val);
	}

	return -EINVAL;
}

static const struct v4l2_ctrl_ops gc4653_ctrl_ops = {
	.s_ctrl = gc4653_set_ctrl,
};

static int gc4653_init_controls(struct gc4653 *gc4653)
{
	struct v4l2_ctrl_handler *handler = &gc4653->ctrl_handler;

	v4l2_ctrl_handler_init(handler, 3);
	handler->lock = &gc4653->mutex;

	gc4653->exposure = v4l2_ctrl_new_std(handler, &gc4653_ctrl_ops,
						V4L2_CID_EXPOSURE, 1,
						GC4653_DEFAULT_VTS - 8, 1,
						GC4653_EXPOSURE_DEFAULT);
	gc4653->vblank = v4l2_ctrl_new_std(handler, &gc4653_ctrl_ops,
						V4L2_CID_VBLANK, GC4653_VBLANK_DEFAULT,
						GC4653_VTS_MAX - GC4653_NATIVE_HEIGHT, 1,
						GC4653_VBLANK_DEFAULT);
	gc4653->analogue_gain = v4l2_ctrl_new_std(handler, &gc4653_ctrl_ops,
						V4L2_CID_ANALOGUE_GAIN,
						gc4653_gain_table[0],
						gc4653_gain_table[ARRAY_SIZE(gc4653_gain_table) - 1],
						1, gc4653_gain_table[0]);

	if (handler->error) {
		int ret = handler->error;

		v4l2_ctrl_handler_free(handler);
		return ret;
	}

	gc4653->sd.ctrl_handler = handler;
	return 0;
}

static int gc4653_power_on(struct gc4653 *gc4653)
{
	int ret;

	/* Board straps AVDD/DVDD/IOVDD; nothing to toggle here. */
	ret = clk_prepare_enable(gc4653->xvclk);
	if (ret)
		return ret;

	/* Sensor needs MCLK alive while reset is released. */
	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(gc4653->reset, 0);
	usleep_range(5000, 10000);

	return 0;
}

static void gc4653_power_off(struct gc4653 *gc4653)
{
	gpiod_set_value_cansleep(gc4653->reset, 1);
	clk_disable_unprepare(gc4653->xvclk);
}

static int gc4653_identify(struct gc4653 *gc4653)
{
	struct regmap *regmap = gc4653->regmap;
	u64 val;
	int ret;

	ret = cci_read(regmap, GC4653_REG_CHIP_ID_H, &val, NULL);
	if (ret)
		return ret;
	if ((val & 0xff) != (GC4653_CHIP_ID >> 8))
		return -ENODEV;

	ret = cci_read(regmap, GC4653_REG_CHIP_ID_L, &val, NULL);
	if (ret)
		return ret;
	if ((val & 0xff) != (GC4653_CHIP_ID & 0xff))
		return -ENODEV;

	return 0;
}

static int gc4653_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct gc4653 *gc4653 = to_gc4653(sd);
	struct regmap *regmap = gc4653->regmap;
	int ret;

	if (!enable) {
		mutex_lock(&gc4653->mutex);
		if (!gc4653->streaming) {
			mutex_unlock(&gc4653->mutex);
			return 0;
		}
		gc4653->streaming = false;
		/* Vendor gc4653_standby(). */
		cci_write(regmap, GC4653_REG_STREAMING, 0x00, NULL);
		cci_write(regmap, CCI_REG8(0x031c), 0xc7, NULL);
		cci_write(regmap, CCI_REG8(0x0317), 0x01, NULL);
		gc4653_power_off(gc4653);
		mutex_unlock(&gc4653->mutex);
		return 0;
	}

	mutex_lock(&gc4653->mutex);
	if (gc4653->streaming) {
		mutex_unlock(&gc4653->mutex);
		return 0;
	}

	ret = gc4653_power_on(gc4653);
	if (ret)
		goto unlock;

	/* Load the mode table while the sensor remains in standby.  Starting
	 * before the cached controls are applied can make a large exposure /
	 * gain transition corrupt the first CSI frame (CRC/word-count). */
	ret = cci_multi_reg_write(regmap, gc4653_init_regs,
				  ARRAY_SIZE(gc4653_init_regs), NULL);
	if (ret) {
		gc4653_power_off(gc4653);
		goto unlock;
	}

	gc4653->streaming = true;

	/* Apply cached controls after the vendor mode table is loaded. */
	ret = __v4l2_ctrl_handler_setup(&gc4653->ctrl_handler);
	if (!ret)
		ret = cci_write(regmap, GC4653_REG_STREAMING, 0x09, NULL);
	if (ret) {
		gc4653->streaming = false;
		gc4653_power_off(gc4653);
	}
	mutex_unlock(&gc4653->mutex);
	return ret;

unlock:
	mutex_unlock(&gc4653->mutex);
	return ret;
}

static int gc4653_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_SRGGB10_1X10;
	return 0;
}

static int gc4653_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index || fse->code != MEDIA_BUS_FMT_SRGGB10_1X10)
		return -EINVAL;

	fse->min_width = GC4653_NATIVE_WIDTH;
	fse->max_width = GC4653_NATIVE_WIDTH;
	fse->min_height = GC4653_NATIVE_HEIGHT;
	fse->max_height = GC4653_NATIVE_HEIGHT;
	return 0;
}

static int gc4653_set_format(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state,
			     struct v4l2_subdev_format *fmt)
{
	/* Single mode: nothing to negotiate. */
	fmt->format.code = MEDIA_BUS_FMT_SRGGB10_1X10;
	fmt->format.width = GC4653_NATIVE_WIDTH;
	fmt->format.height = GC4653_NATIVE_HEIGHT;
	fmt->format.field = V4L2_FIELD_NONE;
	fmt->format.colorspace = V4L2_COLORSPACE_RAW;

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		return 0;

	return 0;
}

static int gc4653_get_frame_interval(struct v4l2_subdev *sd,
				     struct v4l2_subdev_state *state,
				     struct v4l2_subdev_frame_interval *fi)
{
	fi->interval.numerator = 1;
	fi->interval.denominator = 30;
	return 0;
}

static const struct v4l2_subdev_core_ops gc4653_core_ops = {
};

static const struct v4l2_subdev_video_ops gc4653_video_ops = {
	.s_stream = gc4653_s_stream,
};

static const struct v4l2_subdev_pad_ops gc4653_pad_ops = {
	.enum_mbus_code = gc4653_enum_mbus_code,
	.enum_frame_size = gc4653_enum_frame_size,
	.get_fmt = gc4653_set_format,
	.set_fmt = gc4653_set_format,
	.get_frame_interval = gc4653_get_frame_interval,
};

static const struct v4l2_subdev_ops gc4653_subdev_ops = {
	.core = &gc4653_core_ops,
	.video = &gc4653_video_ops,
	.pad = &gc4653_pad_ops,
};

static const struct regmap_config gc4653_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
};

static int gc4653_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct gc4653 *gc4653;
	int ret;

	gc4653 = devm_kzalloc(dev, sizeof(*gc4653), GFP_KERNEL);
	if (!gc4653)
		return -ENOMEM;

	gc4653->client = client;
	gc4653->regmap = devm_regmap_init_i2c(client, &gc4653_regmap_config);
	if (IS_ERR(gc4653->regmap))
		return dev_err_probe(dev, PTR_ERR(gc4653->regmap),
				     "regmap init failed\n");

	gc4653->xvclk = devm_clk_get_optional(dev, "xvclk");
	if (IS_ERR(gc4653->xvclk))
		return dev_err_probe(dev, PTR_ERR(gc4653->xvclk),
				     "failed to get xvclk\n");

	gc4653->reset = devm_gpiod_get_optional(dev, "reset",
						GPIOD_OUT_HIGH);
	if (IS_ERR(gc4653->reset))
		return dev_err_probe(dev, PTR_ERR(gc4653->reset),
				     "failed to get reset gpio\n");

	ret = gc4653_power_on(gc4653);
	if (ret)
		return dev_err_probe(dev, ret, "sensor power-up failed\n");

	ret = gc4653_identify(gc4653);
	if (ret) {
		dev_err_probe(dev, ret, "sensor not found (chip id)\n");
		goto power_off;
	}

	v4l2_i2c_subdev_init(&gc4653->sd, client, &gc4653_subdev_ops);
	gc4653->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	gc4653->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	gc4653->pad.flags = MEDIA_PAD_FL_SOURCE;

	mutex_init(&gc4653->mutex);
	ret = gc4653_init_controls(gc4653);
	if (ret)
		goto power_off;

	ret = media_entity_pads_init(&gc4653->sd.entity, 1, &gc4653->pad);
	if (ret)
		goto ctrl_cleanup;

	ret = v4l2_subdev_init_finalize(&gc4653->sd);
	if (ret)
		goto entity_cleanup;

	ret = v4l2_async_register_subdev_sensor(&gc4653->sd);
	if (ret)
		goto subdev_cleanup;

	/* Stay powered: the capture side owns stream on/off from here. */
	return 0;

subdev_cleanup:
	v4l2_subdev_cleanup(&gc4653->sd);
entity_cleanup:
	media_entity_cleanup(&gc4653->sd.entity);
ctrl_cleanup:
	v4l2_ctrl_handler_free(&gc4653->ctrl_handler);
power_off:
	gc4653_power_off(gc4653);
	return ret;
}

static void gc4653_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gc4653 *gc4653 = to_gc4653(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&gc4653->ctrl_handler);
	gc4653_power_off(gc4653);
}

static const struct of_device_id gc4653_of_match[] = {
	{ .compatible = "galaxycore,gc4653" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, gc4653_of_match);

static struct i2c_driver gc4653_driver = {
	.driver = {
		.name = "gc4653",
		.of_match_table = gc4653_of_match,
	},
	.probe = gc4653_probe,
	.remove = gc4653_remove,
};
module_i2c_driver(gc4653_driver);

MODULE_DESCRIPTION("GalaxyCore GC4653 RAW10 MIPI CSI-2 sensor driver");
MODULE_LICENSE("GPL");
