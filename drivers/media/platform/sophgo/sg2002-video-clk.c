// SPDX-License-Identifier: GPL-2.0-only
/*
 * Sophgo SG2002 video clock profile
 *
 * The codec core (vc_src0: Coda980, WAVE420L, JPEG unit), the VPSS and the
 * capture (src_vip_sys_1) and the codec bus (axi_video_codec) can run at the
 * firmware's rates or at the vendor's VC-overdrive rates. The device tree
 * sets one profile at boot (assigned-clock-rates on the clock controller);
 * this driver lets user space choose the other without a reboot.
 *
 * The choice is the module parameter "profile" (sg2002_video_clk.profile on
 * the kernel command line, or /sys/module/sg2002_video_clk/parameters/profile
 * at run time): "standard" or "fast". A clock's rate is changed only while
 * no consumer has it prepared, so a clock never changes under a running
 * block: at once when the parameter is written and the video is idle,
 * otherwise when a video driver calls sg2002_video_clk_apply() just before it
 * turns its clocks on for the next stream. The read-only parameter "rates"
 * gives the rate the clock framework has for each clock.
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/string.h>
#include <linux/sysfs.h>

#include "sg2002-video-clk.h"

enum {
	VCLK_CODEC,	/* vc_src0 */
	VCLK_VPSS,	/* src_vip_sys_1 */
	VCLK_BUS,	/* axi_video_codec */
	VCLK_NUM,
};

static const char * const vclk_names[VCLK_NUM] = {
	[VCLK_CODEC] = "vc-src0",
	[VCLK_VPSS] = "vip-sys1",
	[VCLK_BUS] = "axi-video-codec",
};

struct vclk_profile {
	const char *name;
	unsigned long rate[VCLK_NUM];
};

/*
 * Each rate is a whole divider of a PLL the clock can take (DISPPLL 1188 MHz,
 * MIPIMPLL 900, CAM1PLL 1080), so the mux's determine_rate finds it exactly
 * and picks that PLL: standard is the firmware's 396 (DISPPLL / 3), 300
 * (MIPIMPLL / 3) and 360 (CAM1PLL / 3); fast is NanoKVM-OS's VC overdrive,
 * 594 (DISPPLL / 2), 396 (DISPPLL / 3) and 450 (MIPIMPLL / 2).
 */
static const struct vclk_profile vclk_profiles[] = {
	{ "standard", { 396000000, 300000000, 360000000 } },
	{ "fast", { 594000000, 396000000, 450000000 } },
};

static DEFINE_MUTEX(vclk_lock);
/* Fast, as the device tree sets at boot. */
static unsigned int vclk_profile = 1;
static struct device *vclk_dev;
static struct clk_bulk_data vclk_clks[VCLK_NUM];

/*
 * Set each clock that nobody has prepared to the profile's rate. A clock in
 * use keeps its rate until a later call finds it idle. Called with vclk_lock.
 */
static void vclk_apply_locked(void)
{
	const struct vclk_profile *p = &vclk_profiles[vclk_profile];
	unsigned int i;

	if (!vclk_dev)
		return;

	for (i = 0; i < VCLK_NUM; i++) {
		struct clk *clk = vclk_clks[i].clk;
		unsigned long now = clk_get_rate(clk);
		int ret;

		if (now == p->rate[i] || clk_hw_is_prepared(__clk_get_hw(clk)))
			continue;
		ret = clk_set_rate(clk, p->rate[i]);
		if (ret) {
			dev_warn_ratelimited(vclk_dev, "%s: %lu Hz refused: %d\n",
					     vclk_names[i], p->rate[i], ret);
			continue;
		}
		dev_info(vclk_dev, "%s: %lu to %lu Hz (%s)\n", vclk_names[i],
			 now, clk_get_rate(clk), p->name);
	}
}

/**
 * sg2002_video_clk_apply - bring the idle video clocks to the chosen profile
 *
 * A video driver calls this just before it turns its clocks on for a stream,
 * so a profile chosen while the video ran takes effect at the next start.
 */
void sg2002_video_clk_apply(void)
{
	mutex_lock(&vclk_lock);
	vclk_apply_locked();
	mutex_unlock(&vclk_lock);
}
EXPORT_SYMBOL_GPL(sg2002_video_clk_apply);

static int profile_set(const char *val, const struct kernel_param *kp)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(vclk_profiles); i++) {
		if (sysfs_streq(val, vclk_profiles[i].name))
			break;
	}
	if (i == ARRAY_SIZE(vclk_profiles))
		return -EINVAL;

	mutex_lock(&vclk_lock);
	vclk_profile = i;
	vclk_apply_locked();
	mutex_unlock(&vclk_lock);
	return 0;
}

static int profile_get(char *buf, const struct kernel_param *kp)
{
	return sysfs_emit(buf, "%s\n", vclk_profiles[READ_ONCE(vclk_profile)].name);
}

static const struct kernel_param_ops profile_ops = {
	.set = profile_set,
	.get = profile_get,
};
module_param_cb(profile, &profile_ops, NULL, 0644);
MODULE_PARM_DESC(profile, "Video clocks: standard (396/300/360 MHz) or fast (594/396/450 MHz)");

static int rates_get(char *buf, const struct kernel_param *kp)
{
	unsigned int i;
	int len = 0;

	mutex_lock(&vclk_lock);
	for (i = 0; vclk_dev && i < VCLK_NUM; i++)
		len += sysfs_emit_at(buf, len, "%s%s=%lu", i ? " " : "",
				     vclk_names[i], clk_get_rate(vclk_clks[i].clk));
	mutex_unlock(&vclk_lock);
	len += sysfs_emit_at(buf, len, "\n");
	return len;
}

static const struct kernel_param_ops rates_ops = {
	.get = rates_get,
};
module_param_cb(rates, &rates_ops, NULL, 0444);
MODULE_PARM_DESC(rates, "The rate of each video clock, in Hz");

static int vclk_probe(struct platform_device *pdev)
{
	unsigned int i;
	int ret;

	for (i = 0; i < VCLK_NUM; i++)
		vclk_clks[i].id = vclk_names[i];
	ret = devm_clk_bulk_get(&pdev->dev, VCLK_NUM, vclk_clks);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "no video clocks\n");

	mutex_lock(&vclk_lock);
	vclk_dev = &pdev->dev;
	/* A profile from the command line, or none: the DT's rates stand. */
	vclk_apply_locked();
	mutex_unlock(&vclk_lock);
	return 0;
}

static const struct of_device_id vclk_of_match[] = {
	{ .compatible = "sophgo,sg2002-video-clock" },
	{ /* sentinel */ }
};

static struct platform_driver vclk_driver = {
	.probe = vclk_probe,
	.driver = {
		.name = "sg2002-video-clk",
		.of_match_table = vclk_of_match,
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(vclk_driver);
