// SPDX-License-Identifier: GPL-2.0-only
/*
 * Sophgo SG2002 codec SRAM arbiter
 *
 * The Coda980 (H.264) and WAVE420L (H.265) cores share an SRAM that the
 * share register in vc_ctrl hands to one of them. Their firmware boots only
 * with the SRAM switched to it; Sophgo's driver flips the register before it
 * runs either core. Here the two mainline drivers take turns instead: one
 * holds the SRAM while it streams, and the other is refused with -EBUSY.
 *
 * The arbiter does no register access of its own except in
 * sg2002_vc_sram_switch(), which the caller makes with its clocks on: vc_ctrl
 * sits behind the codec configuration clock, and on the SG2002 an access to
 * a clock-gated block stalls the bus.
 */

#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>

#include "sg2002-vc-sram.h"

static DEFINE_MUTEX(sram_lock);
/* The user streaming with the SRAM and how many of its contexts do. */
static enum sg2002_vc_sram_user sram_holder;
static unsigned int sram_refs;
/* The user the register was last switched to; 0 before the first switch. */
static enum sg2002_vc_sram_user sram_selected;

static const char *sram_name(enum sg2002_vc_sram_user user)
{
	switch (user) {
	case SG2002_VC_SRAM_H264:
		return "H.264";
	case SG2002_VC_SRAM_H265:
		return "H.265";
	}
	return "nobody";
}

/**
 * sg2002_vc_sram_acquire - hold the codec SRAM while streaming
 * @user: the codec that streams
 *
 * Return: -EBUSY when the other codec holds the SRAM. Otherwise 1 when the
 * register is not switched to @user (the other codec used the SRAM since, or
 * nobody switched it yet), so the caller must switch it and boot its
 * firmware again before it uses the core; 0 when it still is.
 */
int sg2002_vc_sram_acquire(enum sg2002_vc_sram_user user)
{
	int ret;

	mutex_lock(&sram_lock);
	if (sram_refs && sram_holder != user) {
		pr_info_ratelimited("sg2002-vc-sram: %s refused, %s is streaming\n",
				    sram_name(user), sram_name(sram_holder));
		ret = -EBUSY;
	} else {
		sram_holder = user;
		sram_refs++;
		ret = sram_selected != user;
	}
	mutex_unlock(&sram_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(sg2002_vc_sram_acquire);

/**
 * sg2002_vc_sram_release - end a hold taken with sg2002_vc_sram_acquire()
 * @user: the codec that stopped streaming
 *
 * The register keeps its setting, so a later acquire by the same codec does
 * not need a firmware boot.
 */
void sg2002_vc_sram_release(enum sg2002_vc_sram_user user)
{
	mutex_lock(&sram_lock);
	if (WARN_ON(!sram_refs || sram_holder != user)) {
		mutex_unlock(&sram_lock);
		return;
	}
	if (!--sram_refs)
		sram_holder = 0;
	mutex_unlock(&sram_lock);
}
EXPORT_SYMBOL_GPL(sg2002_vc_sram_release);

/**
 * sg2002_vc_sram_switch - give the SRAM to a codec for a firmware boot
 * @user: the codec about to boot its firmware
 * @vc_ctrl: the caller's mapping of the vc_ctrl block
 *
 * Switches the share register unless the other codec holds the SRAM. The
 * caller's clocks, which include the codec configuration clock, must be on.
 *
 * Return: 0 when the register is switched to @user, -EBUSY when the other
 * codec holds the SRAM and the register was left alone.
 */
int sg2002_vc_sram_switch(enum sg2002_vc_sram_user user, void __iomem *vc_ctrl)
{
	int ret = 0;

	mutex_lock(&sram_lock);
	if (sram_refs && sram_holder != user) {
		ret = -EBUSY;
	} else {
		writel(user, vc_ctrl + SG2002_VC_SRAM_SHARE);
		sram_selected = user;
	}
	mutex_unlock(&sram_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(sg2002_vc_sram_switch);

/**
 * sg2002_vc_sram_holder - the codec streaming with the SRAM, 0 for none
 */
enum sg2002_vc_sram_user sg2002_vc_sram_holder(void)
{
	enum sg2002_vc_sram_user user;

	mutex_lock(&sram_lock);
	user = sram_refs ? sram_holder : 0;
	mutex_unlock(&sram_lock);

	return user;
}
EXPORT_SYMBOL_GPL(sg2002_vc_sram_holder);

MODULE_DESCRIPTION("Sophgo SG2002 codec SRAM arbiter");
MODULE_LICENSE("GPL");
