/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Sophgo SG2002 codec SRAM arbiter
 *
 * The H.264 (Coda980) and H.265 (WAVE420L) cores share one SRAM. The
 * share register at vc_ctrl + 0x24 gives it to one of them: 1 for H.264,
 * 2 for H.265. Each core's firmware boots only with the SRAM switched to it,
 * so the two encoders take turns: a driver acquires the SRAM for the time
 * it streams, and the other one gets -EBUSY until it is released.
 */
#ifndef _SG2002_VC_SRAM_H_
#define _SG2002_VC_SRAM_H_

#include <linux/errno.h>
#include <linux/io.h>
#include <linux/kconfig.h>
#include <linux/types.h>

#define SG2002_VC_SRAM_SHARE		0x24

enum sg2002_vc_sram_user {
	SG2002_VC_SRAM_H264 = 1,
	SG2002_VC_SRAM_H265 = 2,
};

#if IS_REACHABLE(CONFIG_VIDEO_SG2002_VC_SRAM)

int sg2002_vc_sram_acquire(enum sg2002_vc_sram_user user);
void sg2002_vc_sram_release(enum sg2002_vc_sram_user user);
int sg2002_vc_sram_switch(enum sg2002_vc_sram_user user, void __iomem *vc_ctrl);
enum sg2002_vc_sram_user sg2002_vc_sram_holder(void);

#else

static inline int sg2002_vc_sram_acquire(enum sg2002_vc_sram_user user)
{
	return 0;
}

static inline void sg2002_vc_sram_release(enum sg2002_vc_sram_user user)
{
}

static inline int sg2002_vc_sram_switch(enum sg2002_vc_sram_user user,
					void __iomem *vc_ctrl)
{
	if (vc_ctrl)
		writel(user, vc_ctrl + SG2002_VC_SRAM_SHARE);
	return 0;
}

static inline enum sg2002_vc_sram_user sg2002_vc_sram_holder(void)
{
	return 0;
}

#endif

#endif /* _SG2002_VC_SRAM_H_ */
