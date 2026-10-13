/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Sophgo SG2002 video clock profile: the drivers of the blocks on the video
 * clocks call sg2002_video_clk_apply() just before they turn their clocks on.
 */
#ifndef SG2002_VIDEO_CLK_H
#define SG2002_VIDEO_CLK_H

#if IS_ENABLED(CONFIG_VIDEO_SOPHGO_SG2002_VIDEO_CLK)
void sg2002_video_clk_apply(void);
#else
static inline void sg2002_video_clk_apply(void) { }
#endif

#endif
