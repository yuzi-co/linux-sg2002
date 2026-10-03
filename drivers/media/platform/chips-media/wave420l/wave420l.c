// SPDX-License-Identifier: GPL-2.0-only
/*
 * Chips&Media WAVE420L H.265 encoder for the Sophgo SG2002
 *
 * A V4L2 stateful mem2mem encoder: NV12, NV21 or packed UYVY in, HEVC Main
 * profile Annex B out, one access unit per picture. The core codes packed
 * 4:2:2 input as 4:2:0 itself, so a captured frame can be encoded as it is,
 * without a conversion pass. The core runs Chips&Media's firmware
 * (monet.bin), which takes its commands through the WAVE4 host interface:
 * INIT_VPU boots it, CREATE_INSTANCE, SET_PARAM and SET_FRAMEBUF set up a
 * sequence, ENC_PIC encodes one picture and FINI_SEQ ends the sequence.
 *
 * The host interface and command layout follow Rockchip's GPL-2.0 kernel
 * driver for the same firmware interface (drivers/video/rockchip/vpu/
 * mpp_dev_h265e.c, Copyright (C) 2016 Fuzhou Rockchip Electronics Co., Ltd)
 * and the mainline wave5 driver, which shares the WAVE4 boot sequence. Where
 * the WAVE420L differs (the RC and sequence register layouts, the layer
 * period register that replaces the intra QP range), the register meanings
 * were taken as facts from Sophgo's documentation of its SDK; no Sophgo code
 * is used.
 *
 * The driver decides where IDR pictures go: the firmware runs with an
 * infinite intra period, and the driver asks for an IDR, with VPS, SPS and
 * PPS in front of it, at every GOP boundary and whenever user space asks for
 * a keyframe. GOP changes therefore take effect without a SET_PARAM, and
 * every keyframe carries its parameter sets.
 *
 * The H.264 (Coda980) and H.265 cores share an SRAM that vc_ctrl gives to
 * one of them, and this firmware boots only with it. The driver takes the
 * SRAM through the sg2002-vc-sram arbiter for the time it streams, so H.264
 * and H.265 encoding exclude each other; the second one to start gets
 * -EBUSY. The core is powered and its firmware booted at stream start, and
 * powered off at stream stop.
 *
 * Every register access happens with the core's clocks on and its reset
 * released: on the SG2002 a read of a clock-gated block stalls the bus.
 */

#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>

#include "../sg2002-vc-sram/sg2002-vc-sram.h"

#define W4_NAME				"wave420l"

/* WAVE4 host interface */
#define W4_PO_CONF			0x0000
#define W4_VCPU_CUR_PC			0x0004
#define W4_VPU_VINT_REASON_USR		0x0030
#define W4_VPU_VINT_REASON_CLR		0x0034
#define W4_VPU_HOST_INT_REQ		0x0038
#define W4_VPU_VINT_CLEAR		0x003c
#define W4_VPU_VPU_INT_STS		0x0044
#define W4_VPU_VINT_ENABLE		0x0048
#define W4_VPU_VINT_REASON		0x004c
#define W4_VPU_RESET_REQ		0x0050
#define W4_VPU_RESET_STATUS		0x0054
#define W4_VPU_REMAP_CTRL		0x0060
#define W4_VPU_REMAP_VADDR		0x0064
#define W4_VPU_REMAP_PADDR		0x0068
#define W4_VPU_REMAP_CORE_START		0x006c
#define W4_VPU_BUSY_STATUS		0x0070
#define W4_VPU_HALT_STATUS		0x0074
#define W4_PRODUCT_NUMBER		0x1044

/* Command registers, 0x100 to 0x1ff; several are reused per command. */
#define W4_CMD_REG_BASE			0x0100
#define W4_CMD_REG_END			0x0200
#define W4_COMMAND			0x0100
#define W4_CORE_INDEX			0x0104
#define W4_INST_INDEX			0x0108
#define W4_SET_PARAM_OPTION		0x010c	/* SET_PARAM */
#define W4_SFB_OPTION			0x010c	/* SET_FRAMEBUF */
#define W4_RET_SUCCESS			0x0110
#define W4_RET_FAIL_REASON		0x0114
#define W4_ADDR_CODE_BASE		0x0118	/* INIT_VPU */
#define W4_CODE_SIZE			0x011c
#define W4_CODE_PARAM			0x0120
#define W4_HW_OPTION			0x0124
#define W4_TIMEOUT_CNT			0x0134
#define W4_RET_FW_VERSION		0x0118	/* GET_FW_VERSION */
#define W4_RET_PRODUCT_NAME		0x011c
#define W4_RET_PRODUCT_VERSION		0x0120
#define W4_BS_START_ADDR		0x0120	/* bitstream, all encoder commands */
#define W4_COMMON_PIC_INFO		0x0120	/* SET_FRAMEBUF */
#define W4_BS_SIZE			0x0124
#define W4_PIC_SIZE			0x0124	/* SET_FRAMEBUF */
#define W4_BS_PARAM			0x0128
#define W4_SET_FB_NUM			0x0128	/* SET_FRAMEBUF */
#define W4_BS_OPTION			0x012c
#define W4_BS_RD_PTR			0x0130
#define W4_BS_WR_PTR			0x0134
#define W4_ADDR_WORK_BASE		0x0138
#define W4_WORK_SIZE			0x013c
#define W4_WORK_PARAM			0x0140
#define W4_ADDR_TEMP_BASE		0x0144
#define W4_TEMP_SIZE			0x0148
#define W4_TEMP_PARAM			0x014c
#define W4_ADDR_SEC_AXI			0x0150
#define W4_SEC_AXI_SIZE			0x0154
#define W4_FBC_STRIDE			0x0154	/* SET_FRAMEBUF */
#define W4_USE_SEC_AXI			0x0158
#define W4_ADDR_SUB_SAMPLED_FB_BASE	0x0158	/* SET_FRAMEBUF */
#define W4_SUB_SAMPLED_ONE_FB_SIZE	0x015c	/* SET_FRAMEBUF */

/* SET_PARAM, common parameters */
#define W4_SET_PARAM_ENABLE		0x015c
#define W4_SEQ_SRC_SIZE			0x0160
#define W4_SEQ_PIC_PARAM		0x016c
#define W4_SEQ_GOP_PARAM		0x0170
#define W4_SEQ_INTRA_PARAM		0x0174
#define W4_SEQ_CONF_WIN_TOP_BOT		0x0178
#define W4_SEQ_CONF_WIN_LEFT_RIGHT	0x017c
#define W4_SEQ_FRAME_RATE		0x0180
#define W4_SEQ_INDEPENDENT_SLICE	0x0184
#define W4_SEQ_DEPENDENT_SLICE		0x0188
#define W4_SEQ_INTRA_REFRESH		0x018c
#define W4_ENC_PARAM			0x0190
#define W4_SEQ_LAYER_PERIOD		0x0194
#define W4_RC_PARAM			0x0198
#define W4_RC_MIN_MAX_QP		0x019c
#define W4_RC_BIT_RATIO_LAYER_0_3	0x01a0
#define W4_RC_BIT_RATIO_LAYER_4_7	0x01a4
#define W4_NR_PARAM			0x01a8
#define W4_NR_WEIGHT			0x01ac
#define W4_NUM_UNITS_IN_TICK		0x01b0
#define W4_TIME_SCALE			0x01b4
#define W4_NUM_TICKS_POC_DIFF_ONE	0x01b8
#define W4_RC_TRANS_RATE		0x01bc
#define W4_RC_TARGET_RATE		0x01c0
#define W4_ROT_PARAM			0x01c4
#define W4_RET_ENC_MIN_FB_NUM		0x01cc
#define W4_RET_ENC_MIN_SRC_BUF_NUM	0x01d8

/* SET_FRAMEBUF, eight buffers per command */
#define W4_ADDR_LUMA_BASE(i)		(0x0160 + (i) * 16)
#define W4_ADDR_CB_BASE(i)		(0x0164 + (i) * 16)
#define W4_ADDR_FBC_Y_OFFSET(i)		(0x0168 + (i) * 16)
#define W4_ADDR_FBC_C_OFFSET(i)		(0x016c + (i) * 16)
#define W4_ADDR_MV_COL(i)		(0x01e0 + (i) * 4)

/* ENC_PIC */
#define W4_ENC_ADDR_REPORT_BASE		0x015c
#define W4_ENC_REPORT_SIZE		0x0160
#define W4_ENC_REPORT_PARAM		0x0164
#define W4_ENC_CODE_OPTION		0x0168
#define W4_ENC_PIC_PARAM		0x016c
#define W4_ENC_SRC_PIC_IDX		0x0170
#define W4_ENC_SRC_ADDR_Y		0x0174
#define W4_ENC_SRC_ADDR_U		0x0178
#define W4_ENC_SRC_ADDR_V		0x017c
#define W4_ENC_SRC_STRIDE		0x0180
#define W4_ENC_SRC_FORMAT		0x0184
#define W4_ENC_PREFIX_SEI_NAL_ADDR	0x0188
#define W4_ENC_PREFIX_SEI_INFO		0x018c
#define W4_ENC_SUFFIX_SEI_NAL_ADDR	0x0190
#define W4_ENC_SUFFIX_SEI_INFO		0x0194
#define W4_ENC_LONGTERM_PIC		0x0198
#define W4_ENC_SUB_FRAME_SYNC_CONFIG	0x019c
#define W4_ENC_CTU_OPT_PARAM		0x01a0
#define W4_ENC_ROI_ADDR_CTU_MAP		0x01a4
#define W4_ENC_CTU_MODE_MAP_ADDR	0x01a8
#define W4_ENC_CTU_QP_MAP_ADDR		0x01ac
#define W4_ENC_SRC_TIMESTAMP_LOW	0x01b0
#define W4_ENC_SRC_TIMESTAMP_HIGH	0x01b4

/* ENC_PIC results */
#define W4_RET_ENC_PIC_IDX		0x01a8
#define W4_RET_ENC_PIC_SLICE_NUM	0x01ac
#define W4_RET_ENC_PIC_SKIP		0x01b0
#define W4_RET_ENC_PIC_NUM_INTRA	0x01b4
#define W4_RET_ENC_PIC_NUM_MERGE	0x01b8
#define W4_RET_ENC_PIC_FLAG		0x01bc
#define W4_RET_ENC_PIC_NUM_SKIP		0x01c0
#define W4_RET_ENC_PIC_AVG_CU_QP	0x01c4
#define W4_RET_ENC_PIC_BYTE		0x01c8
#define W4_RET_ENC_GOP_PIC_IDX		0x01cc
#define W4_RET_ENC_PIC_POC		0x01d0
#define W4_RET_FRAME_CYCLE		0x01d4
#define W4_RET_ENC_USED_SRC_IDX		0x01d8
#define W4_RET_ENC_PIC_NUM		0x01dc
#define W4_RET_ENC_PIC_TYPE		0x01e0

/* Commands */
#define W4_CMD_INIT_VPU			0x0001
#define W4_CMD_SET_PARAM		0x0002
#define W4_CMD_FINI_SEQ			0x0004
#define W4_CMD_ENC_PIC			0x0008
#define W4_CMD_SET_FRAMEBUF		0x0010
#define W4_CMD_GET_FW_VERSION		0x0100
#define W4_CMD_CREATE_INSTANCE		0x4000

/* INST_INDEX: the instance in 15:0, the codec mode in 23:16 (1, HEVC encoder). */
#define W4_INST_HEVC_ENC		(1 << 16)

/* Interrupt reasons */
#define W4_INT_SET_PARAM		1
#define W4_INT_ENC_PIC			3
#define W4_INT_BSBUF_FULL		15

/* All block resets: VCPU, VCLK, MCLK and the per-core clock domains. */
#define W4_RST_BLOCK_ALL		0x7ffffff

/* Remap window 0: enable, bit 11 as every WAVE4 boot sets it, size in pages. */
#define W4_REMAP_ENABLE			BIT(31)
#define W4_REMAP_BIT11			BIT(11)
#define W4_REMAP_PAGES(size)		(((size) >> 12) & 0x1ff)

/* SET_PARAM option and enable mask */
#define W4_OPT_COMMON			0
#define W4_SET_PARAM_ALL		0xffffffff
#define W4_CHANGE_INTRA_PARAM		BIT(3)
#define W4_CHANGE_FRAME_RATE		BIT(6)
#define W4_CHANGE_RC_PARAM		BIT(12)
#define W4_CHANGE_RC_MIN_MAX_QP		BIT(13)
#define W4_CHANGE_NUM_UNITS_IN_TICK	BIT(18)
#define W4_CHANGE_TIME_SCALE		BIT(19)
#define W4_CHANGE_TRANS_RATE		BIT(21)
#define W4_CHANGE_TARGET_RATE		BIT(22)

/* SEQ_GOP_PARAM presets */
#define W4_GOP_PRESET_IPP		2

/* SEQ_INTRA_PARAM decoding refresh types */
#define W4_REFRESH_IDR			2

/* ENC_CODE_OPTION */
#define W4_CODE_IMPLICIT_HEADER		BIT(0)
#define W4_CODE_VCL			BIT(1)
#define W4_CODE_VPS			BIT(2)
#define W4_CODE_SPS			BIT(3)
#define W4_CODE_PPS			BIT(4)

/* ENC_PIC_PARAM */
#define W4_PIC_FORCE_QP			BIT(1)
#define W4_PIC_QP_I(qp)			((qp) << 2)
#define W4_PIC_QP_P(qp)			((qp) << 8)
#define W4_PIC_FORCE_TYPE		BIT(20)
#define W4_PIC_TYPE(t)			((t) << 21)
#define W4_FORCE_TYPE_IDR		3

/*
 * ENC_SRC_FORMAT: the source frame format in bits 2:0, 8 bits left justified,
 * endianness. Planar formats set interleaved chroma and the NV21 order; with
 * bit 2 (packed 4:2:2) the low bits give the byte order instead: 4 YUYV,
 * 5 YVYU, 6 UYVY, 7 VYUY.
 */
#define W4_SRC_NV21			BIT(0)
#define W4_SRC_CBCR_INTERLEAVE		BIT(1)
#define W4_SRC_PACKED_UYVY		6
#define W4_SRC_8BIT_LEFT		(BIT(2) << 3)
#define W4_SRC_ENDIAN(e)		((e) << 6)

/* COMMON_PIC_INFO for the reconstruction buffers */
#define W4_FB_LINEAR			BIT(28)

/*
 * The 128-bit little-endian layout of system memory, as the WAVE4 registers
 * want it: bitstream and source inverted (0xf), frame buffers as is (0).
 */
#define W4_ENDIAN_STREAM		0xf
#define W4_ENDIAN_SOURCE		0xf
#define W4_ENDIAN_FRAME			0x0

/* Secondary AXI (codec SRAM at its own address 0): IMD, RDO and LF lines. */
#define W4_SEC_AXI_ENC			(BIT(9) | BIT(11) | BIT(15))

#define WAVE420L_PRODUCT_CODE		0x4201
#define W4_CODE_BUF_SIZE		SZ_256K
#define W4_TEMP_BUF_SIZE		SZ_1M
#define W4_WORK_BUF_SIZE		SZ_128K
#define W4_MAX_FB			16
#define W4_MAX_SRC			8

#define W4_MIN_W			256
#define W4_MIN_H			128
#define W4_MAX_W			2688
#define W4_MAX_H			1536

#define W4_POLL_US			10
#define W4_RESET_TIMEOUT_US		(100 * USEC_PER_MSEC)
#define W4_CMD_TIMEOUT_US		(500 * USEC_PER_MSEC)

#define W4_DEF_BITRATE			4000000
#define W4_MAX_BITRATE			100000000
#define W4_DEF_GOP			30

static unsigned int timeout_ms = 1000;
module_param(timeout_ms, uint, 0644);
MODULE_PARM_DESC(timeout_ms, "Time a picture may take before it is aborted (default: 1000)");

static bool use_irq = true;
module_param(use_irq, bool, 0444);
MODULE_PARM_DESC(use_irq, "Finish pictures from the interrupt; 0 polls the busy flag (default: 1)");

static unsigned int debug;
module_param(debug, uint, 0644);
MODULE_PARM_DESC(debug, "1 logs every command, 2 also every picture (default: 0)");

static unsigned int enc_mode = 1;
module_param(enc_mode, uint, 0644);
MODULE_PARM_DESC(enc_mode,
		 "Firmware encoder preset at stream start: 1 recommended, 0 custom; 2 (boost) and 3 (fast) hang ENC_PIC with monet.bin 250505 (default: 1)");

static bool sec_axi;
module_param(sec_axi, bool, 0644);
MODULE_PARM_DESC(sec_axi, "Let the core keep line buffers in the codec SRAM (default: 0)");

static unsigned int init_buf_levelx8 = 1;
module_param(init_buf_levelx8, uint, 0644);
MODULE_PARM_DESC(init_buf_levelx8, "Rate control buffer level at start, in eighths; 1 gives keyframes enough bits (trial 13d) (default: 1)");

static bool cu_rc = true;
module_param(cu_rc, bool, 0644);
MODULE_PARM_DESC(cu_rc, "Rate control per CU as well as per picture (default: 1)");

static bool hvs_qp;
module_param(hvs_qp, bool, 0644);
MODULE_PARM_DESC(hvs_qp, "Adjust CU QPs for subjective quality; changes nothing without hvs_qp_scale (default: 0)");

/*
 * The HVS QP adjustment acts only with its scale enabled (RC_PARAM bit 3,
 * the scale in bits 6:4): hvs_qp alone gives byte-identical streams (trial
 * 13d). With a scale of 2 to 4 on scrolling text the background is cleaner
 * (SSIM 0.93 against 0.6 at 3000 kbit/s) but the glyphs lose 1 dB (trial
 * 76), so it stays off.
 */
static unsigned int hvs_qp_scale;
module_param(hvs_qp_scale, uint, 0644);
MODULE_PARM_DESC(hvs_qp_scale, "HVS QP scale 1 to 4 with hvs_qp, 0 off (default: 0)");

static unsigned int max_delta_qp = 10;
module_param(max_delta_qp, uint, 0644);
MODULE_PARM_DESC(max_delta_qp, "Largest QP change between pictures under rate control (default: 10)");

static unsigned int rc_init_qp = 30;
module_param(rc_init_qp, uint, 0644);
MODULE_PARM_DESC(rc_init_qp,
		 "QP of the first picture under rate control, 63 lets the firmware choose (it picks 51) (default: 30)");

static int intra_qp_offset;
module_param(intra_qp_offset, int, 0644);
MODULE_PARM_DESC(intra_qp_offset, "QP of I pictures relative to P pictures under rate control (default: 0)");

/*
 * The clocks, in the order Sophgo's codec driver enables them: the video
 * codec fabric shared with the Coda980 and the JPEG unit, then the H.264
 * core's and the H.265 core's. The firmware boots and takes every setup
 * command without the H.264 clocks, but ENC_PIC then never ends (trial 13).
 * The clock framework counts the shared ones.
 */
static const char * const w4_clk_names[] = {
	"vc-src0", "cfg-reg-vc", "axi-video-codec", "h264c", "apb-h264c", "h265c", "apb-h265c",
};

struct w4_fmt {
	u32 fourcc;
	u32 src_format;
	bool packed;	/* one plane of 4:2:2, two bytes a pixel */
};

static const struct w4_fmt w4_out_fmts[] = {
	{ V4L2_PIX_FMT_NV12, W4_SRC_CBCR_INTERLEAVE, false },
	{ V4L2_PIX_FMT_NV21, W4_SRC_CBCR_INTERLEAVE | W4_SRC_NV21, false },
	{ V4L2_PIX_FMT_UYVY, W4_SRC_PACKED_UYVY, true },
};

struct w4_buf {
	void *vaddr;	/* a cookie only, with DMA_ATTR_NO_KERNEL_MAPPING */
	dma_addr_t dma;
	size_t size;
	unsigned long attrs;
};

struct w4_q_data {
	const struct w4_fmt *fmt;	/* OUTPUT only */
	u32 width;
	u32 height;
	u32 bytesperline;
	u32 sizeimage;
	struct v4l2_rect crop;		/* OUTPUT: the picture encoded */
	u32 sequence;
};

/* The colorimetry user space gave the OUTPUT queue; CAPTURE reports it. */
struct w4_colour {
	u32 colorspace;
	u32 xfer_func;
	u32 ycbcr_enc;
	u32 quantization;
};

struct w4_params {
	u32 bitrate;			/* bit/s */
	bool rc_enable;
	u32 gop;			/* 0: the first picture is the only IDR */
	u32 min_qp;
	u32 max_qp;
	u32 i_qp;
	u32 p_qp;
	u32 vbv_delay;			/* ms */
	u32 level;			/* level * 10 */
	u32 fps_num;			/* frames per second as num / den */
	u32 fps_den;
};

struct w4_dev;

struct w4_ctx {
	struct v4l2_fh fh;
	struct w4_dev *w4;
	struct v4l2_ctrl_handler hdl;
	struct w4_q_data out;
	struct w4_q_data cap;
	struct w4_colour colour;
	struct w4_params params;

	/* lock guards pending and force_idr, which controls set and jobs take. */
	spinlock_t lock;
	u32 pending;			/* SET_PARAM changes for the next picture */
	bool force_idr;

	/* The sequence, between stream start and stop. */
	bool started;
	bool holds_sram;
	bool failed;			/* a picture timed out: no more jobs */
	u32 since_idr;			/* pictures since the last IDR */
	u32 src_idx;
	u32 min_src;
	u32 num_fb;
	struct w4_buf work;
	struct w4_buf fb[W4_MAX_FB];
	struct w4_buf mv;
	struct w4_buf fbc_y;
	struct w4_buf fbc_c;
	struct w4_buf subsampled;

	/* The picture being encoded; read by job completion. */
	bool job_idr;
	ktime_t job_start;
};

struct w4_dev {
	struct device *dev;
	struct v4l2_device v4l2_dev;
	struct video_device vfd;
	struct v4l2_m2m_dev *m2m_dev;
	struct mutex mutex;		/* ioctls and queues */

	void __iomem *base;
	void __iomem *vc_ctrl;
	struct clk_bulk_data clks[ARRAY_SIZE(w4_clk_names)];
	struct reset_control *rstc;
	int irq;

	/* The firmware image in the code buffer, and the shared temp buffer. */
	struct w4_buf code;
	struct w4_buf temp;
	size_t fw_size;
	u32 fw_version;

	/* The context that owns the core between stream start and stop. */
	struct w4_ctx *owner;

	/* hw_lock guards powered and the running job. */
	spinlock_t hw_lock;
	bool powered;
	struct w4_ctx *job_ctx;
	u32 job_reason;
	struct delayed_work timeout_work;
	struct work_struct poll_work;

	unsigned long missed_irqs;
	unsigned long pictures;
};

static inline struct w4_ctx *file_to_ctx(struct file *file)
{
	return container_of(file_to_v4l2_fh(file), struct w4_ctx, fh);
}

static inline u32 w4_read(struct w4_dev *w4, u32 reg)
{
	return readl(w4->base + reg);
}

static inline void w4_write(struct w4_dev *w4, u32 reg, u32 val)
{
	writel(val, w4->base + reg);
}

/* Buffers */

/*
 * A buffer the CPU reads or writes, cleared as dma_alloc_coherent() clears
 * every allocation; or, with cpu false, one only the VPU touches, with
 * DMA_ATTR_NO_KERNEL_MAPPING, which neither dma-direct nor a device pool
 * clears. The reconstructed frames are such buffers: the VPU
 * writes each before it reads it. Clearing their megabytes through the
 * pool's uncached mapping at every stream start delays the first picture.
 */
static int w4_alloc(struct w4_dev *w4, struct w4_buf *b, size_t size, bool cpu)
{
	b->size = PAGE_ALIGN(size);
	b->attrs = cpu ? 0 : DMA_ATTR_NO_KERNEL_MAPPING;
	b->vaddr = dma_alloc_attrs(w4->dev, b->size, &b->dma, GFP_KERNEL, b->attrs);
	if (!b->vaddr) {
		b->size = 0;
		return -ENOMEM;
	}
	return 0;
}

static void w4_free(struct w4_dev *w4, struct w4_buf *b)
{
	if (b->vaddr)
		dma_free_attrs(w4->dev, b->size, b->vaddr, b->dma, b->attrs);
	b->vaddr = NULL;
	b->size = 0;
}

/* Power and firmware */

static void w4_log_state(struct w4_dev *w4, const char *when)
{
	dev_info(w4->dev,
		 "%s: pc 0x%08x busy %u halt 0x%x int_sts %u reason 0x%08x usr 0x%08x ret %u fail 0x%08x\n",
		 when, w4_read(w4, W4_VCPU_CUR_PC), w4_read(w4, W4_VPU_BUSY_STATUS),
		 w4_read(w4, W4_VPU_HALT_STATUS), w4_read(w4, W4_VPU_VPU_INT_STS),
		 w4_read(w4, W4_VPU_VINT_REASON), w4_read(w4, W4_VPU_VINT_REASON_USR),
		 w4_read(w4, W4_RET_SUCCESS), w4_read(w4, W4_RET_FAIL_REASON));
}

/* Mask the sources and clear any latched reason, so the line goes low. */
static void w4_quiesce(struct w4_dev *w4)
{
	w4_write(w4, W4_VPU_VINT_ENABLE, 0);
	w4_write(w4, W4_VPU_VINT_REASON_CLR, w4_read(w4, W4_VPU_VINT_REASON));
	w4_write(w4, W4_VPU_VINT_CLEAR, 1);
	w4_write(w4, W4_VPU_VINT_REASON_USR, 0);
}

static int w4_wait_busy(struct w4_dev *w4, const char *what)
{
	u32 val;
	int ret;

	ret = readl_poll_timeout(w4->base + W4_VPU_BUSY_STATUS, val, !val,
				 W4_POLL_US, W4_CMD_TIMEOUT_US);
	if (ret) {
		dev_err(w4->dev, "%s timed out\n", what);
		w4_log_state(w4, what);
	}
	return ret;
}

static void w4_issue(struct w4_dev *w4, u32 cmd)
{
	w4_write(w4, W4_VPU_BUSY_STATUS, 1);
	w4_write(w4, W4_RET_SUCCESS, 0);
	w4_write(w4, W4_CORE_INDEX, 0);
	w4_write(w4, W4_INST_INDEX, W4_INST_HEVC_ENC | 0);
	w4_write(w4, W4_COMMAND, cmd);
	/* INIT_VPU is started by REMAP_CORE_START, every other one by this. */
	if (cmd != W4_CMD_INIT_VPU)
		w4_write(w4, W4_VPU_HOST_INT_REQ, 1);
}

/* Issue a command and poll for its end; for everything but ENC_PIC. */
static int w4_command(struct w4_dev *w4, u32 cmd, const char *what)
{
	int ret;

	if (debug)
		dev_info(w4->dev, "%s\n", what);
	w4_issue(w4, cmd);
	ret = w4_wait_busy(w4, what);
	if (ret)
		return ret;
	/* A reason the firmware latched for a polled command is not a job's. */
	w4_write(w4, W4_VPU_VINT_REASON_CLR, w4_read(w4, W4_VPU_VINT_REASON));
	w4_write(w4, W4_VPU_VINT_CLEAR, 1);
	if (!w4_read(w4, W4_RET_SUCCESS)) {
		dev_err(w4->dev, "%s failed, reason 0x%08x\n", what,
			w4_read(w4, W4_RET_FAIL_REASON));
		return -EIO;
	}
	return 0;
}

static int w4_load_firmware(struct w4_dev *w4)
{
	const struct firmware *fw;
	const char *name;
	int ret;

	if (w4->code.vaddr)
		return 0;

	if (of_property_read_string(w4->dev->of_node, "firmware-name", &name))
		return dev_err_probe(w4->dev, -EINVAL, "no firmware-name\n");

	ret = request_firmware(&fw, name, w4->dev);
	if (ret) {
		dev_err(w4->dev, "cannot load %s: %d\n", name, ret);
		return ret;
	}
	if (fw->size > W4_CODE_BUF_SIZE) {
		dev_err(w4->dev, "%s is %zu bytes, the code buffer %u\n", name, fw->size,
			W4_CODE_BUF_SIZE);
		ret = -EFBIG;
		goto out;
	}

	ret = w4_alloc(w4, &w4->code, W4_CODE_BUF_SIZE, true);
	if (ret)
		goto out;
	ret = w4_alloc(w4, &w4->temp, W4_TEMP_BUF_SIZE, true);
	if (ret) {
		w4_free(w4, &w4->code);
		goto out;
	}
	/* No byte swap: the WAVE series reads 128-bit little-endian words. */
	memcpy(w4->code.vaddr, fw->data, fw->size);
	w4->fw_size = fw->size;
	dev_info(w4->dev, "%s: %zu bytes\n", name, fw->size);
out:
	release_firmware(fw);
	return ret;
}

static int w4_boot(struct w4_dev *w4)
{
	u32 reg, name;
	int ret;

	w4_write(w4, W4_PO_CONF, 0);
	w4_write(w4, W4_VPU_RESET_REQ, W4_RST_BLOCK_ALL);
	ret = readl_poll_timeout(w4->base + W4_VPU_RESET_STATUS, reg, !reg,
				 W4_POLL_US, W4_RESET_TIMEOUT_US);
	w4_write(w4, W4_VPU_RESET_REQ, 0);
	if (ret) {
		dev_err(w4->dev, "block reset timed out, status 0x%08x\n", reg);
		return ret;
	}

	for (reg = W4_CMD_REG_BASE; reg < W4_CMD_REG_END; reg += 4)
		w4_write(w4, reg, 0);

	w4_write(w4, W4_VPU_REMAP_CTRL, W4_REMAP_ENABLE | W4_REMAP_BIT11 |
		 W4_REMAP_PAGES(w4->code.size));
	w4_write(w4, W4_VPU_REMAP_VADDR, 0);
	w4_write(w4, W4_VPU_REMAP_PADDR, lower_32_bits(w4->code.dma));
	w4_write(w4, W4_ADDR_CODE_BASE, lower_32_bits(w4->code.dma));
	w4_write(w4, W4_CODE_SIZE, w4->code.size);
	w4_write(w4, W4_CODE_PARAM, 0);
	w4_write(w4, W4_TIMEOUT_CNT, 0xffffffff);
	w4_write(w4, W4_HW_OPTION, 0);
	w4_write(w4, W4_VPU_VINT_ENABLE, BIT(W4_INT_ENC_PIC) | BIT(W4_INT_BSBUF_FULL));

	w4_issue(w4, W4_CMD_INIT_VPU);
	w4_write(w4, W4_VPU_REMAP_CORE_START, 1);
	ret = w4_wait_busy(w4, "INIT_VPU");
	if (ret)
		return ret;
	if (!w4_read(w4, W4_RET_SUCCESS)) {
		dev_err(w4->dev, "INIT_VPU failed, reason 0x%08x\n",
			w4_read(w4, W4_RET_FAIL_REASON));
		return -EIO;
	}

	ret = w4_command(w4, W4_CMD_GET_FW_VERSION, "GET_FW_VERSION");
	if (ret)
		return ret;
	if (!w4->fw_version) {
		name = w4_read(w4, W4_RET_PRODUCT_NAME);
		w4->fw_version = w4_read(w4, W4_RET_FW_VERSION);
		dev_info(w4->dev, "firmware revision %u, product \"%c%c%c%c\" 0x%x\n",
			 w4->fw_version, (name >> 24) & 0xff, (name >> 16) & 0xff,
			 (name >> 8) & 0xff, name & 0xff,
			 w4_read(w4, W4_RET_PRODUCT_VERSION));
	}
	return 0;
}

static int w4_power_on(struct w4_dev *w4)
{
	int ret;

	ret = clk_bulk_prepare_enable(ARRAY_SIZE(w4->clks), w4->clks);
	if (ret)
		return ret;

	/* The SG2002 reset controller has no pulse: assert, then deassert. */
	ret = reset_control_assert(w4->rstc);
	if (!ret) {
		usleep_range(10, 20);
		ret = reset_control_deassert(w4->rstc);
	}
	if (ret)
		goto err_clk;
	usleep_range(10, 20);

	/* First register access: clocks on, reset released. */
	w4_quiesce(w4);

	spin_lock_irq(&w4->hw_lock);
	w4->powered = true;
	spin_unlock_irq(&w4->hw_lock);
	if (w4->irq > 0)
		enable_irq(w4->irq);
	return 0;

err_clk:
	clk_bulk_disable_unprepare(ARRAY_SIZE(w4->clks), w4->clks);
	return ret;
}

static void w4_power_off(struct w4_dev *w4)
{
	if (w4->irq > 0)
		disable_irq(w4->irq);

	spin_lock_irq(&w4->hw_lock);
	w4_quiesce(w4);
	w4->powered = false;
	spin_unlock_irq(&w4->hw_lock);

	/* Stop the VCPU before its clocks go. */
	reset_control_assert(w4->rstc);
	clk_bulk_disable_unprepare(ARRAY_SIZE(w4->clks), w4->clks);
}

/* Sequence */

static u32 w4_enc_width(struct w4_ctx *ctx)
{
	return ALIGN(ctx->out.crop.width, 8);
}

static u32 w4_enc_height(struct w4_ctx *ctx)
{
	return ALIGN(ctx->out.crop.height, 8);
}

static void w4_write_buffers(struct w4_ctx *ctx)
{
	struct w4_dev *w4 = ctx->w4;
	u32 size = 0;

	w4_write(w4, W4_ADDR_WORK_BASE, ctx->work.dma);
	w4_write(w4, W4_WORK_SIZE, ctx->work.size);
	w4_write(w4, W4_WORK_PARAM, 0);
	w4_write(w4, W4_ADDR_TEMP_BASE, w4->temp.dma);
	w4_write(w4, W4_TEMP_SIZE, w4->temp.size);
	w4_write(w4, W4_TEMP_PARAM, 0);

	if (sec_axi) {
		u32 w64 = ALIGN(w4_enc_width(ctx), 64);

		/* IMD, then LF (5 luma and 3 chroma lines), then RDO. */
		size = ALIGN(w4_enc_width(ctx), 32) + w64 * 8 + (w64 >> 5) * 22 * 16;
	}
	w4_write(w4, W4_ADDR_SEC_AXI, 0);
	w4_write(w4, W4_SEC_AXI_SIZE, size);
	w4_write(w4, W4_USE_SEC_AXI, sec_axi ? W4_SEC_AXI_ENC : 0);
}

static u32 w4_rc_param(struct w4_ctx *ctx)
{
	struct w4_params *p = &ctx->params;

	return (p->rc_enable ? BIT(0) : 0) |
	       (p->rc_enable && cu_rc ? BIT(1) : 0) |	/* CU level RC */
	       (hvs_qp ? BIT(2) : 0) |
	       (hvs_qp && hvs_qp_scale ? BIT(3) | (min(hvs_qp_scale, 4U) << 4) : 0) |
	       (min(init_buf_levelx8, 8U) << 9) |
	       (min(rc_init_qp, 63U) << 14) |
	       (min(p->vbv_delay, 32767U) << 20);
}

static u32 w4_rc_min_max_qp(struct w4_ctx *ctx)
{
	struct w4_params *p = &ctx->params;

	return p->min_qp | (p->max_qp << 6) | (min(max_delta_qp, 51U) << 12) |
	       ((intra_qp_offset & 0xffff) << 18);
}

static u32 w4_frame_rate(struct w4_ctx *ctx)
{
	return max(1U, DIV_ROUND_CLOSEST(ctx->params.fps_num, ctx->params.fps_den));
}

static void w4_write_rate(struct w4_ctx *ctx)
{
	struct w4_dev *w4 = ctx->w4;
	struct w4_params *p = &ctx->params;

	w4_write(w4, W4_SEQ_FRAME_RATE, w4_frame_rate(ctx));
	/* VUI timing: a picture lasts num_units_in_tick / time_scale seconds. */
	w4_write(w4, W4_NUM_UNITS_IN_TICK, p->fps_den);
	w4_write(w4, W4_TIME_SCALE, p->fps_num);
	w4_write(w4, W4_RC_TARGET_RATE, p->rc_enable ? p->bitrate : 0);
	w4_write(w4, W4_RC_TRANS_RATE, p->rc_enable ? p->bitrate : 0);
}

static int w4_set_param(struct w4_ctx *ctx)
{
	struct w4_dev *w4 = ctx->w4;
	struct w4_params *p = &ctx->params;
	u32 w = w4_enc_width(ctx), h = w4_enc_height(ctx);
	u32 val;
	int ret;

	w4_write(w4, W4_BS_START_ADDR, 0);
	w4_write(w4, W4_BS_SIZE, 0);
	w4_write(w4, W4_BS_RD_PTR, 0);
	w4_write(w4, W4_BS_WR_PTR, 0);
	w4_write(w4, W4_BS_PARAM, W4_ENDIAN_STREAM);
	w4_write_buffers(ctx);

	w4_write(w4, W4_SET_PARAM_OPTION, W4_OPT_COMMON);
	w4_write(w4, W4_SET_PARAM_ENABLE, W4_SET_PARAM_ALL);
	w4_write(w4, W4_SEQ_SRC_SIZE, h << 16 | w);
	/* Main profile, the level, main tier, 8 bits, 4:2:0. */
	w4_write(w4, W4_SEQ_PIC_PARAM, 1 | (p->level << 3) | (8 << 14));
	w4_write(w4, W4_SEQ_GOP_PARAM, W4_GOP_PRESET_IPP);
	/* IDR type, the QP of a fixed-QP I picture, no intra period. */
	w4_write(w4, W4_SEQ_INTRA_PARAM, W4_REFRESH_IDR | (p->i_qp << 3));
	/* The coded size is the crop rounded up to 8: cut that off again. */
	w4_write(w4, W4_SEQ_CONF_WIN_TOP_BOT, (h - ctx->out.crop.height) << 16);
	w4_write(w4, W4_SEQ_CONF_WIN_LEFT_RIGHT, (w - ctx->out.crop.width) << 16);
	w4_write(w4, W4_SEQ_INDEPENDENT_SLICE, 0);
	w4_write(w4, W4_SEQ_DEPENDENT_SLICE, 0);
	w4_write(w4, W4_SEQ_INTRA_REFRESH, 0);

	if (enc_mode) {
		val = 1;
	} else {
		/*
		 * Custom: all CU sizes, TMVP, two merge candidates, deblocking
		 * across slices, SAO, intra CUs in P slices.
		 */
		val = (0x7 << 4) | BIT(7) | (2 << 9) | BIT(12) | BIT(13) | BIT(14) |
		      BIT(16) | BIT(26) | BIT(27);
	}
	w4_write(w4, W4_ENC_PARAM, val);
	w4_write(w4, W4_SEQ_LAYER_PERIOD, 0);
	w4_write(w4, W4_RC_PARAM, w4_rc_param(ctx));
	w4_write(w4, W4_RC_MIN_MAX_QP, w4_rc_min_max_qp(ctx));
	w4_write(w4, W4_RC_BIT_RATIO_LAYER_0_3, 0);
	w4_write(w4, W4_RC_BIT_RATIO_LAYER_4_7, 0);
	w4_write(w4, W4_NR_PARAM, 0);
	w4_write(w4, W4_NR_WEIGHT, 0);
	w4_write(w4, W4_NUM_TICKS_POC_DIFF_ONE, 0);
	w4_write(w4, W4_ROT_PARAM, 0);
	w4_write_rate(ctx);

	ret = w4_command(w4, W4_CMD_SET_PARAM, "SET_PARAM");
	if (ret)
		return ret;

	ctx->num_fb = w4_read(w4, W4_RET_ENC_MIN_FB_NUM);
	ctx->min_src = w4_read(w4, W4_RET_ENC_MIN_SRC_BUF_NUM);
	dev_info(w4->dev, "%ux%u: %u frame buffers, %u source buffers\n", w, h,
		ctx->num_fb, ctx->min_src);
	if (!ctx->num_fb || ctx->num_fb > W4_MAX_FB) {
		dev_err(w4->dev, "firmware asks for %u frame buffers\n", ctx->num_fb);
		return -EINVAL;
	}
	ctx->min_src = clamp(ctx->min_src, 1U, (u32)W4_MAX_SRC);
	return 0;
}

/* Apply runtime changes before the next picture. */
static int w4_change_param(struct w4_ctx *ctx, u32 mask)
{
	struct w4_dev *w4 = ctx->w4;
	int ret;

	w4_write(w4, W4_BS_PARAM, W4_ENDIAN_STREAM);
	w4_write_buffers(ctx);
	w4_write(w4, W4_ENC_ADDR_REPORT_BASE, 0);
	w4_write(w4, W4_ENC_REPORT_SIZE, 0);
	w4_write(w4, W4_ENC_REPORT_PARAM, 0);

	w4_write(w4, W4_SET_PARAM_OPTION, W4_OPT_COMMON);
	w4_write(w4, W4_SET_PARAM_ENABLE, mask);
	if (mask & W4_CHANGE_RC_PARAM)
		w4_write(w4, W4_RC_PARAM, w4_rc_param(ctx));
	if (mask & W4_CHANGE_RC_MIN_MAX_QP)
		w4_write(w4, W4_RC_MIN_MAX_QP, w4_rc_min_max_qp(ctx));
	if (mask & (W4_CHANGE_FRAME_RATE | W4_CHANGE_TARGET_RATE))
		w4_write_rate(ctx);

	ret = w4_command(w4, W4_CMD_SET_PARAM, "SET_PARAM (change)");
	if (ret)
		dev_err(w4->dev, "parameter change 0x%x refused\n", mask);
	return ret;
}

static void w4_free_fbs(struct w4_ctx *ctx)
{
	struct w4_dev *w4 = ctx->w4;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(ctx->fb); i++)
		w4_free(w4, &ctx->fb[i]);
	w4_free(w4, &ctx->mv);
	w4_free(w4, &ctx->fbc_y);
	w4_free(w4, &ctx->fbc_c);
	w4_free(w4, &ctx->subsampled);
}

/*
 * Reconstruction buffers in the compressed (FBC) layout, with their motion
 * vector, FBC offset table and quarter-size subsampled buffers. Sizes after
 * Rockchip's driver, each with a page of margin.
 */
static int w4_set_framebuf(struct w4_ctx *ctx)
{
	struct w4_dev *w4 = ctx->w4;
	u32 w = w4_enc_width(ctx), h = w4_enc_height(ctx);
	u32 stride = ALIGN(w, 32);
	u32 luma = stride * ALIGN(h, 16);
	u32 chroma = ALIGN(stride / 2, 16) * ALIGN(h, 16) / 2;
	u32 mv = ALIGN(DIV_ROUND_UP(w, 64) * DIV_ROUND_UP(h, 64) * 128, 16);
	u32 fbc_y = ALIGN(DIV_ROUND_UP(h, 16) * DIV_ROUND_UP(w, 256) * 128, 16);
	u32 fbc_c = ALIGN(DIV_ROUND_UP(h, 16) * DIV_ROUND_UP(w / 2, 256) * 128, 16);
	u32 sub = ALIGN(w / 4, 16) * ALIGN(h / 4, 8);
	u32 luma_stride = ALIGN(ALIGN(w, 16) * 4, 32);
	u32 chroma_stride = ALIGN(ALIGN(w / 2, 16) * 4, 32);
	unsigned int n = ctx->num_fb, i, first, cnt;
	int ret;

	ret = w4_alloc(w4, &ctx->mv, mv * n + SZ_4K, true);
	ret = ret ?: w4_alloc(w4, &ctx->fbc_y, fbc_y * n + SZ_4K, true);
	ret = ret ?: w4_alloc(w4, &ctx->fbc_c, fbc_c * n + SZ_4K, true);
	ret = ret ?: w4_alloc(w4, &ctx->subsampled, sub * n + SZ_4K, true);
	for (i = 0; !ret && i < n; i++)
		ret = w4_alloc(w4, &ctx->fb[i], luma + 2 * chroma + SZ_4K, false);
	if (ret) {
		dev_err(w4->dev, "cannot allocate %u frame buffers for %ux%u\n", n, w, h);
		return ret;
	}

	w4_write(w4, W4_ADDR_SUB_SAMPLED_FB_BASE, ctx->subsampled.dma);
	w4_write(w4, W4_SUB_SAMPLED_ONE_FB_SIZE, sub);
	w4_write(w4, W4_PIC_SIZE, w << 16 | h);
	w4_write(w4, W4_FBC_STRIDE, luma_stride << 16 | chroma_stride);
	w4_write(w4, W4_COMMON_PIC_INFO, stride);

	for (first = 0; first < n; first += cnt) {
		cnt = min(n - first, 8U);
		w4_write(w4, W4_SFB_OPTION, (W4_ENDIAN_FRAME << 16) |
			 (first + cnt == n ? BIT(4) : 0) | (!first ? BIT(3) : 0));
		w4_write(w4, W4_SET_FB_NUM, first << 8 | (first + cnt - 1));
		for (i = 0; i < cnt; i++) {
			struct w4_buf *fb = &ctx->fb[first + i];

			w4_write(w4, W4_ADDR_LUMA_BASE(i), fb->dma);
			w4_write(w4, W4_ADDR_CB_BASE(i), fb->dma + luma);
			w4_write(w4, W4_ADDR_FBC_Y_OFFSET(i),
				 ctx->fbc_y.dma + (first + i) * fbc_y);
			w4_write(w4, W4_ADDR_FBC_C_OFFSET(i),
				 ctx->fbc_c.dma + (first + i) * fbc_c);
			w4_write(w4, W4_ADDR_MV_COL(i), ctx->mv.dma + (first + i) * mv);
		}
		w4_write(w4, W4_ADDR_WORK_BASE, ctx->work.dma);
		w4_write(w4, W4_WORK_SIZE, ctx->work.size);
		w4_write(w4, W4_WORK_PARAM, 0);
		ret = w4_command(w4, W4_CMD_SET_FRAMEBUF, "SET_FRAMEBUF");
		if (ret)
			return ret;
	}
	return 0;
}

static void w4_stop_sequence(struct w4_ctx *ctx);

static int w4_start_sequence(struct w4_ctx *ctx)
{
	struct w4_dev *w4 = ctx->w4;
	int ret;

	if (w4->owner)
		return -EBUSY;

	ret = sg2002_vc_sram_acquire(SG2002_VC_SRAM_H265);
	if (ret < 0)
		return ret;
	ctx->holds_sram = true;
	w4->owner = ctx;

	ret = w4_load_firmware(w4);
	if (ret)
		goto err;

	ret = w4_power_on(w4);
	if (ret) {
		dev_err(w4->dev, "cannot power the core: %d\n", ret);
		goto err;
	}
	ctx->started = true;

	ret = sg2002_vc_sram_switch(SG2002_VC_SRAM_H265, w4->vc_ctrl);
	if (ret)
		goto err;
	ret = w4_boot(w4);
	if (ret)
		goto err;

	ret = w4_alloc(w4, &ctx->work, W4_WORK_BUF_SIZE, true);
	if (ret)
		goto err;
	w4_write(w4, W4_ADDR_WORK_BASE, ctx->work.dma);
	w4_write(w4, W4_WORK_SIZE, ctx->work.size);
	w4_write(w4, W4_WORK_PARAM, 0);
	ret = w4_command(w4, W4_CMD_CREATE_INSTANCE, "CREATE_INSTANCE");
	if (ret)
		goto err;

	ret = w4_set_param(ctx);
	if (ret)
		goto err;
	ret = w4_set_framebuf(ctx);
	if (ret)
		goto err;

	spin_lock_irq(&ctx->lock);
	ctx->pending = 0;
	ctx->force_idr = true;
	spin_unlock_irq(&ctx->lock);
	ctx->failed = false;
	ctx->since_idr = 0;
	ctx->src_idx = 0;
	dev_info(w4->dev, "encoding %ux%u at %u/%u fps, %s, %u frame buffers\n",
		 ctx->out.crop.width, ctx->out.crop.height, ctx->params.fps_num,
		 ctx->params.fps_den, ctx->params.rc_enable ? "rate control" : "fixed QP",
		 ctx->num_fb);
	return 0;

err:
	w4_stop_sequence(ctx);
	return ret;
}

static void w4_stop_sequence(struct w4_ctx *ctx)
{
	struct w4_dev *w4 = ctx->w4;

	if (ctx->started) {
		/* The instance only exists after CREATE_INSTANCE, but FINI_SEQ is harmless. */
		if (ctx->work.vaddr) {
			w4_write(w4, W4_ADDR_WORK_BASE, ctx->work.dma);
			w4_write(w4, W4_WORK_SIZE, ctx->work.size);
			w4_write(w4, W4_WORK_PARAM, 0);
			w4_command(w4, W4_CMD_FINI_SEQ, "FINI_SEQ");
		}
		w4_power_off(w4);
		ctx->started = false;
	}
	w4_free_fbs(ctx);
	w4_free(w4, &ctx->work);
	if (w4->owner == ctx)
		w4->owner = NULL;
	if (ctx->holds_sram) {
		sg2002_vc_sram_release(SG2002_VC_SRAM_H265);
		ctx->holds_sram = false;
	}
}

/* Pictures */

/*
 * The VPS the firmware writes has vps_extension_flag set with no extension
 * data behind it. Chrome's hardware HEVC decoder refuses such a VPS and
 * fails every picture (trial 13); ffmpeg ignores the flag. Clear it in
 * place: the flag becomes 0, the stop bit follows it, and the bytes the
 * extension took become trailing zero bytes, so the length stays.
 */
struct w4_bits {
	const u8 *buf;		/* the NAL, header included */
	size_t len;
	size_t byte;		/* next NAL byte */
	unsigned int zeros;	/* zero bytes before it, for emulation prevention */
	u8 cur;
	unsigned int left;	/* bits left in cur */
	size_t rbsp_bits;	/* RBSP bits read so far */
	bool overrun;
};

static unsigned int w4_bit(struct w4_bits *b)
{
	if (!b->left) {
		if (b->byte < b->len && b->zeros >= 2 && b->buf[b->byte] == 3) {
			b->byte++;
			b->zeros = 0;
		}
		if (b->byte >= b->len) {
			b->overrun = true;
			return 0;
		}
		b->cur = b->buf[b->byte];
		b->zeros = b->cur ? 0 : b->zeros + 1;
		b->byte++;
		b->left = 8;
	}
	b->left--;
	b->rbsp_bits++;
	return (b->cur >> b->left) & 1;
}

static u32 w4_bits(struct w4_bits *b, unsigned int n)
{
	u32 v = 0;

	while (n--)
		v = v << 1 | w4_bit(b);
	return v;
}

static u32 w4_ue(struct w4_bits *b)
{
	unsigned int zeros = 0;

	while (!w4_bit(b)) {
		if (b->overrun || ++zeros > 31) {
			b->overrun = true;
			return 0;
		}
	}
	return zeros ? (1U << zeros) - 1 + w4_bits(b, zeros) : 0;
}

static void w4_clear_vps_extension(u8 *nal, size_t len)
{
	struct w4_bits b = { .buf = nal, .len = len };
	unsigned int sub, i, n, max_layer_id, sets;
	unsigned int prof[8] = { 0 }, lev[8] = { 0 };
	size_t at;

	w4_bits(&b, 16);			/* NAL header */
	w4_bits(&b, 12);			/* id, base layer flags, max layers */
	sub = w4_bits(&b, 3);			/* vps_max_sub_layers_minus1 */
	w4_bits(&b, 17);			/* nesting flag, reserved 0xffff */
	w4_bits(&b, 32);			/* profile_tier_level: 88 bits ... */
	w4_bits(&b, 32);
	w4_bits(&b, 24);
	w4_bits(&b, 8);				/* ... and the level */
	for (i = 0; i < sub; i++) {
		prof[i] = w4_bit(&b);
		lev[i] = w4_bit(&b);
	}
	if (sub)
		for (i = sub; i < 8; i++)
			w4_bits(&b, 2);
	for (i = 0; i < sub; i++) {
		if (prof[i]) {
			w4_bits(&b, 32);
			w4_bits(&b, 32);
			w4_bits(&b, 24);
		}
		if (lev[i])
			w4_bits(&b, 8);
	}
	n = w4_bit(&b) ? sub + 1 : 1;		/* sub_layer_ordering_info_present */
	for (i = 0; i < n; i++) {
		w4_ue(&b);
		w4_ue(&b);
		w4_ue(&b);
	}
	max_layer_id = w4_bits(&b, 6);
	sets = w4_ue(&b);
	if (sets > 64)
		return;
	for (i = 0; i < sets; i++)
		w4_bits(&b, max_layer_id + 1);
	if (w4_bit(&b)) {			/* vps_timing_info_present_flag */
		w4_bits(&b, 32);
		w4_bits(&b, 32);
		if (w4_bit(&b))
			w4_ue(&b);
		if (w4_ue(&b))			/* HRD parameters: not handled */
			return;
	}
	if (b.overrun || !w4_bit(&b))		/* vps_extension_flag */
		return;

	/* The flag is the last bit read, in the NAL byte before b.byte. */
	at = b.byte - 1;
	if (b.left) {
		nal[at] &= (u8)(0xff << (b.left + 1));
		nal[at] |= (u8)(1 << (b.left - 1));
	} else {
		nal[at] &= 0xfe;
		if (at + 1 >= len)
			return;
		nal[++at] = 0x80;
	}
	memset(nal + at + 1, 0, len - at - 1);
}

/* Find the VPS at the start of a keyframe's buffer and clear its extension. */
static void w4_fix_vps(u8 *buf, size_t len)
{
	size_t i, start = 0, end;

	for (i = 0; i + 4 < len && i < 16; i++) {
		if (!buf[i] && !buf[i + 1] && buf[i + 2] == 1) {
			start = i + 3;
			break;
		}
	}
	if (!start || ((buf[start] >> 1) & 0x3f) != 32)
		return;
	for (end = start + 2; end + 3 < len && end < start + 256; end++)
		if (!buf[end] && !buf[end + 1] && (buf[end + 2] == 1 ||
		    (!buf[end + 2] && buf[end + 3] == 1)))
			break;
	w4_clear_vps_extension(buf + start, end - start);
}

static void w4_job_done(struct w4_dev *w4, struct w4_ctx *ctx, u32 reason, bool timed_out)
{
	struct vb2_v4l2_buffer *src, *dst;
	enum vb2_buffer_state state = VB2_BUF_STATE_DONE;
	u32 ok, fail = 0, bytes = 0, rd = 0, wr = 0, type = 0, recon = 0, qp = 0;
	u32 cycles = 0, cap, n_intra = 0, n_merge = 0, n_skip = 0;
	dma_addr_t start;
	s64 us;

	src = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	dst = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);
	start = vb2_dma_contig_plane_dma_addr(&dst->vb2_buf, 0);
	cap = vb2_plane_size(&dst->vb2_buf, 0);
	us = ktime_us_delta(ktime_get(), ctx->job_start);

	ok = !timed_out && w4_read(w4, W4_RET_SUCCESS);
	if (!timed_out) {
		fail = w4_read(w4, W4_RET_FAIL_REASON);
		bytes = w4_read(w4, W4_RET_ENC_PIC_BYTE);
		rd = w4_read(w4, W4_BS_RD_PTR);
		wr = w4_read(w4, W4_BS_WR_PTR);
		type = w4_read(w4, W4_RET_ENC_PIC_TYPE) & 0xffff;
		recon = w4_read(w4, W4_RET_ENC_PIC_IDX);
		qp = w4_read(w4, W4_RET_ENC_PIC_AVG_CU_QP);
		cycles = w4_read(w4, W4_RET_FRAME_CYCLE);
		if (debug >= 2) {
			/* 8x8 units of the picture coded intra, merged, skipped */
			n_intra = w4_read(w4, W4_RET_ENC_PIC_NUM_INTRA);
			n_merge = w4_read(w4, W4_RET_ENC_PIC_NUM_MERGE);
			n_skip = w4_read(w4, W4_RET_ENC_PIC_NUM_SKIP);
		}
		/* Drop the reasons the picture left in the firmware's own copy. */
		w4_write(w4, W4_VPU_VINT_REASON_USR, 0);
	}

	if (!ok || (reason & BIT(W4_INT_BSBUF_FULL))) {
		dev_err(w4->dev,
			"picture %u failed: %s%s, reason 0x%x, fail 0x%08x, %u bytes, rd 0x%x wr 0x%x of 0x%llx+%u, %lld us\n",
			ctx->out.sequence, timed_out ? "timeout" : "error",
			(reason & BIT(W4_INT_BSBUF_FULL)) ? " (bitstream buffer full)" : "",
			reason, fail, bytes, rd, wr, (u64)start, cap, us);
		if (timed_out) {
			/* The core may still be busy: no more pictures this sequence. */
			w4_log_state(w4, "timeout");
			ctx->failed = true;
		}
		state = VB2_BUF_STATE_ERROR;
		bytes = 0;
	} else if ((s32)recon < 0) {
		/* The firmware kept the picture; nothing was written. */
		dev_warn_ratelimited(w4->dev, "picture %u: no output (recon %d)\n",
				     ctx->out.sequence, (s32)recon);
		bytes = 0;
	} else if (rd != lower_32_bits(start) || bytes > cap) {
		dev_err(w4->dev, "picture %u: %u bytes at 0x%x, buffer 0x%llx+%u\n",
			ctx->out.sequence, bytes, rd, (u64)start, cap);
		state = VB2_BUF_STATE_ERROR;
		bytes = 0;
	}

	w4->pictures++;
	if (debug >= 2)
		dev_info(w4->dev,
			 "picture %u: type %u%s, %u bytes (wr-rd %u), qp %u, %u cycles, %lld us, intra %u merge %u skip %u\n",
			 ctx->out.sequence, type, ctx->job_idr ? " (IDR asked)" : "",
			 bytes, wr - rd, qp, cycles, us, n_intra, n_merge, n_skip);

	if (state == VB2_BUF_STATE_DONE && ctx->job_idr && bytes > 64) {
		u8 *vaddr = vb2_plane_vaddr(&dst->vb2_buf, 0);
		bool cached = dst->vb2_buf.vb2_queue->non_coherent_mem;

		if (vaddr) {
			/* A cached buffer is synced for the CPU only after this. */
			if (cached)
				dma_sync_single_for_cpu(w4->dev, start, 320, DMA_FROM_DEVICE);
			w4_fix_vps(vaddr, min(bytes, 320U));
			if (cached)
				dma_sync_single_for_device(w4->dev, start, 320, DMA_TO_DEVICE);
		}
	}
	vb2_set_plane_payload(&dst->vb2_buf, 0, bytes);
	v4l2_m2m_buf_copy_metadata(src, dst);
	dst->flags &= ~(V4L2_BUF_FLAG_KEYFRAME | V4L2_BUF_FLAG_PFRAME |
			V4L2_BUF_FLAG_BFRAME);
	if (state == VB2_BUF_STATE_DONE && bytes)
		dst->flags |= (ctx->job_idr && type == 0) ?
			      V4L2_BUF_FLAG_KEYFRAME : V4L2_BUF_FLAG_PFRAME;
	src->sequence = ctx->out.sequence++;
	dst->sequence = ctx->cap.sequence++;

	if (v4l2_m2m_is_last_draining_src_buf(ctx->fh.m2m_ctx, src)) {
		static const struct v4l2_event eos = { .type = V4L2_EVENT_EOS };

		dst->flags |= V4L2_BUF_FLAG_LAST;
		v4l2_event_queue_fh(&ctx->fh, &eos);
		v4l2_m2m_mark_stopped(ctx->fh.m2m_ctx);
	}

	v4l2_m2m_buf_done_and_job_finish(w4->m2m_dev, ctx->fh.m2m_ctx, state);
}

static struct w4_ctx *w4_claim(struct w4_dev *w4)
{
	struct w4_ctx *ctx = w4->job_ctx;

	w4->job_ctx = NULL;
	return ctx;
}

static void w4_fail_job(struct w4_dev *w4, struct w4_ctx *ctx)
{
	struct vb2_v4l2_buffer *src, *dst;

	src = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
	dst = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);
	if (dst) {
		vb2_set_plane_payload(&dst->vb2_buf, 0, 0);
		v4l2_m2m_buf_done(dst, VB2_BUF_STATE_ERROR);
	}
	if (src)
		v4l2_m2m_buf_done(src, VB2_BUF_STATE_ERROR);
	v4l2_m2m_job_finish(w4->m2m_dev, ctx->fh.m2m_ctx);
}

static void w4_device_run(void *priv)
{
	struct w4_ctx *ctx = priv;
	struct w4_dev *w4 = ctx->w4;
	struct w4_q_data *q = &ctx->out;
	struct vb2_v4l2_buffer *src, *dst;
	dma_addr_t y, uv, bs;
	u32 code, pic, bs_size, mask;
	unsigned long flags;
	bool idr;

	if (!ctx->started || ctx->failed || w4->owner != ctx) {
		w4_fail_job(w4, ctx);
		return;
	}

	spin_lock_irqsave(&ctx->lock, flags);
	mask = ctx->pending;
	ctx->pending = 0;
	idr = ctx->force_idr || (ctx->params.gop && ctx->since_idr >= ctx->params.gop);
	ctx->force_idr = false;
	spin_unlock_irqrestore(&ctx->lock, flags);

	if (mask && w4_change_param(ctx, mask)) {
		w4_fail_job(w4, ctx);
		return;
	}

	src = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	dst = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);
	if (q->fmt->packed) {
		/* One plane; the core does not use the chroma addresses. */
		y = vb2_dma_contig_plane_dma_addr(&src->vb2_buf, 0) +
		    q->crop.top * q->bytesperline + q->crop.left * 2;
		uv = y;
	} else {
		y = vb2_dma_contig_plane_dma_addr(&src->vb2_buf, 0) +
		    q->crop.top * q->bytesperline + q->crop.left;
		uv = vb2_dma_contig_plane_dma_addr(&src->vb2_buf, 0) +
		     q->bytesperline * q->height + q->crop.top / 2 * q->bytesperline +
		     q->crop.left;
	}
	bs = vb2_dma_contig_plane_dma_addr(&dst->vb2_buf, 0);
	bs_size = vb2_plane_size(&dst->vb2_buf, 0);

	ctx->since_idr = idr ? 1 : ctx->since_idr + 1;
	ctx->job_idr = idr;

	w4_write(w4, W4_BS_START_ADDR, bs);
	w4_write(w4, W4_BS_SIZE, bs_size);
	w4_write(w4, W4_BS_RD_PTR, bs);
	w4_write(w4, W4_BS_WR_PTR, bs);
	w4_write(w4, W4_BS_PARAM, W4_ENDIAN_STREAM);
	w4_write(w4, W4_BS_OPTION, 0);
	w4_write_buffers(ctx);
	w4_write(w4, W4_ENC_ADDR_REPORT_BASE, 0);
	w4_write(w4, W4_ENC_REPORT_SIZE, 0);
	w4_write(w4, W4_ENC_REPORT_PARAM, 0);

	/* VPS, SPS and PPS in front of every IDR, in the same buffer. */
	code = W4_CODE_VCL;
	if (idr)
		code |= W4_CODE_VPS | W4_CODE_SPS | W4_CODE_PPS;
	w4_write(w4, W4_ENC_CODE_OPTION, code);

	pic = 0;
	if (!ctx->params.rc_enable)
		pic |= W4_PIC_FORCE_QP | W4_PIC_QP_I(ctx->params.i_qp) |
		       W4_PIC_QP_P(ctx->params.p_qp);
	if (idr)
		pic |= W4_PIC_FORCE_TYPE | W4_PIC_TYPE(W4_FORCE_TYPE_IDR);
	w4_write(w4, W4_ENC_PIC_PARAM, pic);
	w4_write(w4, W4_ENC_SRC_PIC_IDX, ctx->src_idx);
	ctx->src_idx = (ctx->src_idx + 1) % ctx->min_src;

	w4_write(w4, W4_ENC_SRC_ADDR_Y, y);
	w4_write(w4, W4_ENC_SRC_ADDR_U, uv);
	w4_write(w4, W4_ENC_SRC_ADDR_V, uv);
	/*
	 * Interleaved chroma has the luma stride. Packed input has no chroma
	 * plane; Sophgo's library writes half the line as its chroma stride,
	 * as for separate planes, and so does this.
	 */
	w4_write(w4, W4_ENC_SRC_STRIDE, q->bytesperline << 16 |
		 (q->fmt->packed ? q->bytesperline / 2 : q->bytesperline));
	w4_write(w4, W4_ENC_SRC_FORMAT, q->fmt->src_format | W4_SRC_8BIT_LEFT |
		 W4_SRC_ENDIAN(W4_ENDIAN_SOURCE));
	w4_write(w4, W4_ENC_PREFIX_SEI_NAL_ADDR, 0);
	w4_write(w4, W4_ENC_PREFIX_SEI_INFO, 0);
	w4_write(w4, W4_ENC_SUFFIX_SEI_NAL_ADDR, 0);
	w4_write(w4, W4_ENC_SUFFIX_SEI_INFO, 0);
	w4_write(w4, W4_ENC_ROI_ADDR_CTU_MAP, 0);
	w4_write(w4, W4_ENC_CTU_MODE_MAP_ADDR, 0);
	w4_write(w4, W4_ENC_CTU_QP_MAP_ADDR, 0);
	w4_write(w4, W4_ENC_CTU_OPT_PARAM, 0);
	w4_write(w4, W4_ENC_SRC_TIMESTAMP_LOW, 0);
	w4_write(w4, W4_ENC_SRC_TIMESTAMP_HIGH, 0);
	w4_write(w4, W4_ENC_LONGTERM_PIC, 0);
	w4_write(w4, W4_ENC_SUB_FRAME_SYNC_CONFIG, 0);

	spin_lock_irqsave(&w4->hw_lock, flags);
	w4->job_ctx = ctx;
	w4->job_reason = 0;
	ctx->job_start = ktime_get();
	w4_issue(w4, W4_CMD_ENC_PIC);
	spin_unlock_irqrestore(&w4->hw_lock, flags);

	if (w4->irq > 0)
		mod_delayed_work(system_dfl_wq, &w4->timeout_work, msecs_to_jiffies(timeout_ms));
	else
		queue_work(system_highpri_wq, &w4->poll_work);
}

static irqreturn_t w4_irq(int irq, void *data)
{
	struct w4_dev *w4 = data;
	irqreturn_t ret = IRQ_NONE;
	u32 reason;

	spin_lock(&w4->hw_lock);
	if (w4->powered && w4_read(w4, W4_VPU_VPU_INT_STS)) {
		reason = w4_read(w4, W4_VPU_VINT_REASON);
		w4_write(w4, W4_VPU_VINT_REASON_CLR, reason);
		w4_write(w4, W4_VPU_VINT_CLEAR, 1);
		w4->job_reason |= reason;
		ret = (w4->job_ctx && (reason & (BIT(W4_INT_ENC_PIC) | BIT(W4_INT_BSBUF_FULL)))) ?
		      IRQ_WAKE_THREAD : IRQ_HANDLED;
	}
	spin_unlock(&w4->hw_lock);

	return ret;
}

static irqreturn_t w4_irq_thread(int irq, void *data)
{
	struct w4_dev *w4 = data;
	struct w4_ctx *ctx;
	u32 reason;

	spin_lock_irq(&w4->hw_lock);
	reason = w4->job_reason;
	ctx = w4_claim(w4);
	spin_unlock_irq(&w4->hw_lock);
	if (!ctx)
		return IRQ_HANDLED;

	cancel_delayed_work(&w4->timeout_work);
	w4_job_done(w4, ctx, reason, false);
	return IRQ_HANDLED;
}

static void w4_timeout_work(struct work_struct *work)
{
	struct w4_dev *w4 = container_of(to_delayed_work(work), struct w4_dev, timeout_work);
	struct w4_ctx *ctx = NULL;
	bool busy = true;
	u32 reason = 0;
	s64 left;

	spin_lock_irq(&w4->hw_lock);
	if (w4->job_ctx) {
		left = timeout_ms * 1000LL - ktime_us_delta(ktime_get(), w4->job_ctx->job_start);
		if (left > 0) {
			/* A newer job than the one this work was armed for. */
			mod_delayed_work(system_dfl_wq, &w4->timeout_work, usecs_to_jiffies(left) + 1);
		} else {
			busy = w4_read(w4, W4_VPU_BUSY_STATUS);
			reason = w4->job_reason | w4_read(w4, W4_VPU_VINT_REASON);
			ctx = w4_claim(w4);
		}
	}
	spin_unlock_irq(&w4->hw_lock);
	if (!ctx)
		return;

	if (!busy) {
		w4->missed_irqs++;
		dev_warn_ratelimited(w4->dev,
				     "picture done (reason 0x%x) without an interrupt, %lu so far\n",
				     reason, w4->missed_irqs);
		w4_job_done(w4, ctx, reason, false);
		return;
	}
	w4_job_done(w4, ctx, reason, true);
}

static void w4_poll_work(struct work_struct *work)
{
	struct w4_dev *w4 = container_of(work, struct w4_dev, poll_work);
	struct w4_ctx *ctx;
	u32 val, reason;
	int ret;

	ret = readl_poll_timeout(w4->base + W4_VPU_BUSY_STATUS, val, !val, 50,
				 timeout_ms * 1000);

	spin_lock_irq(&w4->hw_lock);
	reason = w4_read(w4, W4_VPU_VINT_REASON);
	w4_write(w4, W4_VPU_VINT_REASON_CLR, reason);
	w4_write(w4, W4_VPU_VINT_CLEAR, 1);
	ctx = w4_claim(w4);
	spin_unlock_irq(&w4->hw_lock);

	if (ctx)
		w4_job_done(w4, ctx, reason, ret);
}

static const struct v4l2_m2m_ops w4_m2m_ops = {
	.device_run = w4_device_run,
};

/* Formats */

static const struct w4_fmt *w4_find_fmt(u32 fourcc)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(w4_out_fmts); i++)
		if (w4_out_fmts[i].fourcc == fourcc)
			return &w4_out_fmts[i];
	return NULL;
}

static void w4_fill_out(struct v4l2_pix_format *pix)
{
	const struct w4_fmt *fmt = w4_find_fmt(pix->pixelformat);

	pix->width = clamp_t(u32, ALIGN(pix->width, 8), W4_MIN_W, W4_MAX_W);
	pix->height = clamp_t(u32, ALIGN(pix->height, 8), W4_MIN_H, W4_MAX_H);
	pix->field = V4L2_FIELD_NONE;
	if (fmt && fmt->packed) {
		pix->bytesperline = clamp_t(u32, ALIGN(pix->bytesperline, 16),
					    ALIGN(pix->width, 16) * 2,
					    ALIGN(W4_MAX_W, 16) * 4);
		/*
		 * Whole 16-line rows, as for the chroma of NV12 below. The
		 * capture buffers have them with ironkvm-dist patch 0921.
		 */
		pix->sizeimage = max_t(u32, pix->sizeimage,
				       pix->bytesperline * ALIGN(pix->height, 16));
	} else {
		pix->bytesperline = clamp_t(u32, ALIGN(pix->bytesperline, 16),
					    ALIGN(pix->width, 16), ALIGN(W4_MAX_W, 16) * 2);
		/* Room for the core to read whole 16-line rows of chroma at the bottom. */
		pix->sizeimage = max_t(u32, pix->sizeimage,
				       pix->bytesperline * pix->height +
				       pix->bytesperline * ALIGN(pix->height, 16) / 2);
	}
	if (!pix->colorspace)
		pix->colorspace = V4L2_COLORSPACE_REC709;
}

static u32 w4_cap_size(u32 width, u32 height)
{
	/* Half a byte per pixel is more than a fixed QP of 1 gives on screens. */
	return max_t(u32, PAGE_ALIGN(width * height / 2 + SZ_64K), SZ_512K);
}

static int w4_querycap(struct file *file, void *priv, struct v4l2_capability *cap)
{
	strscpy(cap->driver, W4_NAME, sizeof(cap->driver));
	strscpy(cap->card, "WAVE420L H.265 encoder", sizeof(cap->card));
	strscpy(cap->bus_info, "platform:" W4_NAME, sizeof(cap->bus_info));
	return 0;
}

static int w4_enum_fmt_out(struct file *file, void *priv, struct v4l2_fmtdesc *f)
{
	if (f->index >= ARRAY_SIZE(w4_out_fmts))
		return -EINVAL;
	f->pixelformat = w4_out_fmts[f->index].fourcc;
	return 0;
}

static int w4_enum_fmt_cap(struct file *file, void *priv, struct v4l2_fmtdesc *f)
{
	if (f->index)
		return -EINVAL;
	f->pixelformat = V4L2_PIX_FMT_HEVC;
	f->flags = V4L2_FMT_FLAG_COMPRESSED;
	return 0;
}

static int w4_enum_framesizes(struct file *file, void *priv, struct v4l2_frmsizeenum *fs)
{
	if (fs->index || (fs->pixel_format != V4L2_PIX_FMT_HEVC &&
			  !w4_find_fmt(fs->pixel_format)))
		return -EINVAL;
	fs->type = V4L2_FRMSIZE_TYPE_STEPWISE;
	fs->stepwise.min_width = W4_MIN_W;
	fs->stepwise.max_width = W4_MAX_W;
	fs->stepwise.step_width = 8;
	fs->stepwise.min_height = W4_MIN_H;
	fs->stepwise.max_height = W4_MAX_H;
	fs->stepwise.step_height = 8;
	return 0;
}

static void w4_put_colour(const struct w4_ctx *ctx, struct v4l2_pix_format *pix)
{
	pix->colorspace = ctx->colour.colorspace;
	pix->xfer_func = ctx->colour.xfer_func;
	pix->ycbcr_enc = ctx->colour.ycbcr_enc;
	pix->quantization = ctx->colour.quantization;
}

static int w4_g_fmt_out(struct file *file, void *priv, struct v4l2_format *f)
{
	struct w4_ctx *ctx = file_to_ctx(file);
	struct v4l2_pix_format *pix = &f->fmt.pix;

	memset(pix, 0, sizeof(*pix));
	pix->pixelformat = ctx->out.fmt->fourcc;
	pix->width = ctx->out.width;
	pix->height = ctx->out.height;
	pix->bytesperline = ctx->out.bytesperline;
	pix->sizeimage = ctx->out.sizeimage;
	pix->field = V4L2_FIELD_NONE;
	w4_put_colour(ctx, pix);
	return 0;
}

static int w4_g_fmt_cap(struct file *file, void *priv, struct v4l2_format *f)
{
	struct w4_ctx *ctx = file_to_ctx(file);
	struct v4l2_pix_format *pix = &f->fmt.pix;

	memset(pix, 0, sizeof(*pix));
	pix->pixelformat = V4L2_PIX_FMT_HEVC;
	pix->width = ctx->cap.width;
	pix->height = ctx->cap.height;
	pix->sizeimage = ctx->cap.sizeimage;
	pix->field = V4L2_FIELD_NONE;
	w4_put_colour(ctx, pix);
	return 0;
}

static int w4_try_fmt_out(struct file *file, void *priv, struct v4l2_format *f)
{
	struct v4l2_pix_format *pix = &f->fmt.pix;

	if (!w4_find_fmt(pix->pixelformat))
		pix->pixelformat = w4_out_fmts[0].fourcc;
	w4_fill_out(pix);
	return 0;
}

static int w4_try_fmt_cap(struct file *file, void *priv, struct v4l2_format *f)
{
	struct w4_ctx *ctx = file_to_ctx(file);
	struct v4l2_pix_format *pix = &f->fmt.pix;

	/* The picture size follows the OUTPUT queue. */
	pix->pixelformat = V4L2_PIX_FMT_HEVC;
	pix->width = ctx->cap.width;
	pix->height = ctx->cap.height;
	pix->bytesperline = 0;
	pix->field = V4L2_FIELD_NONE;
	w4_put_colour(ctx, pix);
	pix->sizeimage = max_t(u32, PAGE_ALIGN(pix->sizeimage),
			       w4_cap_size(pix->width, pix->height));
	return 0;
}

static int w4_s_fmt_out(struct file *file, void *priv, struct v4l2_format *f)
{
	struct w4_ctx *ctx = file_to_ctx(file);
	struct v4l2_pix_format *pix = &f->fmt.pix;
	struct vb2_queue *vq;

	vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx, f->type);
	if (vb2_is_busy(vq))
		return -EBUSY;
	vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx, V4L2_BUF_TYPE_VIDEO_CAPTURE);
	if (vb2_is_busy(vq))
		return -EBUSY;

	w4_try_fmt_out(file, priv, f);
	ctx->out.fmt = w4_find_fmt(pix->pixelformat);
	ctx->out.width = pix->width;
	ctx->out.height = pix->height;
	ctx->out.bytesperline = pix->bytesperline;
	ctx->out.sizeimage = pix->sizeimage;
	ctx->out.crop = (struct v4l2_rect){ 0, 0, pix->width, pix->height };
	ctx->colour.colorspace = pix->colorspace;
	ctx->colour.xfer_func = pix->xfer_func;
	ctx->colour.ycbcr_enc = pix->ycbcr_enc;
	ctx->colour.quantization = pix->quantization;

	ctx->cap.width = pix->width;
	ctx->cap.height = pix->height;
	ctx->cap.sizeimage = max(ctx->cap.sizeimage, w4_cap_size(pix->width, pix->height));
	return 0;
}

static int w4_s_fmt_cap(struct file *file, void *priv, struct v4l2_format *f)
{
	struct w4_ctx *ctx = file_to_ctx(file);
	struct vb2_queue *vq;

	vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx, f->type);
	if (vb2_is_busy(vq))
		return -EBUSY;

	w4_try_fmt_cap(file, priv, f);
	ctx->cap.sizeimage = f->fmt.pix.sizeimage;
	return 0;
}

static int w4_g_selection(struct file *file, void *priv, struct v4l2_selection *s)
{
	struct w4_ctx *ctx = file_to_ctx(file);

	if (s->type != V4L2_BUF_TYPE_VIDEO_OUTPUT)
		return -EINVAL;

	switch (s->target) {
	case V4L2_SEL_TGT_CROP:
		s->r = ctx->out.crop;
		return 0;
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		s->r = (struct v4l2_rect){ 0, 0, ctx->out.width, ctx->out.height };
		return 0;
	}
	return -EINVAL;
}

static int w4_s_selection(struct file *file, void *priv, struct v4l2_selection *s)
{
	struct w4_ctx *ctx = file_to_ctx(file);
	struct v4l2_rect r = s->r;

	if (s->type != V4L2_BUF_TYPE_VIDEO_OUTPUT || s->target != V4L2_SEL_TGT_CROP)
		return -EINVAL;
	if (vb2_is_streaming(v4l2_m2m_get_vq(ctx->fh.m2m_ctx, s->type)))
		return -EBUSY;

	/* Even sizes and offsets, inside the surface, no smaller than the minimum. */
	r.left = clamp_t(s32, ALIGN_DOWN(r.left, 2), 0, ctx->out.width - W4_MIN_W);
	r.top = clamp_t(s32, ALIGN_DOWN(r.top, 2), 0, ctx->out.height - W4_MIN_H);
	r.width = clamp_t(u32, ALIGN(r.width, 2), W4_MIN_W, ctx->out.width - r.left);
	r.height = clamp_t(u32, ALIGN(r.height, 2), W4_MIN_H, ctx->out.height - r.top);
	/* The coded picture is the crop rounded up to 8; it must fit the surface. */
	if (r.left + ALIGN(r.width, 8) > ctx->out.width)
		r.left = ALIGN_DOWN(ctx->out.width - ALIGN(r.width, 8), 2);
	if (r.top + ALIGN(r.height, 8) > ctx->out.height)
		r.top = ALIGN_DOWN(ctx->out.height - ALIGN(r.height, 8), 2);

	ctx->out.crop = r;
	ctx->cap.width = r.width;
	ctx->cap.height = r.height;
	s->r = r;
	return 0;
}

static int w4_g_parm(struct file *file, void *priv, struct v4l2_streamparm *a)
{
	struct w4_ctx *ctx = file_to_ctx(file);

	if (a->type != V4L2_BUF_TYPE_VIDEO_OUTPUT)
		return -EINVAL;
	a->parm.output.capability = V4L2_CAP_TIMEPERFRAME;
	a->parm.output.timeperframe.numerator = ctx->params.fps_den;
	a->parm.output.timeperframe.denominator = ctx->params.fps_num;
	return 0;
}

static int w4_s_parm(struct file *file, void *priv, struct v4l2_streamparm *a)
{
	struct w4_ctx *ctx = file_to_ctx(file);
	struct v4l2_fract *tpf = &a->parm.output.timeperframe;

	if (a->type != V4L2_BUF_TYPE_VIDEO_OUTPUT)
		return -EINVAL;
	if (!tpf->numerator || !tpf->denominator) {
		tpf->numerator = 1;
		tpf->denominator = 30;
	}
	/* 1 to 240 frames per second, with a numerator below 65536. */
	if (tpf->numerator > 65535) {
		tpf->denominator = DIV_ROUND_CLOSEST_ULL((u64)tpf->denominator * 65535,
						       tpf->numerator);
		tpf->numerator = 65535;
	}
	tpf->denominator = clamp(tpf->denominator, tpf->numerator, tpf->numerator * 240);

	spin_lock_irq(&ctx->lock);
	if (ctx->params.fps_num != tpf->denominator || ctx->params.fps_den != tpf->numerator) {
		ctx->params.fps_num = tpf->denominator;
		ctx->params.fps_den = tpf->numerator;
		ctx->pending |= W4_CHANGE_FRAME_RATE | W4_CHANGE_NUM_UNITS_IN_TICK |
				W4_CHANGE_TIME_SCALE | W4_CHANGE_TARGET_RATE |
				W4_CHANGE_TRANS_RATE;
	}
	spin_unlock_irq(&ctx->lock);

	a->parm.output.capability = V4L2_CAP_TIMEPERFRAME;
	return 0;
}

static int w4_encoder_cmd(struct file *file, void *priv, struct v4l2_encoder_cmd *ec)
{
	struct w4_ctx *ctx = file_to_ctx(file);
	static const struct v4l2_event eos = { .type = V4L2_EVENT_EOS };
	int ret;

	ret = v4l2_m2m_ioctl_try_encoder_cmd(file, priv, ec);
	if (ret < 0)
		return ret;

	if (!vb2_is_streaming(v4l2_m2m_get_src_vq(ctx->fh.m2m_ctx)) ||
	    !vb2_is_streaming(v4l2_m2m_get_dst_vq(ctx->fh.m2m_ctx)))
		return 0;

	ret = v4l2_m2m_ioctl_encoder_cmd(file, priv, ec);
	if (ret < 0)
		return ret;

	if (ec->cmd == V4L2_ENC_CMD_STOP && v4l2_m2m_has_stopped(ctx->fh.m2m_ctx))
		v4l2_event_queue_fh(&ctx->fh, &eos);
	if (ec->cmd == V4L2_ENC_CMD_START)
		vb2_clear_last_buffer_dequeued(&ctx->fh.m2m_ctx->cap_q_ctx.q);
	return 0;
}

static int w4_subscribe_event(struct v4l2_fh *fh, const struct v4l2_event_subscription *sub)
{
	switch (sub->type) {
	case V4L2_EVENT_EOS:
		return v4l2_event_subscribe(fh, sub, 0, NULL);
	case V4L2_EVENT_CTRL:
		return v4l2_ctrl_subscribe_event(fh, sub);
	}
	return -EINVAL;
}

static const struct v4l2_ioctl_ops w4_ioctl_ops = {
	.vidioc_querycap		= w4_querycap,
	.vidioc_enum_fmt_vid_cap	= w4_enum_fmt_cap,
	.vidioc_enum_fmt_vid_out	= w4_enum_fmt_out,
	.vidioc_enum_framesizes		= w4_enum_framesizes,
	.vidioc_g_fmt_vid_cap		= w4_g_fmt_cap,
	.vidioc_g_fmt_vid_out		= w4_g_fmt_out,
	.vidioc_try_fmt_vid_cap		= w4_try_fmt_cap,
	.vidioc_try_fmt_vid_out		= w4_try_fmt_out,
	.vidioc_s_fmt_vid_cap		= w4_s_fmt_cap,
	.vidioc_s_fmt_vid_out		= w4_s_fmt_out,
	.vidioc_g_selection		= w4_g_selection,
	.vidioc_s_selection		= w4_s_selection,
	.vidioc_g_parm			= w4_g_parm,
	.vidioc_s_parm			= w4_s_parm,

	.vidioc_reqbufs			= v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf		= v4l2_m2m_ioctl_querybuf,
	.vidioc_qbuf			= v4l2_m2m_ioctl_qbuf,
	.vidioc_dqbuf			= v4l2_m2m_ioctl_dqbuf,
	.vidioc_prepare_buf		= v4l2_m2m_ioctl_prepare_buf,
	.vidioc_create_bufs		= v4l2_m2m_ioctl_create_bufs,
	.vidioc_expbuf			= v4l2_m2m_ioctl_expbuf,
	.vidioc_streamon		= v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff		= v4l2_m2m_ioctl_streamoff,

	.vidioc_try_encoder_cmd		= v4l2_m2m_ioctl_try_encoder_cmd,
	.vidioc_encoder_cmd		= w4_encoder_cmd,

	.vidioc_subscribe_event		= w4_subscribe_event,
	.vidioc_unsubscribe_event	= v4l2_event_unsubscribe,
};

/* Queues */

static int w4_queue_setup(struct vb2_queue *vq, unsigned int *nbuffers,
			  unsigned int *nplanes, unsigned int sizes[],
			  struct device *alloc_devs[])
{
	struct w4_ctx *ctx = vb2_get_drv_priv(vq);
	u32 size = V4L2_TYPE_IS_OUTPUT(vq->type) ? ctx->out.sizeimage : ctx->cap.sizeimage;

	if (*nplanes)
		return (*nplanes != 1 || sizes[0] < size) ? -EINVAL : 0;
	*nplanes = 1;
	sizes[0] = size;
	return 0;
}

static int w4_buf_prepare(struct vb2_buffer *vb)
{
	struct w4_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	bool out = V4L2_TYPE_IS_OUTPUT(vb->vb2_queue->type);
	u32 size = out ? ctx->out.sizeimage : ctx->cap.sizeimage;

	if (vb2_plane_size(vb, 0) < size)
		return -EINVAL;
	if (out) {
		vbuf->field = V4L2_FIELD_NONE;
		if (!vb2_get_plane_payload(vb, 0))
			vb2_set_plane_payload(vb, 0, size);
	} else {
		vb2_set_plane_payload(vb, 0, 0);
	}
	return 0;
}

static void w4_buf_queue(struct vb2_buffer *vb)
{
	struct w4_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);

	if (V4L2_TYPE_IS_CAPTURE(vb->vb2_queue->type) &&
	    vb2_is_streaming(vb->vb2_queue) &&
	    v4l2_m2m_dst_buf_is_last(ctx->fh.m2m_ctx)) {
		static const struct v4l2_event eos = { .type = V4L2_EVENT_EOS };

		vb2_set_plane_payload(vb, 0, 0);
		vbuf->field = V4L2_FIELD_NONE;
		vbuf->sequence = ctx->cap.sequence++;
		v4l2_m2m_last_buffer_done(ctx->fh.m2m_ctx, vbuf);
		v4l2_event_queue_fh(&ctx->fh, &eos);
		return;
	}
	v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, vbuf);
}

static void w4_return_bufs(struct w4_ctx *ctx, struct vb2_queue *vq,
			   enum vb2_buffer_state state)
{
	struct vb2_v4l2_buffer *vbuf;

	for (;;) {
		if (V4L2_TYPE_IS_OUTPUT(vq->type))
			vbuf = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
		else
			vbuf = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);
		if (!vbuf)
			return;
		v4l2_m2m_buf_done(vbuf, state);
	}
}

static int w4_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct w4_ctx *ctx = vb2_get_drv_priv(vq);
	struct vb2_queue *other;
	int ret;

	v4l2_m2m_update_start_streaming_state(ctx->fh.m2m_ctx, vq);
	if (V4L2_TYPE_IS_OUTPUT(vq->type)) {
		ctx->out.sequence = 0;
		other = v4l2_m2m_get_dst_vq(ctx->fh.m2m_ctx);
	} else {
		ctx->cap.sequence = 0;
		other = v4l2_m2m_get_src_vq(ctx->fh.m2m_ctx);
	}

	/* The sequence starts when both queues stream. */
	if (!vb2_is_streaming(other) || ctx->started)
		return 0;

	ret = w4_start_sequence(ctx);
	if (ret)
		w4_return_bufs(ctx, vq, VB2_BUF_STATE_QUEUED);
	return ret;
}

static void w4_stop_streaming(struct vb2_queue *vq)
{
	struct w4_ctx *ctx = vb2_get_drv_priv(vq);

	/* v4l2_m2m_streamoff() waited for a running job before calling this. */
	w4_return_bufs(ctx, vq, VB2_BUF_STATE_ERROR);
	v4l2_m2m_update_stop_streaming_state(ctx->fh.m2m_ctx, vq);
	if (V4L2_TYPE_IS_OUTPUT(vq->type) && v4l2_m2m_has_stopped(ctx->fh.m2m_ctx)) {
		static const struct v4l2_event eos = { .type = V4L2_EVENT_EOS };

		v4l2_event_queue_fh(&ctx->fh, &eos);
	}

	if (ctx->started || ctx->holds_sram)
		w4_stop_sequence(ctx);
}

static const struct vb2_ops w4_qops = {
	.queue_setup		= w4_queue_setup,
	.buf_prepare		= w4_buf_prepare,
	.buf_queue		= w4_buf_queue,
	.start_streaming	= w4_start_streaming,
	.stop_streaming		= w4_stop_streaming,
};

static int w4_queue_init(void *priv, struct vb2_queue *src_vq, struct vb2_queue *dst_vq)
{
	struct w4_ctx *ctx = priv;
	int ret;

	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	src_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	src_vq->drv_priv = ctx;
	src_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	src_vq->ops = &w4_qops;
	src_vq->mem_ops = &vb2_dma_contig_memops;
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->lock = &ctx->w4->mutex;
	src_vq->dev = ctx->w4->dev;
	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	dst_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	/* User space copies the stream out; let it ask for a cached buffer. */
	dst_vq->allow_cache_hints = 1;
	dst_vq->drv_priv = ctx;
	dst_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	dst_vq->ops = &w4_qops;
	dst_vq->mem_ops = &vb2_dma_contig_memops;
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->lock = &ctx->w4->mutex;
	dst_vq->dev = ctx->w4->dev;
	return vb2_queue_init(dst_vq);
}

/* Controls */

static int w4_s_ctrl_locked(struct w4_ctx *ctx, struct v4l2_ctrl *ctrl)
{
	struct w4_params *p = &ctx->params;

	switch (ctrl->id) {
	case V4L2_CID_MPEG_VIDEO_BITRATE:
		p->bitrate = ctrl->val;
		ctx->pending |= W4_CHANGE_TARGET_RATE | W4_CHANGE_TRANS_RATE;
		return 0;
	case V4L2_CID_MPEG_VIDEO_FRAME_RC_ENABLE:
		/* Rate control on or off needs a new sequence. */
		p->rc_enable = ctrl->val;
		return 0;
	case V4L2_CID_MPEG_VIDEO_BITRATE_MODE:
	case V4L2_CID_MPEG_VIDEO_HEADER_MODE:
	case V4L2_CID_MPEG_VIDEO_PREPEND_SPSPPS_TO_IDR:
	case V4L2_CID_MIN_BUFFERS_FOR_OUTPUT:
	case V4L2_CID_MPEG_VIDEO_HEVC_PROFILE:
	case V4L2_CID_MPEG_VIDEO_HEVC_TIER:
		return 0;
	case V4L2_CID_MPEG_VIDEO_GOP_SIZE:
		/* The driver places the IDRs: no SET_PARAM needed. */
		p->gop = ctrl->val;
		return 0;
	case V4L2_CID_MPEG_VIDEO_FORCE_KEY_FRAME:
		ctx->force_idr = true;
		return 0;
	case V4L2_CID_MPEG_VIDEO_HEVC_MIN_QP:
		p->min_qp = ctrl->val;
		ctx->pending |= W4_CHANGE_RC_MIN_MAX_QP;
		return 0;
	case V4L2_CID_MPEG_VIDEO_HEVC_MAX_QP:
		p->max_qp = ctrl->val;
		ctx->pending |= W4_CHANGE_RC_MIN_MAX_QP;
		return 0;
	case V4L2_CID_MPEG_VIDEO_HEVC_I_FRAME_QP:
		p->i_qp = ctrl->val;
		return 0;
	case V4L2_CID_MPEG_VIDEO_HEVC_P_FRAME_QP:
		p->p_qp = ctrl->val;
		return 0;
	case V4L2_CID_MPEG_VIDEO_VBV_DELAY:
		p->vbv_delay = ctrl->val;
		ctx->pending |= W4_CHANGE_RC_PARAM;
		return 0;
	case V4L2_CID_MPEG_VIDEO_HEVC_LEVEL:
		/* HEVC level n.m, as the firmware takes it: n * 10 + m. */
		switch (ctrl->val) {
		case V4L2_MPEG_VIDEO_HEVC_LEVEL_1:	p->level = 10; break;
		case V4L2_MPEG_VIDEO_HEVC_LEVEL_2:	p->level = 20; break;
		case V4L2_MPEG_VIDEO_HEVC_LEVEL_2_1:	p->level = 21; break;
		case V4L2_MPEG_VIDEO_HEVC_LEVEL_3:	p->level = 30; break;
		case V4L2_MPEG_VIDEO_HEVC_LEVEL_3_1:	p->level = 31; break;
		case V4L2_MPEG_VIDEO_HEVC_LEVEL_4:	p->level = 40; break;
		case V4L2_MPEG_VIDEO_HEVC_LEVEL_4_1:	p->level = 41; break;
		case V4L2_MPEG_VIDEO_HEVC_LEVEL_5:	p->level = 50; break;
		default:				p->level = 51; break;
		}
		return 0;
	}
	return -EINVAL;
}

static int w4_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct w4_ctx *ctx = container_of(ctrl->handler, struct w4_ctx, hdl);
	int ret;

	spin_lock_irq(&ctx->lock);
	ret = w4_s_ctrl_locked(ctx, ctrl);
	spin_unlock_irq(&ctx->lock);
	return ret;
}

static const struct v4l2_ctrl_ops w4_ctrl_ops = {
	.s_ctrl = w4_s_ctrl,
};

static int w4_ctrls_setup(struct w4_ctx *ctx)
{
	struct v4l2_ctrl_handler *hdl = &ctx->hdl;
	const struct v4l2_ctrl_ops *ops = &w4_ctrl_ops;

	v4l2_ctrl_handler_init(hdl, 16);
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_MPEG_VIDEO_BITRATE, 10000, W4_MAX_BITRATE,
			  1, W4_DEF_BITRATE);
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_MPEG_VIDEO_FRAME_RC_ENABLE, 0, 1, 1, 1);
	v4l2_ctrl_new_std_menu(hdl, ops, V4L2_CID_MPEG_VIDEO_BITRATE_MODE,
			       V4L2_MPEG_VIDEO_BITRATE_MODE_CBR,
			       ~BIT(V4L2_MPEG_VIDEO_BITRATE_MODE_CBR),
			       V4L2_MPEG_VIDEO_BITRATE_MODE_CBR);
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_MPEG_VIDEO_GOP_SIZE, 0, 32767, 1, W4_DEF_GOP);
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_MPEG_VIDEO_FORCE_KEY_FRAME, 0, 0, 0, 0);
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_MPEG_VIDEO_HEVC_MIN_QP, 0, 51, 1, 8);
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_MPEG_VIDEO_HEVC_MAX_QP, 0, 51, 1, 51);
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_MPEG_VIDEO_HEVC_I_FRAME_QP, 0, 51, 1, 30);
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_MPEG_VIDEO_HEVC_P_FRAME_QP, 0, 51, 1, 30);
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_MPEG_VIDEO_VBV_DELAY, 0, 32767, 1, 1000);
	v4l2_ctrl_new_std_menu(hdl, ops, V4L2_CID_MPEG_VIDEO_HEADER_MODE,
			       V4L2_MPEG_VIDEO_HEADER_MODE_JOINED_WITH_1ST_FRAME,
			       ~BIT(V4L2_MPEG_VIDEO_HEADER_MODE_JOINED_WITH_1ST_FRAME),
			       V4L2_MPEG_VIDEO_HEADER_MODE_JOINED_WITH_1ST_FRAME);
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_MPEG_VIDEO_PREPEND_SPSPPS_TO_IDR, 1, 1, 1, 1);
	/* One picture in, one access unit out. */
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_MIN_BUFFERS_FOR_OUTPUT, 1, 1, 1, 1);
	v4l2_ctrl_new_std_menu(hdl, ops, V4L2_CID_MPEG_VIDEO_HEVC_PROFILE,
			       V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN,
			       ~BIT(V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN),
			       V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN);
	v4l2_ctrl_new_std_menu(hdl, ops, V4L2_CID_MPEG_VIDEO_HEVC_TIER,
			       V4L2_MPEG_VIDEO_HEVC_TIER_MAIN,
			       ~BIT(V4L2_MPEG_VIDEO_HEVC_TIER_MAIN),
			       V4L2_MPEG_VIDEO_HEVC_TIER_MAIN);
	v4l2_ctrl_new_std_menu(hdl, ops, V4L2_CID_MPEG_VIDEO_HEVC_LEVEL,
			       V4L2_MPEG_VIDEO_HEVC_LEVEL_5_1, 0,
			       V4L2_MPEG_VIDEO_HEVC_LEVEL_4_1);
	if (hdl->error)
		return hdl->error;

	return v4l2_ctrl_handler_setup(hdl);
}

/* File operations */

static int w4_open(struct file *file)
{
	struct w4_dev *w4 = video_drvdata(file);
	struct v4l2_pix_format pix = {
		.pixelformat = V4L2_PIX_FMT_NV12,
		.width = 1920,
		.height = 1080,
	};
	struct w4_ctx *ctx;
	int ret;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;
	ctx->w4 = w4;
	spin_lock_init(&ctx->lock);
	v4l2_fh_init(&ctx->fh, video_devdata(file));

	w4_fill_out(&pix);
	ctx->out.fmt = &w4_out_fmts[0];
	ctx->out.width = pix.width;
	ctx->out.height = pix.height;
	ctx->out.bytesperline = pix.bytesperline;
	ctx->out.sizeimage = pix.sizeimage;
	ctx->out.crop = (struct v4l2_rect){ 0, 0, pix.width, pix.height };
	ctx->cap.width = pix.width;
	ctx->cap.height = pix.height;
	ctx->cap.sizeimage = w4_cap_size(pix.width, pix.height);
	ctx->colour.colorspace = V4L2_COLORSPACE_REC709;
	ctx->params.fps_num = 30;
	ctx->params.fps_den = 1;

	ret = w4_ctrls_setup(ctx);
	if (ret)
		goto err_ctrl;
	ctx->fh.ctrl_handler = &ctx->hdl;
	/* Applied with the whole sequence at stream start. */
	ctx->pending = 0;

	if (mutex_lock_interruptible(&w4->mutex)) {
		ret = -ERESTARTSYS;
		goto err_ctrl;
	}
	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(w4->m2m_dev, ctx, w4_queue_init);
	mutex_unlock(&w4->mutex);
	if (IS_ERR(ctx->fh.m2m_ctx)) {
		ret = PTR_ERR(ctx->fh.m2m_ctx);
		goto err_ctrl;
	}

	v4l2_fh_add(&ctx->fh, file);
	return 0;

err_ctrl:
	v4l2_ctrl_handler_free(&ctx->hdl);
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);
	return ret;
}

static int w4_release(struct file *file)
{
	struct w4_ctx *ctx = file_to_ctx(file);
	struct w4_dev *w4 = ctx->w4;

	mutex_lock(&w4->mutex);
	/* Stops both queues, and with them the sequence. */
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
	if (ctx->started || ctx->holds_sram)
		w4_stop_sequence(ctx);
	mutex_unlock(&w4->mutex);
	v4l2_ctrl_handler_free(&ctx->hdl);
	v4l2_fh_del(&ctx->fh, file);
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);
	return 0;
}

static const struct v4l2_file_operations w4_fops = {
	.owner		= THIS_MODULE,
	.open		= w4_open,
	.release	= w4_release,
	.poll		= v4l2_m2m_fop_poll,
	.unlocked_ioctl	= video_ioctl2,
	.mmap		= v4l2_m2m_fop_mmap,
};

/* Platform device */

static void w4_release_rmem(void *dev)
{
	of_reserved_mem_device_release(dev);
}

static int w4_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct video_device *vfd;
	struct w4_dev *w4;
	unsigned int i;
	u32 product;
	int ret;

	w4 = devm_kzalloc(dev, sizeof(*w4), GFP_KERNEL);
	if (!w4)
		return -ENOMEM;
	w4->dev = dev;
	mutex_init(&w4->mutex);
	spin_lock_init(&w4->hw_lock);
	INIT_DELAYED_WORK(&w4->timeout_work, w4_timeout_work);
	INIT_WORK(&w4->poll_work, w4_poll_work);

	w4->base = devm_platform_ioremap_resource_byname(pdev, "wave");
	if (IS_ERR(w4->base))
		return PTR_ERR(w4->base);

	/*
	 * vc_ctrl is shared with the Coda980 node, which requests the region.
	 * Map it without a request; only the SRAM share register is written,
	 * through the arbiter, with the clocks on.
	 */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "vc-ctrl");
	if (!res)
		return dev_err_probe(dev, -EINVAL, "no vc-ctrl region\n");
	w4->vc_ctrl = devm_ioremap(dev, res->start, resource_size(res));
	if (!w4->vc_ctrl)
		return -ENOMEM;

	for (i = 0; i < ARRAY_SIZE(w4->clks); i++)
		w4->clks[i].id = w4_clk_names[i];
	ret = devm_clk_bulk_get(dev, ARRAY_SIZE(w4->clks), w4->clks);
	if (ret)
		return dev_err_probe(dev, ret, "cannot get clocks\n");

	w4->rstc = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(w4->rstc))
		return dev_err_probe(dev, PTR_ERR(w4->rstc), "cannot get reset\n");

	/* The VCPU and the DMA engines address 32 bits. */
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	/*
	 * Frame buffers and bitstream buffers from the video pool the Coda980
	 * uses: the CMA area is too small and too fragmented for about 9 MiB
	 * of frame buffers next to the capture path (trial 13).
	 */
	ret = of_reserved_mem_device_init(dev);
	if (!ret) {
		ret = devm_add_action_or_reset(dev, w4_release_rmem, dev);
		if (ret)
			return ret;
	} else if (ret != -ENODEV) {
		dev_warn(dev, "no reserved memory pool: %d\n", ret);
	}

	/* Power the core once to see that it answers, and leave it quiet. */
	ret = w4_power_on(w4);
	if (ret)
		return dev_err_probe(dev, ret, "cannot power the core\n");
	product = w4_read(w4, W4_PRODUCT_NUMBER);
	w4_power_off(w4);
	if ((product & 0xfff0) != (WAVE420L_PRODUCT_CODE & 0xfff0))
		return dev_err_probe(dev, -ENODEV, "product number 0x%04x is not a WAVE420L\n",
				     product);

	if (use_irq) {
		w4->irq = platform_get_irq(pdev, 0);
		if (w4->irq < 0)
			return w4->irq;
		/* Enabled only while the core is powered. */
		ret = devm_request_threaded_irq(dev, w4->irq, w4_irq, w4_irq_thread,
						IRQF_ONESHOT | IRQF_NO_AUTOEN,
						dev_name(dev), w4);
		if (ret)
			return dev_err_probe(dev, ret, "cannot request irq %d\n", w4->irq);
	}

	ret = v4l2_device_register(dev, &w4->v4l2_dev);
	if (ret)
		return ret;

	w4->m2m_dev = v4l2_m2m_init(&w4_m2m_ops);
	if (IS_ERR(w4->m2m_dev)) {
		ret = PTR_ERR(w4->m2m_dev);
		goto err_v4l2;
	}

	vfd = &w4->vfd;
	strscpy(vfd->name, W4_NAME, sizeof(vfd->name));
	vfd->fops = &w4_fops;
	vfd->ioctl_ops = &w4_ioctl_ops;
	vfd->minor = -1;
	vfd->release = video_device_release_empty;
	vfd->lock = &w4->mutex;
	vfd->v4l2_dev = &w4->v4l2_dev;
	vfd->vfl_dir = VFL_DIR_M2M;
	vfd->device_caps = V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_M2M;
	video_set_drvdata(vfd, w4);

	ret = video_register_device(vfd, VFL_TYPE_VIDEO, -1);
	if (ret)
		goto err_m2m;

	platform_set_drvdata(pdev, w4);
	dev_info(dev, "WAVE420L (0x%04x) H.265 encoder at /dev/video%d, %s\n", product,
		 vfd->num, w4->irq > 0 ? "interrupt driven" : "polled");
	return 0;

err_m2m:
	v4l2_m2m_release(w4->m2m_dev);
err_v4l2:
	v4l2_device_unregister(&w4->v4l2_dev);
	return ret;
}

static void w4_remove(struct platform_device *pdev)
{
	struct w4_dev *w4 = platform_get_drvdata(pdev);

	video_unregister_device(&w4->vfd);
	cancel_delayed_work_sync(&w4->timeout_work);
	cancel_work_sync(&w4->poll_work);
	v4l2_m2m_release(w4->m2m_dev);
	v4l2_device_unregister(&w4->v4l2_dev);
	/* No context is left, so the core is off and its buffers are idle. */
	w4_free(w4, &w4->temp);
	w4_free(w4, &w4->code);
	if (w4->missed_irqs)
		dev_info(w4->dev, "%lu of %lu pictures finished without an interrupt\n",
			 w4->missed_irqs, w4->pictures);
}

static const struct of_device_id w4_of_match[] = {
	{ .compatible = "sophgo,sg2002-wave420l" },
	{ }
};
MODULE_DEVICE_TABLE(of, w4_of_match);

static struct platform_driver w4_driver = {
	.driver = {
		.name = W4_NAME,
		.of_match_table = w4_of_match,
	},
	.probe = w4_probe,
	.remove = w4_remove,
};
module_platform_driver(w4_driver);

MODULE_DESCRIPTION("Chips&Media WAVE420L H.265 encoder for the Sophgo SG2002");
MODULE_LICENSE("GPL");
