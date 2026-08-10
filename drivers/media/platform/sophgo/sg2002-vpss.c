// SPDX-License-Identifier: GPL-2.0-only
/*
 * Sophgo SG2002 VPSS (VIP scaler / CSC) memory-to-memory driver.
 *
 * The SG2002 "VPSS" register window at 0x0a080000 contains two DRAM-to-IP
 * input DMA engines (IMG_IN) and four scaler cores (SC_D, SC_V1..V3), each
 * with an output CSC and output DMA (ODMA).  This driver uses the IMG_IN_V
 * engine together with the SC_V1 core (the only 4-tap scaler instance) to
 * offer a single v4l2 mem2mem node: packed/planar YUV in, NV12/NV21 out,
 * 1:1 or downscale by up to 4x.
 *
 * Programming model (reverse engineered from the register map; no vendor
 * driver code is reused):
 *  - IMG_IN/SC/ODMA registers are shadowed; a job is committed by setting
 *    the "program done" bit in SC_TOP CFG0 and force-updating the shadow
 *    banks via SC_TOP SHD.
 *  - A job is kicked by the img_v start bit in SC_TOP IMG_CTRL; the
 *    img "ip_clr" bit in IMG DBG must be toggled before every start or the
 *    engine can hang when switching trigger sources.
 *  - Completion is signalled by the scl1_frame_end bit of the shared
 *    SC_TOP interrupt status register (W1C).
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_reserved_mem.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>

#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>

#define DRIVER_NAME	"sg2002-vpss"

/* ---- SC_TOP (base + 0x0000) ---- */
#define TOP_CFG0		0x000	/* bit0 prog done, bit3 ip trig src, bit31 force clk */
#define TOP_CFG1		0x004	/* bit0..4 unit enables, bit12 debug, bit16..23 qos */
#define  CFG1_SC_V1_EN		BIT(1)
#define  CFG1_SC_DEBUG_EN	BIT(12)
#define  CFG1_QOS_EN		(0xff << 16)
#define TOP_SHD			0x010	/* bit0..7 shadow force-up, bit9 shadow read sel */
#define TOP_INTR_MASK		0x030
#define TOP_INTR_STATUS		0x034
#define TOP_INTR_ENABLE		0x038
#define TOP_IMG_CTRL		0x040	/* bit0 img_d start, bit1 img_v start, trig srcs */
#define  IMG_CTRL_START_V	BIT(1)
#define  IMG_CTRL_TRIG_V_MASK	((BIT(12) | BIT(8)) << 1)

/* interrupt status bits (shared line, W1C) */
#define INTR_IMG_IN_V_FRAME_END	BIT(5)
#define INTR_SCL1_FRAME_END	BIT(7)
#define INTR_PROG_TOO_LATE	BIT(10)

/* ---- IMG_IN_V (base + 0x2000) ---- */
#define IMG_BASE		0x2000
#define IMG_CFG			(IMG_BASE + 0x00) /* bit31 fclk, bit12 csc, bit8..11 burst, bit4..7 fmt, bit0..1 src */
#define IMG_OFFSET		(IMG_BASE + 0x04)
#define IMG_SIZE		(IMG_BASE + 0x08) /* (h-1)<<16 | (w-1) */
#define IMG_PITCH_Y		(IMG_BASE + 0x0c)
#define IMG_PITCH_C		(IMG_BASE + 0x10)
#define IMG_SHD			(IMG_BASE + 0x14) /* working-bank read select: 0x4 */
#define IMG_ADDR0_L		(IMG_BASE + 0x24)
#define IMG_ADDR1_L		(IMG_BASE + 0x2c)
#define IMG_ADDR2_L		(IMG_BASE + 0x34)
#define IMG_FIFO_THR		(IMG_BASE + 0x60)
#define IMG_OUTSTANDING		(IMG_BASE + 0x64)
#define IMG_DBG			(IMG_BASE + 0x68) /* bit18 ip_clr, toggle before start */
#define  IMG_DBG_IP_CLR		BIT(18)

#define IMG_SRC_MEM		2
#define IMG_BURST_DEFAULT	7

/* ---- SC_V1 core (base + 0x4000 + 0x1000) ---- */
#define SC_BASE			0x5000
#define SC_CFG_BYPASS		(SC_BASE + 0x00) /* bit1 sc, bit2 gop, bit5 cir, bit6 odma bypass */
#define SC_SHD			(SC_BASE + 0x04) /* working-bank read select: 0x2 */
#define SC_SRC_SIZE		(SC_BASE + 0x0c) /* (h-1)<<12 | (w-1) */
#define SC_CROP_OFFSET		(SC_BASE + 0x10)
#define SC_CROP_SIZE		(SC_BASE + 0x14)
#define SC_COEF0		(SC_BASE + 0x118) /* (0x5<<8)|index = write strobe */
#define SC_COEF1		(SC_BASE + 0x11c)
#define SC_COEF2		(SC_BASE + 0x120)
#define SC_SC_CFG		(SC_BASE + 0x134) /* 0x03: non-tiled, no cb-mode */
#define SC_H_CFG		(SC_BASE + 0x138) /* ratio 5.13 << 8 */
#define SC_V_CFG		(SC_BASE + 0x13c)
#define SC_OUT_SIZE		(SC_BASE + 0x140)
#define SC_INI_PH		(SC_BASE + 0x148)

/* ODMA (core + 0xc00) */
#define ODMA_CFG		(SC_BASE + 0xc00) /* bit16.. flip, bit8..14 fmt, bit0..3 burst */
#define ODMA_ADDR0_L		(SC_BASE + 0xc04)
#define ODMA_ADDR1_L		(SC_BASE + 0xc0c)
#define ODMA_ADDR2_L		(SC_BASE + 0xc14)
#define ODMA_PITCH_Y		(SC_BASE + 0xc1c)
#define ODMA_PITCH_C		(SC_BASE + 0xc20)
#define ODMA_OFFSET_X		(SC_BASE + 0xc24)
#define ODMA_OFFSET_Y		(SC_BASE + 0xc28)
#define ODMA_WIDTH		(SC_BASE + 0xc2c)
#define ODMA_HEIGHT		(SC_BASE + 0xc30)

/* output CSC (core + 0xd00) */
#define CSC_EN			(SC_BASE + 0xd00) /* bit0 matrix enable, bit8 !work_on_border */
#define CSC_COEF0		(SC_BASE + 0xd04)
#define CSC_COEF4		(SC_BASE + 0xd14)
#define CSC_OFFSET		(SC_BASE + 0xd18)

/* hardware format codes (IMG/ODMA fmt field) */
#define HW_FMT_NV12		8
#define HW_FMT_NV21		9
#define HW_FMT_YUYV		13
#define HW_FMT_UYVY		15

#define VPSS_MIN_DIM		64
#define VPSS_MAX_DIM		2880	/* SC_V1 per vendor boundary test */
#define VPSS_MAX_RATIO		4	/* keep out of "factor over 4" cb-mode */

struct vpss_fmt {
	u32	fourcc;
	u8	hw_fmt;
	u8	y_depth;	/* bits per pixel of the luma/packed plane */
	bool	is_420sp;	/* two-plane semiplanar 4:2:0 */
};

static const struct vpss_fmt vpss_out_fmts[] = {
	{ V4L2_PIX_FMT_NV12, HW_FMT_NV12, 8, true },
	{ V4L2_PIX_FMT_NV21, HW_FMT_NV21, 8, true },
};

static const struct vpss_fmt vpss_cap_fmts[] = {
	{ V4L2_PIX_FMT_UYVY, HW_FMT_UYVY, 16, false },
	{ V4L2_PIX_FMT_YUYV, HW_FMT_YUYV, 16, false },
	{ V4L2_PIX_FMT_NV12, HW_FMT_NV12, 8, true },
	{ V4L2_PIX_FMT_NV21, HW_FMT_NV21, 8, true },
};

struct vpss_q_data {
	const struct vpss_fmt	*fmt;
	u32			width;
	u32			height;
	u32			bytesperline;
	u32			sizeimage;
};

struct vpss_dev {
	struct v4l2_device	v4l2_dev;
	struct video_device	vfd;
	struct v4l2_m2m_dev	*m2m_dev;
	struct device		*dev;
	void __iomem		*regs;
	int			irq;
	struct clk_bulk_data	*clks;
	int			num_clks;

	struct mutex		lock;		/* serialises vb2 + job state */
	spinlock_t		irq_lock;	/* guards the in-flight job */
	struct vpss_ctx		*curr;
	struct vb2_v4l2_buffer	*curr_src;
	struct vb2_v4l2_buffer	*curr_dst;

	bool			coef_is_identity;
};

struct vpss_ctx {
	struct v4l2_fh		fh;
	struct vpss_dev		*dev;
	struct vpss_q_data	q_data[2];	/* [OUTPUT]=src, [CAPTURE]=dst */
};

static const s16 vpss_bicubic_coef[128][4] = {
	{   -2, 1024,    2,    0 }, {   -6, 1024,    6,    0 }, {  -10, 1023,   11,    0 }, {  -13, 1022,   15,    0 },
	{  -17, 1022,   20,   -1 }, {  -20, 1019,   26,   -1 }, {  -23, 1017,   31,   -1 }, {  -27, 1016,   37,   -2 },
	{  -30, 1013,   43,   -2 }, {  -33, 1011,   49,   -3 }, {  -35, 1007,   55,   -3 }, {  -38, 1005,   61,   -4 },
	{  -41, 1001,   68,   -4 }, {  -43,  997,   75,   -5 }, {  -46,  994,   82,   -6 }, {  -48,  990,   89,   -7 },
	{  -50,  984,   97,   -7 }, {  -52,  980,  104,   -8 }, {  -54,  975,  112,   -9 }, {  -56,  970,  120,  -10 },
	{  -58,  965,  128,  -11 }, {  -60,  959,  137,  -12 }, {  -61,  953,  145,  -13 }, {  -63,  947,  154,  -14 },
	{  -64,  941,  162,  -15 }, {  -65,  934,  171,  -16 }, {  -67,  928,  180,  -17 }, {  -68,  922,  189,  -19 },
	{  -69,  914,  199,  -20 }, {  -70,  907,  208,  -21 }, {  -71,  899,  218,  -22 }, {  -72,  892,  227,  -23 },
	{  -72,  884,  237,  -25 }, {  -73,  876,  247,  -26 }, {  -74,  868,  257,  -27 }, {  -74,  859,  267,  -28 },
	{  -75,  852,  277,  -30 }, {  -75,  843,  287,  -31 }, {  -75,  834,  297,  -32 }, {  -76,  826,  308,  -34 },
	{  -76,  817,  318,  -35 }, {  -76,  807,  329,  -36 }, {  -76,  798,  340,  -38 }, {  -76,  789,  350,  -39 },
	{  -76,  779,  361,  -40 }, {  -76,  770,  372,  -42 }, {  -75,  759,  383,  -43 }, {  -75,  749,  394,  -44 },
	{  -75,  741,  404,  -46 }, {  -74,  730,  415,  -47 }, {  -74,  720,  426,  -48 }, {  -74,  711,  437,  -50 },
	{  -73,  699,  449,  -51 }, {  -72,  688,  460,  -52 }, {  -72,  678,  471,  -53 }, {  -71,  668,  482,  -55 },
	{  -71,  658,  493,  -56 }, {  -70,  647,  504,  -57 }, {  -69,  636,  515,  -58 }, {  -68,  625,  526,  -59 },
	{  -67,  614,  537,  -60 }, {  -66,  603,  548,  -61 }, {  -65,  592,  559,  -62 }, {  -64,  581,  570,  -63 },
	{  -63,  570,  581,  -64 }, {  -62,  559,  592,  -65 }, {  -61,  548,  603,  -66 }, {  -60,  537,  614,  -67 },
	{  -59,  526,  625,  -68 }, {  -58,  515,  636,  -69 }, {  -57,  504,  647,  -70 }, {  -56,  494,  657,  -71 },
	{  -55,  482,  668,  -71 }, {  -53,  471,  678,  -72 }, {  -52,  459,  689,  -72 }, {  -51,  449,  699,  -73 },
	{  -50,  438,  710,  -74 }, {  -48,  426,  720,  -74 }, {  -47,  415,  730,  -74 }, {  -46,  405,  740,  -75 },
	{  -44,  393,  750,  -75 }, {  -43,  382,  760,  -75 }, {  -42,  372,  770,  -76 }, {  -40,  361,  779,  -76 },
	{  -39,  350,  789,  -76 }, {  -38,  340,  798,  -76 }, {  -36,  329,  807,  -76 }, {  -35,  319,  816,  -76 },
	{  -34,  309,  825,  -76 }, {  -32,  297,  834,  -75 }, {  -31,  287,  843,  -75 }, {  -30,  278,  851,  -75 },
	{  -28,  266,  860,  -74 }, {  -27,  257,  868,  -74 }, {  -26,  247,  876,  -73 }, {  -25,  237,  884,  -72 },
	{  -23,  227,  892,  -72 }, {  -22,  218,  899,  -71 }, {  -21,  208,  907,  -70 }, {  -20,  199,  914,  -69 },
	{  -19,  190,  921,  -68 }, {  -17,  180,  928,  -67 }, {  -16,  170,  935,  -65 }, {  -15,  162,  941,  -64 },
	{  -14,  154,  947,  -63 }, {  -13,  145,  953,  -61 }, {  -12,  137,  959,  -60 }, {  -11,  128,  965,  -58 },
	{  -10,  120,  970,  -56 }, {   -9,  112,  975,  -54 }, {   -8,  104,  980,  -52 }, {   -7,   96,  985,  -50 },
	{   -7,   90,  989,  -48 }, {   -6,   83,  993,  -46 }, {   -5,   75,  997,  -43 }, {   -4,   68, 1001,  -41 },
	{   -4,   62, 1004,  -38 }, {   -3,   54, 1008,  -35 }, {   -3,   49, 1011,  -33 }, {   -2,   43, 1013,  -30 },
	{   -2,   37, 1016,  -27 }, {   -1,   30, 1018,  -23 }, {   -1,   26, 1019,  -20 }, {   -1,   21, 1021,  -17 },
	{    0,   15, 1022,  -13 }, {    0,   11, 1023,  -10 }, {    0,    6, 1024,   -6 }, {    0,    2, 1024,   -2 },
};

static const s16 vpss_identity_coef[128][4] = {
	{    0, 1024,    0,    0 }, {    0, 1024,    0,    0 }, {    0, 1024,    0,    0 }, {    0, 1024,    0,    0 },
	{    0, 1024,    0,    0 }, {    0, 1024,    0,    0 }, {    0, 1024,    0,    0 }, {    0, 1024,    0,    0 },
	{    0, 1024,    0,    0 }, {    0, 1024,    0,    0 }, {    0, 1024,    0,    0 }, {    0, 1024,    0,    0 },
	{    0, 1024,    0,    0 }, {    0, 1024,    0,    0 }, {    0, 1024,    0,    0 }, {    0, 1024,    0,    0 },
	{    0, 1024,    0,    0 }, {    0, 1024,    0,    0 }, {    0, 1024,    0,    0 }, {    0, 1024,    0,    0 },
	{    0, 1024,    0,    0 }, {    0, 1024,    0,    0 }, {    0, 1024,    0,    0 }, {    0, 1024,    0,    0 },
	{    0, 1024,    0,    0 }, {    0, 1024,    0,    0 }, {    0, 1024,    0,    0 }, {    0, 1024,    0,    0 },
	{    0, 1024,    0,    0 }, {    0, 1024,    0,    0 }, {    0, 1024,    0,    0 }, {    0, 1024,    0,    0 },
	{    0, 1024,    0,    0 }, {    0, 1024,    0,    0 }, {    0, 1024,    0,    0 }, {    0, 1024,    0,    0 },
	{    0, 1024,    0,    0 }, {    0, 1024,    0,    0 }, {    0, 1024,    0,    0 }, {    0, 1024,    0,    0 },
	{    0, 1024,    0,    0 }, {    0, 1024,    0,    0 }, {    0, 1024,    0,    0 }, {    0, 1024,    0,    0 },
	{    0, 1024,    0,    0 }, {    0, 1024,    0,    0 }, {    0, 1024,    0,    0 }, {    0, 1024,    0,    0 },
	{    0, 1024,    0,    0 }, {    0, 1024,    0,    0 }, {    0, 1024,    0,    0 }, {    0, 1024,    0,    0 },
	{    0, 1024,    0,    0 }, {    0, 1024,    0,    0 }, {    0, 1024,    0,    0 }, {    0, 1024,    0,    0 },
	{    0, 1024,    0,    0 }, {    0, 1024,    0,    0 }, {    0, 1024,    0,    0 }, {    0, 1024,    0,    0 },
	{    0, 1024,    0,    0 }, {    0, 1024,    0,    0 }, {    0, 1024,    0,    0 }, {    0, 1024,    0,    0 },
	{    0, 1024,    0,    0 }, {    0, 1024,    0,    0 }, {    0, 1024,    0,    0 }, {    0, 1024,    0,    0 },
	{    0, 1024,    0,    0 }, {    0, 1024,    0,    0 }, {    0, 1024,    0,    0 }, {    0, 1024,    0,    0 },
	{    0, 1024,    0,    0 }, {    0, 1024,    0,    0 }, {    0, 1024,    0,    0 }, {    0, 1024,    0,    0 },
	{    0, 1024,    0,    0 }, {    0, 1024,    0,    0 }, {    0, 1024,    0,    0 }, {    0, 1024,    0,    0 },
	{    0, 1024,    0,    0 }, {    0, 1024,    0,    0 }, {    0, 1024,    0,    0 }, {    0, 1024,    0,    0 },
	{    0, 1024,    0,    0 }, {    0, 1024,    0,    0 }, {    0, 1024,    0,    0 }, {    0, 1024,    0,    0 },
	{    0, 1024,    0,    0 }, {    0, 1024,    0,    0 }, {    0, 1024,    0,    0 }, {    0, 1024,    0,    0 },
	{    0, 1024,    0,    0 }, {    0, 1024,    0,    0 }, {    0, 1024,    0,    0 }, {    0, 1024,    0,    0 },
	{    0, 1024,    0,    0 }, {    0, 1024,    0,    0 }, {    0, 1024,    0,    0 }, {    0, 1024,    0,    0 },
	{    0, 1024,    0,    0 }, {    0, 1024,    0,    0 }, {    0, 1024,    0,    0 }, {    0, 1024,    0,    0 },
	{    0, 1024,    0,    0 }, {    0, 1024,    0,    0 }, {    0, 1024,    0,    0 }, {    0, 1024,    0,    0 },
	{    0, 1024,    0,    0 }, {    0, 1024,    0,    0 }, {    0, 1024,    0,    0 }, {    0, 1024,    0,    0 },
	{    0, 1024,    0,    0 }, {    0, 1024,    0,    0 }, {    0, 1024,    0,    0 }, {    0, 1024,    0,    0 },
	{    0, 1024,    0,    0 }, {    0, 1024,    0,    0 }, {    0, 1024,    0,    0 }, {    0, 1024,    0,    0 },
	{    0, 1024,    0,    0 }, {    0, 1024,    0,    0 }, {    0, 1024,    0,    0 }, {    0, 1024,    0,    0 },
	{    0, 1024,    0,    0 }, {    0, 1024,    0,    0 }, {    0, 1024,    0,    0 }, {    0, 1024,    0,    0 },
};

static inline void vpss_write(struct vpss_dev *vpss, u32 reg, u32 val)
{
	writel(val, vpss->regs + reg);
}

static inline u32 vpss_read(struct vpss_dev *vpss, u32 reg)
{
	return readl(vpss->regs + reg);
}

static void vpss_write_mask(struct vpss_dev *vpss, u32 reg, u32 mask, u32 val)
{
	vpss_write(vpss, reg, (vpss_read(vpss, reg) & ~mask) | (val & mask));
}

static void vpss_set_addr(struct vpss_dev *vpss, u32 reg, dma_addr_t addr)
{
	vpss_write(vpss, reg, lower_32_bits(addr));
	vpss_write(vpss, reg + 4, upper_32_bits(addr));
}

/*
 * Load one 4-tap coefficient set (128 phases) into the shared scaler RAM.
 * Entry i is committed by the (0x5<<8)|i strobe on COEF0.
 */
static void vpss_load_coef(struct vpss_dev *vpss, bool identity)
{
	const s16 (*tab)[4] = identity ? vpss_identity_coef : vpss_bicubic_coef;
	int i;

	if (vpss->coef_is_identity == identity)
		return;

	for (i = 0; i < 128; i++) {
		vpss_write(vpss, SC_COEF1, (tab[i][1] << 16) | (tab[i][0] & 0x0fff));
		vpss_write(vpss, SC_COEF2, (tab[i][3] << 16) | (tab[i][2] & 0x0fff));
		vpss_write(vpss, SC_COEF0, (0x5 << 8) | i);
	}
	vpss->coef_is_identity = identity;
}

static void vpss_hw_init(struct vpss_dev *vpss)
{
	/* working-bank read select for IMG_IN_V and SC_V1 */
	vpss_write(vpss, IMG_SHD, 0x4);
	vpss_write(vpss, SC_SHD, 0x2);

	/* APB reads come from the shadow bank */
	vpss_write(vpss, TOP_SHD, 0x200);

	/* accept every block interrupt at the top level; the mask decides */
	vpss_write(vpss, TOP_INTR_ENABLE, 0xffffffff);
	vpss_write(vpss, TOP_INTR_MASK,
		   INTR_IMG_IN_V_FRAME_END | INTR_SCL1_FRAME_END | INTR_PROG_TOO_LATE);

	vpss->coef_is_identity = true;
	vpss_load_coef(vpss, false);
}

static void vpss_job_hw(struct vpss_dev *vpss, struct vpss_ctx *ctx,
			dma_addr_t src, dma_addr_t dst)
{
	struct vpss_q_data *q_in = &ctx->q_data[V4L2_BUF_TYPE_VIDEO_OUTPUT];
	struct vpss_q_data *q_out = &ctx->q_data[V4L2_BUF_TYPE_VIDEO_CAPTURE];
	u32 sw = q_in->width, sh = q_in->height;
	u32 dw = q_out->width, dh = q_out->height;
	u32 ratio;
	bool scale = (sw != dw) || (sh != dh);

	/* IMG_IN_V: DRAM source, no input CSC */
	vpss_write(vpss, IMG_FIFO_THR,
		   (q_in->fmt->is_420sp ? 0x40 << 16 : 0x80 << 16) | 0x80);
	vpss_write(vpss, IMG_OUTSTANDING, 0xf);
	vpss_write(vpss, IMG_CFG,
		   (IMG_BURST_DEFAULT << 8) | (q_in->fmt->hw_fmt << 4) | IMG_SRC_MEM);
	vpss_write(vpss, IMG_OFFSET, 0);
	vpss_write(vpss, IMG_SIZE, ((sh - 1) << 16) | (sw - 1));
	vpss_write(vpss, IMG_PITCH_Y, q_in->bytesperline);
	vpss_write(vpss, IMG_PITCH_C,
		   q_in->fmt->is_420sp ? sw : q_in->bytesperline);
	vpss_set_addr(vpss, IMG_ADDR0_L, src);
	vpss_set_addr(vpss, IMG_ADDR1_L,
		      q_in->fmt->is_420sp ? src + (u64)q_in->bytesperline * sh : 0);
	vpss_set_addr(vpss, IMG_ADDR2_L, 0);

	/* software trigger for img_v */
	vpss_write_mask(vpss, TOP_IMG_CTRL, IMG_CTRL_TRIG_V_MASK, 0);

	/* SC_V1: full-frame crop, 4-tap scale */
	vpss_write(vpss, SC_SRC_SIZE, ((sh - 1) << 12) | (sw - 1));
	vpss_write(vpss, SC_CROP_OFFSET, 0);
	vpss_write(vpss, SC_CROP_SIZE, ((sh - 1) << 16) | (sw - 1));
	vpss_write(vpss, SC_OUT_SIZE, ((dh - 1) << 16) | (dw - 1));

	ratio = DIV_ROUND_CLOSEST((sw - 1) << 13, dw - 1);
	vpss_write_mask(vpss, SC_H_CFG, 0x03ffff00, ratio << 8);
	ratio = DIV_ROUND_CLOSEST((sh - 1) << 13, dh - 1);
	vpss_write_mask(vpss, SC_V_CFG, 0x03ffff00, ratio << 8);
	vpss_write(vpss, SC_SC_CFG, 0x03);
	vpss_write(vpss, SC_INI_PH, 0);
	vpss_load_coef(vpss, !scale);

	/* bypass circle/GOP engines, keep scaler + ODMA active */
	vpss_write(vpss, SC_CFG_BYPASS, BIT(2) | BIT(5));

	/* output CSC: matrix mode with identity coefficients (YUV passthrough) */
	vpss_write(vpss, CSC_COEF0, 1024);
	vpss_write(vpss, CSC_COEF0 + 4, 0);
	vpss_write(vpss, CSC_COEF0 + 8, 1024);
	vpss_write(vpss, CSC_COEF0 + 12, 0);
	vpss_write(vpss, CSC_COEF4, 1024);
	vpss_write(vpss, CSC_OFFSET, 0);
	vpss_write(vpss, CSC_EN, BIT(0) | BIT(8));

	/* ODMA */
	vpss_write(vpss, ODMA_CFG, (q_out->fmt->hw_fmt << 8) | IMG_BURST_DEFAULT);
	vpss_write(vpss, ODMA_OFFSET_X, 0);
	vpss_write(vpss, ODMA_OFFSET_Y, 0);
	vpss_write(vpss, ODMA_WIDTH, dw - 1);
	vpss_write(vpss, ODMA_HEIGHT, dh - 1);
	vpss_write(vpss, ODMA_PITCH_Y, q_out->bytesperline);
	vpss_write(vpss, ODMA_PITCH_C, dw);
	vpss_set_addr(vpss, ODMA_ADDR0_L, dst);
	vpss_set_addr(vpss, ODMA_ADDR1_L, dst + (u64)q_out->bytesperline * dh);
	vpss_set_addr(vpss, ODMA_ADDR2_L, 0);

	/* enable SC_V1 (+ debug, QoS), then commit the shadow banks */
	vpss_write(vpss, TOP_CFG1, CFG1_SC_V1_EN | CFG1_SC_DEBUG_EN | CFG1_QOS_EN);
	vpss_write_mask(vpss, TOP_CFG0, BIT(0), BIT(0));
	vpss_write_mask(vpss, TOP_SHD, 0xff, 0xff);

	/* load-bearing: clear the img engine, then kick img_v */
	vpss_write_mask(vpss, IMG_DBG, IMG_DBG_IP_CLR, IMG_DBG_IP_CLR);
	vpss_write_mask(vpss, TOP_IMG_CTRL, 0x3, IMG_CTRL_START_V);
}

/* ------------------------------------------------------------------ */

static struct vpss_q_data *vpss_get_q_data(struct vpss_ctx *ctx,
					   enum v4l2_buf_type type)
{
	return &ctx->q_data[V4L2_TYPE_IS_OUTPUT(type) ?
			    V4L2_BUF_TYPE_VIDEO_OUTPUT : V4L2_BUF_TYPE_VIDEO_CAPTURE];
}

static const struct vpss_fmt *vpss_find_fmt(u32 fourcc, bool output)
{
	const struct vpss_fmt *fmts = output ? vpss_cap_fmts : vpss_out_fmts;
	int i, n = output ? ARRAY_SIZE(vpss_cap_fmts) : ARRAY_SIZE(vpss_out_fmts);

	for (i = 0; i < n; i++)
		if (fmts[i].fourcc == fourcc)
			return &fmts[i];
	return &fmts[0];
}

static int vpss_enum_fmt(struct file *file, void *priv,
			 struct v4l2_fmtdesc *f)
{
	const struct vpss_fmt *fmts;
	int n;

	if (V4L2_TYPE_IS_OUTPUT(f->type)) {
		fmts = vpss_out_fmts;
		n = ARRAY_SIZE(vpss_out_fmts);
	} else {
		fmts = vpss_cap_fmts;
		n = ARRAY_SIZE(vpss_cap_fmts);
	}
	if (f->index >= n)
		return -EINVAL;
	f->pixelformat = fmts[f->index].fourcc;
	return 0;
}

static bool vpss_valid_fourcc(u32 fourcc)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(vpss_out_fmts); i++)
		if (vpss_out_fmts[i].fourcc == fourcc)
			return true;
	for (i = 0; i < ARRAY_SIZE(vpss_cap_fmts); i++)
		if (vpss_cap_fmts[i].fourcc == fourcc)
			return true;
	return false;
}

static int vpss_enum_framesizes(struct file *file, void *priv,
				struct v4l2_frmsizeenum *fsize)
{
	if (fsize->index)
		return -EINVAL;
	if (!vpss_valid_fourcc(fsize->pixel_format))
		return -EINVAL;
	fsize->type = V4L2_FRMSIZE_TYPE_STEPWISE;
	fsize->stepwise.min_width = VPSS_MIN_DIM;
	fsize->stepwise.max_width = VPSS_MAX_DIM;
	fsize->stepwise.step_width = 2;
	fsize->stepwise.min_height = VPSS_MIN_DIM;
	fsize->stepwise.max_height = VPSS_MAX_DIM;
	fsize->stepwise.step_height = 2;
	return 0;
}

static int vpss_g_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct vpss_ctx *ctx = file->private_data;
	struct vpss_q_data *q_data = vpss_get_q_data(ctx, f->type);
	struct v4l2_pix_format *pix = &f->fmt.pix;

	pix->width = q_data->width;
	pix->height = q_data->height;
	pix->pixelformat = q_data->fmt->fourcc;
	pix->field = V4L2_FIELD_NONE;
	pix->bytesperline = q_data->bytesperline;
	pix->sizeimage = q_data->sizeimage;
	pix->colorspace = V4L2_COLORSPACE_REC709;
	pix->xfer_func = V4L2_XFER_FUNC_709;
	pix->ycbcr_enc = V4L2_YCBCR_ENC_709;
	pix->quantization = V4L2_QUANTIZATION_LIM_RANGE;
	return 0;
}

static int vpss_try_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct v4l2_pix_format *pix = &f->fmt.pix;
	const struct vpss_fmt *fmt;

	fmt = vpss_find_fmt(pix->pixelformat, V4L2_TYPE_IS_OUTPUT(f->type));
	pix->pixelformat = fmt->fourcc;
	pix->width = clamp_t(u32, pix->width, VPSS_MIN_DIM, VPSS_MAX_DIM);
	pix->height = clamp_t(u32, pix->height, VPSS_MIN_DIM, VPSS_MAX_DIM);
	/* 4:2:0 formats and the packed 4:2:2 subsample grid need even dims */
	pix->width = round_down(pix->width, 2);
	pix->height = round_down(pix->height, 2);
	pix->field = V4L2_FIELD_NONE;
	pix->bytesperline = pix->width * fmt->y_depth / 8;
	pix->sizeimage = pix->bytesperline * pix->height;
	if (fmt->is_420sp)
		pix->sizeimage += pix->width * pix->height / 2;
	return 0;
}

static int vpss_s_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct vpss_ctx *ctx = file->private_data;
	struct vpss_q_data *q_data;
	struct vb2_queue *vq;
	int ret;

	vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx, f->type);
	if (!vq)
		return -EINVAL;
	if (vb2_is_busy(vq))
		return -EBUSY;

	ret = vpss_try_fmt(file, priv, f);
	if (ret)
		return ret;

	q_data = vpss_get_q_data(ctx, f->type);
	q_data->fmt = vpss_find_fmt(f->fmt.pix.pixelformat,
				    V4L2_TYPE_IS_OUTPUT(f->type));
	q_data->width = f->fmt.pix.width;
	q_data->height = f->fmt.pix.height;
	q_data->bytesperline = f->fmt.pix.bytesperline;
	q_data->sizeimage = f->fmt.pix.sizeimage;
	return 0;
}

/* ------------------------------------------------------------------ */

static int vpss_queue_setup(struct vb2_queue *vq, unsigned int *nbuffers,
			    unsigned int *nplanes, unsigned int sizes[],
			    struct device *alloc_devs[])
{
	struct vpss_ctx *ctx = vb2_get_drv_priv(vq);
	struct vpss_q_data *q_data = vpss_get_q_data(ctx, vq->type);

	if (*nplanes)
		return sizes[0] < q_data->sizeimage ? -EINVAL : 0;

	*nplanes = 1;
	sizes[0] = q_data->sizeimage;
	return 0;
}

static int vpss_buf_prepare(struct vb2_buffer *vb)
{
	struct vpss_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct vpss_q_data *q_data = vpss_get_q_data(ctx, vb->vb2_queue->type);

	if (vb2_plane_size(vb, 0) < q_data->sizeimage)
		return -EINVAL;
	vb2_set_plane_payload(vb, 0, q_data->sizeimage);
	return 0;
}

static void vpss_buf_queue(struct vb2_buffer *vb)
{
	struct vpss_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);

	v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, to_vb2_v4l2_buffer(vb));
}

static void vpss_buf_finish(struct vb2_buffer *vb)
{
	struct vpss_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct vpss_q_data *q_data = vpss_get_q_data(ctx, vb->vb2_queue->type);

	vb2_set_plane_payload(vb, 0, q_data->sizeimage);
}

static void vpss_stop_streaming(struct vb2_queue *vq)
{
	struct vpss_ctx *ctx = vb2_get_drv_priv(vq);
	struct vb2_v4l2_buffer *vb;

	/* drop a job that is still parked in the m2m queues */
	for (;;) {
		if (V4L2_TYPE_IS_OUTPUT(vq->type))
			vb = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
		else
			vb = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);
		if (!vb)
			break;
		v4l2_m2m_buf_done(vb, VB2_BUF_STATE_ERROR);
	}
}

static const struct vb2_ops vpss_qops = {
	.queue_setup	= vpss_queue_setup,
	.buf_prepare	= vpss_buf_prepare,
	.buf_finish	= vpss_buf_finish,
	.buf_queue	= vpss_buf_queue,
	.stop_streaming	= vpss_stop_streaming,
};

static int vpss_queue_init(void *priv, struct vb2_queue *src_vq,
			   struct vb2_queue *dst_vq)
{
	struct vpss_ctx *ctx = priv;
	int ret;

	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	src_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	src_vq->drv_priv = ctx;
	src_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	src_vq->ops = &vpss_qops;
	src_vq->mem_ops = &vb2_dma_contig_memops;
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC |
				  V4L2_BUF_FLAG_TSTAMP_SRC_EOF;
	src_vq->lock = &ctx->dev->lock;
	src_vq->dev = ctx->dev->dev;

	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	dst_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	dst_vq->drv_priv = ctx;
	dst_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	dst_vq->ops = &vpss_qops;
	dst_vq->mem_ops = &vb2_dma_contig_memops;
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC |
				  V4L2_BUF_FLAG_TSTAMP_SRC_EOF;
	dst_vq->lock = &ctx->dev->lock;
	dst_vq->dev = ctx->dev->dev;

	return vb2_queue_init(dst_vq);
}

/* ------------------------------------------------------------------ */

static void vpss_job_finish(struct vpss_dev *vpss, enum vb2_buffer_state state)
{
	struct vpss_ctx *ctx;
	struct vb2_v4l2_buffer *src, *dst;
	unsigned long flags;

	spin_lock_irqsave(&vpss->irq_lock, flags);
	ctx = vpss->curr;
	src = vpss->curr_src;
	dst = vpss->curr_dst;
	vpss->curr = NULL;
	vpss->curr_src = NULL;
	vpss->curr_dst = NULL;
	spin_unlock_irqrestore(&vpss->irq_lock, flags);

	if (WARN_ON(!ctx || !src || !dst))
		return;

	pm_runtime_put(vpss->dev);

	v4l2_m2m_buf_done(src, state);
	v4l2_m2m_buf_done(dst, state);
	v4l2_m2m_job_finish(vpss->m2m_dev, ctx->fh.m2m_ctx);
}

static void vpss_device_run(void *priv)
{
	struct vpss_ctx *ctx = priv;
	struct vpss_dev *vpss = ctx->dev;
	struct vb2_v4l2_buffer *src, *dst;
	struct vpss_q_data *q_in = &ctx->q_data[V4L2_BUF_TYPE_VIDEO_OUTPUT];
	struct vpss_q_data *q_out = &ctx->q_data[V4L2_BUF_TYPE_VIDEO_CAPTURE];
	unsigned long flags;
	int ret;

	src = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	dst = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);
	if (WARN_ON(!src || !dst))
		return;

	/* 1:1 or downscale by at most VPSS_MAX_RATIO in each dimension */
	if (q_out->width > q_in->width || q_out->height > q_in->height ||
	    q_out->width * VPSS_MAX_RATIO < q_in->width ||
	    q_out->height * VPSS_MAX_RATIO < q_in->height) {
		dev_err(vpss->dev, "rejecting %ux%u -> %ux%u conversion\n",
			q_in->width, q_in->height, q_out->width, q_out->height);
		src = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
		dst = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);
		v4l2_m2m_buf_done(src, VB2_BUF_STATE_ERROR);
		v4l2_m2m_buf_done(dst, VB2_BUF_STATE_ERROR);
		v4l2_m2m_job_finish(vpss->m2m_dev, ctx->fh.m2m_ctx);
		return;
	}

	ret = pm_runtime_get_sync(vpss->dev);
	if (ret < 0) {
		pm_runtime_put_noidle(vpss->dev);
		src = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
		dst = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);
		v4l2_m2m_buf_done(src, VB2_BUF_STATE_ERROR);
		v4l2_m2m_buf_done(dst, VB2_BUF_STATE_ERROR);
		v4l2_m2m_job_finish(vpss->m2m_dev, ctx->fh.m2m_ctx);
		return;
	}

	src = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
	dst = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);

	spin_lock_irqsave(&vpss->irq_lock, flags);
	vpss->curr = ctx;
	vpss->curr_src = src;
	vpss->curr_dst = dst;
	spin_unlock_irqrestore(&vpss->irq_lock, flags);

	src->vb2_buf.timestamp = ktime_get_ns();

	vpss_job_hw(vpss, ctx,
		    vb2_dma_contig_plane_dma_addr(&src->vb2_buf, 0),
		    vb2_dma_contig_plane_dma_addr(&dst->vb2_buf, 0));
}

static int vpss_job_ready(void *priv)
{
	struct vpss_ctx *ctx = priv;

	return v4l2_m2m_num_src_bufs_ready(ctx->fh.m2m_ctx) > 0 &&
	       v4l2_m2m_num_dst_bufs_ready(ctx->fh.m2m_ctx) > 0;
}

static const struct v4l2_m2m_ops vpss_m2m_ops = {
	.device_run	= vpss_device_run,
	.job_ready	= vpss_job_ready,
};

static irqreturn_t vpss_isr(int irq, void *data)
{
	struct vpss_dev *vpss = data;
	u32 status = vpss_read(vpss, TOP_INTR_STATUS);

	status &= INTR_IMG_IN_V_FRAME_END | INTR_SCL1_FRAME_END |
		  INTR_PROG_TOO_LATE;
	if (!status)
		return IRQ_NONE;

	/* W1C */
	vpss_write(vpss, TOP_INTR_STATUS, status);

	if (status & INTR_PROG_TOO_LATE) {
		dev_err_ratelimited(vpss->dev, "job programming too late\n");
		vpss_job_finish(vpss, VB2_BUF_STATE_ERROR);
	} else if (status & INTR_SCL1_FRAME_END) {
		vpss_job_finish(vpss, VB2_BUF_STATE_DONE);
	}
	return IRQ_HANDLED;
}

/* ------------------------------------------------------------------ */

static int vpss_open(struct file *file)
{
	struct vpss_dev *vpss = video_drvdata(file);
	struct vpss_ctx *ctx;
	struct vpss_q_data *q_in, *q_out;
	int ret;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->dev = vpss;
	v4l2_fh_init(&ctx->fh, video_devdata(file));
	file->private_data = ctx;
	v4l2_fh_add(&ctx->fh, file);

	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(vpss->m2m_dev, ctx,
					    vpss_queue_init);
	if (IS_ERR(ctx->fh.m2m_ctx)) {
		ret = PTR_ERR(ctx->fh.m2m_ctx);
		goto err_fh;
	}

	/* defaults: 1080p UYVY -> NV12 1:1 */
	q_in = &ctx->q_data[V4L2_BUF_TYPE_VIDEO_OUTPUT];
	q_in->fmt = vpss_find_fmt(V4L2_PIX_FMT_UYVY, true);
	q_in->width = 1920;
	q_in->height = 1080;
	q_in->bytesperline = 1920 * 2;
	q_in->sizeimage = q_in->bytesperline * 1080;

	q_out = &ctx->q_data[V4L2_BUF_TYPE_VIDEO_CAPTURE];
	q_out->fmt = vpss_find_fmt(V4L2_PIX_FMT_NV12, false);
	q_out->width = 1920;
	q_out->height = 1080;
	q_out->bytesperline = 1920;
	q_out->sizeimage = 1920 * 1080 * 3 / 2;

	return 0;

err_fh:
	v4l2_fh_del(&ctx->fh, file);
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);
	return ret;
}

static int vpss_release(struct file *file)
{
	struct vpss_ctx *ctx = file->private_data;

	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
	v4l2_fh_del(&ctx->fh, file);
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);
	return 0;
}

static const struct v4l2_file_operations vpss_fops = {
	.owner		= THIS_MODULE,
	.open		= vpss_open,
	.release	= vpss_release,
	.poll		= v4l2_m2m_fop_poll,
	.unlocked_ioctl	= video_ioctl2,
	.mmap		= v4l2_m2m_fop_mmap,
};

static int vpss_querycap(struct file *file, void *priv,
			 struct v4l2_capability *cap)
{
	strscpy(cap->driver, DRIVER_NAME, sizeof(cap->driver));
	strscpy(cap->card, "SG2002 VPSS scaler/CSC", sizeof(cap->card));
	strscpy(cap->bus_info, "platform:" DRIVER_NAME, sizeof(cap->bus_info));
	return 0;
}

static const struct v4l2_ioctl_ops vpss_ioctl_ops = {
	.vidioc_querycap		= vpss_querycap,
	.vidioc_enum_fmt_vid_out	= vpss_enum_fmt,
	.vidioc_enum_fmt_vid_cap	= vpss_enum_fmt,
	.vidioc_enum_framesizes		= vpss_enum_framesizes,
	.vidioc_g_fmt_vid_out		= vpss_g_fmt,
	.vidioc_g_fmt_vid_cap		= vpss_g_fmt,
	.vidioc_try_fmt_vid_out		= vpss_try_fmt,
	.vidioc_try_fmt_vid_cap		= vpss_try_fmt,
	.vidioc_s_fmt_vid_out		= vpss_s_fmt,
	.vidioc_s_fmt_vid_cap		= vpss_s_fmt,
	.vidioc_reqbufs			= v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf		= v4l2_m2m_ioctl_querybuf,
	.vidioc_qbuf			= v4l2_m2m_ioctl_qbuf,
	.vidioc_dqbuf			= v4l2_m2m_ioctl_dqbuf,
	.vidioc_prepare_buf		= v4l2_m2m_ioctl_prepare_buf,
	.vidioc_create_bufs		= v4l2_m2m_ioctl_create_bufs,
	.vidioc_expbuf			= v4l2_m2m_ioctl_expbuf,
	.vidioc_streamon		= v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff		= v4l2_m2m_ioctl_streamoff,
};

/* ------------------------------------------------------------------ */

static int vpss_runtime_resume(struct device *dev)
{
	struct vpss_dev *vpss = dev_get_drvdata(dev);
	int ret;

	ret = clk_bulk_prepare_enable(vpss->num_clks, vpss->clks);
	if (ret)
		return ret;

	vpss_hw_init(vpss);
	return 0;
}

static int vpss_runtime_suspend(struct device *dev)
{
	struct vpss_dev *vpss = dev_get_drvdata(dev);

	clk_bulk_disable_unprepare(vpss->num_clks, vpss->clks);
	return 0;
}

static const struct dev_pm_ops vpss_pm_ops = {
	.runtime_suspend	= vpss_runtime_suspend,
	.runtime_resume		= vpss_runtime_resume,
};

static const char * const vpss_clk_names[] = {
	"sys0", "sys1", "sys2", "img_v", "sc_top", "sc_v1",
};

static int vpss_probe(struct platform_device *pdev)
{
	struct vpss_dev *vpss;
	struct video_device *vfd;
	int ret, i;

	vpss = devm_kzalloc(&pdev->dev, sizeof(*vpss), GFP_KERNEL);
	if (!vpss)
		return -ENOMEM;

	vpss->dev = &pdev->dev;
	mutex_init(&vpss->lock);
	spin_lock_init(&vpss->irq_lock);
	platform_set_drvdata(pdev, vpss);

	vpss->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(vpss->regs))
		return PTR_ERR(vpss->regs);

	vpss->irq = platform_get_irq(pdev, 0);
	if (vpss->irq < 0)
		return vpss->irq;

	ret = devm_request_irq(&pdev->dev, vpss->irq, vpss_isr, IRQF_SHARED,
			       dev_name(&pdev->dev), vpss);
	if (ret)
		return ret;

	vpss->num_clks = ARRAY_SIZE(vpss_clk_names);
	vpss->clks = devm_kcalloc(&pdev->dev, vpss->num_clks,
				  sizeof(*vpss->clks), GFP_KERNEL);
	if (!vpss->clks)
		return -ENOMEM;
	for (i = 0; i < vpss->num_clks; i++)
		vpss->clks[i].id = vpss_clk_names[i];
	ret = devm_clk_bulk_get(&pdev->dev, vpss->num_clks, vpss->clks);
	if (ret)
		return ret;

	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(40));
	if (ret)
		return ret;

	ret = v4l2_device_register(&pdev->dev, &vpss->v4l2_dev);
	if (ret)
		return ret;

	vpss->m2m_dev = v4l2_m2m_init(&vpss_m2m_ops);
	if (IS_ERR(vpss->m2m_dev)) {
		ret = PTR_ERR(vpss->m2m_dev);
		goto err_v4l2;
	}

	vfd = &vpss->vfd;
	vfd->fops = &vpss_fops;
	vfd->ioctl_ops = &vpss_ioctl_ops;
	vfd->release = video_device_release_empty;
	vfd->lock = &vpss->lock;
	vfd->v4l2_dev = &vpss->v4l2_dev;
	vfd->vfl_dir = VFL_DIR_M2M;
	strscpy(vfd->name, DRIVER_NAME, sizeof(vfd->name));
	video_set_drvdata(vfd, vpss);
	vfd->device_caps = V4L2_CAP_VIDEO_M2M | V4L2_CAP_STREAMING;

	ret = video_register_device(vfd, VFL_TYPE_VIDEO, -1);
	if (ret)
		goto err_m2m;

	ret = of_reserved_mem_device_init(&pdev->dev);
	if (ret && ret != -ENODEV)
		dev_warn(&pdev->dev, "failed to initialize reserved memory: %d\n",
			 ret);

	pm_runtime_enable(&pdev->dev);

	dev_info(&pdev->dev, "registered as /dev/video%d\n", vfd->num);
	return 0;

err_m2m:
	v4l2_m2m_release(vpss->m2m_dev);
err_v4l2:
	v4l2_device_unregister(&vpss->v4l2_dev);
	return ret;
}

static void vpss_remove(struct platform_device *pdev)
{
	struct vpss_dev *vpss = platform_get_drvdata(pdev);

	of_reserved_mem_device_release(&pdev->dev);

	pm_runtime_disable(&pdev->dev);
	video_unregister_device(&vpss->vfd);
	v4l2_m2m_release(vpss->m2m_dev);
	v4l2_device_unregister(&vpss->v4l2_dev);
}

static const struct of_device_id vpss_of_match[] = {
	{ .compatible = "sophgo,sg2002-vpss" },
	{ }
};
MODULE_DEVICE_TABLE(of, vpss_of_match);

static struct platform_driver vpss_driver = {
	.probe	= vpss_probe,
	.remove	= vpss_remove,
	.driver	= {
		.name		= DRIVER_NAME,
		.of_match_table	= vpss_of_match,
		.pm		= &vpss_pm_ops,
	},
};
module_platform_driver(vpss_driver);

MODULE_DESCRIPTION("Sophgo SG2002 VPSS scaler/CSC mem2mem driver");
MODULE_AUTHOR("George Whewell <georgerw@gmail.com>");
MODULE_LICENSE("GPL");
