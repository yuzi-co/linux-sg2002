// SPDX-License-Identifier: GPL-2.0-only
/*
 * Minimal SG2002 CSI-2 capture driver
 *
 * This is deliberately a narrow bring-up driver for the NanoKVM capture
 * path: one CSI-2 D-PHY input, one to four lanes, and the VI FE0 CSI
 * bridge DMA engine. The frame geometry is adopted from the bound
 * source: the NanoKVM's LT6911UXE HDMI bridge (UYVY 1920x1080) or a RAW
 * Bayer sensor such as the LicheeRV Nano's GC4653 (SRGGB10 2560x1440,
 * packed 4 px / 5 B by the bridge DMA). The register programming is
 * derived from the vendor cif and vi drivers.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/spinlock.h>
#include <linux/videodev2.h>
#include <linux/workqueue.h>

#include <media/v4l2-async.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mc.h>
#include <media/v4l2-subdev.h>
#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-v4l2.h>

#define SG2002_DRV_NAME		"sg2002-capture"

#define SG2002_WIDTH		1920
#define SG2002_HEIGHT		1080
#define SG2002_BYTES_PER_LINE	ALIGN(SG2002_WIDTH * 2, 16)
#define SG2002_SIZE_IMAGE	(SG2002_BYTES_PER_LINE * SG2002_HEIGHT)

/* CSI MAC0 register blocks. */
#define MAC_TOP_CTRL		0x000
#define MAC_TOP_MODE		GENMASK(2, 0)
#define MAC_TOP_CSI_ENABLE	BIT(4)
#define MAC_TOP_VS_INV		BIT(5)
#define MAC_TOP_HS_INV		BIT(6)

#define MAC_CSI_BASE		0x400
#define MAC_CSI_CTRL		(MAC_CSI_BASE + 0x00)
#define MAC_CSI_LANE_MODE	GENMASK(2, 0)
#define MAC_CSI_IRQ_CTRL	(MAC_CSI_BASE + 0x04)
#define MAC_CSI_IRQ_MASK	GENMASK(7, 0)
#define MAC_CSI_IRQ_CLEAR	GENMASK(15, 8)
#define MAC_CSI_IRQ_ECC_ERR	BIT(0)
#define MAC_CSI_IRQ_CRC_ERR	BIT(1)
#define MAC_CSI_IRQ_HDR_ERR	BIT(2)
#define MAC_CSI_IRQ_WC_ERR	BIT(3)
#define MAC_CSI_IRQ_FIFO_FULL	BIT(4)
#define MAC_CSI_IRQ_ERRORS	GENMASK(4, 0)
#define MAC_CSI_IRQ_FATAL	(MAC_CSI_IRQ_CRC_ERR | \
				 MAC_CSI_IRQ_WC_ERR | MAC_CSI_IRQ_FIFO_FULL)
#define MAC_CSI_IRQ_RESERVED	GENMASK(7, 5)
#define MAC_CSI_VC_MAP		(MAC_CSI_BASE + 0x18)
#define MAC_CSI_IRQ_STATUS	(MAC_CSI_BASE + 0x60)
#define MAC_CSI_VGEN		(MAC_CSI_BASE + 0x70)
#define MAC_CSI_VGEN_MODE	GENMASK(1, 0)

/* CSI wrapper register blocks. */
#define WRAP_TOP_POWER		0x000
#define WRAP_TOP_PD_IBIAS	BIT(14)
#define WRAP_TOP_PD_RXLP	GENMASK(21, 16)
#define WRAP_TOP_ROUTE		0x004
#define WRAP_TOP_CLK_P0_TO_P1	BIT(6)
#define WRAP_TOP_CLK_P1_TO_P0	BIT(7)
#define WRAP_TOP_CLK_CHANNEL	GENMASK(21, 16)
#define WRAP_TOP_DESKEW_FORCE	0x080
#define WRAP_TOP_CLOCK_INVERT	GENMASK(5, 0)
#define WRAP_TOP_DESKEW_CODE03	0x084
#define WRAP_TOP_DESKEW_CODE45	0x088

#define WRAP_PHY4_BASE		0x300
#define WRAP_PHY4_MODE		(WRAP_PHY4_BASE + 0x00)
#define WRAP_PHY4_DATA_SEL	(WRAP_PHY4_BASE + 0x04)
#define WRAP_PHY4_CLK_SEL	(WRAP_PHY4_BASE + 0x08)
#define WRAP_PHY4_LANE_ENABLE	(WRAP_PHY4_BASE + 0x0c)
#define WRAP_PHY4_HS_SETTLE	(WRAP_PHY4_BASE + 0x10)
#define WRAP_PHY4_CLOCK_SEL	GENMASK(2, 0)
#define WRAP_PHY4_CLOCK_SWAP	BIT(4)
#define WRAP_PHY4_DATA_SWAP	GENMASK(11, 8)
#define WRAP_PHY4_DESKEW_LANES	GENMASK(3, 0)
#define WRAP_PHY4_SETTLE		GENMASK(7, 0)
#define WRAP_PHY4_AUTO_IGNORE	BIT(16)
#define WRAP_PHY4_AUTO_SYNC	BIT(17)

/* VI FE0 CSI bridge and DMA6. */
#define VI_CSIBDG0		0x00800
#define VI_CSIBDG_TOP_CTRL	(VI_CSIBDG0 + 0x00)
#define VI_CSIBDG_INT_CTRL	(VI_CSIBDG0 + 0x04)
#define VI_CSIBDG_CH0_SIZE	(VI_CSIBDG0 + 0x10)
#define VI_CSIBDG_INT_STATUS0	(VI_CSIBDG0 + 0xe0)
#define VI_CSIBDG_INT_STATUS1	(VI_CSIBDG0 + 0xe4)
#define VI_CSIBDG_CSI_MODE	GENMASK(1, 0)
#define VI_CSIBDG_YUV_INPUT	BIT(2)
#define VI_CSIBDG_CH0_DMA	BIT(6)
#define VI_CSIBDG_CSI_ENABLE	BIT(24)
#define VI_CSIBDG_UPDATE		BIT(31)
#define VI_CSIBDG_WIDTH		GENMASK(12, 0)
#define VI_CSIBDG_HEIGHT		GENMASK(28, 16)

#define VI_PRE_RAW_FRAME_VALID	0x00028
#define VI_PRE_RAW_FE0_FRAME_VALID BIT(0)
#define VI_PRE_RAW_FE0_PQ_VALID	BIT(4)
#define VI_PRE_RAW_FE0_VALID	(VI_PRE_RAW_FE0_FRAME_VALID | \
				 VI_PRE_RAW_FE0_PQ_VALID)

#define VI_DMA6			0x00b00
#define VI_DMA_SYS_CTRL		(VI_DMA6 + 0x00)
#define VI_DMA_BASE_ADDR		(VI_DMA6 + 0x04)
#define VI_DMA_SEG_LEN		(VI_DMA6 + 0x08)
#define VI_DMA_STRIDE		(VI_DMA6 + 0x0c)
#define VI_DMA_SEG_NUM		(VI_DMA6 + 0x10)
#define VI_DMA_BASE_HIGH	GENMASK(15, 8)
#define VI_DMA_BASE_SEL		BIT(16)
#define VI_DMA_STRIDE_SEL	BIT(17)
#define VI_DMA_SEG_LEN_SEL	BIT(18)
#define VI_DMA_SEG_NUM_SEL	BIT(19)

#define VI_ISP_TOP		0x70000
#define VI_ISP_INT_EVENT0	(VI_ISP_TOP + 0x00)
#define VI_ISP_INT_EVENT1	(VI_ISP_TOP + 0x04)
#define VI_ISP_INT_EVENT2	(VI_ISP_TOP + 0x08)
#define VI_ISP_INT_EVENT0_EN	(VI_ISP_TOP + 0x10)
#define VI_ISP_INT_EVENT1_EN	(VI_ISP_TOP + 0x14)
#define VI_ISP_INT_EVENT2_EN	(VI_ISP_TOP + 0x18)
#define VI_ISP_SW_CTRL0		(VI_ISP_TOP + 0x20)
#define VI_ISP_SW_CTRL1		(VI_ISP_TOP + 0x24)
#define VI_ISP_CTRL_MODE0	(VI_ISP_TOP + 0x28)
#define VI_ISP_CTRL_MODE1	(VI_ISP_TOP + 0x2c)
#define VI_ISP_SCENARIOS_CTRL	(VI_ISP_TOP + 0x30)
#define VI_ISP_SW_RST		(VI_ISP_TOP + 0x34)
#define VI_ISP_SCENARIOS_MASK	GENMASK(22, 2)
#define VI_ISP_SCENARIO_PRE2BE_L	BIT(2)
#define VI_ISP_SCENARIO_BE_WDMA_L	BIT(9)
#define VI_ISP_SCENARIO_FE_DMA_SHARE	BIT(22)
#define VI_ISP_SCENARIOS_SINGLE_YUV \
	(VI_ISP_SCENARIO_PRE2BE_L | VI_ISP_SCENARIO_BE_WDMA_L | \
	 VI_ISP_SCENARIO_FE_DMA_SHARE)
#define VI_ISP_FRAME_DONE_FE0_CH0 BIT(0)
#define VI_ISP_FE0_HW_TRIGGER	BIT(0)
#define VI_ISP_FE0_HW_SHADOW	BIT(16)
#define VI_ISP_FE0_HW_PQ		BIT(0)

/* Video subsystem glue needed before either CSI MAC0 or VI is used. */
#define VIP_SYS_RESET		0x00
#define VIP_SYS_CSI_MAC0_RESET	BIT(12)
#define VIP_SYS_CLK_ENABLE	0x04
#define VIP_SYS_AXI_ISP_ENABLE	BIT(0)
#define VIP_SYS_AXI_SWITCH	0x10
#define VIP_SYS_AXI_OFFLINE_PORTS GENMASK(2, 0)
#define VIP_SYS_AXI_VENDOR_ROUTES (BIT(0) | BIT(1))
#define VIP_SYS_CLK_SELECT	0x1c
#define VIP_SYS_CSI_MAC0_SOURCE	GENMASK(5, 4)
#define VIP_SYS_AXI_RT_LP	0x20
#define VIP_SYS_AXI_OFFLINE_LP	0x24
#define VIP_SYS_X2P_LP		0x28
#define VIP_SYS_SC_AXI_LP	0x2c
#define VIP_SYS_LP_DISABLE	0x00000c01
#define VIP_SYS_SC_LP_DISABLE	0x0c010c01
#define VIP_SYS_CSI_MAC0_RATIO	0x38
#define VIP_SYS_CSI_MAC0_DIVIDER GENMASK(20, 16)
#define VIP_SYS_CSI_MAC0_DIVIDER_ENABLE BIT(0)
#define VIP_SYS_CSI_MAC0_UPDATE	BIT(2)

#define DDRTOP_CLK_GATING	0x14
#define DDRTOP_DISABLE_GATING	GENMASK(11, 0)

#define SG2002_HS_SETTLE		8
#define SG2002_NUM_DATA_LANES	4
#define SG2002_NUM_PHYS_LANES	6

struct sg2002_buffer {
	struct vb2_v4l2_buffer vb;
	struct list_head list;
	dma_addr_t dma_addr;
};

struct sg2002_capture {
	struct device *dev;
	void __iomem *mac;
	void __iomem *wrap;
	void __iomem *vi;
	void __iomem *vip_sys;
	struct regmap *ddr_top;
	u32 ddr_top_gating;
	bool ddr_top_gating_changed;
	u32 scenarios_ctrl;
	bool scenarios_ctrl_changed;
	int vi_irq;
	int csi_irq;

	struct clk_bulk_data *clks;
	int num_clks;
	struct reset_control *resets;

	struct media_device mdev;
	struct v4l2_device v4l2_dev;
	struct v4l2_async_notifier notifier;
	struct v4l2_subdev *source;
	unsigned int source_pad;
	struct media_pipeline pipe;

	struct video_device vdev;
	struct media_pad pad;
	struct vb2_queue queue;
	struct mutex lock;
	spinlock_t qlock;
	struct list_head queued;
	struct sg2002_buffer *active;
	unsigned int sequence;
	bool streaming;
	bool source_streaming;
	bool pipeline_started;
	bool clocks_enabled;
	bool error_pending;
	bool removing;
	struct work_struct error_work;

	void *scratch;
	dma_addr_t scratch_dma;

	struct v4l2_mbus_config_mipi_csi2 bus;

	/* Active frame geometry. Defaults are the proven NanoKVM HDMI path
	 * (LT6911UXE -> UYVY 1920x1080); a bound RAW Bayer source switches
	 * these to the sensor's mode (see sg2002_adopt_source_format). */
	u32 pixelformat;
	u32 mbus_code;
	u32 width;
	u32 height;
	u32 bytesperline;
	u32 sizeimage;
	bool yuv_input;
};

static inline struct sg2002_buffer *to_sg2002_buffer(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);

	return container_of(vbuf, struct sg2002_buffer, vb);
}

static void sg2002_set_hdmi_defaults(struct sg2002_capture *cap)
{
	cap->pixelformat = V4L2_PIX_FMT_UYVY;
	cap->mbus_code = MEDIA_BUS_FMT_UYVY8_1X16;
	cap->width = SG2002_WIDTH;
	cap->height = SG2002_HEIGHT;
	cap->bytesperline = SG2002_BYTES_PER_LINE;
	cap->sizeimage = SG2002_SIZE_IMAGE;
	cap->yuv_input = true;
}

static u32 sg2002_raw10_pixelformat(u32 code)
{
	switch (code) {
	case MEDIA_BUS_FMT_SRGGB10_1X10:
		return V4L2_PIX_FMT_SRGGB10P;
	case MEDIA_BUS_FMT_SGRBG10_1X10:
		return V4L2_PIX_FMT_SGRBG10P;
	case MEDIA_BUS_FMT_SGBRG10_1X10:
		return V4L2_PIX_FMT_SGBRG10P;
	case MEDIA_BUS_FMT_SBGGR10_1X10:
		return V4L2_PIX_FMT_SBGGR10P;
	default:
		return 0;
	}
}

/*
 * Adopt a bound RAW Bayer source's geometry. Only the four RAW10 orders
 * switch the driver off the HDMI defaults; anything else (or a failed
 * get_fmt) keeps the proven LT6911UXE behaviour byte-for-byte. The VI
 * bridge DMA moves the CSI-2 payload verbatim, so RAW10 leaves the
 * sensor packed four-pixels-per-five-bytes.
 */
static void sg2002_adopt_source_format(struct sg2002_capture *cap)
{
	struct v4l2_subdev_format fmt = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
		.pad = cap->source_pad,
	};
	u32 pixelformat;

	if (v4l2_subdev_call(cap->source, pad, get_fmt, NULL, &fmt))
		return;

	pixelformat = sg2002_raw10_pixelformat(fmt.format.code);
	if (!pixelformat)
		return;

	cap->pixelformat = pixelformat;
	cap->mbus_code = fmt.format.code;
	cap->width = fmt.format.width;
	cap->height = fmt.format.height;
	cap->bytesperline = ALIGN(cap->width * 10 / 8, 16);
	cap->sizeimage = cap->bytesperline * cap->height;
	cap->yuv_input = false;
	dev_info(cap->dev, "RAW source %ux%u code %#x -> %p4cc, %u B/line\n",
		 cap->width, cap->height, cap->mbus_code,
		 &cap->pixelformat, cap->bytesperline);
}

static void sg2002_update_bits(void __iomem *base, u32 reg, u32 mask, u32 val)
{
	u32 value = readl(base + reg);

	value &= ~mask;
	value |= val & mask;
	writel(value, base + reg);
}

static void sg2002_csi_irq_enable(struct sg2002_capture *cap, bool enable)
{
	u32 mask = enable ? MAC_CSI_IRQ_RESERVED : FIELD_MAX(MAC_CSI_IRQ_MASK);

	sg2002_update_bits(cap->mac, MAC_CSI_IRQ_CTRL,
			   MAC_CSI_IRQ_MASK | MAC_CSI_IRQ_CLEAR,
			   FIELD_PREP(MAC_CSI_IRQ_MASK, mask) |
			   FIELD_PREP(MAC_CSI_IRQ_CLEAR,
				      FIELD_MAX(MAC_CSI_IRQ_CLEAR)));
}

static void sg2002_csibdg_clear_status(struct sg2002_capture *cap)
{
	u32 status;

	status = readl(cap->vi + VI_CSIBDG_INT_STATUS0);
	if (status)
		writel(status, cap->vi + VI_CSIBDG_INT_STATUS0);
	status = readl(cap->vi + VI_CSIBDG_INT_STATUS1);
	if (status)
		writel(status, cap->vi + VI_CSIBDG_INT_STATUS1);
}

static void sg2002_ddr_restore_clock_gating(struct sg2002_capture *cap)
{
	int ret;

	if (!cap->ddr_top_gating_changed)
		return;
	ret = regmap_update_bits(cap->ddr_top, DDRTOP_CLK_GATING,
				 DDRTOP_DISABLE_GATING, cap->ddr_top_gating);
	if (ret) {
		dev_warn(cap->dev, "failed to restore DDR-top gating: %d\n", ret);
		return;
	}
	cap->ddr_top_gating_changed = false;
}

static int sg2002_ddr_disable_clock_gating(struct sg2002_capture *cap)
{
	unsigned int before;
	unsigned int after;
	int ret;

	ret = regmap_read(cap->ddr_top, DDRTOP_CLK_GATING, &before);
	if (ret)
		return ret;
	cap->ddr_top_gating = before & DDRTOP_DISABLE_GATING;
	ret = regmap_update_bits(cap->ddr_top, DDRTOP_CLK_GATING,
				 DDRTOP_DISABLE_GATING, DDRTOP_DISABLE_GATING);
	if (ret)
		return ret;
	cap->ddr_top_gating_changed = true;
	ret = regmap_read(cap->ddr_top, DDRTOP_CLK_GATING, &after);
	if (ret) {
		sg2002_ddr_restore_clock_gating(cap);
		return ret;
	}

	dev_dbg(cap->dev, "ddrtop clock gating: %#x -> %#x\n", before, after);
	return 0;
}

static void sg2002_restore_scenarios(struct sg2002_capture *cap)
{
	if (!cap->scenarios_ctrl_changed)
		return;
	writel(cap->scenarios_ctrl, cap->vi + VI_ISP_SCENARIOS_CTRL);
	cap->scenarios_ctrl_changed = false;
}

static void sg2002_configure_single_yuv_scenario(struct sg2002_capture *cap)
{
	u32 before = readl(cap->vi + VI_ISP_SCENARIOS_CTRL);
	u32 after = (before & ~VI_ISP_SCENARIOS_MASK) |
		    VI_ISP_SCENARIOS_SINGLE_YUV;

	cap->scenarios_ctrl = before;
	cap->scenarios_ctrl_changed = before != after;
	writel(after, cap->vi + VI_ISP_SCENARIOS_CTRL);
	dev_dbg(cap->dev, "VI scenario: %#x -> %#x\n", before,
		 readl(cap->vi + VI_ISP_SCENARIOS_CTRL));
}

static void sg2002_set_deskew(struct sg2002_capture *cap, unsigned int lane,
			      u8 phase)
{
	u32 reg = lane < 4 ? WRAP_TOP_DESKEW_CODE03 :
			       WRAP_TOP_DESKEW_CODE45;
	u32 shift = (lane % 4) * 8;

	sg2002_update_bits(cap->wrap, reg, GENMASK(shift + 7, shift),
			    (u32)phase << shift);
	sg2002_update_bits(cap->wrap, WRAP_TOP_DESKEW_FORCE, BIT(16 + lane),
			    phase ? BIT(16 + lane) : 0);
}

static void sg2002_vip_init(struct sg2002_capture *cap)
{
	u32 value;

	value = readl(cap->vip_sys + VIP_SYS_RESET);
	writel(value | VIP_SYS_CSI_MAC0_RESET, cap->vip_sys + VIP_SYS_RESET);
	udelay(20);
	writel(value & ~VIP_SYS_CSI_MAC0_RESET, cap->vip_sys + VIP_SYS_RESET);

	sg2002_update_bits(cap->vip_sys, VIP_SYS_CLK_ENABLE,
			   VIP_SYS_AXI_ISP_ENABLE, VIP_SYS_AXI_ISP_ENABLE);
	/*
	 * CVI_SYS_Init disables automatic low-power entry for all VIP fabrics.
	 * DMA6 reaches memory through the offline AXI and X2P paths.
	 */
	writel(VIP_SYS_LP_DISABLE, cap->vip_sys + VIP_SYS_AXI_RT_LP);
	writel(VIP_SYS_LP_DISABLE, cap->vip_sys + VIP_SYS_AXI_OFFLINE_LP);
	writel(VIP_SYS_LP_DISABLE, cap->vip_sys + VIP_SYS_X2P_LP);
	writel(VIP_SYS_SC_LP_DISABLE, cap->vip_sys + VIP_SYS_SC_AXI_LP);
	/*
	 * Match the vendor VIP base probe: SC_TOP and ISP_RAW use offline AXI
	 * ports, while ISP_YUV remains on the online port.
	 */
	sg2002_update_bits(cap->vip_sys, VIP_SYS_AXI_SWITCH,
			   VIP_SYS_AXI_OFFLINE_PORTS, VIP_SYS_AXI_VENDOR_ROUTES);

	sg2002_update_bits(cap->vip_sys, VIP_SYS_CLK_SELECT,
			   VIP_SYS_CSI_MAC0_SOURCE,
			    FIELD_PREP(VIP_SYS_CSI_MAC0_SOURCE, 2));
	/*
	 * VIP_SYS2 is 594 MHz from the boot clock tree. Ratio 31 gives the full
	 * parent rate; enable and latch the normal divider.
	 */
	sg2002_update_bits(cap->vip_sys, VIP_SYS_CSI_MAC0_RATIO,
			   VIP_SYS_CSI_MAC0_DIVIDER,
			    FIELD_PREP(VIP_SYS_CSI_MAC0_DIVIDER, 31));
	sg2002_update_bits(cap->vip_sys, VIP_SYS_CSI_MAC0_RATIO,
			   VIP_SYS_CSI_MAC0_DIVIDER_ENABLE,
			    VIP_SYS_CSI_MAC0_DIVIDER_ENABLE);
	sg2002_update_bits(cap->vip_sys, VIP_SYS_CSI_MAC0_RATIO,
			   VIP_SYS_CSI_MAC0_UPDATE, VIP_SYS_CSI_MAC0_UPDATE);
	udelay(5);
	dev_dbg(cap->dev,
		 "vip: axi=%#x lp=%#x/%#x/%#x/%#x select=%#x mac0-ratio=%#x\n",
		 readl(cap->vip_sys + VIP_SYS_AXI_SWITCH),
		 readl(cap->vip_sys + VIP_SYS_AXI_RT_LP),
		 readl(cap->vip_sys + VIP_SYS_AXI_OFFLINE_LP),
		 readl(cap->vip_sys + VIP_SYS_X2P_LP),
		 readl(cap->vip_sys + VIP_SYS_SC_AXI_LP),
		 readl(cap->vip_sys + VIP_SYS_CLK_SELECT),
		 readl(cap->vip_sys + VIP_SYS_CSI_MAC0_RATIO));
}

static void sg2002_configure_lanes(struct sg2002_capture *cap)
{
	const struct v4l2_mbus_config_mipi_csi2 *bus = &cap->bus;
	u32 data_sel = 0;
	u32 swaps = 0;
	u32 route;
	unsigned int i;

	for (i = 0; i < bus->num_data_lanes; i++) {
		data_sel |= (u32)bus->data_lanes[i] << (i * 4);
		if (bus->lane_polarities[i + 1])
			swaps |= BIT(8 + i);
	}

	writel(data_sel, cap->wrap + WRAP_PHY4_DATA_SEL);
	writel(FIELD_PREP(WRAP_PHY4_CLOCK_SEL, bus->clock_lane) |
	       (bus->lane_polarities[0] ? WRAP_PHY4_CLOCK_SWAP : 0) |
	       swaps, cap->wrap + WRAP_PHY4_CLK_SEL);

	route = readl(cap->wrap + WRAP_TOP_ROUTE);
	route &= ~(WRAP_TOP_CLK_P0_TO_P1 | WRAP_TOP_CLK_P1_TO_P0 |
		   WRAP_TOP_CLK_CHANNEL);
	route |= BIT(16 + bus->clock_lane);
	if (bus->clock_lane < 3)
		route |= WRAP_TOP_CLK_P0_TO_P1;
	else
		route |= WRAP_TOP_CLK_P1_TO_P0;
	writel(route, cap->wrap + WRAP_TOP_ROUTE);

	/* A 4-lane topology crosses the two three-wire analogue ports. */
	for (i = 0; i < SG2002_NUM_PHYS_LANES; i++) {
		u8 phase = 0;
		unsigned int j;

		if (i == bus->clock_lane) {
			phase = 0x00;
		} else {
			for (j = 0; j < bus->num_data_lanes; j++) {
				if (i != bus->data_lanes[j])
					continue;
				phase = (i < 3) == (bus->clock_lane < 3) ?
					0x03 : 0x08;
				break;
			}
		}

		sg2002_set_deskew(cap, i, phase);
	}

	/* D-PHY mode, fixed settle time, no automatic deskew. */
	sg2002_update_bits(cap->wrap, WRAP_PHY4_MODE, GENMASK(1, 0), 0);
	sg2002_update_bits(cap->wrap, WRAP_PHY4_HS_SETTLE,
			    WRAP_PHY4_SETTLE | WRAP_PHY4_AUTO_IGNORE |
			    WRAP_PHY4_AUTO_SYNC,
			    FIELD_PREP(WRAP_PHY4_SETTLE, SG2002_HS_SETTLE));
	sg2002_update_bits(cap->wrap, WRAP_TOP_DESKEW_FORCE,
			    WRAP_TOP_CLOCK_INVERT, WRAP_TOP_CLOCK_INVERT);
}

static void sg2002_csi_enable(struct sg2002_capture *cap, bool enable)
{
	u32 value;

	if (!enable) {
		sg2002_csi_irq_enable(cap, false);
		sg2002_update_bits(cap->wrap, WRAP_PHY4_LANE_ENABLE,
				    WRAP_PHY4_DESKEW_LANES, 0);
		sg2002_update_bits(cap->wrap, WRAP_TOP_POWER,
				    WRAP_TOP_PD_IBIAS | WRAP_TOP_PD_RXLP,
				    WRAP_TOP_PD_IBIAS | WRAP_TOP_PD_RXLP);
		sg2002_update_bits(cap->mac, MAC_TOP_CTRL,
				    MAC_TOP_CSI_ENABLE, 0);
		return;
	}

	/* Match cif_streaming(): force the warm PHY fully off before its
	 * selectors, analogue front end, and lane enables are changed. */
	sg2002_csi_irq_enable(cap, false);
	sg2002_update_bits(cap->wrap, WRAP_PHY4_LANE_ENABLE,
			    WRAP_PHY4_DESKEW_LANES, 0);
	sg2002_update_bits(cap->wrap, WRAP_TOP_POWER,
			    WRAP_TOP_PD_IBIAS | WRAP_TOP_PD_RXLP,
			    WRAP_TOP_PD_IBIAS | WRAP_TOP_PD_RXLP);

	sg2002_configure_lanes(cap);

	value = readl(cap->mac + MAC_TOP_CTRL);
	value &= ~MAC_TOP_MODE;
	value |= FIELD_PREP(MAC_TOP_MODE, 1) | MAC_TOP_CSI_ENABLE |
		 MAC_TOP_VS_INV | MAC_TOP_HS_INV;
	writel(value, cap->mac + MAC_TOP_CTRL);

	sg2002_update_bits(cap->mac, MAC_CSI_CTRL, MAC_CSI_LANE_MODE,
			    FIELD_PREP(MAC_CSI_LANE_MODE,
				       cap->bus.num_data_lanes - 1));
	writel(0x3210, cap->mac + MAC_CSI_VC_MAP);
	sg2002_update_bits(cap->mac, MAC_CSI_VGEN, MAC_CSI_VGEN_MODE, 0);
	sg2002_update_bits(cap->wrap, WRAP_TOP_POWER, WRAP_TOP_PD_IBIAS, 0);
	sg2002_update_bits(cap->wrap, WRAP_PHY4_LANE_ENABLE,
			    WRAP_PHY4_DESKEW_LANES, 0);
	sg2002_update_bits(cap->wrap, WRAP_TOP_POWER, WRAP_TOP_PD_RXLP, 0);
	udelay(20);
	sg2002_update_bits(cap->wrap, WRAP_PHY4_LANE_ENABLE,
			    WRAP_PHY4_DESKEW_LANES,
			    FIELD_PREP(WRAP_PHY4_DESKEW_LANES, 0xf));
}

static void sg2002_dma_set_addr(struct sg2002_capture *cap, dma_addr_t addr)
{
	u32 sys = readl(cap->vi + VI_DMA_SYS_CTRL);

	sys &= ~VI_DMA_BASE_HIGH;
	sys |= FIELD_PREP(VI_DMA_BASE_HIGH, upper_32_bits(addr));
	writel(lower_32_bits(addr), cap->vi + VI_DMA_BASE_ADDR);
	writel(sys, cap->vi + VI_DMA_SYS_CTRL);
}

static void sg2002_dma_set_hw_mode(struct sg2002_capture *cap)
{
	u32 sys = readl(cap->vi + VI_DMA_SYS_CTRL);

	sys &= ~(VI_DMA_STRIDE_SEL | VI_DMA_SEG_LEN_SEL | VI_DMA_SEG_NUM_SEL);
	sys |= VI_DMA_BASE_SEL;
	writel(sys, cap->vi + VI_DMA_SYS_CTRL);
}

static void sg2002_preraw_fe0_trigger(struct sg2002_capture *cap)
{
	writel(VI_PRE_RAW_FE0_PQ_VALID, cap->vi + VI_PRE_RAW_FRAME_VALID);
	writel(VI_PRE_RAW_FE0_VALID, cap->vi + VI_PRE_RAW_FRAME_VALID);
}

static void sg2002_vi_enable(struct sg2002_capture *cap, dma_addr_t addr)
{
	u32 ctrl;
	unsigned long flags;

	/* All three event banks feed the same level IRQ.  A stale status in
	 * an unacknowledged bank otherwise leaves the CPU in an IRQ storm. */
	writel(0, cap->vi + VI_ISP_INT_EVENT0_EN);
	writel(0, cap->vi + VI_ISP_INT_EVENT1_EN);
	writel(0, cap->vi + VI_ISP_INT_EVENT2_EN);

	/* The vendor reset order is AXI first, then all blocks, then release. */
	writel(0x40, cap->vi + VI_ISP_SW_RST);
	writel(0x37f, cap->vi + VI_ISP_SW_RST);
	writel(0x40, cap->vi + VI_ISP_SW_RST);
	writel(0, cap->vi + VI_ISP_SW_RST);

	writel(~0U, cap->vi + VI_ISP_INT_EVENT0);
	writel(~0U, cap->vi + VI_ISP_INT_EVENT1);
	writel(~0U, cap->vi + VI_ISP_INT_EVENT2);
	ctrl = FIELD_PREP(VI_CSIBDG_CSI_MODE, 1);
	if (cap->yuv_input)
		ctrl |= VI_CSIBDG_YUV_INPUT;
	writel(cap->bytesperline, cap->vi + VI_DMA_SEG_LEN);
	writel(cap->bytesperline, cap->vi + VI_DMA_STRIDE);
	writel(cap->height, cap->vi + VI_DMA_SEG_NUM);
	sg2002_dma_set_addr(cap, addr);
	ctrl |= VI_CSIBDG_CH0_DMA;
	writel(ctrl, cap->vi + VI_CSIBDG_TOP_CTRL);
	writel(0, cap->vi + VI_CSIBDG_INT_CTRL);
	writel(FIELD_PREP(VI_CSIBDG_WIDTH, cap->width - 1) |
	       FIELD_PREP(VI_CSIBDG_HEIGHT, cap->height - 1),
	       cap->vi + VI_CSIBDG_CH0_SIZE);

	sg2002_preraw_fe0_trigger(cap);
	writel(VI_ISP_FE0_HW_TRIGGER | VI_ISP_FE0_HW_SHADOW,
	       cap->vi + VI_ISP_CTRL_MODE0);
	writel(VI_ISP_FE0_HW_PQ, cap->vi + VI_ISP_CTRL_MODE1);
	sg2002_configure_single_yuv_scenario(cap);
	sg2002_dma_set_hw_mode(cap);
	writel(0x00000d01, cap->vi + VI_ISP_SW_CTRL1);
	writel(0x0d010000, cap->vi + VI_ISP_SW_CTRL0);

	sg2002_csibdg_clear_status(cap);
	spin_lock_irqsave(&cap->qlock, flags);
	WRITE_ONCE(cap->streaming, true);
	cap->error_pending = false;
	spin_unlock_irqrestore(&cap->qlock, flags);
	writel(VI_ISP_FRAME_DONE_FE0_CH0, cap->vi + VI_ISP_INT_EVENT0_EN);
	sg2002_csi_irq_enable(cap, true);
	ctrl |= VI_CSIBDG_CSI_ENABLE | VI_CSIBDG_UPDATE;
	writel(ctrl, cap->vi + VI_CSIBDG_TOP_CTRL);
}

/* Safe in hard IRQ context: stop bridge DMA and all VI completion IRQs. */
static void sg2002_vi_quiesce(struct sg2002_capture *cap)
{
	sg2002_update_bits(cap->vi, VI_CSIBDG_TOP_CTRL,
			    VI_CSIBDG_CSI_ENABLE | VI_CSIBDG_CH0_DMA |
			    VI_CSIBDG_UPDATE, 0);
	writel(0, cap->vi + VI_ISP_INT_EVENT0_EN);
	writel(0, cap->vi + VI_ISP_INT_EVENT1_EN);
	writel(0, cap->vi + VI_ISP_INT_EVENT2_EN);
}

static void sg2002_vi_disable(struct sg2002_capture *cap)
{
	sg2002_vi_quiesce(cap);
	writel(~0U, cap->vi + VI_ISP_INT_EVENT0);
	writel(~0U, cap->vi + VI_ISP_INT_EVENT1);
	writel(~0U, cap->vi + VI_ISP_INT_EVENT2);
	writel(0, cap->vi + VI_PRE_RAW_FRAME_VALID);
	sg2002_csibdg_clear_status(cap);
	sg2002_restore_scenarios(cap);
}

static struct sg2002_buffer *sg2002_next_buffer(struct sg2002_capture *cap)
{
	struct sg2002_buffer *buf;

	buf = list_first_entry_or_null(&cap->queued, struct sg2002_buffer, list);
	if (buf)
		list_del_init(&buf->list);

	return buf;
}

static void sg2002_return_buffers(struct sg2002_capture *cap,
				  enum vb2_buffer_state state)
{
	struct sg2002_buffer *buf;
	unsigned long flags;

	for (;;) {
		spin_lock_irqsave(&cap->qlock, flags);
		buf = cap->active;
		cap->active = NULL;
		if (!buf)
			buf = sg2002_next_buffer(cap);
		spin_unlock_irqrestore(&cap->qlock, flags);

		if (!buf)
			break;
		vb2_buffer_done(&buf->vb.vb2_buf, state);
	}
}

/* The caller holds cap->lock, which also protects cap->source. */
static void sg2002_stop_capture(struct sg2002_capture *cap,
				enum vb2_buffer_state state)
{
	unsigned long flags;

	spin_lock_irqsave(&cap->qlock, flags);
	WRITE_ONCE(cap->streaming, false);
	cap->error_pending = false;
	spin_unlock_irqrestore(&cap->qlock, flags);

	if (cap->clocks_enabled) {
		sg2002_vi_disable(cap);
		sg2002_csi_enable(cap, false);
		synchronize_irq(cap->vi_irq);
		synchronize_irq(cap->csi_irq);

		if (cap->source_streaming) {
			v4l2_subdev_disable_streams(cap->source, cap->source_pad,
						    BIT(0));
			cap->source_streaming = false;
		}
	}

	sg2002_return_buffers(cap, state);
	if (cap->scratch)
		dma_free_coherent(cap->dev, cap->sizeimage, cap->scratch,
				  cap->scratch_dma);
	cap->scratch = NULL;
	if (cap->clocks_enabled) {
		sg2002_ddr_restore_clock_gating(cap);
		clk_bulk_disable_unprepare(cap->num_clks, cap->clks);
		cap->clocks_enabled = false;
	}
	if (cap->pipeline_started) {
		video_device_pipeline_stop(&cap->vdev);
		cap->pipeline_started = false;
	}
}

static void sg2002_error_work(struct work_struct *work)
{
	struct sg2002_capture *cap =
		container_of(work, struct sg2002_capture, error_work);
	unsigned long flags;
	bool pending;

	mutex_lock(&cap->lock);
	spin_lock_irqsave(&cap->qlock, flags);
	pending = cap->error_pending;
	spin_unlock_irqrestore(&cap->qlock, flags);
	if (pending)
		sg2002_stop_capture(cap, VB2_BUF_STATE_ERROR);
	mutex_unlock(&cap->lock);
}

/* Safe in hard IRQ context: defer the sleeping stream teardown. */
static void sg2002_capture_error(struct sg2002_capture *cap)
{
	unsigned long flags;
	bool streaming;

	spin_lock_irqsave(&cap->qlock, flags);
	streaming = cap->streaming;
	if (streaming) {
		WRITE_ONCE(cap->streaming, false);
		cap->error_pending = true;
		/* Wake userspace before the stream's buffers are returned. */
		vb2_queue_error(&cap->queue);
	}
	spin_unlock_irqrestore(&cap->qlock, flags);
	if (!streaming)
		return;

	sg2002_csi_irq_enable(cap, false);
	sg2002_vi_quiesce(cap);
	schedule_work(&cap->error_work);
}

static irqreturn_t sg2002_irq(int irq, void *data)
{
	struct sg2002_capture *cap = data;
	struct sg2002_buffer *done, *next;
	unsigned long flags;
	u32 status0, status1, status2;

	status0 = readl(cap->vi + VI_ISP_INT_EVENT0);
	status1 = readl(cap->vi + VI_ISP_INT_EVENT1);
	status2 = readl(cap->vi + VI_ISP_INT_EVENT2);
	writel(status0, cap->vi + VI_ISP_INT_EVENT0);
	writel(status1, cap->vi + VI_ISP_INT_EVENT1);
	writel(status2, cap->vi + VI_ISP_INT_EVENT2);
	if (!(status0 | status1 | status2))
		return IRQ_NONE;
	if (!(status0 & VI_ISP_FRAME_DONE_FE0_CH0))
		return IRQ_HANDLED;
	if (!READ_ONCE(cap->streaming))
		return IRQ_HANDLED;

	spin_lock_irqsave(&cap->qlock, flags);
	/* An error may have stopped the stream while this IRQ waited on qlock. */
	if (!READ_ONCE(cap->streaming)) {
		spin_unlock_irqrestore(&cap->qlock, flags);
		return IRQ_HANDLED;
	}
	done = cap->active;
	next = sg2002_next_buffer(cap);
	cap->active = next;
	sg2002_dma_set_addr(cap, next ? next->dma_addr : cap->scratch_dma);
	sg2002_preraw_fe0_trigger(cap);
	if (done) {
		done->vb.sequence = cap->sequence++;
		done->vb.field = V4L2_FIELD_NONE;
		done->vb.vb2_buf.timestamp = ktime_get_ns();
	}
	spin_unlock_irqrestore(&cap->qlock, flags);

	if (done)
		vb2_buffer_done(&done->vb.vb2_buf, VB2_BUF_STATE_DONE);

	return IRQ_HANDLED;
}

static irqreturn_t sg2002_csi_irq(int irq, void *data)
{
	struct sg2002_capture *cap = data;
	u32 status;

	status = readl(cap->mac + MAC_CSI_IRQ_STATUS) & MAC_CSI_IRQ_ERRORS;
	if (!status)
		return IRQ_NONE;

	if (!READ_ONCE(cap->streaming)) {
		sg2002_csi_irq_enable(cap, false);
		return IRQ_HANDLED;
	}

	sg2002_update_bits(cap->mac, MAC_CSI_IRQ_CTRL, MAC_CSI_IRQ_CLEAR,
			    FIELD_PREP(MAC_CSI_IRQ_CLEAR, status));
	dev_warn_ratelimited(cap->dev,
			     "CSI errors %#x%s%s%s%s%s\n", status,
			     status & MAC_CSI_IRQ_ECC_ERR ? " ECC" : "",
			     status & MAC_CSI_IRQ_CRC_ERR ? " CRC" : "",
			     status & MAC_CSI_IRQ_HDR_ERR ? " header" : "",
			     status & MAC_CSI_IRQ_WC_ERR ? " word-count" : "",
			     status & MAC_CSI_IRQ_FIFO_FULL ? " FIFO-full" : "");
	if (status & MAC_CSI_IRQ_FATAL) {
		/*
		 * A FIFO overrun loses byte alignment, while CRC and word-count
		 * failures make the fixed DMA frame geometry untrustworthy. Stop
		 * DMA immediately, then do the sleeping source and VB2 teardown in
		 * process context.
		 */
		sg2002_capture_error(cap);
	}
	return IRQ_HANDLED;
}

static int sg2002_queue_setup(struct vb2_queue *q, unsigned int *num_buffers,
			      unsigned int *num_planes, unsigned int sizes[],
			      struct device *alloc_devs[])
{
	struct sg2002_capture *cap = vb2_get_drv_priv(q);

	if (*num_planes)
		return sizes[0] < cap->sizeimage ? -EINVAL : 0;

	*num_planes = 1;
	sizes[0] = cap->sizeimage;
	return 0;
}

static int sg2002_buf_prepare(struct vb2_buffer *vb)
{
	struct sg2002_capture *cap = vb2_get_drv_priv(vb->vb2_queue);
	struct sg2002_buffer *buf = to_sg2002_buffer(vb);

	if (vb2_plane_size(vb, 0) < cap->sizeimage)
		return -EINVAL;

	buf->dma_addr = vb2_dma_contig_plane_dma_addr(vb, 0);
	vb2_set_plane_payload(vb, 0, cap->sizeimage);
	return 0;
}

static void sg2002_buf_queue(struct vb2_buffer *vb)
{
	struct sg2002_capture *cap = vb2_get_drv_priv(vb->vb2_queue);
	struct sg2002_buffer *buf = to_sg2002_buffer(vb);
	unsigned long flags;

	spin_lock_irqsave(&cap->qlock, flags);
	list_add_tail(&buf->list, &cap->queued);
	spin_unlock_irqrestore(&cap->qlock, flags);
}

static int sg2002_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct sg2002_capture *cap = vb2_get_drv_priv(q);
	struct sg2002_buffer *first;
	struct v4l2_dv_timings timings;
	unsigned long flags;
	int i;
	int ret;

	if (cap->removing) {
		ret = -ENODEV;
		goto err_buffers;
	}
	if (!cap->source) {
		ret = -ENOLINK;
		goto err_buffers;
	}

	/*
	 * LT6911UXC has no HPD interrupt. Its query_dv_timings operation polls
	 * the current mode and updates its active source-pad format, which must
	 * happen before pipeline link validation reads that format.
	 */
	ret = v4l2_subdev_call(cap->source, pad, query_dv_timings,
			       cap->source_pad, &timings);
	if (ret && ret != -ENOIOCTLCMD)
		goto err_buffers;

	ret = video_device_pipeline_start(&cap->vdev, &cap->pipe);
	if (ret)
		goto err_buffers;
	cap->pipeline_started = true;

	ret = clk_bulk_prepare_enable(cap->num_clks, cap->clks);
	if (ret)
		goto err_stop;
	cap->clocks_enabled = true;
	for (i = 0; i < cap->num_clks; i++)
		dev_dbg(cap->dev, "clock %s: %lu Hz\n", cap->clks[i].id,
			 clk_get_rate(cap->clks[i].clk));

	/* CVI_SYS_Init disables all twelve DDR-top path gates before VI is
	 * opened.  Preserve unrelated bits and apply that runtime policy here. */
	ret = sg2002_ddr_disable_clock_gating(cap);
	if (ret)
		goto err_stop;

	ret = reset_control_assert(cap->resets);
	if (ret)
		goto err_stop;
	udelay(5);
	ret = reset_control_deassert(cap->resets);
	if (ret)
		goto err_stop;
	sg2002_vip_init(cap);

	cap->scratch = dma_alloc_coherent(cap->dev, cap->sizeimage,
					  &cap->scratch_dma, GFP_KERNEL);
	if (!cap->scratch) {
		ret = -ENOMEM;
		goto err_stop;
	}

	spin_lock_irqsave(&cap->qlock, flags);
	first = sg2002_next_buffer(cap);
	cap->active = first;
	cap->sequence = 0;
	spin_unlock_irqrestore(&cap->qlock, flags);
	if (!first) {
		ret = -ENOBUFS;
		goto err_stop;
	}

	ret = v4l2_subdev_enable_streams(cap->source, cap->source_pad, BIT(0));
	if (ret && ret != -ENOIOCTLCMD)
		goto err_stop;
	cap->source_streaming = true;

	sg2002_csi_enable(cap, true);
	sg2002_vi_enable(cap, first->dma_addr);
	return 0;

err_stop:
	sg2002_stop_capture(cap, VB2_BUF_STATE_QUEUED);
	return ret;
err_buffers:
	sg2002_return_buffers(cap, VB2_BUF_STATE_QUEUED);
	return ret;
}

static void sg2002_stop_streaming(struct vb2_queue *q)
{
	struct sg2002_capture *cap = vb2_get_drv_priv(q);

	sg2002_stop_capture(cap, VB2_BUF_STATE_ERROR);
}

static const struct vb2_ops sg2002_vb2_ops = {
	.queue_setup = sg2002_queue_setup,
	.buf_prepare = sg2002_buf_prepare,
	.buf_queue = sg2002_buf_queue,
	.start_streaming = sg2002_start_streaming,
	.stop_streaming = sg2002_stop_streaming,
};

static void sg2002_fill_pix_format(struct sg2002_capture *cap,
				   struct v4l2_pix_format *pix)
{
	pix->width = cap->width;
	pix->height = cap->height;
	pix->pixelformat = cap->pixelformat;
	pix->field = V4L2_FIELD_NONE;
	pix->bytesperline = cap->bytesperline;
	pix->sizeimage = cap->sizeimage;
	if (cap->yuv_input) {
		pix->colorspace = V4L2_COLORSPACE_REC709;
		pix->ycbcr_enc = V4L2_YCBCR_ENC_709;
		pix->quantization = V4L2_QUANTIZATION_LIM_RANGE;
		pix->xfer_func = V4L2_XFER_FUNC_709;
	} else {
		pix->colorspace = V4L2_COLORSPACE_RAW;
		pix->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
		pix->quantization = V4L2_QUANTIZATION_FULL_RANGE;
		pix->xfer_func = V4L2_XFER_FUNC_NONE;
	}
}

static int sg2002_querycap(struct file *file, void *priv,
			   struct v4l2_capability *cap)
{
	strscpy(cap->driver, SG2002_DRV_NAME, sizeof(cap->driver));
	strscpy(cap->card, "SG2002 CSI capture", sizeof(cap->card));
	return 0;
}

static int sg2002_enum_fmt(struct file *file, void *priv,
			   struct v4l2_fmtdesc *fmt)
{
	struct sg2002_capture *cap = video_drvdata(file);

	if (fmt->index)
		return -EINVAL;

	fmt->pixelformat = cap->pixelformat;
	return 0;
}

static int sg2002_g_fmt(struct file *file, void *priv, struct v4l2_format *fmt)
{
	struct sg2002_capture *cap = video_drvdata(file);

	sg2002_fill_pix_format(cap, &fmt->fmt.pix);
	return 0;
}

static int sg2002_try_fmt(struct file *file, void *priv,
			  struct v4l2_format *fmt)
{
	struct sg2002_capture *cap = video_drvdata(file);

	sg2002_fill_pix_format(cap, &fmt->fmt.pix);
	return 0;
}

static int sg2002_s_fmt(struct file *file, void *priv, struct v4l2_format *fmt)
{
	struct sg2002_capture *cap = video_drvdata(file);

	if (vb2_is_busy(&cap->queue))
		return -EBUSY;

	return sg2002_try_fmt(file, priv, fmt);
}

static int sg2002_subscribe_event(struct v4l2_fh *fh,
				  const struct v4l2_event_subscription *sub)
{
	if (sub->type != V4L2_EVENT_SOURCE_CHANGE)
		return -EINVAL;

	return v4l2_src_change_event_subscribe(fh, sub);
}

static int sg2002_enum_framesizes(struct file *file, void *priv,
				  struct v4l2_frmsizeenum *fsize)
{
	struct sg2002_capture *cap = video_drvdata(file);

	if (fsize->index || fsize->pixel_format != cap->pixelformat)
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete.width = cap->width;
	fsize->discrete.height = cap->height;
	return 0;
}

static const struct v4l2_ioctl_ops sg2002_ioctl_ops = {
	.vidioc_querycap = sg2002_querycap,
	.vidioc_enum_fmt_vid_cap = sg2002_enum_fmt,
	.vidioc_g_fmt_vid_cap = sg2002_g_fmt,
	.vidioc_try_fmt_vid_cap = sg2002_try_fmt,
	.vidioc_s_fmt_vid_cap = sg2002_s_fmt,
	.vidioc_enum_framesizes = sg2002_enum_framesizes,
	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_prepare_buf = vb2_ioctl_prepare_buf,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_expbuf = vb2_ioctl_expbuf,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,
	.vidioc_subscribe_event = sg2002_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

static const struct v4l2_file_operations sg2002_fops = {
	.owner = THIS_MODULE,
	.open = v4l2_fh_open,
	.release = vb2_fop_release,
	.poll = vb2_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap = vb2_fop_mmap,
};

static int sg2002_link_validate(struct media_link *link)
{
	struct video_device *vdev =
		media_entity_to_video_device(link->sink->entity);
	struct sg2002_capture *cap = video_get_drvdata(vdev);
	struct v4l2_subdev *source;
	struct v4l2_subdev_format source_fmt = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
		.pad = link->source->index,
	};
	int ret;

	if (!is_media_entity_v4l2_subdev(link->source->entity))
		return -EPIPE;

	source = media_entity_to_v4l2_subdev(link->source->entity);
	ret = v4l2_subdev_call(source, pad, get_fmt, NULL, &source_fmt);
	if (ret)
		return ret;

	if (source_fmt.format.width == cap->width &&
	    source_fmt.format.height == cap->height &&
	    source_fmt.format.code == cap->mbus_code &&
	    source_fmt.format.field == V4L2_FIELD_NONE)
		return 0;

	dev_dbg(cap->dev,
		"link '%s':%u -> '%s':%u has unsupported format %ux%u code %#x field %u\n",
		link->source->entity->name, link->source->index,
		link->sink->entity->name, link->sink->index,
		source_fmt.format.width, source_fmt.format.height,
		source_fmt.format.code, source_fmt.format.field);

	return -EPIPE;
}

static const struct media_entity_operations sg2002_entity_ops = {
	.link_validate = sg2002_link_validate,
};

static void sg2002_notify(struct v4l2_subdev *sd, unsigned int notification,
			  void *arg)
{
	struct sg2002_capture *cap =
		container_of(sd->v4l2_dev, struct sg2002_capture, v4l2_dev);
	struct media_pad *source;
	struct v4l2_event *event = arg;

	if (notification != V4L2_DEVICE_NOTIFY_EVENT ||
	    event->type != V4L2_EVENT_SOURCE_CHANGE)
		return;

	source = media_pad_remote_pad_first(&cap->pad);
	if (!source || source->entity != &sd->entity)
		return;

	/*
	 * DMA geometry is fixed at 1920x1080 UYVY. A source change while it is
	 * running cannot be applied safely, so stop DMA and fail the queue until
	 * userspace negotiates the fixed format again.
	 */
	if (READ_ONCE(cap->streaming))
		sg2002_capture_error(cap);

	v4l2_event_queue(&cap->vdev, event);
}

static int sg2002_notify_bound(struct v4l2_async_notifier *notifier,
			       struct v4l2_subdev *subdev,
			       struct v4l2_async_connection *asc)
{
	struct sg2002_capture *cap =
		container_of(notifier, struct sg2002_capture, notifier);
	struct media_pad *source;
	int ret;

	ret = v4l2_create_fwnode_links_to_pad(subdev, &cap->pad,
					      MEDIA_LNK_FL_ENABLED |
					      MEDIA_LNK_FL_IMMUTABLE);
	if (ret)
		return ret;

	source = media_pad_remote_pad_unique(&cap->pad);
	if (IS_ERR(source))
		return PTR_ERR(source);

	mutex_lock(&cap->lock);
	cap->source = subdev;
	cap->source_pad = source->index;
	sg2002_adopt_source_format(cap);
	mutex_unlock(&cap->lock);
	dev_info(cap->dev, "bound source %s pad %u\n", subdev->name,
		 cap->source_pad);
	return 0;
}

static void sg2002_notify_unbind(struct v4l2_async_notifier *notifier,
				 struct v4l2_subdev *subdev,
				 struct v4l2_async_connection *asc)
{
	struct sg2002_capture *cap =
		container_of(notifier, struct sg2002_capture, notifier);

	mutex_lock(&cap->lock);
	if (cap->source == subdev) {
		/* The source remains valid until this callback returns. */
		if (cap->clocks_enabled)
			sg2002_capture_error(cap);
		sg2002_stop_capture(cap, VB2_BUF_STATE_ERROR);
		cap->source = NULL;
	}
	mutex_unlock(&cap->lock);
}

static int sg2002_notify_complete(struct v4l2_async_notifier *notifier)
{
	struct sg2002_capture *cap =
		container_of(notifier, struct sg2002_capture, notifier);

	return v4l2_device_register_subdev_nodes(&cap->v4l2_dev);
}

static const struct v4l2_async_notifier_operations sg2002_notify_ops = {
	.bound = sg2002_notify_bound,
	.unbind = sg2002_notify_unbind,
	.complete = sg2002_notify_complete,
};

static int sg2002_parse_endpoint(struct sg2002_capture *cap,
				 struct fwnode_handle **endpoint)
{
	struct v4l2_fwnode_endpoint ep = {
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	bool used[SG2002_NUM_PHYS_LANES] = { };
	unsigned int i;
	int ret;

	*endpoint = fwnode_graph_get_endpoint_by_id(dev_fwnode(cap->dev), 0, 0,
						    FWNODE_GRAPH_ENDPOINT_NEXT);
	if (!*endpoint)
		return -ENOTCONN;

	ret = v4l2_fwnode_endpoint_parse(*endpoint, &ep);
	if (ret)
		return ret;

	if (ep.bus_type != V4L2_MBUS_CSI2_DPHY ||
	    !ep.bus.mipi_csi2.num_data_lanes ||
	    ep.bus.mipi_csi2.num_data_lanes > SG2002_NUM_DATA_LANES)
		return -EINVAL;

	if (ep.bus.mipi_csi2.clock_lane >= SG2002_NUM_PHYS_LANES)
		return -EINVAL;
	used[ep.bus.mipi_csi2.clock_lane] = true;

	for (i = 0; i < ep.bus.mipi_csi2.num_data_lanes; i++) {
		unsigned int lane = ep.bus.mipi_csi2.data_lanes[i];

		if (lane >= SG2002_NUM_PHYS_LANES || used[lane])
			return -EINVAL;
		used[lane] = true;
	}

	cap->bus = ep.bus.mipi_csi2;
	return 0;
}

static int sg2002_register_media(struct sg2002_capture *cap,
				 struct fwnode_handle *endpoint)
{
	struct v4l2_async_connection *asc;
	struct vb2_queue *q = &cap->queue;
	struct video_device *vdev = &cap->vdev;
	int ret;

	strscpy(cap->mdev.model, "Sophgo SG2002 CSI", sizeof(cap->mdev.model));
	cap->mdev.dev = cap->dev;
	media_device_init(&cap->mdev);

	cap->v4l2_dev.mdev = &cap->mdev;
	ret = v4l2_device_register(cap->dev, &cap->v4l2_dev);
	if (ret)
		goto err_media_cleanup;
	cap->v4l2_dev.notify = sg2002_notify;

	ret = media_device_register(&cap->mdev);
	if (ret)
		goto err_v4l2_unregister;

	mutex_init(&cap->lock);
	spin_lock_init(&cap->qlock);
	INIT_LIST_HEAD(&cap->queued);
	INIT_WORK(&cap->error_work, sg2002_error_work);

	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP | VB2_DMABUF;
	q->lock = &cap->lock;
	q->drv_priv = cap;
	q->buf_struct_size = sizeof(struct sg2002_buffer);
	q->ops = &sg2002_vb2_ops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->min_queued_buffers = 2;
	q->dev = cap->dev;
	ret = vb2_queue_init(q);
	if (ret)
		goto err_media_unregister;

	strscpy(vdev->name, "SG2002 CSI capture", sizeof(vdev->name));
	vdev->fops = &sg2002_fops;
	vdev->ioctl_ops = &sg2002_ioctl_ops;
	vdev->v4l2_dev = &cap->v4l2_dev;
	vdev->queue = q;
	vdev->lock = &cap->lock;
	vdev->release = video_device_release_empty;
	vdev->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	vdev->vfl_dir = VFL_DIR_RX;
	vdev->entity.ops = &sg2002_entity_ops;
	video_set_drvdata(vdev, cap);

	cap->pad.flags = MEDIA_PAD_FL_SINK | MEDIA_PAD_FL_MUST_CONNECT;
	ret = media_entity_pads_init(&vdev->entity, 1, &cap->pad);
	if (ret)
		goto err_queue_release;

	ret = video_register_device(vdev, VFL_TYPE_VIDEO, -1);
	if (ret)
		goto err_entity_cleanup;

	v4l2_async_nf_init(&cap->notifier, &cap->v4l2_dev);
	asc = v4l2_async_nf_add_fwnode_remote(&cap->notifier, endpoint,
					      struct v4l2_async_connection);
	if (IS_ERR(asc)) {
		ret = PTR_ERR(asc);
		goto err_video_unregister;
	}

	cap->notifier.ops = &sg2002_notify_ops;
	ret = v4l2_async_nf_register(&cap->notifier);
	if (ret)
		goto err_notifier_cleanup;

	return 0;

err_notifier_cleanup:
	v4l2_async_nf_cleanup(&cap->notifier);
err_video_unregister:
	vb2_video_unregister_device(vdev);
	goto err_entity_cleanup_unregistered;
err_entity_cleanup:
	media_entity_cleanup(&vdev->entity);
err_queue_release:
	vb2_queue_release(q);
	goto err_media_unregister;
err_entity_cleanup_unregistered:
	media_entity_cleanup(&vdev->entity);
err_media_unregister:
	media_device_unregister(&cap->mdev);
err_v4l2_unregister:
	v4l2_device_unregister(&cap->v4l2_dev);
err_media_cleanup:
	media_device_cleanup(&cap->mdev);
	return ret;
}

static int sg2002_probe(struct platform_device *pdev)
{
	struct sg2002_capture *cap;
	struct fwnode_handle *endpoint = NULL;
	int irq;
	int ret;

	cap = devm_kzalloc(&pdev->dev, sizeof(*cap), GFP_KERNEL);
	if (!cap)
		return -ENOMEM;

	ret = of_reserved_mem_device_init(&pdev->dev);
	if (ret && ret != -ENODEV)
		dev_warn(&pdev->dev, "failed to initialize reserved memory: %d\n",
			 ret);

	cap->dev = &pdev->dev;
	platform_set_drvdata(pdev, cap);
	sg2002_set_hdmi_defaults(cap);

	cap->mac = devm_platform_ioremap_resource_byname(pdev, "csi-mac0");
	if (IS_ERR(cap->mac))
		return PTR_ERR(cap->mac);
	cap->wrap = devm_platform_ioremap_resource_byname(pdev, "csi-wrap");
	if (IS_ERR(cap->wrap))
		return PTR_ERR(cap->wrap);
	cap->vi = devm_platform_ioremap_resource_byname(pdev, "vi");
	if (IS_ERR(cap->vi))
		return PTR_ERR(cap->vi);
	cap->vip_sys = devm_platform_ioremap_resource_byname(pdev, "vip-sys");
	if (IS_ERR(cap->vip_sys))
		return PTR_ERR(cap->vip_sys);
	cap->ddr_top = syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
						      "sophgo,ddr-top");
	if (IS_ERR(cap->ddr_top))
		return dev_err_probe(&pdev->dev, PTR_ERR(cap->ddr_top),
				     "failed to get DDR-top syscon\n");
	cap->num_clks = devm_clk_bulk_get_all(&pdev->dev, &cap->clks);
	if (cap->num_clks < 0)
		return dev_err_probe(&pdev->dev, cap->num_clks,
				     "failed to get clocks\n");

	cap->resets = devm_reset_control_array_get_optional_exclusive(&pdev->dev);
	if (IS_ERR(cap->resets))
		return dev_err_probe(&pdev->dev, PTR_ERR(cap->resets),
				     "failed to get resets\n");

	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(40));
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "failed to set DMA mask\n");

	ret = sg2002_parse_endpoint(cap, &endpoint);
	if (ret) {
		fwnode_handle_put(endpoint);
		return dev_err_probe(&pdev->dev, ret,
				     "invalid or missing CSI-2 endpoint\n");
	}

	irq = platform_get_irq_byname(pdev, "vi");
	if (irq < 0)
		irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		ret = irq;
		goto err_endpoint_put;
	}
	cap->vi_irq = irq;

	ret = devm_request_irq(&pdev->dev, irq, sg2002_irq, 0,
			       dev_name(&pdev->dev), cap);
	if (ret)
		goto err_endpoint_put;

	cap->csi_irq = platform_get_irq_byname(pdev, "csi");
	if (cap->csi_irq < 0) {
		ret = cap->csi_irq;
		goto err_endpoint_put;
	}
	ret = devm_request_irq(&pdev->dev, cap->csi_irq, sg2002_csi_irq, 0,
			       "sg2002-csi", cap);
	if (ret)
		goto err_endpoint_put;

	ret = sg2002_register_media(cap, endpoint);
	if (ret)
		goto err_endpoint_put;

	fwnode_handle_put(endpoint);
	dev_info(&pdev->dev, "registered four-lane 1080p UYVY capture\n");
	return 0;

err_endpoint_put:
	fwnode_handle_put(endpoint);
	return ret;
}

static void sg2002_remove(struct platform_device *pdev)
{
	struct sg2002_capture *cap = platform_get_drvdata(pdev);

	of_reserved_mem_device_release(&pdev->dev);

	mutex_lock(&cap->lock);
	cap->removing = true;
	if (cap->clocks_enabled)
		sg2002_capture_error(cap);
	mutex_unlock(&cap->lock);
	cancel_work_sync(&cap->error_work);
	/*
	 * Release VB2 before the notifier: queue release calls stop_streaming(),
	 * which may still need cap->source to stop the remote subdevice.
	 */
	vb2_video_unregister_device(&cap->vdev);
	v4l2_async_nf_unregister(&cap->notifier);
	v4l2_async_nf_cleanup(&cap->notifier);
	media_device_unregister(&cap->mdev);
	v4l2_device_unregister(&cap->v4l2_dev);
	media_device_cleanup(&cap->mdev);
	mutex_destroy(&cap->lock);
}

static const struct of_device_id sg2002_of_match[] = {
	{ .compatible = "sophgo,sg2002-csi-capture" },
	{ }
};
MODULE_DEVICE_TABLE(of, sg2002_of_match);

static struct platform_driver sg2002_driver = {
	.probe = sg2002_probe,
	.remove = sg2002_remove,
	.driver = {
		.name = SG2002_DRV_NAME,
		.of_match_table = sg2002_of_match,
	},
};
module_platform_driver(sg2002_driver);
