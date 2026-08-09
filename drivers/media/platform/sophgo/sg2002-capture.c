// SPDX-License-Identifier: GPL-2.0-only
/*
 * Minimal SG2002 CSI-2 YUV capture driver
 *
 * This is deliberately a narrow bring-up driver for the NanoKVM capture
 * path: one CSI-2 D-PHY input, four lanes, UYVY 1920x1080, and the VI FE0
 * CSI bridge DMA engine. The register programming is derived from the
 * vendor cif and vi drivers.
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
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/spinlock.h>
#include <linux/videodev2.h>

#include <media/v4l2-async.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mc.h>
#include <media/v4l2-subdev.h>
#include <media/videobuf2-dma-contig.h>

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
#define MAC_CSI_IRQ_RESERVED	GENMASK(7, 5)
#define MAC_CSI_VC_MAP		(MAC_CSI_BASE + 0x18)
#define MAC_CSI_DECODE_STATUS	(MAC_CSI_BASE + 0x40)
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
#define WRAP_PHY4_DEBUG0	(WRAP_PHY4_BASE + 0x90)
#define WRAP_PHY4_DEBUG1	(WRAP_PHY4_BASE + 0x94)

/* VI FE0 CSI bridge and DMA6. */
#define VI_CSIBDG0		0x00800
#define VI_CSIBDG_TOP_CTRL	(VI_CSIBDG0 + 0x00)
#define VI_CSIBDG_INT_CTRL	(VI_CSIBDG0 + 0x04)
#define VI_CSIBDG_CH0_SIZE	(VI_CSIBDG0 + 0x10)
#define VI_CSIBDG_CH0_DEBUG0	(VI_CSIBDG0 + 0xa0)
#define VI_CSIBDG_CH0_DEBUG1	(VI_CSIBDG0 + 0xa4)
#define VI_CSIBDG_CH0_DEBUG2	(VI_CSIBDG0 + 0xa8)
#define VI_CSIBDG_CH0_DEBUG3	(VI_CSIBDG0 + 0xac)
#define VI_CSIBDG_INT_STATUS0	(VI_CSIBDG0 + 0xe0)
#define VI_CSIBDG_INT_STATUS1	(VI_CSIBDG0 + 0xe4)
#define VI_CSIBDG_DEBUG		(VI_CSIBDG0 + 0xe8)
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
#define VI_DMA_STATUS		(VI_DMA6 + 0x14)
#define VI_DMA_DUMMY		(VI_DMA6 + 0x1c)
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
#define VI_ISP_BLK_IDLE		(VI_ISP_TOP + 0x38)
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
#define VI_PRE_RAW_DEBUG_STATE	0x0002c
#define VI_PRE_RAW_DEBUG_INFO	0x00034
#define VI_WDMA_CORE0		0x72000
#define VI_WDMA_IP_DISABLE	(VI_WDMA_CORE0 + 0x04)
#define VI_WDMA_DISABLE_SEGLEN	(VI_WDMA_CORE0 + 0x08)
#define VI_WDMA_STATUS0		(VI_WDMA_CORE0 + 0x10)
#define VI_WDMA_STATUS1		(VI_WDMA_CORE0 + 0x14)
#define VI_WDMA_RING_ENABLE	(VI_WDMA_CORE0 + 0x18)
#define VI_WDMA_PERF		(VI_WDMA_CORE0 + 0x20)
#define VI_WDMA_NEXT_ADDR0	(VI_WDMA_CORE0 + 0x100)

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
#define VIP_SYS_LP_STATUS	0x50
#define VIP_SYS_OFF_MON1	0xb0
#define VIP_SYS_OFF_MON2	0xb4
#define VIP_SYS_OFF_MON3	0xd0
#define VIP_SYS_OFF_MON4	0xd4
#define VIP_SYS_OFF_MON5	0xd8
#define VIP_SYS_OFF_MON6	0xdc
#define VIP_SYS_OFF_MON7	0xe0
#define VIP_SYS_CSI_MAC0_DIVIDER GENMASK(20, 16)
#define VIP_SYS_CSI_MAC0_DIVIDER_ENABLE BIT(0)
#define VIP_SYS_CSI_MAC0_UPDATE	BIT(2)

#define DDRTOP_CLK_GATING	0x14
#define DDRTOP_DISABLE_GATING	GENMASK(11, 0)

#define SG2002_HS_SETTLE		8
#define SG2002_NUM_DATA_LANES	4
#define SG2002_NUM_PHYS_LANES	6
#define SG2002_STAGE_PHY_RESET	1
#define SG2002_STAGE_MAC_RESET	2
#define SG2002_STAGE_AXI_GATE	3
#define SG2002_STAGE_CLK_SELECT	4
#define SG2002_STAGE_CSI		5
#define SG2002_STAGE_BRIDGE	6
#define SG2002_STAGE_DMA		7
#define SG2002_STAGE_IRQ		8

static unsigned int bringup_stage = SG2002_STAGE_IRQ;
module_param(bringup_stage, uint, 0600);
MODULE_PARM_DESC(bringup_stage,
		 "bring-up stage: 0=source/clocks, 1=PHY reset, 2=MAC reset, 3=AXI gate, 4=clock select, 5=CSI, 6=bridge, 7=DMA, 8=IRQ");

static unsigned int bringup_skip;
module_param(bringup_skip, uint, 0600);
MODULE_PARM_DESC(bringup_skip, "bitmask of prerequisite stages 1-5 to skip");

static bool disable_ddr_clock_gating = true;
module_param(disable_ddr_clock_gating, bool, 0600);
MODULE_PARM_DESC(disable_ddr_clock_gating,
		 "disable DDR-top path gating while DMA is streaming");

static bool configure_scenarios = true;
module_param(configure_scenarios, bool, 0600);
MODULE_PARM_DESC(configure_scenarios,
		 "configure the vendor single-YUV VI DMA scenario");

#define SG2002_STAGE_ENABLED(cap, stage) \
	((cap)->active_stage >= (stage) && \
	 ((stage) > SG2002_STAGE_CSI || !((cap)->active_skip & BIT(stage))))

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
	unsigned int active_stage;
	unsigned int active_skip;
	bool streaming;

	void *scratch;
	dma_addr_t scratch_dma;

	struct v4l2_mbus_config_mipi_csi2 bus;
};

static inline struct sg2002_buffer *to_sg2002_buffer(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);

	return container_of(vbuf, struct sg2002_buffer, vb);
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

	if (SG2002_STAGE_ENABLED(cap, SG2002_STAGE_MAC_RESET)) {
		value = readl(cap->vip_sys + VIP_SYS_RESET);
		writel(value | VIP_SYS_CSI_MAC0_RESET,
		       cap->vip_sys + VIP_SYS_RESET);
		udelay(20);
		writel(value & ~VIP_SYS_CSI_MAC0_RESET,
		       cap->vip_sys + VIP_SYS_RESET);
	}

	if (SG2002_STAGE_ENABLED(cap, SG2002_STAGE_AXI_GATE)) {
		sg2002_update_bits(cap->vip_sys, VIP_SYS_CLK_ENABLE,
				    VIP_SYS_AXI_ISP_ENABLE,
				    VIP_SYS_AXI_ISP_ENABLE);
		/* CVI_SYS_Init disables automatic low-power entry for all VIP
		 * fabrics before opening VI.  In particular, DMA6 reaches memory
		 * through the offline AXI and X2P paths. */
		writel(VIP_SYS_LP_DISABLE, cap->vip_sys + VIP_SYS_AXI_RT_LP);
		writel(VIP_SYS_LP_DISABLE, cap->vip_sys + VIP_SYS_AXI_OFFLINE_LP);
		writel(VIP_SYS_LP_DISABLE, cap->vip_sys + VIP_SYS_X2P_LP);
		writel(VIP_SYS_SC_LP_DISABLE, cap->vip_sys + VIP_SYS_SC_AXI_LP);
		/* Match the vendor VIP base probe: SC_TOP and ISP_RAW use their
		 * offline AXI ports, while ISP_YUV remains on the online port. */
		sg2002_update_bits(cap->vip_sys, VIP_SYS_AXI_SWITCH,
				    VIP_SYS_AXI_OFFLINE_PORTS,
				    VIP_SYS_AXI_VENDOR_ROUTES);
	}
	if (SG2002_STAGE_ENABLED(cap, SG2002_STAGE_CLK_SELECT)) {
		sg2002_update_bits(cap->vip_sys, VIP_SYS_CLK_SELECT,
				    VIP_SYS_CSI_MAC0_SOURCE,
				    FIELD_PREP(VIP_SYS_CSI_MAC0_SOURCE, 2));
		/* VIP_SYS2 is 594 MHz from the boot clock tree.  Match the
		 * vendor RX_MAC_CLK_600M sequence: ratio 31 gives the full
		 * parent rate, then enable and latch the normal divider. */
		sg2002_update_bits(cap->vip_sys, VIP_SYS_CSI_MAC0_RATIO,
				    VIP_SYS_CSI_MAC0_DIVIDER,
				    FIELD_PREP(VIP_SYS_CSI_MAC0_DIVIDER, 31));
		sg2002_update_bits(cap->vip_sys, VIP_SYS_CSI_MAC0_RATIO,
				    VIP_SYS_CSI_MAC0_DIVIDER_ENABLE,
				    VIP_SYS_CSI_MAC0_DIVIDER_ENABLE);
		sg2002_update_bits(cap->vip_sys, VIP_SYS_CSI_MAC0_RATIO,
				    VIP_SYS_CSI_MAC0_UPDATE,
				    VIP_SYS_CSI_MAC0_UPDATE);
		udelay(5);
	}
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

	for (i = 0; i < SG2002_NUM_DATA_LANES; i++) {
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
			for (j = 0; j < SG2002_NUM_DATA_LANES; j++) {
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
				       SG2002_NUM_DATA_LANES - 1));
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
	ctrl = FIELD_PREP(VI_CSIBDG_CSI_MODE, 1) | VI_CSIBDG_YUV_INPUT;
	if (SG2002_STAGE_ENABLED(cap, SG2002_STAGE_DMA)) {
		writel(SG2002_BYTES_PER_LINE, cap->vi + VI_DMA_SEG_LEN);
		writel(SG2002_BYTES_PER_LINE, cap->vi + VI_DMA_STRIDE);
		writel(SG2002_HEIGHT, cap->vi + VI_DMA_SEG_NUM);
		sg2002_dma_set_addr(cap, addr);
		ctrl |= VI_CSIBDG_CH0_DMA;
	}
	writel(ctrl, cap->vi + VI_CSIBDG_TOP_CTRL);
	writel(0, cap->vi + VI_CSIBDG_INT_CTRL);
	writel(FIELD_PREP(VI_CSIBDG_WIDTH, SG2002_WIDTH - 1) |
	       FIELD_PREP(VI_CSIBDG_HEIGHT, SG2002_HEIGHT - 1),
	       cap->vi + VI_CSIBDG_CH0_SIZE);

	if (SG2002_STAGE_ENABLED(cap, SG2002_STAGE_DMA))
		sg2002_preraw_fe0_trigger(cap);
	else
		writel(0, cap->vi + VI_PRE_RAW_FRAME_VALID);
	writel(VI_ISP_FE0_HW_TRIGGER | VI_ISP_FE0_HW_SHADOW,
	       cap->vi + VI_ISP_CTRL_MODE0);
	writel(VI_ISP_FE0_HW_PQ, cap->vi + VI_ISP_CTRL_MODE1);
	if (configure_scenarios &&
	    SG2002_STAGE_ENABLED(cap, SG2002_STAGE_DMA))
		sg2002_configure_single_yuv_scenario(cap);
	if (SG2002_STAGE_ENABLED(cap, SG2002_STAGE_DMA))
		sg2002_dma_set_hw_mode(cap);
	writel(0x00000d01, cap->vi + VI_ISP_SW_CTRL1);
	writel(0x0d010000, cap->vi + VI_ISP_SW_CTRL0);

	sg2002_csibdg_clear_status(cap);
	WRITE_ONCE(cap->streaming,
		   SG2002_STAGE_ENABLED(cap, SG2002_STAGE_IRQ));
	if (SG2002_STAGE_ENABLED(cap, SG2002_STAGE_IRQ)) {
		writel(VI_ISP_FRAME_DONE_FE0_CH0,
		       cap->vi + VI_ISP_INT_EVENT0_EN);
		if (SG2002_STAGE_ENABLED(cap, SG2002_STAGE_CSI))
			sg2002_csi_irq_enable(cap, true);
	}
	ctrl |= VI_CSIBDG_CSI_ENABLE | VI_CSIBDG_UPDATE;
	writel(ctrl, cap->vi + VI_CSIBDG_TOP_CTRL);
}

static void sg2002_vi_disable(struct sg2002_capture *cap)
{
	sg2002_update_bits(cap->vi, VI_CSIBDG_TOP_CTRL,
			    VI_CSIBDG_CSI_ENABLE | VI_CSIBDG_UPDATE, 0);
	writel(0, cap->vi + VI_ISP_INT_EVENT0_EN);
	writel(0, cap->vi + VI_ISP_INT_EVENT1_EN);
	writel(0, cap->vi + VI_ISP_INT_EVENT2_EN);
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
	if (!SG2002_STAGE_ENABLED(cap, SG2002_STAGE_IRQ) ||
	    !READ_ONCE(cap->streaming))
		return IRQ_HANDLED;

	spin_lock_irqsave(&cap->qlock, flags);
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
	return IRQ_HANDLED;
}

static int sg2002_queue_setup(struct vb2_queue *q, unsigned int *num_buffers,
			      unsigned int *num_planes, unsigned int sizes[],
			      struct device *alloc_devs[])
{
	if (*num_planes)
		return sizes[0] < SG2002_SIZE_IMAGE ? -EINVAL : 0;

	*num_planes = 1;
	sizes[0] = SG2002_SIZE_IMAGE;
	return 0;
}

static int sg2002_buf_prepare(struct vb2_buffer *vb)
{
	struct sg2002_buffer *buf = to_sg2002_buffer(vb);

	if (vb2_plane_size(vb, 0) < SG2002_SIZE_IMAGE)
		return -EINVAL;

	buf->dma_addr = vb2_dma_contig_plane_dma_addr(vb, 0);
	vb2_set_plane_payload(vb, 0, SG2002_SIZE_IMAGE);
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
	unsigned long flags;
	int i;
	int ret;

	if (!cap->source) {
		ret = -ENOLINK;
		goto err_buffers;
	}

	ret = video_device_pipeline_start(&cap->vdev, &cap->pipe);
	if (ret)
		goto err_buffers;

	ret = clk_bulk_prepare_enable(cap->num_clks, cap->clks);
	if (ret)
		goto err_pipeline;
	for (i = 0; i < cap->num_clks; i++)
		dev_dbg(cap->dev, "clock %s: %lu Hz\n", cap->clks[i].id,
			 clk_get_rate(cap->clks[i].clk));

	cap->active_stage = min(bringup_stage, (unsigned int)SG2002_STAGE_IRQ);
	cap->active_skip = bringup_skip;
	dev_dbg(cap->dev, "starting bring-up stage %u, skip mask %#x\n",
		 cap->active_stage, cap->active_skip);

	/* CVI_SYS_Init disables all twelve DDR-top path gates before VI is
	 * opened.  Preserve unrelated bits and apply that runtime policy here. */
	if (disable_ddr_clock_gating &&
	    SG2002_STAGE_ENABLED(cap, SG2002_STAGE_DMA)) {
		ret = sg2002_ddr_disable_clock_gating(cap);
		if (ret)
			goto err_clocks;
	}

	if (SG2002_STAGE_ENABLED(cap, SG2002_STAGE_PHY_RESET)) {
		ret = reset_control_assert(cap->resets);
		if (ret)
			goto err_clocks;
		udelay(5);
		ret = reset_control_deassert(cap->resets);
		if (ret)
			goto err_clocks;
	}
	sg2002_vip_init(cap);

	if (SG2002_STAGE_ENABLED(cap, SG2002_STAGE_DMA)) {
		cap->scratch = dma_alloc_coherent(cap->dev, SG2002_SIZE_IMAGE,
						  &cap->scratch_dma, GFP_KERNEL);
		if (!cap->scratch) {
			ret = -ENOMEM;
			goto err_clocks;
		}
	}

	spin_lock_irqsave(&cap->qlock, flags);
	first = sg2002_next_buffer(cap);
	cap->active = first;
	cap->sequence = 0;
	spin_unlock_irqrestore(&cap->qlock, flags);
	if (!first) {
		ret = -ENOBUFS;
		goto err_scratch;
	}

	ret = v4l2_subdev_enable_streams(cap->source, cap->source_pad, BIT(0));
	if (ret && ret != -ENOIOCTLCMD)
		goto err_scratch;

	if (SG2002_STAGE_ENABLED(cap, SG2002_STAGE_CSI))
		sg2002_csi_enable(cap, true);
	if (SG2002_STAGE_ENABLED(cap, SG2002_STAGE_BRIDGE))
		sg2002_vi_enable(cap,
				 SG2002_STAGE_ENABLED(cap, SG2002_STAGE_IRQ) ?
				 first->dma_addr : cap->scratch_dma);
	if (SG2002_STAGE_ENABLED(cap, SG2002_STAGE_CSI)) {
		usleep_range(20000, 21000);
		dev_dbg(cap->dev,
			 "status: decode=%#x csi-irq=%#x phy=%#x/%#x events=%#x/%#x/%#x en=%#x/%#x/%#x\n",
			 readl(cap->mac + MAC_CSI_DECODE_STATUS),
			 readl(cap->mac + MAC_CSI_IRQ_STATUS),
			 readl(cap->wrap + WRAP_PHY4_DEBUG0),
			 readl(cap->wrap + WRAP_PHY4_DEBUG1),
			 readl(cap->vi + VI_ISP_INT_EVENT0),
			 readl(cap->vi + VI_ISP_INT_EVENT1),
			 readl(cap->vi + VI_ISP_INT_EVENT2),
			 readl(cap->vi + VI_ISP_INT_EVENT0_EN),
			 readl(cap->vi + VI_ISP_INT_EVENT1_EN),
			 readl(cap->vi + VI_ISP_INT_EVENT2_EN));
		dev_dbg(cap->dev,
			 "bridge: ctrl=%#x/%#x count=%#x/%#x/%#x/%#x irq=%#x/%#x debug=%#x\n",
			 readl(cap->vi + VI_CSIBDG_TOP_CTRL),
			 readl(cap->vi + VI_CSIBDG_INT_CTRL),
			 readl(cap->vi + VI_CSIBDG_CH0_DEBUG0),
			 readl(cap->vi + VI_CSIBDG_CH0_DEBUG1),
			 readl(cap->vi + VI_CSIBDG_CH0_DEBUG2),
			 readl(cap->vi + VI_CSIBDG_CH0_DEBUG3),
			 readl(cap->vi + VI_CSIBDG_INT_STATUS0),
			 readl(cap->vi + VI_CSIBDG_INT_STATUS1),
			 readl(cap->vi + VI_CSIBDG_DEBUG));
		dev_dbg(cap->dev,
			 "vi: preraw valid/state/done=%#x/%#x/%#x dma6 ctrl/status=%#x/%#x\n",
			 readl(cap->vi + VI_PRE_RAW_FRAME_VALID),
			 readl(cap->vi + VI_PRE_RAW_DEBUG_STATE),
			 readl(cap->vi + VI_PRE_RAW_DEBUG_INFO),
			 readl(cap->vi + VI_DMA_SYS_CTRL),
			 readl(cap->vi + VI_DMA_STATUS));
		dev_dbg(cap->dev,
			 "dma6: target=%pad base=%#x geometry=%#x/%#x/%#x dummy=%#x\n",
			 &first->dma_addr, readl(cap->vi + VI_DMA_BASE_ADDR),
			 readl(cap->vi + VI_DMA_SEG_LEN),
			 readl(cap->vi + VI_DMA_STRIDE),
			 readl(cap->vi + VI_DMA_SEG_NUM),
			 readl(cap->vi + VI_DMA_DUMMY));
		dev_dbg(cap->dev,
			 "wdma0: disable=%#x/%#x ring=%#x perf=%#x status=%#x/%#x next=%#x idle=%#x\n",
			 readl(cap->vi + VI_WDMA_IP_DISABLE),
			 readl(cap->vi + VI_WDMA_DISABLE_SEGLEN),
			 readl(cap->vi + VI_WDMA_RING_ENABLE),
			 readl(cap->vi + VI_WDMA_PERF),
			 readl(cap->vi + VI_WDMA_STATUS0),
			 readl(cap->vi + VI_WDMA_STATUS1),
			 readl(cap->vi + VI_WDMA_NEXT_ADDR0),
			 readl(cap->vi + VI_ISP_BLK_IDLE));
		dev_dbg(cap->dev,
			 "vip-status: lp=%#x off-mon=%#x/%#x/%#x/%#x/%#x/%#x/%#x\n",
			 readl(cap->vip_sys + VIP_SYS_LP_STATUS),
			 readl(cap->vip_sys + VIP_SYS_OFF_MON1),
			 readl(cap->vip_sys + VIP_SYS_OFF_MON2),
			 readl(cap->vip_sys + VIP_SYS_OFF_MON3),
			 readl(cap->vip_sys + VIP_SYS_OFF_MON4),
			 readl(cap->vip_sys + VIP_SYS_OFF_MON5),
			 readl(cap->vip_sys + VIP_SYS_OFF_MON6),
			 readl(cap->vip_sys + VIP_SYS_OFF_MON7));
	}
	return 0;

err_scratch:
	if (cap->scratch)
		dma_free_coherent(cap->dev, SG2002_SIZE_IMAGE, cap->scratch,
				  cap->scratch_dma);
	cap->scratch = NULL;
err_clocks:
	sg2002_ddr_restore_clock_gating(cap);
	clk_bulk_disable_unprepare(cap->num_clks, cap->clks);
err_pipeline:
	video_device_pipeline_stop(&cap->vdev);
err_buffers:
	sg2002_return_buffers(cap, VB2_BUF_STATE_QUEUED);
	return ret;
}

static void sg2002_stop_streaming(struct vb2_queue *q)
{
	struct sg2002_capture *cap = vb2_get_drv_priv(q);

	WRITE_ONCE(cap->streaming, false);
	if (SG2002_STAGE_ENABLED(cap, SG2002_STAGE_BRIDGE))
		sg2002_vi_disable(cap);
	if (SG2002_STAGE_ENABLED(cap, SG2002_STAGE_CSI))
		sg2002_csi_enable(cap, false);
	synchronize_irq(cap->vi_irq);
	synchronize_irq(cap->csi_irq);
	v4l2_subdev_disable_streams(cap->source, cap->source_pad, BIT(0));
	sg2002_return_buffers(cap, VB2_BUF_STATE_ERROR);
	if (cap->scratch)
		dma_free_coherent(cap->dev, SG2002_SIZE_IMAGE, cap->scratch,
				  cap->scratch_dma);
	cap->scratch = NULL;
	sg2002_ddr_restore_clock_gating(cap);
	clk_bulk_disable_unprepare(cap->num_clks, cap->clks);
	video_device_pipeline_stop(&cap->vdev);
}

static const struct vb2_ops sg2002_vb2_ops = {
	.queue_setup = sg2002_queue_setup,
	.buf_prepare = sg2002_buf_prepare,
	.buf_queue = sg2002_buf_queue,
	.start_streaming = sg2002_start_streaming,
	.stop_streaming = sg2002_stop_streaming,
};

static void sg2002_fill_pix_format(struct v4l2_pix_format *pix)
{
	pix->width = SG2002_WIDTH;
	pix->height = SG2002_HEIGHT;
	pix->pixelformat = V4L2_PIX_FMT_UYVY;
	pix->field = V4L2_FIELD_NONE;
	pix->bytesperline = SG2002_BYTES_PER_LINE;
	pix->sizeimage = SG2002_SIZE_IMAGE;
	pix->colorspace = V4L2_COLORSPACE_REC709;
	pix->ycbcr_enc = V4L2_YCBCR_ENC_709;
	pix->quantization = V4L2_QUANTIZATION_LIM_RANGE;
	pix->xfer_func = V4L2_XFER_FUNC_709;
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
	if (fmt->index)
		return -EINVAL;

	fmt->pixelformat = V4L2_PIX_FMT_UYVY;
	return 0;
}

static int sg2002_g_fmt(struct file *file, void *priv, struct v4l2_format *fmt)
{
	sg2002_fill_pix_format(&fmt->fmt.pix);
	return 0;
}

static int sg2002_try_fmt(struct file *file, void *priv,
			  struct v4l2_format *fmt)
{
	sg2002_fill_pix_format(&fmt->fmt.pix);
	return 0;
}

static int sg2002_s_fmt(struct file *file, void *priv, struct v4l2_format *fmt)
{
	struct sg2002_capture *cap = video_drvdata(file);

	if (vb2_is_busy(&cap->queue))
		return -EBUSY;

	return sg2002_try_fmt(file, priv, fmt);
}

static int sg2002_enum_framesizes(struct file *file, void *priv,
				  struct v4l2_frmsizeenum *fsize)
{
	if (fsize->index || fsize->pixel_format != V4L2_PIX_FMT_UYVY)
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete.width = SG2002_WIDTH;
	fsize->discrete.height = SG2002_HEIGHT;
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
	return 0;
}

static const struct media_entity_operations sg2002_entity_ops = {
	.link_validate = sg2002_link_validate,
};

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

	cap->source = subdev;
	cap->source_pad = source->index;
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

	cap->source = NULL;
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
	    ep.bus.mipi_csi2.num_data_lanes != SG2002_NUM_DATA_LANES)
		return -EINVAL;

	if (ep.bus.mipi_csi2.clock_lane >= SG2002_NUM_PHYS_LANES)
		return -EINVAL;
	used[ep.bus.mipi_csi2.clock_lane] = true;

	for (i = 0; i < SG2002_NUM_DATA_LANES; i++) {
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

	ret = media_device_register(&cap->mdev);
	if (ret)
		goto err_v4l2_unregister;

	mutex_init(&cap->lock);
	spin_lock_init(&cap->qlock);
	INIT_LIST_HEAD(&cap->queued);

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
	video_unregister_device(vdev);
	goto err_queue_release;
err_entity_cleanup:
	media_entity_cleanup(&vdev->entity);
err_queue_release:
	vb2_queue_release(q);
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

	cap->dev = &pdev->dev;
	platform_set_drvdata(pdev, cap);

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

	v4l2_async_nf_unregister(&cap->notifier);
	v4l2_async_nf_cleanup(&cap->notifier);
	video_unregister_device(&cap->vdev);
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
