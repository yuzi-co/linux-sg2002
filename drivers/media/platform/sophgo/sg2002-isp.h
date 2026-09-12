/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * SG2002 linear Bayer ISP, owned by sg2002-capture.
 *
 * Register offsets/fields follow the CV181x (mars) VI register map. The
 * online path is FE0 -> BE -> RAW/CFA -> RGB/CSC -> YUV -> DMA46/47.
 * There is no firmware or auxiliary CPU transaction in this path.
 *
 * Initial fixed-function profile: linear Bayer, unity white balance,
 * demosaic and RGB-to-YCbCr conversion. Statistics, lens shading, HDR,
 * temporal filtering and their unallocated DMA channels are disabled.
 * Native semiplanar output is NV21, suitable for the VPSS mem2mem input.
 */
#ifndef SG2002_ISP_H
#define SG2002_ISP_H

#define ISP_FE_CTRL		0x00000
#define ISP_FE_SIZE		0x00004
#define ISP_BE_CTRL		0x18000
#define ISP_BE_PQ		0x18004
#define ISP_BE_SIZE		0x18008
#define ISP_PRE_WDMA		0x1b900
#define ISP_RAW_CTRL		0x30010
#define ISP_RAW_PQ		0x30014
#define ISP_RAW_SIZE		0x30018
#define ISP_RAW_YUV_IN		0x30020
#define ISP_RAW_BAYER		0x30030
#define ISP_RAW_RDMI		0x30034
#define ISP_RAW_DPCM		0x3003c
#define ISP_CFA_CTRL		0x31000
#define ISP_CFA_SHADOW		BIT(0)
#define ISP_CFA_ENABLE		BIT(1)
#define ISP_RAW_RDMA_CONFIG	0x36104
#define ISP_RGB_CTRL		0x50000
#define ISP_RGB_SIZE		0x50024
#define ISP_CSC_CTRL		0x56000
#define ISP_CSC_ENABLE		BIT(0)
#define ISP_CSC_SHADOW		BIT(1)
#define ISP_CSC_COEFF01		0x56010
#define ISP_YUV_MONO		0x6000c
#define ISP_YUV_CTRL		0x60060
#define ISP_YUV_SIZE		0x60064
#define ISP_YUV_BYPASS_H		BIT(3)
#define ISP_YUV_BYPASS_V		BIT(4)
#define ISP_CROP_Y		0x69000
#define ISP_CROP_UV		0x6a000
#define ISP_CROP_DMA_ENABLE	BIT(1)
#define ISP_DMA_Y		0x69100
#define ISP_DMA_UV		0x6a100
#define ISP_DMA_CONTROL		0x00
#define ISP_DMA_ADDR		0x04
#define ISP_DMA_SEGLEN		0x08
#define ISP_DMA_STRIDE		0x0c
#define ISP_DMA_LINES		0x10
#define ISP_DMA_ADDR_HIGH		GENMASK(15, 8)
#define ISP_DMA_SW_ADDR		BIT(16)
#define ISP_DMA_SW_GEOMETRY	GENMASK(19, 17)
#define ISP_VI_SEL_SIZE		0x7f404
#define ISP_BLK_IDLE		0x70038

#define ISP_FRAME_DONE_POST	BIT(13)
#define ISP_FRAME_ERROR		BIT(8)
#define ISP_DMA_ERROR		BIT(16)
#define ISP_ONLINE_STAGES		(BIT(0) | BIT(8) | BIT(10) | BIT(11))
#define ISP_ONLINE_SCENARIO	(BIT(1) | BIT(2) | BIT(5))
#define ISP_PIPELINE_IDLE		(BIT(0) | GENMASK(9, 3))

struct sg2002_isp_reg {
	u32 offset;
	u32 mask;
	u32 value;
};

static void sg2002_isp_update(void __iomem *vi, u32 reg, u32 mask, u32 value)
{
	writel((readl(vi + reg) & ~mask) | (value & mask), vi + reg);
}

static void sg2002_isp_set_addr(void __iomem *vi, dma_addr_t addr,
			       u32 stride, u32 height)
{
	dma_addr_t chroma = addr + stride * height;

	writel(lower_32_bits(addr), vi + ISP_DMA_Y + ISP_DMA_ADDR);
	sg2002_isp_update(vi, ISP_DMA_Y + ISP_DMA_CONTROL,
			   ISP_DMA_ADDR_HIGH,
			   FIELD_PREP(ISP_DMA_ADDR_HIGH, upper_32_bits(addr)));
	writel(lower_32_bits(chroma), vi + ISP_DMA_UV + ISP_DMA_ADDR);
	sg2002_isp_update(vi, ISP_DMA_UV + ISP_DMA_CONTROL,
			   ISP_DMA_ADDR_HIGH,
			   FIELD_PREP(ISP_DMA_ADDR_HIGH, upper_32_bits(chroma)));
}

static void sg2002_isp_configure(void __iomem *vi, u32 width, u32 height,
				u32 stride, u32 bayer, dma_addr_t addr)
{
	/* Each mask targets documented writable controls, not status or LUTs.
	 * Preserve reset tuning values of disabled blocks for later controls.
	 */
	static const struct sg2002_isp_reg bypass[] = {
		{ 0x01008, BIT(0), 0 }, /* FE BLC LE */
		{ 0x01808, BIT(0), 0 }, /* FE BLC SE */
		{ 0x02000, BIT(0), 0 }, /* RGBMAP LE */
		{ 0x02300, BIT(0), 0 }, /* RGBMAP SE */
		{ 0x18800, BIT(0), 0 }, /* BE crop LE */
		{ 0x19000, BIT(0), 0 }, /* BE crop SE */
		{ 0x19808, BIT(0), 0 }, /* BE BLC LE */
		{ 0x1a008, BIT(0), 0 }, /* BE BLC SE */
		{ 0x1a8a0, BIT(0) | BIT(8), BIT(8) }, /* AF DMA */
		{ 0x1b008, GENMASK(3, 0), 0 }, /* DPC LE */
		{ 0x1b108, GENMASK(3, 0), 0 }, /* DPC SE */
		{ 0x3200c, BIT(0), 0 }, /* lens shading */
		{ 0x3300c, BIT(0), 0 }, /* GMS */
		{ 0x340a0, BIT(0) | BIT(8) | BIT(16) | BIT(24),
			BIT(8) | BIT(24) }, /* AE/AWB LE DMA */
		{ 0x350a0, BIT(0) | BIT(8) | BIT(16) | BIT(24),
			BIT(8) | BIT(24) }, /* AE/AWB SE DMA */
		{ 0x3c004, GENMASK(3, 0), 1 }, /* BNR delayed bypass */
		{ 0x3d000, BIT(0), 0 }, /* RAW crop LE */
		{ 0x3e000, BIT(0), 0 }, /* RAW crop SE */
		{ 0x3f000, BIT(0) | BIT(31), BIT(31) }, /* LMAP LE */
		{ 0x3f200, BIT(0) | BIT(31), BIT(31) }, /* LMAP SE */
		{ 0x40008, BIT(0), 0 }, /* unity WBG LE */
		{ 0x41008, BIT(0), 0 }, /* unity WBG SE */
		{ 0x44000, BIT(0), 0 }, /* LCAC */
		{ 0x45000, BIT(0), 0 }, /* RGB CAC */
		{ 0x52024, BIT(2), 0 }, /* CCM0 */
		{ 0x52124, BIT(2), 0 }, /* CCM1 */
		{ 0x52200, BIT(0), 0 }, /* RGB gamma */
		{ 0x52300, BIT(0), 0 }, /* Y gamma */
		{ 0x53000, BIT(0) | BIT(1) | BIT(24), BIT(24) }, /* MANR */
		{ 0x5306c, GENMASK(15, 8), GENMASK(15, 8) }, /* MANR DMA */
		{ 0x54000, BIT(0), 0 }, /* CLUT */
		{ 0x55014, BIT(0), 0 }, /* dehaze */
		{ 0x57000, BIT(0), 0 }, /* RGB dither */
		{ 0x5c00c, BIT(0), 0 }, /* HISTV */
		{ 0x5c028, BIT(0) | BIT(8), BIT(8) }, /* HISTV DMA */
		{ 0x5d000, BIT(0) | BIT(4), 0 }, /* HDR fusion */
		{ 0x5d008, GENMASK(19, 16), BIT(16) }, /* long exposure */
		{ 0x5e000, GENMASK(2, 0) | GENMASK(17, 16),
			GENMASK(17, 16) }, /* LTM + both DMA channels */
		{ 0x5e064, BIT(0), 0 }, /* LTM sharpening */
		{ 0x61010, BIT(0), 0 }, /* 444 -> 422 */
		{ 0x61014, BIT(1) | BIT(3), 0 }, /* TNR / forced mono */
		{ 0x61020, GENMASK(13, 8), GENMASK(13, 8) }, /* TNR DMA */
		{ 0x61a00, BIT(0), 0 }, /* FBC encode */
		{ 0x61d00, BIT(0), 0 }, /* FBC decode */
		{ 0x61e00, BIT(0), 0 }, /* Y dither */
		{ 0x61e04, BIT(0), 0 }, /* UV dither */
		{ 0x62000, BIT(0), 0 }, /* CA */
		{ 0x63000, BIT(0), 0 }, /* CA lite */
		{ 0x64004, GENMASK(3, 0), 1 }, /* YNR delayed bypass */
		{ 0x65000, BIT(0) | BIT(4), 0 }, /* CNR / purple fringe */
		{ 0x66000, BIT(0), 0 }, /* sharpening */
		{ 0x67000, BIT(0), 0 }, /* Y curve */
		{ 0x6800c, BIT(0) | BIT(4) | BIT(8), 0 }, /* DCI */
		{ 0x680a0, BIT(0) | BIT(8), BIT(8) }, /* DCI DMA */
		{ 0x6b000, GENMASK(3, 0), 0 }, /* LDCI */
		{ 0x6b0a0, GENMASK(1, 0) | GENMASK(9, 8),
			GENMASK(9, 8) }, /* LDCI DMA */
		{ 0x6c000, BIT(0), 0 }, /* pre-sharpening */
	};
	u32 size = (width - 1) | ((height - 1) << 16);
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(bypass); i++)
		sg2002_isp_update(vi, bypass[i].offset, bypass[i].mask,
				   bypass[i].value);

	/* Geometry and Bayer order must agree at every online stage. */
	sg2002_isp_update(vi, ISP_FE_CTRL, 0xff, bayer | (bayer << 4));
	writel(size, vi + ISP_FE_SIZE);
	sg2002_isp_update(vi, ISP_BE_CTRL, GENMASK(9, 0), bayer | (bayer << 4));
	writel(size, vi + ISP_BE_SIZE);
	writel(size, vi + ISP_VI_SEL_SIZE);
	writel(1, vi + ISP_BE_PQ);
	sg2002_isp_update(vi, ISP_PRE_WDMA, GENMASK(1, 0), 0);
	sg2002_isp_update(vi, ISP_RAW_RDMA_CONFIG, GENMASK(1, 0), 0);
	sg2002_isp_update(vi, ISP_RAW_CTRL, BIT(0), 0);
	writel(size, vi + ISP_RAW_SIZE);
	writel(bayer, vi + ISP_RAW_BAYER);
	writel(0, vi + ISP_RAW_YUV_IN);
	writel(0, vi + ISP_RAW_RDMI);
	writel(0, vi + ISP_RAW_DPCM);
	writel(1, vi + ISP_RAW_PQ);
	sg2002_isp_update(vi, ISP_CFA_CTRL, ISP_CFA_ENABLE | ISP_CFA_SHADOW,
			   ISP_CFA_ENABLE | ISP_CFA_SHADOW);
	sg2002_isp_update(vi, ISP_RGB_CTRL, GENMASK(4, 0) | BIT(8),
			   bayer | BIT(8));
	writel(size, vi + ISP_RGB_SIZE);
	/* Full-range BT.601, signed Q10 coefficients and 10-bit offsets.
	 * These are the neutral table emitted by isp_algo_csc_main:
	 *  306  601 117; -173 -339 512; 512 -429 -83; offsets 0,512,512.
	 * The tuning ABI writes two's-complement coefficients masked to 14
	 * bits, not the VPSS block's sign-magnitude representation.
	 */
	writel(0x02590132, vi + ISP_CSC_COEFF01);
	writel(0x3f530075, vi + ISP_CSC_COEFF01 + 4);
	writel(0x02003ead, vi + ISP_CSC_COEFF01 + 8);
	writel(0x3e530200, vi + ISP_CSC_COEFF01 + 12);
	writel(0x00003fad, vi + ISP_CSC_COEFF01 + 16);
	writel(0x02000200, vi + ISP_CSC_COEFF01 + 20);
	sg2002_isp_update(vi, ISP_CSC_CTRL, ISP_CSC_ENABLE | ISP_CSC_SHADOW,
			   ISP_CSC_ENABLE | ISP_CSC_SHADOW);
	writel(size, vi + ISP_YUV_SIZE);
	sg2002_isp_update(vi, ISP_YUV_MONO, BIT(2), 0);
	sg2002_isp_update(vi, ISP_YUV_CTRL, ISP_YUV_BYPASS_H | ISP_YUV_BYPASS_V, 0);

	/* DMA46 is Y, DMA47 interleaved VU at half the line count. Both
	 * addresses and geometry are supplied by this driver's vb2 queue.
	 */
	for (i = 0; i < 2; i++) {
		u32 dma = i ? ISP_DMA_UV : ISP_DMA_Y;
		u32 crop = i ? ISP_CROP_UV : ISP_CROP_Y;

		writel(stride, vi + dma + ISP_DMA_SEGLEN);
		writel(stride, vi + dma + ISP_DMA_STRIDE);
		writel(height >> i, vi + dma + ISP_DMA_LINES);
		sg2002_isp_update(vi, dma + ISP_DMA_CONTROL,
				   ISP_DMA_SW_ADDR | ISP_DMA_SW_GEOMETRY,
				   ISP_DMA_SW_ADDR | ISP_DMA_SW_GEOMETRY);
		sg2002_isp_update(vi, crop, BIT(0) | ISP_CROP_DMA_ENABLE,
				   ISP_CROP_DMA_ENABLE);
		sg2002_isp_update(vi, crop + 0x14, BIT(1), 0);
	}
	sg2002_isp_set_addr(vi, addr, stride, height);
}

static void sg2002_isp_quiesce(void __iomem *vi)
{
	sg2002_isp_update(vi, ISP_CROP_Y, ISP_CROP_DMA_ENABLE, 0);
	sg2002_isp_update(vi, ISP_CROP_UV, ISP_CROP_DMA_ENABLE, 0);
}

#endif
