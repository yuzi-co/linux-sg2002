// SPDX-License-Identifier: GPL-2.0+
/*
 * Sophgo DWMAC platform driver
 *
 * Copyright (C) 2024 Inochi Amaoto <inochiama@gmail.com>
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/platform_device.h>

#include "stmmac_platform.h"

struct sophgo_dwmac_data {
	int (*init)(struct platform_device *pdev,
		    struct plat_stmmacenet_data *plat_dat,
		    struct stmmac_resources *stmmac_res);
	bool has_internal_rx_delay;
};

static int sophgo_sg2044_dwmac_init(struct platform_device *pdev,
				    struct plat_stmmacenet_data *plat_dat,
				    struct stmmac_resources *stmmac_res)
{
	plat_dat->clk_tx_i = devm_clk_get_enabled(&pdev->dev, "tx");
	if (IS_ERR(plat_dat->clk_tx_i))
		return dev_err_probe(&pdev->dev, PTR_ERR(plat_dat->clk_tx_i),
				     "failed to get tx clock\n");

	plat_dat->flags |= STMMAC_FLAG_SPH_DISABLE;
	plat_dat->set_clk_tx_rate = stmmac_set_clk_tx_rate;
	plat_dat->multicast_filter_bins = 0;

	return 0;
}

/*
 * CV1800B/SG2002 integrate a 10/100 "internal EPHY". Mainline neither
 * powers it up nor calibrates it; the vendor splits that across two
 * places we have to do ourselves here:
 *
 *   1. Power-up — the vendor first-stage bootloader (U-Boot
 *      board/cvitek/mars/board.c::cv181x_ephy_id_init) releases the
 *      EPHY from shutdown before Linux runs. Mainline U-Boot doesn't,
 *      so without it the EPHY never answers on MDIO and phy attach
 *      fails -EINVAL ("validation of <mode> ... failed").
 *
 *   2. Analog calibration — the vendor PHY driver
 *      (drivers/net/phy/cvitek.c::cv182xa_phy_config_init) programs the
 *      EPHY's MLT3/link-pulse/TP-idle/10-100BaseT/AGC/LPF-HPF analog
 *      tables. Without it the PHY attaches but never establishes a wire
 *      link (carrier stays 0 even with a cable). We port that sequence
 *      with its non-efuse default trims (the vendor's per-chip efuse
 *      trimming only tightens signal margins) and the CV181X ("mars")
 *      LPF/HPF coefficients.
 *
 * Everything is MMIO into the EPHY control/analog block — a small
 * always-on window at 0x03009000 (top-wrap at +0x800, MII page select
 * at +0x07c, accessed through the APB interface enabled by +0x804[0]).
 */
#define CV1800B_EPHY_BASE	0x03009000
#define CV1800B_EPHY_SIZE	0x1000

static void cv1800b_ephy_init(struct device *dev)
{
	void __iomem *r;

	r = ioremap(CV1800B_EPHY_BASE, CV1800B_EPHY_SIZE);
	if (!r) {
		dev_warn(dev, "cv1800b: cannot map EPHY regs; link will fail\n");
		return;
	}

	/* --- power-up (vendor U-Boot cv181x_ephy_id_init) --- */
	writel(0x0001, r + 0x804);	/* rg_ephy_apb_rw_sel: drive via APB */
	writel(0x0900, r + 0x800);	/* release 0x800[0]=0 shutdown */
	writel(0x0904, r + 0x800);	/* release 0x800[2]=1 dig_rst_n */
	writel(0x0500, r + 0x07c);	/* MII page 5 */
	writel(0x0c00, r + 0x040);	/* release ANA_PD */
	writel(0x0c7e, r + 0x040);	/* release ANA_EN */
	usleep_range(1000, 2000);	/* wait PLL lock */
	writel(0x0906, r + 0x800);	/* release 0x800[1]=1 ana_rst_n */
	writel(0x0043, r + 0x008);	/* PHY ID high */
	writel(0x5649, r + 0x00c);	/* PHY ID low (-> 0x00435649) */

	/* --- analog calibration (vendor cv182xa_phy_config_init) --- */
	writel(0x0000, r + 0x07c);	/* MII page 0 */

	/* TX bias / echo / Rterm — non-efuse defaults */
	writel(0x5a5a, r + 0x064);	/* rg_eth_txitune */
	writel(0x0000, r + 0x054);	/* rg_eth_txechoiadj */
	writel(0x0bb0, r + 0x058);	/* rg_eth_txrterm/echorcadj */

	/* 100BaseT */
	writel(0x0c10, r + 0x05c);	/* rise update */
	writel(0x0003, r + 0x068);	/* falling phase */
	writel(0x0000, r + 0x054);	/* double TX bias */

	/* MLT3 positive phase (page 16) */
	writel(0x1000, r + 0x07c);
	writel(0x1000, r + 0x068);
	writel(0x3020, r + 0x06c);
	writel(0x5040, r + 0x070);
	writel(0x7060, r + 0x074);
	writel(0x1708, r + 0x058);
	writel(0x3827, r + 0x05c);
	writel(0x5748, r + 0x060);
	writel(0x7867, r + 0x064);

	/* MLT3 negative phase (page 17) */
	writel(0x1100, r + 0x07c);
	writel(0x9080, r + 0x040);
	writel(0xb0a0, r + 0x044);
	writel(0xd0c0, r + 0x048);
	writel(0xf0e0, r + 0x04c);
	writel(0x9788, r + 0x050);
	writel(0xb8a7, r + 0x054);
	writel(0xd7c8, r + 0x058);
	writel(0xf8e7, r + 0x05c);

	/* enable TX Rterm + rx vcm (page 5) */
	writel(0x0500, r + 0x07c);
	writel(readl(r + 0x040) | 0x0001, r + 0x040);
	writel(readl(r + 0x04c) | 0x0820, r + 0x04c);

	/* link pulse shaping (page 10) */
	writel(0x0a00, r + 0x07c);
	writel(0x3e00, r + 0x040);
	writel(0x7864, r + 0x044);
	writel(0x6470, r + 0x048);
	writel(0x5f62, r + 0x04c);
	writel(0x5a5a, r + 0x050);
	writel(0x5458, r + 0x054);
	writel(0xb23a, r + 0x058);
	writel(0x94a0, r + 0x05c);
	writel(0x9092, r + 0x060);
	writel(0x8a8e, r + 0x064);
	writel(0x8688, r + 0x068);
	writel(0x8484, r + 0x06c);
	writel(0x0082, r + 0x070);

	/* TP_IDLE (page 11) */
	writel(0x0b00, r + 0x07c);
	writel(0x5252, r + 0x040);
	writel(0x5252, r + 0x044);
	writel(0x4b52, r + 0x048);
	writel(0x3d47, r + 0x04c);
	writel(0xaa99, r + 0x050);
	writel(0x989e, r + 0x054);
	writel(0x9395, r + 0x058);
	writel(0x9091, r + 0x05c);
	writel(0x8e8f, r + 0x060);
	writel(0x8d8e, r + 0x064);
	writel(0x8c8c, r + 0x068);
	writel(0x8b8b, r + 0x06c);
	writel(0x008a, r + 0x070);

	/* 10BaseT data (page 13) */
	writel(0x0d00, r + 0x07c);
	writel(0x1e0a, r + 0x040);
	writel(0x3862, r + 0x044);
	writel(0x1e62, r + 0x048);
	writel(0x2a08, r + 0x04c);
	writel(0x244c, r + 0x050);
	writel(0x1a44, r + 0x054);
	writel(0x061c, r + 0x058);

	/* (page 14) */
	writel(0x0e00, r + 0x07c);
	writel(0x2d30, r + 0x040);
	writel(0x3470, r + 0x044);
	writel(0x0648, r + 0x048);
	writel(0x261c, r + 0x04c);
	writel(0x3160, r + 0x050);
	writel(0x2d5e, r + 0x054);

	/* (page 15) */
	writel(0x0f00, r + 0x07c);
	writel(0x2922, r + 0x040);
	writel(0x366e, r + 0x044);
	writel(0x0752, r + 0x048);
	writel(0x2556, r + 0x04c);
	writel(0x2348, r + 0x050);
	writel(0x0c30, r + 0x054);

	/* (page 16) */
	writel(0x1000, r + 0x07c);
	writel(0x1e08, r + 0x040);
	writel(0x3868, r + 0x044);
	writel(0x1462, r + 0x048);
	writel(0x1a0e, r + 0x04c);
	writel(0x305e, r + 0x050);
	writel(0x2f62, r + 0x054);

	/* LED out select (page 1) */
	writel(0x0100, r + 0x07c);
	writel(readl(r + 0x068) & ~0x0f00, r + 0x068);

	/* AGC max/min swing (page 19) */
	writel(0x1300, r + 0x07c);
	writel(0x0012, r + 0x058);
	writel(0x6848, r + 0x05c);

	/* CV181X ("mars") LPF/HPF (page 18) */
	writel(0x1200, r + 0x07c);
	writel(0x0808, r + 0x048);	/* lpf */
	writel(0x0808, r + 0x04c);
	writel(0x32f8, r + 0x050);	/* hpf */
	writel(0xf8dc, r + 0x054);

	/* back to page 0, start auto-neg, force full-duplex */
	writel(0x0000, r + 0x07c);
	writel(0x090e, r + 0x800);	/* EPHY start auto-neg */
	writel(readl(r) | 0x0100, r);	/* BMCR full-duplex */

	writel(0x0000, r + 0x804);	/* hand MDIO back to ETH_MAC */

	iounmap(r);
}

static int sophgo_cv1800b_dwmac_init(struct platform_device *pdev,
				     struct plat_stmmacenet_data *plat_dat,
				     struct stmmac_resources *stmmac_res)
{
	cv1800b_ephy_init(&pdev->dev);

	plat_dat->flags |= STMMAC_FLAG_SPH_DISABLE;
	plat_dat->multicast_filter_bins = 0;

	return 0;
}

static int sophgo_dwmac_probe(struct platform_device *pdev)
{
	struct plat_stmmacenet_data *plat_dat;
	const struct sophgo_dwmac_data *data;
	struct stmmac_resources stmmac_res;
	struct device *dev = &pdev->dev;
	int ret;

	ret = stmmac_get_platform_resources(pdev, &stmmac_res);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to get platform resources\n");

	plat_dat = devm_stmmac_probe_config_dt(pdev, stmmac_res.mac);
	if (IS_ERR(plat_dat))
		return dev_err_probe(dev, PTR_ERR(plat_dat),
				     "failed to parse DT parameters\n");

	data = device_get_match_data(&pdev->dev);
	if (data && data->init) {
		ret = data->init(pdev, plat_dat, &stmmac_res);
		if (ret)
			return ret;
	}

	if (data && data->has_internal_rx_delay) {
		plat_dat->phy_interface = phy_fix_phy_mode_for_mac_delays(plat_dat->phy_interface,
									  false, true);
		if (plat_dat->phy_interface == PHY_INTERFACE_MODE_NA)
			return -EINVAL;
	}

	return stmmac_dvr_probe(dev, plat_dat, &stmmac_res);
}

static const struct sophgo_dwmac_data sg2042_dwmac_data = {
	.init = sophgo_sg2044_dwmac_init,
	.has_internal_rx_delay = true,
};

static const struct sophgo_dwmac_data sg2044_dwmac_data = {
	.init = sophgo_sg2044_dwmac_init,
};

static const struct sophgo_dwmac_data cv1800b_dwmac_data = {
	.init = sophgo_cv1800b_dwmac_init,
};

static const struct of_device_id sophgo_dwmac_match[] = {
	{ .compatible = "sophgo,sg2042-dwmac", .data = &sg2042_dwmac_data },
	{ .compatible = "sophgo,sg2044-dwmac", .data = &sg2044_dwmac_data },
	{ .compatible = "sophgo,cv1800b-dwmac", .data = &cv1800b_dwmac_data },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, sophgo_dwmac_match);

static struct platform_driver sophgo_dwmac_driver = {
	.probe  = sophgo_dwmac_probe,
	.remove = stmmac_pltfr_remove,
	.driver = {
		.name = "sophgo-dwmac",
		.pm = &stmmac_pltfr_pm_ops,
		.of_match_table = sophgo_dwmac_match,
	},
};
module_platform_driver(sophgo_dwmac_driver);

MODULE_AUTHOR("Inochi Amaoto <inochiama@gmail.com>");
MODULE_DESCRIPTION("Sophgo DWMAC platform driver");
MODULE_LICENSE("GPL");
