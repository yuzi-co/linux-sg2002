// SPDX-License-Identifier: GPL-2.0-only
/*
 * Sophgo SG2002 DesignWare NIST TRNG.
 * Command sequence based on drivers/hw_random/starfive-vic-rng.c in
 * Barebox, copyright 2020 Shanghai StarFive Technology Co., Ltd., and
 * Synopsys's dw_nist_trng reference driver in embARC OSP.
 */

#include <linux/clk.h>
#include <linux/hw_random.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#define SG2002_RNG_CTRL		0x00
#define SG2002_RNG_MODE		0x04
#define SG2002_RNG_SMODE		0x08
#define SG2002_RNG_STAT		0x0c
#define SG2002_RNG_IE		0x10
#define SG2002_RNG_ISTAT		0x14
#define SG2002_RNG_ALARM		0x18
#define SG2002_RNG_DATA		0x24
#define SG2002_RNG_BUSY		BIT(31)
#define SG2002_RNG_DONE		BIT(4)
#define SG2002_RNG_ALARMS		BIT(3)
#define SG2002_RNG_ZEROIZED	BIT(0)
#define SG2002_RNG_NOISE		1
#define SG2002_RNG_CREATE_STATE	3
#define SG2002_RNG_GENERATE	6
#define SG2002_RNG_ZEROIZE	15
#define SG2002_RNG_TIMEOUT_US	1000000

struct sg2002_rng {
	struct hwrng rng;
	struct device *dev;
	void __iomem *base;
	struct clk_bulk_data clocks[2];
	u32 data[4];
	u32 previous[4];
	size_t available;
	bool have_previous;
	bool failed;
};

static int sg2002_rng_command(struct sg2002_rng *priv, u32 command)
{
	u32 status;
	u32 done = command == SG2002_RNG_ZEROIZE ?
		   SG2002_RNG_ZEROIZED : SG2002_RNG_DONE;
	int ret;

	ret = readl_poll_timeout(priv->base + SG2002_RNG_STAT, status,
				 !(status & SG2002_RNG_BUSY), 10,
				 SG2002_RNG_TIMEOUT_US);
	if (ret)
		return ret;

	/* A completion left by firmware or a previous command is not ours. */
	writel(done, priv->base + SG2002_RNG_ISTAT);
	writel(command, priv->base + SG2002_RNG_CTRL);
	ret = readl_poll_timeout(priv->base + SG2002_RNG_ISTAT, status,
				 status & (done | SG2002_RNG_ALARMS), 10,
				 SG2002_RNG_TIMEOUT_US);
	if (status & SG2002_RNG_ALARMS) {
		dev_err_ratelimited(priv->dev, "RNG alarm %#x\n",
				    readl(priv->base + SG2002_RNG_ALARM));
		return -EIO;
	}
	if (!ret)
		writel(done, priv->base + SG2002_RNG_ISTAT);
	return ret;
}

static int sg2002_rng_generate(struct sg2002_rng *priv)
{
	int ret, i;

	/* Start each block with a fresh noise seed, as in the Barebox driver.
	 * CREATE_STATE is only valid after zeroization and seed generation.
	 */
	ret = sg2002_rng_command(priv, SG2002_RNG_ZEROIZE);
	if (ret)
		goto fail;
	ret = sg2002_rng_command(priv, SG2002_RNG_NOISE);
	if (ret)
		goto fail;
	ret = sg2002_rng_command(priv, SG2002_RNG_CREATE_STATE);
	if (ret)
		goto fail;
	ret = sg2002_rng_command(priv, SG2002_RNG_GENERATE);
	if (ret)
		goto fail;

	for (i = 0; i < ARRAY_SIZE(priv->data); i++)
		priv->data[i] = readl(priv->base + SG2002_RNG_DATA + i * 4);
	ret = sg2002_rng_command(priv, SG2002_RNG_ZEROIZE);
	if (ret)
		goto fail;

	/* Detect a stuck output; this does not establish an entropy estimate. */
	if (priv->have_previous &&
	    !memcmp(priv->previous, priv->data, sizeof(priv->data))) {
		ret = -EIO;
		goto fail;
	}
	memcpy(priv->previous, priv->data, sizeof(priv->data));
	priv->have_previous = true;
	priv->available = sizeof(priv->data);
	return 0;
fail:
	/* Fail closed until the provider is reinitialized. */
	priv->failed = true;
	priv->available = 0;
	memzero_explicit(priv->data, sizeof(priv->data));
	return ret;
}

static int sg2002_rng_init(struct hwrng *rng)
{
	struct sg2002_rng *priv = container_of(rng, struct sg2002_rng, rng);
	int ret;

	ret = clk_bulk_prepare_enable(ARRAY_SIZE(priv->clocks), priv->clocks);
	if (ret)
		return ret;
	/* Clear stale W1C status before starting a new provider lifetime. */
	writel(0, priv->base + SG2002_RNG_IE);
	writel(0x1f, priv->base + SG2002_RNG_ALARM);
	writel(0x1f, priv->base + SG2002_RNG_ISTAT);
	/* AES-128 DRBG, internal noise, secure mode, ten rejected seeds. */
	writel(0, priv->base + SG2002_RNG_MODE);
	writel((10 << 2) | BIT(1), priv->base + SG2002_RNG_SMODE);
	priv->failed = false;
	priv->have_previous = false;
	priv->available = 0;
	ret = sg2002_rng_generate(priv);
	if (ret)
		clk_bulk_disable_unprepare(ARRAY_SIZE(priv->clocks), priv->clocks);
	return ret;
}

static void sg2002_rng_cleanup(struct hwrng *rng)
{
	struct sg2002_rng *priv = container_of(rng, struct sg2002_rng, rng);

	if (!priv->failed)
		sg2002_rng_command(priv, SG2002_RNG_ZEROIZE);
	priv->available = 0;
	memzero_explicit(priv->data, sizeof(priv->data));
	memzero_explicit(priv->previous, sizeof(priv->previous));
	clk_bulk_disable_unprepare(ARRAY_SIZE(priv->clocks), priv->clocks);
}

static int sg2002_rng_read(struct hwrng *rng, void *data, size_t max, bool wait)
{
	struct sg2002_rng *priv = container_of(rng, struct sg2002_rng, rng);
	size_t count;
	int ret;

	if (priv->failed)
		return -EIO;
	if (!max)
		return 0;
	if (!priv->available) {
		if (!wait)
			return 0;
		ret = sg2002_rng_generate(priv);
		if (ret)
			return ret;
	}
	count = min(max, priv->available);
	memcpy(data, (u8 *)priv->data + sizeof(priv->data) - priv->available, count);
	priv->available -= count;
	return count;
}

static int sg2002_rng_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sg2002_rng *priv;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	priv->dev = dev;
	priv->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	/* The security APB window hangs if the shared eFuse clocks are gated. */
	priv->clocks[0].id = "core";
	priv->clocks[1].id = "apb";
	ret = devm_clk_bulk_get(dev, ARRAY_SIZE(priv->clocks), priv->clocks);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get clocks\n");
	priv->rng.name = dev_name(dev);
	priv->rng.init = sg2002_rng_init;
	priv->rng.cleanup = sg2002_rng_cleanup;
	priv->rng.read = sg2002_rng_read;
	priv->rng.quality = 900;
	return devm_hwrng_register(dev, &priv->rng);
}

static const struct of_device_id sg2002_rng_match[] = {
	{ .compatible = "sophgo,sg2002-trng" },
	{ }
};
MODULE_DEVICE_TABLE(of, sg2002_rng_match);

static struct platform_driver sg2002_rng_driver = {
	.probe = sg2002_rng_probe,
	.driver = {
		.name = "sg2002-rng",
		.of_match_table = sg2002_rng_match,
	},
};
module_platform_driver(sg2002_rng_driver);

MODULE_DESCRIPTION("Sophgo SG2002 hardware random number generator");
MODULE_AUTHOR("George Whewell <georgerw@gmail.com>");
MODULE_LICENSE("GPL");
