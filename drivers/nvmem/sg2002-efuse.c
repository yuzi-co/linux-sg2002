// SPDX-License-Identifier: GPL-2.0-only
/* Read-only access to the SG2002's public eFuse shadow words. */

#include <linux/clk.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/nvmem-provider.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#define SG2002_EFUSE_SHADOW 0x100
#define SG2002_EFUSE_SIZE 0x100

struct sg2002_efuse {
	void __iomem *base;
	struct clk_bulk_data clocks[2];
	struct mutex lock;
};

/*
 * Only the eight-byte factory UID, the ten user words and the eight-byte
 * device ID are public here. Sophgo's vendor kernel reads the UID from
 * shadow words 0x0c and 0x10. Keep the physical shadow offsets: user
 * words have four-byte gaps. NVMEM fills all other ranges with zero
 * without reading the hardware. In particular, this excludes the loader
 * and device encryption keys.
 */
static const struct nvmem_keepout sg2002_efuse_keepout[] = {
	{ .start = 0x00, .end = 0x0c },
	{ .start = 0x14, .end = 0x40 },
	{ .start = 0x44, .end = 0x48 },
	{ .start = 0x4c, .end = 0x50 },
	{ .start = 0x54, .end = 0x58 },
	{ .start = 0x5c, .end = 0x60 },
	{ .start = 0x64, .end = 0x68 },
	{ .start = 0x6c, .end = 0x70 },
	{ .start = 0x74, .end = 0x78 },
	{ .start = 0x7c, .end = 0x80 },
	{ .start = 0x84, .end = 0x88 },
	{ .start = 0x94, .end = SG2002_EFUSE_SIZE },
};

static int sg2002_efuse_read(void *context, unsigned int offset,
			   void *val, size_t bytes)
{
	struct sg2002_efuse *efuse = context;
	u8 *buf = val;
	int ret;

	mutex_lock(&efuse->lock);
	ret = clk_bulk_prepare_enable(ARRAY_SIZE(efuse->clocks), efuse->clocks);
	if (ret)
		goto unlock;

	/* The shadow aperture supports word reads, including for byte cells. */
	while (bytes) {
		unsigned int skip = offset % sizeof(u32);
		size_t count = min_t(size_t, bytes, sizeof(u32) - skip);
		__le32 word;

		word = cpu_to_le32(readl(efuse->base + SG2002_EFUSE_SHADOW +
					(offset & ~3U)));
		memcpy(buf, (u8 *)&word + skip, count);
		buf += count;
		offset += count;
		bytes -= count;
	}

	clk_bulk_disable_unprepare(ARRAY_SIZE(efuse->clocks), efuse->clocks);
unlock:
	mutex_unlock(&efuse->lock);
	return ret;
}

static int sg2002_efuse_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct nvmem_config config = {
		.dev = dev,
		.name = "sg2002-efuse",
		.id = NVMEM_DEVID_AUTO,
		.type = NVMEM_TYPE_OTP,
		.owner = THIS_MODULE,
		.size = SG2002_EFUSE_SIZE,
		.word_size = 1,
		.stride = 1,
		.read_only = true,
		.root_only = true,
		.reg_read = sg2002_efuse_read,
		.keepout = sg2002_efuse_keepout,
		.nkeepout = ARRAY_SIZE(sg2002_efuse_keepout),
	};
	struct sg2002_efuse *efuse;
	int ret;

	efuse = devm_kzalloc(dev, sizeof(*efuse), GFP_KERNEL);
	if (!efuse)
		return -ENOMEM;

	efuse->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(efuse->base))
		return PTR_ERR(efuse->base);

	efuse->clocks[0].id = "efuse";
	efuse->clocks[1].id = "apb";
	ret = devm_clk_bulk_get(dev, ARRAY_SIZE(efuse->clocks), efuse->clocks);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get clocks\n");

	ret = devm_mutex_init(dev, &efuse->lock);
	if (ret)
		return ret;

	config.priv = efuse;
	return PTR_ERR_OR_ZERO(devm_nvmem_register(dev, &config));
}

static const struct of_device_id sg2002_efuse_match[] = {
	{ .compatible = "sophgo,sg2002-efuse" },
	{ }
};
MODULE_DEVICE_TABLE(of, sg2002_efuse_match);

static struct platform_driver sg2002_efuse_driver = {
	.probe = sg2002_efuse_probe,
	.driver = {
		.name = "sg2002-efuse",
		.of_match_table = sg2002_efuse_match,
	},
};
module_platform_driver(sg2002_efuse_driver);

MODULE_DESCRIPTION("Sophgo SG2002 public eFuse shadow reader");
MODULE_AUTHOR("George Whewell <georgerw@gmail.com>");
MODULE_LICENSE("GPL");
