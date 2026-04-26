// SPDX-License-Identifier: GPL-2.0
/*
 * Sophgo CV1800 PWM driver.
 *
 * Four channels per IP instance, each with its own period/hlperiod
 * 32-bit register pair. Polarity / output-enable / start / done /
 * update bits live in shared per-instance bitfields.
 *
 * Register-level reference: u-boot-2021.10/drivers/pwm/cvitek_pwm.c
 * from milkv-duo/duo-buildroot-sdk. The vendor BSP ships *only* a
 * U-Boot driver — there's no Linux kernel one in the 5.10 tree —
 * so this is a fresh mainline driver written against that as the
 * SoC reference, using the modern pwm_chip / .apply() API.
 */
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>

#define CV1800_PWM_HLPERIOD(n)		(0x00 + (n) * 8)
#define CV1800_PWM_PERIOD(n)		(0x04 + (n) * 8)
#define CV1800_PWM_POLARITY		0x40
#define CV1800_PWM_START		0x44
#define CV1800_PWM_OE			0xd0

#define CV1800_PWM_NUM_CHANNELS		4

struct cv1800_pwm {
	void __iomem *base;
	struct clk *clk;
};

static inline struct cv1800_pwm *to_cv1800_pwm(struct pwm_chip *chip)
{
	return pwmchip_get_drvdata(chip);
}

static int cv1800_pwm_apply(struct pwm_chip *chip, struct pwm_device *pwm,
			    const struct pwm_state *state)
{
	struct cv1800_pwm *priv = to_cv1800_pwm(chip);
	u32 mask = BIT(pwm->hwpwm);
	unsigned long rate;
	u64 period_cycles, duty_cycles, hl_cycles;
	u32 val;

	rate = clk_get_rate(priv->clk);
	if (!rate)
		return -EINVAL;

	period_cycles = mul_u64_u64_div_u64(state->period, rate, NSEC_PER_SEC);
	duty_cycles   = mul_u64_u64_div_u64(state->duty_cycle, rate, NSEC_PER_SEC);

	/* Hardware is a 32-bit cycle counter; clamp accordingly. We
	 * also follow the vendor U-Boot driver's policy of avoiding
	 * exactly-0 and exactly-100% duty (the IP can produce them but
	 * vendor's tested path stays one cycle inside both extremes). */
	if (period_cycles < 2)
		period_cycles = 2;
	if (period_cycles > U32_MAX)
		period_cycles = U32_MAX;

	if (duty_cycles == 0)
		duty_cycles = 1;
	if (duty_cycles >= period_cycles)
		duty_cycles = period_cycles - 1;

	hl_cycles = period_cycles - duty_cycles;

	val = readl(priv->base + CV1800_PWM_POLARITY);
	if (state->polarity == PWM_POLARITY_INVERSED)
		val |= mask;
	else
		val &= ~mask;
	writel(val, priv->base + CV1800_PWM_POLARITY);

	/* Pause the channel before re-arming with new period/duty so
	 * a half-written pair isn't latched into the running output. */
	val = readl(priv->base + CV1800_PWM_START);
	writel(val & ~mask, priv->base + CV1800_PWM_START);

	writel((u32)hl_cycles,     priv->base + CV1800_PWM_HLPERIOD(pwm->hwpwm));
	writel((u32)period_cycles, priv->base + CV1800_PWM_PERIOD(pwm->hwpwm));

	val = readl(priv->base + CV1800_PWM_OE);
	if (state->enabled)
		val |= mask;
	else
		val &= ~mask;
	writel(val, priv->base + CV1800_PWM_OE);

	if (state->enabled) {
		val = readl(priv->base + CV1800_PWM_START);
		writel(val | mask, priv->base + CV1800_PWM_START);
	}

	return 0;
}

static int cv1800_pwm_get_state(struct pwm_chip *chip, struct pwm_device *pwm,
				struct pwm_state *state)
{
	struct cv1800_pwm *priv = to_cv1800_pwm(chip);
	u32 mask = BIT(pwm->hwpwm);
	unsigned long rate;
	u32 period_cycles, hl_cycles, polarity, start;

	rate = clk_get_rate(priv->clk);
	if (!rate)
		return -EINVAL;

	period_cycles = readl(priv->base + CV1800_PWM_PERIOD(pwm->hwpwm));
	hl_cycles     = readl(priv->base + CV1800_PWM_HLPERIOD(pwm->hwpwm));
	polarity      = readl(priv->base + CV1800_PWM_POLARITY);
	start         = readl(priv->base + CV1800_PWM_START);

	state->period     = DIV_ROUND_CLOSEST_ULL((u64)period_cycles * NSEC_PER_SEC, rate);
	state->duty_cycle = DIV_ROUND_CLOSEST_ULL((u64)(period_cycles - hl_cycles) * NSEC_PER_SEC, rate);
	state->polarity   = (polarity & mask) ? PWM_POLARITY_INVERSED : PWM_POLARITY_NORMAL;
	state->enabled    = (start & mask) != 0;

	return 0;
}

static const struct pwm_ops cv1800_pwm_ops = {
	.apply = cv1800_pwm_apply,
	.get_state = cv1800_pwm_get_state,
};

static int cv1800_pwm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pwm_chip *chip;
	struct cv1800_pwm *priv;

	chip = devm_pwmchip_alloc(dev, CV1800_PWM_NUM_CHANNELS, sizeof(*priv));
	if (IS_ERR(chip))
		return PTR_ERR(chip);
	priv = to_cv1800_pwm(chip);

	priv->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	priv->clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(priv->clk))
		return dev_err_probe(dev, PTR_ERR(priv->clk),
				     "failed to enable PWM clock\n");

	chip->ops = &cv1800_pwm_ops;

	return devm_pwmchip_add(dev, chip);
}

static const struct of_device_id cv1800_pwm_of_match[] = {
	{ .compatible = "sophgo,cv1800-pwm" },
	{}
};
MODULE_DEVICE_TABLE(of, cv1800_pwm_of_match);

static struct platform_driver cv1800_pwm_driver = {
	.probe = cv1800_pwm_probe,
	.driver = {
		.name = "cv1800-pwm",
		.of_match_table = cv1800_pwm_of_match,
	},
};
module_platform_driver(cv1800_pwm_driver);

MODULE_DESCRIPTION("Sophgo CV1800 PWM driver");
MODULE_LICENSE("GPL");
