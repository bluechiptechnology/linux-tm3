/*
 * drivers/pwm/pwm-sunxi.c
 *
 * Allwinnertech pulse-width-modulation controller driver
 *
 * Copyright (C) 2015 AllWinner
 *
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */
#include <linux/types.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/pwm.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/pinctrl/pinconf.h>
/*#include <linux/sunxi-gpio.h>*/
#include <linux/pinctrl/consumer.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_iommu.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>

#include <linux/io.h>
#include <linux/clk.h>

#define PWM_DEBUG 0
#define PWM_NUM_MAX 4

#define PWM_PIN_STATE_ACTIVE "active"
#define PWM_PIN_STATE_SLEEP "sleep"

#define SETMASK(width, shift)   ((width?((-1U) >> (32-width)):0)  << (shift))
#define CLRMASK(width, shift)   (~(SETMASK(width, shift)))
#define GET_BITS(shift, width, reg)     \
		(((reg) & SETMASK(width, shift)) >> (shift))
#define SET_BITS(shift, width, reg, val) \
	    (((reg) & CLRMASK(width, shift)) | (val << (shift)))

#if PWM_DEBUG
#define pwm_debug(msg...) pr_info
#else
#define pwm_debug(msg...)
#endif

#if ((defined CONFIG_ARCH_SUN8IW12P1) ||\
			(defined CONFIG_ARCH_SUN8IW17P1) ||\
			(defined CONFIG_ARCH_SUN50IW6P1) ||\
			(defined CONFIG_ARCH_SUN8IW15P1) ||\
			(defined CONFIG_ARCH_SUN50IW3P1))
#define CLK_GATE_SUPPORT
#endif

struct sunxi_pwm_config {
	unsigned int reg_busy_offset;
	unsigned int reg_busy_shift;
	unsigned int reg_enable_offset;
	unsigned int reg_enable_shift;
	unsigned int reg_clk_gating_offset;
	unsigned int reg_clk_gating_shift;
	unsigned int reg_bypass_offset;
	unsigned int reg_bypass_shift;
	unsigned int reg_pulse_start_offset;
	unsigned int reg_pulse_start_shift;
	unsigned int reg_mode_offset;
	unsigned int reg_mode_shift;
	unsigned int reg_polarity_offset;
	unsigned int reg_polarity_shift;
	unsigned int reg_period_offset;
	unsigned int reg_period_shift;
	unsigned int reg_period_width;
	unsigned int reg_active_offset;
	unsigned int reg_active_shift;
	unsigned int reg_active_width;
	unsigned int reg_prescal_offset;
	unsigned int reg_prescal_shift;
	unsigned int reg_prescal_width;

};

struct sunxi_pwm_chip {
	struct pwm_chip chip;
	void __iomem *base;
	struct sunxi_pwm_config *config;
#if defined(CLK_GATE_SUPPORT)
	struct clk	*pwm_clk;
#endif
};

static inline struct sunxi_pwm_chip *to_sunxi_pwm_chip(struct pwm_chip *chip)
{
	return container_of(chip, struct sunxi_pwm_chip, chip);
}

static inline u32 sunxi_pwm_readl(struct pwm_chip *chip, u32 offset)
{
	struct sunxi_pwm_chip *pc = to_sunxi_pwm_chip(chip);
	u32 value = 0;

	value = readl(pc->base + offset);

	return value;
}

static inline u32 sunxi_pwm_writel(struct pwm_chip *chip, u32 offset, u32 value)
{
	struct sunxi_pwm_chip *pc = to_sunxi_pwm_chip(chip);

	writel(value, pc->base + offset);

	return 0;
}

static int sunxi_pwm_pin_set_state(struct device *dev, char *name)
{
	struct pinctrl *pctl;
	struct pinctrl_state *state;
	int ret = -1;

	pctl = pinctrl_get(dev);
	if (IS_ERR(pctl)) {
		dev_err(dev, "pinctrl_get failed!\n");
		ret = PTR_ERR(pctl);
		goto exit;
	}

	state = pinctrl_lookup_state(pctl, name);
	if (IS_ERR(state)) {
		dev_err(dev, "pinctrl_lookup_state(%s) failed!\n", name);
		ret = PTR_ERR(state);
		goto exit;
	}

	ret = pinctrl_select_state(pctl, state);
	if (ret < 0) {
		dev_err(dev, "pinctrl_select_state(%s) failed!\n", name);
		goto exit;
	}
	ret = 0;

exit:
	return ret;
}

static int sunxi_pwm_get_config(struct platform_device *pdev,
				struct sunxi_pwm_config *config)
{
	struct device_node *np = pdev->dev.of_node;
	int ret = 0;

	/* read register config */
	ret = of_property_read_u32(np,
			"reg_busy_offset", &config->reg_busy_offset);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"failed to get reg_busy_offset! err=%d\n", ret);
		goto err;
	}

	ret = of_property_read_u32(np,
			"reg_busy_shift", &config->reg_busy_shift);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"failed to get reg_busy_shift! err=%d\n", ret);
		goto err;
	}

	ret = of_property_read_u32(np,
			"reg_enable_offset", &config->reg_enable_offset);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"failed to get reg_enable_offset! err=%d\n", ret);
		goto err;
	}

	ret = of_property_read_u32(np,
			"reg_enable_shift", &config->reg_enable_shift);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"failed to get reg_enable_shift! err=%d\n", ret);
		goto err;
	}

	ret = of_property_read_u32(np,
		"reg_clk_gating_offset", &config->reg_clk_gating_offset);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"failed to get reg_clk_gating_offset! err=%d\n", ret);
		goto err;
	}

	ret = of_property_read_u32(np,
			"reg_clk_gating_shift", &config->reg_clk_gating_shift);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"failed to get reg_clk_gating_shift! err=%d\n", ret);
		goto err;
	}

	ret = of_property_read_u32(np,
			"reg_bypass_offset", &config->reg_bypass_offset);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"failed to get reg_bypass_offset! err=%d\n", ret);
		goto err;
	}

	ret = of_property_read_u32(np,
			"reg_bypass_shift", &config->reg_bypass_shift);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"failed to get reg_bypass_shift! err=%d\n", ret);
		goto err;
	}

	ret = of_property_read_u32(np,
		"reg_pulse_start_offset", &config->reg_pulse_start_offset);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"failed to get reg_bypass_offset! err=%d\n", ret);
		goto err;
	}

	ret = of_property_read_u32(np,
		"reg_pulse_start_shift", &config->reg_pulse_start_shift);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"failed to get reg_pulse_start_shift! err=%d\n", ret);
		goto err;
	}

	ret = of_property_read_u32(np,
			"reg_mode_offset", &config->reg_mode_offset);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"failed to get reg_mode_offset! err=%d\n", ret);
		goto err;
	}

	ret = of_property_read_u32(np,
			"reg_mode_shift", &config->reg_mode_shift);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"failed to get reg_mode_shift! err=%d\n", ret);
		goto err;
	}

	ret = of_property_read_u32(np,
			"reg_polarity_offset", &config->reg_polarity_offset);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"failed to get reg_polarity_offset! err=%d\n", ret);
		goto err;
	}

	ret = of_property_read_u32(np,
			"reg_polarity_shift", &config->reg_polarity_shift);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"failed to get reg_polarity_shift! err=%d\n", ret);
		goto err;
	}

	ret = of_property_read_u32(np,
			"reg_period_offset", &config->reg_period_offset);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"failed to get reg_period_offset! err=%d\n", ret);
		goto err;
	}

	ret = of_property_read_u32(np,
			"reg_period_shift", &config->reg_period_shift);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"failed to get reg_period_shift! err=%d\n", ret);
		goto err;
	}

	ret = of_property_read_u32(np,
			"reg_period_width", &config->reg_period_width);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"failed to get reg_period_width! err=%d\n", ret);
		goto err;
	}

	ret = of_property_read_u32(np,
			"reg_active_offset", &config->reg_active_offset);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"failed to get reg_duty_offset! err=%d\n", ret);
		goto err;
	}

	ret = of_property_read_u32(np,
			"reg_active_shift", &config->reg_active_shift);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"failed to get reg_duty_shift! err=%d\n", ret);
		goto err;
	}

	ret = of_property_read_u32(np,
			"reg_active_width", &config->reg_active_width);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"failed to get reg_duty_width! err=%d\n", ret);
		goto err;
	}

	ret = of_property_read_u32(np,
			"reg_prescal_offset", &config->reg_prescal_offset);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"failed to get reg_duty_width! err=%d\n", ret);
		goto err;
	}
	ret = of_property_read_u32(np,
			"reg_prescal_shift", &config->reg_prescal_shift);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"failed to get reg_duty_width! err=%d\n", ret);
		goto err;
	}
	ret = of_property_read_u32(np,
			"reg_prescal_width", &config->reg_prescal_width);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"failed to get reg_duty_width! err=%d\n", ret);
		goto err;
	}
err:

	of_node_put(np);

	return ret;
}

static int sunxi_pwm_set_polarity(struct pwm_chip *chip,
			struct pwm_device *pwm, enum pwm_polarity polarity)
{
	u32 temp;
	struct sunxi_pwm_chip *pc = to_sunxi_pwm_chip(chip);
	unsigned int reg_offset, reg_shift;

	reg_offset = pc->config[pwm->pwm - chip->base].reg_polarity_offset;
	reg_shift = pc->config[pwm->pwm - chip->base].reg_polarity_shift;
	temp = sunxi_pwm_readl(chip, reg_offset);
	if (polarity == PWM_POLARITY_NORMAL)
		temp = SET_BITS(reg_shift, 1, temp, 1);
	else
		temp = SET_BITS(reg_shift, 1, temp, 0);

	sunxi_pwm_writel(chip, reg_offset, temp);

	return 0;
}

static int sunxi_pwm_config(struct pwm_chip *chip, struct pwm_device *pwm,
		int duty_ns, int period_ns)
{
	u32 pre_scal[11][2] = {
		/* reg_value  clk_pre_div */
		{15, 1},
		{0, 120},
		{1, 180},
		{2, 240},
		{3, 360},
		{4, 480},
		{8, 12000},
		{9, 24000},
		{10, 36000},
		{11, 48000},
		{12, 72000}
		};
	u32 freq;
	u32 pre_scal_id = 0;
	u32 entire_cycles = 256;
	u32 active_cycles = 192;
	u32 entire_cycles_max = 65536;
	u32 temp;
	struct sunxi_pwm_chip *pc = to_sunxi_pwm_chip(chip);
	unsigned int reg_offset, reg_shift, reg_width;

	reg_offset = pc->config[pwm->pwm - chip->base].reg_bypass_offset;
	reg_shift = pc->config[pwm->pwm - chip->base].reg_bypass_shift;

	if (period_ns < 42) {
		/* if freq lt 24M, then direct output 24M clock */
		temp = sunxi_pwm_readl(chip, reg_offset);
		temp = SET_BITS(reg_shift, 1, temp, 1);
		sunxi_pwm_writel(chip, reg_offset, temp);
		return 0;
	}

	/* disable bypass function */
	temp = sunxi_pwm_readl(chip, reg_offset);
	temp = SET_BITS(reg_shift, 1, temp, 0);
	sunxi_pwm_writel(chip, reg_offset, temp);

	/*DPR default to active high*/
	reg_offset = pc->config[pwm->pwm - chip->base].reg_polarity_offset;
	reg_shift = pc->config[pwm->pwm - chip->base].reg_polarity_shift;
	temp = sunxi_pwm_readl(chip, reg_offset);
	temp = SET_BITS(reg_shift, 1, temp, 1);
	sunxi_pwm_writel(chip, reg_offset, temp);

	if (period_ns < 10667)
		freq = 93747;
	else if (period_ns > 1000000000)
		freq = 1;
	else
		freq = 1000000000 / period_ns;

	/* clock source rate is  24Mhz */
	entire_cycles = 24000000 / freq / pre_scal[pre_scal_id][1];

	while (entire_cycles > entire_cycles_max) {
		pre_scal_id++;

		if (pre_scal_id > 10)
			break;

		entire_cycles = 24000000 / freq / pre_scal[pre_scal_id][1];
	}

	if (period_ns < 5*100*1000)
		active_cycles = (duty_ns * entire_cycles + (period_ns/2))
								/ period_ns;
	else if (period_ns >= 5*100*1000 && period_ns < 6553500)
		active_cycles = ((duty_ns / 100) * entire_cycles +
				(period_ns / 2 / 100)) / (period_ns/100);
	else
		active_cycles = ((duty_ns / 10000) * entire_cycles +
				(period_ns / 2 / 10000)) / (period_ns/10000);

	/* config prescal */
	reg_offset = pc->config[pwm->pwm - chip->base].reg_prescal_offset;
	reg_shift = pc->config[pwm->pwm - chip->base].reg_prescal_shift;
	reg_width = pc->config[pwm->pwm - chip->base].reg_prescal_width;

	temp = sunxi_pwm_readl(chip, reg_offset);
	temp = SET_BITS(reg_shift, reg_width, temp, (pre_scal[pre_scal_id][0]));
	sunxi_pwm_writel(chip, reg_offset, temp);

	/* config active cycles */
	reg_offset = pc->config[pwm->pwm - chip->base].reg_active_offset;
	reg_shift = pc->config[pwm->pwm - chip->base].reg_active_shift;
	reg_width = pc->config[pwm->pwm - chip->base].reg_active_width;

	temp = sunxi_pwm_readl(chip, reg_offset);
	temp = SET_BITS(reg_shift, reg_width, temp, (active_cycles));
	sunxi_pwm_writel(chip, reg_offset, temp);

	/* config period cycles */
	reg_offset = pc->config[pwm->pwm - chip->base].reg_period_offset;
	reg_shift = pc->config[pwm->pwm - chip->base].reg_period_shift;
	reg_width = pc->config[pwm->pwm - chip->base].reg_period_width;
	temp = sunxi_pwm_readl(chip, reg_offset);
	temp = SET_BITS(reg_shift, reg_width, temp, (entire_cycles - 1));

	sunxi_pwm_writel(chip, reg_offset, temp);

	return 0;
}

static int sunxi_pwm_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	u32 value = 0, index = 0;
	struct sunxi_pwm_chip *pc = to_sunxi_pwm_chip(chip);
	unsigned int reg_offset, reg_shift;
	struct device_node *sub_np;
	struct platform_device *pwm_pdevice;

	index = pwm->pwm - chip->base;
	sub_np = of_parse_phandle(chip->dev->of_node, "pwms", index);
	if (IS_ERR_OR_NULL(sub_np)) {
		pr_err("%s: can't parse \"pwms\" property\n", __func__);
		return -ENODEV;
	}
	pwm_pdevice = of_find_device_by_node(sub_np);
	if (IS_ERR_OR_NULL(pwm_pdevice)) {
		pr_err("%s: can't parse pwm device\n", __func__);
		return -ENODEV;
	}
	sunxi_pwm_pin_set_state(&pwm_pdevice->dev, PWM_PIN_STATE_ACTIVE);

	/* enable clk for pwm controller */
	reg_offset = pc->config[pwm->pwm - chip->base].reg_clk_gating_offset;
	reg_shift = pc->config[pwm->pwm - chip->base].reg_clk_gating_shift;
	value = sunxi_pwm_readl(chip, reg_offset);
	value = SET_BITS(reg_shift, 1, value, 1);
	sunxi_pwm_writel(chip, reg_offset, value);

	/* enable pwm controller */
	reg_offset = pc->config[pwm->pwm - chip->base].reg_enable_offset;
	reg_shift = pc->config[pwm->pwm - chip->base].reg_enable_shift;
	value = sunxi_pwm_readl(chip, reg_offset);
	value = SET_BITS(reg_shift, 1, value, 1);
	sunxi_pwm_writel(chip, reg_offset, value);

	return 0;
}

static void sunxi_pwm_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	u32 value = 0, index = 0;
	struct sunxi_pwm_chip *pc = to_sunxi_pwm_chip(chip);
	unsigned int reg_offset, reg_shift;
	struct device_node *sub_np;
	struct platform_device *pwm_pdevice;

	index = pwm->pwm - chip->base;
	sub_np = of_parse_phandle(chip->dev->of_node, "pwms", index);
	if (IS_ERR_OR_NULL(sub_np)) {
		pr_err("%s: can't parse \"pwms\" property\n", __func__);
		return;
	}
	pwm_pdevice = of_find_device_by_node(sub_np);
	if (IS_ERR_OR_NULL(pwm_pdevice)) {
		pr_err("%s: can't parse pwm device\n", __func__);
		return;
	}

	/* disable pwm controller */
	reg_offset = pc->config[pwm->pwm - chip->base].reg_enable_offset;
	reg_shift = pc->config[pwm->pwm - chip->base].reg_enable_shift;
	value = sunxi_pwm_readl(chip, reg_offset);
	value = SET_BITS(reg_shift, 1, value, 0);
	sunxi_pwm_writel(chip, reg_offset, value);

	/* disable pwm controller */
	reg_offset = pc->config[pwm->pwm - chip->base].reg_clk_gating_offset;
	reg_shift = pc->config[pwm->pwm - chip->base].reg_clk_gating_shift;
	value = sunxi_pwm_readl(chip, reg_offset);
	value = SET_BITS(reg_shift, 1, value, 0);
	sunxi_pwm_writel(chip, reg_offset, value);

	sunxi_pwm_pin_set_state(&pwm_pdevice->dev, PWM_PIN_STATE_SLEEP);
}

static struct pwm_ops sunxi_pwm_ops = {
	.config = sunxi_pwm_config,
	.enable = sunxi_pwm_enable,
	.disable = sunxi_pwm_disable,
	.set_polarity = sunxi_pwm_set_polarity,
	.owner = THIS_MODULE,
};

static int sunxi_pwm_probe(struct platform_device *pdev)
{
	int ret;
	struct sunxi_pwm_chip *pwm;
	struct device_node *np = pdev->dev.of_node;
	int i;
	struct platform_device *pwm_pdevice;
	struct device_node *sub_np;

	pwm = devm_kzalloc(&pdev->dev, sizeof(*pwm), GFP_KERNEL);
	if (!pwm) {
		ret = -ENOMEM;
		dev_err(&pdev->dev, "failed to allocate memory!\n");
		return ret;
	}

	/* io map pwm base */
	pwm->base = (void __iomem *)of_iomap(pdev->dev.of_node, 0);
	if (!pwm->base) {
		dev_err(&pdev->dev, "unable to map pwm registers\n");
		ret = -EINVAL;
		goto err_iomap;
	}

	/* read property pwm-number */
	ret = of_property_read_u32(np, "pwm-number", &pwm->chip.npwm);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"failed to get pwm number: %d, force to one!\n", ret);
		/* force to one pwm if read property fail */
		pwm->chip.npwm = 1;
	}

	/* read property pwm-base */
	ret = of_property_read_u32(np, "pwm-base", &pwm->chip.base);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"failed to get pwm-base: %d, force to -1 !\n", ret);
		/* force to one pwm if read property fail */
		pwm->chip.base = -1;
	}
	pwm->chip.dev = &pdev->dev;
	pwm->chip.ops = &sunxi_pwm_ops;

	/* add pwm chip to pwm-core */
	ret = pwmchip_add(&pwm->chip);
	if (ret < 0) {
		dev_err(&pdev->dev, "pwmchip_add() failed: %d\n", ret);
		goto err_add;
	}
	platform_set_drvdata(pdev, pwm);

	pwm->config = devm_kzalloc(&pdev->dev,
			sizeof(*pwm->config) * pwm->chip.npwm, GFP_KERNEL);
	if (!pwm->config) {
		ret = -ENOMEM;
		dev_err(&pdev->dev, "failed to allocate memory!\n");
		goto err_alloc;
	}

	for (i = 0; i < pwm->chip.npwm; i++) {
		sub_np = of_parse_phandle(np, "pwms", i);
		if (IS_ERR_OR_NULL(sub_np)) {
			pr_err("%s: can't parse \"pwms\" property\n",
				__func__);
			return -EINVAL;
		}

		pwm_pdevice = of_find_device_by_node(sub_np);
		ret =	sunxi_pwm_get_config(pwm_pdevice, &pwm->config[i]);
		if (ret)
			goto err_get_config;
	}

#if defined(CLK_GATE_SUPPORT)
	pwm->pwm_clk = of_clk_get(pdev->dev.of_node, 0);
	if (IS_ERR_OR_NULL(pwm->pwm_clk)) {
		pr_err("%s: can't get pwm clk\n", __func__);
		return -EINVAL;
	}
	clk_prepare_enable(pwm->pwm_clk);
#endif
	return 0;

err_get_config:
err_alloc:
	pwmchip_remove(&pwm->chip);
err_add:
	iounmap(pwm->base);
err_iomap:
	return ret;
}

static int sunxi_pwm_remove(struct platform_device *pdev)
{
	struct sunxi_pwm_chip *pwm = platform_get_drvdata(pdev);
#if defined CLK_GATE_SUPPORT
	clk_disable(pwm->pwm_clk);
#endif
	return pwmchip_remove(&pwm->chip);
}

static int sunxi_pwm_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int sunxi_pwm_resume(struct platform_device *pdev)
{
	return 0;
}

#if !defined(CONFIG_OF)
struct platform_device sunxi_pwm_device = {
	.name = "sunxi_pwm",
	.id = -1,
};
#else
static const struct of_device_id sunxi_pwm_match[] = {
	{ .compatible = "allwinner,sunxi-pwm", },
	{ .compatible = "allwinner,sunxi-s_pwm", },
	{},
};
#endif

static struct platform_driver sunxi_pwm_driver = {
	.probe = sunxi_pwm_probe,
	.remove = sunxi_pwm_remove,
	.suspend = sunxi_pwm_suspend,
	.resume = sunxi_pwm_resume,
	.driver = {
		.name = "sunxi_pwm",
		.owner  = THIS_MODULE,
		.of_match_table = sunxi_pwm_match,
	 },
};

static int __init pwm_module_init(void)
{
	int ret = 0;

	pr_info("pwm module init!\n");

#if !defined(CONFIG_OF)
	ret = platform_device_register(&sunxi_pwm_device);
#endif
	if (ret == 0)
		ret = platform_driver_register(&sunxi_pwm_driver);

	return ret;
}

static void __exit pwm_module_exit(void)
{
	pr_info("pwm module exit!\n");

	platform_driver_unregister(&sunxi_pwm_driver);
#if !defined(CONFIG_OF)
	platform_device_unregister(&sunxi_pwm_device);
#endif
}

subsys_initcall(pwm_module_init);
module_exit(pwm_module_exit);

MODULE_AUTHOR("tyle");
MODULE_DESCRIPTION("pwm driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:sunxi-pwm");
