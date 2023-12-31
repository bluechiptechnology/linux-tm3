/*
 * PCIe RC driver for Allwinner Core
 *
 * Copyright (C) 2016 Allwinner Co., Ltd.
 *
 * Author: wangjx <wangjx@allwinnertech.com>
 *	   ysn <yangshounan@allwinnertech.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/resource.h>
#include <linux/signal.h>
#include <linux/types.h>
#include <linux/sunxi-gpio.h>

#include "pcie-designware.h"

#define PCIE_LINK_WIDTH_SPEED_CONTROL		0x80C
#define PORT_LOGIC_SPEED_CHANGE			(0x1 << 17)
#define PORT_LOGIC_LINK_WIDTH_MASK		(0x1ff << 8)

#define USER_DEFINED_REGISTER_LIST		0x1000
#define PCIE_LTSSM_ENABLE			0x00
#define PCIE_INT_PENDING			0x18
#define PCIE_AWMISC_INF0_CTRL			0x30
#define PCIE_ARMISC_INF0_CTRL			0X34
#define PCIE_LINK_STATUS			0X43C
#define PCIE_PHY_CFG				0x800
#define SYS_CLK					0
#define PAD_CLK					1
#define RDLH_LINK_UP				(1<<1)
#define SMLH_LINK_UP				(1<<0)
#define PCIE_LINK_TRAINING			(1<<0)
#define PCIE_LINK_UP_MASK			(0x3<<16)
#define LINK_WAIT_MAX_RETRIES			10
#define LINK_WAIT_USLEEP_MIN			90000
#define LINK_WAIT_USLEEP_MAX			100000

#define PCIE_LINK_WIDTH_SPEED_CONTROL		0x80C
#define PORT_LOGIC_SPEED_CHANGE			(0x1 << 17)
#define LINK_CONTROL2_LINK_STATUS2		0xa0
#define PCIE_MSI_ADDR_LO			0x820
#define PCIE_MSI_ADDR_HI			0x824
#define PCIE_MSI_INTR0_ENABLE			0x828
#define PCIE_MSI_INTR0_MASK			0x82C
#define PCIE_MSI_INTR0_STATUS			0x830

#define PCIE_ATU_VIEWPORT			0x900
#define PCIE_ATU_REGION_INBOUND			(0x1 << 31)
#define PCIE_ATU_REGION_OUTBOUND		(0x0 << 31)
#define PCIE_ATU_REGION_INDEX2			(0x2 << 0)
#define PCIE_ATU_REGION_INDEX1			(0x1 << 0)
#define PCIE_ATU_REGION_INDEX0			(0x0 << 0)
#define PCIE_ATU_CR1				0x904
#define PCIE_ATU_TYPE_MEM			(0x0 << 0)
#define PCIE_ATU_TYPE_IO			(0x2 << 0)
#define PCIE_ATU_TYPE_CFG0			(0x4 << 0)
#define PCIE_ATU_TYPE_CFG1			(0x5 << 0)
#define PCIE_ATU_CR2				0x908
#define PCIE_ATU_ENABLE				(0x1 << 31)
#define PCIE_ATU_BAR_MODE_ENABLE		(0x1 << 30)
#define PCIE_ATU_LOWER_BASE			0x90C
#define PCIE_ATU_UPPER_BASE			0x910
#define PCIE_ATU_LIMIT				0x914
#define PCIE_ATU_LOWER_TARGET			0x918
#define PCIE_ATU_BUS(x)				(((x) & 0xff) << 24)
#define PCIE_ATU_DEV(x)				(((x) & 0x1f) << 19)
#define PCIE_ATU_FUNC(x)			(((x) & 0x7) << 16)
#define PCIE_ATU_UPPER_TARGET			0x91C

#define PCIE_MISC_CONTROL_1_CFG			0x8bc
#define PCIE_ADDR_PAGE_CFG			0x1020
#define PCIE_AWMISC_CTRL			0x1030
#define PCIE_ARMISC_CTRL			0x1034
#define PCIE_PAGE				0x10000
#define PCIE_ADDRESS_ALIGNING			(~0x3)
#define PCIE_HIGH_16				16
#define PCIE_BAR_NUM				6
#define PCIE_IO_FLAGS				0x1
#define PCIE_BAR_REG				0x4
#define PCIE_HIGH16_MASK			0xffff0000
#define PCIE_LOW16_MASK				0x0000ffff
#define PCIE_INTERRUPT_LINE_MASK		0xffff00ff
#define PCIE_INTERRUPT_LINE_ENABLE		0x00000100
#define	PCIE_PRIMARY_BUS_MASK			0xff000000
#define PCIE_PRIMARY_BUS_ENABLE			0x00010100
#define PCIE_MEMORY_MASK			0xfff00000

void __iomem *dbi_base;
static void __iomem *mem_base_start;

enum gpio_type {
	PCIE_REST = 0,
	PCIE_POWER,
	PCIE_REG,
	MAX_GPIO_NUM,
};

enum power_type {
	PCIE_VDD = 0,
	PCIE_VCC,
	PCIE_VCC_SLOT,
	MAX_POWER_NUM,
};

struct power {
	struct regulator *pmic;
	int power_vol;
	char power_str[32];
};

struct sunxi_pcie {
	struct device		dev;
	void __iomem		*dbi_base;
	void __iomem		*app_base;
	int			link_irq;
	int			msi_irq;
	int			speed_gen;
	struct pcie_port	pp;
	struct clk		*pcie_ref;
	struct clk		*pcie_axi;
	struct clk		*pcie_aux;
	struct clk		*pcie_bus;
	struct clk		*pcie_power;
	struct clk		*pcie_rst;
	int			iodvdd;
#ifdef CONFIG_PM
	u32			msi_enable;
	u32			buffer[16];
#endif
	struct gpio_config	gpio[MAX_GPIO_NUM];
	struct power		power[MAX_POWER_NUM];
};
#define to_sunxi_pcie(x)  container_of((x), struct sunxi_pcie, pp)

static inline u32 sunxi_pcie_readl(struct sunxi_pcie *pcie, u32 offset)
{
	return readl(pcie->app_base + offset);
}

static inline void sunxi_pcie_writel(u32 val, struct sunxi_pcie *pcie, u32 offset)
{
	writel(val, pcie->app_base + offset);
}

static inline void sunxi_pcie_writel_rcl(struct pcie_port *pp, u32 val, u32 reg)
{
	writel(val, pp->dbi_base + reg);
}

static inline u32 sunxi_pcie_readl_rcl(struct pcie_port *pp, u32 reg)
{
	return readl(pp->dbi_base + reg);
}

static int sunxi_pcie_cfg_read(void __iomem *addr, int size, u32 *val)
{
	if ((uintptr_t)addr & (size - 1)) {
		*val = 0;
		return PCIBIOS_BAD_REGISTER_NUMBER;
	}

	if (size == 4)
		*val = readl(addr);
	else if (size == 2)
		*val = readw(addr);
	else if (size == 1)
		*val = readb(addr);
	else {
		*val = 0;
		return PCIBIOS_BAD_REGISTER_NUMBER;
	}

	return PCIBIOS_SUCCESSFUL;
}

static int sunxi_pcie_cfg_write(void __iomem *addr, int size, u32 val)
{
	if ((uintptr_t)addr & (size - 1))
		return PCIBIOS_BAD_REGISTER_NUMBER;

	if (size == 4)
		writel(val, addr);
	else if (size == 2)
		writew(val, addr);
	else if (size == 1)
		writeb(val, addr);
	else
		return PCIBIOS_BAD_REGISTER_NUMBER;

	return PCIBIOS_SUCCESSFUL;
}


static void sunxi_pcie_clk_clect(struct pcie_port *pp, char on)
{
	struct sunxi_pcie *pcie = to_sunxi_pcie(pp);
	u32 val;

	val = sunxi_pcie_readl(pcie, PCIE_PHY_CFG);
	if (on)
		val |= 0x1<<31;
	else
		val &= ~(0x1<<31);
	sunxi_pcie_writel(val, pcie, PCIE_PHY_CFG);
}

static void sunxi_pcie_ltssm_enable(struct pcie_port *pp)
{
	struct sunxi_pcie *pcie = to_sunxi_pcie(pp);
	u32 val;

	val = sunxi_pcie_readl(pcie, PCIE_LTSSM_ENABLE);
	val |= PCIE_LINK_TRAINING;
	sunxi_pcie_writel(val, pcie, PCIE_LTSSM_ENABLE);
}

static void sunxi_pcie_irqpending(struct pcie_port *pp)
{
	struct sunxi_pcie *pcie = to_sunxi_pcie(pp);
	u32 val;

	val = sunxi_pcie_readl(pcie, PCIE_INT_PENDING);
	val &= ~(0x3<<16);
	sunxi_pcie_writel(val, pcie, PCIE_INT_PENDING);
}

static void sunxi_pcie_phy_cfg(struct sunxi_pcie *pcie, int enable)
{
	u32 val;

	val = sunxi_pcie_readl(pcie, PCIE_PHY_CFG);
	if (enable)
		val |= 0x1<<0;
	else
		val &= ~(0x1<<0);
	sunxi_pcie_writel(val, pcie, PCIE_PHY_CFG);
}

static void sunxi_pcie_irqmask(struct pcie_port *pp)
{
	struct sunxi_pcie *pcie = to_sunxi_pcie(pp);
	u32 val;

	val = sunxi_pcie_readl(pcie, PCIE_INT_PENDING);
	val |= 0x3<<16;
	sunxi_pcie_writel(val, pcie, PCIE_INT_PENDING);
}

static void sunxi_pcie_ltssm_disable(struct pcie_port *pp)
{
	struct sunxi_pcie *pcie = to_sunxi_pcie(pp);
	u32 val;

	val = sunxi_pcie_readl(pcie, PCIE_LTSSM_ENABLE);
	val &= ~PCIE_LINK_TRAINING;
	sunxi_pcie_writel(val, pcie, PCIE_LTSSM_ENABLE);
}

static int sunxi_pcie_wait_for_speed_change(struct pcie_port *pp)
{
	u32 tmp;
	unsigned int retries;

	for (retries = 0; retries < 200; retries++) {
		tmp = sunxi_pcie_readl_rcl(pp, PCIE_LINK_WIDTH_SPEED_CONTROL);
		/* Test if the speed change finished. */
		if (!(tmp & PORT_LOGIC_SPEED_CHANGE))
			return 0;
		usleep_range(100, 1000);
	}

	dev_err(pp->dev, "Speed change timeout\n");
	return -EINVAL;
}

int sunxi_pcie_wait_for_link(struct pcie_port *pp)
{
	int retries;
	/* check if the link is up or not */
	for (retries = 0; retries < LINK_WAIT_MAX_RETRIES; retries++) {
		if (dw_pcie_link_up(pp)) {
			dev_info(pp->dev, "link up\n");
			return 0;
		}
		usleep_range(LINK_WAIT_USLEEP_MIN, LINK_WAIT_USLEEP_MAX);
	}

	dev_err(pp->dev, "phy link never came up\n");

	return -ETIMEDOUT;
}

static int sunxi_pcie_establish_link(struct pcie_port *pp)
{
	if (dw_pcie_link_up(pp)) {
		dev_err(pp->dev, "link is already up\n");
		return 0;
	}
	sunxi_pcie_ltssm_enable(pp);
	sunxi_pcie_wait_for_link(pp);

	return 1;
}

static int sunxi_pcie_link_up_status(struct pcie_port *pp)
{
	u32 rc;
	int ret;
	struct sunxi_pcie *pcie = to_sunxi_pcie(pp);

	rc = sunxi_pcie_readl(pcie, PCIE_LINK_STATUS);
	if ((rc&RDLH_LINK_UP) && (rc&SMLH_LINK_UP))
		ret = 1;
	else
		ret = 0;

	return ret;
}

static int sunxi_pcie_speed_chang(struct pcie_port *pp, int gen)
{
	int val;
	int ret;

	val = sunxi_pcie_readl_rcl(pp, LINK_CONTROL2_LINK_STATUS2);
	val &=  ~0xf;
	val |= gen;
	sunxi_pcie_writel_rcl(pp, val, LINK_CONTROL2_LINK_STATUS2);

	val = sunxi_pcie_readl_rcl(pp, PCIE_LINK_WIDTH_SPEED_CONTROL);
	val &= ~PORT_LOGIC_SPEED_CHANGE;
	sunxi_pcie_writel_rcl(pp, val, PCIE_LINK_WIDTH_SPEED_CONTROL);

	val = sunxi_pcie_readl_rcl(pp, PCIE_LINK_WIDTH_SPEED_CONTROL);
	val |= PORT_LOGIC_SPEED_CHANGE;
	sunxi_pcie_writel_rcl(pp, val, PCIE_LINK_WIDTH_SPEED_CONTROL);

	ret = sunxi_pcie_wait_for_speed_change(pp);
	if (!ret)
		dev_info(pp->dev, "PCI-e speed of Gen%d\n", gen);
	else
		dev_info(pp->dev, "PCI-e speed of Gen1\n");

	return 0;
}

static int sunxi_pcie_regulator_bypass(struct pcie_port *pp)
{
	struct sunxi_pcie *pcie = to_sunxi_pcie(pp);
	int ret;

	ret = clk_prepare_enable(pcie->pcie_rst);
	if (ret) {
		dev_err(pp->dev, "unable to enable pcie_rst clock\n");
		goto err_pcie_rst;
	}
	usleep_range(1000, 2000);

	if (pcie->iodvdd < 2000)
		sunxi_pcie_phy_cfg(pcie, 1);
	else
		sunxi_pcie_phy_cfg(pcie, 0);

	usleep_range(1000, 2000);
	ret = clk_prepare_enable(pcie->pcie_power);
	if (ret) {
		dev_err(pp->dev, "unable to enable pcie_power clock\n");
		goto err_pcie_power;
	}

	return 0;

err_pcie_power:
	clk_disable_unprepare(pcie->pcie_rst);
err_pcie_rst:
	return ret;
}

static int sunxi_pcie_clk_setup(struct pcie_port *pp)
{
	struct sunxi_pcie *pcie = to_sunxi_pcie(pp);
	int ret;

#ifndef CONFIG_PCIE_SUNXI_EXTERNAL_CLOCK
	ret = clk_prepare_enable(pcie->pcie_ref);
	if (ret) {
		dev_err(pp->dev, "unable to enable pcie_ref clock\n");
		goto err_pcie_ref;
	}
#endif
	ret = clk_prepare_enable(pcie->pcie_axi);
	if (ret) {
		dev_err(pp->dev, "unable to enable pcie_axi clock\n");
		goto err_pcie_axi;
	}

	ret = clk_prepare_enable(pcie->pcie_aux);
	if (ret) {
		dev_err(pp->dev, "unable to enable pcie_aux clock\n");
		goto err_pcie_aux;
	}

	ret = clk_prepare_enable(pcie->pcie_bus);
	if (ret) {
		dev_err(pp->dev, "unable to enable pcie_bus clock\n");
		goto err_pcie_bus;
	}

	return 0;

err_pcie_bus:
	clk_disable_unprepare(pcie->pcie_aux);
err_pcie_aux:
	clk_disable_unprepare(pcie->pcie_axi);
err_pcie_axi:
#ifndef CONFIG_PCIE_SUNXI_EXTERNAL_CLOCK
	clk_disable_unprepare(pcie->pcie_ref);
err_pcie_ref:
#endif
	return ret;
}

static void sunxi_pcie_clk_exit(struct pcie_port *pp)
{
	struct sunxi_pcie *pcie = to_sunxi_pcie(pp);

#ifndef CONFIG_PCIE_SUNXI_EXTERNAL_CLOCK
	clk_disable_unprepare(pcie->pcie_ref);
#endif
	clk_disable_unprepare(pcie->pcie_axi);
	clk_disable_unprepare(pcie->pcie_aux);
	clk_disable_unprepare(pcie->pcie_bus);
	clk_disable_unprepare(pcie->pcie_rst);
	clk_disable_unprepare(pcie->pcie_power);
}

static irqreturn_t sunxi_plat_pcie_msi_irq_handler(int irq, void *arg)
{
	struct pcie_port *pp = (struct pcie_port *)arg;

	return dw_handle_msi_irq(pp);
}

static irqreturn_t sunxi_plat_pcie_linkup_handler(int irq, void *arg)
{
	struct pcie_port *pp = (struct pcie_port *)arg;

	sunxi_pcie_irqpending(pp);

	return IRQ_HANDLED;
}

static int sunxi_get_gpio_info(struct device_node *node, const char *name,
			struct gpio_config *gc, struct sunxi_pcie *sunxi_pcie)
{
	int gnum;

	gnum = of_get_named_gpio_flags(node, name, 0, (enum of_gpio_flags *)gc);
	if (!gpio_is_valid(gnum)) {
		gc->gpio = 0xFFFFFFF0;
		dev_err(&sunxi_pcie->dev, "get %s from device_tree failed\n", name);
		return -EINVAL;
	}

	return 0;

}
static int sunxi_pcie_gpio_request(struct sunxi_pcie *sunxi_pcie)
{
	int i;
	struct gpio_config *gc = NULL;

	for (i = 0; i < MAX_GPIO_NUM; i++) {
		gc = &sunxi_pcie->gpio[i];
		if (gc == NULL || gc->gpio == 0xFFFFFFF0)
			continue;
		if (gpio_request(gc->gpio, NULL) < 0) {
			dev_err(&sunxi_pcie->dev, "gpio%d request failed!\n", gc->gpio);
			continue;
		}
		dev_info(&sunxi_pcie->dev, "gpio%d request success!\n", gc->gpio);
	}

	return 0;
}

static int sunxi_pcie_gpio_output(struct sunxi_pcie *sunxi_pcie,
			enum gpio_type gpio_id, int val)
{
	struct gpio_config *gc = NULL;

	gc = &sunxi_pcie->gpio[gpio_id];
	if (gc == NULL || gc->gpio == 0xFFFFFFF0)
		return -1;
	if (val) {
		if (0 != gpio_direction_output(gc->gpio, 1)) {
			dev_err(&sunxi_pcie->dev, "gpio%d set 0 err!", gc->gpio);
			return -1;
		}
	} else {
		if (0 != gpio_direction_output(gc->gpio, 0)) {
			dev_err(&sunxi_pcie->dev, "gpio%d set 0 err!", gc->gpio);
			return -1;
		}
	}

	return 0;
}
static int sunxi_pcie_gpio_reset(struct platform_device *pdev, struct sunxi_pcie *sunxi_pcie)
{
	struct device_node *np = pdev->dev.of_node;

	sunxi_get_gpio_info(np, "pcie_power", &sunxi_pcie->gpio[PCIE_POWER], sunxi_pcie);
	sunxi_get_gpio_info(np, "pcie_rest", &sunxi_pcie->gpio[PCIE_REST], sunxi_pcie);
	sunxi_get_gpio_info(np, "pcie_reg", &sunxi_pcie->gpio[PCIE_REG], sunxi_pcie);
	sunxi_pcie_gpio_request(sunxi_pcie);

	sunxi_pcie_gpio_output(sunxi_pcie, PCIE_POWER, 1);
	usleep_range(1000, 1100);
	sunxi_pcie_gpio_output(sunxi_pcie, PCIE_REST, 1);
	usleep_range(1000, 1100);
	sunxi_pcie_gpio_output(sunxi_pcie, PCIE_REST, 0);
	usleep_range(1000, 1100);
	sunxi_pcie_gpio_output(sunxi_pcie, PCIE_REST, 1);
	usleep_range(1000, 1100);
	sunxi_pcie_gpio_output(sunxi_pcie, PCIE_REG, 1);

	return 0;
}

static int get_value_string(struct device_node *np, const char *name,
			    char *string)
{
	int ret;
	const char *const_str;

	ret = of_property_read_string(np, name, &const_str);
	if (ret) {
		strcpy(string, "");
		pr_err("fetch %s from device_tree failed\n", name);
		return -EINVAL;
	}
	strcpy(string, const_str);
	pr_debug("%s = %s\n", name, string);
	return 0;
}

static int get_value_int(struct device_node *np, const char *name,
			  u32 *value)
{
	int ret;

	ret = of_property_read_u32(np, name, value);
	if (ret) {
		*value = 0;
		pr_err("fetch %s from device_tree failed\n", name);
	}
	pr_debug("%s = %x\n", name, *value);

	return 0;
}

static int sunxi_get_power_info(struct device_node *node, struct sunxi_pcie *sunxi_pcie, enum power_type type)
{
	struct power *power = NULL;

	switch (type) {
	case PCIE_VDD:
		power = &sunxi_pcie->power[PCIE_VDD];
		get_value_string(node, "pcie_vdd", power->power_str);
		get_value_int(node, "pcie_vdd_vol", &power->power_vol);
		break;
	case PCIE_VCC:
		power = &sunxi_pcie->power[PCIE_VCC];
		get_value_string(node, "pcie_vcc", power->power_str);
		get_value_int(node, "pcie_vcc_vol", &power->power_vol);
		break;
	case PCIE_VCC_SLOT:
		power = &sunxi_pcie->power[PCIE_VCC_SLOT];
		get_value_string(node, "pcie_vcc_slot", power->power_str);
		get_value_int(node, "pcie_vcc_slot_vol", &power->power_vol);
		break;
	default:
		return -1;
	}

	return 0;
}
static int sunxi_pcie_set_pmu(struct sunxi_pcie *sunxi_pcie, enum power_type pmic_ch, int on)
{
	int ret = 0;
	struct power *power = NULL;

	power = &sunxi_pcie->power[pmic_ch];
	if (on) {
		if (power[pmic_ch].pmic
		    && regulator_is_enabled(power->pmic)) {
			pr_debug("regulator_is already enabled\n");
		} else {
			if (strcmp(power->power_str, "")) {
				power->pmic = regulator_get(NULL, power->power_str);
				if (IS_ERR_OR_NULL(power->pmic)) {
					pr_err("get regulator %s error!\n",
						power->power_str);
					power->pmic = NULL;
					return -1;
				}
			} else {
				power->pmic = NULL;
				return 0;
			}
		}
		ret = regulator_set_voltage(power->pmic, power->power_vol, power->power_vol);
		pr_info("set regulator %s = %d,return %x\n",
			power->power_str, power->power_vol, ret);
		ret = regulator_enable(power->pmic);

	} else {
		if (power->pmic == NULL)
			return 0;
		ret = regulator_disable(power->pmic);
		if (!regulator_is_enabled(power->pmic)) {
			pr_info("regulator_is already disabled\n");
			regulator_put(power->pmic);
			power->pmic = NULL;
		}
	}

	return ret;
}

static int sunxi_pcie_get_clk(struct platform_device *pdev, struct sunxi_pcie *sunxi_pcie)
{
	struct device_node *node = pdev->dev.of_node;

	sunxi_pcie->pcie_ref = of_clk_get(node, 0);
	if (IS_ERR_OR_NULL(sunxi_pcie->pcie_ref)) {
		dev_err(&pdev->dev, "%s:get pcie ref  clk failed\n", __func__);
		return -1;
	}

	sunxi_pcie->pcie_axi = of_clk_get(node, 1);
	if (IS_ERR_OR_NULL(sunxi_pcie->pcie_axi)) {
		dev_err(&pdev->dev, "%s:get pcie axi clk failed\n", __func__);
		return -1;
	}

	sunxi_pcie->pcie_aux = of_clk_get(node, 2);
	if (IS_ERR_OR_NULL(sunxi_pcie->pcie_aux)) {
		dev_err(&pdev->dev, "%s:get pcie aux clk failed\n", __func__);
		return -1;
	}

	sunxi_pcie->pcie_bus = of_clk_get(node, 3);
	if (IS_ERR_OR_NULL(sunxi_pcie->pcie_bus)) {
		dev_err(&pdev->dev, "%s:get pcie bus clk failed\n", __func__);
		return -1;
	}
	sunxi_pcie->pcie_power = of_clk_get(node, 4);
	if (IS_ERR_OR_NULL(sunxi_pcie->pcie_power)) {
		dev_err(&pdev->dev, "%s:get pcie power clk failed\n", __func__);
		return -1;
	}
	sunxi_pcie->pcie_rst = of_clk_get(node, 5);
	if (IS_ERR_OR_NULL(sunxi_pcie->pcie_rst)) {
		dev_err(&pdev->dev, "%s:get pcie reset clk failed\n", __func__);
		return -1;
	}

	return 0;
}

#ifdef CONFIG_ARCH_SUN50IW6
spinlock_t cut_page_reg_spinlock = __SPIN_LOCK_UNLOCKED(cut_page_reg_spinlock);
struct pci_page sunxi_pcie_bus_cutpage_config(struct pci_dev *dev, int barnum, u32 bar_base, unsigned long offset)
{
	u32 cutpage;
	struct pci_page sunxi_page;
	unsigned long size;

	size = pci_resource_len(dev, barnum);
	sunxi_page.mem_base = mem_base_start;
	bar_base = bar_base & PCIE_HIGH16_MASK;
	sunxi_page.offset = offset & PCIE_LOW16_MASK;
	cutpage = bar_base + (((size - 1) & offset) & PCIE_HIGH16_MASK);
	writel(cutpage >> PCIE_HIGH_16, dbi_base + PCIE_ADDR_PAGE_CFG);

	return sunxi_page;
}
EXPORT_SYMBOL(sunxi_pcie_bus_cutpage_config);

struct pci_page sunxi_pcie_device_cutpage_config(u32 bar_base, unsigned long offset)
{
	u32 cutpage;
	struct pci_page sunxi_page;

	sunxi_page.mem_base = mem_base_start;
	bar_base = bar_base & PCIE_HIGH16_MASK;
	sunxi_page.offset = offset & PCIE_LOW16_MASK;
	cutpage = bar_base + (offset & PCIE_HIGH16_MASK);
	writel(cutpage >> PCIE_HIGH_16, dbi_base + PCIE_ADDR_PAGE_CFG);

	return sunxi_page;
}
EXPORT_SYMBOL(sunxi_pcie_device_cutpage_config);

int sunxi_pcie_cutpage_base(u32 bar_base)
{
	writel(bar_base >> PCIE_HIGH_16, dbi_base + PCIE_ADDR_PAGE_CFG);

	return 0;
}
EXPORT_SYMBOL(sunxi_pcie_cutpage_base);

unsigned long sunxi_pcie_cutpage_spin_lock(void)
{
	unsigned long flags = 0;

	spin_lock_irqsave(&cut_page_reg_spinlock, flags);

	return flags;
}
EXPORT_SYMBOL(sunxi_pcie_cutpage_spin_lock);

void sunxi_pcie_cutpage_spin_unlock(unsigned long flags)
{
	spin_unlock_irqrestore(&cut_page_reg_spinlock, flags);
}
EXPORT_SYMBOL(sunxi_pcie_cutpage_spin_unlock);
#endif

static void sunxi_pcie_prog_outbound_atu(struct pcie_port *pp, int index,
				int type, u64 cpu_addr, u64 pci_addr, u32 size)
{
	dw_pcie_writel_rc(pp, PCIE_ATU_VIEWPORT,
				  PCIE_ATU_REGION_OUTBOUND | index);
	dw_pcie_writel_rc(pp, PCIE_ATU_LOWER_BASE,
				  lower_32_bits(cpu_addr));
	dw_pcie_writel_rc(pp, PCIE_ATU_UPPER_BASE,
				  upper_32_bits(cpu_addr));
	dw_pcie_writel_rc(pp, PCIE_ATU_LIMIT,
				lower_32_bits(cpu_addr + size - 1));
	dw_pcie_writel_rc(pp, PCIE_ATU_LOWER_TARGET,
				lower_32_bits(pci_addr));
	dw_pcie_writel_rc(pp, PCIE_ATU_UPPER_TARGET,
				upper_32_bits(pci_addr));
	dw_pcie_writel_rc(pp, PCIE_ATU_CR1, type);
	dw_pcie_writel_rc(pp, PCIE_ATU_CR2, PCIE_ATU_ENABLE);
}

static int sunxi_pcie_rd_other_conf(struct pcie_port *pp, struct pci_bus *bus,
			u32 devfn, int where, int size, u32 *val)
{
	int ret, type;
	u32 busdev, cfg_size;
	u64 cpu_addr;
	void __iomem *va_cfg_base;
	int pcie_page;
	unsigned long flags = 0;

	busdev = PCIE_ATU_BUS(bus->number) | PCIE_ATU_DEV(PCI_SLOT(devfn)) |
		 PCIE_ATU_FUNC(PCI_FUNC(devfn));

	if (bus->parent->number == pp->root_bus_nr) {
		type = PCIE_ATU_TYPE_CFG0;
		cpu_addr = pp->cfg0_base;
		cfg_size = pp->cfg0_size;
		va_cfg_base = pp->va_cfg0_base;
	} else {
		type = PCIE_ATU_TYPE_CFG1;
		cpu_addr = pp->cfg1_base;
		cfg_size = pp->cfg1_size;
		va_cfg_base = pp->va_cfg1_base;
	}

	flags = sunxi_pcie_cutpage_spin_lock();

	//printk("sunxi_pcie_rd_other_conf: %x %x %x %x\r\n", busdev, pp->cfg0_base, pp->cfg0_size, va_cfg_base);

	sunxi_pcie_writel_rcl(pp, pp->cfg0_base >> PCIE_HIGH_16, PCIE_ADDR_PAGE_CFG);
	sunxi_pcie_prog_outbound_atu(pp, PCIE_ATU_REGION_INDEX1,
				  type, cpu_addr,
				  busdev, cfg_size);
	ret = sunxi_pcie_cfg_read(va_cfg_base + where, size, val);
/*
	for (i = 0; i < PCIE_BAR_NUM; i++) {
		pcie_page = readl(pp->va_cfg0_base + PCI_BASE_ADDRESS_0 + i * PCIE_BAR_REG);
		mem_base = readl(pp->dbi_base + PCI_MEMORY_BASE);
		if (((pcie_page >> MEM_BASE_LEN) & MEM_BASE_MASK)
				== (mem_base & MEM_BASE_MASK))
			break;
	}
	*/
	
	//printk("sunxi_pcie_rd_other_conf pcie_page: %x\r\n", pcie_page);
    //Move PCIe memory window to MMIO. Hardcoded and must match device tree setting
	pcie_page = 0x05500000;
	sunxi_pcie_writel_rcl(pp, pcie_page >> PCIE_HIGH_16, PCIE_ADDR_PAGE_CFG);

	sunxi_pcie_prog_outbound_atu(pp, PCIE_ATU_REGION_INDEX0,
					  PCIE_ATU_TYPE_MEM, pp->mem_base,
					  pp->mem_bus_addr, pp->mem_size);

	sunxi_pcie_cutpage_spin_unlock(flags);

	return ret;
}

static int sunxi_pcie_wr_other_conf(struct pcie_port *pp, struct pci_bus *bus,
			unsigned int devfn, int where, int size, u32 val)
{
	int ret, type;
	u32 busdev, cfg_size;
	u64 cpu_addr;
	void __iomem *va_cfg_base;
	int pcie_page;
	unsigned long flags = 0;

	//int mem_base, i;

	busdev = PCIE_ATU_BUS(bus->number) | PCIE_ATU_DEV(PCI_SLOT(devfn)) |
		 PCIE_ATU_FUNC(PCI_FUNC(devfn));

	if (bus->parent->number == pp->root_bus_nr) {
		type = PCIE_ATU_TYPE_CFG0;
		cpu_addr = pp->cfg0_base;
		cfg_size = pp->cfg0_size;
		va_cfg_base = pp->va_cfg0_base;
	} else {
		type = PCIE_ATU_TYPE_CFG1;
		cpu_addr = pp->cfg1_base;
		cfg_size = pp->cfg1_size;
		va_cfg_base = pp->va_cfg1_base;
	}

	flags = sunxi_pcie_cutpage_spin_lock();

	sunxi_pcie_writel_rcl(pp, pp->cfg0_base >> PCIE_HIGH_16, PCIE_ADDR_PAGE_CFG);
	sunxi_pcie_prog_outbound_atu(pp, PCIE_ATU_REGION_INDEX1,
				  type, cpu_addr,
				  busdev, cfg_size);
	//printk("sunxi_pcie_wr_other_conf: %x %x %x %x\r\n", pp->cfg0_base, where, size, val);
	ret = sunxi_pcie_cfg_write(va_cfg_base + where, size, val);

/*	for (i = 0; i < PCIE_BAR_NUM; i++) {
		pcie_page = readl(pp->va_cfg0_base + PCI_BASE_ADDRESS_0 + i * PCIE_BAR_REG);
		mem_base = readl(pp->dbi_base + PCI_MEMORY_BASE);
		if (((pcie_page >> MEM_BASE_LEN) & MEM_BASE_MASK)
				== (mem_base & MEM_BASE_MASK))
			break;
	}
	*/

	//printk("sunxi_pcie_wr_other_conf pcie_page: %x\r\n", pcie_page);
	pcie_page = 0x05500000;
	sunxi_pcie_writel_rcl(pp, pcie_page >> PCIE_HIGH_16, PCIE_ADDR_PAGE_CFG);

	sunxi_pcie_prog_outbound_atu(pp, PCIE_ATU_REGION_INDEX0,
					  PCIE_ATU_TYPE_MEM, pp->mem_base,
					  pp->mem_bus_addr, pp->mem_size);

	sunxi_pcie_cutpage_spin_unlock(flags);

	return ret;
}

static void sunxi_plat_pcie_host_init(struct pcie_port *pp)
{
	struct sunxi_pcie *pcie = to_sunxi_pcie(pp);

	sunxi_pcie_ltssm_disable(pp);
	sunxi_pcie_clk_clect(pp, PAD_CLK);

	dw_pcie_setup_rc(pp);

	sunxi_pcie_establish_link(pp);
	if (IS_ENABLED(CONFIG_PCI_MSI))
		dw_pcie_msi_init(pp);
	sunxi_pcie_speed_chang(pp, pcie->speed_gen);
}

static struct pcie_host_ops sunxi_plat_pcie_host_ops = {
	.link_up = sunxi_pcie_link_up_status,
	.host_init = sunxi_plat_pcie_host_init,
	.rd_other_conf = sunxi_pcie_rd_other_conf,
	.wr_other_conf = sunxi_pcie_wr_other_conf,
};

static int sunxi_plat_add_pcie_port(struct pcie_port *pp,
				 struct platform_device *pdev)
{
	int ret;

	if (IS_ENABLED(CONFIG_PCI_MSI)) {
		pp->msi_irq = platform_get_irq(pdev, 0);
		if (pp->msi_irq < 0)
			return pp->msi_irq;

		ret = devm_request_irq(&pdev->dev, pp->msi_irq,
					sunxi_plat_pcie_msi_irq_handler,
					IRQF_SHARED, "sunxi-plat-pcie-msi", pp);
		if (ret) {
			dev_err(&pdev->dev, "failed to request MSI IRQ\n");
			return ret;
		}
	}

	pp->root_bus_nr = -1;
	pp->ops = &sunxi_plat_pcie_host_ops;

	ret = dw_pcie_host_init(pp);
	if (ret) {
		dev_err(&pdev->dev, "failed to initialize host\n");
		return ret;
	}
	dbi_base = pp->dbi_base;
	mem_base_start = pp->va_cfg0_base;

	return 0;
}

static int sunxi_plat_pcie_probe(struct platform_device *pdev)
{
	struct sunxi_pcie *sunxi_pcie;
	struct pcie_port *pp;
	struct resource *res;  /* Resource from DT */
	int ret, i;

	sunxi_pcie = devm_kzalloc(&pdev->dev, sizeof(*sunxi_pcie),
					GFP_KERNEL);
	if (!sunxi_pcie)
		return -ENOMEM;

	pp = &sunxi_pcie->pp;
	pp->dev = &pdev->dev;
	sunxi_pcie->dev = pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	sunxi_pcie->dbi_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(sunxi_pcie->dbi_base))
		return PTR_ERR(sunxi_pcie->dbi_base);

	pp->dbi_base = sunxi_pcie->dbi_base;
	sunxi_pcie->app_base = sunxi_pcie->dbi_base + USER_DEFINED_REGISTER_LIST;
	sunxi_pcie_get_clk(pdev, sunxi_pcie);

	ret = of_property_read_u32(pdev->dev.of_node, "pcie_speed_gen", &sunxi_pcie->speed_gen);
	if (ret) {
		dev_info(&pdev->dev, "get speed Gen failed\n");
		sunxi_pcie->speed_gen = 0x1;
	}
	ret = of_property_read_u32(pdev->dev.of_node, "pcie_iodvdd", &sunxi_pcie->iodvdd);
	if (ret) {
		dev_info(&pdev->dev, "get iodvdd failed\n");
		sunxi_pcie->iodvdd = 1800;
	}
	for (i = 0; i < MAX_POWER_NUM; i++)
		sunxi_get_power_info(pdev->dev.of_node, sunxi_pcie, i);

	sunxi_pcie_set_pmu(sunxi_pcie, PCIE_VDD, 1);
	sunxi_pcie_set_pmu(sunxi_pcie, PCIE_VCC, 1);
	sunxi_pcie_set_pmu(sunxi_pcie, PCIE_VCC_SLOT, 1);
	if (sunxi_pcie_clk_setup(pp))
		return -1;

	if (sunxi_pcie_regulator_bypass(pp))
		return -1;

	sunxi_pcie_irqmask(pp);
	sunxi_pcie->link_irq = platform_get_irq(pdev, 1);
	ret = devm_request_irq(&pdev->dev, sunxi_pcie->link_irq,
				 sunxi_plat_pcie_linkup_handler,
					IRQF_SHARED, "sunxi-pcie-linkup", pp);
	if (ret) {
		dev_err(&pdev->dev, "failed to request linkup IRQ\n");
		return ret;
	}
	sunxi_pcie_gpio_reset(pdev, sunxi_pcie);
	ret = sunxi_plat_add_pcie_port(pp, pdev);
	if (ret < 0)
		return ret;

	platform_set_drvdata(pdev, sunxi_pcie);

	return 0;
}

static int sunxi_plat_pcie_remove(struct platform_device *pdev)
{
	struct sunxi_pcie *sunxi_pcie = platform_get_drvdata(pdev);
	struct pcie_port *pp = &sunxi_pcie->pp;

	sunxi_pcie_clk_exit(pp);
	sunxi_pcie_set_pmu(sunxi_pcie, PCIE_VDD, 0);
	sunxi_pcie_set_pmu(sunxi_pcie, PCIE_VCC, 0);
	sunxi_pcie_set_pmu(sunxi_pcie, PCIE_VCC_SLOT, 0);

	return 0;
}

#ifdef CONFIG_PM
static int sunxi_pcie_hw_init(struct sunxi_pcie *sunxi_pcie)
{
	struct pcie_port *pp = &sunxi_pcie->pp;
	int val, i;

	if (sunxi_pcie_clk_setup(pp))
		return -1;

	if (sunxi_pcie_regulator_bypass(pp))
		return -1;

	usleep_range(100, 1000);
	sunxi_pcie_ltssm_disable(pp);
	sunxi_pcie_clk_clect(pp, PAD_CLK);

	dw_pcie_setup_rc(pp);
	sunxi_pcie_ltssm_enable(pp);
	if (IS_ENABLED(CONFIG_PCI_MSI)) {
		u64 msi_target;

		msi_target = virt_to_phys((void *)pp->msi_data);

		/* program the msi_data */
		sunxi_pcie_writel_rcl(pp, (u32)(msi_target & 0xffffffff), PCIE_MSI_ADDR_LO);
		sunxi_pcie_writel_rcl(pp, (u32)(msi_target >> 32 & 0xffffffff), PCIE_MSI_ADDR_HI);
	}

	val = sunxi_pcie_readl_rcl(pp, LINK_CONTROL2_LINK_STATUS2);
	val &=  ~0xf;
	val |= sunxi_pcie->speed_gen;
	sunxi_pcie_writel_rcl(pp, val, LINK_CONTROL2_LINK_STATUS2);

	val = sunxi_pcie_readl_rcl(pp, PCIE_LINK_WIDTH_SPEED_CONTROL);
	val &= ~PORT_LOGIC_SPEED_CHANGE;
	sunxi_pcie_writel_rcl(pp, val, PCIE_LINK_WIDTH_SPEED_CONTROL);

	val = sunxi_pcie_readl_rcl(pp, PCIE_LINK_WIDTH_SPEED_CONTROL);
	val |= PORT_LOGIC_SPEED_CHANGE;
	sunxi_pcie_writel_rcl(pp, val, PCIE_LINK_WIDTH_SPEED_CONTROL);

	for (i = 0; i < 16; i++)
		sunxi_pcie_writel_rcl(pp, sunxi_pcie->buffer[i], i * 4);

	sunxi_pcie_writel_rcl(pp, sunxi_pcie->msi_enable, PCIE_MSI_INTR0_ENABLE);

	return 0;
}

static int sunxi_pcie_hw_exit(struct sunxi_pcie *sunxi_pcie)
{
	struct pcie_port *pp = &sunxi_pcie->pp;
	int i;

	for (i = 0; i < 16; i++)
		sunxi_pcie->buffer[i] = sunxi_pcie_readl_rcl(pp, i * 4);

	sunxi_pcie->msi_enable = sunxi_pcie_readl_rcl(pp, PCIE_MSI_INTR0_ENABLE);
	sunxi_pcie_clk_exit(pp);

	return 0;
}

static int sunxi_pcie_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct sunxi_pcie *sunxi_pcie = platform_get_drvdata(pdev);

	sunxi_pcie_hw_exit(sunxi_pcie);
	sunxi_pcie_set_pmu(sunxi_pcie, PCIE_VDD, 0);
	sunxi_pcie_set_pmu(sunxi_pcie, PCIE_VCC, 0);
	sunxi_pcie_set_pmu(sunxi_pcie, PCIE_VCC_SLOT, 0);

	return 0;
}

static int sunxi_pcie_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct sunxi_pcie *sunxi_pcie = platform_get_drvdata(pdev);

	sunxi_pcie_set_pmu(sunxi_pcie, PCIE_VDD, 1);
	sunxi_pcie_set_pmu(sunxi_pcie, PCIE_VCC, 1);
	sunxi_pcie_set_pmu(sunxi_pcie, PCIE_VCC_SLOT, 1);

	sunxi_pcie_hw_init(sunxi_pcie);

	return 0;
}
static const struct dev_pm_ops sunxi_pcie_pm_ops = {
	.suspend_noirq = sunxi_pcie_suspend,
	.resume_noirq = sunxi_pcie_resume,
};
#define SUNXI_PCIE_PM_OPS (&sunxi_pcie_pm_ops)
#else
#define SUNXI_PCIE_PM_OPS NULL
#endif /* CONFIG_PM */

static const struct of_device_id sunxi_plat_pcie_of_match[] = {
	{ .compatible = "allwinner,sun50i-pcie", },
	{},
};
MODULE_DEVICE_TABLE(of, sunxi_plat_pcie_of_match);

static struct platform_driver sunxi_plat_pcie_driver = {
	.driver = {
		.name	= "sunxi-pcie",
		.owner	= THIS_MODULE,
		.of_match_table = sunxi_plat_pcie_of_match,
		.pm = SUNXI_PCIE_PM_OPS,
	},
	.remove = sunxi_plat_pcie_remove,
	.probe	= sunxi_plat_pcie_probe,
};

static int __init sunxi_pcie_init(void)
{
	return platform_driver_register(&sunxi_plat_pcie_driver);
}

static void __exit sunxi_pcie_exit(void)
{
	platform_driver_unregister(&sunxi_plat_pcie_driver);
}
fs_initcall(sunxi_pcie_init);
module_exit(sunxi_pcie_exit);

MODULE_AUTHOR("wangjx <wangjx@allwinnertech.com>");
MODULE_DESCRIPTION("Allwinner PCIe host controller glue platform driver");
MODULE_LICENSE("GPL v2");
