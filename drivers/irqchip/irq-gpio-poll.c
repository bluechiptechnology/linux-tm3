/*
 * Conexant Digicolor SoCs IRQ chip driver
 *
 * Author: Baruch Siach <baruch@tkos.co.il>
 *
 * Copyright (C) 2014 Paradox Innovation Ltd.
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/kthread.h>

#include <asm/exception.h>

struct gpio_poll_data 
{	
	struct irq_domain *irq_domain;
	struct task_struct	* gpiopolltask;
	int gpiopolltaskrun;
	spinlock_t		lock;
	int				irq;
	int				irqlevel;
	int				gpio;
	struct device_node *node;
};

static int gpio_poll_thread(void *pArg)
{
	struct gpio_poll_data * d = (struct gpio_poll_data *)pArg;
	unsigned long flags;

	while (!kthread_should_stop()) 
	{

		//msleep(1);  /* 1s */

		usleep_range(500, 500);

		if (gpio_get_value(d->gpio))
		{
			//printk("gpio poll task: irq\r\n");
			local_irq_save(flags);
			generic_handle_irq(d->irq);
			local_irq_restore(flags);
		}
	}

	return 0;
}

/*
* We don't need to ACK IRQs on the SA1100 unless they're GPIOs
* this is for internal IRQs i.e. from IRQ LCD to RTCAlrm.
*/
static void gpio_poll_ack_irq(struct irq_data *data)
{
	//printk("gpio_poll_ack_irq: %d\r\n", data->hwirq);
}

static void gpio_poll_mask_irq(struct irq_data *data)
{
	struct gpio_poll_data *d = irq_data_get_irq_chip_data(data);

	//printk("gpio_poll_mask_irq: %d\r\n", data->hwirq);

	/*
	if (d->gpiopolltask != NULL)
	{
		kthread_stop(d->gpiopolltask);
		d->gpiopolltask = NULL;
	}
	*/
}

static void gpio_poll_unmask_irq(struct irq_data *data)
{
	struct gpio_poll_data *d = irq_data_get_irq_chip_data(data);
	unsigned long flags;

	//printk("gpio_poll_unmask_irq: %d\r\n", data->hwirq);

	local_irq_save(flags);

	if (d->gpiopolltask == NULL)
	{
		d->gpiopolltask = kthread_create(gpio_poll_thread, d, "gpiopolltask");
		if (IS_ERR(d->gpiopolltask))
		{
			d->gpiopolltask = NULL;
			printk("Failed to create gpiopolltask\r\n");
			return;
		}

		wake_up_process(d->gpiopolltask);
	}

	local_irq_restore(flags);
}

static int gpio_poll_set_wake(struct irq_data *data, unsigned int on)
{
	printk("gpio_poll_set_wake\r\n");
	return 0;
}

static int gpio_poll_set_type(struct irq_data *data, unsigned int type)
{
	struct gpio_poll_data *d = irq_data_get_irq_chip_data(data);
	unsigned int pos = data->hwirq;
	int status = 0;

	printk("gpio_poll_set_type: %x %d %d\r\n", d, pos, type);

	if (type == IRQF_TRIGGER_NONE)
	{
		d->irqlevel = -1;
	}
	else if(type == IRQ_TYPE_LEVEL_HIGH)
	{
		d->irqlevel = 1;
	}
	else if (type == IRQ_TYPE_LEVEL_LOW)
	{
		d->irqlevel = 0;
	}
	else
	{
		return -EINVAL;
	}

	printk("gpio_poll_set_type-\r\n");

	return status;
}


static struct irq_chip  gpio_poll_chip = {
	.name = "GPIO_POLL",
	.irq_ack = gpio_poll_ack_irq,
	.irq_mask = gpio_poll_mask_irq,
	.irq_unmask = gpio_poll_unmask_irq,
	.irq_set_wake = gpio_poll_set_wake,
	.irq_set_type = gpio_poll_set_type,
	.flags = IRQ_TYPE_LEVEL_MASK,
};


/*
static void __exception_irq_entry digicolor_handle_irq(struct pt_regs *regs)
{
	struct irq_domain_chip_generic *dgc = digicolor_irq_domain->gc;
	struct irq_chip_generic *gc = dgc->gc[0];
	u32 status, hwirq;

	do {
		status = irq_reg_readl(gc, IC_INT0STATUS_LO);
		if (status) {
			hwirq = ffs(status) - 1;
		} else {
			status = irq_reg_readl(gc, IC_INT0STATUS_XLO);
			if (status)
				hwirq = ffs(status) - 1 + 32;
			else
				return;
		}

		handle_domain_irq(digicolor_irq_domain, hwirq, regs);
	} while (1);
}
*/

/**
* gpiochip_irq_map() - maps an IRQ into a GPIO irqchip
* @d: the irqdomain used by this irqchip
* @irq: the global irq number used by this GPIO irqchip irq
* @hwirq: the local IRQ/GPIO line offset on this gpiochip
*
* This function will set up the mapping for a certain IRQ line on a
* gpiochip by assigning the gpiochip as chip data, and using the irqchip
* stored inside the gpiochip.
*/
static int gpio_poll_irq_map(struct irq_domain *d, unsigned int irq, irq_hw_number_t hwirq)
{
	struct gpio_poll_data * chip = d->host_data;

	printk("gpio_poll_irq_map: %d\r\n", hwirq);

	irq_set_chip_data(irq, chip);

	chip->gpio = of_get_named_gpio(chip->node, "gpio", 0);
	if ((chip->gpio < 0) && (chip->gpio != -ENOENT))
	{
		printk("gpio_poll_irq_map failed to get gpio\r\n");
		return ERR_PTR(chip->gpio);
	}

	printk("gpio_poll_irq_map using GPIO: %d\r\n", chip->gpio);

	gpio_request(chip->gpio, NULL);
	gpio_direction_input(chip->gpio);

	irq_set_chip_and_handler(irq, &gpio_poll_chip, handle_simple_irq);

	//irq_set_chip_data(irq, chip);

	//irq_set_chip_and_handler(irq, chip->irqchip, chip->irq_handler);
	//irq_set_noprobe(irq);

	irq_set_irq_type(irq, 0);

	chip->irq = irq;

	return 0;
}

static void  gpio_poll_irq_unmap(struct irq_domain *d, unsigned int irq)
{
	struct gpio_poll_data *chip = d->host_data;

	printk("gpio_poll_irq_unmap: %d\r\n", irq);

	irq_set_chip_and_handler(irq, NULL, NULL);
	irq_set_chip_data(irq, NULL);
}

static const struct irq_domain_ops gpiochip_domain_ops = {
	.map = gpio_poll_irq_map,
	.unmap = gpio_poll_irq_unmap,
	/* Virtually all GPIO irqchips are twocell:ed */
	.xlate = irq_domain_xlate_twocell,
};


static int __init gpio_poll_of_init(struct device_node *node, struct device_node *parent)
{
	struct gpio_poll_data * gpiopolldata;

	printk("gpio_poll_of_init\r\n");

	gpiopolldata = kzalloc(sizeof(struct gpio_poll_data), GFP_KERNEL);
	if (!gpiopolldata)
	{
		return -ENOMEM;
	}

	gpiopolldata->node = node;

	gpiopolldata->irq_domain = irq_domain_add_simple(node, 1, 0, &gpiochip_domain_ops, gpiopolldata);
	if (!gpiopolldata->irq_domain)
	{
		printk("gpio_poll_of_init failed\r\n");
		return -EINVAL;
	}

	printk("gpio_poll_of_init success\r\n");

	return 0;
}

IRQCHIP_DECLARE(gpio_poll_ic, "gpiopoll-ic", gpio_poll_of_init);
