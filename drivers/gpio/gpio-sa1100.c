/*
 * drivers/gpio/gpio-sa1100.c
 *
 * Generic SA-1100 GPIO handling
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/platform_device.h>
#include <linux/syscore_ops.h>

#include <mach/irqs.h>
#include <mach/generic.h>

/*
 * SA1100 GPIO edge detection for IRQs:
 * IRQs are generated on Falling-Edge, Rising-Edge, or both.
 * Use this instead of directly setting GRER/GFER.
 */

#define SA1100_NGPIO 28

struct sa1100_gpio_chip {
	struct gpio_chip gc;
	void __iomem *regbase;
	struct irq_domain *domain;
	int gpio_rising;
	int gpio_falling;
	int gpio_mask;
	int irq_base;
};

#define to_sgc(chip)	container_of(chip, struct sa1100_gpio_chip, gc)

#define GPLR_OFFSET	0x00  /* GPIO Pin Level Reg.             */
#define GPDR_OFFSET	0x04  /* GPIO Pin Direction Reg.         */
#define GPSR_OFFSET	0x08  /* GPIO Pin output Set Reg.        */
#define GPCR_OFFSET	0x0C  /* GPIO Pin output Clear Reg.      */
#define GRER_OFFSET	0x10  /* GPIO Rising-Edge detect Reg.    */
#define GFER_OFFSET	0x14  /* GPIO Falling-Edge detect Reg.   */
#define GEDR_OFFSET	0x18  /* GPIO Edge Detect status Reg.    */
#define GAFR_OFFSET	0x1C  /* GPIO Alternate Function Reg.    */

static int sa1100_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	struct sa1100_gpio_chip *sgc = to_sgc(chip);
	return readl_relaxed(sgc->regbase + GPLR_OFFSET) & BIT(offset);
}

static void sa1100_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
	struct sa1100_gpio_chip *sgc = to_sgc(chip);
	writel_relaxed(BIT(offset), sgc->regbase +
				(value ? GPSR_OFFSET : GPCR_OFFSET));
}

static int sa1100_direction_input(struct gpio_chip *chip, unsigned offset)
{
	struct sa1100_gpio_chip *sgc = to_sgc(chip);
	unsigned long flags;
	uint32_t tmp;

	local_irq_save(flags);

	tmp = readl_relaxed(sgc->regbase + GPDR_OFFSET);
	tmp &= ~BIT(offset);
	writel_relaxed(tmp, sgc->regbase + GPDR_OFFSET);

	local_irq_restore(flags);
	return 0;
}

static int sa1100_direction_output(struct gpio_chip *chip, unsigned offset, int value)
{
	struct sa1100_gpio_chip *sgc = to_sgc(chip);
	unsigned long flags;
	uint32_t tmp;

	local_irq_save(flags);
	sa1100_gpio_set(chip, offset, value);

	tmp = readl_relaxed(sgc->regbase + GPDR_OFFSET);
	tmp |= BIT(offset);
	writel_relaxed(tmp, sgc->regbase + GPDR_OFFSET);

	local_irq_restore(flags);
	return 0;
}

static int sa1100_gpio_to_irq(struct gpio_chip *chip, unsigned offset)
{
	struct sa1100_gpio_chip *sgc = to_sgc(chip);
	return irq_find_mapping(sgc->domain, offset);
}

static int sa1100_gpio_type(struct irq_data *d, unsigned int type)
{
	struct sa1100_gpio_chip *sgc = irq_data_get_irq_chip_data(d);
	unsigned int mask = BIT(d->hwirq);

	if (type == IRQ_TYPE_PROBE) {
		if ((sgc->gpio_rising | sgc->gpio_falling) & mask)
			return 0;
		type = IRQ_TYPE_EDGE_RISING | IRQ_TYPE_EDGE_FALLING;
	}

	if (type & IRQ_TYPE_EDGE_RISING)
		sgc->gpio_rising |= mask;
	else
		sgc->gpio_rising &= ~mask;

	if (type & IRQ_TYPE_EDGE_FALLING)
		sgc->gpio_falling |= mask;
	else
		sgc->gpio_falling &= ~mask;

	writel_relaxed(sgc->gpio_rising & sgc->gpio_mask,
			sgc->regbase + GRER_OFFSET);
	writel_relaxed(sgc->gpio_falling & sgc->gpio_mask,
			sgc->regbase + GFER_OFFSET);

	return 0;
}

/*
 * GPIO IRQs must be acknowledged.
 */
static void sa1100_gpio_ack(struct irq_data *d)
{
	struct sa1100_gpio_chip *sgc = irq_data_get_irq_chip_data(d);

	writel_relaxed(BIT(d->hwirq), sgc->regbase + GEDR_OFFSET);
}

static int sa1100_gpio_wake(struct irq_data *d, unsigned int on)
{
	return sa11x0_gpio_set_wake(d->hwirq, on);
}

static void sa1100_gpio_mask(struct irq_data *d)
{
	struct sa1100_gpio_chip *sgc = irq_data_get_irq_chip_data(d);

	sgc->gpio_mask &= ~BIT(d->hwirq);

	writel_relaxed(sgc->gpio_rising & sgc->gpio_mask,
			sgc->regbase + GRER_OFFSET);
	writel_relaxed(sgc->gpio_falling & sgc->gpio_mask,
			sgc->regbase + GFER_OFFSET);
}

static void sa1100_gpio_unmask(struct irq_data *d)
{
	struct sa1100_gpio_chip *sgc = irq_data_get_irq_chip_data(d);

	sgc->gpio_mask |= BIT(d->hwirq);

	writel_relaxed(sgc->gpio_rising & sgc->gpio_mask,
			sgc->regbase + GRER_OFFSET);
	writel_relaxed(sgc->gpio_falling & sgc->gpio_mask,
			sgc->regbase + GFER_OFFSET);
}

static void
sa1100_gpio_handler(unsigned int irq, struct irq_desc *desc)
{
	struct sa1100_gpio_chip *sgc = irq_get_handler_data(irq);
	unsigned int hwirq = 0;
	unsigned int mask = readl_relaxed(sgc->regbase + GEDR_OFFSET);
	/*
	 * clear down all currently active IRQ sources.
	 * We will be processing them all.
	 */
	writel_relaxed(mask, sgc->regbase + GEDR_OFFSET);

	while (mask) {
		if (mask & 1)
			generic_handle_irq(irq_find_mapping(sgc->domain,
						hwirq));
		mask >>= 1;
		hwirq++;
	}
}

static struct irq_chip sa1100_gpio_irq_chip = {
	.name		= "GPIO",
	.irq_ack	= sa1100_gpio_ack,
	.irq_mask	= sa1100_gpio_mask,
	.irq_unmask	= sa1100_gpio_unmask,
	.irq_set_type	= sa1100_gpio_type,
	.irq_set_wake	= sa1100_gpio_wake,
};

static int sa1100_gpio_irqdomain_map(struct irq_domain *d, unsigned int irq,
		irq_hw_number_t hwirq)
{
	struct sa1100_gpio_chip *sgc = d->host_data;

	irq_set_chip_data(irq, sgc);
	irq_set_chip_and_handler(irq, &sa1100_gpio_irq_chip, handle_edge_irq);
	set_irq_flags(irq, IRQF_VALID | IRQF_PROBE);

	return 0;
}

static struct irq_domain_ops sa1100_gpio_irqdomain_ops = {
	.map = sa1100_gpio_irqdomain_map,
	.xlate = irq_domain_xlate_onetwocell,
};

static struct sa1100_gpio_chip *chip;

static int sa1100_gpio_probe(struct platform_device *pdev)
{
	struct resource *res;
	int ret;
	unsigned int i, irq;
	struct sa1100_gpio_chip *sgc;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -EINVAL;

	sgc = devm_kzalloc(&pdev->dev, sizeof(*sgc), GFP_KERNEL);
	if  (!sgc)
		return -ENOMEM;

	sgc->regbase = devm_ioremap_resource(&pdev->dev, res);
	if (!sgc->regbase)
		return -EINVAL;

	sgc->gc.label = "gpio";
	sgc->gc.direction_input = sa1100_direction_input;
	sgc->gc.direction_output = sa1100_direction_output;
	sgc->gc.set = sa1100_gpio_set;
	sgc->gc.get = sa1100_gpio_get;
	sgc->gc.to_irq = sa1100_gpio_to_irq;

	sgc->gc.base = 0;
	sgc->gc.ngpio = SA1100_NGPIO;

	sgc->irq_base = IRQ_GPIO0;

	/* clear all GPIO edge detects */
	writel_relaxed(0, sgc->regbase + GFER_OFFSET);
	writel_relaxed(0, sgc->regbase + GRER_OFFSET);
	writel_relaxed(-1, sgc->regbase + GEDR_OFFSET);

	/* Initialize GPIO chips */
	ret = gpiochip_add(&sgc->gc);
	if (ret)
		return ret;

	sgc->domain = irq_domain_add_legacy(NULL, SA1100_NGPIO, IRQ_GPIO0, 0,
			&sa1100_gpio_irqdomain_ops, sgc);
	/*
	 * Install handler for GPIO 11-27 edge detect interrupts
	 */
	for (i = 0; i < 11; i++) {
		irq = platform_get_irq(pdev, i);
		if (irq < 0)
			goto err_irq;
		irq_set_handler_data(irq, sgc);
		irq_set_chained_handler(irq, sa1100_gpio_handler);
	}
	irq = platform_get_irq(pdev, 11);
	if (irq < 0)
		goto err_irq;
	irq_set_handler_data(irq, sgc);
	irq_set_chained_handler(irq, sa1100_gpio_handler);

	chip = sgc;

	return 0;

err_irq:
	dev_err(&pdev->dev, "Error retrieving irq resource %d (%d)\n", i, irq);
	i = gpiochip_remove(&sgc->gc);
	if (i)
		dev_err(&pdev->dev, "Error removing gpio chip (%d)!\n", i);

	return ret;
}

static struct platform_driver sa1100_gpio_driver = {
	.probe		= sa1100_gpio_probe,
	.driver		= {
		.name	= "sa1100-gpio",
	},
};

static int __init sa1100_gpio_init(void)
{
	return platform_driver_register(&sa1100_gpio_driver);
}
postcore_initcall(sa1100_gpio_init);

#ifdef CONFIG_PM
static unsigned long	saved_gplr;
static unsigned long	saved_gpdr;
static unsigned long	saved_grer;
static unsigned long	saved_gfer;

static int sa1100_gpio_suspend(void)
{
	struct sa1100_gpio_chip *sgc = chip;
	saved_gplr = readl_relaxed(sgc->regbase + GPLR_OFFSET);
	saved_gpdr = readl_relaxed(sgc->regbase + GPDR_OFFSET);
	saved_grer = readl_relaxed(sgc->regbase + GRER_OFFSET);
	saved_gfer = readl_relaxed(sgc->regbase + GFER_OFFSET);

	/* Clear GPIO transition detect bits */
	writel_relaxed(0xffffffff, sgc->regbase + GEDR_OFFSET);

	/* FIXME: Original code also reprogramed GRER/GFER here,
	 * I don't see the purpose though.
	GRER = PWER & sgc->gpio_rising;
	GFER = PWER & sgc->gpio_falling;
	 */

	return 0;
}

static void sa1100_gpio_resume(void)
{
	struct sa1100_gpio_chip *sgc = chip;
	/* restore level with set/clear */
	writel_relaxed(saved_gplr, sgc->regbase + GPSR_OFFSET);
	writel_relaxed(~saved_gplr, sgc->regbase + GPCR_OFFSET);

	writel_relaxed(saved_grer, sgc->regbase + GRER_OFFSET);
	writel_relaxed(saved_gfer, sgc->regbase + GFER_OFFSET);
	writel_relaxed(saved_gpdr, sgc->regbase + GPDR_OFFSET);
}
#else
#define sa1100_gpio_suspend	NULL
#define sa1100_gpio_resume	NULL
#endif

static struct syscore_ops sa1100_gpio_syscore_ops = {
	.suspend	= sa1100_gpio_suspend,
	.resume		= sa1100_gpio_resume,
};

static int __init sa1100_gpio_sysinit(void)
{
	if (chip)
		register_syscore_ops(&sa1100_gpio_syscore_ops);
	return 0;
}
postcore_initcall(sa1100_gpio_sysinit);
