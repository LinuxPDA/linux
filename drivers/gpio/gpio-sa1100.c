/*
 * linux/arch/arm/mach-sa1100/gpio.c
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
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/syscore_ops.h>
#include <mach/irqs.h>
#include <mach/generic.h>

/*
 * SA1100 GPIO edge detection for IRQs:
 * IRQs are generated on Falling-Edge, Rising-Edge, or both.
 * Use this instead of directly setting GRER/GFER.
 */
static int GPIO_IRQ_rising_edge;
static int GPIO_IRQ_falling_edge;
static int GPIO_IRQ_mask = (1 << 11) - 1;
static int irq_base;

static void __iomem *regbase;

#define GPIO_MAX 27

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
	return readl_relaxed(regbase + GPLR_OFFSET) & (1 << offset);
}

static void sa1100_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
	writel_relaxed(1 << offset, regbase +
				(value ? GPSR_OFFSET : GPCR_OFFSET));
}

static int sa1100_direction_input(struct gpio_chip *chip, unsigned offset)
{
	unsigned long flags;
	uint32_t tmp;

	local_irq_save(flags);

	tmp = readl_relaxed(regbase + GPDR_OFFSET);
	tmp &= ~(1 << offset);
	writel_relaxed(tmp, regbase + GPDR_OFFSET);

	local_irq_restore(flags);
	return 0;
}

static int sa1100_direction_output(struct gpio_chip *chip, unsigned offset, int value)
{
	unsigned long flags;
	uint32_t tmp;

	local_irq_save(flags);
	sa1100_gpio_set(chip, offset, value);

	tmp = readl_relaxed(regbase + GPDR_OFFSET);
	tmp |= 1 << offset;
	writel_relaxed(tmp, regbase + GPDR_OFFSET);

	local_irq_restore(flags);
	return 0;
}

static int sa1100_gpio_to_irq(struct gpio_chip *chip, unsigned offset)
{
	return irq_base + offset;
}

static int sa1100_irq_to_gpio(int irq)
{
	return irq - irq_base;
}

static int sa1100_gpio_type(struct irq_data *d, unsigned int type)
{
	unsigned int mask;

	mask = 1 << sa1100_irq_to_gpio(d->irq);

	if (type == IRQ_TYPE_PROBE) {
		if ((GPIO_IRQ_rising_edge | GPIO_IRQ_falling_edge) & mask)
			return 0;
		type = IRQ_TYPE_EDGE_RISING | IRQ_TYPE_EDGE_FALLING;
	}

	if (type & IRQ_TYPE_EDGE_RISING) {
		GPIO_IRQ_rising_edge |= mask;
	} else
		GPIO_IRQ_rising_edge &= ~mask;
	if (type & IRQ_TYPE_EDGE_FALLING) {
		GPIO_IRQ_falling_edge |= mask;
	} else
		GPIO_IRQ_falling_edge &= ~mask;

	writel_relaxed(GPIO_IRQ_rising_edge & GPIO_IRQ_mask, regbase + GRER_OFFSET);
	writel_relaxed(GPIO_IRQ_falling_edge & GPIO_IRQ_mask, regbase + GFER_OFFSET);

	return 0;
}

/*
 * GPIO IRQs must be acknowledged.
 */
static void sa1100_gpio_ack(struct irq_data *d)
{
	writel_relaxed(1 << sa1100_irq_to_gpio(d->irq), regbase + GEDR_OFFSET);
}

static int sa1100_gpio_wake(struct irq_data *d, unsigned int on)
{
	return sa11x0_gpio_set_wake(sa1100_irq_to_gpio(d->irq), on);
}

static void sa1100_gpio_mask(struct irq_data *d)
{
	unsigned int mask = 1 << sa1100_irq_to_gpio(d->irq);

	GPIO_IRQ_mask &= ~mask;

	writel_relaxed(GPIO_IRQ_rising_edge & GPIO_IRQ_mask, regbase + GRER_OFFSET);
	writel_relaxed(GPIO_IRQ_falling_edge & GPIO_IRQ_mask, regbase + GFER_OFFSET);
}

static void sa1100_gpio_unmask(struct irq_data *d)
{
	unsigned int mask = 1 << sa1100_irq_to_gpio(d->irq);

	GPIO_IRQ_mask |= mask;

	writel_relaxed(GPIO_IRQ_rising_edge & GPIO_IRQ_mask, regbase + GRER_OFFSET);
	writel_relaxed(GPIO_IRQ_falling_edge & GPIO_IRQ_mask, regbase + GFER_OFFSET);
}

/*
 * IRQ11 (GPIO11 through 27) handler.  We enter here with the
 * irq_controller_lock held, and IRQs disabled.  Decode the IRQ
 * and call the handler.
 */
static void
sa1100_high_gpio_handler(unsigned int irq, struct irq_desc *desc)
{
	unsigned int mask;

	mask = readl_relaxed(regbase + GEDR_OFFSET) & 0xfffff800;
	do {
		/*
		 * clear down all currently active IRQ sources.
		 * We will be processing them all.
		 */
		writel_relaxed(mask, regbase + GEDR_OFFSET);

		irq = sa1100_gpio_to_irq(NULL, 11);
		mask >>= 11;
		do {
			if (mask & 1)
				generic_handle_irq(irq);
			mask >>= 1;
			irq++;
		} while (mask);

		mask = readl_relaxed(regbase + GEDR_OFFSET) & 0xfffff800;
	} while (mask);
}

/*
 * This depends on a fact that low-gpio irqs
 * are sequential and start from 0
 */
static void
sa1100_low_gpio_handler(unsigned int irq, struct irq_desc *desc)
{
	unsigned int mask = 1 << irq;

	mask &= readl_relaxed(regbase + GEDR_OFFSET);
	if (mask) {
		writel_relaxed(mask, regbase + GEDR_OFFSET);
		generic_handle_irq(sa1100_gpio_to_irq(NULL, irq));
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

static struct gpio_chip sa1100_gpio_chip = {
	.label			= "gpio",
	.direction_input	= sa1100_direction_input,
	.direction_output	= sa1100_direction_output,
	.set			= sa1100_gpio_set,
	.get			= sa1100_gpio_get,
	.to_irq			= sa1100_gpio_to_irq,
	.base			= 0,
	.ngpio			= GPIO_MAX + 1,
};

static int sa1100_gpio_probe(struct platform_device *pdev)
{
	struct resource *res;
	int ret;
	int i, irq;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -EINVAL;
	regbase = ioremap(res->start, resource_size(res));
	if (!regbase)
		return -EINVAL;

	/* clear all GPIO edge detects */
	writel_relaxed(0, regbase + GFER_OFFSET);
	writel_relaxed(0, regbase + GRER_OFFSET);
	writel_relaxed(-1, regbase + GEDR_OFFSET);

	/* Initialize GPIO chips */
	ret = gpiochip_add(&sa1100_gpio_chip);
	if (ret)
		goto err_add;

	irq_base = IRQ_GPIO0;
	for (irq = 0; irq <= GPIO_MAX; irq ++) {
		irq_set_chip_and_handler(irq + irq_base,
				&sa1100_gpio_irq_chip,
				handle_edge_irq);
		set_irq_flags(irq + irq_base, IRQF_VALID | IRQF_PROBE);
	}
	/*
	 * Install handler for GPIO 11-27 edge detect interrupts
	 */
	for (i = 0; i < 11; i++) {
		irq = platform_get_irq(pdev, i);
		if (irq < 0)
			goto err_irq;
		irq_set_chained_handler(irq, sa1100_low_gpio_handler);
	}
	irq = platform_get_irq(pdev, 11);
	if (irq < 0)
		goto err_irq;
	irq_set_chained_handler(irq, sa1100_high_gpio_handler);

	return 0;

err_irq:
	dev_err(&pdev->dev, "Error retrieving irq resource %d (%d)\n", i, irq);
	i = gpiochip_remove(&sa1100_gpio_chip);
	if (i)
		dev_err(&pdev->dev, "Error removing gpio chip (%d)!\n", i);
err_add:
	iounmap(regbase);

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
	saved_gplr = readl_relaxed(regbase + GPLR_OFFSET);
	saved_gpdr = readl_relaxed(regbase + GPDR_OFFSET);
	saved_grer = readl_relaxed(regbase + GRER_OFFSET);
	saved_gfer = readl_relaxed(regbase + GFER_OFFSET);

	/* Clear GPIO transition detect bits */
	writel_relaxed(0xffffffff, regbase + GEDR_OFFSET);

	/* FIXME: Original code also reprogramed GRER/GFER here,
	 * I don't see the purpose though.
	GRER = PWER & GPIO_IRQ_rising_edge;
	GFER = PWER & GPIO_IRQ_falling_edge;
	 */

	return 0;
}

static void sa1100_gpio_resume(void)
{
	/* restore level with set/clear */
	writel_relaxed(saved_gplr, regbase + GPSR_OFFSET);
	writel_relaxed(~saved_gplr, regbase + GPCR_OFFSET);

	writel_relaxed(saved_grer, regbase + GRER_OFFSET);
	writel_relaxed(saved_gfer, regbase + GFER_OFFSET);
	writel_relaxed(saved_gpdr, regbase + GPDR_OFFSET);
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
	register_syscore_ops(&sa1100_gpio_syscore_ops);
	return 0;
}
postcore_initcall(sa1100_gpio_sysinit);
