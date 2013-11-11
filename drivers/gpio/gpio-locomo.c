/*
 * Sharp LoCoMo support for GPIO
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This file contains all generic LoCoMo support.
 *
 * All initialization functions provided here are intended to be called
 * from machine specific code with proper arguments when required.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/spinlock.h>
#include <linux/irq.h>
#include <linux/mfd/locomo.h>

#define LOCOMO_GPIO_NR_IRQS 16

struct locomo_gpio {
	void __iomem *regs;
	int irq;

	spinlock_t lock;
	struct gpio_chip gpio;
	int irq_base;

	u16 rising_edge;
	u16 falling_edge;

	u16 save_gpo;
	u16 save_gpe;
};

static int locomo_gpio_get(struct gpio_chip *chip,
		unsigned offset)
{
	struct locomo_gpio *lg = container_of(chip, struct locomo_gpio, gpio);

	return readw(lg->regs + LOCOMO_GPL) & (1 << offset);
}

static void __locomo_gpio_set(struct gpio_chip *chip,
		unsigned offset, int value)
{
	struct locomo_gpio *lg = container_of(chip, struct locomo_gpio, gpio);
	u16  r;

	r = readw(lg->regs + LOCOMO_GPO);
	if (value)
		r |= 1 << offset;
	else
		r &= ~(1 << offset);
	writew(r, lg->regs + LOCOMO_GPO);
}

static void locomo_gpio_set(struct gpio_chip *chip,
		unsigned offset, int value)
{
	struct locomo_gpio *lg = container_of(chip, struct locomo_gpio, gpio);
	unsigned long flags;

	spin_lock_irqsave(&lg->lock, flags);

	__locomo_gpio_set(chip, offset, value);

	spin_unlock_irqrestore(&lg->lock, flags);
}

static int locomo_gpio_direction_input(struct gpio_chip *chip,
			unsigned offset)
{
	struct locomo_gpio *lg = container_of(chip, struct locomo_gpio, gpio);
	unsigned long flags;
	u16 r;

	spin_lock_irqsave(&lg->lock, flags);

	r = readw(lg->regs + LOCOMO_GPD);
	r |= (1 << offset);
	writew(r, lg->regs + LOCOMO_GPD);

	r = readw(lg->regs + LOCOMO_GPE);
	r |= (1 << offset);
	writew(r, lg->regs + LOCOMO_GPE);

	spin_unlock_irqrestore(&lg->lock, flags);

	return 0;
}

static int locomo_gpio_direction_output(struct gpio_chip *chip,
			unsigned offset, int value)
{
	struct locomo_gpio *lg = container_of(chip, struct locomo_gpio, gpio);
	unsigned long flags;
	u16 r;

	spin_lock_irqsave(&lg->lock, flags);

	__locomo_gpio_set(chip, offset, value);

	r = readw(lg->regs + LOCOMO_GPD);
	r &= ~(1 << offset);
	writew(r, lg->regs + LOCOMO_GPD);

	r = readw(lg->regs + LOCOMO_GPE);
	r &= ~(1 << offset);
	writew(r, lg->regs + LOCOMO_GPE);

	spin_unlock_irqrestore(&lg->lock, flags);

	return 0;
}

static int locomo_gpio_to_irq(struct gpio_chip *chip, unsigned offset)
{
	struct locomo_gpio *lg = container_of(chip, struct locomo_gpio, gpio);

	return lg->irq_base + offset;
}

static void
locomo_gpio_handler(unsigned int irq, struct irq_desc *desc)
{
	u16 req;
	struct locomo_gpio *lg = irq_get_handler_data(irq);
	int i = lg->irq_base;

	req = readw(lg->regs + LOCOMO_GIR) &
	      readw(lg->regs + LOCOMO_GPD);

	while (req) {
		if (req & 1)
			generic_handle_irq(i);
		req >>= 1;
		i ++;
	}
}

static void locomo_gpio_ack_irq(struct irq_data *d)
{
	struct locomo_gpio *lg = irq_data_get_irq_chip_data(d);
	unsigned long flags;
	u16 r;

	spin_lock_irqsave(&lg->lock, flags);

	r = readw(lg->regs + LOCOMO_GWE);
	r |= (0x0001 << (d->irq - lg->irq_base));
	writew(r, lg->regs + LOCOMO_GWE);

	r = readw(lg->regs + LOCOMO_GIS);
	r &= ~(0x0001 << (d->irq - lg->irq_base));
	writew(r, lg->regs + LOCOMO_GIS);

	r = readw(lg->regs + LOCOMO_GWE);
	r &= ~(0x0001 << (d->irq - lg->irq_base));
	writew(r, lg->regs + LOCOMO_GWE);

	spin_unlock_irqrestore(&lg->lock, flags);
}

static void locomo_gpio_mask_irq(struct irq_data *d)
{
	struct locomo_gpio *lg = irq_data_get_irq_chip_data(d);
	unsigned long flags;
	u16 r;

	spin_lock_irqsave(&lg->lock, flags);

	r = readw(lg->regs + LOCOMO_GIE);
	r &= ~(0x0001 << (d->irq - lg->irq_base));
	writew(r, lg->regs + LOCOMO_GIE);

	spin_unlock_irqrestore(&lg->lock, flags);
}

static void locomo_gpio_unmask_irq(struct irq_data *d)
{
	struct locomo_gpio *lg = irq_data_get_irq_chip_data(d);
	unsigned long flags;
	u16 r;

	spin_lock_irqsave(&lg->lock, flags);

	r = readw(lg->regs + LOCOMO_GIE);
	r |= (0x0001 << (d->irq - lg->irq_base));
	writew(r, lg->regs + LOCOMO_GIE);

	spin_unlock_irqrestore(&lg->lock, flags);
}

static int locomo_gpio_type(struct irq_data *d, unsigned int type)
{
	unsigned int mask;
	struct locomo_gpio *lg = irq_data_get_irq_chip_data(d);
	unsigned long flags;

	mask = 1 << (d->irq - lg->irq_base);

	if (type == IRQ_TYPE_PROBE) {
		if ((lg->rising_edge | lg->falling_edge) & mask)
			return 0;
		type = IRQ_TYPE_EDGE_RISING | IRQ_TYPE_EDGE_FALLING;
	}

	if (type & IRQ_TYPE_EDGE_RISING)
		lg->rising_edge |= mask;
	else
		lg->rising_edge &= ~mask;
	if (type & IRQ_TYPE_EDGE_FALLING)
		lg->falling_edge |= mask;
	else
		lg->falling_edge &= ~mask;

	spin_lock_irqsave(&lg->lock, flags);

	writew(lg->rising_edge, lg->regs + LOCOMO_GRIE);
	writew(lg->falling_edge, lg->regs + LOCOMO_GFIE);

	spin_unlock_irqrestore(&lg->lock, flags);

	return 0;
}

static struct irq_chip locomo_gpio_chip = {
	.name		= "LOCOMO-gpio",
	.irq_ack	= locomo_gpio_ack_irq,
	.irq_mask	= locomo_gpio_mask_irq,
	.irq_unmask	= locomo_gpio_unmask_irq,
	.irq_set_type	= locomo_gpio_type,
};

static void locomo_gpio_setup_irq(struct locomo_gpio *lg)
{
	int irq;

	lg->irq_base = irq_alloc_descs(-1, 0, LOCOMO_GPIO_NR_IRQS, -1);

	/* Install handlers for IRQ_LOCOMO_* */
	for (irq = lg->irq_base; irq < lg->irq_base + LOCOMO_GPIO_NR_IRQS; irq++) {
		irq_set_chip_and_handler(irq, &locomo_gpio_chip, handle_edge_irq);
		irq_set_chip_data(irq, lg);
		set_irq_flags(irq, IRQF_VALID | IRQF_PROBE);
	}

	/*
	 * Install handler for IRQ_LOCOMO_HW.
	 */
	irq_set_handler_data(lg->irq, lg);
	irq_set_chained_handler(lg->irq, locomo_gpio_handler);
}

#ifdef CONFIG_PM_SLEEP
static int locomo_gpio_suspend(struct device *dev)
{
	struct locomo_gpio *lg = dev_get_drvdata(dev);
	unsigned long flags;

	spin_lock_irqsave(&lg->lock, flags);

	lg->save_gpo = readw(lg->regs + LOCOMO_GPO);
	writew(0x00, lg->regs + LOCOMO_GPO);

	lg->save_gpo = readw(lg->regs + LOCOMO_GPE);
	writew(0x00, lg->regs + LOCOMO_GPE);

	spin_unlock_irqrestore(&lg->lock, flags);
	return 0;
}

static int locomo_gpio_resume(struct device *dev)
{
	struct locomo_gpio *lg = dev_get_drvdata(dev);
	unsigned long flags;

	spin_lock_irqsave(&lg->lock, flags);

	writew(lg->save_gpo, lg->regs + LOCOMO_GPO);

	writew(lg->save_gpe, lg->regs + LOCOMO_GPE);

	spin_unlock_irqrestore(&lg->lock, flags);
	return 0;
}
static SIMPLE_DEV_PM_OPS(locomo_gpio_pm, locomo_gpio_suspend, locomo_gpio_resume);
#define LOCOMO_GPIO_PM	(&locomo_gpio_pm)
#else
#define LOCOMO_GPIO_PM	NULL
#endif

static int locomo_gpio_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct locomo_gpio *lg;
	int ret;
	struct locomo_gpio_platform_data *pdata = dev_get_platdata(&pdev->dev);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	lg = devm_kzalloc(&pdev->dev, sizeof(struct locomo_gpio),
			GFP_KERNEL);
	if (!lg)
		return -ENOMEM;

	lg->irq = platform_get_irq(pdev, 0);
	if (lg->irq < 0)
		return -ENXIO;

	lg->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(lg->regs))
		return PTR_ERR(lg->regs);

	spin_lock_init(&lg->lock);

	platform_set_drvdata(pdev, lg);

	writew(0, lg->regs + LOCOMO_GPO);
	writew(0, lg->regs + LOCOMO_GPE);
	writew(0, lg->regs + LOCOMO_GPD);
	writew(0, lg->regs + LOCOMO_GIE);

	lg->gpio.base = pdata ? pdata->gpio_base : -1;
	lg->gpio.label = "locomo-gpio";
	lg->gpio.ngpio = 16;
	lg->gpio.set = locomo_gpio_set;
	lg->gpio.get = locomo_gpio_get;
	lg->gpio.direction_input = locomo_gpio_direction_input;
	lg->gpio.direction_output = locomo_gpio_direction_output;
	lg->gpio.to_irq = locomo_gpio_to_irq;

	ret = gpiochip_add(&lg->gpio);
	if (ret)
		return ret;

	locomo_gpio_setup_irq(lg);

	return 0;
}

static int locomo_gpio_remove(struct platform_device *pdev)
{
	struct locomo_gpio *lg = platform_get_drvdata(pdev);
	int ret;

	ret = gpiochip_remove(&lg->gpio);
	if (ret) {
		dev_err(&pdev->dev, "Can't remove gpio chip: %d\n", ret);
		return ret;
	}

	irq_set_chained_handler(lg->irq, NULL);
	irq_set_handler_data(lg->irq, NULL);
	irq_free_descs(lg->irq_base, LOCOMO_GPIO_NR_IRQS);

	return 0;
}

static struct platform_driver locomo_gpio_driver = {
	.probe		= locomo_gpio_probe,
	.remove		= locomo_gpio_remove,
	.driver		= {
		.name	= "locomo-gpio",
		.owner	= THIS_MODULE,
		.pm	= LOCOMO_GPIO_PM,
	},
};
module_platform_driver(locomo_gpio_driver);

MODULE_DESCRIPTION("Sharp LoCoMo GPIO driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("John Lenz <lenz@cs.wisc.edu>");
