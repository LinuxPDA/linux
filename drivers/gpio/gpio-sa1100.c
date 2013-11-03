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
#include <linux/platform_device.h>

#include <mach/irqs.h>

#define SA1100_NGPIO 28

struct sa1100_gpio_chip {
	struct gpio_chip gc;
	void __iomem *regbase;
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
	return offset < 11 ? (IRQ_GPIO0 + offset) : (IRQ_GPIO11 - 11 + offset);
}

static int sa1100_gpio_probe(struct platform_device *pdev)
{
	struct resource *res;
	int ret;
	struct sa1100_gpio_chip *sgc;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -EINVAL;

	sgc = kzalloc(sizeof(*sgc), GFP_KERNEL);
	if  (!sgc)
		return -ENOMEM;

	sgc->regbase = ioremap(res->start, resource_size(res));
	if (!sgc->regbase) {
		kfree(sgc);
		return -EINVAL;
	}

	sgc->gc.label = "gpio";
	sgc->gc.direction_input = sa1100_direction_input;
	sgc->gc.direction_output = sa1100_direction_output;
	sgc->gc.set = sa1100_gpio_set;
	sgc->gc.get = sa1100_gpio_get;
	sgc->gc.to_irq = sa1100_gpio_to_irq;

	sgc->gc.base = 0;
	sgc->gc.ngpio = SA1100_NGPIO;

	/* Initialize GPIO chips */
	ret = gpiochip_add(&sgc->gc);
	if (ret) {
		iounmap(sgc->regbase);
		kfree(sgc);
	}

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
