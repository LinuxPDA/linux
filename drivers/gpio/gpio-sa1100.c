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
#include <linux/ioport.h>
#include <linux/slab.h>

#include <mach/irqs.h>

#define SA1100_NGPIO 28

static void __iomem *regbase;

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
	return readl_relaxed(regbase + GPLR_OFFSET) & BIT(offset);
}

static void sa1100_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
	writel_relaxed(BIT(offset), regbase +
				(value ? GPSR_OFFSET : GPCR_OFFSET));
}

static int sa1100_direction_input(struct gpio_chip *chip, unsigned offset)
{
	unsigned long flags;
	uint32_t tmp;

	local_irq_save(flags);

	tmp = readl_relaxed(regbase + GPDR_OFFSET);
	tmp &= ~BIT(offset);
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
	tmp |= BIT(offset);
	writel_relaxed(tmp, regbase + GPDR_OFFSET);

	local_irq_restore(flags);
	return 0;
}

static int sa1100_gpio_to_irq(struct gpio_chip *chip, unsigned offset)
{
	return offset < 11 ? (IRQ_GPIO0 + offset) : (IRQ_GPIO11 - 11 + offset);
}

static struct gpio_chip sa1100_gpio_chip = {
	.label			= "gpio",
	.direction_input	= sa1100_direction_input,
	.direction_output	= sa1100_direction_output,
	.set			= sa1100_gpio_set,
	.get			= sa1100_gpio_get,
	.to_irq			= sa1100_gpio_to_irq,
	.base			= 0,
	.ngpio			= SA1100_NGPIO,
};

void __init sa1100_init_gpio(struct resource *res)
{
	regbase = ioremap(res->start, resource_size(res));
	if (!regbase)
		return;

	gpiochip_add(&sa1100_gpio_chip);
}
