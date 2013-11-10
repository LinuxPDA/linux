/*
 * linux/drivers/leds/leds-locomo.c
 *
 * Copyright (C) 2005 John Lenz <lenz@cs.wisc.edu>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/mfd/locomo.h>

struct locomo_led {
	struct led_classdev led;
	void __iomem *reg;
};

static void locomoled_brightness_set(struct led_classdev *led_cdev,
				enum led_brightness value)
{
	struct locomo_led *led = container_of(led_cdev, struct locomo_led, led);
	unsigned long flags;

	local_irq_save(flags);
	if (value)
		writew(LOCOMO_LPT_TOFH, led->reg);
	else
		writew(LOCOMO_LPT_TOFL, led->reg);
	local_irq_restore(flags);
}

static int locomo_led_register(
		struct locomo_led *led,
		struct device *dev,
		const char *name,
		const char *trigger,
		void __iomem *reg)
{
	led->led.name = name;
	led->led.default_trigger = trigger;
	led->led.brightness_set = locomoled_brightness_set;
	led->reg = reg;

	return led_classdev_register(dev, &led->led);
}

static int locomoled_probe(struct platform_device *pdev)
{
	int ret;
	struct resource *mem;
	void __iomem *regs;
	struct locomo_led *leds;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem)
		return -EINVAL;

	regs = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(regs))
		return PTR_ERR(regs);

	leds = devm_kzalloc(&pdev->dev, 2 * sizeof(*leds), GFP_KERNEL);
	if (!leds)
		return -ENOMEM;

	platform_set_drvdata(pdev, leds);

	ret = locomo_led_register(leds,
			&pdev->dev,
			"locomo:amber:charge",
			"main-battery-charging",
			regs + LOCOMO_LPT0);
	if (ret < 0)
		return ret;

	ret = locomo_led_register(leds + 1,
			&pdev->dev,
			"locomo:green:mail",
			"nand-disk",
			regs + LOCOMO_LPT1);
	if (ret < 0)
		led_classdev_unregister(&leds[0].led);

	return ret;
}

static int locomoled_remove(struct platform_device *pdev)
{
	struct locomo_led *leds = platform_get_drvdata(pdev);
	led_classdev_unregister(&leds[0].led);
	led_classdev_unregister(&leds[1].led);
	return 0;
}

static struct platform_driver locomoled_driver = {
	.driver = {
		.name = "locomo-led"
	},
	.probe	= locomoled_probe,
	.remove	= locomoled_remove,
};

module_platform_driver(locomoled_driver);

MODULE_AUTHOR("John Lenz <lenz@cs.wisc.edu>");
MODULE_DESCRIPTION("Locomo LED driver");
MODULE_LICENSE("GPL");
