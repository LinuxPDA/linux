/*
 * Backlight control code for Sharp Zaurus SL-5500
 *
 * Copyright 2005 John Lenz <lenz@cs.wisc.edu>
 * Maintainer: Pavel Machek <pavel@ucw.cz> (unless John wants to :-)
 * GPL v2
 *
 * This driver assumes single CPU. That's okay, because collie is
 * slightly old hardware, and no one is going to retrofit second CPU to
 * old PDA.
 */

/* LCD power functions */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/mfd/locomo.h>

static struct platform_device *locomolcd_dev;
static struct locomo_lcd_platform_data lcd_data;
static bool locomolcd_is_on;
static bool locomolcd_is_suspended;
static void __iomem *locomolcd_bl_regs;

static struct gpio locomo_gpios[] = {
	{ 0, GPIOF_OUT_INIT_LOW, "LCD VSHA on" },
	{ 0, GPIOF_OUT_INIT_LOW, "LCD VSHD on" },
	{ 0, GPIOF_OUT_INIT_LOW, "LCD Vee on" },
	{ 0, GPIOF_OUT_INIT_LOW, "LCD MOD" },
};

static void locomolcd_on(void)
{
	gpio_set_value(lcd_data.gpio_lcd_vsha_on, 1);
	mdelay(2);

	gpio_set_value(lcd_data.gpio_lcd_vshd_on, 1);
	mdelay(2);

	locomo_m62332_senddata(locomolcd_dev->dev.parent, lcd_data.comadj, 0);
	mdelay(5);

	gpio_set_value(lcd_data.gpio_lcd_vee_on, 1);
	mdelay(10);

	/* TFTCRST | CPSOUT=0 | CPSEN */
	writew(0x01, locomolcd_bl_regs + LOCOMO_TC);

	/* Set CPSD */
	writew(6, locomolcd_bl_regs + LOCOMO_CPSD);

	/* TFTCRST | CPSOUT=0 | CPSEN */
	writew((0x04 | 0x01), locomolcd_bl_regs + LOCOMO_TC);
	mdelay(10);

	gpio_set_value(lcd_data.gpio_lcd_mod, 1);
}

static void locomolcd_off(void)
{
	/* TFTCRST=1 | CPSOUT=1 | CPSEN = 0 */
	writew(0x06, locomolcd_bl_regs + LOCOMO_TC);
	mdelay(1);

	gpio_set_value(lcd_data.gpio_lcd_vsha_on, 0);
	mdelay(110);

	gpio_set_value(lcd_data.gpio_lcd_vee_on, 0);
	mdelay(700);

	locomo_m62332_senddata(locomolcd_dev->dev.parent, 0, 0);
	mdelay(5);

	/* TFTCRST=0 | CPSOUT=0 | CPSEN = 0 */
	writew(0, locomolcd_bl_regs + LOCOMO_TC);
	gpio_set_value(lcd_data.gpio_lcd_mod, 0);
	gpio_set_value(lcd_data.gpio_lcd_vshd_on, 0);
}

void locomolcd_power(int on)
{
	unsigned long flags;

	printk("LCD power %d\n", on);
	local_irq_save(flags);

	locomolcd_is_on = on;

	if (!locomolcd_dev || locomolcd_is_suspended) {
		local_irq_restore(flags);
		return;
	}

	if (on)
		locomolcd_on();
	else
		locomolcd_off();

	local_irq_restore(flags);
}
EXPORT_SYMBOL(locomolcd_power);

#ifdef CONFIG_PM_SLEEP
static int locomolcd_suspend(struct device *dev)
{
	locomolcd_is_suspended = true;
	locomolcd_off();

	return 0;
}

static int locomolcd_resume(struct device *dev)
{
	locomolcd_is_suspended = false;

	if (locomolcd_is_on)
		locomolcd_on();

	return 0;
}

static SIMPLE_DEV_PM_OPS(locomolcd_pm, locomolcd_suspend, locomolcd_resume);
#define LOCOMOLCD_PM	(&locomolcd_pm)
#else
#define LOCOMOLCD_PM	NULL
#endif

static int locomolcd_probe(struct platform_device *dev)
{
	unsigned long flags;
	struct resource *res;
	struct locomo_lcd_platform_data *pdata;
	int rc;

	res = platform_get_resource(dev, IORESOURCE_MEM, 0);
	if (!res)
		return -EINVAL;
	locomolcd_bl_regs = devm_ioremap_resource(&dev->dev, res);
	if (!locomolcd_bl_regs)
		return -EINVAL;

	pdata = dev_get_platdata(&dev->dev);
	if (!pdata)
		return -EINVAL;

	lcd_data = *pdata;

	locomo_gpios[0].gpio = lcd_data.gpio_lcd_vsha_on;
	locomo_gpios[1].gpio = lcd_data.gpio_lcd_vshd_on;
	locomo_gpios[2].gpio = lcd_data.gpio_lcd_vee_on;
	locomo_gpios[3].gpio = lcd_data.gpio_lcd_mod;
	dev_info(&dev->dev, "GPIOs: %d %d %d %d\n",
			locomo_gpios[0].gpio,
			locomo_gpios[1].gpio,
			locomo_gpios[2].gpio,
			locomo_gpios[3].gpio);

	rc = gpio_request_array(locomo_gpios, ARRAY_SIZE(locomo_gpios));
	if (rc)
		return rc;

	local_irq_save(flags);
	locomolcd_dev = dev;

	if (locomolcd_is_on)
		locomolcd_on();

	local_irq_restore(flags);

	return 0;
}

static int locomolcd_remove(struct platform_device *dev)
{
	unsigned long flags;

	local_irq_save(flags);

	locomolcd_off();
	locomolcd_dev = NULL;

	local_irq_restore(flags);

	gpio_free_array(locomo_gpios, ARRAY_SIZE(locomo_gpios));

	return 0;
}

static void locomolcd_shutdown(struct platform_device *dev)
{
	locomolcd_off();
}

static struct platform_driver locomolcd_driver = {
	.driver = {
		.name	= "locomo-lcd",
		.owner	= THIS_MODULE,
		.pm	= LOCOMOLCD_PM,
	},
	.probe	= locomolcd_probe,
	.remove	= locomolcd_remove,
	.shutdown = locomolcd_shutdown,
};

module_platform_driver(locomolcd_driver);

MODULE_AUTHOR("John Lenz <lenz@cs.wisc.edu>, Pavel Machek <pavel@ucw.cz>");
MODULE_DESCRIPTION("Collie LCD driver");
MODULE_LICENSE("GPL");
