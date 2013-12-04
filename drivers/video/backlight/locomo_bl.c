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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/mfd/locomo.h>

struct locomo_bl {
	void __iomem *regs;
	int current_intensity;
	int gpio_fl_vr;
};

static const struct {
	u16 duty, bpwf;
	bool vr;
} locomo_bl_pwm[] = {
	{ 0, 161, false },
	{ 117, 161, false },
	{ 163, 148, false },
	{ 194, 161, false },
	{ 194, 161, true },
};

static int locomo_bl_set_intensity(struct backlight_device *bd)
{
	int intensity = bd->props.brightness;
	struct locomo_bl *bl = dev_get_drvdata(&bd->dev);

	if (bd->props.power != FB_BLANK_UNBLANK)
		intensity = 0;
	if (bd->props.fb_blank != FB_BLANK_UNBLANK)
		intensity = 0;
	if (bd->props.state & BL_CORE_SUSPENDED)
		intensity = 0;

	gpio_set_value(bl->gpio_fl_vr, locomo_bl_pwm[intensity].vr);

	writew(locomo_bl_pwm[intensity].bpwf, bl->regs + LOCOMO_ALS);
	udelay(100);
	writew(locomo_bl_pwm[intensity].duty, bl->regs + LOCOMO_ALD);
	udelay(100);
	writew(locomo_bl_pwm[intensity].bpwf | LOCOMO_ALC_EN,
			bl->regs + LOCOMO_ALS);

	bl->current_intensity = intensity;
	if (bd->props.state & BL_CORE_SUSPENDED)
		writew(0x00, bl->regs + LOCOMO_ALS);

	return 0;
}

static int locomo_bl_get_intensity(struct backlight_device *bd)
{
	struct locomo_bl *bl = dev_get_drvdata(&bd->dev);
	return bl->current_intensity;
}

static const struct backlight_ops locomo_bl_ops = {
	.options	= BL_CORE_SUSPENDRESUME,
	.get_brightness = locomo_bl_get_intensity,
	.update_status  = locomo_bl_set_intensity,
};

static int locomo_bl_probe(struct platform_device *dev)
{
	struct backlight_properties props;
	struct resource *res;
	struct locomo_bl_platform_data *pdata;
	struct locomo_bl *bl;
	struct backlight_device *locomo_bl_device;
	int rc;

	bl = devm_kmalloc(&dev->dev, sizeof(struct locomo_bl), GFP_KERNEL);
	if (!bl)
		return -ENOMEM;

	res = platform_get_resource(dev, IORESOURCE_MEM, 0);
	if (!res)
		return -EINVAL;
	bl->regs = devm_ioremap_resource(&dev->dev, res);
	if (!bl->regs)
		return -EINVAL;

	pdata = dev_get_platdata(&dev->dev);
	if (!pdata)
		return -EINVAL;

	bl->gpio_fl_vr = pdata->gpio_fl_vr;
	rc = devm_gpio_request_one(&dev->dev, bl->gpio_fl_vr, GPIOF_OUT_INIT_LOW, "FL VR");
	if (rc)
		return rc;

	writew(0, bl->regs + LOCOMO_ALS);
	writew(0, bl->regs + LOCOMO_ALD);

	memset(&props, 0, sizeof(struct backlight_properties));
	props.type = BACKLIGHT_RAW;
	props.max_brightness = ARRAY_SIZE(locomo_bl_pwm) - 1;
	props.brightness = props.max_brightness / 2;
	locomo_bl_device = backlight_device_register("locomo-bl",
						&dev->dev, bl,
						&locomo_bl_ops, &props);

	if (IS_ERR(locomo_bl_device))
		return PTR_ERR(locomo_bl_device);

	platform_set_drvdata(dev, locomo_bl_device);

	/* Set up frontlight so that screen is readable */
	backlight_update_status(locomo_bl_device);

	return 0;
}

static int locomo_bl_remove(struct platform_device *dev)
{
	struct backlight_device *locomo_bl_device = platform_get_drvdata(dev);

	locomo_bl_device->props.brightness = 0;
	locomo_bl_device->props.power = 0;
	locomo_bl_set_intensity(locomo_bl_device);

	backlight_device_unregister(locomo_bl_device);

	return 0;
}

static void locomo_bl_shutdown(struct platform_device *dev)
{
	struct backlight_device *locomo_bl_device = platform_get_drvdata(dev);

	locomo_bl_device->props.brightness = 0;
	locomo_bl_device->props.power = 0;
	locomo_bl_set_intensity(locomo_bl_device);
}

static struct platform_driver locomo_bl_driver = {
	.driver = {
		.name	= "locomo-backlight",
		.owner	= THIS_MODULE,
	},
	.probe	= locomo_bl_probe,
	.remove	= locomo_bl_remove,
	/* Turn off bl on power off/reboot */
	.shutdown = locomo_bl_shutdown,
};

module_platform_driver(locomo_bl_driver);

MODULE_AUTHOR("John Lenz <lenz@cs.wisc.edu>, Pavel Machek <pavel@ucw.cz>");
MODULE_DESCRIPTION("Collie Backlight driver");
MODULE_LICENSE("GPL");
