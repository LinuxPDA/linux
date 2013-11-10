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
static bool locomolcd_is_on;
static int current_intensity;
static struct backlight_device *locomolcd_bl_device;
static void __iomem *locomolcd_bl_regs;
static void __iomem *locomolcd_fl_regs;

static int comadj = 128; // 118 for poodle;

#define LOCOMO_GPIO_BASE 50
static int gpio_lcd_vsha_on	 = LOCOMO_GPIO_BASE + 4;
static int gpio_lcd_vshd_on	 = LOCOMO_GPIO_BASE + 5;
static int gpio_lcd_vee_on 	 = LOCOMO_GPIO_BASE + 6;
static int gpio_lcd_mod		 = LOCOMO_GPIO_BASE + 7;
static int gpio_fl_vr		 = LOCOMO_GPIO_BASE + 9;

static struct gpio locomo_gpios[] = {
	{ 0, GPIOF_OUT_INIT_LOW, "FL VR" },
	{ 0, GPIOF_OUT_INIT_LOW, "LCD VSHA on" },
	{ 0, GPIOF_OUT_INIT_LOW, "LCD VSHD on" },
	{ 0, GPIOF_OUT_INIT_LOW, "LCD Vee on" },
	{ 0, GPIOF_OUT_INIT_LOW, "LCD MOD" },
};

static void locomolcd_on(void)
{
	gpio_set_value(gpio_lcd_vsha_on, 1);
	mdelay(2);

	gpio_set_value(gpio_lcd_vshd_on, 1);
	mdelay(2);

	locomo_m62332_senddata(locomolcd_dev->dev.parent, comadj, 0);
	mdelay(5);

	gpio_set_value(gpio_lcd_vee_on, 1);
	mdelay(10);

	/* TFTCRST | CPSOUT=0 | CPSEN */
	writew(0x01, locomolcd_bl_regs + LOCOMO_TC);

	/* Set CPSD */
	writew(6, locomolcd_bl_regs + LOCOMO_CPSD);

	/* TFTCRST | CPSOUT=0 | CPSEN */
	writew((0x04 | 0x01), locomolcd_bl_regs + LOCOMO_TC);
	mdelay(10);

	gpio_set_value(gpio_lcd_mod, 1);
}

static void locomolcd_off(void)
{
	/* TFTCRST=1 | CPSOUT=1 | CPSEN = 0 */
	writew(0x06, locomolcd_bl_regs + LOCOMO_TC);
	mdelay(1);

	gpio_set_value(gpio_lcd_vsha_on, 0);
	mdelay(110);

	gpio_set_value(gpio_lcd_vee_on, 0);
	mdelay(700);

	/* TFTCRST=0 | CPSOUT=0 | CPSEN = 0 */
	writew(0, locomolcd_bl_regs + LOCOMO_TC);
	gpio_set_value(gpio_lcd_mod, 0);
	gpio_set_value(gpio_lcd_vshd_on, 0);
}

void locomolcd_power(int on)
{
	unsigned long flags;

	local_irq_save(flags);

	locomolcd_is_on = on;

	if (!locomolcd_dev) {
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

static const struct {
	u16 duty, bpwf;
	bool vr;
} locomolcd_pwm[] = {
	{ 0, 161, false },
	{ 117, 161, false },
	{ 163, 148, false },
	{ 194, 161, false },
	{ 194, 161, true },
};

static void locomo_frontlight_set(int duty, int bpwf)
{
	writew(bpwf, locomolcd_fl_regs + LOCOMO_ALS);
	udelay(100);
	writew(duty, locomolcd_fl_regs + LOCOMO_ALD);
	writew(bpwf | LOCOMO_ALC_EN, locomolcd_fl_regs + LOCOMO_ALS);
}

static int locomolcd_set_intensity(struct backlight_device *bd)
{
	int intensity = bd->props.brightness;

	if (bd->props.power != FB_BLANK_UNBLANK)
		intensity = 0;
	if (bd->props.fb_blank != FB_BLANK_UNBLANK)
		intensity = 0;
	if (bd->props.state & BL_CORE_SUSPENDED)
		intensity = 0;

	gpio_set_value(gpio_fl_vr, locomolcd_pwm[intensity].vr);
	locomo_frontlight_set(locomolcd_pwm[intensity].duty,
			locomolcd_pwm[intensity].bpwf);

	current_intensity = intensity;
	if (bd->props.state & BL_CORE_SUSPENDED) {
		writew(0x00, locomolcd_bl_regs + LOCOMO_TC);
		writew(0x00, locomolcd_fl_regs + LOCOMO_ALS); /* FL */
	}
	return 0;
}

static int locomolcd_get_intensity(struct backlight_device *bd)
{
	return current_intensity;
}

static const struct backlight_ops locomobl_data = {
	.options	= BL_CORE_SUSPENDRESUME,
	.get_brightness = locomolcd_get_intensity,
	.update_status  = locomolcd_set_intensity,
};

static int locomolcd_probe(struct platform_device *dev)
{
	struct backlight_properties props;
	unsigned long flags;
	struct resource *res;
	int rc;

	res = platform_get_resource(dev, IORESOURCE_MEM, 0);
	if (!res)
		return -EINVAL;
	locomolcd_bl_regs = devm_ioremap_resource(&dev->dev, res);
	if (!locomolcd_bl_regs)
		return -EINVAL;

	res = platform_get_resource(dev, IORESOURCE_MEM, 1);
	if (!res)
		return -EINVAL;
	locomolcd_fl_regs = devm_ioremap_resource(&dev->dev, res);
	if (!locomolcd_fl_regs)
		return -EINVAL;

	locomo_gpios[0].gpio = gpio_fl_vr;
	locomo_gpios[1].gpio = gpio_lcd_vsha_on;
	locomo_gpios[2].gpio = gpio_lcd_vshd_on;
	locomo_gpios[3].gpio = gpio_lcd_vee_on;
	locomo_gpios[4].gpio = gpio_lcd_mod;

	rc = gpio_request_array(locomo_gpios, ARRAY_SIZE(locomo_gpios));
	if (rc)
		return rc;

	local_irq_save(flags);
	locomolcd_dev = dev;

	/* Frontlight */
	writew(0, locomolcd_fl_regs + LOCOMO_ALS);
	writew(0, locomolcd_fl_regs + LOCOMO_ALD);

	if (locomolcd_is_on)
		locomolcd_on();

	local_irq_restore(flags);

	memset(&props, 0, sizeof(struct backlight_properties));
	props.type = BACKLIGHT_RAW;
	props.max_brightness = ARRAY_SIZE(locomolcd_pwm) - 1;
	props.brightness = props.max_brightness / 2;
	locomolcd_bl_device = backlight_device_register("locomo-bl",
							&dev->dev, NULL,
							&locomobl_data, &props);

	if (IS_ERR(locomolcd_bl_device)) {
		gpio_free_array(locomo_gpios, ARRAY_SIZE(locomo_gpios));
		return PTR_ERR(locomolcd_bl_device);
	}

	/* Set up frontlight so that screen is readable */
	backlight_update_status(locomolcd_bl_device);

	return 0;
}

static int locomolcd_remove(struct platform_device *dev)
{
	unsigned long flags;

	locomolcd_bl_device->props.brightness = 0;
	locomolcd_bl_device->props.power = 0;
	locomolcd_set_intensity(locomolcd_bl_device);

	backlight_device_unregister(locomolcd_bl_device);

	local_irq_save(flags);
	locomolcd_off();
	locomolcd_is_on = false;
	locomolcd_dev = NULL;
	local_irq_restore(flags);

	gpio_free_array(locomo_gpios, ARRAY_SIZE(locomo_gpios));

	return 0;
}

static struct platform_driver locomolcd_driver = {
	.driver = {
		.name	= "locomo-backlight",
		.owner	= THIS_MODULE,
	},
	.probe	= locomolcd_probe,
	.remove	= locomolcd_remove,
};

module_platform_driver(locomolcd_driver);

MODULE_AUTHOR("John Lenz <lenz@cs.wisc.edu>, Pavel Machek <pavel@ucw.cz>");
MODULE_DESCRIPTION("Collie LCD driver");
MODULE_LICENSE("GPL");
