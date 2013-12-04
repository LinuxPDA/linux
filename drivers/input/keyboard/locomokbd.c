/*
 * LoCoMo keyboard driver for Linux-based ARM PDAs:
 * 	- SHARP Zaurus Collie (SL-5500)
 * 	- SHARP Zaurus Poodle (SL-5600)
 *
 * Copyright (c) 2005 John Lenz
 * Based on from xtkbd.c
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mfd/locomo.h>

#define LOCOMOKBD_NUMKEYS	128

#define KEY_ACTIVITY		KEY_F16
#define KEY_CONTACT		KEY_F18
#define KEY_CENTER		KEY_F15

static const unsigned char
locomokbd_keycode[LOCOMOKBD_NUMKEYS] = {
	0, KEY_ESC, KEY_ACTIVITY, 0, 0, 0, 0, 0, 0, 0,				/* 0 - 9 */
	0, 0, 0, 0, 0, 0, 0, KEY_MENU, KEY_HOME, KEY_CONTACT,			/* 10 - 19 */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,						/* 20 - 29 */
	0, 0, 0, KEY_CENTER, 0, KEY_MAIL, 0, 0, 0, 0,				/* 30 - 39 */
	0, 0, 0, 0, 0, 0, 0, 0, 0, KEY_RIGHT,					/* 40 - 49 */
	KEY_UP, KEY_LEFT, 0, 0, KEY_P, 0, KEY_O, KEY_I, KEY_Y, KEY_T,		/* 50 - 59 */
	KEY_E, KEY_W, 0, 0, 0, 0, KEY_DOWN, KEY_ENTER, 0, 0,			/* 60 - 69 */
	KEY_BACKSPACE, 0, KEY_L, KEY_U, KEY_H, KEY_R, KEY_D, KEY_Q, 0, 0,	/* 70 - 79 */
	0, 0, 0, 0, 0, 0, KEY_ENTER, KEY_RIGHTSHIFT, KEY_K, KEY_J,		/* 80 - 89 */
	KEY_G, KEY_F, KEY_X, KEY_S, 0, 0, 0, 0, 0, 0,				/* 90 - 99 */
	0, 0, KEY_DOT, 0, KEY_COMMA, KEY_N, KEY_B, KEY_C, KEY_Z, KEY_A,		/* 100 - 109 */
	KEY_LEFTSHIFT, KEY_TAB, KEY_LEFTCTRL, 0, 0, 0, 0, 0, 0, 0,		/* 110 - 119 */
	KEY_M, KEY_SPACE, KEY_V, KEY_APOSTROPHE, KEY_SLASH, 0, 0, 0		/* 120 - 128 */
};

#define KB_ROWS			16
#define KB_COLS			8
#define KB_ROWMASK(r)		(1 << (r))
#define SCANCODE(c,r)		( ((c)<<4) + (r) + 1 )

#define KB_DELAY		8
#define SCAN_INTERVAL		(HZ/10)

struct locomokbd {
	unsigned char keycode[LOCOMOKBD_NUMKEYS];
	struct input_dev *input;
	char phys[32];

	void __iomem *base;
	int irq;
	spinlock_t lock;

	struct timer_list timer;
	unsigned long suspend_jiffies;
	unsigned int count_cancel;
};

/* helper functions for reading the keyboard matrix */
static inline void locomokbd_charge_all(void __iomem *membase)
{
	writew(0x00FF, membase + LOCOMO_KSC);
}

static inline void locomokbd_activate_all(void __iomem *membase)
{
	unsigned long r;

	writew(0, membase + LOCOMO_KSC);
	r = readw(membase + LOCOMO_KIC);
	r &= 0xFEFF;
	writew(r, membase + LOCOMO_KIC);
}

static inline void locomokbd_activate_col(void __iomem *membase, int col)
{
	unsigned short nset;
	unsigned short nbset;

	nset = 0xFF & ~(1 << col);
	nbset = (nset << 8) + nset;
	writew(nbset, membase + LOCOMO_KSC);
}

static inline void locomokbd_reset_col(void __iomem *membase, int col)
{
	unsigned short nbset;

	nbset = ((0xFF & ~(1 << col)) << 8) + 0xFF;
	writew(nbset, membase + LOCOMO_KSC);
}

/*
 * The LoCoMo keyboard only generates interrupts when a key is pressed.
 * So when a key is pressed, we enable a timer.  This timer scans the
 * keyboard, and this is how we detect when the key is released.
 */

/* Scan the hardware keyboard and push any changes up through the input layer */
static void locomokbd_scankeyboard(struct locomokbd *locomokbd)
{
	unsigned int row, col, rowd;
	unsigned long flags;
	unsigned int num_pressed;
	void __iomem *membase = locomokbd->base;

	spin_lock_irqsave(&locomokbd->lock, flags);

	locomokbd_charge_all(membase);

	num_pressed = 0;
	for (col = 0; col < KB_COLS; col++) {

		locomokbd_activate_col(membase, col);
		udelay(KB_DELAY);

		rowd = ~readw(membase + LOCOMO_KIB);
		for (row = 0; row < KB_ROWS; row++) {
			unsigned int scancode, pressed, key;

			scancode = SCANCODE(col, row);
			pressed = rowd & KB_ROWMASK(row);
			key = locomokbd->keycode[scancode];

			input_report_key(locomokbd->input, key, pressed);
			if (likely(!pressed))
				continue;

			num_pressed++;

			/* The "Cancel/ESC" key is labeled "On/Off" on
			 * Collie and Poodle and should suspend the device
			 * if it was pressed for more than a second. */
			if (unlikely(key == KEY_ESC)) {
				if (!time_after(jiffies,
					locomokbd->suspend_jiffies + HZ))
					continue;
				if (locomokbd->count_cancel++
					!= (HZ/SCAN_INTERVAL + 1))
					continue;
				input_event(locomokbd->input, EV_PWR,
					KEY_SUSPEND, 1);
				locomokbd->suspend_jiffies = jiffies;
			} else
				locomokbd->count_cancel = 0;
		}
		locomokbd_reset_col(membase, col);
	}
	locomokbd_activate_all(membase);

	input_sync(locomokbd->input);

	/* if any keys are pressed, enable the timer */
	if (num_pressed)
		mod_timer(&locomokbd->timer, jiffies + SCAN_INTERVAL);
	else
		locomokbd->count_cancel = 0;

	spin_unlock_irqrestore(&locomokbd->lock, flags);
}

/*
 * LoCoMo keyboard interrupt handler.
 */
static irqreturn_t locomokbd_interrupt(int irq, void *dev_id)
{
	struct locomokbd *locomokbd = dev_id;
	u16 r;

	r = readw(locomokbd->base + LOCOMO_KIC);
	if ((r & 0x0001) == 0)
		return IRQ_HANDLED;

	writew(r & ~0x0100, locomokbd->base + LOCOMO_KIC); /* Ack */

	/** wait chattering delay **/
	udelay(100);

	locomokbd_scankeyboard(locomokbd);
	return IRQ_HANDLED;
}

/*
 * LoCoMo timer checking for released keys
 */
static void locomokbd_timer_callback(unsigned long data)
{
	struct locomokbd *locomokbd = (struct locomokbd *) data;

	locomokbd_scankeyboard(locomokbd);
}

static int locomokbd_open(struct input_dev *dev)
{
	struct locomokbd *locomokbd = input_get_drvdata(dev);
	u16 r;
	
	r = readw(locomokbd->base + LOCOMO_KIC) | 0x0010;
	writew(r, locomokbd->base + LOCOMO_KIC);
	return 0;
}

static void locomokbd_close(struct input_dev *dev)
{
	struct locomokbd *locomokbd = input_get_drvdata(dev);
	u16 r;
	
	r = readw(locomokbd->base + LOCOMO_KIC) & ~0x0010;
	writew(r, locomokbd->base + LOCOMO_KIC);
}

static int locomokbd_probe(struct platform_device *dev)
{
	struct locomokbd *locomokbd;
	struct input_dev *input_dev;
	int i, err;
	struct resource *res;

	locomokbd = devm_kzalloc(&dev->dev, sizeof(struct locomokbd), GFP_KERNEL);
	if (!locomokbd)
		return -ENOMEM;

	input_dev = input_allocate_device();
	if (!input_dev)
		return -ENOMEM;

	res = platform_get_resource(dev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	locomokbd->irq = platform_get_irq(dev, 0);
	if (locomokbd->irq < 0)
		return -ENXIO;

	platform_set_drvdata(dev, locomokbd);

	locomokbd->base = devm_ioremap_resource(&dev->dev, res);
	if (IS_ERR(locomokbd->base))
		return PTR_ERR(locomokbd->base);

	spin_lock_init(&locomokbd->lock);

	init_timer(&locomokbd->timer);
	locomokbd->timer.function = locomokbd_timer_callback;
	locomokbd->timer.data = (unsigned long) locomokbd;

	locomokbd->suspend_jiffies = jiffies;

	locomokbd->input = input_dev;
	strcpy(locomokbd->phys, "locomokbd/input0");

	input_dev->name = "LoCoMo keyboard";
	input_dev->phys = locomokbd->phys;
	input_dev->id.bustype = BUS_HOST;
	input_dev->id.vendor = 0x0001;
	input_dev->id.product = 0x0001;
	input_dev->id.version = 0x0100;
	input_dev->open = locomokbd_open;
	input_dev->close = locomokbd_close;
	input_dev->dev.parent = &dev->dev;

	input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_REP) |
				BIT_MASK(EV_PWR);
	input_dev->keycode = locomokbd->keycode;
	input_dev->keycodesize = sizeof(locomokbd_keycode[0]);
	input_dev->keycodemax = ARRAY_SIZE(locomokbd_keycode);

	input_set_drvdata(input_dev, locomokbd);

	memcpy(locomokbd->keycode, locomokbd_keycode, sizeof(locomokbd->keycode));
	for (i = 0; i < LOCOMOKBD_NUMKEYS; i++)
		set_bit(locomokbd->keycode[i], input_dev->keybit);
	clear_bit(0, input_dev->keybit);

	writew(0, locomokbd->base + LOCOMO_KEYBOARD + LOCOMO_KIC);
	writew(0, locomokbd->base + LOCOMO_KEYBOARD + LOCOMO_KIC);

	/* attempt to get the interrupt */
	err = request_irq(locomokbd->irq, locomokbd_interrupt, 0,
			"locomokbd", locomokbd);
	if (err) {
		printk(KERN_ERR "locomokbd: Can't get irq for keyboard\n");
		goto err_free_mem;
	}

	err = input_register_device(locomokbd->input);
	if (err)
		goto err_free_irq;

	return 0;

err_free_irq:
	free_irq(locomokbd->irq, locomokbd);
err_free_mem:
	platform_set_drvdata(dev, NULL);
	input_free_device(input_dev);

	return err;
}

static int locomokbd_remove(struct platform_device *dev)
{
	struct locomokbd *locomokbd = platform_get_drvdata(dev);

	free_irq(locomokbd->irq, locomokbd);

	del_timer_sync(&locomokbd->timer);

	input_unregister_device(locomokbd->input);
	platform_set_drvdata(dev, NULL);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int locomokbd_resume(struct device *dev)
{
	struct locomokbd *locomokbd = dev_get_drvdata(dev);
	unsigned long flags;
	u16 r;

	spin_lock_irqsave(&locomokbd->lock, flags);

	writew(0, locomokbd->base + LOCOMO_KSC);
	r = readw(locomokbd->base + LOCOMO_KIC);
	r &= 0xFEFF;
	writew(r, locomokbd->base + LOCOMO_KIC);
	writew(0x1, locomokbd->base + LOCOMO_KCMD);

	spin_unlock_irqrestore(&locomokbd->lock, flags);

	return 0;
}

static SIMPLE_DEV_PM_OPS(locomo_kbd_pm, NULL, locomokbd_resume);
#define LOCOMO_KBD_PM	(&locomo_kbd_pm)
#else
#define LOCOMO_KBD_PM	NULL
#endif

static struct platform_driver locomokbd_driver = {
	.driver = {
		.name	= "locomo-kbd",
		.owner	= THIS_MODULE,
		.pm	= LOCOMO_KBD_PM,
	},
	.probe	= locomokbd_probe,
	.remove	= locomokbd_remove,
};

module_platform_driver(locomokbd_driver);

MODULE_AUTHOR("John Lenz <lenz@cs.wisc.edu>");
MODULE_DESCRIPTION("LoCoMo keyboard driver");
MODULE_LICENSE("GPL");
