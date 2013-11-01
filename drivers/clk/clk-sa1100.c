/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>

#include <mach/hardware.h>

enum {
	osc32k768, osc3M6864,
	gpio27_3M6864,
	clk_max,
};

static struct clk *clks[clk_max];

/*
 * In reality it is a switch between 32k768 and 3M6864, driven on GPIO27,
 * if it is enabled by GPIO Alternate function.
 * However nobody uses 32k768 output (as it seems), so there is no point
 * implementing it as a switch.
 */
static int gpio27_enabled;
static int clk_gpio27_enable(struct clk_hw *hw)
{
	/*
	 * First, set up the 3.6864MHz clock on GPIO 27 for the SA-1111:
	 * (SA-1110 Developer's Manual, section 9.1.2.1)
	 */
	GAFR |= GPIO_32_768kHz;
	GPDR |= GPIO_32_768kHz;
	TUCR = TUCR_3_6864MHz;

	gpio27_enabled = 1;
	return 0;
}

static void clk_gpio27_disable(struct clk_hw *hw)
{
	gpio27_enabled = 0;
	TUCR = 0;
	GPDR &= ~GPIO_32_768kHz;
	GAFR &= ~GPIO_32_768kHz;
}

static int clk_gpio27_is_enabled(struct clk_hw *hw)
{
	return gpio27_enabled;
}

static const struct clk_ops clk_gpio27_ops = {
	.enable = clk_gpio27_enable,
	.disable = clk_gpio27_disable,
	.is_enabled = clk_gpio27_is_enabled,
};

static struct clk *clk_register_gpio27(struct device *dev, const char *name,
		const char *parent_name, unsigned long flags)
{
	struct clk_hw *gpio27;
	struct clk *clk;
	struct clk_init_data init;

	/* allocate the gpio27 */
	gpio27 = kzalloc(sizeof(struct clk_hw), GFP_KERNEL);
	if (!gpio27) {
		pr_err("%s: could not allocate gpio27d clk\n", __func__);
		return ERR_PTR(-ENOMEM);
	}

	init.name = name;
	init.ops = &clk_gpio27_ops;
	init.flags = flags | CLK_IS_BASIC;
	init.parent_names = (parent_name ? &parent_name : NULL);
	init.num_parents = (parent_name ? 1 : 0);

	/* struct clk_gpio27 assignments */
	gpio27->init = &init;

	clk = clk_register(dev, gpio27);

	if (IS_ERR(clk))
		kfree(gpio27);

	return clk;
}

/* Fixed clocks on sa1100 */
int __init sa1100_clocks_init(void)
{
	int i;

	clks[osc32k768] = clk_register_fixed_rate(NULL, "osc32k768", NULL,
			CLK_IS_ROOT, 32768);
	clks[osc3M6864] = clk_register_fixed_rate(NULL, "osc3M6864", NULL,
			CLK_IS_ROOT, 3686400);
	clks[gpio27_3M6864] = clk_register_gpio27(NULL, "gpio27_3M6864",
			"osc3M6864", 0);

	clk_register_clkdev(clks[gpio27_3M6864], NULL, "sa1111.0");
	clk_register_clkdev(clks[osc32k768], NULL, "sa1100-rtc");

	for (i = 0; i < ARRAY_SIZE(clks); i++)
		if (IS_ERR(clks[i])) {
			pr_err("SA1100 clk %d: register failed with %ld\n",
				i, PTR_ERR(clks[i]));
			return PTR_ERR(clks[i]);
		}

	return 0;
}
