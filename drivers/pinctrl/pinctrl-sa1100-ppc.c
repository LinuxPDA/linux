#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/platform_device.h>

static const struct pinctrl_pin_desc sa1100_ppc_pins[] = {
	PINCTRL_PIN(0, "LDD0"),
	PINCTRL_PIN(1, "LDD1"),
	PINCTRL_PIN(2, "LDD2"),
	PINCTRL_PIN(3, "LDD3"),
	PINCTRL_PIN(4, "LDD4"),
	PINCTRL_PIN(5, "LDD5"),
	PINCTRL_PIN(6, "LDD6"),
	PINCTRL_PIN(7, "LDD7"),
	PINCTRL_PIN(8, "L_PCLK"),
	PINCTRL_PIN(9, "L_LCLK"),
	PINCTRL_PIN(10, "L_FCLK"),
	PINCTRL_PIN(11, "L_BIAS"),
	PINCTRL_PIN(12, "TXD1"),
	PINCTRL_PIN(13, "RXD1"),
	PINCTRL_PIN(14, "TXD2"),
	PINCTRL_PIN(15, "RXD2"),
	PINCTRL_PIN(16, "TXD3"),
	PINCTRL_PIN(17, "RXD3"),
	PINCTRL_PIN(18, "TXD4"),
	PINCTRL_PIN(19, "RXD4"),
	PINCTRL_PIN(20, "SCLK"),
	PINCTRL_PIN(21, "SFRM"),
};

struct sa1100_group {
	const char *name;
	const unsigned int *pins;
	const unsigned num_pins;
};

#define SA1100_GRP(NAME) { \
	.name = __stringify(NAME) "_grp", \
	.pins = NAME ## _pins, \
	.num_pins = ARRAY_SIZE(NAME ## _pins), \
}

static const unsigned int sp1_pins[] = { 12, 13 };
static const unsigned int sp2_pins[] = { 14, 15 };
static const unsigned int sp3_pins[] = { 16, 17 };
static const unsigned int sp4_pins[] = { 18, 19 };

static const struct sa1100_group sa1100_groups[] = {
	SA1100_GRP(sp1),
	SA1100_GRP(sp2),
	SA1100_GRP(sp3),
	SA1100_GRP(sp4),
};

static int sa1100_get_groups_count(struct pinctrl_dev *pctldev)
{
	return ARRAY_SIZE(sa1100_groups);
}

static const char *sa1100_get_group_name(struct pinctrl_dev *pctldev,
				       unsigned selector)
{
	return sa1100_groups[selector].name;
}

static int sa1100_get_group_pins(struct pinctrl_dev *pctldev, unsigned selector,
			       const unsigned ** pins,
			       unsigned *num_pins)
{
	*pins = sa1100_groups[selector].pins;
	*num_pins = sa1100_groups[selector].num_pins;
	return 0;
}

static struct pinctrl_ops sa1100_ppc_pctrl_ops = {
	.get_groups_count = sa1100_get_groups_count,
	.get_group_name = sa1100_get_group_name,
	.get_group_pins = sa1100_get_group_pins,
};

struct sa1100_ppc_pmx_func {
	const char *name;
	const char * const *groups;
	const unsigned num_groups;
};

static const char * sp1_groups[] = { "sp1_grp" };
static const char * sp2_groups[] = { "sp2_grp" };
static const char * sp3_groups[] = { "sp3_grp" };
static const char * sp4_groups[] = { "sp4_grp" };

#define SA1100_PMX_FUNC(NAME) { \
	.name = __stringify(NAME), \
	.groups = NAME ## _groups , \
	.num_groups = ARRAY_SIZE(NAME ## _groups), \
}

static const struct sa1100_ppc_pmx_func sa1100_ppc_functions[] = {
	SA1100_PMX_FUNC(sp1),
	SA1100_PMX_FUNC(sp2),
	SA1100_PMX_FUNC(sp3),
	SA1100_PMX_FUNC(sp4),
};

int sa1100_ppc_get_functions_count(struct pinctrl_dev *pctldev)
{
	return ARRAY_SIZE(sa1100_ppc_functions);
}

const char *sa1100_ppc_get_fname(struct pinctrl_dev *pctldev, unsigned selector)
{
	return sa1100_ppc_functions[selector].name;
}

static int sa1100_ppc_get_groups(struct pinctrl_dev *pctldev, unsigned selector,
			  const char * const **groups,
			  unsigned * const num_groups)
{
	*groups = sa1100_ppc_functions[selector].groups;
	*num_groups = sa1100_ppc_functions[selector].num_groups;
	return 0;
}

int sa1100_ppc_enable(struct pinctrl_dev *pctldev, unsigned selector,
		unsigned group)
{
	pr_info("Enable %d %d\n", selector, group);
	return 0;
}

void sa1100_ppc_disable(struct pinctrl_dev *pctldev, unsigned selector,
		unsigned group)
{
	pr_info("Disable %d %d\n", selector, group);
}


static struct pinmux_ops sa1100_ppc_pmxops = {
	.get_functions_count = sa1100_ppc_get_functions_count,
	.get_function_name = sa1100_ppc_get_fname,
	.get_function_groups = sa1100_ppc_get_groups,
	.enable = sa1100_ppc_enable,
	.disable = sa1100_ppc_disable,
};

static struct pinctrl_desc sa1100_ppc_desc = {
	.name = "sa1100-ppc",
	.pins = sa1100_ppc_pins,
	.npins = ARRAY_SIZE(sa1100_ppc_pins),
	.pctlops = &sa1100_ppc_pctrl_ops,
	.pmxops = &sa1100_ppc_pmxops,
	.owner = THIS_MODULE,
};

int sa1100_ppc_probe(struct platform_device *pdev)
{
	struct pinctrl_dev *pctl;
	pctl = pinctrl_register(&sa1100_ppc_desc, &pdev->dev, NULL);
	if (!pctl)
		pr_err("could not register sa1100 pin driver\n");
	platform_set_drvdata(pdev, pctl);
	printk("PROBED PINCTRL\n");
	return 0;
}

static int sa1100_ppc_remove(struct platform_device *pdev)
{
	struct pinctrl_dev *pctl;
	pctl = platform_get_drvdata(pdev);
	pinctrl_unregister(pctl);
	return 0;
}

static struct platform_driver sa1100_ppc_driver = {
	.probe = sa1100_ppc_probe,
	.remove = sa1100_ppc_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "sa11x0-ppc",
	},
};

static int __init sa1100_ppc_init(void)
{
	return platform_driver_register(&sa1100_ppc_driver);
}
postcore_initcall(sa1100_ppc_init);

static void __exit sa1100_ppc_exit(void)
{
	platform_driver_unregister(&sa1100_ppc_driver);
}
module_exit(sa1100_ppc_exit);

MODULE_LICENSE("GPLv2");
MODULE_AUTHOR("Dmitry Eremin-Solenikov");
MODULE_DESCRIPTION("SA1100 PPC driver");
