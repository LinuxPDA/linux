/*
 * linux/arch/arm/mach-sa1100/collie.c
 *
 * May be copied or modified under the terms of the GNU General Public
 * License.  See linux/COPYING for more information.
 *
 * This file contains all Collie-specific tweaks.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * ChangeLog:
 *  2006 Pavel Machek <pavel@ucw.cz>
 *  03-06-2004 John Lenz <lenz@cs.wisc.edu>
 *  06-04-2002 Chris Larson <kergoth@digitalnemesis.net>
 *  04-16-2001 Lineo Japan,Inc. ...
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/tty.h>
#include <linux/delay.h>
#include <linux/platform_data/sa11x0-serial.h>
#include <linux/platform_device.h>
#include <linux/mfd/ucb1x00.h>
#include <linux/mfd/locomo.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/timer.h>
#include <linux/gpio.h>
#include <linux/power/gpio-charger.h>
#include <linux/mmc/host.h>
#include <linux/spi/mmc_spi.h>

#include <video/sa1100fb.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/page.h>
#include <asm/setup.h>
#include <mach/collie.h>

#include <asm/mach/arch.h>
#include <asm/mach/flash.h>
#include <asm/mach/map.h>

#include <asm/hardware/scoop.h>
#include <asm/mach/sharpsl_param.h>
#include <linux/platform_data/mfd-mcp-sa11x0.h>
#include <mach/irqs.h>

#include "generic.h"

static struct resource collie_scoop_resources[] = {
	[0] = DEFINE_RES_MEM(0x40800000, SZ_4K),
};

static struct scoop_config collie_scoop_setup = {
	.io_dir 	= COLLIE_SCOOP_IO_DIR,
	.io_out		= COLLIE_SCOOP_IO_OUT,
	.gpio_base	= COLLIE_SCOOP_GPIO_BASE,
};

struct platform_device colliescoop_device = {
	.name		= "sharp-scoop",
	.id		= -1,
	.dev		= {
 		.platform_data	= &collie_scoop_setup,
	},
	.num_resources	= ARRAY_SIZE(collie_scoop_resources),
	.resource	= collie_scoop_resources,
};

static struct scoop_pcmcia_dev collie_pcmcia_scoop[] = {
	{
	.dev		= &colliescoop_device.dev,
	.irq		= COLLIE_IRQ_GPIO_CF_IRQ,
	.cd_irq		= COLLIE_IRQ_GPIO_CF_CD,
	.cd_irq_str	= "PCMCIA0 CD",
	},
};

static struct scoop_pcmcia_config collie_pcmcia_config = {
	.devs		= &collie_pcmcia_scoop[0],
	.num_devs	= 1,
};

static struct ucb1x00_plat_data collie_ucb1x00_data = {
	.gpio_base	= COLLIE_TC35143_GPIO_BASE,
};

static struct mcp_plat_data collie_mcp_data = {
	.mccr0		= MCCR0_ADM | MCCR0_ExtClk,
	.sclk_rate	= 9216000,
	.codec_pdata	= &collie_ucb1x00_data,
};

/*
 * Collie AC IN
 */
static char *collie_ac_supplied_to[] = {
	"main-battery",
	"backup-battery",
};


static struct gpio_charger_platform_data collie_power_data = {
	.name			= "charger",
	.type			= POWER_SUPPLY_TYPE_MAINS,
	.gpio			= COLLIE_GPIO_AC_IN,
	.supplied_to		= collie_ac_supplied_to,
	.num_supplicants	= ARRAY_SIZE(collie_ac_supplied_to),
};

static struct platform_device collie_power_device = {
	.name			= "gpio-charger",
	.id			= -1,
	.dev.platform_data	= &collie_power_data,
};

/*
 * low-level UART features.
 */
static struct gpio collie_uart_gpio[] = {
	{ COLLIE_GPIO_CTS, GPIOF_IN, "CTS" },
	{ COLLIE_GPIO_RTS, GPIOF_OUT_INIT_LOW, "RTS" },
	{ COLLIE_GPIO_DTR, GPIOF_OUT_INIT_LOW, "DTR" },
	{ COLLIE_GPIO_DSR, GPIOF_IN, "DSR" },
};

static bool collie_uart_gpio_ok;

static void collie_uart_set_mctrl(struct uart_port *port, u_int mctrl)
{
	if (!collie_uart_gpio_ok) {
		int rc = gpio_request_array(collie_uart_gpio,
				ARRAY_SIZE(collie_uart_gpio));
		if (rc)
			printk("collie_uart_set_mctrl: gpio request %d\n", rc);
		else
			collie_uart_gpio_ok = true;
	}

	if (collie_uart_gpio_ok) {
		gpio_set_value(COLLIE_GPIO_RTS, !(mctrl & TIOCM_RTS));
		gpio_set_value(COLLIE_GPIO_DTR, !(mctrl & TIOCM_DTR));
	}
}

static u_int collie_uart_get_mctrl(struct uart_port *port)
{
	int ret = TIOCM_CD;

	if (!collie_uart_gpio_ok) {
		int rc = gpio_request_array(collie_uart_gpio,
				ARRAY_SIZE(collie_uart_gpio));
		if (rc)
			printk("collie_uart_get_mctrl: gpio request %d\n", rc);
		else
			collie_uart_gpio_ok = true;
	}

	if (!collie_uart_gpio_ok)
		return ret;

	if (gpio_get_value(COLLIE_GPIO_CTS))
		ret |= TIOCM_CTS;
	if (gpio_get_value(COLLIE_GPIO_DSR))
		ret |= TIOCM_DSR;

	return ret;
}

static struct sa1100_port_fns collie_port_fns __initdata = {
	.set_mctrl	= collie_uart_set_mctrl,
	.get_mctrl	= collie_uart_get_mctrl,
};


static struct resource locomo_resources[] = {
	[0] = DEFINE_RES_MEM(0x40000000, SZ_8K),
	[1] = DEFINE_RES_IRQ(IRQ_GPIO25),
};

static struct locomo_platform_data locomo_info = {
	.gpio_data = {
		.gpio_base = COLLIE_LOCOMO_GPIO_BASE,
	},
	.lcd_data = {
		.comadj	          = 128,
		.gpio_lcd_vsha_on = COLLIE_GPIO_LCD_VSHA_ON,
		.gpio_lcd_vshd_on = COLLIE_GPIO_LCD_VSHD_ON,
		.gpio_lcd_vee_on  = COLLIE_GPIO_LCD_VEE_ON,
		.gpio_lcd_mod     = COLLIE_GPIO_LCD_MOD,
	},
	.bl_data = {
		.gpio_fl_vr       = COLLIE_GPIO_FL_VR,
	},
	.gpio_amp1_on	  = COLLIE_GPIO_AMP1_ON,
	.gpio_amp2_on	  = COLLIE_GPIO_AMP2_ON,
};

static struct platform_device collie_locomo_device = {
	.name		= "locomo",
	.id		= 0,
	.dev		= {
		.platform_data  = &locomo_info,
	},
	.num_resources	= ARRAY_SIZE(locomo_resources),
	.resource	= locomo_resources,
};

static int collie_mmc_init(struct device *dev,
		irqreturn_t (*isr)(int, void*), void *mmc)
{
	int ret = gpio_request(COLLIE_GPIO_CARD_POWER, "MMC power");
	if (!ret)
		ret = gpio_direction_output(COLLIE_GPIO_CARD_POWER, 0);
	if (ret)
		gpio_free(COLLIE_GPIO_CARD_POWER);
	return ret;
}

static void collie_mmc_exit(struct device *dev, void *mmc)
{
	gpio_free(COLLIE_GPIO_CARD_POWER);
}

static void collie_mmc_setpower(struct device *dev, unsigned int mask)
{
	gpio_set_value(COLLIE_GPIO_CARD_POWER, !!mask);
}

static struct mmc_spi_platform_data collie_mmc_data = {
	.init		= collie_mmc_init,
	.exit		= collie_mmc_exit,
	.setpower	= collie_mmc_setpower,
	.detect_delay 	= 200,
	.powerup_msecs  = 200,
	.ocr_mask 	= MMC_VDD_32_33 | MMC_VDD_33_34,
	.flags		= MMC_SPI_USE_CD_GPIO | MMC_SPI_USE_RO_GPIO,
	.cd_gpio	= COLLIE_GPIO_CARD_DETECT,
	.ro_gpio	= COLLIE_GPIO_CARD_RO,
	.caps2		= MMC_CAP2_RO_ACTIVE_HIGH,
};

static struct spi_board_info collie_spi_board_info[] __initdata = {
	{
		.modalias	= "mmc_spi",
		.platform_data	= &collie_mmc_data,
		.max_speed_hz	= 25000000,
		.bus_num	= 0,
		.chip_select	= 0,
		.mode		= SPI_MODE_0, // FIXME 0?
	},
};

static struct platform_device *devices[] __initdata = {
	&collie_locomo_device,
	&colliescoop_device,
	&collie_power_device,
};

static struct mtd_partition collie_partitions[] = {
	{
		.name		= "bootloader",
		.offset 	= 0,
		.size		= 0x000C0000,
		.mask_flags	= MTD_WRITEABLE
	}, {
		.name		= "kernel",
		.offset 	= MTDPART_OFS_APPEND,
		.size		= 0x00100000,
	}, {
		.name		= "rootfs",
		.offset 	= MTDPART_OFS_APPEND,
		.size		= 0x00e20000,
	}
};

static int collie_flash_init(void)
{
	int rc = gpio_request(COLLIE_GPIO_VPEN, "flash Vpp enable");
	if (rc)
		return rc;

	rc = gpio_direction_output(COLLIE_GPIO_VPEN, 1);
	if (rc)
		gpio_free(COLLIE_GPIO_VPEN);

	return rc;
}

static void collie_set_vpp(int vpp)
{
	gpio_set_value(COLLIE_GPIO_VPEN, vpp);
}

static void collie_flash_exit(void)
{
	gpio_free(COLLIE_GPIO_VPEN);
}

static struct flash_platform_data collie_flash_data = {
	.map_name	= "jedec_probe",
	.init		= collie_flash_init,
	.set_vpp	= collie_set_vpp,
	.exit		= collie_flash_exit,
	.parts		= collie_partitions,
	.nr_parts	= ARRAY_SIZE(collie_partitions),
};

static struct resource collie_flash_resources[] = {
	DEFINE_RES_MEM(SA1100_CS0_PHYS, SZ_32M),
};

static struct sa1100fb_mach_info collie_lcd_info = {
	.pixclock	= 171521,	.bpp		= 16,
	.xres		= 320,		.yres		= 240,

	.hsync_len	= 5,		.vsync_len	= 1,
	.left_margin	= 11,		.upper_margin	= 2,
	.right_margin	= 30,		.lower_margin	= 0,

	.sync		= FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,

	.lccr0		= LCCR0_Color | LCCR0_Sngl | LCCR0_Act,
	.lccr3		= LCCR3_OutEnH | LCCR3_PixRsEdg | LCCR3_ACBsDiv(2),

#ifdef CONFIG_BACKLIGHT_LOCOMO
	.lcd_power	= locomolcd_power
#endif
};

static void __init collie_init(void)
{
	int ret = 0;

	/* cpu initialize */
	GAFR = GPIO_SSP_TXD | GPIO_SSP_SCLK | GPIO_SSP_SFRM | GPIO_SSP_CLK |
		GPIO_MCP_CLK | GPIO_32_768kHz;

	GPDR = GPIO_LDD8 | GPIO_LDD9 | GPIO_LDD10 | GPIO_LDD11 | GPIO_LDD12 |
		GPIO_LDD13 | GPIO_LDD14 | GPIO_LDD15 | GPIO_SSP_TXD |
		GPIO_SSP_SCLK | GPIO_SSP_SFRM | GPIO_SDLC_SCLK |
		_COLLIE_GPIO_UCB1x00_RESET | _COLLIE_GPIO_nMIC_ON |
		_COLLIE_GPIO_nREMOCON_ON | GPIO_32_768kHz;

	PPDR = PPC_LDD0 | PPC_LDD1 | PPC_LDD2 | PPC_LDD3 | PPC_LDD4 | PPC_LDD5 |
		PPC_LDD6 | PPC_LDD7 | PPC_L_PCLK | PPC_L_LCLK | PPC_L_FCLK | PPC_L_BIAS |
		PPC_TXD1 | PPC_TXD2 | PPC_TXD3 | PPC_TXD4 | PPC_SCLK | PPC_SFRM;

	PWER = _COLLIE_GPIO_AC_IN | _COLLIE_GPIO_CO | _COLLIE_GPIO_ON_KEY |
		_COLLIE_GPIO_WAKEUP | _COLLIE_GPIO_nREMOCON_INT | PWER_RTC;

	PGSR = _COLLIE_GPIO_nREMOCON_ON;

	PSDR = PPC_RXD1 | PPC_RXD2 | PPC_RXD3 | PPC_RXD4;

	PCFR = PCFR_OPDE;

	GPSR |= _COLLIE_GPIO_UCB1x00_RESET;

	sharpsl_save_param();

	sa11x0_ppc_configure_mcp();


	platform_scoop_config = &collie_pcmcia_config;

	if (sharpsl_param.comadj != -1)
		locomo_info.lcd_data.comadj = sharpsl_param.comadj;

	ret = platform_add_devices(devices, ARRAY_SIZE(devices));
	if (ret) {
		printk(KERN_WARNING "collie: Unable to register LoCoMo device\n");
	}

	sa11x0_register_lcd(&collie_lcd_info);
	sa11x0_register_mtd(&collie_flash_data, collie_flash_resources,
			    ARRAY_SIZE(collie_flash_resources));
	sa11x0_register_mcp(&collie_mcp_data);

	spi_register_board_info(collie_spi_board_info,
			ARRAY_SIZE(collie_spi_board_info));
}

static struct map_desc collie_io_desc[] __initdata = {
	{	/* 32M main flash (cs0) */
		.virtual	= 0xe8000000,
		.pfn		= __phys_to_pfn(0x00000000),
		.length		= 0x02000000,
		.type		= MT_DEVICE
	}, {	/* 32M boot flash (cs1) */
		.virtual	= 0xea000000,
		.pfn		= __phys_to_pfn(0x08000000),
		.length		= 0x02000000,
		.type		= MT_DEVICE
	}
};

static void __init collie_map_io(void)
{
	sa1100_map_io();
	iotable_init(collie_io_desc, ARRAY_SIZE(collie_io_desc));

	sa1100_register_uart_fns(&collie_port_fns);
	sa1100_register_uart(0, 3);
	sa1100_register_uart(1, 1);
}

MACHINE_START(COLLIE, "Sharp-Collie")
	.map_io		= collie_map_io,
	.nr_irqs	= SA1100_NR_IRQS,
	.init_irq	= sa1100_init_irq,
	.init_time	= sa1100_timer_init,
	.init_machine	= collie_init,
	.init_late	= sa11x0_init_late,
	.restart	= sa11x0_restart,
MACHINE_END
