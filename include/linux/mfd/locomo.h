/*
 * arch/arm/include/asm/hardware/locomo.h
 *
 * This file contains the definitions for the LoCoMo G/A Chip
 *
 * (C) Copyright 2004 John Lenz
 *
 * May be copied or modified under the terms of the GNU General Public
 * License.  See linux/COPYING for more information.
 *
 * Based on sa1111.h
 */
#ifndef _ASM_ARCH_LOCOMO
#define _ASM_ARCH_LOCOMO

/* LOCOMO version */
#define LOCOMO_VER	0x00

/* Pin status */
#define LOCOMO_ST	0x04

/* Pin status */
#define LOCOMO_C32K	0x08

/* Interrupt controller */
#define LOCOMO_ICR	0x0C

/* MCS decoder for boot selecting */
#define LOCOMO_MCSX0	0x10
#define LOCOMO_MCSX1	0x14
#define LOCOMO_MCSX2	0x18
#define LOCOMO_MCSX3	0x1c

/* Touch panel controller */
#define LOCOMO_ASD	0x20		/* AD start delay */
#define LOCOMO_HSD	0x28		/* HSYS delay */
#define LOCOMO_HSC	0x2c		/* HSYS period */
#define LOCOMO_TADC	0x30		/* tablet ADC clock */


/* Long time timer */
#define LOCOMO_LTC	0xd8		/* LTC interrupt setting */
#define LOCOMO_LTINT	0xdc		/* LTC interrupt */

/* DAC control signal for LCD (COMADJ ) */
#define LOCOMO_DAC		0xe0
/* DAC control */
#define	LOCOMO_DAC_SCLOEB	0x08	/* SCL pin output data       */
#define	LOCOMO_DAC_TEST		0x04	/* Test bit                  */
#define	LOCOMO_DAC_SDA		0x02	/* SDA pin level (read-only) */
#define	LOCOMO_DAC_SDAOEB	0x01	/* SDA pin output data       */

/* SPI interface */
#define LOCOMO_SPI	0x60
#define LOCOMO_SPIMD	0x00		/* SPI mode setting */
#define LOCOMO_SPICT	0x04		/* SPI mode control */
#define LOCOMO_SPIST	0x08		/* SPI status */
#define	LOCOMO_SPI_TEND	(1 << 3)	/* Transfer end bit */
#define	LOCOMO_SPI_REND	(1 << 2)	/* Receive end bit */
#define	LOCOMO_SPI_RFW	(1 << 1)	/* write buffer bit */
#define	LOCOMO_SPI_RFR	(1)		/* read buffer bit */

#define LOCOMO_SPIIS	0x10		/* SPI interrupt status */
#define LOCOMO_SPIWE	0x14		/* SPI interrupt status write enable */
#define LOCOMO_SPIIE	0x18		/* SPI interrupt enable */
#define LOCOMO_SPIIR	0x1c		/* SPI interrupt request */
#define LOCOMO_SPITD	0x20		/* SPI transfer data write */
#define LOCOMO_SPIRD	0x24		/* SPI receive data read */
#define LOCOMO_SPITS	0x28		/* SPI transfer data shift */
#define LOCOMO_SPIRS	0x2C		/* SPI receive data shift */

/* GPIO */
#define LOCOMO_GPIO		0x90
#define LOCOMO_GPD		0x00	/* GPIO direction */
#define LOCOMO_GPE		0x04	/* GPIO input enable */
#define LOCOMO_GPL		0x08	/* GPIO level */
#define LOCOMO_GPO		0x0c	/* GPIO out data setting */
#define LOCOMO_GRIE		0x10	/* GPIO rise detection */
#define LOCOMO_GFIE		0x14	/* GPIO fall detection */
#define LOCOMO_GIS		0x18	/* GPIO edge detection status */
#define LOCOMO_GWE		0x1c	/* GPIO status write enable */
#define LOCOMO_GIE		0x20	/* GPIO interrupt enable */
#define LOCOMO_GIR		0x24	/* GPIO interrupt request */

/* Start the definitions of the devices.  Each device has an initial
 * base address and a series of offsets from that base address. */

/* Keyboard controller */
#define LOCOMO_KEYBOARD		0x40
#define LOCOMO_KIB		0x00	/* KIB level */
#define LOCOMO_KSC		0x04	/* KSTRB control */
#define LOCOMO_KCMD		0x08	/* KSTRB command */
#define LOCOMO_KIC		0x0c	/* Key interrupt */

/* Front light adjustment controller */
#define LOCOMO_FRONTLIGHT	0xc8
#define LOCOMO_ALS		0x00	/* Adjust light cycle */
#define LOCOMO_ALD		0x04	/* Adjust light duty */

#define LOCOMO_ALC_EN		0x8000

/* Backlight controller: TFT signal */
#define LOCOMO_TFT		0x38
#define LOCOMO_TC		0x00		/* TFT control signal */
#define LOCOMO_CPSD		0x04		/* CPS delay */

/* Audio controller */
#define LOCOMO_AUDIO		0x54
#define LOCOMO_ACC		0x00	/* Audio clock */
#define LOCOMO_PAIF		0xD0	/* PCM audio interface */
/* Audio clock */
#define	LOCOMO_ACC_XON		0x80
#define	LOCOMO_ACC_XEN		0x40
#define	LOCOMO_ACC_XSEL0	0x00
#define	LOCOMO_ACC_XSEL1	0x20
#define	LOCOMO_ACC_MCLKEN	0x10
#define	LOCOMO_ACC_64FSEN	0x08
#define	LOCOMO_ACC_CLKSEL000	0x00	/* mclk  2 */
#define	LOCOMO_ACC_CLKSEL001	0x01	/* mclk  3 */
#define	LOCOMO_ACC_CLKSEL010	0x02	/* mclk  4 */
#define	LOCOMO_ACC_CLKSEL011	0x03	/* mclk  6 */
#define	LOCOMO_ACC_CLKSEL100	0x04	/* mclk  8 */
#define	LOCOMO_ACC_CLKSEL101	0x05	/* mclk 12 */
/* PCM audio interface */
#define	LOCOMO_PAIF_SCINV	0x20
#define	LOCOMO_PAIF_SCEN	0x10
#define	LOCOMO_PAIF_LRCRST	0x08
#define	LOCOMO_PAIF_LRCEVE	0x04
#define	LOCOMO_PAIF_LRCINV	0x02
#define	LOCOMO_PAIF_LRCEN	0x01

/* LED controller */
#define LOCOMO_LED		0xe8
#define LOCOMO_LPT0		0x00
#define LOCOMO_LPT1		0x04
/* LED control */
#define LOCOMO_LPT_TOFH		0x80
#define LOCOMO_LPT_TOFL		0x08
#define LOCOMO_LPT_TOH(TOH)	((TOH & 0x7) << 4)
#define LOCOMO_LPT_TOL(TOL)	((TOL & 0x7))

struct locomo_gpio_platform_data {
	unsigned int gpio_base;
};

struct locomo_lcd_platform_data {
	u8 comadj;
	int gpio_lcd_vsha_on;
	int gpio_lcd_vshd_on;
	int gpio_lcd_vee_on;
	int gpio_lcd_mod;
};

struct locomo_bl_platform_data {
	int gpio_fl_vr;
};

struct locomo_platform_data {
	struct locomo_gpio_platform_data gpio_data;
	struct locomo_lcd_platform_data lcd_data;
	struct locomo_bl_platform_data bl_data;

	int gpio_amp1_on;
	int gpio_amp2_on;
};

/* Send data to i2c DAC connected to LoCoMo i2c bus */
void locomo_m62332_senddata(struct device *dev, unsigned int dac_data, int channel);
/* Rather hackish interface while we don't have CDF */
void locomolcd_power(int on);

#endif
