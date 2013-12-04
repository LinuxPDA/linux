//#define DEBUG
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>
#include <linux/mfd/locomo.h>
#include <linux/delay.h>

struct locomospi_dev {
	struct spi_bitbang bitbang;
	void __iomem *base;
	int clock_base;
	int clock_div;
};

static void locomospi_reg_open(struct locomospi_dev *spidev)
{
	u16 r;

	spidev->clock_div = DIV_64;
	spidev->clock_base = CLOCK_18MHZ;
	writew(LOCOMO_SPIMD_MSB1ST | LOCOMO_SPIMD_DOSTAT | LOCOMO_SPIMD_RCPOL |
		  LOCOMO_SPIMD_TCPOL | (spidev->clock_base << 3) | spidev->clock_div,
		  spidev->base + LOCOMO_SPIMD);

	/* if (locomospi_carddetect()) { */
	r = readw(spidev->base + LOCOMO_SPIMD);
	r |= LOCOMO_SPIMD_XON;
	writew(r, spidev->base + LOCOMO_SPIMD);

	r = readw(spidev->base + LOCOMO_SPIMD);
	r |= LOCOMO_SPIMD_XEN;
	writew(r, spidev->base + LOCOMO_SPIMD);
	/* } */

	writew(LOCOMO_SPICT_CS, spidev->base + LOCOMO_SPICT);

	r = readw(spidev->base + LOCOMO_SPICT);
	r |= (LOCOMO_SPICT_CEN | LOCOMO_SPICT_RXUEN | LOCOMO_SPICT_ALIGNEN);
	writew(r, spidev->base + LOCOMO_SPICT);

	udelay(200);

	r = readw(spidev->base + LOCOMO_SPICT);
	writew(r, spidev->base + LOCOMO_SPICT);

	r = readw(spidev->base + LOCOMO_SPICT);
	r &= ~LOCOMO_SPICT_CS;
	writew(r, spidev->base + LOCOMO_SPICT);
}

static void locomospi_reg_release(struct locomospi_dev *spidev)
{
	u16 r;

	r = readw(spidev->base + LOCOMO_SPICT);
	r &= ~LOCOMO_SPICT_CEN;
	writew(r, spidev->base + LOCOMO_SPICT);

	r = readw(spidev->base + LOCOMO_SPIMD);
	r &= ~LOCOMO_SPIMD_XEN;
	writew(r, spidev->base + LOCOMO_SPIMD);

	r = readw(spidev->base + LOCOMO_SPIMD);
	r &= ~LOCOMO_SPIMD_XON;
	writew(r, spidev->base + LOCOMO_SPIMD);

	r = readw(spidev->base + LOCOMO_SPICT);
	r |= LOCOMO_SPIMD_XEN; /* FIXME */
	writew(r, spidev->base + LOCOMO_SPICT);
}


static void locomospi_chipselect(struct spi_device *spi, int is_active)
{
	struct locomospi_dev *spidev;
	u16 r;

	dev_dbg(&spi->dev, "SPI cs: %d\n", is_active);

	spidev = spi_master_get_devdata(spi->master);

	r = readw(spidev->base + LOCOMO_SPICT);
	if (!!is_active ^ !!(spi->mode & SPI_CS_HIGH))
		r &= ~LOCOMO_SPICT_CS;
	else
		r |= LOCOMO_SPICT_CS;
	writew(r, spidev->base + LOCOMO_SPICT);
}

static u32 locomospi_txrx_word(struct spi_device *spi,
		unsigned nsecs,
		u32 word, u8 bits)
{
	struct locomospi_dev *spidev;
	int wait;
	int j;
	u32 rx;

	spidev = spi_master_get_devdata(spi->master);

	if (spidev->clock_div == 4)
		wait = 0x10000;
	else
		wait = 8;

	for (j = 0; j < wait; j++) {
		if (readw(spidev->base + LOCOMO_SPIST) & LOCOMO_SPI_RFW)
			break;
	}

	writeb(word, spidev->base + LOCOMO_SPITD);
	ndelay(nsecs);

	for (j = 0; j < wait; j++) {
		if (readw(spidev->base + LOCOMO_SPIST) & LOCOMO_SPI_RFR)
			break;
	}

	rx = readb(spidev->base + LOCOMO_SPIRD);
	ndelay(nsecs);

	dev_dbg(&spi->dev, "SPI txrx: %02x/%02x\n", word, rx);

	return rx;
}

static void locomo_spi_set_speed(struct locomospi_dev *spidev, u32 hz)
{
	u16 r;

	if (hz >= 24576000) {
		spidev->clock_base = CLOCK_25MHZ;
		spidev->clock_div = DIV_1;
	} else if (hz >= 22579200) {
		spidev->clock_base = CLOCK_22MHZ;
		spidev->clock_div = DIV_1;
	} else if (hz >= 18432000) {
		spidev->clock_base = CLOCK_18MHZ;
		spidev->clock_div = DIV_1;
	} else if (hz >= 12288000) {
		spidev->clock_base = CLOCK_25MHZ;
		spidev->clock_div = DIV_2;
	} else if (hz >= 11289600) {
		spidev->clock_base = CLOCK_22MHZ;
		spidev->clock_div = DIV_2;
	} else if (hz >= 9216000) {
		spidev->clock_base = CLOCK_18MHZ;
		spidev->clock_div = DIV_2;
	} else if (hz >= 6144000) {
		spidev->clock_base = CLOCK_25MHZ;
		spidev->clock_div = DIV_4;
	} else if (hz >= 5644800) {
		spidev->clock_base = CLOCK_22MHZ;
		spidev->clock_div = DIV_4;
	} else if (hz >= 4608000) {
		spidev->clock_base = CLOCK_18MHZ;
		spidev->clock_div = DIV_4;
	} else if (hz >= 3072000) {
		spidev->clock_base = CLOCK_25MHZ;
		spidev->clock_div = DIV_8;
	} else if (hz >= 2822400) {
		spidev->clock_base = CLOCK_22MHZ;
		spidev->clock_div = DIV_8;
	} else if (hz >= 2304000) {
		spidev->clock_base = CLOCK_18MHZ;
		spidev->clock_div = DIV_8;
	} else if (hz >= 384000) {
		spidev->clock_base = CLOCK_25MHZ;
		spidev->clock_div = DIV_64;
	} else if (hz >= 352800) {
		spidev->clock_base = CLOCK_22MHZ;
		spidev->clock_div = DIV_64;
	} else {		/* set to 288 Khz */
		spidev->clock_base = CLOCK_18MHZ;
		spidev->clock_div = DIV_64;
	}

	r = readw(spidev->base + LOCOMO_SPIMD);
	if ((r & LOCOMO_SPIMD_CLKSEL) == spidev->clock_div &&
			(r & LOCOMO_SPIMD_XSEL) == (spidev->clock_div << 3))
		return;

	r &= ~(LOCOMO_SPIMD_XSEL | LOCOMO_SPIMD_CLKSEL | LOCOMO_SPIMD_XEN);
	writew(r, spidev->base + LOCOMO_SPIMD);

	r |= (spidev->clock_div | (spidev->clock_base << 3) | LOCOMO_SPIMD_XEN);
	writew(r, spidev->base + LOCOMO_SPIMD);

	udelay(300);
}

static int locomo_spi_setup_transfer(struct spi_device *spi, struct spi_transfer *t)
{
	struct locomospi_dev *spidev;
	u16 r;
	u32 hz = 0;
	int rc = spi_bitbang_setup_transfer(spi, t);

	if (rc)
		return rc;

	if (t)
		hz = t->speed_hz;
	if (!hz)
		hz = spi->max_speed_hz;

	spidev = spi_master_get_devdata(spi->master);

	r = readw(spidev->base + LOCOMO_SPIMD);
	if (hz == 0) {
		r &= ~LOCOMO_SPIMD_XON;
		writew(r, spidev->base + LOCOMO_SPIMD);
	} else {
		r |= LOCOMO_SPIMD_XON;
		writew(r, spidev->base + LOCOMO_SPIMD);
		locomo_spi_set_speed(spidev, hz);
	}

	return 0;
}


static int locomo_spi_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct spi_master *master;
	struct locomospi_dev *spidev;
	int ret;

	printk(KERN_DEBUG "Collie MMC over SPI Driver\n");

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	master = spi_alloc_master(&pdev->dev, sizeof(struct locomospi_dev));
	if (!master)
		return -ENOMEM;

	platform_set_drvdata(pdev, master);

	master->bits_per_word_mask = SPI_BPW_RANGE_MASK(8, 8); // FIXME
	master->bus_num = 0; // FIXME
	master->num_chipselect = 1;

	spidev = spi_master_get_devdata(master);
	spidev->bitbang.master = spi_master_get(master);

	spidev->bitbang.setup_transfer = locomo_spi_setup_transfer;
	spidev->bitbang.chipselect = locomospi_chipselect;
	spidev->bitbang.txrx_word[SPI_MODE_0] = locomospi_txrx_word;
	spidev->bitbang.txrx_word[SPI_MODE_1] = locomospi_txrx_word;
	spidev->bitbang.txrx_word[SPI_MODE_2] = locomospi_txrx_word;
	spidev->bitbang.txrx_word[SPI_MODE_3] = locomospi_txrx_word;

	spidev->bitbang.master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_CS_HIGH;

	spidev->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(spidev->base)) {
		ret = PTR_ERR(spidev->base);
		goto out_put;
	}

	locomospi_reg_open(spidev);

	ret = spi_bitbang_start(&spidev->bitbang);
	if (ret) {
		dev_err(&pdev->dev, "bitbang start failed with %d\n", ret);
		goto out_put;
	}

	return 0;

out_put:
	spi_master_put(master);
	return ret;
}

static int locomo_spi_remove(struct platform_device *pdev)
{
	struct spi_master *master = platform_get_drvdata(pdev);
	struct locomospi_dev *spidev = spi_master_get_devdata(master);

	spi_bitbang_stop(&spidev->bitbang);
	locomospi_reg_release(spidev);
	spi_master_put(master);


	return 0;
}

static struct platform_driver locomo_spi_driver = {
	.probe = locomo_spi_probe,
	.remove = locomo_spi_remove,
	.driver = {
		.name = "locomo-spi",
		.owner = THIS_MODULE,
	},
};
module_platform_driver(locomo_spi_driver);

MODULE_AUTHOR("Thomas Kunze thommy@tabao.de");
MODULE_DESCRIPTION("Collie mmc driver");
MODULE_LICENSE("GPL");

