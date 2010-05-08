/*
 * at91_spi.c - at91 SPI driver (BOOTSTRAP/BITBANG VERSION)
 *
 * Copyright (C) 2006 David Brownell
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
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>

#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>

#include <asm/arch/gpio.h>


/*
 * FIXME this bitbanging version is just to help bootstrap systems until
 * there's a native SPI+IRQ+DMA controller driver ... such a driver should
 * be a drop-in replacement for this one, and much faster.
 *
 * remember:
 *
 *	- other at91 parts (like at91sam9) have multiple controllers
 *	  and different pin muxing; this version is at91rm9200 specfic.
 *
 *	- at91sam9261 SPI0 pins are directly muxed with MMC/SD pins.
 *
 *	- rm9200 spi chipselects drop wrongly, so the native driver
 *	  will need to use gpios much like this does.
 *
 *	- real hardware only allows 8..16 bits per word, while this
 *	  bitbanger allows 1..32 (incompatible superset).
 *
 *	- this disregards clock parameters.  with inlined gpio calls,
 *	  gcc 3.4.4 produces about 1.5 mbit/sec, more than 2x faster
 *	  than using the subroutined veresion from txrx_word().
 *
 *	- suspend/resume and <linux/clk.h> support is missing ...
 */

#define	spi_miso_bit	AT91_PIN_PA0
#define	spi_mosi_bit	AT91_PIN_PA1
#define	spi_sck_bit	AT91_PIN_PA2

struct at91_spi {
	struct spi_bitbang	bitbang;
	struct platform_device	*pdev;
};

/*----------------------------------------------------------------------*/

static inline void setsck(struct spi_device *spi, int is_on)
{
	at91_set_gpio_value(spi_sck_bit, is_on);
}

static inline void setmosi(struct spi_device *spi, int is_on)
{
	at91_set_gpio_value(spi_mosi_bit, is_on);
}

static inline int getmiso(struct spi_device *spi)
{
	return at91_get_gpio_value(spi_miso_bit);
}

static void at91_spi_chipselect(struct spi_device *spi, int is_active)
{
	unsigned long cs = (unsigned long) spi->controller_data;

	/* set default clock polarity */
	if (is_active)
		setsck(spi, spi->mode & SPI_CPOL);

	/* only support active-low (default) */
	at91_set_gpio_value(cs, !is_active);
}

/*
 * NOTE:  this is "as fast as we can"; it should be a function of
 * the device clock ...
 */
#define	spidelay(X)	do{} while(0)

#define	EXPAND_BITBANG_TXRX
#include <linux/spi/spi_bitbang.h>

static u32 at91_spi_txrx_word_mode0(struct spi_device *spi,
		unsigned nsecs, u32 word, u8 bits)
{
	return bitbang_txrx_be_cpha0(spi, nsecs, 0, word, 8);
}

static u32 at91_spi_txrx_word_mode1(struct spi_device *spi,
		unsigned nsecs, u32 word, u8 bits)
{
	return bitbang_txrx_be_cpha1(spi, nsecs, 0, word, 8);
}

static u32 at91_spi_txrx_word_mode2(struct spi_device *spi,
		unsigned nsecs, u32 word, u8 bits)
{
	return bitbang_txrx_be_cpha0(spi, nsecs, 1, word, 8);
}

static u32 at91_spi_txrx_word_mode3(struct spi_device *spi,
		unsigned nsecs, u32 word, u8 bits)
{
	return bitbang_txrx_be_cpha1(spi, nsecs, 1, word, 8);
}

/*----------------------------------------------------------------------*/

static int __init at91_spi_probe(struct platform_device *pdev)
{
	int			status;
	struct spi_master	*master;
	struct at91_spi		*at91_spi;

	if (pdev->id != 0)	/* SPI0 bus */
		return -EINVAL;

	master = spi_alloc_master(&pdev->dev, sizeof *at91_spi);
	if (!master)
		return -ENOMEM;

	at91_spi = spi_master_get_devdata(master);
	at91_spi->pdev = pdev;
	platform_set_drvdata(pdev, at91_spi);

	/* SPI and bitbang hookup */
	master->bus_num = 0;
	master->num_chipselect = 4;

	at91_spi->bitbang.master = spi_master_get(master);
	at91_spi->bitbang.chipselect = at91_spi_chipselect;
	at91_spi->bitbang.txrx_word[SPI_MODE_0] = at91_spi_txrx_word_mode0;
	at91_spi->bitbang.txrx_word[SPI_MODE_1] = at91_spi_txrx_word_mode1;
	at91_spi->bitbang.txrx_word[SPI_MODE_2] = at91_spi_txrx_word_mode2;
	at91_spi->bitbang.txrx_word[SPI_MODE_3] = at91_spi_txrx_word_mode3;

	status = spi_bitbang_start(&at91_spi->bitbang);
	if (status < 0)
		(void) spi_master_put(at91_spi->bitbang.master);

	return status;
}

static int __exit at91_spi_remove(struct platform_device *pdev)
{
	struct at91_spi	*at91_spi = platform_get_drvdata(pdev);
	int status;

	/* stop() unregisters child devices too */
	status = spi_bitbang_stop(&at91_spi->bitbang);
	(void) spi_master_put(at91_spi->bitbang.master);

	platform_set_drvdata(pdev, NULL);
	return status;
}

static struct platform_driver at91_spi_driver = {
	.probe		= at91_spi_probe,
	.remove		= __exit_p(at91_spi_remove),
	.driver		= {
		.name	= "at91_spi",
		.owner	= THIS_MODULE,
	},
};

static int __init at91_spi_init(void)
{
	at91_set_gpio_output(spi_sck_bit, 0);
	at91_set_gpio_output(spi_mosi_bit, 0);
	at91_set_gpio_input(spi_miso_bit, 1 /* pullup */);

	/* register driver */
	return platform_driver_register(&at91_spi_driver);
}

static void __exit at91_spi_exit(void)
{
	platform_driver_unregister(&at91_spi_driver);
}

device_initcall(at91_spi_init);
module_exit(at91_spi_exit);

MODULE_ALIAS("at91_spi.0");

MODULE_DESCRIPTION("AT91 SPI support (BOOTSTRAP/BITBANG VERSION)");
MODULE_AUTHOR("David Brownell");
MODULE_LICENSE("GPL");
