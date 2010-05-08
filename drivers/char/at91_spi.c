/*
 * Serial Peripheral Interface (SPI) driver for the Atmel AT91RM9200 (Thunder)
 *
 *  Copyright (C) SAN People (Pty) Ltd
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/completion.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/atmel_pdc.h>
#include <asm/io.h>
#include <asm/semaphore.h>

#include <asm/arch/at91_spi.h>
#include <asm/arch/board.h>
#include <asm/arch/spi.h>

#undef DEBUG_SPI

static struct spi_local spi_dev[NR_SPI_DEVICES];	/* state of the SPI devices */
static int spi_enabled = 0;
static struct semaphore spi_lock;			/* protect access to SPI bus */
static int current_device = -1;				/* currently selected SPI device */
static struct clk *spi_clk;				/* SPI clock */
static void __iomem *spi_base;				/* SPI peripheral base-address */

DECLARE_COMPLETION(transfer_complete);


#define at91_spi_read(reg)		__raw_readl(spi_base + (reg))
#define at91_spi_write(reg, val)	__raw_writel((val), spi_base + (reg))


/* ......................................................................... */

/*
 * Access and enable the SPI bus.
 * This MUST be called before any transfers are performed.
 */
void spi_access_bus(short device)
{
	/* Ensure that requested device is valid */
	if ((device < 0) || (device >= NR_SPI_DEVICES))
		panic("at91_spi: spi_access_bus called with invalid device");

	if (spi_enabled == 0) {
		clk_enable(spi_clk);				/* Enable Peripheral clock */
		at91_spi_write(AT91_SPI_CR, AT91_SPI_SPIEN);	/* Enable SPI */
#ifdef DEBUG_SPI
		printk("SPI on\n");
#endif
	}
	spi_enabled++;

	/* Lock the SPI bus */
	down(&spi_lock);
	current_device = device;

	/* Configure SPI bus for device */
	at91_spi_write(AT91_SPI_MR, AT91_SPI_MSTR | AT91_SPI_MODFDIS | (spi_dev[device].pcs << 16));
}

/*
 * Relinquish control of the SPI bus.
 */
void spi_release_bus(short device)
{
	if (device != current_device)
		panic("at91_spi: spi_release called with invalid device");

	/* Release the SPI bus */
	current_device = -1;
	up(&spi_lock);

	spi_enabled--;
	if (spi_enabled == 0) {
		at91_spi_write(AT91_SPI_CR, AT91_SPI_SPIDIS);	/* Disable SPI */
		clk_disable(spi_clk);				/* Disable Peripheral clock */
#ifdef DEBUG_SPI
		printk("SPI off\n");
#endif
	}
}

/*
 * Perform a data transfer over the SPI bus
 */
int spi_transfer(struct spi_transfer_list* list)
{
	struct spi_local *device = (struct spi_local *) &spi_dev[current_device];
	int tx_size;

	if (!list)
		panic("at91_spi: spi_transfer called with NULL transfer list");
	if (current_device == -1)
		panic("at91_spi: spi_transfer called without acquiring bus");

#ifdef DEBUG_SPI
	printk("SPI transfer start [%i]\n", list->nr_transfers);
#endif

	/* If we are in 16-bit mode, we need to modify what we pass to the PDC */
	tx_size = (at91_spi_read(AT91_SPI_CSR(current_device)) & AT91_SPI_BITS_16) ? 2 : 1;

	/* Store transfer list */
	device->xfers = list;
	list->curr = 0;

	/* Assume there must be at least one transfer */
	device->tx = dma_map_single(NULL, list->tx[0], list->txlen[0], DMA_TO_DEVICE);
	device->rx = dma_map_single(NULL, list->rx[0], list->rxlen[0], DMA_FROM_DEVICE);

	/* Program PDC registers */
	at91_spi_write(ATMEL_PDC_TPR, device->tx);
	at91_spi_write(ATMEL_PDC_RPR, device->rx);
	at91_spi_write(ATMEL_PDC_TCR, list->txlen[0] / tx_size);
	at91_spi_write(ATMEL_PDC_RCR, list->rxlen[0] / tx_size);

	/* Is there a second transfer? */
	if (list->nr_transfers > 1) {
		device->txnext = dma_map_single(NULL, list->tx[1], list->txlen[1], DMA_TO_DEVICE);
		device->rxnext = dma_map_single(NULL, list->rx[1], list->rxlen[1], DMA_FROM_DEVICE);

		/* Program Next PDC registers */
		at91_spi_write(ATMEL_PDC_TNPR, device->txnext);
		at91_spi_write(ATMEL_PDC_RNPR, device->rxnext);
		at91_spi_write(ATMEL_PDC_TNCR, list->txlen[1] / tx_size);
		at91_spi_write(ATMEL_PDC_RNCR, list->rxlen[1] / tx_size);
	}
	else {
		device->txnext = 0;
		device->rxnext = 0;
		at91_spi_write(ATMEL_PDC_TNCR, 0);
		at91_spi_write(ATMEL_PDC_RNCR, 0);
	}

	// TODO: If we are doing consecutive transfers (at high speed, or
	//   small buffers), then it might be worth modifying the 'Delay between
	//   Consecutive Transfers' in the CSR registers.
	//   This is an issue if we cannot chain the next buffer fast enough
	//   in the interrupt handler.

	/* Enable transmitter and receiver */
	at91_spi_write(ATMEL_PDC_PTCR, ATMEL_PDC_RXTEN | ATMEL_PDC_TXTEN);

	at91_spi_write(AT91_SPI_IER, AT91_SPI_ENDRX);		/* enable buffer complete interrupt */
	wait_for_completion(&transfer_complete);

#ifdef DEBUG_SPI
	printk("SPI transfer end\n");
#endif

	return 0;
}

/* ......................................................................... */

/*
 * Handle interrupts from the SPI controller.
 */
static irqreturn_t at91spi_interrupt(int irq, void *dev_id)
{
	unsigned int status;
	struct spi_local *device = (struct spi_local *) &spi_dev[current_device];
	struct spi_transfer_list *list = device->xfers;

#ifdef DEBUG_SPI
	printk("SPI interrupt %i\n", current_device);
#endif

	if (!list)
		panic("at91_spi: spi_interrupt with a NULL transfer list");

		status = at91_spi_read(AT91_SPI_SR) & at91_spi_read(AT91_SPI_IMR);	/* read status */

	dma_unmap_single(NULL, device->tx, list->txlen[list->curr], DMA_TO_DEVICE);
	dma_unmap_single(NULL, device->rx, list->rxlen[list->curr], DMA_FROM_DEVICE);

	device->tx = device->txnext;	/* move next transfer to current transfer */
	device->rx = device->rxnext;

	list->curr = list->curr + 1;
	if (list->curr == list->nr_transfers) {		/* all transfers complete */
		at91_spi_write(AT91_SPI_IDR, AT91_SPI_ENDRX);		/* disable interrupt */

		/* Disable transmitter and receiver */
		at91_spi_write(ATMEL_PDC_PTCR, ATMEL_PDC_RXTDIS | ATMEL_PDC_TXTDIS);

		device->xfers = NULL;
		complete(&transfer_complete);
	}
	else if (list->curr+1 == list->nr_transfers) {	/* no more next transfers */
		device->txnext = 0;
		device->rxnext = 0;
		at91_spi_write(ATMEL_PDC_TNCR, 0);
		at91_spi_write(ATMEL_PDC_RNCR, 0);
	}
	else {
		int i = (list->curr)+1;

		/* If we are in 16-bit mode, we need to modify what we pass to the PDC */
		int tx_size = (at91_spi_read(AT91_SPI_CSR(current_device)) & AT91_SPI_BITS_16) ? 2 : 1;

		device->txnext = dma_map_single(NULL, list->tx[i], list->txlen[i], DMA_TO_DEVICE);
		device->rxnext = dma_map_single(NULL, list->rx[i], list->rxlen[i], DMA_FROM_DEVICE);
		at91_spi_write(ATMEL_PDC_TNPR, device->txnext);
		at91_spi_write(ATMEL_PDC_RNPR, device->rxnext);
		at91_spi_write(ATMEL_PDC_TNCR, list->txlen[i] / tx_size);
		at91_spi_write(ATMEL_PDC_RNCR, list->rxlen[i] / tx_size);
	}
	return IRQ_HANDLED;
}

/* ......................................................................... */

/*
 * Initialize the SPI controller
 */
static int __init at91spi_probe(struct platform_device *pdev)
{
	int i;
	unsigned long scbr;
	struct resource *res;

	init_MUTEX(&spi_lock);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENXIO;

	if (!request_mem_region(res->start, res->end - res->start + 1, "at91_spi"))
		return -EBUSY;

	spi_base = ioremap(res->start, res->end - res->start + 1);
	if (!spi_base) {
		release_mem_region(res->start, res->end - res->start + 1);
		return -ENOMEM;
	}

	spi_clk = clk_get(NULL, "spi_clk");
	if (IS_ERR(spi_clk)) {
		printk(KERN_ERR "at91_spi: no clock defined\n");
		iounmap(spi_base);
		release_mem_region(res->start, res->end - res->start + 1);
		return -ENODEV;
	}

	at91_spi_write(AT91_SPI_CR, AT91_SPI_SWRST);	/* software reset of SPI controller */

	/*
	 * Calculate the correct SPI baud-rate divisor.
	 */
	scbr = clk_get_rate(spi_clk) / (2 * DEFAULT_SPI_CLK);
	scbr = scbr + 1;		/* round up */

	printk(KERN_INFO "at91_spi: Baud rate set to %ld\n", clk_get_rate(spi_clk) / (2 * scbr));

	/* Set Chip Select registers to good defaults */
	for (i = 0; i < 4; i++) {
		at91_spi_write(AT91_SPI_CSR(i), AT91_SPI_CPOL | AT91_SPI_BITS_8 | (16 << 16) | (scbr << 8));
	}

	at91_spi_write(ATMEL_PDC_PTCR, ATMEL_PDC_RXTDIS | ATMEL_PDC_TXTDIS);

	memset(&spi_dev, 0, sizeof(spi_dev));
	spi_dev[0].pcs = 0xE;
	spi_dev[1].pcs = 0xD;
	spi_dev[2].pcs = 0xB;
	spi_dev[3].pcs = 0x7;

	if (request_irq(AT91RM9200_ID_SPI, at91spi_interrupt, 0, "spi", NULL)) {
		clk_put(spi_clk);
		iounmap(spi_base);
		release_mem_region(res->start, res->end - res->start + 1);
		return -EBUSY;
	}

	at91_spi_write(AT91_SPI_CR, AT91_SPI_SPIEN);		/* Enable SPI */

	return 0;
}

static int __devexit at91spi_remove(struct platform_device *pdev)
{
	struct resource *res;

	at91_spi_write(AT91_SPI_CR, AT91_SPI_SPIDIS);		/* Disable SPI */
	clk_put(spi_clk);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	iounmap(spi_base);
	release_mem_region(res->start, res->end - res->start + 1);

	free_irq(AT91RM9200_ID_SPI, 0);
	return 0;
}

static struct platform_driver at91spi_driver = {
	.probe		= at91spi_probe,
	.remove		= __devexit_p(at91spi_remove),
	.driver		= {
		.name	= "at91_spi",
		.owner	= THIS_MODULE,
	},
};

static int __init at91spi_init(void)
{
	return platform_driver_register(&at91spi_driver);
}

static void __exit at91spi_exit(void)
{
	platform_driver_unregister(&at91spi_driver);
}

EXPORT_SYMBOL(spi_access_bus);
EXPORT_SYMBOL(spi_release_bus);
EXPORT_SYMBOL(spi_transfer);

module_init(at91spi_init);
module_exit(at91spi_exit);

MODULE_LICENSE("GPL")
MODULE_AUTHOR("Andrew Victor")
MODULE_DESCRIPTION("SPI driver for Atmel AT91RM9200")
