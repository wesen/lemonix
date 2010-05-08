/*
 * linux/arch/arm/mach-at91/board-sam9260ek.c
 *
 *  Copyright (C) 2005 SAN People
 *  Copyright (C) 2006 Atmel
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/types.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>

#include <asm/hardware.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/irq.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <asm/arch/board.h>
#include <asm/arch/gpio.h>
#include <asm/arch/at91sam926x_mc.h>

#include <linux/spi/eeprom.h>

#include "generic.h"


/*
 * Serial port configuration.
 *    0 .. 5 = USART0 .. USART5
 *    6      = DBGU
 */
static struct at91_uart_config __initdata ek_uart_config = {
	.console_tty	= 0,				/* ttyS0 */
	.nr_tty		= 5,
	.tty_map	= { 6, 0, 1, 2, 3, -1, -1 }	/* ttyS0, ..., ttyS6 */
};

static void __init ek_map_io(void)
{
	/* Initialize processor: 18.432 MHz crystal */
	at91sam9260_initialize(18432000);

	/* Setup the serial ports and console */
	at91_init_serial(&ek_uart_config);
}

static void __init ek_init_irq(void)
{
	at91sam9260_init_interrupts(NULL);
}


/*
 * USB Host port
 */
static struct at91_usbh_data __initdata ek_usbh_data = {
	.ports		= 2,
};

/*
 * USB Device port
 */
static struct at91_udc_data __initdata ek_udc_data = {
	.vbus_pin	= AT91_PIN_PC5,
	.pullup_pin	= 0,		/* pull-up driven by UDC */
};

//csung
#define AT250X0_PAGE_SIZE	32

static struct spi_eeprom at25_data = {
	.name = "at25160a",
	.byte_len = 2048, 
	.page_size = AT250X0_PAGE_SIZE,
	.flags = EE_ADDR2,
};
//<--csung


/*
 * SPI devices.
 */
	static struct spi_board_info ek_spi_devices[] = {
#if !defined(CONFIG_MMC_AT91)
		{	/* DataFlash chip */
			.modalias	= "mtd_dataflash",
			.chip_select	= 1,
			.max_speed_hz	= 15 * 1000 * 1000,
			.bus_num	= 0,
		},
#if defined(CONFIG_MTD_AT91_DATAFLASH_CARD)
		{	/* DataFlash card */
			.modalias	= "mtd_dataflash",
			.chip_select	= 0,
			.max_speed_hz	= 15 * 1000 * 1000,
			.bus_num	= 0,
		},
#endif
#endif
#if defined(CONFIG_SND_AT73C213) || defined(CONFIG_SND_AT73C213_MODULE)
		{	/* AT73C213 DAC */
			.modalias	= "at73c213",
			.chip_select	= 0,
			.max_speed_hz	= 10 * 1000 * 1000,
			.bus_num	= 1,
		},
#endif

		//csung
#if defined(CONFIG_SPI_AT25) || defined(CONFIG_SPI_AT25_MODULE)
		{
			.modalias	= "at25",
			.chip_select	= 0,
			.max_speed_hz   = 5000000,
			.bus_num	= 1,
			.mode      = SPI_MODE_0, 
			.platform_data	= &at25_data,
		},
#endif
		//<--csung
	};


/*
 * MACB Ethernet device
 */
static struct at91_eth_data __initdata ek_macb_data = {
	.phy_irq_pin	= AT91_PIN_PA7,
#ifdef CKLEE_SB
	.is_rmii  = 1,
#else
	.is_rmii  = 0,
#endif
};


/*
 * NAND flash
 */
#if 0 //nand partition original code
static struct mtd_partition __initdata ek_nand_partition[] = {
	{
		.name	= "Partition 1",
		.offset	= 0,
		.size	= SZ_256K,
	},
	{
		.name	= "Partition 2",
		.offset	= MTDPART_OFS_NXTBLK,
		.size	= MTDPART_SIZ_FULL,
	},
};
#endif

static struct mtd_partition __initdata ek_nand_partition[] = {
	{
		.name	= "Nand Flash",
		.offset	= 0,
		.size	= MTDPART_SIZ_FULL,
	},
};


static struct mtd_partition *nand_partitions(int size, int *num_partitions)
{
	*num_partitions = ARRAY_SIZE(ek_nand_partition);
	return ek_nand_partition;
}

static struct at91_nand_data __initdata ek_nand_data = {
	.ale		= 21,
	.cle		= 22,
	//	.det_pin	= ... not connected
	.rdy_pin	= AT91_PIN_PC17,
	.enable_pin	= AT91_PIN_PC14,
	.partition_info	= nand_partitions,
	.bus_width_16	= 0,
};


/*
 * MCI (SD/MMC)
 */
static struct at91_mmc_data __initdata ek_mmc_data = {
	.slot_b		= 1,
	.wire4		= 1,
	//	.det_pin	= ... not connected
	//	.wp_pin		= ... not connected
	//	.vcc_pin	= ... not connected
};

static void __init ek_board_init(void)
{
	/* Serial */
	at91_add_device_serial();
	/* USB Host */
	at91_add_device_usbh(&ek_usbh_data);
	/* USB Device */
	at91_add_device_udc(&ek_udc_data);
	/* SPI */
	at91_add_device_spi(ek_spi_devices, ARRAY_SIZE(ek_spi_devices));
	/* Ethernet */
	at91_add_device_eth(&ek_macb_data);
	/* MMC */
	at91_add_device_mmc(0, &ek_mmc_data);
	/* ADC */
	//	at91_add_device_adc();
	/* I2C */
	at91_add_device_i2c();
	/* RTT */
	at91_add_device_rtt();
	/* watchdog */
	at91_add_device_watchdog();

	/* NAND */
	at91_add_device_nand(&ek_nand_data);

	/* GPIO */
	at91_add_device_gpio();

#ifndef CONFIG_INNOPIATECH	/* CHWKIM - 2007-11-05 */
	at91_adj_bus_matrix_pri();
#endif
}

MACHINE_START(AT91SAM9260EK, "Atmel AT91SAM9260-EK")
/* Maintainer: Atmel */
.phys_io	= AT91_BASE_SYS,
	.io_pg_offst	= (AT91_VA_BASE_SYS >> 18) & 0xfffc,
	.boot_params	= AT91_SDRAM_BASE + 0x100,
	.timer		= &at91sam926x_timer,
	.map_io		= ek_map_io,
	.init_irq	= ek_init_irq,
	.init_machine	= ek_board_init,
	MACHINE_END
