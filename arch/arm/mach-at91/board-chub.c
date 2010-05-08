/*
 * linux/arch/arm/mach-at91/board-chub.c
 *
 *  Copyright (C) 2005 SAN People, adapted for Promwad Chub board
 *  by Kuten Ivan
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

#include <asm/hardware.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/irq.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <asm/arch/board.h>
#include <asm/arch/gpio.h>

#include "generic.h"

/*
 * Serial port configuration.
 *    0 .. 3 = USART0 .. USART3
 *    4      = DBGU
 */
static struct at91_uart_config __initdata chub_uart_config = {
	.console_tty	= 0,				/* ttyS0 */
	.nr_tty		= 5,
	.tty_map	= { 4, 0, 1, 2, 3 }		/* ttyS0, ..., ttyS4 */
};

static void __init chub_init_irq(void)
{
	at91rm9200_init_interrupts(NULL);
}

static void __init chub_map_io(void)
{
	/* Initialize clocks: 18.432 MHz crystal */
	at91rm9200_initialize(18432000, AT91RM9200_PQFP);

	/* Setup the serial ports and console */
	at91_init_serial(&chub_uart_config);
}

static struct at91_eth_data __initdata chub_eth_data = {
	.phy_irq_pin	= AT91_PIN_PB29,
	.is_rmii	= 0,
};

static struct mtd_partition __initdata chub_nand_partition[] = {
	{
		.name	= "NAND Partition 1",
		.offset = 0,
		.size	= MTDPART_SIZ_FULL,
	},
};

static struct mtd_partition *nand_partitions(int size, int *num_partitions)
{
	*num_partitions = ARRAY_SIZE(chub_nand_partition);
	return chub_nand_partition;
}

static struct at91_nand_data __initdata chub_nand_data = {
	.ale		= 22,
	.cle		= 21,
	.enable_pin	= AT91_PIN_PA27,
	.partition_info	= nand_partitions,
};

static struct spi_board_info chub_spi_devices[] = {
	{	/* DataFlash chip */
		.modalias	= "mtd_dataflash",
		.chip_select	= 0,
		.max_speed_hz	= 15 * 1000 * 1000,
	},
};

static void __init chub_board_init(void)
{
	/* Serial */
	at91_add_device_serial();
	/* I2C */
	at91_add_device_i2c();
	/* Ethernet */
	at91_add_device_eth(&chub_eth_data);
	/* SPI */
	at91_add_device_spi(chub_spi_devices, ARRAY_SIZE(chub_spi_devices));
	/* NAND Flash */
	at91_add_device_nand(&chub_nand_data);
	/* Disable write protect for NAND */
	at91_set_gpio_output(AT91_PIN_PB7, 1);
	/* Power enable for 3x RS-232 and 1x RS-485 */
	at91_set_gpio_output(AT91_PIN_PB9, 1);
	/* Disable write protect for FRAM */
	at91_set_gpio_output(AT91_PIN_PA21, 1);
	/* Disable write protect for Dataflash */
	at91_set_gpio_output(AT91_PIN_PA19, 1);
}

MACHINE_START(CHUB, "Promwad Chub")
	/* Maintainer: Ivan Kuten AT Promwad DOT com */
	.phys_io	= AT91_BASE_SYS,
	.io_pg_offst	= (AT91_VA_BASE_SYS >> 18) & 0xfffc,
	.boot_params	= AT91_SDRAM_BASE + 0x100,
	.timer		= &at91rm9200_timer,
	.map_io		= chub_map_io,
	.init_irq	= chub_init_irq,
	.init_machine	= chub_board_init,
MACHINE_END
