/*
 * linux/arch/arm/mach-at91/board-dk.c
 *
 *  Copyright (C) 2005 SAN People
 *
 *  Epson S1D framebuffer glue code is:
 *     Copyright (C) 2005 Thibaut VARENE <varenet@parisc-linux.org>
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
#include <linux/mtd/physmap.h>

#include <asm/hardware.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/irq.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <asm/arch/board.h>
#include <asm/arch/gpio.h>
#include <asm/arch/at91rm9200_mc.h>

#include "generic.h"


/*
 * Serial port configuration.
 *    0 .. 3 = USART0 .. USART3
 *    4      = DBGU
 */
static struct at91_uart_config __initdata dk_uart_config = {
	.console_tty	= 0,				/* ttyS0 */
	.nr_tty		= 2,
	.tty_map	= { 4, 1, -1, -1, -1 }		/* ttyS0, ..., ttyS4 */
};

static void __init dk_map_io(void)
{
	/* Initialize processor: 18.432 MHz crystal */
	at91rm9200_initialize(18432000, AT91RM9200_BGA);

	/* Setup the LEDs */
	at91_init_leds(AT91_PIN_PB2, AT91_PIN_PB2);

	/* Setup the serial ports and console */
	at91_init_serial(&dk_uart_config);
}

static void __init dk_init_irq(void)
{
	at91rm9200_init_interrupts(NULL);
}

#if defined(CONFIG_FB_S1D13XXX) || defined(CONFIG_FB_S1D13XXX_MODULE)
#include <video/s1d13xxxfb.h>
#include <asm/arch/ics1523.h>

/* EPSON S1D13806 FB */
#define AT91_FB_REG_BASE	0x30000000L
#define AT91_FB_REG_SIZE	0x200
#define AT91_FB_VMEM_BASE	0x30200000L
#define AT91_FB_VMEM_SIZE	0x140000L

static void __init dk_init_video(void)
{
	/* NWAIT Signal */
	at91_set_A_periph(AT91_PIN_PC6, 0);

	/* Initialization of the Static Memory Controller for Chip Select 2 */
	at91_sys_write(AT91_SMC_CSR(2), AT91_SMC_DBW_16			/* 16 bit */
				| AT91_SMC_WSEN | AT91_SMC_NWS_(4)	/* wait states */
				| AT91_SMC_TDF_(1)			/* float time */
	);

	at91_ics1523_init();
}

/* CRT:    (active)   640x480 60Hz (PCLK=CLKI=25.175MHz)
   Memory: Embedded SDRAM (MCLK=CLKI3=50.000MHz) (BUSCLK=60.000MHz) */
static const struct s1d13xxxfb_regval dk_s1dfb_initregs[] = {
	{S1DREG_MISC,			0x00},	/* Enable Memory/Register select bit */
	{S1DREG_COM_DISP_MODE,		0x00},	/* disable display output */
	{S1DREG_GPIO_CNF0,		0x00},
	{S1DREG_GPIO_CNF1,		0x00},
	{S1DREG_GPIO_CTL0,		0x08},
	{S1DREG_GPIO_CTL1,		0x00},
	{S1DREG_CLK_CNF,		0x01},	/* no divide, MCLK source is CLKI3 0x02*/
	{S1DREG_LCD_CLK_CNF,		0x00},
	{S1DREG_CRT_CLK_CNF,		0x00},
	{S1DREG_MPLUG_CLK_CNF,		0x00},
	{S1DREG_CPU2MEM_WST_SEL,	0x01},	/* 2*period(MCLK) - 4ns > period(BCLK) */
	{S1DREG_SDRAM_REF_RATE,		0x03},	/* 32768 <= MCLK <= 50000 (MHz) */
	{S1DREG_SDRAM_TC0,		0x00},	/* MCLK source freq (MHz): */
	{S1DREG_SDRAM_TC1,		0x01},	/* 42 <= MCLK <= 50 */
	{S1DREG_MEM_CNF,		0x80},	/* SDRAM Initialization - needed before mem access */
	{S1DREG_PANEL_TYPE,		0x25},	/* std TFT 16bit, 8bit SCP format 2, single passive LCD */
	{S1DREG_MOD_RATE,		0x00},	/* toggle every FPFRAME */
	{S1DREG_LCD_DISP_HWIDTH,	0x4F},	/* 680 pix */
	{S1DREG_LCD_NDISP_HPER,		0x12},	/* 152 pix */
	{S1DREG_TFT_FPLINE_START,	0x01},	/* 13 pix */
	{S1DREG_TFT_FPLINE_PWIDTH,	0x0B},	/* 96 pix */
	{S1DREG_LCD_DISP_VHEIGHT0,	0xDF},
	{S1DREG_LCD_DISP_VHEIGHT1,	0x01},	/* 480 lines */
	{S1DREG_LCD_NDISP_VPER,		0x2C},	/* 44 lines */
	{S1DREG_TFT_FPFRAME_START,	0x0A},	/* 10 lines */
	{S1DREG_TFT_FPFRAME_PWIDTH,	0x01},	/* 2 lines */
	{S1DREG_LCD_DISP_MODE,		0x05},  /* 16 bpp */
	{S1DREG_LCD_MISC,		0x00},	/* dithering enabled, dual panel buffer enabled */
	{S1DREG_LCD_DISP_START0,	0x00},
	{S1DREG_LCD_DISP_START1,	0xC8},
	{S1DREG_LCD_DISP_START2,	0x00},
	{S1DREG_LCD_MEM_OFF0,		0x80},
	{S1DREG_LCD_MEM_OFF1,		0x02},
	{S1DREG_LCD_PIX_PAN,		0x00},
	{S1DREG_LCD_DISP_FIFO_HTC,	0x3B},
	{S1DREG_LCD_DISP_FIFO_LTC,	0x3C},
	{S1DREG_CRT_DISP_HWIDTH,	0x4F},	/* 680 pix */
	{S1DREG_CRT_NDISP_HPER,		0x13},	/* 160 pix */
	{S1DREG_CRT_HRTC_START,		0x01},	/* 13 pix */
	{S1DREG_CRT_HRTC_PWIDTH,	0x0B},	/* 96 pix */
	{S1DREG_CRT_DISP_VHEIGHT0,	0xDF},
	{S1DREG_CRT_DISP_VHEIGHT1,	0x01},	/* 480 lines */
	{S1DREG_CRT_NDISP_VPER,		0x2B},	/* 44 lines */
	{S1DREG_CRT_VRTC_START,		0x09},	/* 10 lines */
	{S1DREG_CRT_VRTC_PWIDTH,	0x01},	/* 2 lines */
	{S1DREG_TV_OUT_CTL,		0x10},
	{S1DREG_CRT_DISP_MODE,		0x05},	/* 16 bpp */
	{S1DREG_CRT_DISP_START0,	0x00},
	{S1DREG_CRT_DISP_START1,	0x00},
	{S1DREG_CRT_DISP_START2,	0x00},
	{S1DREG_CRT_MEM_OFF0,		0x80},
	{S1DREG_CRT_MEM_OFF1,		0x02},
	{S1DREG_CRT_PIX_PAN,		0x00},
	{S1DREG_CRT_DISP_FIFO_HTC,	0x3B},
	{S1DREG_CRT_DISP_FIFO_LTC,	0x3C},
	{S1DREG_LCD_CUR_CTL,		0x00},	/* inactive */
	{S1DREG_LCD_CUR_START,		0x01},
	{S1DREG_LCD_CUR_XPOS0,		0x00},
	{S1DREG_LCD_CUR_XPOS1,		0x00},
	{S1DREG_LCD_CUR_YPOS0,		0x00},
	{S1DREG_LCD_CUR_YPOS1,		0x00},
	{S1DREG_LCD_CUR_BCTL0,		0x00},
	{S1DREG_LCD_CUR_GCTL0,		0x00},
	{S1DREG_LCD_CUR_RCTL0,		0x00},
	{S1DREG_LCD_CUR_BCTL1,		0x1F},
	{S1DREG_LCD_CUR_GCTL1,		0x3F},
	{S1DREG_LCD_CUR_RCTL1,		0x1F},
	{S1DREG_LCD_CUR_FIFO_HTC,	0x00},
	{S1DREG_CRT_CUR_CTL,		0x00},	/* inactive */
	{S1DREG_CRT_CUR_START,		0x01},
	{S1DREG_CRT_CUR_XPOS0,		0x00},
	{S1DREG_CRT_CUR_XPOS1,		0x00},
	{S1DREG_CRT_CUR_YPOS0,		0x00},
	{S1DREG_CRT_CUR_YPOS1,		0x00},
	{S1DREG_CRT_CUR_BCTL0,		0x00},
	{S1DREG_CRT_CUR_GCTL0,		0x00},
	{S1DREG_CRT_CUR_RCTL0,		0x00},
	{S1DREG_CRT_CUR_BCTL1,		0x1F},
	{S1DREG_CRT_CUR_GCTL1,		0x3F},
	{S1DREG_CRT_CUR_RCTL1,		0x1F},
	{S1DREG_CRT_CUR_FIFO_HTC,	0x00},
	{S1DREG_BBLT_CTL0,		0x00},
	{S1DREG_BBLT_CTL0,		0x00},
	{S1DREG_BBLT_CC_EXP,		0x00},
	{S1DREG_BBLT_OP,		0x00},
	{S1DREG_BBLT_SRC_START0,	0x00},
	{S1DREG_BBLT_SRC_START1,	0x00},
	{S1DREG_BBLT_SRC_START2,	0x00},
	{S1DREG_BBLT_DST_START0,	0x00},
	{S1DREG_BBLT_DST_START1,	0x00},
	{S1DREG_BBLT_DST_START2,	0x00},
	{S1DREG_BBLT_MEM_OFF0,		0x00},
	{S1DREG_BBLT_MEM_OFF1,		0x00},
	{S1DREG_BBLT_WIDTH0,		0x00},
	{S1DREG_BBLT_WIDTH1,		0x00},
	{S1DREG_BBLT_HEIGHT0,		0x00},
	{S1DREG_BBLT_HEIGHT1,		0x00},
	{S1DREG_BBLT_BGC0,		0x00},
	{S1DREG_BBLT_BGC1,		0x00},
	{S1DREG_BBLT_FGC0,		0x00},
	{S1DREG_BBLT_FGC1,		0x00},
	{S1DREG_LKUP_MODE,		0x00},	/* LCD LUT r | LCD and CRT/TV LUT w */
	{S1DREG_LKUP_ADDR,		0x00},
	{S1DREG_PS_CNF,			0x00},	/* Power Save disable */
	{S1DREG_PS_STATUS,		0x02},	/* LCD Panel down, mem up */
	{S1DREG_CPU2MEM_WDOGT,		0x00},
	{S1DREG_COM_DISP_MODE,		0x02},	/* enable CRT display output */
};

static struct s1d13xxxfb_pdata dk_s1dfb_pdata = {
	.initregs		= dk_s1dfb_initregs,
	.initregssize		= ARRAY_SIZE(dk_s1dfb_initregs),
	.platform_init_video	= dk_init_video,
};

static u64 s1dfb_dmamask = 0xffffffffUL;

static struct resource dk_s1dfb_resource[] = {
	[0] = {	/* video mem */
		.name   = "s1d13806 memory",
		.start  = AT91_FB_VMEM_BASE,
		.end    = AT91_FB_VMEM_BASE + AT91_FB_VMEM_SIZE -1,
		.flags  = IORESOURCE_MEM,
	},
	[1] = {	/* video registers */
		.name   = "s1d13806 registers",
		.start  = AT91_FB_REG_BASE,
		.end    = AT91_FB_REG_BASE + AT91_FB_REG_SIZE -1,
		.flags  = IORESOURCE_MEM,
	},
};

static struct platform_device dk_s1dfb_device = {
	.name		= "s1d13806fb",
	.id		= -1,
	.dev		= {
			.dma_mask		= &s1dfb_dmamask,
			.coherent_dma_mask	= 0xffffffff,
			.platform_data		= &dk_s1dfb_pdata,
	},
	.resource	= dk_s1dfb_resource,
	.num_resources	= ARRAY_SIZE(dk_s1dfb_resource),
};

static void __init dk_add_device_video(void)
{
	platform_device_register(&dk_s1dfb_device);
}
#else
static void __init dk_add_device_video(void) {}
#endif

static struct at91_eth_data __initdata dk_eth_data = {
	.phy_irq_pin	= AT91_PIN_PC4,
	.is_rmii	= 1,
};

static struct at91_usbh_data __initdata dk_usbh_data = {
	.ports		= 2,
};

static struct at91_udc_data __initdata dk_udc_data = {
	.vbus_pin	= AT91_PIN_PD4,
	.pullup_pin	= AT91_PIN_PD5,
};

static struct at91_cf_data __initdata dk_cf_data = {
	.det_pin	= AT91_PIN_PB0,
	.rst_pin	= AT91_PIN_PC5,
	// .irq_pin	= ... not connected
	// .vcc_pin	= ... always powered
};

static struct at91_mmc_data __initdata dk_mmc_data = {
	.slot_b		= 0,
	.wire4		= 1,
};

static struct spi_board_info dk_spi_devices[] = {
	{	/* DataFlash chip */
		.modalias	= "mtd_dataflash",
		.chip_select	= 0,
		.max_speed_hz	= 15 * 1000 * 1000,
	},
	{	/* UR6HCPS2-SP40 PS2-to-SPI adapter */
		.modalias	= "ur6hcps2",
		.chip_select	= 1,
		.max_speed_hz	= 250 *  1000,
	},
	{	/* TLV1504 ADC, 4 channels, 10 bits; one is a temp sensor */
		.modalias	= "tlv1504",
		.chip_select	= 2,
		.max_speed_hz	= 20 * 1000 * 1000,
	},
#ifdef CONFIG_MTD_AT91_DATAFLASH_CARD
	{	/* DataFlash card */
		.modalias	= "mtd_dataflash",
		.chip_select	= 3,
		.max_speed_hz	= 15 * 1000 * 1000,
	}
#endif
};

static struct mtd_partition __initdata dk_nand_partition[] = {
	{
		.name	= "NAND Partition 1",
		.offset	= 0,
		.size	= MTDPART_SIZ_FULL,
	},
};

static struct mtd_partition *nand_partitions(int size, int *num_partitions)
{
	*num_partitions = ARRAY_SIZE(dk_nand_partition);
	return dk_nand_partition;
}

static struct at91_nand_data __initdata dk_nand_data = {
	.ale		= 22,
	.cle		= 21,
	.det_pin	= AT91_PIN_PB1,
	.rdy_pin	= AT91_PIN_PC2,
	// .enable_pin	= ... not there
	.partition_info	= nand_partitions,
};

#define DK_FLASH_BASE	AT91_CHIPSELECT_0
#define DK_FLASH_SIZE	0x200000

static struct physmap_flash_data dk_flash_data = {
	.width		= 2,
};

static struct resource dk_flash_resource = {
	.start		= DK_FLASH_BASE,
	.end		= DK_FLASH_BASE + DK_FLASH_SIZE - 1,
	.flags		= IORESOURCE_MEM,
};

static struct platform_device dk_flash = {
	.name		= "physmap-flash",
	.id		= 0,
	.dev		= {
				.platform_data	= &dk_flash_data,
			},
	.resource	= &dk_flash_resource,
	.num_resources	= 1,
};

static struct at91_gpio_led dk_leds[] = {
	{
		.name		= "led0",
		.gpio		= AT91_PIN_PB2,
		.trigger	= "timer",
	}
};

static void __init dk_board_init(void)
{
	/* Serial */
	at91_add_device_serial();
	/* Ethernet */
	at91_add_device_eth(&dk_eth_data);
	/* USB Host */
	at91_add_device_usbh(&dk_usbh_data);
	/* USB Device */
	at91_add_device_udc(&dk_udc_data);
	at91_set_multi_drive(dk_udc_data.pullup_pin, 1);	/* pullup_pin is connected to reset */
	/* Compact Flash */
	at91_add_device_cf(&dk_cf_data);
	/* I2C */
	at91_add_device_i2c();
	/* SPI */
	at91_add_device_spi(dk_spi_devices, ARRAY_SIZE(dk_spi_devices));
#ifdef CONFIG_MTD_AT91_DATAFLASH_CARD
	/* DataFlash card */
	at91_set_gpio_output(AT91_PIN_PB7, 0);
#else
	/* MMC */
	at91_set_gpio_output(AT91_PIN_PB7, 1);	/* this MMC card slot can optionally use SPI signaling (CS3). */
	at91_add_device_mmc(0, &dk_mmc_data);
#endif
	/* NAND */
	at91_add_device_nand(&dk_nand_data);
	/* NOR Flash */
	platform_device_register(&dk_flash);
	/* LEDs */
	at91_gpio_leds(dk_leds, ARRAY_SIZE(dk_leds));
	/* VGA */
	dk_add_device_video();
}

MACHINE_START(AT91RM9200DK, "Atmel AT91RM9200-DK")
	/* Maintainer: SAN People/Atmel */
	.phys_io	= AT91_BASE_SYS,
	.io_pg_offst	= (AT91_VA_BASE_SYS >> 18) & 0xfffc,
	.boot_params	= AT91_SDRAM_BASE + 0x100,
	.timer		= &at91rm9200_timer,
	.map_io		= dk_map_io,
	.init_irq	= dk_init_irq,
	.init_machine	= dk_board_init,
MACHINE_END
