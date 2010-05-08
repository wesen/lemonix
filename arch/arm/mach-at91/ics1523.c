/*
 * arch/arm/mach-at91rm9200/ics1523.c
 *
 *  Copyright (C) 2003 ATMEL Rousset
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

#include <asm/hardware.h>
#include <asm/io.h>

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/init.h>

#include <asm/arch/ics1523.h>
#include <asm/arch/at91_twi.h>
#include <asm/arch/gpio.h>

/* TWI Errors */
#define	AT91_TWI_ERROR	(AT91_TWI_NACK | AT91_TWI_UNRE | AT91_TWI_OVRE)


static void __iomem *twi_base;

#define at91_twi_read(reg)		__raw_readl(twi_base + (reg))
#define at91_twi_write(reg, val)	__raw_writel((val), twi_base + (reg))


/* -----------------------------------------------------------------------------
 * Initialization of TWI CLOCK
 * ----------------------------------------------------------------------------- */

static void at91_ics1523_SetTwiClock(unsigned int mck_khz)
{
	int sclock;

	/* Here, CKDIV = 1 and CHDIV = CLDIV  ==> CLDIV = CHDIV = 1/4*((Fmclk/FTWI) -6) */
	sclock = (10*mck_khz / ICS_TRANSFER_RATE);
	if (sclock % 10 >= 5)
		sclock = (sclock /10) - 5;
	else
		sclock = (sclock /10)- 6;
	sclock = (sclock + (4 - sclock %4)) >> 2;	/* div 4 */

	at91_twi_write(AT91_TWI_CWGR, 0x00010000 | sclock | (sclock << 8));
}

/* -----------------------------------------------------------------------------
 * Read a byte with TWI Interface from the Clock Generator ICS1523
 * ----------------------------------------------------------------------------- */

static int at91_ics1523_ReadByte(unsigned char reg_address, unsigned char *data_in)
{
	int Status, nb_trial;

	at91_twi_write(AT91_TWI_MMR, AT91_TWI_MREAD | AT91_TWI_IADRSZ_1 | ((ICS_ADDR << 16) & AT91_TWI_DADR));
	at91_twi_write(AT91_TWI_IADR, reg_address);
	at91_twi_write(AT91_TWI_CR, AT91_TWI_START | AT91_TWI_STOP);

	/* Program temporizing period (300us) */
	udelay(300);

	/* Wait TXcomplete ... */
	nb_trial = 0;
	Status = at91_twi_read(AT91_TWI_SR);
	while (!(Status & AT91_TWI_TXCOMP) && (nb_trial < 10)) {
		nb_trial++;
		Status = at91_twi_read(AT91_TWI_SR);
	}

	if (Status & AT91_TWI_TXCOMP) {
		*data_in = (unsigned char) at91_twi_read(AT91_TWI_RHR);
		return ICS1523_ACCESS_OK;
	}
	else
		return ICS1523_ACCESS_ERROR;
}

/* -----------------------------------------------------------------------------
 * Write a byte with TWI Interface to the Clock Generator ICS1523
 * ----------------------------------------------------------------------------- */

static int at91_ics1523_WriteByte(unsigned char reg_address, unsigned char data_out)
{
	int Status, nb_trial;

	at91_twi_write(AT91_TWI_MMR, AT91_TWI_IADRSZ_1 | ((ICS_ADDR << 16) & AT91_TWI_DADR));
	at91_twi_write(AT91_TWI_IADR, reg_address);
	at91_twi_write(AT91_TWI_THR, data_out);
	at91_twi_write(AT91_TWI_CR, AT91_TWI_START | AT91_TWI_STOP);

	/* Program temporizing period (300us) */
	udelay(300);

	nb_trial = 0;
	Status = at91_twi_read(AT91_TWI_SR);
	while (!(Status & AT91_TWI_TXCOMP) && (nb_trial < 10)) {
		nb_trial++;
		if (Status & AT91_TWI_ERROR) {
			/* If Underrun OR NACK - Start again */
			at91_twi_write(AT91_TWI_CR, AT91_TWI_START | AT91_TWI_STOP);

			/*  Program temporizing period (300us) */
			udelay(300);
		}
		Status = at91_twi_read(AT91_TWI_SR);
	};

	if (Status & AT91_TWI_TXCOMP)
		return ICS1523_ACCESS_OK;
	else
		return ICS1523_ACCESS_ERROR;
}

/* -----------------------------------------------------------------------------
 * Initialization of the Clock Generator ICS1523
 * ----------------------------------------------------------------------------- */

int at91_ics1523_init(void)
{
	int		nb_trial;
	int		ack = ICS1523_ACCESS_OK;
	unsigned int	status = 0xffffffff;
	struct clk	*twi_clk;

	/* Map in TWI peripheral */
	twi_base = ioremap(AT91RM9200_BASE_TWI, SZ_16K);
	if (!twi_base)
		return -ENOMEM;

	/* pins used for TWI interface */
	at91_set_A_periph(AT91_PIN_PA25, 0);            /* TWD */
	at91_set_multi_drive(AT91_PIN_PA25, 1);
	at91_set_A_periph(AT91_PIN_PA26, 0);            /* TWCK */
	at91_set_multi_drive(AT91_PIN_PA26, 1);

	/* Enable the TWI clock */
	twi_clk = clk_get(NULL, "twi_clk");
	if (IS_ERR(twi_clk))
		return ICS1523_ACCESS_ERROR;
	clk_enable(twi_clk);

	/* Disable interrupts */
	at91_twi_write(AT91_TWI_IDR, -1);

	/* Reset peripheral */
	at91_twi_write(AT91_TWI_CR, AT91_TWI_SWRST);

	/* Set Master mode */
	at91_twi_write(AT91_TWI_CR, AT91_TWI_MSEN);

	/* Set TWI Clock Waveform Generator Register */
	at91_ics1523_SetTwiClock(60000);     /* MCK in KHz = 60000 KHz */

	/* ICS1523 Initialisation */
	ack |= at91_ics1523_WriteByte ((unsigned char) ICS_ICR, (unsigned char) 0);
	ack |= at91_ics1523_WriteByte ((unsigned char) ICS_OE, (unsigned char) (ICS_OEF | ICS_OET2 | ICS_OETCK));
	ack |= at91_ics1523_WriteByte ((unsigned char) ICS_OD, (unsigned char) (ICS_INSEL | 0x7F));
	ack |= at91_ics1523_WriteByte ((unsigned char) ICS_DPAO, (unsigned char) 0);

	nb_trial = 0;
	do {
		nb_trial++;
		ack |= at91_ics1523_WriteByte ((unsigned char) ICS_ICR, (unsigned char) (ICS_ENDLS | ICS_ENPLS | ICS_PDEN /*| ICS_FUNCSEL*/));
		ack |= at91_ics1523_WriteByte ((unsigned char) ICS_LCR, (unsigned char) (ICS_PSD | ICS_PFD));
		ack |= at91_ics1523_WriteByte ((unsigned char) ICS_FD0, (unsigned char) 0x39) ; /* 0x7A */
		ack |= at91_ics1523_WriteByte ((unsigned char) ICS_FD1, (unsigned char) 0x00);
		ack |= at91_ics1523_WriteByte ((unsigned char) ICS_SWRST, (unsigned char) (ICS_PLLR));

		/* Program 1ms temporizing period */
		mdelay(1);

		at91_ics1523_ReadByte ((unsigned char) ICS_SR, (char *)&status);
	} while (!((unsigned int) status & (unsigned int) ICS_PLLLOCK) && (nb_trial < 10));

	ack |= at91_ics1523_WriteByte ((unsigned char) ICS_DPAC, (unsigned char) 0x03) ; /* 0x01 */
	ack |= at91_ics1523_WriteByte ((unsigned char) ICS_SWRST, (unsigned char) (ICS_DPAR));

	/* Program 1ms temporizing period */
	mdelay(1);

	ack |= at91_ics1523_WriteByte ((unsigned char) ICS_DPAO, (unsigned char) 0x00);

	/* Program 1ms temporizing period */
	mdelay(1);
	
	/* All done - cleanup */
	iounmap(twi_base);
	clk_disable(twi_clk);
	clk_put(twi_clk);

	return ack;
}
