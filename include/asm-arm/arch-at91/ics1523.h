//*----------------------------------------------------------------------------
//*         ATMEL Microcontroller Software Support  -  ROUSSET  -
//*----------------------------------------------------------------------------
//* The software is delivered "AS IS" without warranty or condition of any
//* kind, either express, implied or statutory. This includes without
//* limitation any warranty or condition with respect to merchantability or
//* fitness for any particular purpose, or against the infringements of
//* intellectual property rights of others.
//*----------------------------------------------------------------------------
//* File Name           : ics1523.h
//* Object              : Clock Generator Prototyping File.
//*
//* 1.0 08/28/02 ED     : Creation
//* 1.2 13/01/03 FB		: Update on lib V3
//*----------------------------------------------------------------------------

#ifndef ics1523_h
#define ics1523_h

/*-------------------------------------------*/
/* ICS1523 TWI Serial Clock Definition       */
/*-------------------------------------------*/

#define		ICS_MIN_CLOCK		100		/* Min Frequency Access Clock KHz */
#define		ICS_MAX_CLOCK		400		/* Max Frequency Access Clock KHz */
#define		ICS_TRANSFER_RATE	ICS_MAX_CLOCK	/* Transfer speed to apply */

#define		ICS_WRITE_CLK_PNB	30		/* TWCK Clock Periods required to write */
#define		ICS_READ_CLK_PNB	40		/* TWCK Clock Periods required to read */

/*-------------------------------------------*/
/* ICS1523 Write Operation Definition        */
/*-------------------------------------------*/

#define		ICS1523_ACCESS_OK	0		/* OK */
#define		ICS1523_ACCESS_ERROR	-1		/* NOK */

/*-------------------------------------------*/
/* ICS1523 Device Addresses Definition       */
/*-------------------------------------------*/

#define		ICS_ADDR		0x26		/* Device Address */

/*--------------------------------------------------*/
/* ICS1523 Registers Internal Addresses Definition  */
/*--------------------------------------------------*/

#define		ICS_ICR			0x0		/* Input Control Register */
#define		ICS_LCR			0x1		/* Loop Control Register */
#define		ICS_FD0			0x2		/* PLL FeedBack Divider LSBs */
#define		ICS_FD1			0x3		/* PLL FeedBack Divider MSBs */
#define		ICS_DPAO		0x4		/* Dynamic Phase Aligner Offset */
#define		ICS_DPAC		0x5		/* Dynamic Phase Aligner Resolution */
#define		ICS_OE			0x6		/* Output Enables Register */
#define		ICS_OD			0x7		/* Osc Divider Register */
#define		ICS_SWRST		0x8		/* DPA & PLL Reset Register */
#define		ICS_VID			0x10		/* Chip Version Register */
#define		ICS_RID			0x11		/* Chip Revision Register */
#define		ICS_SR			0x12		/* Status Register */

/*------------------------------------------------------*/
/* ICS1523 Input Control Register Bits Definition       */
/*------------------------------------------------------*/

#define		ICS_PDEN		0x1		/* Phase Detector Enable */
#define		ICS_PDPOL		0x2		/* Phase Detector Enable Polarity */
#define		ICS_REFPOL		0x4		/* External Reference Polarity */
#define		ICS_FBKPOL		0x8		/* External Feedback Polarity */
#define		ICS_FBKSEL		0x10		/* External Feedback Select */
#define		ICS_FUNCSEL		0x20		/* Function Out Select */
#define		ICS_ENPLS		0x40		/* Enable PLL Lock/Ref Status Output */
#define		ICS_ENDLS		0x80		/* Enable DPA Lock/Ref Status Output */

/*-----------------------------------------------------*/
/* ICS1523 Loop Control Register Bits Definition       */
/*-----------------------------------------------------*/

#define		ICS_PFD			0x7		/* Phase Detector Gain */
#define		ICS_PSD			0x30		/* Post-Scaler Divider */

/*----------------------------------------------------*/
/* ICS1523 PLL FeedBack Divider LSBs Definition       */
/*----------------------------------------------------*/

#define		ICS_FBDL		0xFF		/* PLL FeedBack Divider LSBs */

/*----------------------------------------------------*/
/* ICS1523 PLL FeedBack Divider MSBs Definition       */
/*----------------------------------------------------*/

#define		ICS_FBDM		0xF		/* PLL FeedBack Divider MSBs */

/*------------------------------------------------------------*/
/* ICS1523 Dynamic Phase Aligner Offset Bits Definition       */
/*------------------------------------------------------------*/

#define		ICS_DPAOS		0x2F		/* Dynamic Phase Aligner Offset */
#define		ICS_FILSEL		0x80		/* Loop Filter Select */

/*----------------------------------------------------------------*/
/* ICS1523 Dynamic Phase Aligner Resolution Bits Definition       */
/*----------------------------------------------------------------*/

#define		ICS_DPARES		0x3		/* Dynamic Phase Aligner Resolution */
#define		ICS_MMREV		0xFC		/* Metal Mask Revision Number */

/*-------------------------------------------------------*/
/* ICS1523 Output Enables Register Bits Definition       */
/*-------------------------------------------------------*/

#define		ICS_OEPCK		0x1		/* Output Enable for PECL PCLK Outputs */
#define		ICS_OETCK		0x2		/* Output Enable for STTL CLK Output */
#define		ICS_OEP2		0x4		/* Output Enable for PECL CLK/2 Outputs */
#define		ICS_OET2		0x8		/* Output Enable for STTL CLK/2 Output */
#define		ICS_OEF			0x10		/* Output Enable for STTL FUNC Output */
#define		ICS_CLK2INV		0x20		/* CLK/2 Invert */
#define		ICS_OSCL		0xC0		/* SSTL Clock Scaler */

/*----------------------------------------------------*/
/* ICS1523 Osc Divider Register Bits Definition       */
/*----------------------------------------------------*/

#define		ICS_OSCDIV		0x7F		/* Oscillator Divider Modulus */
#define		ICS_INSEL		0x80		/* Input Select */

/*---------------------------------------------------*/
/* ICS1523 DPA & PLL Reset Register Definition       */
/*---------------------------------------------------*/

#define		ICS_DPAR		0x0A		/* DPA Reset Command */
#define		ICS_PLLR		0x50		/* PLL Reset Command */

/*------------------------------------------------*/
/* ICS1523 Chip Version Register Definition       */
/*------------------------------------------------*/

#define		ICS_CHIPV		0xFF		/* Chip Version */

/*-------------------------------------------------*/
/* ICS1523 Chip Revision Register Definition       */
/*-------------------------------------------------*/

#define		ICS_CHIPR		0xFF		/* Chip Revision */

/*------------------------------------------*/
/* ICS1523 Status Register Definition       */
/*------------------------------------------*/

#define		ICS_DPALOCK		0x1		/* DPA Lock Status */
#define		ICS_PLLLOCK		0x2		/* PLL Lock Status */

int at91_ics1523_init(void);

#endif /* ics1523_h */
