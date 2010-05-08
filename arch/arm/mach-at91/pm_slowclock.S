/*
 * arch/arm/mach-at91/pm_slow_clock.S
 *
 *  Copyright (C) 2006 Savin Zlobec
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/linkage.h>
#include <asm/hardware.h>
#include <asm/arch/at91_pmc.h>
#include <asm/arch/at91rm9200_mc.h>

#define MCKRDY_TIMEOUT		1000
#define MOSCRDY_TIMEOUT 	1000
#define PLLALOCK_TIMEOUT	1000

	.macro wait_mckrdy
	mov	r2, #MCKRDY_TIMEOUT
1:	sub	r2, r2, #1
	cmp	r2, #0
	beq	2f
	ldr	r3, [r1, #AT91_PMC_SR]
	tst	r3, #AT91_PMC_MCKRDY
	beq	1b
2:
	.endm

	.macro wait_moscrdy
	mov	r2, #MOSCRDY_TIMEOUT
1:	sub	r2, r2, #1
	cmp	r2, #0
	beq	2f
	ldr	r3, [r1, #AT91_PMC_SR]
	tst	r3, #AT91_PMC_MOSCS
	beq	1b
2:
	.endm

	.macro wait_pllalock
	mov	r2, #PLLALOCK_TIMEOUT
1:	sub	r2, r2, #1
	cmp	r2, #0
	beq	2f
	ldr	r3, [r1, #AT91_PMC_SR]
	tst	r3, #AT91_PMC_LOCKA
	beq	1b
2:
	.endm

	.macro wait_plladis
	mov	r2, #PLLALOCK_TIMEOUT
1:	sub	r2, r2, #1
	cmp	r2, #0
	beq	2f
	ldr	r3, [r1, #AT91_PMC_SR]
	tst	r3, #AT91_PMC_LOCKA
	bne	1b
2:
	.endm

	.text

ENTRY(at91rm9200_slow_clock)

	ldr	r1, .at91_va_base_sys

	/* Put SDRAM in self refresh mode */

	b	1f
	.align	5
1:	mcr	p15, 0, r0, c7, c10, 4
	mov	r2, #1
	str	r2, [r1, #AT91_SDRAMC_SRR]

	/* Save Master clock setting */

	ldr	r2, [r1, #AT91_PMC_MCKR]
	str	r2, .saved_mckr

	/*
	 * Set the Master clock source to slow clock
	 *
	 * First set the CSS field, wait for MCKRDY
	 * and than set the PRES and MDIV fields.
	 *
	 * See eratta #2[78] for details.
	 */

	bic	r2, r2, #3
	str	r2, [r1, #AT91_PMC_MCKR]

	wait_mckrdy

	mov	r2, #0
	str	r2, [r1, #AT91_PMC_MCKR]

	/* Save PLLA setting and disable it */

	ldr	r2, [r1, #AT91_CKGR_PLLAR]
	str	r2, .saved_pllar

	mov	r2, #0
	str	r2, [r1, #AT91_CKGR_PLLAR]

	wait_plladis

	/* Turn off the main oscillator */

	ldr	r2, [r1, #AT91_CKGR_MOR]
	bic	r2, r2, #AT91_PMC_MOSCEN
	str	r2, [r1, #AT91_CKGR_MOR]

	/* Wait for interrupt */

	mcr	p15, 0, r0, c7, c0, 4

	/* Turn on the main oscillator */

	ldr	r2, [r1, #AT91_CKGR_MOR]
	orr	r2, r2, #AT91_PMC_MOSCEN
	str	r2, [r1, #AT91_CKGR_MOR]

	wait_moscrdy

	/* Restore PLLA setting */

	ldr	r2, .saved_pllar
	str	r2, [r1, #AT91_CKGR_PLLAR]

	wait_pllalock

	/*
	 * Restore master clock setting
	 *
	 * First set PRES if it was not 0,
	 * than set CSS and MDIV fields.
	 * After every change wait for
	 * MCKRDY.
	 *
	 * See eratta #2[78] for details.
	 */

	ldr	r2, .saved_mckr
	tst	r2, #0x1C
	beq	2f
	and	r2, r2, #0x1C
	str	r2, [r1, #AT91_PMC_MCKR]

	wait_mckrdy

2:	ldr	r2, .saved_mckr
	str	r2, [r1, #AT91_PMC_MCKR]

	wait_mckrdy

	mov	pc, lr

.saved_mckr:
	.word 0

.saved_pllar:
	.word 0

.at91_va_base_sys:
	.word AT91_VA_BASE_SYS

ENTRY(at91rm9200_slow_clock_sz)
	.word .-at91rm9200_slow_clock
