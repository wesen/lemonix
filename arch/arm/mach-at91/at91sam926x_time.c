/*
 * linux/arch/arm/mach-at91/at91sam926x_time.c
 *
 * Copyright (C) 2005-2006 M. Amine SAYA, ATMEL Rousset, France
 * Revision	 2005 M. Nicolas Diremdjian, ATMEL Rousset, France
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/time.h>

#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/mach/time.h>

#include <asm/arch/at91_pit.h>
#include <asm/arch/at91_rtt.h>
#include <asm/arch/at91_tc.h>
#include <linux/clocksource.h>
#include <linux/clockchips.h>

static struct clock_event_device clockevent_at91sam926x;

static irqreturn_t at91sam926x_timer_interrupt(int irq, void *dev_id)
{
	if (at91_sys_read(AT91_PIT_SR) & AT91_PIT_PITS)
	{
		/* This is a shared interrupt */
		/* read PIVR to clear interrupt */
		at91_sys_read(AT91_PIT_PIVR);
		clockevent_at91sam926x.event_handler(&clockevent_at91sam926x);
		return IRQ_HANDLED;
	} else
		return IRQ_NONE;
}

static int at91sam926x_set_next_event(unsigned long evt,
				       struct clock_event_device *unused)
{
	at91_sys_write(AT91_PIT_MR, evt | AT91_PIT_PITIEN | AT91_PIT_PITEN);
	/*
	 * We disable PITEN immediately, because we don't want the PIT
	 * to be restarted automatically.
	 */
	at91_sys_write(AT91_PIT_MR, evt | AT91_PIT_PITIEN);
	return 0;
}

static void at91sam926x_set_mode(enum clock_event_mode mode,
				 struct clock_event_device *evt)
{
	unsigned long helper = 0;

	/* Disable timer */
	at91_sys_write(AT91_PIT_MR, 0);

	switch(mode) {
	case CLOCK_EVT_MODE_PERIODIC:
		printk ("CLOCK_EVT_MODE_PERIODIC\n");
		helper = (CLOCK_TICK_RATE / HZ) - 1;
		printk ("AT91 timer period: %lu\n", helper);

		/* Set timer period  */
		at91_sys_write(AT91_PIT_MR,
			       (CLOCK_TICK_RATE/HZ - 1) |
			       AT91_PIT_PITIEN | AT91_PIT_PITEN);
		break;
	case CLOCK_EVT_MODE_ONESHOT:
		printk ("CLOCK_EVT_MODE_ONESHOT\n");
		break;
	case CLOCK_EVT_MODE_SHUTDOWN:
		printk ("CLOCK_EVT_MODE_SHUTDOWN\n");
		break;
	case CLOCK_EVT_MODE_UNUSED:
		printk ("CLOCK_EVT_MODE_UNUSED\n");
		break;
	}
}

static struct clock_event_device clockevent_at91sam926x = {
	.name           = "at91sam926x-PIT",
	.features	= CLOCK_EVT_FEAT_PERIODIC | CLOCK_EVT_FEAT_ONESHOT,
	.shift          = 32,
	.set_mode       = at91sam926x_set_mode,
	.set_next_event = at91sam926x_set_next_event,
};

static struct irqaction at91sam926x_timer_irq = {
	.name		= "at91sam926x-PIT",
	.flags		= IRQF_SHARED | IRQF_DISABLED | IRQF_TIMER,
	.handler	= at91sam926x_timer_interrupt
};

void __init at91sam926x_clockevent_init(void)
{
	/* Reset time-stamp counter */
	at91_sys_write(AT91_PIT_MR, 0);
	at91_sys_read(AT91_PIT_PIVR);

	/* Make IRQs happen for the system timer. */
	setup_irq(AT91_ID_SYS, &at91sam926x_timer_irq);

	clockevent_at91sam926x.mult = div_sc(CLOCK_TICK_RATE, NSEC_PER_SEC,
					     clockevent_at91sam926x.shift);
	clockevent_at91sam926x.max_delta_ns =
		clockevent_delta2ns(0xffffe, &clockevent_at91sam926x);
	clockevent_at91sam926x.min_delta_ns =
		clockevent_delta2ns(0xf, &clockevent_at91sam926x);
	clockevent_at91sam926x.cpumask = cpumask_of_cpu(0);
	clockevents_register_device(&clockevent_at91sam926x);
}

cycle_t at91sam926x_get_cycles(void)
{
	/*
	 * RTT_VR can be updated asynchronously from the Master Clock
	 * so we read it twice at the same value for better accuracy
	 *
	 */
	unsigned long rtt_vr1, rtt_vr2;
	rtt_vr1 = at91_sys_read(AT91_RTT_VR);
	for(;;) {
		rtt_vr2 = at91_sys_read(AT91_RTT_VR);
		if (rtt_vr1 == rtt_vr2)
			break;
		rtt_vr1 = rtt_vr2;
	}
	return rtt_vr1;
}

static struct clocksource clocksource_at91sam926x = {
	.name           = "at91sam926x-RTT",
	.rating         = 200,
	.read           = at91sam926x_get_cycles,
	.mask           = CLOCKSOURCE_MASK(32),
	.shift          = 17,
	.flags          = CLOCK_SOURCE_IS_CONTINUOUS,
};

static void __init at91sam926x_clocksource_init(void)
{
	/* Set RTT freq = SLOW_CLOCK */
	at91_sys_write(AT91_RTT_MR, AT91_RTT_RTTRST | 1);

	clocksource_at91sam926x.mult =
		clocksource_hz2mult(AT91_SLOW_CLOCK,
					clocksource_at91sam926x.shift);
	clocksource_register(&clocksource_at91sam926x);
}

static void __init at91sam926x_timer_init(void)
{
	at91sam926x_clocksource_init();
	at91sam926x_clockevent_init();
}

#ifdef CONFIG_PM
static void at91sam926x_timer_suspend(void)
{
	/* Disable timer */
	at91_sys_write(AT91_PIT_MR, 0);
}
#else
#define at91sam926x_timer_suspend	NULL
#endif

struct sys_timer at91sam926x_timer = {
	.init		= at91sam926x_timer_init,
#if 0
	.suspend	= at91sam926x_timer_suspend,
	.resume		= at91sam926x_timer_reset,
#endif
};

