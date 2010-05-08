#include <linux/clocksource.h>
#include <linux/workqueue.h>
#include <linux/cpufreq.h>
#include <linux/jiffies.h>
#include <linux/init.h>
#include <linux/dmi.h>
#include <linux/acpi_pmtmr.h>
#include <linux/paravirt.h>

#include <asm/delay.h>
#include <asm/tsc.h>
#include <asm/io.h>

#include "mach_timer.h"

#include <linux/kvm_para.h>
#include <asm/paravirt.h>
#include <asm/hypercall.h>

static cycle_t read_hyper(void)
{
	struct timespec now;
	int ret;

	ret = hypercall(1, __NR_hypercall_get_ktime, __pa(&now));
	WARN_ON(ret);

	return now.tv_nsec + now.tv_sec * (cycles_t)1e9;
}

static struct clocksource clocksource_hyper = {
	.name			= "hyper",
	.rating			= 200,
	.read			= read_hyper,
	.mask			= CLOCKSOURCE_MASK(64),
	.mult			= 1,
	.shift			= 0,
	.flags			= CLOCK_SOURCE_IS_CONTINUOUS,
};

static int __init init_hyper_clocksource(void)
{
	if (!paravirt_enabled())
		return 0;

	return clocksource_register(&clocksource_hyper);
}

module_init(init_hyper_clocksource);
