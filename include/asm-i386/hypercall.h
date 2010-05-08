#ifndef __ASM_HYPERCALL_H
#define __ASM_HYPERCALL_H

#include <linux/hardirq.h>

#define CONFIG_PARAVIRT 1
#ifdef CONFIG_PARAVIRT

/*
 * Hypercalls, according to the calling convention
 * documented in include/linux/kvm_para.h
 *
 * Copyright (C) 2007, Red Hat, Inc., Ingo Molnar <mingo@redhat.com>
 * Copyright (C) 2007, Qumranet, Inc., Dor Laor <dor.laor@qumranet.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2.  See
 * the COPYING file in the top-level directory.
 */
static inline int __hypercall0(unsigned int nr)
{
	int ret;
	asm (" call hypercall_addr\n"
		: "=a" (ret)
		: "a" (nr)
		: "memory", "cc"
	);
	return ret;
}

static inline int __hypercall1(unsigned int nr, unsigned long p1)
{
	int ret;
	asm (" call hypercall_addr\n"
		: "=a" (ret)
		: "a" (nr),
		  "b" (p1)
		: "memory", "cc"
	);
	return ret;
}

static inline int
__hypercall2(unsigned int nr, unsigned long p1, unsigned long p2)
{
	int ret;
	asm (" call hypercall_addr\n"
		: "=a" (ret)
		: "a" (nr),
		  "b" (p1),
		  "c" (p2)
		: "memory", "cc"
	);
	return ret;
}

static inline int
__hypercall3(unsigned int nr, unsigned long p1, unsigned long p2,
	     unsigned long p3)
{
	int ret;
	asm (" call hypercall_addr\n"
		: "=a" (ret)
		: "a" (nr),
		  "b" (p1),
		  "c" (p2),
		  "d" (p3)
		: "memory", "cc"
	);
	return ret;
}

static inline int
__hypercall4(unsigned int nr, unsigned long p1, unsigned long p2,
	     unsigned long p3, unsigned long p4)
{
	int ret;
	asm (" call hypercall_addr\n"
		: "=a" (ret)
		: "a" (nr),
		  "b" (p1),
		  "c" (p2),
		  "d" (p3),
		  "S" (p4)
		: "memory", "cc"
	);
	return ret;
}

static inline int
__hypercall5(unsigned int nr, unsigned long p1, unsigned long p2,
	     unsigned long p3, unsigned long p4, unsigned long p5)
{
	int ret;
	asm (" call hypercall_addr\n"
		: "=a" (ret)
		: "a" (nr),
		  "b" (p1),
		  "c" (p2),
		  "d" (p3),
		  "S" (p4),
		  "D" (p5)
		: "memory", "cc"
	);
	return ret;
}

static inline int
__hypercall6(unsigned int nr, unsigned long p1, unsigned long p2,
	     unsigned long p3, unsigned long p4, unsigned long p5,
	     unsigned long p6)
{
	int ret;
	asm (" call hypercall_addr\n"
		: "=a" (ret)
		: "a" (nr),
		  "b" (p1),
		  "c" (p2),
		  "d" (p3),
		  "S" (p4),
		  "D" (p5),
		  "bp" (p6)
		: "memory", "cc"
	);
	return ret;
}


#define hypercall(nr_params, args...)				\
({								\
	int __ret;						\
								\
	per_cpu(irq_stat, raw_smp_processor_id()).hypercalls++;	\
	__ret = __hypercall##nr_params(args);			\
								\
	__ret;							\
})

#endif /* CONFIG_PARAVIRT */

#endif	/* __ASM_HYPERCALL_H */
