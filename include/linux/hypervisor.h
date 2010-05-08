/*
 * Linux hypervisor support
 */
#ifndef __LINUX_HYPERVISOR_H
#define __LINUX_HYPERVISOR_H

#include <linux/spinlock_types.h>

struct task_struct;

#ifndef CONFIG_HYPERVISOR

static inline void exit_hyper_vcpu(struct task_struct *t)
{
}

#else /* CONFIG_HYPERVISOR */

struct hyper_vcpu
{
	spinlock_t lock;
	struct task_struct *task;
	void *arch_vcpu;
};

extern int hyper_bind_vcpu(struct task_struct *task, struct hyper_vcpu *vcpu,
			   void *arch_vcpu);
extern int hyper_unbind_vcpu(struct task_struct *task);
extern void exit_hyper_vcpu(struct task_struct *t);

#endif /* CONFIG_HYPERVISOR */

#endif /* __LINUX_HYPERVISOR_H */
