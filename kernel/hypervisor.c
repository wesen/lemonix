/*
 * kernel/hypervisor.c
 *
 * Linux hypervisor support
 *
 * Started by Ingo Molnar:
 *
 *  Copyright (C) 2007 Red Hat, Inc., Ingo Molnar <mingo@redhat.com>
 */
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/kvm.h>
#include <linux/kvm_para.h>
#include <asm/hypercall.h>

int hyper_bind_vcpu(struct task_struct *task, struct hyper_vcpu *vcpu,
		    void *arch_vcpu)
{
	int err = -EINVAL;

	if (!unlikely(vcpu))
		goto out;

	spin_lock(&vcpu->lock);
	if (!task->vcpu && !vcpu->task) {
		vcpu->task = task;
		WARN_ON(vcpu->arch_vcpu);
		vcpu->arch_vcpu = arch_vcpu;
		task->vcpu = vcpu;
		err = 0;
	}
	spin_unlock(&vcpu->lock);
out:
	WARN_ON(err);

	return err;
}
EXPORT_SYMBOL_GPL(hyper_bind_vcpu);

int hyper_unbind_vcpu(struct task_struct *task)
{
	struct hyper_vcpu *vcpu = task->vcpu;
	int err = -EINVAL;

	if (!unlikely(vcpu))
		goto out;

	spin_lock(&vcpu->lock);
	if (vcpu && vcpu->task == task) {
		vcpu->task = NULL;
		vcpu->arch_vcpu = NULL;
		task->vcpu = NULL;
		err = 0;
	}
	spin_unlock(&vcpu->lock);
out:
	WARN_ON(err);

	return err;
}
EXPORT_SYMBOL_GPL(hyper_unbind_vcpu);

void exit_hyper_vcpu(struct task_struct *t)
{
	int err;

	err = hyper_unbind_vcpu(t);
	WARN_ON(err);
}

#ifdef CONFIG_EVENT_TRACE
void trace_hyper(unsigned long v1, unsigned long v2, unsigned long v3)
{
	long ret;

	ret = hypercall(4, __NR_hypercall_trace, CALLER_ADDR0, v1, v2, v3);
	WARN_ON(ret);
}
EXPORT_SYMBOL_GPL(trace_hyper);
#endif

