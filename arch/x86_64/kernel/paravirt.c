#include <linux/errno.h>
#include <linux/module.h>
#include <linux/efi.h>
#include <linux/bcd.h>
#include <linux/start_kernel.h>
#include <linux/kvm.h>
#include <linux/kvm_para.h>

#include <asm/hypercall.h>

struct paravirt_ops paravirt_ops;

EXPORT_SYMBOL_GPL(paravirt_ops);

static DEFINE_PER_CPU(struct kvm_vcpu_para_state, para_state);
static DEFINE_PER_CPU(struct kvm_cr3_cache, cr3_cache);

/*
 * This is the vm-syscall address - to be patched by the host to
 * VMCALL (Intel) or VMMCALL (AMD), depending on the CPU model:
 */
asm (
	"	.globl hypercall_addr			\n"
	"	.align 4				\n"
	"	hypercall_addr:				\n"
	"		movl $-38, %eax			\n"
	"		ret				\n"
);

extern unsigned char hypercall_addr[6];

EXPORT_SYMBOL_GPL(hypercall_addr);

static void test_hypercall(void)
{
        int ret;
	unsigned long dummy = 123456;

        ret = hypercall(1, __NR_hypercall_test, dummy);
        pr_debug("hypercall test #1, ret: 0x%x\n", ret);
}

int kvm_guest_register_para(int cpu)
{
	struct kvm_vcpu_para_state *para_state = &per_cpu(para_state, cpu);
	struct kvm_cr3_cache *cr3_cache = &per_cpu(cr3_cache, cpu);

	printk(KERN_DEBUG "kvm guest on VCPU#%d: trying to register para_state %p\n",
		cpu, para_state);
	/*
	 * Try to write to a magic MSR (which is invalid on any real CPU),
	 * and thus signal to KVM that we wish to entering paravirtualized
	 * mode:
	 */
	para_state->guest_version = KVM_PARA_API_VERSION;
	para_state->host_version = -1;
	para_state->size = sizeof(*para_state);
	para_state->ret = -1;
	para_state->hypercall_gpa = __pa(hypercall_addr);
	cr3_cache->entry_count = KVM_CR3_CACHE_SIZE;
	para_state->cr3_cache_gpa = __pa(cr3_cache);

	if (wrmsr_safe(MSR_KVM_API_MAGIC, __pa(para_state), 0)) {
		printk(KERN_INFO "KVM guest: WRMSR probe failed.\n");
		return 0;
	}

	printk(KERN_DEBUG "kvm guest: host returned %d\n", para_state->ret);
	printk(KERN_DEBUG "kvm guest: host version: %d\n", para_state->host_version);
	printk(KERN_DEBUG "kvm guest: cr3 cache size: %d\n", cr3_cache->entry_count);
	printk(KERN_DEBUG "kvm guest: syscall entry: %02x %02x %02x %02x\n",
			hypercall_addr[0], hypercall_addr[1],
			hypercall_addr[2], hypercall_addr[3]);
	if (para_state->ret) {
		printk(KERN_ERR "kvm guest: host refused registration.\n");
		return 0;
	}
	test_hypercall();

	return 1;
}

asmlinkage void __init kvm_probe(void)
{
	int paravirt;

	printk(KERN_DEBUG "KVM paravirtualization probe\n");

	paravirt = kvm_guest_register_para(smp_processor_id());
	if (!paravirt) {
		printk(KERN_INFO "Not a KVM para-guest\n");
		return;
	}

	printk(KERN_INFO "KVM para-guest: OK\n");

	paravirt_ops.paravirt_enabled = 1;
}
