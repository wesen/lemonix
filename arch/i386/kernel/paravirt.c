/*  Paravirtualization interfaces
    Copyright (C) 2006 Rusty Russell IBM Corporation

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/efi.h>
#include <linux/bcd.h>
#include <linux/start_kernel.h>
#include <linux/kvm.h>
#include <linux/kvm_para.h>

#include <asm/bug.h>
#include <asm/paravirt.h>
#include <asm/desc.h>
#include <asm/setup.h>
#include <asm/arch_hooks.h>
#include <asm/time.h>
#include <asm/irq.h>
#include <asm/delay.h>
#include <asm/fixmap.h>
#include <asm/apic.h>
#include <asm/tlbflush.h>
#include <asm/timer.h>

#include <asm/i8259.h>
#include <asm/hypercall.h>
#include <io_ports.h>

/* nop stub */
static void native_nop(void)
{
}

static void __init default_banner(void)
{
	printk(KERN_INFO "Booting paravirtualized kernel on %s\n",
	       paravirt_ops.name);
}

char *memory_setup(void)
{
	return paravirt_ops.memory_setup();
}

/* Simple instruction patching code. */
#define DEF_NATIVE(name, code)					\
	extern const char start_##name[], end_##name[];		\
	asm("start_" #name ": " code "; end_" #name ":")
DEF_NATIVE(cli, "cli");
DEF_NATIVE(sti, "sti");
DEF_NATIVE(popf, "push %eax; popf");
DEF_NATIVE(pushf, "pushf; pop %eax");
DEF_NATIVE(pushf_cli, "pushf; pop %eax; cli");
DEF_NATIVE(iret, "iret");
DEF_NATIVE(sti_sysexit, "sti; sysexit");

static const struct native_insns
{
	const char *start, *end;
} native_insns[] = {
	[PARAVIRT_IRQ_DISABLE] = { start_cli, end_cli },
	[PARAVIRT_IRQ_ENABLE] = { start_sti, end_sti },
	[PARAVIRT_RESTORE_FLAGS] = { start_popf, end_popf },
	[PARAVIRT_SAVE_FLAGS] = { start_pushf, end_pushf },
	[PARAVIRT_SAVE_FLAGS_IRQ_DISABLE] = { start_pushf_cli, end_pushf_cli },
	[PARAVIRT_INTERRUPT_RETURN] = { start_iret, end_iret },
	[PARAVIRT_STI_SYSEXIT] = { start_sti_sysexit, end_sti_sysexit },
};

static unsigned native_patch(u8 type, u16 clobbers, void *insns, unsigned len)
{
	unsigned int insn_len;

	/* Don't touch it if we don't have a replacement */
	if (type >= ARRAY_SIZE(native_insns) || !native_insns[type].start)
		return len;

	insn_len = native_insns[type].end - native_insns[type].start;

	/* Similarly if we can't fit replacement. */
	if (len < insn_len)
		return len;

	memcpy(insns, native_insns[type].start, insn_len);
	return insn_len;
}

static fastcall unsigned long native_get_debugreg(int regno)
{
	unsigned long val = 0; 	/* Damn you, gcc! */

	switch (regno) {
	case 0:
		asm("movl %%db0, %0" :"=r" (val)); break;
	case 1:
		asm("movl %%db1, %0" :"=r" (val)); break;
	case 2:
		asm("movl %%db2, %0" :"=r" (val)); break;
	case 3:
		asm("movl %%db3, %0" :"=r" (val)); break;
	case 6:
		asm("movl %%db6, %0" :"=r" (val)); break;
	case 7:
		asm("movl %%db7, %0" :"=r" (val)); break;
	default:
		BUG();
	}
	return val;
}

static fastcall void native_set_debugreg(int regno, unsigned long value)
{
	switch (regno) {
	case 0:
		asm("movl %0,%%db0"	: /* no output */ :"r" (value));
		break;
	case 1:
		asm("movl %0,%%db1"	: /* no output */ :"r" (value));
		break;
	case 2:
		asm("movl %0,%%db2"	: /* no output */ :"r" (value));
		break;
	case 3:
		asm("movl %0,%%db3"	: /* no output */ :"r" (value));
		break;
	case 6:
		asm("movl %0,%%db6"	: /* no output */ :"r" (value));
		break;
	case 7:
		asm("movl %0,%%db7"	: /* no output */ :"r" (value));
		break;
	default:
		BUG();
	}
}

void init_IRQ(void)
{
	paravirt_ops.init_IRQ();
}

static fastcall void native_clts(void)
{
	asm volatile ("clts");
}

static fastcall unsigned long native_read_cr0(void)
{
	unsigned long val;
	asm volatile("movl %%cr0,%0\n\t" :"=r" (val));
	return val;
}

static fastcall void native_write_cr0(unsigned long val)
{
	asm volatile("movl %0,%%cr0": :"r" (val));
}

static fastcall unsigned long native_read_cr2(void)
{
	unsigned long val;
	asm volatile("movl %%cr2,%0\n\t" :"=r" (val));
	return val;
}

static fastcall void native_write_cr2(unsigned long val)
{
	asm volatile("movl %0,%%cr2": :"r" (val));
}

static fastcall unsigned long native_read_cr3(void)
{
	unsigned long val;
	asm volatile("movl %%cr3,%0\n\t" :"=r" (val));
	return val;
}

static fastcall void native_write_cr3(unsigned long val)
{
	asm volatile("movl %0,%%cr3": :"r" (val));
}

static fastcall unsigned long native_read_cr4(void)
{
	unsigned long val;
	asm volatile("movl %%cr4,%0\n\t" :"=r" (val));
	return val;
}

static fastcall unsigned long native_read_cr4_safe(void)
{
	unsigned long val;
	/* This could fault if %cr4 does not exist */
	asm("1: movl %%cr4, %0		\n"
		"2:				\n"
		".section __ex_table,\"a\"	\n"
		".long 1b,2b			\n"
		".previous			\n"
		: "=r" (val): "0" (0));
	return val;
}

static fastcall void native_write_cr4(unsigned long val)
{
	asm volatile("movl %0,%%cr4": :"r" (val));
}

static fastcall unsigned long native_save_fl(void)
{
	unsigned long f;
	asm volatile("pushfl ; popl %0":"=g" (f): /* no input */);
	return f;
}

static fastcall void native_restore_fl(unsigned long f)
{
	asm volatile("pushl %0 ; popfl": /* no output */
			     :"g" (f)
			     :"memory", "cc");
}

static fastcall void native_irq_disable(void)
{
	asm volatile("cli": : :"memory");
}

static fastcall void native_irq_enable(void)
{
	asm volatile("sti": : :"memory");
}

static fastcall void native_safe_halt(void)
{
	asm volatile("sti; hlt": : :"memory");
}

static fastcall void native_halt(void)
{
	asm volatile("hlt": : :"memory");
}

static fastcall void native_wbinvd(void)
{
	asm volatile("wbinvd": : :"memory");
}

static fastcall unsigned long long native_read_msr(unsigned int msr, int *err)
{
	unsigned long long val;

	asm volatile("2: rdmsr ; xorl %0,%0\n"
		     "1:\n\t"
		     ".section .fixup,\"ax\"\n\t"
		     "3:  movl %3,%0 ; jmp 1b\n\t"
		     ".previous\n\t"
 		     ".section __ex_table,\"a\"\n"
		     "   .align 4\n\t"
		     "   .long 	2b,3b\n\t"
		     ".previous"
		     : "=r" (*err), "=A" (val)
		     : "c" (msr), "i" (-EFAULT));

	return val;
}

static fastcall int native_write_msr(unsigned int msr, unsigned long long val)
{
	int err;
	asm volatile("2: wrmsr ; xorl %0,%0\n"
		     "1:\n\t"
		     ".section .fixup,\"ax\"\n\t"
		     "3:  movl %4,%0 ; jmp 1b\n\t"
		     ".previous\n\t"
 		     ".section __ex_table,\"a\"\n"
		     "   .align 4\n\t"
		     "   .long 	2b,3b\n\t"
		     ".previous"
		     : "=a" (err)
		     : "c" (msr), "0" ((u32)val), "d" ((u32)(val>>32)),
		       "i" (-EFAULT));
	return err;
}

static fastcall notrace unsigned long long native_read_tsc(void)
{
	unsigned long long val;
	asm volatile("rdtsc" : "=A" (val));
	return val;
}

static fastcall unsigned long long native_read_pmc(void)
{
	unsigned long long val;
	asm volatile("rdpmc" : "=A" (val));
	return val;
}

static fastcall void native_load_tr_desc(void)
{
	asm volatile("ltr %w0"::"q" (GDT_ENTRY_TSS*8));
}

static fastcall void native_load_gdt(const struct Xgt_desc_struct *dtr)
{
	asm volatile("lgdt %0"::"m" (*dtr));
}

static fastcall void native_load_idt(const struct Xgt_desc_struct *dtr)
{
	asm volatile("lidt %0"::"m" (*dtr));
}

static fastcall void native_store_gdt(struct Xgt_desc_struct *dtr)
{
	asm ("sgdt %0":"=m" (*dtr));
}

static fastcall void native_store_idt(struct Xgt_desc_struct *dtr)
{
	asm ("sidt %0":"=m" (*dtr));
}

static fastcall unsigned long native_store_tr(void)
{
	unsigned long tr;
	asm ("str %0":"=r" (tr));
	return tr;
}

static fastcall void native_load_tls(struct thread_struct *t, unsigned int cpu)
{
#define C(i) get_cpu_gdt_table(cpu)[GDT_ENTRY_TLS_MIN + i] = t->tls_array[i]
	C(0); C(1); C(2);
#undef C
}

static fastcall inline void native_write_dt_entry(void *dt, int entry, u32 entry_low, u32 entry_high)
{
	u32 *lp = (u32 *)((char *)dt + entry*8);
	lp[0] = entry_low;
	lp[1] = entry_high;
}

static fastcall void native_write_ldt_entry(void *dt, int entrynum, u32 low, u32 high)
{
	native_write_dt_entry(dt, entrynum, low, high);
}

static fastcall void native_write_gdt_entry(void *dt, int entrynum, u32 low, u32 high)
{
	native_write_dt_entry(dt, entrynum, low, high);
}

static fastcall void native_write_idt_entry(void *dt, int entrynum, u32 low, u32 high)
{
	native_write_dt_entry(dt, entrynum, low, high);
}

static fastcall void native_load_esp0(struct tss_struct *tss,
				      struct thread_struct *thread)
{
	tss->esp0 = thread->esp0;

	/* This can only happen when SEP is enabled, no need to test "SEP"arately */
	if (unlikely(tss->ss1 != thread->sysenter_cs)) {
		tss->ss1 = thread->sysenter_cs;
		wrmsr(MSR_IA32_SYSENTER_CS, thread->sysenter_cs, 0);
	}
}

static fastcall void native_io_delay(void)
{
	asm volatile("outb %al,$0x80");
}

static fastcall void native_flush_tlb(void)
{
	__native_flush_tlb();
}

/*
 * Global pages have to be flushed a bit differently. Not a real
 * performance problem because this does not happen often.
 */
static fastcall void native_flush_tlb_global(void)
{
	__native_flush_tlb_global();
}

static fastcall void native_flush_tlb_single(u32 addr)
{
	__native_flush_tlb_single(addr);
}

#ifndef CONFIG_X86_PAE
static fastcall void native_set_pte(pte_t *ptep, pte_t pteval)
{
	*ptep = pteval;
}

static fastcall void native_set_pte_at(struct mm_struct *mm, u32 addr, pte_t *ptep, pte_t pteval)
{
	*ptep = pteval;
}

static fastcall void native_set_pmd(pmd_t *pmdp, pmd_t pmdval)
{
	*pmdp = pmdval;
}

#else /* CONFIG_X86_PAE */

static fastcall void native_set_pte(pte_t *ptep, pte_t pte)
{
	ptep->pte_high = pte.pte_high;
	smp_wmb();
	ptep->pte_low = pte.pte_low;
}

static fastcall void native_set_pte_at(struct mm_struct *mm, u32 addr, pte_t *ptep, pte_t pte)
{
	ptep->pte_high = pte.pte_high;
	smp_wmb();
	ptep->pte_low = pte.pte_low;
}

static fastcall void native_set_pte_present(struct mm_struct *mm, unsigned long addr, pte_t *ptep, pte_t pte)
{
	ptep->pte_low = 0;
	smp_wmb();
	ptep->pte_high = pte.pte_high;
	smp_wmb();
	ptep->pte_low = pte.pte_low;
}

static fastcall void native_set_pte_atomic(pte_t *ptep, pte_t pteval)
{
	set_64bit((unsigned long long *)ptep,pte_val(pteval));
}

static fastcall void native_set_pmd(pmd_t *pmdp, pmd_t pmdval)
{
	set_64bit((unsigned long long *)pmdp,pmd_val(pmdval));
}

static fastcall void native_set_pud(pud_t *pudp, pud_t pudval)
{
	*pudp = pudval;
}

static fastcall void native_pte_clear(struct mm_struct *mm, unsigned long addr, pte_t *ptep)
{
	ptep->pte_low = 0;
	smp_wmb();
	ptep->pte_high = 0;
}

static fastcall void native_pmd_clear(pmd_t *pmd)
{
	u32 *tmp = (u32 *)pmd;
	*tmp = 0;
	smp_wmb();
	*(tmp + 1) = 0;
}
#endif /* CONFIG_X86_PAE */

/* These are in entry.S */
extern void native_iret(void);
extern void native_irq_enable_sysexit(void);

static int __init print_banner(void)
{
	paravirt_ops.banner();
	return 0;
}
core_initcall(print_banner);

struct paravirt_ops paravirt_ops = {
	.name = "bare hardware",
	.paravirt_enabled = 0,
	.kernel_rpl = 0,

 	.patch = native_patch,
	.banner = default_banner,
	.arch_setup = native_nop,
	.memory_setup = machine_specific_memory_setup,
	.get_wallclock = native_get_wallclock,
	.set_wallclock = native_set_wallclock,
	.time_init = hpet_time_init,
	.init_IRQ = native_init_IRQ,

	.cpuid = native_cpuid,
	.get_debugreg = native_get_debugreg,
	.set_debugreg = native_set_debugreg,
	.clts = native_clts,
	.read_cr0 = native_read_cr0,
	.write_cr0 = native_write_cr0,
	.read_cr2 = native_read_cr2,
	.write_cr2 = native_write_cr2,
	.read_cr3 = native_read_cr3,
	.write_cr3 = native_write_cr3,
	.read_cr4 = native_read_cr4,
	.read_cr4_safe = native_read_cr4_safe,
	.write_cr4 = native_write_cr4,
	.save_fl = native_save_fl,
	.restore_fl = native_restore_fl,
	.irq_disable = native_irq_disable,
	.irq_enable = native_irq_enable,
	.safe_halt = native_safe_halt,
	.halt = native_halt,
	.wbinvd = native_wbinvd,
	.read_msr = native_read_msr,
	.write_msr = native_write_msr,
	.read_tsc = native_read_tsc,
	.read_pmc = native_read_pmc,
	.get_scheduled_cycles = native_read_tsc,
	.get_cpu_khz = native_calculate_cpu_khz,
	.load_tr_desc = native_load_tr_desc,
	.set_ldt = native_set_ldt,
	.load_gdt = native_load_gdt,
	.load_idt = native_load_idt,
	.store_gdt = native_store_gdt,
	.store_idt = native_store_idt,
	.store_tr = native_store_tr,
	.load_tls = native_load_tls,
	.write_ldt_entry = native_write_ldt_entry,
	.write_gdt_entry = native_write_gdt_entry,
	.write_idt_entry = native_write_idt_entry,
	.load_esp0 = native_load_esp0,

	.set_iopl_mask = native_set_iopl_mask,
	.io_delay = native_io_delay,

#ifdef CONFIG_X86_LOCAL_APIC
	.apic_write = native_apic_write,
	.apic_write_atomic = native_apic_write_atomic,
	.apic_read = native_apic_read,
	.setup_boot_clock = setup_boot_APIC_clock,
	.setup_secondary_clock = setup_secondary_APIC_clock,
#endif
	.set_lazy_mode = (void *)native_nop,

	.flush_tlb_user = native_flush_tlb,
	.flush_tlb_kernel = native_flush_tlb_global,
	.flush_tlb_single = native_flush_tlb_single,

	.map_pt_hook = (void *)native_nop,

	.alloc_pt = (void *)native_nop,
	.alloc_pd = (void *)native_nop,
	.alloc_pd_clone = (void *)native_nop,
	.release_pt = (void *)native_nop,
	.release_pd = (void *)native_nop,

	.set_pte = native_set_pte,
	.set_pte_at = native_set_pte_at,
	.set_pmd = native_set_pmd,
	.pte_update = (void *)native_nop,
	.pte_update_defer = (void *)native_nop,
#ifdef CONFIG_X86_PAE
	.set_pte_atomic = native_set_pte_atomic,
	.set_pte_present = native_set_pte_present,
	.set_pud = native_set_pud,
	.pte_clear = native_pte_clear,
	.pmd_clear = native_pmd_clear,
#endif

	.irq_enable_sysexit = native_irq_enable_sysexit,
	.iret = native_iret,

	.startup_ipi_hook = (void *)native_nop,
};

/*
 * NOTE: CONFIG_PARAVIRT is experimental and the paravirt_ops
 * semantics are subject to change. Hence we only do this
 * internal-only export of this, until it gets sorted out and
 * all lowlevel CPU ops used by modules are separately exported.
 */
EXPORT_SYMBOL_GPL(paravirt_ops);

/*
 * KVM paravirtualization optimizations:
 */
int __read_mostly kvm_paravirt = 1;

static int __init kvm_paravirt_setup(char *str)
{
        kvm_paravirt = simple_strtoul(str, NULL, 0);
        return 1;
}
__setup("kvm_paravirt=", kvm_paravirt_setup);

/*
 * No need for any "IO delay" on KVM:
 */
static void fastcall kvm_io_delay(void)
{
}

static DEFINE_PER_CPU(struct kvm_vcpu_para_state, para_state);
static DEFINE_PER_CPU(struct kvm_cr3_cache, cr3_cache);

/*
 * Special, register-to-cr3 instruction based hypercall API
 * variant to the KVM host. This utilizes the cr3 filter capability
 * of the hardware - if this works out then no VM exit happens,
 * if a VM exit happens then KVM will get the virtual address too.
 */
static void fastcall kvm_write_cr3(unsigned long guest_cr3)
{
	struct kvm_cr3_cache *cache = &get_cpu_var(cr3_cache);
	int idx, ret;

	/*
	 * Check the cache (maintained by the host) for a matching
	 * guest_cr3 => host_cr3 mapping. Use it if found:
	 */
	for (idx = 0; idx < cache->entry_count; idx++) {
		if (cache->entry[idx].guest_cr3 == guest_cr3) {
			/*
			 * Cache-hit: we load the cached host-CR3 value.
			 * This never causes any VM exit. (if it does then the
			 * hypervisor could do nothing with this instruction
			 * and the guest OS would be aborted)
			 */
			asm volatile("movl %0, %%cr3"
				: : "r" (cache->entry[idx].host_cr3));
			goto out;
		}
	}

	/*
	 * Cache-miss. Use the cr3 hypercall to load the guest-physical
	 * address (the host will also update the cr3 cache):
	 */
	ret = hypercall(1, __NR_hypercall_load_cr3, guest_cr3);
	WARN_ON(ret);
out:
	put_cpu_var(cr3_cache);
}

static fastcall void kvm_release_pd(u32 pfn)
{
	long ret;

	ret = hypercall(1, __NR_hypercall_flush_cr3_cache, pfn << PAGE_SHIFT);
	WARN_ON(ret);
}

/*
 * Avoid the VM exit upon cr3 load by using the cached
 * ->active_mm->pgd value:
 */
static void fastcall kvm_flush_tlb_user(void)
{
	kvm_write_cr3(__pa(current->active_mm->pgd));
}

static void fastcall kvm_flush_tlb_single(u32 addr)
{
	__native_flush_tlb_single(addr);
}
/*
 * Disable global pages, do a flush, then enable global pages:
 */
static fastcall void kvm_flush_tlb_kernel(void)
{
	unsigned long orig_cr4 = read_cr4();

	write_cr4(orig_cr4 & ~X86_CR4_PGE);
	kvm_flush_tlb_user();
	write_cr4(orig_cr4);
}

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
	/*
	 * Did the user disable it explicitly via kvm_paravirt=0?
	 */
	if (!kvm_paravirt)
		return;

	printk(KERN_DEBUG "KVM paravirtualization probe\n");

	kvm_paravirt = kvm_guest_register_para(smp_processor_id());
	if (!kvm_paravirt) {
		printk(KERN_INFO "Not a KVM para-guest\n");
		return;
	}

	printk(KERN_INFO "KVM para-guest: OK\n");

	paravirt_ops.name = "KVM";
	paravirt_ops.io_delay = kvm_io_delay;
	paravirt_ops.flush_tlb_user = kvm_flush_tlb_user;
	paravirt_ops.flush_tlb_kernel = kvm_flush_tlb_kernel;
	paravirt_ops.flush_tlb_single = kvm_flush_tlb_single;
	paravirt_ops.write_cr3 = kvm_write_cr3;
	paravirt_ops.release_pd = kvm_release_pd;

	paravirt_ops.paravirt_enabled = 1;
}

paravirt_probe(kvm_probe);
