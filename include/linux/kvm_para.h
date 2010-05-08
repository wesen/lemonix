#ifndef __LINUX_KVM_PARA_H
#define __LINUX_KVM_PARA_H

#include <linux/paravirt.h>

/*
 * Guest OS interface for KVM paravirtualization
 *
 * Note: this interface is totally experimental, and is certain to change
 *       as we make progress.
 *
 * Copyright (C) 2007, Red Hat, Inc., Ingo Molnar <mingo@redhat.com>
 * Copyright (C) 2007, Qumranet, Inc., Dor Laor <dor.laor@qumranet.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2.  See
 * the COPYING file in the top-level directory.
 */

#define KVM_CR3_CACHE_SIZE	4

struct kvm_cr3_cache_entry {
	u64 guest_cr3;
	u64 host_cr3;
};

struct kvm_cr3_cache {
	u32 entry_count;
	u32 __pad;
	struct kvm_cr3_cache_entry entry[KVM_CR3_CACHE_SIZE];
} __attribute__ ((aligned(PAGE_SIZE)));

/*
 * Per-VCPU descriptor area shared between guest and host. Writable to
 * both guest and host. Registered with the host by the guest when
 * a guest acknowledges paravirtual mode.
 *
 * NOTE: all addresses are guest-physical addresses (gpa), to make it
 * easier for the hypervisor to map between the various addresses.
 */
struct kvm_vcpu_para_state {
	/*
	 * API version information for compatibility. If there's any support
	 * mismatch (too old host trying to execute too new guest) then
	 * the host will deny entry into paravirtual mode. Any other
	 * combination (new host + old guest and new host + new guest)
	 * is supposed to work - new host versions will support all old
	 * guest API versions.
	 */
	u32 guest_version;
	u32 host_version;
	u32 size;
	u32 ret;

	/*
	 * The address of the vm exit instruction (VMCALL or VMMCALL),
	 * which the host will patch according to the CPU model the
	 * VM runs on:
	 */
	u64 hypercall_gpa;

	/*
	 * Pointer to the struct kvm_cr3_cache CR3 cache:
	 */
	u64 cr3_cache_gpa;

} __attribute__ ((aligned(PAGE_SIZE)));

/*
 * Network device state for PV guests
 */
#define KVM_ETH_RX_RING_SIZE 64
#define KVM_ETH_TX_RING_SIZE 64

#define KVM_ETH_MAGIC 0x12341234
#define KVM_ETH_IRQ_MASK 	0x1

struct kvm_tx_ring_desc {
	u32 size;
	u32 __pad;
	u64 page_gpa;
};

struct kvm_rx_ring_desc {
	u32 size;
	u32 __pad;
	u64 page_gpa;
};

struct kvm_eth_state {
	u32 tx_head;
	u32 tx_tail;
	struct kvm_tx_ring_desc tx_ring[KVM_ETH_TX_RING_SIZE];

	u32 rx_head;
	u32 rx_tail;
	struct kvm_rx_ring_desc rx_ring[KVM_ETH_RX_RING_SIZE];

	atomic_t irq_enable;
	u32 irq_gen_guest;
	u32 irq_gen_host;

	u32 magic;
	u32 irq;
} __attribute__ ((aligned(PAGE_SIZE)));


#define KVM_PARA_API_VERSION 1

/*
 * This is used for an RDMSR's ECX parameter to probe for a KVM host.
 * Hopefully no CPU vendor will use up this number. This is placed well
 * out of way of the typical space occupied by CPU vendors' MSR indices,
 * and we think (or at least hope) it wont be occupied in the future
 * either.
 */
#define MSR_KVM_API_MAGIC 0x87655678

#define KVM_EINVAL 1

/*
 * Hypercall calling convention:
 *
 * Each hypercall may have 0-6 parameters.
 *
 * 64-bit hypercall index is in RAX, goes from 0 to __NR_hypercalls-1
 *
 * 64-bit parameters 1-6 are in the standard gcc x86_64 calling convention
 * order: RDI, RSI, RDX, RCX, R8, R9.
 *
 * 32-bit index is EAX, parameters are: EBX, ECX, EDX, ESI, EDI, EBP.
 * (the first 3 are according to the gcc regparm calling convention)
 *
 * No registers are clobbered by the hypercall, except that the
 * return value is in RAX.
 */
#define __NR_hypercall_test			0
#define __NR_hypercall_load_cr3			1
#define __NR_hypercall_register_eth		2
#define __NR_hypercall_send_eth			3
#define __NR_hypercall_get_ktime		4
#define __NR_hypercall_disk_count		5
#define __NR_hypercall_disk_read		6
#define __NR_hypercall_disk_write		7
#define __NR_hypercall_disk_info		8
#define __NR_hypercall_disk_register_completion_area	10
#define __NR_hypercall_disk_register_submit_area	11
#define __NR_hypercall_disk_trigger_scan		12
#define __NR_hypercall_flush_cr3_cache			13
#define __NR_hypercall_trace			14

#define __NR_hypercalls				15

asmlinkage void __init kvm_probe(void);

#endif
