/*
 * KVM/NET host network driver
 * 
 * Copyright (C) 2007, Red Hat, Inc., Ingo Molnar <mingo@redhat.com>
 * Copyright (C) 2007, Qumranet, Inc., Dor Laor <dor.laor@qumranet.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2.  See
 * the COPYING file in the top-level directory.
 *
 */

//#define DEBUG

#include "kvm.h"
#include "kvm_net.h"

#include <linux/kvm_para.h>
#include <linux/kvm.h>
#include <linux/highmem.h>

#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/irq.h>
#include <linux/mm.h>
#include <linux/etherdevice.h>
#include <linux/netdevice.h>
#include <linux/if_ether.h>
#include <linux/ethtool.h>
#include <linux/skbuff.h>
#include <linux/if_arp.h>
#include <linux/inet.h>

static struct kvm_eth_state *eth_state_hva;

/*
 * The host ring state holds the gpa_to_hpa translation that is done once
 */
static struct {
	unsigned long *tx_desc_page_hva[KVM_ETH_TX_RING_SIZE];
	unsigned long *rx_desc_page_hva[KVM_ETH_RX_RING_SIZE];
} host_ring_state;

int hypercall_register_eth(struct kvm_vcpu *vcpu, gpa_t eth_state_gpa)
{
	hpa_t eth_state_hpa;
	hpa_t page_hpa;
	int i, ret;

	pr_debug("hypercall_register_eth(%p): %08Lx\n", vcpu, eth_state_gpa);

	/*
	 * We only want to work with nice round addresses:
	 */
	if (PAGE_ALIGN(eth_state_gpa) != eth_state_gpa) {
		WARN_ON(1);
		return -EINVAL;
	}
	eth_state_hpa = gpa_to_hpa(vcpu, eth_state_gpa);
	if (is_error_hpa(eth_state_hpa)) {
		WARN_ON(1);
                return -EINVAL;
	}
	pr_debug("eth_state_hpa: %08Lx\n", eth_state_hpa);

	/*
	 * TODO: kunmap() this upon exit:
	 */
	eth_state_hva = kmap(pfn_to_page(eth_state_hpa >> PAGE_SHIFT));

	pr_debug("eth_state_hva: %p\n", eth_state_hva);
	pr_debug("eth_state_hva magic: %08x\n", eth_state_hva->magic);
	WARN_ON(eth_state_hva->magic != KVM_ETH_MAGIC);

	spin_lock(&vcpu->kvm->lock);
	for (i = 0; i < KVM_ETH_TX_RING_SIZE; i++) {
		page_hpa = gpa_to_hpa(vcpu,
				      eth_state_hva->tx_ring[i].page_gpa);
		ret = is_error_hpa(page_hpa) ? -EFAULT : 0;
		if (ret) {
			WARN_ON(1);
			spin_unlock(&vcpu->kvm->lock);
			return -EINVAL;
		}
		/*
		 * TODO: unmap these upon exit:
		 */
		host_ring_state.tx_desc_page_hva[i] =
				kmap(pfn_to_page(page_hpa >> PAGE_SHIFT));
	}

	for (i = 0; i < KVM_ETH_RX_RING_SIZE; i++) {
		page_hpa = gpa_to_hpa(vcpu,
				      eth_state_hva->rx_ring[i].page_gpa);
		ret = is_error_hpa(page_hpa) ? -EFAULT : 0;
		if (ret) {
			WARN_ON(1);
			spin_unlock(&vcpu->kvm->lock);
			return -EINVAL;
		}
		/*
		 * TODO: unmap these upon exit:
		 */
		host_ring_state.rx_desc_page_hva[i] =
				kmap(pfn_to_page(page_hpa >> PAGE_SHIFT));
	}
	spin_unlock(&vcpu->kvm->lock);

	return 0;
}

int hypercall_send_eth(struct kvm_vcpu *vcpu)
{
	struct kvm_eth_state *eth_state = eth_state_hva;
	struct net_device *netdev = vcpu->kvm->host_netdev;
	struct kvm_tx_ring_desc *tx_desc;
	unsigned long *buff_hva;
	struct sk_buff *skb;
	unsigned int ret = 0;
	int idx;

	if (!eth_state || !netdev)
		return -EINVAL;

	pr_debug("eth_state->tx_head: %d\n", eth_state->tx_head);
	pr_debug("eth_state->tx_tail: %d\n", eth_state->tx_tail);

	while (eth_state->tx_tail != eth_state->tx_head) {

		tx_desc = get_tail_tx_desc(eth_state);
		idx = tx_desc - eth_state->tx_ring;
		pr_debug(".. tx_desc->size:     %d\n", tx_desc->size);
		buff_hva = host_ring_state.tx_desc_page_hva[idx];
		pr_debug(".. buff_hva: %p\n", buff_hva);

		if (!netif_running(netdev)) {
			pr_debug("netdev %s is not running, not sending!\n",
				netdev->name);
			ret = -EFAULT;
			break;
		}

		skb = netdev_alloc_skb(netdev, tx_desc->size);
		if (!skb) {
			ret = -ENOMEM;
			break;
		}
		skb_put(skb, tx_desc->size);
		memcpy(skb->data, buff_hva, tx_desc->size);

		skb->dev = netdev;
		skb->protocol = eth_type_trans(skb, netdev);
		skb->ip_summed = CHECKSUM_UNNECESSARY;

		trace_special((unsigned long)skb, eth_state->tx_head, eth_state->tx_tail);
		//print_skb(skb);
		netif_rx_ni(skb);

		free_tail_tx_desc(eth_state, tx_desc);
		pr_debug(".. eth_state->tx_head: %d\n", eth_state->tx_head);
		pr_debug(".. eth_state->tx_tail: %d\n", eth_state->tx_tail);
	}

	return ret;
}

/*
 * Host side xmit. This prepares the packet into the shared buffer
 * and injects an interrupt.
 */
static int kvmnet_host_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct kvm_eth_state *eth_state = eth_state_hva;
	struct kvm_rx_ring_desc *rx_desc;
	struct kvm *kvm = dev->priv;
	struct kvm_vcpu *vcpu = &kvm->vcpus[0];
	struct task_struct *task;
	unsigned long *buff_hva;
	unsigned int irq;
	int ret = 0;
	u32 idx;

	trace_special((unsigned long)skb, eth_state->rx_head, eth_state->rx_tail);

	/* Note that caller will free the skb on error so there is no leak*/
	if (!netif_running(dev)) {
		pr_debug("netdev %s is not running, very weird!\n",
			 dev->name);
		WARN_ON(1);
		return -EFAULT;
	}

        idx = eth_state->rx_head;
	rx_desc = eth_state->rx_ring + idx;
        buff_hva = host_ring_state.rx_desc_page_hva[idx];
	pr_debug(".. buff_hva: %p\n", buff_hva);

	rx_desc->size = skb->len;
	memcpy(buff_hva, skb->data, skb->len);
	commit_head_rx_desc(eth_state, rx_desc);

//	rmb();
//	if (!atomic_read(&eth_state->irq_enable))
//		goto out;

	irq = eth_state->irq;
	task = vcpu->hyper_vcpu.task;
	WARN_ON(!task);

	WARN_ON(!irq || irq >= NR_IRQS);
	if (irq >= NR_IRQS)
		goto out;
	/*
	 * Inject the irq and cause the vcpu to vmexit to shorten irq latency.
	 */
	eth_state->irq_gen_host++;
	pr_debug("test: host-dev xmit, injecting irq %d\n", irq);
	set_bit(irq + 32, vcpu->irq_pending);
	set_bit((irq + 32) / BITS_PER_LONG, &vcpu->irq_summary);

	/*
	 * A bit racy - replace with proper halt-wakeup:
	 */
	signal_wake_up(task, 0);
#if 0
	if (task->state == TASK_INTERRUPTIBLE)
		wake_up_process(task);
	else if (task->state == TASK_RUNNING)
		kick_process(task);
#endif
out:
	dev_kfree_skb(skb);

	return ret;
}

struct net_device_stats kvmnet_stats;
DEFINE_PER_CPU(struct pcpu_kvmstats, pcpu_kvmstats);

static const struct ethtool_ops kvmnet_ethtool_ops = {
	.get_link		= always_on,
	.get_tso		= ethtool_op_get_tso,
	.set_tso		= ethtool_op_set_tso,
	.get_tx_csum		= always_on,
	.get_sg			= always_on,
	.get_rx_csum		= always_on,
};

/*
 * The kvmnet device is special. There is only one instance and
 * it is statically allocated. Don't do this for other devices.
 */

int kvmnet_host_init(struct kvm* kvm)
{
	struct net_device *netdev;

	WARN_ON(sizeof(struct kvm_eth_state) > PAGE_SIZE);

	printk(KERN_INFO "%s size of kvm_eth_state is %d\n", __FUNCTION__,
		(int)sizeof(struct kvm_eth_state));
	netdev = alloc_etherdev(sizeof(struct kvm_eth_state));
	netdev->priv = kvm;
	kvm->host_netdev = netdev;

	netdev->ethtool_ops = &kvmnet_ethtool_ops;

	netdev->flags &= ~IFF_MULTICAST;
	netdev->mtu = 1500;
	netdev->weight = 16;
	netdev->hard_start_xmit = kvmnet_host_xmit;
	strcpy(netdev->name, "kvmnet-host");

	netdev->dev_addr[0] = 0x00;
	netdev->dev_addr[1] = 0x11;
	netdev->dev_addr[2] = 0x22;
	netdev->dev_addr[3] = 0x33;
	netdev->dev_addr[4] = 0x44;
	netdev->dev_addr[5] = 0x66;
	netdev->tx_queue_len = KVM_ETH_RX_RING_SIZE-1;

	return register_netdev(netdev);
};

void kvmnet_host_exit(struct kvm* kvm)
{
	unregister_netdev(kvm->host_netdev);
	free_netdev(kvm->host_netdev);
}

EXPORT_SYMBOL_GPL(kvmnet_host_init);
EXPORT_SYMBOL_GPL(kvmnet_host_exit);
