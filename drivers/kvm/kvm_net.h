/*
 * KVM/NET host network driver
 *
 * Copyright (C) 2007, Red Hat, Inc., Ingo Molnar <mingo@redhat.com>
 * Copyright (C) 2007, Qumranet, Inc., Dor Laor <dor.laor@qumranet.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2.  See
 * the COPYING file in the top-level directory.
 */

#include "kvm.h"

#include <linux/kvm_para.h>
#include <linux/kvm.h>

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

#undef pr_debug
#ifdef DEBUG
# define pr_debug(fmt,arg...) \
		do { printk("%s:%d " fmt, __FUNCTION__, __LINE__, ##arg); } while (0)
#else 
# define pr_debug(fmt,arg...)
#endif

static inline void print_skb(struct sk_buff *skb)
{
#ifdef DEBUG
	int i;

	printk("skb %p, data: %p, len: %d\n", skb, skb->data, skb->len);
	for (i = 0; i < skb->len; i++) {
		printk(" %02x", skb->data[i]);
		if ((i & 15) == 15)
			printk("\n");
	}
	if ((i & 15) != 15)
		printk("\n");
#endif
}

struct pcpu_kvmstats {
	unsigned long packets;
	unsigned long bytes;
};


/*
 * Ring of buffers:
 */
static inline unsigned int tx_idx_next(unsigned int idx)
{
	return (idx + 1) & (KVM_ETH_TX_RING_SIZE - 1);
}

static inline struct kvm_tx_ring_desc *
get_head_tx_desc(struct kvm_eth_state *eth_state)
{
	WARN_ON(tx_idx_next(eth_state->tx_head) == eth_state->tx_tail);

	return eth_state->tx_ring + eth_state->tx_head;
}

static inline struct kvm_tx_ring_desc *
get_tail_tx_desc(struct kvm_eth_state *eth_state)
{
	WARN_ON(eth_state->tx_head == eth_state->tx_tail);

	return eth_state->tx_ring + eth_state->tx_tail;
}

/*
 * Make the new TX descriptor visible to the host:
 */
static inline void
commit_head_tx_desc(struct kvm_eth_state *eth_state,
		    struct kvm_tx_ring_desc *tx_desc)
{
	WARN_ON(tx_desc - eth_state->tx_ring != eth_state->tx_head);

	eth_state->tx_head = tx_idx_next(eth_state->tx_head);
}

static inline void
free_tail_tx_desc(struct kvm_eth_state *eth_state,
		  struct kvm_tx_ring_desc *tx_desc)
{
	WARN_ON(tx_desc - eth_state->tx_ring != eth_state->tx_tail);

	eth_state->tx_tail = tx_idx_next(eth_state->tx_tail);
}

static inline unsigned int rx_idx_next(unsigned int idx)
{
	return (idx + 1) & (KVM_ETH_RX_RING_SIZE - 1);
}

static inline struct kvm_rx_ring_desc *
get_head_rx_desc(struct kvm_eth_state *eth_state)
{
	WARN_ON(rx_idx_next(eth_state->rx_head) == eth_state->rx_tail);

	return eth_state->rx_ring + eth_state->rx_head;
}

static inline struct kvm_rx_ring_desc *
get_tail_rx_desc(struct kvm_eth_state *eth_state)
{
	WARN_ON(eth_state->rx_head == eth_state->rx_tail);

	return eth_state->rx_ring + eth_state->rx_tail;
}

/*
 * Make the new RX descriptor visible to the host:
 */
static inline void
commit_head_rx_desc(struct kvm_eth_state *eth_state,
		    struct kvm_rx_ring_desc *rx_desc)
{
	WARN_ON(rx_desc - eth_state->rx_ring != eth_state->rx_head);

	eth_state->rx_head = rx_idx_next(eth_state->rx_head);
}

static inline void
free_tail_rx_desc(struct kvm_eth_state *eth_state,
		  struct kvm_rx_ring_desc *rx_desc)
{
	WARN_ON(rx_desc - eth_state->rx_ring != eth_state->rx_tail);

	eth_state->rx_tail = rx_idx_next(eth_state->rx_tail);
}

extern struct net_device_stats kvmnet_stats;
DECLARE_PER_CPU(struct pcpu_kvmstats, pcpu_kvmstats);

static inline struct net_device_stats *get_stats(struct net_device *dev)
{
	struct net_device_stats *stats = dev->priv;
	unsigned long packets = 0;
	unsigned long bytes = 0;
	int i;

	for_each_possible_cpu(i) {
		const struct pcpu_kvmstats *kvm_stats;

		kvm_stats = &per_cpu(pcpu_kvmstats, i);
		bytes   += kvm_stats->bytes;
		packets += kvm_stats->packets;
	}
	stats->rx_packets = packets;
	stats->tx_packets = packets;
	stats->rx_bytes = bytes;
	stats->tx_bytes = bytes;

	return stats;
}

static u32 always_on(struct net_device *dev)
{
	return 1;
}

