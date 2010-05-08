/*
 * KVM/NET guest para virtualized network driver
 *
 * Copyright (C) 2007, Red Hat, Inc., Ingo Molnar <mingo@redhat.com>
 * Copyright (C) 2007, Qumranet, Inc., Dor Laor <dor.laor@qumranet.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2.  See
 * the COPYING file in the top-level directory.
 */

//#define DEBUG

#include "kvm.h"
#include "kvm_net.h"
#include <linux/paravirt.h>
#include <linux/kvm_para.h>
#include <linux/kvm.h>

#include <asm/hypercall.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/percpu.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/mm.h>
#include <linux/version.h>
#include <linux/etherdevice.h>
#include <linux/netdevice.h>
#include <linux/if_ether.h>
#include <linux/ethtool.h>
#include <linux/skbuff.h>
#include <linux/if_arp.h>
#include <linux/inet.h>

#include <linux/pci.h>
#include <linux/init.h>
#include <linux/ioport.h>

MODULE_AUTHOR ("Ingo Molnar and Dor Laor");
MODULE_DESCRIPTION ("Implements guest para virtual network driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1");

#ifndef CONFIG_PARAVIRT

/*
 * This is the vm-syscall address - to be patched by the host to
 * VMCALL (Intel) or VMMCALL (AMD), depending on the CPU model:
 */
asm (
        "       .globl hypercall_addr                   \n"
        "       .align 4                                \n"
        "       hypercall_addr:                         \n"
	"               movl $-38, %eax                 \n"
        "               ret                             \n"
);

extern unsigned char hypercall_addr[6];

static void test_hypercall(void)
{
        int ret;
	unsigned long dummy = 123456;

        ret = hypercall(1, __NR_hypercall_test, dummy);
        pr_debug("hypercall test #1, ret: 0x%x\n", ret);
}

#ifndef CONFIG_X86_64
static DEFINE_PER_CPU(struct kvm_vcpu_para_state, para_state);
#endif

static int kvm_guest_register_para(int cpu)
{
	struct page *hypercall_addr_page;
	struct kvm_vcpu_para_state *para_state;

#ifdef CONFIG_X86_64
	struct page *pstate_page;
	if ((pstate_page = alloc_page(GFP_KERNEL)) == NULL)
		return -ENOMEM;
	para_state = (struct kvm_vcpu_para_state*)page_address(pstate_page);
#else
	para_state =  &per_cpu(para_state, cpu);
#endif
        /*
         * Try to write to a magic MSR (which is invalid on any real CPU),
         * and thus signal to KVM that we wish to entering para-virtualized
         * mode:
         */
        para_state->guest_version = KVM_PARA_API_VERSION;
        para_state->host_version = -1;
        para_state->size = sizeof(*para_state);
        para_state->ret = -1;

	hypercall_addr_page = vmalloc_to_page(hypercall_addr);
	para_state->hypercall_gpa = page_to_pfn(hypercall_addr_page) << PAGE_SHIFT | 
				    offset_in_page(hypercall_addr);
	printk(KERN_DEBUG "kvm guest: hypercall gpa is 0x%lx\n", (long)para_state->hypercall_gpa);

        if (wrmsr_safe(MSR_KVM_API_MAGIC, __pa(para_state), 0)) {
                printk(KERN_INFO "KVM guest: WRMSR probe failed.\n");
                return -1;
        }

        printk(KERN_DEBUG "kvm guest: host returned %d\n", para_state->ret);
        printk(KERN_DEBUG "kvm guest: host version: %d\n", para_state->host_version);
        printk(KERN_DEBUG "kvm guest: syscall entry: %02x %02x %02x %02x\n",
                        hypercall_addr[0], hypercall_addr[1],
                        hypercall_addr[2], hypercall_addr[3]);

        if (para_state->ret) {
                printk(KERN_ERR "kvm guest: host refused registration.\n");
                return -1;
        }
        test_hypercall();

        return 0;
}

#endif

/*
 * The higher levels take care of making this non-re-entrant (it's
 * called with bh's disabled).
 */
static int kvmnet_guest_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct kvm_eth_state *eth_state = dev->priv;
	struct kvm_tx_ring_desc *tx_desc;
	unsigned int ret;

	trace_hyper((unsigned long)skb, eth_state->tx_head, eth_state->tx_tail);
	WARN_ON(eth_state->magic != KVM_ETH_MAGIC);

	/* Note that caller will free the skb on error so there is no leak*/
	if (!netif_running(dev)) {
		pr_debug("netdev %s is not running, very weird!\n",
			 dev->name);
		WARN_ON(1);
		return -EFAULT;
	}

	tx_desc = get_head_tx_desc(eth_state);
	tx_desc->size = skb->len;
	memcpy(__va(tx_desc->page_gpa), skb->data, skb->len);
	commit_head_tx_desc(eth_state, tx_desc);
	dev_kfree_skb(skb);

	ret = hypercall(0, __NR_hypercall_send_eth);
	WARN_ON(ret);
	pr_debug("hypercall ret: %d\n", ret);

	return 0;
}

static int kvmnet_guest_rx(struct net_device *netdev, int quota)
{
	struct kvm_eth_state *eth_state = netdev->priv;
	struct kvm_rx_ring_desc *rx_desc;
	unsigned char *buff_gva;
	struct sk_buff *skb = NULL;
	int work_done = 0;

	pr_debug("kvmnet: got quota=%d\n", quota);
	WARN_ON(eth_state->magic != KVM_ETH_MAGIC);

	if (!netif_running(netdev)) {
		pr_debug("netdev %s is not running, not sending!\n",
			netdev->name);
		WARN_ON(1);
		return work_done;
	}

	pr_debug("kvmnet: rx ring head: %d\n", eth_state->rx_head);
	pr_debug("kvmnet: rx ring tail: %d\n", eth_state->rx_tail);

	while (eth_state->rx_tail != eth_state->rx_head && quota--) {

		work_done++;
		pr_debug(".. iteration #%03d\n", work_done);
		rx_desc = get_tail_rx_desc(eth_state);
		pr_debug(".. rx_desc->size:     %d\n", rx_desc->size);
		pr_debug(".. rx_desc->page_gpa: %08Lx\n", rx_desc->page_gpa);
		buff_gva = __va(rx_desc->page_gpa);
		pr_debug(".. buff_gva: %p\n", buff_gva);
		skb = netdev_alloc_skb(netdev, rx_desc->size);
		if (!skb) {
			WARN_ON(1);
			break;
		}
		skb_put(skb, rx_desc->size);
		memcpy(skb->data, buff_gva, rx_desc->size);

		skb->dev = netdev;
		skb->protocol = eth_type_trans(skb, netdev);
		skb->ip_summed = CHECKSUM_UNNECESSARY;

		trace_hyper((unsigned long)skb, eth_state->rx_head, eth_state->rx_tail);
		print_skb(skb);
		netif_rx_ni(skb);

		free_tail_rx_desc(eth_state, rx_desc);
		pr_debug(".. eth_state->rx_head: %d\n", eth_state->rx_head);
		pr_debug(".. eth_state->rx_tail: %d\n", eth_state->rx_tail);
	}
	WARN_ON(!quota);
	
        pr_debug(".. The end. work_done=%d\n", work_done);

	//stats->rx_packets += work_done;

	return work_done;
}

static irqreturn_t kvmnet_guest_rx_irq_handler(int irq, void *dev_id)
{
	struct net_device *netdev = dev_id;
	struct kvm_eth_state *eth_state = netdev->priv;

	trace_hyper(irq, 0, 0);
	pr_debug("kvmnet: got irq, netdev: %p, eth_state: %p\n",
			netdev, eth_state);
	WARN_ON(eth_state->magic != KVM_ETH_MAGIC);

#if 0
	/*
	 * Test whether the host issued the irq. Use gen numbers for communication
	 */
	rmb();
	if (eth_state->irq_gen_host <= eth_state->irq_gen_guest &&
	    eth_state->irq_gen_guest - eth_state->irq_gen_host > 0xfff) /*wrap around */{
		pr_debug("kvmnet:not our irq\n");
		WARN_ON(1);
		return IRQ_NONE;
	}
	eth_state->irq_gen_guest++;
#endif

	if (!netif_running(netdev)) {
		pr_debug("netdev %s is not running, not receiving!\n",
			netdev->name);
		WARN_ON(1);
		return IRQ_HANDLED;
	}

	pr_debug("kvmnet: rx ring head: %d\n", eth_state->rx_head);
	pr_debug("kvmnet: rx ring tail: %d\n", eth_state->rx_tail);

	atomic_set(&eth_state->irq_enable, 1);
	kvmnet_guest_rx(netdev, KVM_ETH_RX_RING_SIZE);
#if 0
	if (likely(netif_rx_schedule_prep(netdev))) {
		atomic_set(&eth_state->irq_enable, 0);
		__netif_rx_schedule(netdev);
	} else
		atomic_set(&eth_state->irq_enable, 1);
#endif

	return IRQ_HANDLED;
}

static int kvmnet_guest_poll(struct net_device *netdev, int *budget)
{
	struct kvm_eth_state *eth_state = netdev->priv;
	int orig_budget = min(*budget, netdev->quota);
	int done = 1;
	int work_done;

	pr_debug("got budget=%d, orig_budget=%d\n", *budget, orig_budget);

	work_done = kvmnet_guest_rx(netdev, orig_budget);
	pr_debug("work_done=%d\n", work_done);
	if (likely(work_done > 0)) {
		*budget -= work_done;
		netdev->quota -= work_done;
		done = (work_done < orig_budget);
	}

	if (done) {
		pr_debug("deschedule pooling\n");
		// re-enable interrupts after poll
		atomic_set(&eth_state->irq_enable, 1);
		netif_rx_complete(netdev);
	}
	pr_debug("End of functions\n");

	return !done;
}

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
struct net_device *kvmnet_dev;

struct kvm_eth_state *eth_state;

static int kvmnet_register_guest(struct net_device *netdev)
{
	int ret, i;
	gpa_t eth_state_gpa;
	struct page *eth_state_page;

	eth_state_page = vmalloc_to_page(eth_state);
	WARN_ON(eth_state_page == NULL);
	if (!eth_state_page)
		return -1;

	netdev->priv = eth_state;
	printk(KERN_INFO "registering netdev %p, eth_state: %p\n",
		netdev, netdev->priv);

	netdev->dev_addr[0] = 0x00;
	netdev->dev_addr[1] = 0x11;
	netdev->dev_addr[2] = 0x22;
	netdev->dev_addr[3] = 0x33;
	netdev->dev_addr[4] = 0x44;
	netdev->dev_addr[5] = 0x55;

	eth_state->tx_head = 0;
	eth_state->tx_tail = 0;

	for (i = 0; i < KVM_ETH_TX_RING_SIZE; i++) {
		unsigned long page_gva;

		page_gva = __get_free_page(GFP_KERNEL | __GFP_ZERO);

		WARN_ON(!page_gva);
		WARN_ON((int)(page_gva != (unsigned long)__va(__pa(page_gva))));
		eth_state->tx_ring[i].page_gpa = __pa(page_gva);
	}

	eth_state->rx_head = 0;
	eth_state->rx_tail = 0;

	eth_state->magic = KVM_ETH_MAGIC;

	//enable irqs
	atomic_set(&eth_state->irq_enable, 1);

	for (i = 0; i < KVM_ETH_RX_RING_SIZE; i++) {
		unsigned long page_gva;

		page_gva = __get_free_page(GFP_KERNEL | __GFP_ZERO);

		WARN_ON(!page_gva);
		WARN_ON((int)(page_gva != (unsigned long)__va(__pa(page_gva))));
		eth_state->rx_ring[i].page_gpa = __pa(page_gva);
	}

	eth_state_gpa = page_to_pfn(eth_state_page) << PAGE_SHIFT |
				    offset_in_page(eth_state);

	ret = hypercall(1, __NR_hypercall_register_eth, eth_state_gpa);
	WARN_ON(ret);

	netdev->hard_start_xmit = kvmnet_guest_xmit;
//	netdev->poll = kvmnet_guest_poll;
	strcpy(netdev->name, "kvmnet-guest");
	ret = request_irq(eth_state->irq, kvmnet_guest_rx_irq_handler,
		IRQF_SHARED | IRQF_SAMPLE_RANDOM, "kvmnet-guest",
		netdev);
	WARN_ON(ret);
	set_irq_chip_and_handler_name(eth_state->irq, &dummy_irq_chip,
		handle_simple_irq, "kvm");

	return ret;
}


#define KVMNET_DRIVER_NAME "paravirt_network_driver"
#define KVMNET_DRIVER_VERSION "1"
#define PCI_VENDOR_ID_KVMNET 0x5002
#define PCI_DEVICE_ID_KVMNET 0x1234

static struct pci_device_id kvmnet_pci_tbl[] = {
//	{PCI_VENDOR_ID_KVMNET, PCI_DEVICE_ID_KVMNET, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{PCI_ANY_ID, PCI_ANY_ID, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{0,}
};
MODULE_DEVICE_TABLE (pci, kvmnet_pci_tbl);

static int __devinit kvmnet_init_one(struct pci_dev *pdev,
				     const struct pci_device_id *ent)
{
	u8 pci_rev;
	struct net_device *netdev;
	int rs;

	WARN_ON(sizeof(struct kvm_eth_state) > PAGE_SIZE);

	printk(KERN_INFO "%s size of kvm_eth_state is %d\n", __FUNCTION__,
		(int)sizeof(struct kvm_eth_state));
	netdev = alloc_etherdev(sizeof(struct kvm_eth_state));
	kvmnet_dev = netdev;

	netdev->ethtool_ops = &kvmnet_ethtool_ops;
	netdev->flags &= ~IFF_MULTICAST;
	netdev->mtu = 1500;
	netdev->weight = 16;

//#warning fixme

	if (!paravirt_enabled()) {
		printk(KERN_ERR "%s failed registering para\n", __FUNCTION__);
		return -1;
	}

	if (eth_state) {
		printk(KERN_ERR "kvmnet-guest already registered\n");
		return -1;
	}

	eth_state = vmalloc(sizeof(struct kvm_eth_state));
	WARN_ON(!eth_state);
	if (!eth_state)
		return -1;

	if (pdev->irq)
		eth_state->irq = pdev->irq;
	else
		eth_state->irq = 10;

	printk(KERN_DEBUG "%s: irq is %d\n", __FUNCTION__, pdev->irq);

	if (kvmnet_register_guest(netdev) < 0)
		return -1;

	netdev->tx_queue_len = KVM_ETH_TX_RING_SIZE-1;

	pci_read_config_byte(pdev, PCI_REVISION_ID, &pci_rev);

	if (pdev->vendor == PCI_VENDOR_ID_KVMNET &&
	    pdev->device == PCI_DEVICE_ID_KVMNET) {
		printk(KERN_INFO "pci dev %s (id %04x:%04x rev %02x) is a "
		       "guest paravirt network device\n",
		       pci_name(pdev), pdev->vendor, pdev->device, pci_rev);
	}

        rs = register_netdev(netdev);
	netif_wake_queue(netdev);
	netif_poll_enable(netdev);

	return 0;
}

static void __devexit kvmnet_remove_one(struct pci_dev *pdev)
{
	pci_disable_device(pdev);
	synchronize_irq(eth_state->irq);
	free_irq(eth_state->irq, kvmnet_dev);
	vfree(eth_state);
	eth_state = NULL;
}


static struct pci_driver kvmnet_pci_driver = {
	.name		= KVMNET_DRIVER_NAME,
	.id_table	= kvmnet_pci_tbl,
	.probe		= kvmnet_init_one,
	.remove		= __devexit_p(kvmnet_remove_one),
};


int __init kvmnet_init(void)
{
	if (paravirt_enabled())
		pci_module_init(&kvmnet_pci_driver);

	pr_debug("Finished registering & wakeup + pool guest netdev\n");
	return 0;
}

static void __exit kvmnet_exit(void)
{
	netif_poll_disable(kvmnet_dev);
	netif_stop_queue(kvmnet_dev);
	unregister_netdev(kvmnet_dev);

	pci_unregister_driver(&kvmnet_pci_driver);
	free_netdev(kvmnet_dev);
}

module_init(kvmnet_init);
module_exit(kvmnet_exit);
