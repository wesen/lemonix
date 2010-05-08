/*
 * KVM virtual block device driver for Linux
 *
 * (C) Copyright 2007 Intel Corporation
 *
 * This file is licensed to you under the terms of the GNU General Public License
 * (GPL) version 2. See the COPYING file in the main directory of the kernel for
 * the license text.
 *
 * Authors:
 *	Arjan van de Ven <arjan@linux.intel.com>
 *
 */

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/blkdev.h>
#include <linux/kvm_para.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/list.h>

#include <asm/hypercall.h>

#include "kvm_block.h"

static int kvm_block_major;

#define DISK_IRQ	9
#define SLOT_COUNT	(PAGE_SIZE/sizeof(uint64_t))

struct kvm_block_dev {
	struct list_head 	list;
	uint64_t		size;
	spinlock_t		lock;
	struct tasklet_struct	tasklet;
	int			nr;

	struct request_queue 	*queue;
	struct gendisk 		*gd;

	/*
	 * the following to pages are shared with the hypervisor; the completion page
	 * is used by the hypervisor to place completed IO's in, while the submit page
	 * is used by this guest driver to place IOs to be done in. In both cases the
	 * pages are arrays of uint64_t's that contain the guest physical address of
	 * a struct kvmio_page. These arrays are indexed by "slot number", which is
	 * TCQ identifier and unique for each IO. struct kvm_iopage's have to
	 * be page aligned!
	 */
	uint64_t	*completion_page;
	uint64_t	*submit_page;
};


/* highest slot used.. to short-change scanning  */
static int highest_slot;

static LIST_HEAD(kvm_devices);

static int kvm_get_disk_count(void)
{
	int ret;
	ret = hypercall(0, __NR_hypercall_disk_count);
	return ret;
}

static int kvm_get_disk_info(int nr, struct kvm_disk_info *info)
{
	int ret;
	ret = hypercall(2, __NR_hypercall_disk_info, nr, __pa((unsigned long) info));
	return ret;
}


static int kvm_register_completion_page(struct kvm_block_dev *dev)
{
	int ret;
	unsigned long irq = DISK_IRQ;
	ret = hypercall(3, __NR_hypercall_disk_register_completion_area, dev->nr,  __pa(dev->completion_page), irq);
	if (ret) {
		printk(KERN_ERR "kvm-block: Hypervisor registration of the completion page failed : %i / %i \n", ret, -ret);
		return ret;
	}

	ret = hypercall(3, __NR_hypercall_disk_register_submit_area, dev->nr,  __pa(dev->submit_page), irq);
	if (ret)
		printk(KERN_ERR "kvm-block: Hypervisor registration of the submit page failed : %i / %i \n", ret, -ret);
	return ret;
}


/*
 * scan the completion page for completed IO's and process them
 * returns the number of completed IO's
 */
static int kvm_scan_completion(struct kvm_block_dev *dev)
{
	int i;
	int ret = 0;
	struct kvm_iopage *iopage;
	unsigned long completed;
	unsigned long flags;

	for (i=0; i <= highest_slot; i++) {
		if (dev->completion_page[i]) {
			struct request *req;
			cpu_relax();
			completed = dev->completion_page[i]; /* FIXME: atomic swap */
			dev->completion_page[i] = 0;
			iopage = (struct kvm_iopage*) __va(completed);

			req = (struct request *)(unsigned long)iopage->guest_data;

			if (dev != req->rq_disk->private_data) {
				printk(KERN_ERR "kvm-block: fatal hypervisor completion mismatch \n");
				BUG();
			}
			spin_lock_irqsave(&dev->lock, flags);
			if (!end_that_request_first(req, 1, iopage->sector_count)) {
				add_disk_randomness(req->rq_disk);
				blk_queue_end_tag(dev->queue, req);
				end_that_request_last(req, 1);
			}
			spin_unlock_irqrestore(&dev->lock, flags);
			free_page((unsigned long)iopage);
			ret++;
		}
	}
	/* the queue may have been stopped; if we completed things it can start again */
	if (ret) {
		spin_lock_irqsave(&dev->lock, flags);
		blk_start_queue(dev->queue);
		spin_unlock_irqrestore(&dev->lock, flags);
	}
	return ret;
}

static void kvm_tasklet(unsigned long data)
{
	struct kvm_block_dev *dev = (struct kvm_block_dev*)data;
	kvm_scan_completion(dev);
}


/*
 * synchronous IO submission path -- fallback for low memory situations
 */

static int kvm_transfer_synchronous(struct kvm_block_dev *dev, unsigned long start_sector, unsigned long nr_sectors, uint64_t buffer, int write, struct bio *bio)
{
	unsigned long ret;
	if (write) {
		ret = hypercall(4, __NR_hypercall_disk_write, dev->nr, start_sector, nr_sectors, (unsigned long)buffer);
		if (ret)
			printk(KERN_ERR "kvm-block: hypercall write failed : %li / %li\n", ret, -ret);
	} else {
		ret = hypercall(4, __NR_hypercall_disk_read, dev->nr, start_sector, nr_sectors, (unsigned long)buffer);
		if (ret)
			printk(KERN_ERR "kvm-block: hypercall read failed: %li / %li \n", ret, -ret);
	}

	return 0;
}

static int xfer_bio_synchronous(struct kvm_block_dev *dev, struct bio *bio)
{
	int i;
	struct bio_vec *vec;
	sector_t sector = bio->bi_sector;

	bio_for_each_segment(vec, bio, i) {
		uint64_t phys = page_to_phys(bio_iovec_idx((bio), (i))->bv_page) + bio_iovec_idx((bio), (i))->bv_offset;

		kvm_transfer_synchronous(dev, sector, bio_cur_sectors(bio), phys, bio_data_dir(bio)==WRITE, bio);
		sector += bio_cur_sectors(bio);
	}
	return 0;
}

static int xfer_request_synchronous(struct kvm_block_dev *dev, struct request *req)
{
	struct bio *bio;
	int sectors = 0;

	rq_for_each_bio(bio, req) {
		xfer_bio_synchronous(dev, bio);
		sectors += bio->bi_size/512;
	}
	return sectors;
}

/* asynchronous IO submission routines */

static int kvm_transfer_vector(struct kvm_block_dev *dev, unsigned long buffer, int slot)
{
	dev->submit_page[slot] = __pa(buffer);
	return 0;
}

/*
 * tickle the host to look at the array of submitted pages
 */
static int kvm_trigger_scan(struct kvm_block_dev *dev)
{
	int ret;
	ret = hypercall(1, __NR_hypercall_disk_trigger_scan, dev->nr);
	if (ret)
		printk(KERN_ERR "kvm-block: trigger error: %i / %i ", ret, -ret);
	return ret;
}


/*
 * fill in the vector array of the io description page
 */
static int bio_vector(struct kvm_block_dev *dev, struct bio *bio, struct kvm_iopage *iopage, int *pos, int *sectors)
{
	int i;
	struct bio_vec *vec;

	bio_for_each_segment(vec, bio, i) {
		uint64_t phys = page_to_phys(bio_iovec_idx((bio), (i))->bv_page) + bio_iovec_idx((bio), (i))->bv_offset;
		iopage->vector[*pos].data_gpa = phys;
		iopage->vector[*pos].data_len = bio_cur_sectors(bio)*512;
		(*pos)++;
		(*sectors)+= bio_cur_sectors(bio);
	}
	return 0;
}

static int xfer_request_vector(struct kvm_block_dev *dev, struct request *req, char *page)
{
	struct bio *bio = NULL;
	struct kvm_iopage *iopage = (struct kvm_iopage *)page;
	int pos = 0, sectors = 0;

	rq_for_each_bio(bio, req) {
		bio_vector(dev, bio, iopage, &pos, &sectors);
	}
	iopage->sector = req->sector;
	iopage->guest_data = (unsigned long)req;
	iopage->sector_count = sectors;
	iopage->write = (rq_data_dir(req)==WRITE);
	iopage->entries = pos;

	kvm_transfer_vector(dev, (unsigned long) iopage, req->tag);

	return 0;
}

static void kvm_block_request(request_queue_t *queue)
{
	struct request *req;
	struct kvm_block_dev *dev = NULL;
	int scan = 0;

	while ( (req = elv_next_request(queue)) != NULL) {
		dev = req->rq_disk->private_data;

		if (!blk_fs_request(req)) {
			end_request(req, 0);
			printk("kvm-block: non-fs request received; aborting request\n");
		} else {
			char *page = NULL;

			page = (char*)__get_free_page(GFP_ATOMIC);
			if (page) {
				if (blk_queue_start_tag(dev->queue, req) >= 0) {
					if (req->tag > highest_slot)
						highest_slot = req->tag;
					xfer_request_vector(dev, req, page);
					scan++;
				} else
					blk_stop_queue(dev->queue);
			} else {
				int sectors;
				/*
				 * fall back to synchronous IO to prevent
				 * OOM deadlock
				 */
				sectors = xfer_request_synchronous(dev, req);
				if (!end_that_request_first(req, 1, sectors)) {
					add_disk_randomness(req->rq_disk);
					blkdev_dequeue_request(req);
					end_that_request_last(req, 1);
				}
			}
		}
	}
	/* trigger the host to scan the submit queue */
	if (scan && dev)
		kvm_trigger_scan(dev);
}


static int kvm_disk_open(struct inode *inode, struct file *filp)
{
	struct kvm_block_dev *dev = inode->i_bdev->bd_disk->private_data;
	filp->private_data = dev;
	return 0;
}

static int kvm_disk_release(struct inode *inode, struct file *filp)
{
/*	struct kvm_block_dev *dev = inode->i_bdev->bd_disk->private_data; */


	return 0;
}

static struct block_device_operations kvm_bops = {
	.owner	 = THIS_MODULE,
	.open	 = kvm_disk_open,
	.release = kvm_disk_release,
};

static irqreturn_t kvm_block_irq_handler(int irq, void *dev_id)
{
	struct kvm_block_dev *dev = dev_id;

	/* do all the work in a tasklet */
	tasklet_schedule(&dev->tasklet);
	return IRQ_HANDLED;
}



int kvm_register_disk(int disknr)
{
	struct kvm_block_dev *dev;
	int ret;
	struct kvm_disk_info *info;

	info = (void*)get_zeroed_page(GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	ret = kvm_get_disk_info(disknr, info);
	if (ret)
		printk("kvm-block: failed to get disk info: %i \n", ret);

	dev = kmalloc(sizeof(struct kvm_block_dev), GFP_KERNEL);
	if (!dev) {
		free_page((unsigned long)info);
		return -ENOMEM;
	}

	dev->nr = disknr;
	dev->completion_page = (void*)get_zeroed_page(GFP_KERNEL);
	dev->submit_page = (void*)get_zeroed_page(GFP_KERNEL);

	if (!dev->completion_page)
		goto error;
	if (!dev->submit_page)
		goto error;

	if (kvm_register_completion_page(dev))
		goto error;


	spin_lock_init(&dev->lock);
	dev->queue = blk_init_queue(kvm_block_request, &dev->lock);
	blk_queue_max_phys_segments(dev->queue, 126);
	blk_queue_max_sectors(dev->queue, 126*8);
	blk_queue_init_tags(dev->queue, SLOT_COUNT, NULL);
	dev->gd = alloc_disk(64);
	dev->gd->queue = dev->queue;
	dev->gd->major = kvm_block_major;
	dev->gd->fops = &kvm_bops;
	dev->gd->private_data = dev;
	dev->gd->first_minor = 64*disknr;
	dev->gd->minors = 64;

	tasklet_init(&dev->tasklet, kvm_tasklet, (unsigned long)dev);

	ret = request_irq(DISK_IRQ, kvm_block_irq_handler, IRQF_SHARED | IRQF_SAMPLE_RANDOM, "kvm-block", dev);
	WARN_ON(ret);
	set_irq_chip_and_handler_name(DISK_IRQ, &dummy_irq_chip, handle_simple_irq, "kvm");

	sprintf(dev->gd->disk_name, "vda");

	set_capacity(dev->gd, info->sectors);

	list_add(&dev->list, &kvm_devices);
	add_disk(dev->gd);
	free_page((unsigned long)info);
	return 0;

error:
	free_page((unsigned long)info);
	free_page((unsigned long)dev->completion_page);
	free_page((unsigned long)dev->submit_page);
	kfree(dev);
	return -ENOMEM;
}

void kvm_unregister_disk(struct kvm_block_dev *dev)
{
	list_del_init(&dev->list);
	del_gendisk(dev->gd);
	free_irq(DISK_IRQ, dev);
	kfree(dev);
}


int init_kvm_driver(void)
{
	int i,max;
	kvm_block_major = register_blkdev(0, "kvm-block");
	printk("KVM virtual block driver on major %i \n", kvm_block_major);
	if (kvm_block_major < 0)
		return -ENODEV;

	max = kvm_get_disk_count();
	for (i=0; i<max; i++)
		kvm_register_disk(i);

	return 0;
}

void exit_kvm_driver(void)
{
	struct kvm_block_dev *dev, *tmp;

	list_for_each_entry_safe(dev, tmp, &kvm_devices, list)
		kvm_unregister_disk(dev);
	unregister_blkdev(kvm_block_major, "kvm-block");
	kvm_block_major = 0;
}

module_init(init_kvm_driver);
module_exit(exit_kvm_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Arjan van de Ven <arjan@linux.intel.com>");
MODULE_DESCRIPTION("KVM coopvirtual block device driver");
