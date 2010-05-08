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

#include "kvm.h"
#include "kvm_block.h"

struct kvm_blockhost {
	int nr;
	int irq;
	uint64_t size;
	struct file 	*filp;
	uint64_t disk_completion_page_hpa;
	uint64_t disk_submit_page_hpa;
	struct kvm_vcpu *vcpu_slots;
	struct delayed_work slot_work;
};


#define MAX_DISK 16
#define SLOT_COUNT (PAGE_SIZE/sizeof(uint64_t))

/* irq mitigation constant */
#define WRITE_IRQ_DELAY 20


static struct kvm_blockhost block[MAX_DISK];


static void run_slots(struct work_struct *dummy);


static int valid_disk(int nr)
{
	if (nr<0)
		return 0;
	if (nr>=MAX_DISK)
		return 0;
	if (!block[nr].filp)
		return 0;
	return 1;
}


int hypercall_disk_register_completion_area(struct kvm_vcpu *vcpu,
			int nr, unsigned long completion_page_gpa, int irq)
{
	printk("KVM Block device registered at irq %i\n", irq);
	if (nr < 0)
		return -EFAULT;
	if (nr >= MAX_DISK)
		return -EFAULT;
	block[nr].disk_completion_page_hpa = gpa_to_hpa(vcpu, completion_page_gpa);
	block[nr].irq = irq;
	block[nr].vcpu_slots= vcpu;
	INIT_DELAYED_WORK(&block[nr].slot_work,  run_slots);
	block[nr].nr = nr;
	return 0;
}

int hypercall_disk_register_submit_area(struct kvm_vcpu *vcpu, int nr,
			unsigned long submit_page_gpa, int irq)
{
	if (nr < 0)
		return -EFAULT;
	if (nr >= MAX_DISK)
		return -EFAULT;

	block[nr].disk_submit_page_hpa = gpa_to_hpa(vcpu, submit_page_gpa);
	block[nr].irq = irq;
	block[nr].vcpu_slots= vcpu;


	return 0;
}



int hypercall_disk_read(struct kvm_vcpu *vcpu, int nr, unsigned long start_sector, unsigned long nr_sectors, unsigned long buf_gpa)
{
	unsigned char *buff_hva;
	hpa_t buff_hpa;
	int ret = 0;

	if (!valid_disk(nr))
		return -EINVAL;

	/* the synchronous interface doesn't allow > 4Kb */
	if (nr_sectors>8)
		return -EIO;

        vcpu_put(vcpu);

	buff_hpa = gpa_to_hpa(vcpu, buf_gpa);
	ret = is_error_hpa(buff_hpa) ? -EFAULT : 0;

	if (ret<0)
		goto out;

        buff_hva = kmap(pfn_to_page(buff_hpa >> PAGE_SHIFT));
	kernel_read(block[nr].filp,  start_sector*512, buff_hva + (buff_hpa&4095), nr_sectors*512);
	kunmap(pfn_to_page(buff_hpa >> PAGE_SHIFT));

out:
        vcpu_load(vcpu);
	return ret;
}

/* FIXME: this probably should go to the generic kernel, just like kernel_read is */
static int kernel_write(struct file *file, unsigned long offset,
        char *addr, unsigned long count)
{
        mm_segment_t old_fs;
        loff_t pos = offset;
        int result;

        old_fs = get_fs();
        set_fs(get_ds());
        /* The cast to a user pointer is valid due to the set_fs() */
        result = vfs_write(file, (void __user *)addr, count, &pos);
        set_fs(old_fs);
        return result;
}

int hypercall_disk_write(struct kvm_vcpu *vcpu, int nr, unsigned long start_sector,
		unsigned long nr_sectors, unsigned long buf_gpa)
{
	unsigned char *buff_hva;
	hpa_t buff_hpa;
	int ret = 0;

	if (!valid_disk(nr))
		return -EINVAL;

	/* synchronous interface does not allow more than 4Kb at a time */
	if (nr_sectors>8)
		return -EIO;

        vcpu_put(vcpu);

	buff_hpa = gpa_to_hpa(vcpu, buf_gpa);
	ret = is_error_hpa(buff_hpa) ? -EFAULT : 0;

	if (ret<0)
		goto out;

        buff_hva = kmap(pfn_to_page(buff_hpa >> PAGE_SHIFT));
	ret = kernel_write(block[nr].filp,  start_sector*512,  buff_hva + (buff_hpa&4095), nr_sectors*512);
	kunmap(pfn_to_page(buff_hpa >> PAGE_SHIFT));

out:
        vcpu_load(vcpu);
	return ret;
}


/* we can use one big vmap if all entries are PAGE_SIZE in size */
static int can_vmap(struct kvm_iopage *iopage)
{
	int i;
	for (i=0; i < iopage->entries; i++)
		if (iopage->vector[i].data_len != PAGE_SIZE)
			return 0;
	return 1;
}



static int do_disk_vector_async(struct kvm_vcpu *vcpu, int nr, unsigned long iopage_gpa, unsigned long slot)
{
	hpa_t	iopage_hpa;
	hpa_t	data_hpa;
	struct kvm_iopage *iopage_hva;
	unsigned long start_data;
	int i;
	uint64_t *completion;
	int read=0;

        /* first, get the vector mapped into our address space */
	iopage_hpa = gpa_to_hpa(vcpu, iopage_gpa);
	if (is_error_hpa(iopage_hpa))
		return 1;

	iopage_hva = kmap(pfn_to_page(iopage_hpa >> PAGE_SHIFT));

	start_data = iopage_hva->sector * 512;

	/* if we can use vmap that's fastest */
	if (can_vmap(iopage_hva)) {
		struct page **page_array;
		void *address;
		int i;
		page_array = kmalloc(sizeof(struct page*) * iopage_hva->entries, GFP_KERNEL);

		if (!page_array)
			goto iterate;

		for (i=0; i < iopage_hva->entries; i++) {
			hpa_t hpa;
			hpa =  gpa_to_hpa(vcpu, iopage_hva->vector[i].data_gpa);
			page_array[i] = pfn_to_page(hpa >> PAGE_SHIFT);
		}
		address = vmap(page_array, iopage_hva->entries,  VM_MAP, PAGE_KERNEL);
		if (!address) {
			kfree(page_array);
			goto iterate;
		}
		if (iopage_hva->write)
			kernel_write(block[nr].filp, start_data, address, iopage_hva->entries * PAGE_SIZE);
		else {
			kernel_read(block[nr].filp, start_data, address, iopage_hva->entries * PAGE_SIZE);
			read = 1;
		}
		vunmap(address);
		kfree(page_array);
	} else {
iterate:
	/* now, iterate over the buffer */
		for (i = 0; i < iopage_hva->entries; i++) {
			char *data_hva;

			data_hpa =  gpa_to_hpa(vcpu, iopage_hva->vector[i].data_gpa);
		        data_hva = kmap(pfn_to_page(data_hpa >> PAGE_SHIFT));
			if (iopage_hva->write)
				kernel_write(block[nr].filp, start_data, data_hva + (data_hpa&4095), iopage_hva->vector[i].data_len);
			else
				kernel_read(block[nr].filp, start_data, data_hva + (data_hpa&4095), iopage_hva->vector[i].data_len);
			start_data += iopage_hva->vector[i].data_len;
			kunmap(pfn_to_page(data_hpa >> PAGE_SHIFT));
		}
	}
	kunmap(pfn_to_page(((unsigned long)iopage_hva) >> PAGE_SHIFT));


	completion = (uint64_t*)kmap_atomic(pfn_to_page(block[nr].disk_completion_page_hpa >> PAGE_SHIFT), KM_USER0);
	completion[slot] = iopage_gpa;
	kunmap_atomic(completion, KM_USER0);

	/* reads should not delay the irq handling */
	return 1 + read*WRITE_IRQ_DELAY;
}


static void raise_irq(struct kvm_vcpu *vcpu, int irq)
{
	irq += 32;
        vcpu_load(vcpu);
	set_bit(irq, vcpu->irq_pending);
	set_bit(irq / BITS_PER_LONG, &vcpu->irq_summary);
	vcpu_put(vcpu);
}


static void run_slots(struct work_struct *works)
{
	int i;
	struct kvm_blockhost *host;
	uint64_t iop;
	int work = 0;
	int need_irq = 0;
	struct kvm_vcpu *vcpu = NULL;
	uint64_t *iopage_slots;

	host = container_of(works, struct kvm_blockhost, slot_work.work);

	if (!host->disk_submit_page_hpa)
		return;

	vcpu = host->vcpu_slots;

	iopage_slots = kmap(pfn_to_page(host->disk_submit_page_hpa >> PAGE_SHIFT));

	for (i=0; i< SLOT_COUNT; i++) {
		if (iopage_slots[i]!=0) {
			iop = xchg(&iopage_slots[i],0);
			if (iop) {
				need_irq += do_disk_vector_async(vcpu, host->nr, iop, i);
				work++;
			}
		}
		if (need_irq>WRITE_IRQ_DELAY) {
			need_irq = 0;
			raise_irq(vcpu, host->irq);
		}
	}

	if (need_irq)
		raise_irq(vcpu, host->irq);
	kunmap(pfn_to_page(host->disk_submit_page_hpa >> PAGE_SHIFT));
}



int hypercall_disk_trigger_scan(int nr)
{

	if (!valid_disk(nr))
		return -EINVAL;

	schedule_delayed_work_on(0, &block[nr].slot_work, 0);
	return 0;
}

int hypercall_disk_getdiskcount(void)
{
	return 1; /* hardcoded to 1 disk for now */
}

int hypercall_disk_getinfo(struct kvm_vcpu *vcpu, unsigned long number, unsigned long info_gpa)
{
	unsigned char *info_hva;
	struct inode *inode;
	hpa_t info_hpa;
	int ret = 0;
	struct kvm_disk_info info;



	if (number<0)
		return -EINVAL;
	if (number >= MAX_DISK)
		return -EINVAL;



	if (!block[number].filp) {
		block[number].filp = filp_open("/tmp/hyperfile",O_RDWR | O_LARGEFILE, 0600);
		if (IS_ERR(block[number].filp))
			block[number].filp = NULL;
	}

	if (block[number].filp == NULL) {
		printk(KERN_ERR "kvm_block: filp open failed: %p -- %li \n", block[number].filp, - (unsigned long)block[number].filp);
		return -EPERM;
	}

	inode = block[number].filp->f_path.dentry->d_inode;
	block[number].size = inode->i_size;



	info_hpa = gpa_to_hpa(vcpu, info_gpa);
	ret = is_error_hpa(info_hpa) ? -EFAULT : 0;
	if (ret)
		return ret;



	/* must be page aligned */
	if (info_hpa&4095)
		return -EINVAL;


	memset(&info, 0, sizeof(struct kvm_disk_info));
	info.sectors = block[number].size / 512;

        info_hva = kmap_atomic(pfn_to_page(info_hpa >> PAGE_SHIFT), KM_USER0);
        memcpy(info_hva, &info, sizeof(struct kvm_disk_info));
	kunmap_atomic(pfn_to_page(info_hpa >> PAGE_SHIFT), KM_USER0);


	return ret;

}
