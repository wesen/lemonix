/*
 * User-space interface to the SPI bus on Atmel AT91RM9200
 *
 *  Copyright (C) 2003 SAN People (Pty) Ltd
 *
 * Based on SPI driver by Rick Bronson
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/highmem.h>
#include <linux/pagemap.h>
#include <asm/arch/spi.h>

#ifdef CONFIG_DEVFS_FS
#include <linux/devfs_fs_kernel.h>
#endif


#undef DEBUG_SPIDEV

/* ......................................................................... */

/*
 * Read or Write to SPI bus.
 */
static ssize_t spidev_rd_wr(struct file *file, char *buf, size_t count, loff_t *offset)
{
	unsigned int spi_device = (unsigned int) file->private_data;

	struct mm_struct * mm;
	struct page ** maplist;
	struct spi_transfer_list* list;
	int    pgcount;

	unsigned int ofs, pagelen;
	int res, i, err;

	if (!count) {
		return 0;
	}

	list = kmalloc(sizeof(struct spi_transfer_list), GFP_KERNEL);
	if (!list) {
		return -ENOMEM;
	}

	mm = current->mm;

	pgcount = ((unsigned long)buf+count+PAGE_SIZE-1)/PAGE_SIZE - (unsigned long)buf/PAGE_SIZE;

	if (pgcount >= MAX_SPI_TRANSFERS) {
		kfree(list);
		return -EFBIG;
	}

	maplist = kmalloc (pgcount * sizeof (struct page *), GFP_KERNEL);

	if (!maplist) {
		kfree(list);
		return -ENOMEM;
	}
	flush_cache_all();
	down_read(&mm->mmap_sem);
	err= get_user_pages(current, mm, (unsigned long)buf, pgcount, 1, 0, maplist, NULL);
	up_read(&mm->mmap_sem);

	if (err < 0) {
		kfree(list);
		kfree(maplist);
		return err;
	}
	pgcount = err;

#ifdef DEBUG_SPIDEV
	printk("spidev_rd_rw: %i %i\n", count, pgcount);
#endif

	/* Set default return value = transfer length */
	res = count;

	/*
	 * At this point, the virtual area buf[0] .. buf[count-1] will have
	 * corresponding pages mapped in the physical memory and locked until
	 * we unmap the kiobuf.  The pages cannot be swapped out or moved
	 * around.
	 */
	ofs = (unsigned long) buf & (PAGE_SIZE -1);
	pagelen = PAGE_SIZE - ofs;
	if (count < pagelen)
		pagelen = count;

	for (i = 0; i < pgcount; i++) {
		flush_dcache_page(maplist[i]);

		list->tx[i] = list->rx[i] = page_address(maplist[i]) + ofs;
		list->txlen[i] = list->rxlen[i] = pagelen;

#ifdef DEBUG_SPIDEV
		printk("  %i: %x  (%i)\n", i, list->tx[i], list->txlen[i]);
#endif

		ofs = 0;	/* all subsequent transfers start at beginning of a page */
		count = count - pagelen;
		pagelen = (count < PAGE_SIZE) ? count : PAGE_SIZE;
	}
	list->nr_transfers = pgcount;

	/* Perform transfer on SPI bus */
	spi_access_bus(spi_device);
	spi_transfer(list);
	spi_release_bus(spi_device);

	while (pgcount--) {
		page_cache_release (maplist[pgcount]);
	}
	flush_cache_all();

	kfree(maplist);
	kfree(list);

	return res;
}

static int spidev_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	int spi_device = MINOR(inode->i_rdev);

	if (spi_device >= NR_SPI_DEVICES)
		return -ENODEV;

	// TODO: This interface can be used to configure the SPI bus.
	// Configurable options could include: Speed, Clock Polarity, Clock Phase

	switch(cmd) {
		default:
			return -ENOIOCTLCMD;
	}
}

/*
 * Open the SPI device
 */
static int spidev_open(struct inode *inode, struct file *file)
{
	unsigned int spi_device = MINOR(inode->i_rdev);

	if (spi_device >= NR_SPI_DEVICES)
		return -ENODEV;

	/*
	 * 'private_data' is actually a pointer, but we overload it with the
	 * value we want to store.
	 */
	file->private_data = (void *)spi_device;

	return 0;
}

/*
 * Close the SPI device
 */
static int spidev_close(struct inode *inode, struct file *file)
{
	return 0;
}

/* ......................................................................... */

static struct file_operations spidev_fops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.read		= spidev_rd_wr,
	.write		= (int (*) (struct file *file, const char *buf, size_t count, loff_t *offset))spidev_rd_wr,
	.ioctl		= spidev_ioctl,
	.open		= spidev_open,
	.release	= spidev_close,
};

/*
 * Install the SPI /dev interface driver
 */
static int __init at91_spidev_init(void)
{
#ifdef CONFIG_DEVFS_FS
	int i;
#endif

	if (register_chrdev(SPI_MAJOR, "spi", &spidev_fops)) {
		printk(KERN_ERR "at91_spidev: Unable to get major %d for SPI bus\n", SPI_MAJOR);
		return -EIO;
	}

#ifdef CONFIG_DEVFS_FS
	devfs_mk_dir("spi");
	for (i = 0; i < NR_SPI_DEVICES; i++) {
		devfs_mk_cdev(MKDEV(SPI_MAJOR, i), S_IFCHR | S_IRUSR | S_IWUSR, "spi/%d",i);
	}
#endif
	printk(KERN_INFO "AT91 SPI driver loaded\n");

	return 0;
}

/*
 * Remove the SPI /dev interface driver
 */
static void __exit at91_spidev_exit(void)
{
#ifdef CONFIG_DEVFS_FS
	int i;
	for (i = 0; i < NR_SPI_DEVICES; i++) {
		devfs_remove("spi/%d", i);
	}

	devfs_remove("spi");
#endif

	if (unregister_chrdev(SPI_MAJOR, "spi")) {
		printk(KERN_ERR "at91_spidev: Unable to release major %d for SPI bus\n", SPI_MAJOR);
		return;
	}
}

module_init(at91_spidev_init);
module_exit(at91_spidev_exit);

MODULE_LICENSE("GPL")
MODULE_AUTHOR("Andrew Victor")
MODULE_DESCRIPTION("SPI /dev interface for Atmel AT91RM9200")
