/*
 * Flash memory access on AMD Alchemy evaluation boards
 *
 * $Id: alchemy-flash.c,v 1.2 2005/11/07 11:14:26 gleixner Exp $
 *
 * (C) 2003, 2004 Pete Popov <ppopov@embeddedalley.com>
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>

#include <asm/io.h>

#ifdef 	DEBUG_RW
#define	DBG(x...)	printk(x)
#else
#define	DBG(x...)
#endif

#define BOARD_MAP_NAME "SystemBase Flash"
#define BOARD_FLASH_SIZE_4  0x00400000 /* 4MB */
#define BOARD_FLASH_SIZE_8  0x00400000 /* 8MB */
#define BOARD_FLASH_START 0x10000000 /* 4MB */
#define BOARD_FLASH_WIDTH 2 /* 16-bits */


static struct map_info systembase_map_4 = {
	.name =	BOARD_MAP_NAME,
	.size = BOARD_FLASH_SIZE_4,
	.bankwidth = BOARD_FLASH_WIDTH,
	.phys = BOARD_FLASH_START,
};

static struct map_info systembase_map_8 = {
	.name =	BOARD_MAP_NAME,
	.size = BOARD_FLASH_SIZE_8,
	.bankwidth = BOARD_FLASH_WIDTH,
	.phys = BOARD_FLASH_START,
};

static struct mtd_partition systembase_partitions_4[] = {
	{
		.name = "Bootstrap",
		.size = 0x1000,
		.offset = 0x0000000,
		//               .mask_flags = MTD_WRITEABLE
	},{
		.name = "BootLoader Env",
		.size = 0x01000,
		.offset = MTDPART_OFS_APPEND,
	},{
		.name = "BootLoader",
		.size = 0x20000,
		.offset = MTDPART_OFS_APPEND,
		//                .mask_flags = MTD_WRITEABLE
	},{
		.name = "OS",
		.size = 0x150000,
		.offset = MTDPART_OFS_APPEND,
		//	.offset = 0x00020000
	},{
		.name = "Root Filesystem",
		.size = 0x280000,
		.offset = MTDPART_OFS_APPEND,
		//.offset = 0x00172000
	},{
		.name = "Config Filesystem",
			//.size = (BOARD_FLASH_SIZE-(0x00172000+0x264000)), /* last is config */
			.size = MTDPART_SIZ_FULL,
			//					.size = 0x10000,
			.offset = MTDPART_OFS_APPEND,
	}
};

static struct mtd_partition systembase_partitions_8[] = {
	{
		.name = "Bootstrap",
		.size = 0x1000,
		.offset = 0x0000000,
		//               .mask_flags = MTD_WRITEABLE
	},{
		.name = "BootLoader Env",
		.size = 0x01000,
		.offset = MTDPART_OFS_APPEND,
	},{
		.name = "BootLoader",
		.size = 0x20000,
		.offset = MTDPART_OFS_APPEND,
		//                .mask_flags = MTD_WRITEABLE
	},{
		.name = "OS",
		.size = 0x150000,
		.offset = MTDPART_OFS_APPEND,
		//	.offset = 0x00020000
	},{
		.name = "Root Filesystem",
		.size = 0x280000,
		.offset = MTDPART_OFS_APPEND,
		//.offset = 0x00172000
	},{
		.name = "Config Filesystem",
			//.size = (BOARD_FLASH_SIZE-(0x00172000+0x264000)), /* last is config */
			.size = 0x10000,
			//					.size = 0x10000,
			.offset = MTDPART_OFS_APPEND,
	},{
		.name = "Data Filesystem",
			//.size = (BOARD_FLASH_SIZE-(0x00172000+0x264000)), /* last is config */
			.size = MTDPART_SIZ_FULL,
			//					.size = 0x10000,
			.offset = MTDPART_OFS_APPEND,
	}
};

static struct mtd_info *mymtd;

int __init systembase_mtd_init(void)
{
	struct mtd_partition *parts;
	int nb_parts = 0;
	unsigned long window_addr;
	unsigned long window_size;

	/* Default flash buswidth */
	systembase_map_4.bankwidth = BOARD_FLASH_WIDTH;

	window_addr = 0x10000000; 
	window_size = BOARD_FLASH_SIZE_4;
#ifdef CONFIG_MIPS_MIRAGE_WHY
	/* Boot ROM flash bank only; no user bank */
	window_addr = 0x1C000000;
	window_size = 0x04000000;
	/* USERFS from 0x1C00 0000 to 0x1FC00000 */
	systembase_partitions[0].size = 0x03C00000;
#endif

	/*
	 * Static partition definition selection
	 */
	parts = systembase_partitions_4;
	nb_parts = ARRAY_SIZE(systembase_partitions_4);
	systembase_map_4.size = window_size;

	/*
	 * Now let's probe for the actual flash.  Do it here since
	 * specific machine settings might have been set above.
	 */
	printk(KERN_NOTICE BOARD_MAP_NAME ": probing %d-bit flash bus\n",
			systembase_map_4.bankwidth*8);
	systembase_map_4.virt = ioremap(window_addr, window_size);
	mymtd = do_map_probe("jedec_probe", &systembase_map_4);
	if (!mymtd) {
		iounmap(systembase_map_4.virt);
		return -ENXIO;
	}
	mymtd->owner = THIS_MODULE;

	add_mtd_partitions(mymtd, parts, nb_parts);
	return 0;
}

static void __exit systembase_mtd_cleanup(void)
{
	if (mymtd) {
		del_mtd_partitions(mymtd);
		map_destroy(mymtd);
		iounmap(systembase_map_4.virt);
	}
}

module_init(systembase_mtd_init);
module_exit(systembase_mtd_cleanup);

MODULE_AUTHOR("Innopiatech, Inc");
MODULE_DESCRIPTION(BOARD_MAP_NAME " MTD driver");
MODULE_LICENSE("GPL");
