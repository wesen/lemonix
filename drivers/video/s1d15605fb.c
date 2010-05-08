/*
 *  drivers/video/s1d15605.c
 *
 * Adapted from several sources including:
 * 1) Driver for AT91 LCD Controller
 *    Copyright (C) 2006 Atmel
 *
 * 2) Copyright (C) 2005 S. Kevin Hester
 *
 *   This file is subject to the terms and conditions of the GNU General Public
 *   License. See the file COPYING in the main directory of this archive for
 *   more details.
 *
 *   This is a basic framebuffer driver for the Optrex F-51320 128x64 mono LCD
 *   display.  This display uses a clone of the common Epson SED 1531 display
 *   controller.
 *
 *   I've heavily borrowed code from the vfb.c driver.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifdef DEBUG
#define MSG(string, args...) printk("s1d15605fb:" string, ##args)
#else
#define MSG(string, args...)
#endif

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/delay.h>

#include <asm/uaccess.h>

#include <asm/arch/board.h>
#include <asm/arch/gpio.h>

#ifdef CONFIG_PMAC_BACKLIGHT
#include <asm/backlight.h>
#endif

#define VIDEOWIDTH		128
#define VIDEOHEIGHT		64
#define VIDEODEPTH		1	/* bits/pixel */
#define VIDEOWIDTH_BYTES	((VIDEOWIDTH * VIDEODEPTH) / 8)

/* The number of bytes that actually go to the device */
#define ACTUALVIDEOMEMSIZE	(VIDEOWIDTH_BYTES * VIDEOHEIGHT)
#define VIDEOMEMSIZE		PAGE_SIZE

static struct fb_var_screeninfo s1d15605_default __initdata = {
	.xres		= VIDEOWIDTH,
	.yres		= VIDEOHEIGHT,
	.xres_virtual	= VIDEOWIDTH,
	.yres_virtual	= VIDEOHEIGHT,
	.bits_per_pixel	= VIDEODEPTH,
	.red		= { 0, 1, 0 },
	.green		= { 0, 1, 0 },
	.blue		= { 0, 1, 0 },
	.activate	= FB_ACTIVATE_NOW,
	.pixclock	= 20000,
	.vmode		= FB_VMODE_NONINTERLACED,
};

static struct fb_fix_screeninfo s1d15605_fix __initdata = {
	.id		= "s1d15605",
	.type		= FB_TYPE_PACKED_PIXELS,
	.visual		= FB_VISUAL_MONO10,
	.xpanstep	= 0,
	.ypanstep	= 0,
	.ywrapstep	= 0,
	.accel		= FB_ACCEL_NONE,
};

struct s1d15605fb_info {
	struct fb_info		*info;
	char			*mmio;
	unsigned long		reset_pin;
	struct platform_device	*pdev;
};

/*
 * LCD device interface
 */
#define	RESET_DISPLAY		0xE2
#define	LCD_BIAS_1_9		0xA2
#define	ADC_SELECT_REVERSE	0xA1
#define	COMMON_OUTPUT_NORMAL	0xC0
#define	V5_RESISTOR_RATIO	0x26
#define	ELECTRONIC_VOLUME_SET	0x81
#define	ELECTRONIC_VOLUME_INIT	0x20
#define	POWER_CONTROL_SET	0x28
#define	VOLTAGE_REGULATOR	0x02
#define	VOLTAGE_FOLLOWER	0x01
#define	BOOSTER_CIRCUIT		0x04
#define	DISPLAY_ON		0xAF
#define	START_LINE_SET		0x40
#define	PAGE_ADDRESS_SET	0xB0
#define	COLUMN_ADDRESS_HIGH	0x10
#define	COLUMN_ADDRESS_LOW	0x00
#define	RESISTOR_RATIO_START	0x20

#define	NUM_OF_PAGES		8
#define	NUM_OF_COLUMNS		128

#define	WRITE_COMMAND(x)	__raw_writeb((x), (sinfo)->mmio)
#define	READ_COMMAND		__raw_readb((sinfo)->mmio)
#define	WRITE_DATA(x)		__raw_writeb((x), (sinfo)->mmio + (0x10000))
#define	READ_DATA		__raw_readb((sinfo)->mmio + (0x10000))


/*
 *	s1d15605fb_resize_framebuffer
 *
 *	Free allocated space if different.  Allocate on new of changed.
 *	Returns -ENOMEM if the new framebuffer can not be allocated,
 *	zero on success.
 */
static int s1d15605fb_resize_framebuffer(struct s1d15605fb_info *sinfo)
{
	struct fb_info			*info = sinfo->info;
	struct fb_fix_screeninfo	*fix = &info->fix;
	struct fb_var_screeninfo	*var = &info->var;
	unsigned int			new_size;
	void				*new_vaddr;

	new_size = ((var->xres_virtual * var->yres_virtual * var->bits_per_pixel) / 8);

	MSG("%s: x (%d) y (%d) bpp (%d): new size 0x%08x\n", __FUNCTION__,
		var->xres_virtual, var->yres_virtual, var->bits_per_pixel, new_size);

	if (new_size == fix->smem_len)
		return 0;

	if (fix->smem_len) {
		kfree(info->screen_base);
	}

	new_vaddr = kmalloc(new_size, GFP_KERNEL);

	if (!new_vaddr) {
		fix->smem_len = 0;
		return -ENOMEM;
	}

	info->screen_base = new_vaddr;
	fix->smem_start = (unsigned)new_vaddr;
	fix->smem_len = new_size;
	fix->line_length = (var->xres_virtual * var->bits_per_pixel) / 8;

	dev_info(info->device,
		"%luKiB frame buffer at %08lx (mapped at %p)\n",
		(unsigned long)info->fix.smem_len / 1024,
		(unsigned long)info->fix.smem_start,
		info->screen_base);

	return 0;
}


/*
 * The s1d15605 seems to be divided into eight 128 pixel wide pages (from top to
 * bottom) each page seems to be eight pixels high, where these eight pixels are
 * one byte
 */
static void s1d15605_update(struct fb_info *info)
{
	struct s1d15605fb_info	*sinfo = info->par;
	int			page, i, row, colmask;
	u8			retVal, *rowPtr;

	WRITE_COMMAND(START_LINE_SET);
	for (page = 0; page < NUM_OF_PAGES; ++page) {
		WRITE_COMMAND(PAGE_ADDRESS_SET + page);
		WRITE_COMMAND(COLUMN_ADDRESS_HIGH);
		WRITE_COMMAND(COLUMN_ADDRESS_LOW);

		for (i = 0; i < NUM_OF_COLUMNS; ++i)
		{
			/* point of opportunity: optimization */
			colmask = (1 << (i & 0x7));
			rowPtr = (u8*)(info->screen_base);
			rowPtr += (VIDEOWIDTH_BYTES * 8 * page);
			rowPtr += (i >> 3);
			retVal = 0;
			for (row = 0; row < 8; ++row)
			{
				retVal = (retVal >> 1) | (((*rowPtr) & colmask) ? 0x80 : 0);
				rowPtr += VIDEOWIDTH_BYTES;
			}
			WRITE_DATA(retVal);
		}
	}

	WRITE_COMMAND(DISPLAY_ON);
}


/*
 * Setting the video mode has been split into two parts.
 * First part, xxxfb_check_var, must not write anything
 * to hardware, it should only verify and adjust var.
 * This means it doesn't alter par but it does use hardware
 * data from it to check this var.
 */
static int s1d15605_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	/*
	 * Some very basic checks
	 */
	if (!var->xres)
		var->xres = 1;
	if (!var->yres)
		var->yres = 1;
	if (var->xres > var->xres_virtual)
		var->xres_virtual = var->xres;
	if (var->yres > var->yres_virtual)
		var->yres_virtual = var->yres;

	if(var->bits_per_pixel > VIDEODEPTH)
		return -EINVAL;

	/*
	 * Memory limit
	 */
	if (((var->yres_virtual * var->bits_per_pixel * var->yres_virtual) >> 3) >
			ACTUALVIDEOMEMSIZE)
		return -ENOMEM;

	/*
	 * Now that we checked it we alter var. The reason being is that the video
	 * mode passed in might not work but slight changes to it might make it
	 * work. This way we let the user know what is acceptable.
	 */
	switch (var->bits_per_pixel) {
	case 1:
		var->red.offset = var->green.offset = var->blue.offset = 0;
		var->red.length = var->green.length = var->blue.length
			= var->bits_per_pixel;
		break;
	default:
		return -EINVAL;
	}

	var->xoffset = var->yoffset = 0;
	var->red.msb_right = var->green.msb_right = var->blue.msb_right =
		var->transp.msb_right = 0;

	return 0;
}


/*
 * This routine actually sets the video mode. It's in here where we
 * the hardware state info->par and fix which can be affected by the
 * change in par. For this driver it doesn't do much.
 */
static int s1d15605_set_par(struct fb_info *info)
{
	int	ret;

	MSG("%s:\n", __func__);
	MSG("  * resolution: %ux%u (%ux%u virtual)\n",
		 info->var.xres, info->var.yres,
		 info->var.xres_virtual, info->var.yres_virtual);

	ret = s1d15605fb_resize_framebuffer(info->par);

	info->fix.visual = FB_VISUAL_MONO10;
	return ret;
}


/*
 * Set a single color register. The values supplied are already
 * rounded down to the hardware's capabilities (according to the
 * entries in the var structure). Return != 0 for invalid regno.
 */
static int s1d15605_setcolreg(u_int regno, u_int red, u_int green, u_int blue,
			u_int transp, struct fb_info *info)
{
	if (regno > 1)	/* no. of hw registers - we only do mono now */
		return 1;

	return 0;
}


/*
 * Currently, the routine will simply shut-off the backlight and prevent
 * updates/refreshes.  Modify according to application.
 *
 * 0 unblank, 1 blank, 2 no vsync, 3 no hsync, 4 off
 */
static int s1d15605_blank(int blank, struct fb_info *info)
{
#ifdef CONFIG_PMAC_BACKLIGHT
	if (blank)
		pmac_backlight->props.power = FB_BLANK_POWERDOWN;
	else
		pmac_backlight->props.power = FB_BLANK_UNBLANK;
	backlight_update_status(pmac_backlight);
#endif
	return 1;
}


/*
 * Pan or Wrap the Display
 *
 * This call looks only at xoffset, yoffset and the FB_VMODE_YWRAP flag
 */
/*
static int s1d15605_pan_display(struct fb_var_screeninfo *var,
			struct fb_info *info)
{
	if (var->vmode & FB_VMODE_YWRAP) {
		if (var->yoffset < 0
		    || var->yoffset >= info->var.yres_virtual
		    || var->xoffset)
			return -EINVAL;
	} else {
		if (var->xoffset + var->xres > info->var.xres_virtual ||
		    var->yoffset + var->yres > info->var.yres_virtual)
			return -EINVAL;
	}
	info->var.xoffset = var->xoffset;
	info->var.yoffset = var->yoffset;
	if (var->vmode & FB_VMODE_YWRAP)
		info->var.vmode |= FB_VMODE_YWRAP;
	else
		info->var.vmode &= ~FB_VMODE_YWRAP;
	return 0;
}
*/


static void s1d15605_copyarea(struct fb_info *info, const struct fb_copyarea *region)
{
	cfb_copyarea(info, region);
	s1d15605_update(info);
}


static void s1d15605_fillrect (struct fb_info *info, const struct fb_fillrect *rect)
{
	cfb_fillrect(info, rect);
	s1d15605_update(info);
}


static void s1d15605_imageblit(struct fb_info *p, const struct fb_image *image)
{
	cfb_imageblit(p, image);
	s1d15605_update(p);
}


/*
 * Write the users data to our framebuffer, and then trigger a psuedo DMA
 */
static ssize_t s1d15605_write(struct file *file, const char *buf,
			size_t count, loff_t *ppos)
{
	unsigned long p = *ppos;
	struct inode *inode = file->f_dentry->d_inode;
	int fbidx = iminor(inode);
	struct fb_info *info = registered_fb[fbidx];
	int err;

	if (p > info->fix.smem_len)
		return -ENOSPC;
	if (count >= info->fix.smem_len)
		count = info->fix.smem_len;
	err = 0;
	if (count + p > info->fix.smem_len) {
		count = info->fix.smem_len - p;
		err = -ENOSPC;
	}
	if (count) {
		char *base_addr;

		base_addr = info->screen_base;
		count -= copy_from_user(base_addr+p, buf, count);
		*ppos += count;
		err = -EFAULT;
	}

	s1d15605_update(info);

	if (count)
		return count;

	return err;
}

#ifdef	USE_PRIVATE_VMA_FXS
static void s1d15605_vma_open(struct vm_area_struct *vma)
{
	// FIXME - store stats in the device data via vm_private_data
}


static void s1d15605_vma_close(struct vm_area_struct *vma)
{
	// FIXME - store stats in the device data via vm_private_data
}


static struct page *s1d15605_vma_nopage(struct vm_area_struct *vma,
				unsigned long address, int *type)
{
	struct page *page;
	struct fb_info *info = vma->vm_private_data;

	page = virt_to_page(info->screen_base);
	get_page(page);

	// FIXME - now someone has a link to our page, start periodically blitting
	// latest updates to the actual device.

	return page;
}


static struct vm_operations_struct s1d15605_vm_ops = {
	.open	= s1d15605_vma_open,
	.close	= s1d15605_vma_close,
	.nopage	= s1d15605_vma_nopage
};


/* We don't do much here - because we have special vm_ops */
static int s1d15605_mmap(struct fb_info *info, struct vm_area_struct *vma)
{
	vma->vm_ops = &s1d15605_vm_ops;
	vma->vm_flags |= VM_RESERVED;
	vma->vm_private_data = info;
	s1d15605_vma_open(vma);

	return 0;
}
#endif /* USE_PRIVATE_VMA_FXS */


static struct fb_ops s1d15605fb_ops = {
	.owner		= THIS_MODULE,
	.fb_check_var	= s1d15605_check_var,
	.fb_set_par	= s1d15605_set_par,
	.fb_setcolreg	= s1d15605_setcolreg,
	.fb_blank	= s1d15605_blank,
//	.fb_pan_display	= s1d15605_pan_display,
	.fb_fillrect	= s1d15605_fillrect,
	.fb_copyarea	= s1d15605_copyarea,
	.fb_imageblit	= s1d15605_imageblit,
	.fb_write	= s1d15605_write,
#ifdef	USE_PRIVATE_VMA_FXS
	.fb_mmap	= s1d15605_mmap,
#endif
};


static void s1d15605_device_init(struct s1d15605fb_info *sinfo) {

	char	value;

	/* release the reset line by reading the device - proto hardware */
	value = READ_COMMAND;
	value = READ_COMMAND;

#ifdef CONFIG_MACH_KB9200
	/* new boards have dedicated reset line */
	gpio_set_value(sinfo->reset_pin, 1);
#endif

	/* initialize the device within 5ms */
	WRITE_COMMAND(RESET_DISPLAY);
	WRITE_COMMAND(LCD_BIAS_1_9);
	WRITE_COMMAND(ADC_SELECT_REVERSE);
	WRITE_COMMAND(COMMON_OUTPUT_NORMAL);
	WRITE_COMMAND(V5_RESISTOR_RATIO);
	WRITE_COMMAND(ELECTRONIC_VOLUME_SET);
	WRITE_COMMAND(ELECTRONIC_VOLUME_INIT);
	WRITE_COMMAND(POWER_CONTROL_SET | VOLTAGE_REGULATOR | VOLTAGE_FOLLOWER | BOOSTER_CIRCUIT);
	WRITE_COMMAND(DISPLAY_ON);

	WRITE_COMMAND(RESISTOR_RATIO_START + 4);
	WRITE_COMMAND(ELECTRONIC_VOLUME_SET);
	WRITE_COMMAND(0x33);
}


static int s1d15605fb_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct fb_info *info;
	struct s1d15605fb_info *sinfo;
	int ret;

	MSG("%s\n", __func__);

	if (!(info = framebuffer_alloc(sizeof(struct s1d15605fb_info), dev))) {
		dev_err(dev, "Cannot allocate framebuffer struct\n");
		return -ENOMEM;
	}

	sinfo = info->par;
	sinfo->info = info;
	sinfo->pdev = pdev;

	if (pdev->num_resources < 2) {
		dev_err(dev, "Resources unusable\n");
		ret = -ENODEV;
		goto free_info;
	}

	info->fbops = &s1d15605fb_ops;
	strcpy(info->fix.id, pdev->name);

	info->fix.mmio_start = pdev->resource[0].start;
	info->fix.mmio_len = pdev->resource[0].end - pdev->resource[0].start + 1;
	sinfo->reset_pin = pdev->resource[1].start;

	ret = s1d15605fb_resize_framebuffer(sinfo);
	if (ret < 0) {
		dev_err(dev, "Cannot resize framebuffer: %d\n", ret);
		goto free_fb;
	}

	if (!request_mem_region(info->fix.mmio_start,
				info->fix.mmio_len, pdev->name)) {
		ret = -EBUSY;
		goto free_fb;
	}

	sinfo->mmio = ioremap(info->fix.mmio_start, info->fix.mmio_len);
	if (!sinfo->mmio) {
		dev_err(dev, "Cannot map LCD memory region\n");
		goto release_mem;
	}

	s1d15605_device_init(sinfo);

	ret = fb_find_mode(&info->var, info, NULL, NULL, 0, NULL, 1);

	if (!ret || (ret == 4))
		info->var = s1d15605_default;

	info->fix = s1d15605_fix;
	info->flags = FBINFO_FLAG_DEFAULT |
/*		FBINFO_HWACCEL_YPAN | */
		FBINFO_HWACCEL_FILLRECT | FBINFO_HWACCEL_COPYAREA;

	ret = register_framebuffer(info);
	if (ret < 0) {
		dev_err(dev, "Failed to register framebuffer device: %d\n", ret);
		goto unmap_mmio;
	}

	dev_set_drvdata(dev, info);

	memset(info->screen_base, 0, info->fix.smem_len);
	info->var.activate |= FB_ACTIVATE_NOW;
	ret = fb_set_var(info, &info->var);
	if (ret) {
		dev_warn(dev, "Unable to set display parameters\n");
	}

	info->var.activate &= ~(FB_ACTIVATE_FORCE | FB_ACTIVATE_NOW);

	dev_dbg(dev, "%s SUCCESS\n", __func__);

	dev_info(dev, "Driver $Revision: 1.1 $\n");

	return 0;

unmap_mmio:
	iounmap(sinfo->mmio);
release_mem:
	release_mem_region(info->fix.mmio_start, info->fix.mmio_len);
free_fb:
	kfree(info->screen_base);

free_info:
	framebuffer_release(info);

	dev_dbg(dev, "%s FAILED\n", __func__);
	return ret;
}


static int s1d15605fb_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct fb_info *info = dev_get_drvdata(dev);
	struct s1d15605fb_info *sinfo = info->par;

	if (!sinfo)
		return 0;

	unregister_framebuffer(info);

	iounmap(sinfo->mmio);
	release_mem_region(info->fix.mmio_start, info->fix.mmio_len);

	kfree(info->screen_base);

	dev_set_drvdata(dev, NULL);
	framebuffer_release(info);
	return 0;
}


static struct platform_driver s1d15605fb_driver = {
	.probe		= s1d15605fb_probe,
	.remove		= s1d15605fb_remove,
	.driver		= {
		.name	= "s1d15605fb",
		.owner	= THIS_MODULE,
	},
};


static int __init s1d15605fb_init(void)
{
	return platform_driver_register(&s1d15605fb_driver);
}


static void __exit s1d15605fb_exit(void)
{
	platform_driver_unregister(&s1d15605fb_driver);
}


module_init(s1d15605fb_init);
module_exit(s1d15605fb_exit);


MODULE_AUTHOR("KwikByte");
MODULE_DESCRIPTION("Epson S1D15605 LCD Controller framebuffer driver");
MODULE_LICENSE("GPL");
