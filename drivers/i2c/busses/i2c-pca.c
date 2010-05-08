/*
 *  Platform driver for PCA9564 I2C bus controller.
 *
 *  (C) 2006 Andrew Victor
 *
 *  Based on i2c-pca-isa.c driver for PCA9564 on ISA boards
 *    Copyright (C) 2004 Arcom Control Systems
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/platform_device.h>

#include <linux/i2c.h>
#include <linux/i2c-algo-pca.h>

#include <asm/io.h>

#include "../algos/i2c-algo-pca.h"

#define PCA_OWN_ADDRESS		0x55	/* our address for slave mode */
#define PCA_CLOCK		I2C_PCA_CON_59kHz

//#define REG_SHIFT		2
#define REG_SHIFT		0

//#define DEBUG_IO

#define PCA_IO_SIZE 4

static void __iomem *base_addr;
static int irq;
static wait_queue_head_t pca_wait;

static int pca_getown(struct i2c_algo_pca_data *adap)
{
	return PCA_OWN_ADDRESS;
}

static int pca_getclock(struct i2c_algo_pca_data *adap)
{
	return PCA_CLOCK;
}

static void pca_writebyte(struct i2c_algo_pca_data *adap, int reg, int val)
{
#ifdef DEBUG_IO
	static char *names[] = { "T/O", "DAT", "ADR", "CON" };
	printk("*** write %s at %#lx <= %#04x\n", names[reg], (unsigned long) base_addr+reg, val);
#endif
	udelay(1);
	outb(val, base_addr + (reg << REG_SHIFT));
}

static int pca_readbyte(struct i2c_algo_pca_data *adap, int reg)
{
	int res;

	udelay(1);
	res = inb(base_addr + (reg << REG_SHIFT));
#ifdef DEBUG_IO
	{
		static char *names[] = { "STA", "DAT", "ADR", "CON" };
		printk("*** read  %s => %#04x\n", names[reg], res);
	}
#endif
	return res;
}

static int pca_waitforinterrupt(struct i2c_algo_pca_data *adap)
{
	int ret = 0;

	if (irq > -1) {
		ret = wait_event_interruptible(pca_wait,
				pca_readbyte(adap, I2C_PCA_CON) & I2C_PCA_CON_SI);
	} else {
		while ((pca_readbyte(adap, I2C_PCA_CON) & I2C_PCA_CON_SI) == 0)
			udelay(100);
	}
	return ret;
}

static irqreturn_t pca_handler(int this_irq, void *dev_id)
{
	wake_up_interruptible(&pca_wait);
	return IRQ_HANDLED;
}

static struct i2c_algo_pca_data pca_i2c_data = {
	.get_own		= pca_getown,
	.get_clock		= pca_getclock,
	.write_byte		= pca_writebyte,
	.read_byte		= pca_readbyte,
	.wait_for_interrupt	= pca_waitforinterrupt,
};

static struct i2c_adapter pca_i2c_ops = {
	.owner          = THIS_MODULE,
	.id		= I2C_HW_A_PLAT,
	.algo_data	= &pca_i2c_data,
	.name		= "PCA9564",
	.class		= I2C_CLASS_HWMON,
};

static int __devinit pca_i2c_probe(struct platform_device *pdev)
{
	struct resource *res;

	init_waitqueue_head(&pca_wait);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	if (!request_mem_region(res->start, PCA_IO_SIZE, "PCA9564"))
		return -ENXIO;

	base_addr = ioremap(res->start, PCA_IO_SIZE);
	if (base_addr == NULL)
		goto out_region;

	irq = platform_get_irq(pdev, 0);
	if (irq > -1) {
		if (request_irq(irq, pca_handler, 0, "pca9564", NULL) < 0) {
			printk(KERN_ERR "i2c-pca: Request irq%d failed\n", irq);
			goto out_remap;
		}
	}

	/* set up the driverfs linkage to our parent device */
	pca_i2c_ops.dev.parent = &pdev->dev;

	if (i2c_pca_add_bus(&pca_i2c_ops) < 0) {
		printk(KERN_ERR "i2c-pca: Failed to add i2c bus\n");
		goto out_irq;
	}

	return 0;

 out_irq:
	if (irq > -1)
		free_irq(irq, &pca_i2c_ops);

 out_remap:
	iounmap(base_addr);

 out_region:
	release_mem_region(res->start, PCA_IO_SIZE);
	return -ENODEV;
}

static int __devexit pca_i2c_remove(struct platform_device *pdev)
{
	struct resource *res;

	i2c_del_adapter(&pca_i2c_ops);

	if (irq > 0)
		free_irq(irq, NULL);

	iounmap(base_addr);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(res->start, PCA_IO_SIZE);

	return 0;
}

static struct platform_driver pca_i2c_driver = {
	.probe		= pca_i2c_probe,
	.remove		= __devexit_p(pca_i2c_remove),
	.driver		= {
		.name	= "pca9564",
		.owner	= THIS_MODULE,
	},
};

static int __init pca_i2c_init(void)
{
	return platform_driver_register(&pca_i2c_driver);
}

static void __exit pca_i2c_exit(void)
{
	platform_driver_unregister(&pca_i2c_driver);
}

module_init(pca_i2c_init);
module_exit(pca_i2c_exit);

MODULE_AUTHOR("Andrew Victor");
MODULE_DESCRIPTION("PCA9564 platform driver");
MODULE_LICENSE("GPL");
