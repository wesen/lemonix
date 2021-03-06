/*
 *    Filename: cfag12864b.c
 *     Version: 0.1.0
 * Description: cfag12864b LCD driver
 *     License: GPLv2
 *     Depends: ks0108
 *
 *      Author: Copyright (C) Miguel Ojeda Sandonis <maxextreme@gmail.com>
 *        Date: 2006-10-31
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/mutex.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/workqueue.h>
#include <linux/cfag12864b.h>
#include <linux/pca9539.h>


#define CFAG12864B_NAME "cfag12864b"

/*
 * Module Parameters
 */

static unsigned int cfag12864b_rate = 20;

unsigned int cfag12864b_getrate(void)
{
	return cfag12864b_rate;
}

/*
 * cfag12864b Commands
 *
 *	E = Enable signal
 *		Everytime E switch from low to high,
 *		cfag12864b/ks0108 reads the command/data.
 *
 *	CS1 = First ks0108controller.
 *		If high, the first ks0108 controller receives commands/data.
 *
 *	CS2 = Second ks0108 controller
 *		If high, the second ks0108 controller receives commands/data.
 *
 *	DI = Data/Instruction
 *		If low, cfag12864b will expect commands.
 *		If high, cfag12864b will expect data.
 *
 */

#define bit(n) (((unsigned char)1)<<(n))

#define CFAG12864B_BIT_CS1	(0)
#define CFAG12864B_BIT_CS2	(1)
#define CFAG12864B_BIT_RW	(2)
#define CFAG12864B_BIT_DI	(3)
#define CFAG12864B_BIT_E	(4)

//shlee added
#define DATA_IN_ADDR 0;
#define CONT_IN_ADDR 1;
#define DATA_OUT_ADDR 2;
#define CONT_OUT_ADDR 3;
#define DATA_MOD_ADDR 6;
#define CONT_MOD_ADDR 7;


static unsigned int ks0108_delay = 2;

/*
 * ks0108 Exported Commands (don't lock)
 *
 *   You _should_ lock in the top driver: This functions _should not_
 *   get race conditions in any way. Locking for each byte here would be
 *   so slow and useless.
 *
 *   There are not bit definitions because they are not flags,
 *   just arbitrary combinations defined by the documentation for each
 *   function in the ks0108 LCD controller. If you want to know what means
 *   a specific combination, look at the function's name.
 *
 *   The ks0108_writecontrol bits need to be reverted ^(0,1,3) because
 *   the parallel port also revert them using a "not" logic gate.
 */

#define bit(n) (((unsigned char)1)<<(n))

void ks0108_writedata(unsigned char byte)
{
	u8 in_addr,data,ret,out_addr;
	
	out_addr = DATA_OUT_ADDR;
	in_addr = DATA_IN_ADDR;
	data = (u8) byte;
	pca9539_send (out_addr,data);
	udelay(2);
	
	ret = pca9539_recv (in_addr);
	
	while(ret != data)
	{
		udelay(2);
		pca9539_send (out_addr,data);
		udelay(2);
		ret = pca9539_recv (in_addr);
	}
	pca9539_send (out_addr,data);

	
}

void ks0108_writecontrol(unsigned char byte)
{
	u8 out_addr,in_addr,val,ret;
	
	udelay(ks0108_delay);
	
	in_addr = CONT_IN_ADDR;
	out_addr = CONT_OUT_ADDR;
	val = (u8) byte;
	pca9539_send (out_addr,val);
	udelay(2);

	ret = pca9539_recv (in_addr);
	
	while(ret != val)
	{
//		printk("ce\n");
		udelay(2);
		pca9539_send (out_addr,val);
		udelay(2);
		ret = pca9539_recv (in_addr);
	}
	pca9539_send (out_addr,val);

}

void ks0108_displaystate(unsigned char state)
{
	ks0108_writedata((state ? bit(0) : 0) | bit(1) | bit(2) | bit(3) | bit(4) | bit(5));
}

void ks0108_startline(unsigned char startline)
{
	ks0108_writedata(min(startline,(unsigned char)63) | bit(6) | bit(7));
}

void ks0108_address(unsigned char address)
{
	ks0108_writedata(min(address,(unsigned char)63) | bit(6));
}

void ks0108_page(unsigned char page)
{
	ks0108_writedata(min(page,(unsigned char)7) | bit(3) | bit(4) | bit(5) | bit(7));
}

static unsigned char cfag12864b_state;

static void cfag12864b_set(void)
{
	ks0108_writecontrol(cfag12864b_state);
}

static void cfag12864b_setbit(unsigned char state, unsigned char n)
{
	if (state)
		cfag12864b_state |= bit(n);
	else
		cfag12864b_state &= ~bit(n);
}

static void cfag12864b_e(unsigned char state)
{
	cfag12864b_setbit(state, CFAG12864B_BIT_E);
	cfag12864b_set();
}

static void cfag12864b_cs1(unsigned char state)
{
	cfag12864b_setbit(state, CFAG12864B_BIT_CS1);
}

static void cfag12864b_cs2(unsigned char state)
{
	cfag12864b_setbit(state, CFAG12864B_BIT_CS2);
}

static void cfag12864b_di(unsigned char state)
{
	cfag12864b_setbit(state, CFAG12864B_BIT_DI);
}

static void cfag12864b_rw(unsigned char state)
{
	cfag12864b_setbit(state, CFAG12864B_BIT_RW);
}

static void cfag12864b_setcontrollers(unsigned char first,
		unsigned char second)
{
	if (first)
		cfag12864b_cs1(0);
	else
		cfag12864b_cs1(1);

	if (second)
		cfag12864b_cs2(0);
	else
		cfag12864b_cs2(1);
}

static void cfag12864b_controller(unsigned char which)
{
	if (which == 0)
		cfag12864b_setcontrollers(1, 0);
	else if (which == 1)
		cfag12864b_setcontrollers(0, 1);
}

static void cfag12864b_displaystate(unsigned char state)
{
	cfag12864b_di(0);
	cfag12864b_rw(0);
	cfag12864b_e(1);
	ks0108_displaystate(state);
	cfag12864b_e(0);
}

static void cfag12864b_address(unsigned char address)
{
	cfag12864b_di(0);
	cfag12864b_rw(0);
	cfag12864b_e(1);
	ks0108_address(address);
	cfag12864b_e(0);
}

static void cfag12864b_page(unsigned char page)
{
	cfag12864b_di(0);
	cfag12864b_rw(0);
	cfag12864b_e(1);
	ks0108_page(page);
	cfag12864b_e(0);
}

static void cfag12864b_startline(unsigned char startline)
{
	cfag12864b_di(0);
	cfag12864b_rw(0);
	cfag12864b_e(1);
	ks0108_startline(startline);
	cfag12864b_e(0);
}

static void cfag12864b_writebyte(unsigned char byte)
{
	cfag12864b_di(1);
	cfag12864b_rw(0);
	cfag12864b_e(1);
	ks0108_writedata(byte);
	cfag12864b_e(0);
}

static void cfag12864b_nop(void)
{
	cfag12864b_startline(0);
}

/*
 * cfag12864b Internal Commands
 */

static void cfag12864b_on(void)
{
	cfag12864b_setcontrollers(1, 1);
	cfag12864b_displaystate(1);
}

static void cfag12864b_off(void)
{
	cfag12864b_setcontrollers(1, 1);
	cfag12864b_displaystate(0);
}

static void cfag12864b_clear(void)
{
	unsigned char i, j;

	cfag12864b_setcontrollers(1, 1);
	for (i = 0; i < CFAG12864B_PAGES; i++) {
		cfag12864b_page(i);
		cfag12864b_address(0);
		for (j = 0; j < CFAG12864B_ADDRESSES; j++)
			cfag12864b_writebyte(0);
	}
}

/*
 * Update work
 */

unsigned char *cfag12864b_buffer;
static unsigned char *cfag12864b_cache;
static DEFINE_MUTEX(cfag12864b_mutex);
static unsigned char cfag12864b_updating;
static void cfag12864b_update(struct work_struct *delayed_work);
static struct workqueue_struct *cfag12864b_workqueue;
static DECLARE_DELAYED_WORK(cfag12864b_work, cfag12864b_update);

static void cfag12864b_queue(void)
{
	queue_delayed_work(cfag12864b_workqueue, &cfag12864b_work,
		HZ / cfag12864b_rate);
}

unsigned char cfag12864b_enable(void)
{
	unsigned char ret;

	mutex_lock(&cfag12864b_mutex);

	if (!cfag12864b_updating) {
		cfag12864b_updating = 1;
		cfag12864b_queue();
		ret = 0;
	} else
		ret = 1;

	mutex_unlock(&cfag12864b_mutex);

	return ret;
}

void cfag12864b_disable(void)
{
	mutex_lock(&cfag12864b_mutex);

	if (cfag12864b_updating) {
		cfag12864b_updating = 0;
		cancel_delayed_work(&cfag12864b_work);
		flush_workqueue(cfag12864b_workqueue);
	}

	mutex_unlock(&cfag12864b_mutex);
}

unsigned char cfag12864b_isenabled(void)
{
	return cfag12864b_updating;
}

static void cfag12864b_update(struct work_struct *work)
{
	unsigned char c;
	unsigned short i, j, k, b;

	if (memcmp(cfag12864b_cache, cfag12864b_buffer, CFAG12864B_SIZE)) {
		for (i = 0; i < CFAG12864B_CONTROLLERS; i++) {
			cfag12864b_controller(i);
			cfag12864b_nop();
			for (j = 0; j < CFAG12864B_PAGES; j++) {
				cfag12864b_page(j);
				cfag12864b_nop();
				cfag12864b_address(0);
				cfag12864b_nop();
				for (k = 0; k < CFAG12864B_ADDRESSES; k++) {
					for (c = 0, b = 0; b < 8; b++)
						if (cfag12864b_buffer
							[i * CFAG12864B_ADDRESSES / 8
							+ k / 8 + (j * 8 + b) *
							CFAG12864B_WIDTH / 8]
							& bit(k % 8))
							c |= bit(b);
					cfag12864b_writebyte(c);
				}
			}
		}

		memcpy(cfag12864b_cache, cfag12864b_buffer, CFAG12864B_SIZE);
	}

	if (cfag12864b_updating)
		cfag12864b_queue();
}

/*
 * cfag12864b Exported Symbols
 */

EXPORT_SYMBOL_GPL(cfag12864b_buffer);
EXPORT_SYMBOL_GPL(cfag12864b_getrate);
EXPORT_SYMBOL_GPL(cfag12864b_enable);
EXPORT_SYMBOL_GPL(cfag12864b_disable);
EXPORT_SYMBOL_GPL(cfag12864b_isenabled);

/*
 * Is the module inited?
 */

static unsigned char cfag12864b_inited;
unsigned char cfag12864b_isinited(void)
{
	return cfag12864b_inited;
}
EXPORT_SYMBOL_GPL(cfag12864b_isinited);

/*
 * Module Init & Exit
 */

static int __init cfag12864b_init(void)
{
	int ret = -EINVAL;
	u8 in_addr,val;

	in_addr = DATA_MOD_ADDR;
    val = 0x0;
	pca9539_send (in_addr,val);
	udelay(10);
	pca9539_send (in_addr,val);
	
	in_addr = CONT_MOD_ADDR;
    val = 0x0;
	pca9539_send (in_addr,val);
	udelay(10);
	pca9539_send (in_addr,val);

	if (PAGE_SIZE < CFAG12864B_SIZE) {
		printk(KERN_ERR CFAG12864B_NAME ": ERROR: "
			"page size (%i) < cfag12864b size (%i)\n",
			(unsigned int)PAGE_SIZE, CFAG12864B_SIZE);
		ret = -ENOMEM;
		goto none;
	}

	cfag12864b_buffer = (unsigned char *) __get_free_page(GFP_KERNEL);
	if (cfag12864b_buffer == NULL) {
		printk(KERN_ERR CFAG12864B_NAME ": ERROR: "
			"can't get a free page\n");
		ret = -ENOMEM;
		goto none;
	}

	cfag12864b_cache = kmalloc(sizeof(unsigned char) *
		CFAG12864B_SIZE, GFP_KERNEL);
	if (cfag12864b_buffer == NULL) {
		printk(KERN_ERR CFAG12864B_NAME ": ERROR: "
			"can't alloc cache buffer (%i bytes)\n",
			CFAG12864B_SIZE);
		ret = -ENOMEM;
		goto bufferalloced;
	}

	cfag12864b_workqueue = create_singlethread_workqueue(CFAG12864B_NAME);
	if (cfag12864b_workqueue == NULL)
		goto cachealloced;

	memset(cfag12864b_buffer, 0, CFAG12864B_SIZE);

	cfag12864b_clear();
	cfag12864b_on();

	cfag12864b_inited = 1;
	return 0;

cachealloced:
	kfree(cfag12864b_cache);

bufferalloced:
	free_page((unsigned long) cfag12864b_buffer);

none:
	return ret;
}

static void __exit cfag12864b_exit(void)
{
	cfag12864b_disable();
	cfag12864b_off();
	destroy_workqueue(cfag12864b_workqueue);
	kfree(cfag12864b_cache);
	free_page((unsigned long) cfag12864b_buffer);
}

module_init(cfag12864b_init);
module_exit(cfag12864b_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Miguel Ojeda Sandonis <maxextreme@gmail.com>");
MODULE_DESCRIPTION("cfag12864b LCD driver");
