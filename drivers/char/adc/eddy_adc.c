/*
 * ADC driver for Atmel AT91SAM9x processors.
 *
 * Copyright (C) 2007 Renaud CERRATO r.cerrato@til-technologies.fr
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

/*
 * The ADC Timer Mode Register can be only written to once. If the
 * timeout need to be set from Linux, be sure that the bootstrap or the
 * bootloader doesn't write to this register.
 */

#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/bitops.h>
#include <linux/uaccess.h>
#include <linux/delay.h>

#include <asm/arch/eddy_adc.h>
#include <asm/ioctls.h>
#include <linux/mtd/gen_probe.h>

#include <asm/arch/gpio.h>


// Set the period between reads (max allowed value 1000)
#define SAMPLE_INTERVAL_MS 5

// Assume client always reads data at 1 second intervals; so the client will read:
#define READ_SAMPLES 1000/SAMPLE_INTERVAL_MS
// Allocate to be at least 5 x bigger so that ISR will never catch up with reading.
#define MAX_ADCSAMPLES 5* READ_SAMPLES



#ifdef ADC_TIMER

static int adc_timeout = -1;		/* default timeout = 3 sec */

static struct timer_list adc_timer;

#define ADC_DELAY (1000 * HZ / 1000) /* every 1 sec */
module_param(adc_timeout, int, 0);
MODULE_PARM_DESC(adc_timeout, "ADC time in seconds. (default = disabled)");

#endif // ADC_TIMER

struct clk *at91adc_clk;

struct eddy_adc_channel {
	spinlock_t		lock;			/* port lock */
	unsigned int		iobase;			/* in/out[bwl] */
	unsigned char __iomem	*membase;		/* read/write[bwl] */
	unsigned long		mapbase;		/* for ioremap */
	int    channel_num;        /* channel number */
};
struct eddy_adc_value {
	int ch0_value;
	int ch1_value;
	int ch2_value;
	int ch3_value;
};

static struct eddy_adc_channel adc_channel;
static struct eddy_adc_value adc_value;

static unsigned long at91adc_busy;

static int init_adc_register(void);
static int get_adc_value(void);

struct clk *clk_get(struct device *dev, const char *id);
int clk_enable(struct clk *clk);

static int get_adc_value()
{

	iowrite32(AT91_ADC_START, (adc_channel.membase + AT91_ADC_CR));

	// Wait for conversion to be complete.
	while ((ioread32(adc_channel.membase + AT91_ADC_SR) & AT91_ADC_DRDY) == 0) 
	{
		cpu_relax();
	}

	if(adc_channel.channel_num & 0x1)
	{
		adc_value.ch0_value = ioread32(adc_channel.membase + AT91_ADC_CHR0);
	}
	if(adc_channel.channel_num & 0x2)
	{
		adc_value.ch1_value = ioread32(adc_channel.membase + AT91_ADC_CHR1);
	}
	if(adc_channel.channel_num & 0x4)
	{
		adc_value.ch2_value = ioread32(adc_channel.membase + AT91_ADC_CHR2);
	}
	if(adc_channel.channel_num & 0x8)
	{
		adc_value.ch3_value = ioread32(adc_channel.membase + AT91_ADC_CHR3);
	}
#ifdef ADC_DEBUG
	printk("adc_channel.channel_num = 0x%x \n",adc_channel.channel_num);
	
	printk("ch0 = %d, ch1= %d,ch2= %d,ch3=%d \n",
			adc_value.ch0_value,
			adc_value.ch1_value,
			adc_value.ch2_value,
			adc_value.ch3_value);
#endif 
	return 0;
}
/* ......................................................................... */
/*
 * ADC device is opened, and ADC starts running.
 */
static int eddy_adc_open(struct inode *inode, struct file *file)
{
	adc_value.ch0_value = 0;
	adc_value.ch1_value = 0;
	adc_value.ch2_value = 0;
	adc_value.ch3_value = 0;

	if (test_and_set_bit(0, &at91adc_busy))
	{
		return -EBUSY;
	}

	return nonseekable_open(inode, file);
}

/*
 * Close the ADC device.
 */
static int eddy_adc_close(struct inode *inode, struct file *file)
{
	clear_bit(0, &at91adc_busy);
	return 0;
}

/*
 * Change the ADC time interval.
 */

#ifdef ADC_TIMER

static void adc_reloading_timer (unsigned long unused)
{  
	mod_timer(&adc_timer, jiffies + ADC_DELAY);
}
#endif //ADC_TIMER

/*
 * Handle commands from user-space.
 */
static int eddy_adc_ioctl(struct inode *inode, struct file *file,
		unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	struct eddy_adc_value tmp_adc_value;
	int tmp = 0;

	switch (cmd) {
		case ADCGETVALUE:
			get_adc_value();

			tmp_adc_value.ch0_value = adc_value.ch0_value ;
			tmp_adc_value.ch1_value = adc_value.ch1_value ;
			tmp_adc_value.ch2_value = adc_value.ch2_value ;
			tmp_adc_value.ch3_value = adc_value.ch3_value ;

			if (copy_to_user(argp, &tmp_adc_value, sizeof(tmp_adc_value)))
			{
				return -EFAULT;
			}
			return 0;

		case ADCGETMODE:
			return (ioread32(adc_channel.membase + AT91_ADC_MR));

		case ADCSETMODE:
			if(copy_from_user(&tmp, (void *) arg, sizeof(int)))
			{
				return -EFAULT;
			}
			else
			{
				tmp &= 0xFFFFFFF;
				iowrite32(tmp, (adc_channel.membase + AT91_ADC_MR));
				return 0;
			}

		case ADCGETCHANNEL:

			return (ioread32(adc_channel.membase + AT91_ADC_CHSR));

		case ADCSETCHANNEL:
			if(copy_from_user(&tmp, (void *) arg, sizeof(int)))
			{
				return -EFAULT;
			}
			else
			{
				adc_value.ch0_value = 0;
				adc_value.ch1_value = 0;
				adc_value.ch2_value = 0;
				adc_value.ch3_value = 0;
				
				tmp &= 0xf;
				adc_channel.channel_num = tmp;
				iowrite32(tmp, (adc_channel.membase + AT91_ADC_CHER));

				tmp = ~tmp & 0xf;
				iowrite32(tmp, (adc_channel.membase + AT91_ADC_CHDR));
				return 0;
			}
	}
	return -ENOTTY;
}

/*
 * Pat the ADC whenever device is written to.
 */
static ssize_t eddy_adc_write(struct file *file, const char *data, size_t len, loff_t *ppos)
{
	return 0;
}

/* ......................................................................... */

static const struct file_operations at91adc_fops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.ioctl		= eddy_adc_ioctl,
	.open		= eddy_adc_open,
	.release	= eddy_adc_close,
	.write		= eddy_adc_write,
};

static struct miscdevice at91adc_miscdev = {
	.minor		= ADC_MINOR,
	.name		= "adc",
	.fops		= &at91adc_fops,
};

static int init_adc_register()
{

	// Initialize ADC. The following two lines set the appropriate PMC bit
	// for the ADC. Easier than mapping PMC registers and then setting the bit.
	at91adc_clk = clk_get(NULL, // Device pointer - not required.
			"adc_clk"); // Clock name.
	clk_enable(at91adc_clk);

	// Reset the ADC
	iowrite32(AT91_ADC_SWRST, (adc_channel.membase + AT91_ADC_CR));

	// Enable both ADC channels
	iowrite32((AT91_ADC_CH(1) | AT91_ADC_CH(0)), (adc_channel.membase + AT91_ADC_CHER));

	// Configure ADC mode register.
	// From table 43-31 in page #775 and page#741 of AT91SAM9260 user manual:
	// Maximum ADC clock frequency = 5MHz = MCK / ((PRESCAL+1) * 2)
	// PRESCAL = ((MCK / 5MHz) / 2) -1 = ((100MHz / 5MHz)/2)-1) = 9
	// Maximum startup time = 15uS = (STARTUP+1)*8/ADC_CLOCK
	// STARTUP = ((15uS*ADC_CLOK)/8)-1 = ((15uS*5MHz)/8)-1 = 9
	// Minimum hold time = 1.2uS = (SHTIM+1)/ADC_CLOCK
	// SHTIM = (1.2uS*ADC_CLOCK)-1 = (1.2uS*5MHz)-1 = 5, Use 9 to ensure 2uS hold time.
	// Enable sleep mode and hardware trigger from TIOA output from TC0.
	iowrite32((AT91_ADC_SHTIM_(9) | AT91_ADC_STARTUP_(9) | AT91_ADC_PRESCAL_(9) |
				AT91_ADC_SLEEP ), (adc_channel.membase + AT91_ADC_MR));
	return 0;
}


static int __init at91adc_probe(struct platform_device *pdev)
{
	int res;
	int size = pdev->resource[0].end - pdev->resource[0].start + 1;

	adc_channel.mapbase	= pdev->resource[0].start;

	if (!request_mem_region(adc_channel.mapbase, size, "eddy_adc"))
		return -EBUSY;

	adc_channel.membase = ioremap(adc_channel.mapbase, size);
	if (adc_channel.membase == NULL) {
		release_mem_region(adc_channel.mapbase, size);
		return -ENOMEM;
	}

	if (at91adc_miscdev.parent)
		return -EBUSY;
	at91adc_miscdev.parent = &pdev->dev;

	res = misc_register(&at91adc_miscdev);
	if (res)
		return res;

	init_adc_register();

	return 0;
}

static int __exit at91adc_remove(struct platform_device *pdev)
{
	int res;

	res = misc_deregister(&at91adc_miscdev);
	if (!res)
		at91adc_miscdev.parent = NULL;

	return res;
}


#ifdef CONFIG_PM

static int at91adc_suspend(struct platform_device *pdev, pm_message_t message)
{
	return 0;
}

static int at91adc_resume(struct platform_device *pdev)
{
	return 0;
}

#else
#define at91adc_suspend	NULL
#define at91adc_resume	NULL
#endif

static struct platform_driver at91adc_driver = {
	.probe		= at91adc_probe,
	.remove		= __exit_p(at91adc_remove),
	.suspend	= at91adc_suspend,
	.resume		= at91adc_resume,
	.driver		= {
		.name	= "eddy_adc",
		.owner	= THIS_MODULE,
	},
};

static struct resource adc_resources[] = {
	[0] = {
		.start	= AT91SAM9260_BASE_ADC,
		.end	= AT91SAM9260_BASE_ADC + SZ_16K - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= AT91SAM9260_ID_ADC,
		.end	= AT91SAM9260_ID_ADC,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device at91sam9260_adc_device = {
	.name		= "eddy_adc",
	.id		= -1,
	.resource	= adc_resources,
	.num_resources	= ARRAY_SIZE(adc_resources),
};


static int __init at91sam_adc_init(void)
{
	int ret = 0;
	at91_set_A_periph(AT91_PIN_PC0, 0);		/* adc 0 */
	at91_set_A_periph(AT91_PIN_PC1, 0);		/* adc 1 */
	at91_set_A_periph(AT91_PIN_PC2, 0);		/* adc 2 */
	at91_set_A_periph(AT91_PIN_PC3, 0);		/* adc 3 */
	
	platform_device_register(&at91sam9260_adc_device);

#ifdef ADC_TIMER
	init_timer(&adc_timer);
	adc_timer.function = adc_reloading_timer;
	mod_timer(&adc_timer, jiffies + ADC_DELAY);
#endif //ADC_TIMER
	ret = platform_driver_register(&at91adc_driver);

	return ret;
}

static void __exit at91sam_adc_exit(void)
{
#ifdef ADC_TIMER
	del_timer(&adc_timer);
#endif //ADC_TIMER
	platform_driver_unregister(&at91adc_driver);
}

module_init(at91sam_adc_init);
module_exit(at91sam_adc_exit);

MODULE_AUTHOR("John Lee");
MODULE_DESCRIPTION("ADC driver for Eddy V2.1 Development Kit");
MODULE_LICENSE("GPL");
MODULE_ALIAS_MISCDEV(ADC_MINOR);
