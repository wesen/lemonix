#include <linux/module.h>
#include <linux/types.h>

#include <linux/errno.h>
#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/major.h>
#include <linux/string.h>
#include <linux/fcntl.h>
#include <linux/ptrace.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/serial.h>

#include <asm/system.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/bitops.h>

#include <asm/ioctls.h>
#include <asm/arch/gpio.h>
#include <asm/arch/at91_pio.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include "../i2c/chips/alpu_bypass.h"

//--> shlee_define
#define PULLUP_ENABLE 1
#define PULLUP_DISABLE 0
#define GPIO_HIGH 1
#define GPIO_LOW 0

#define PIOA_ADDR	0x0
#define PIOB_ADDR	0x200
#define PIOC_ADDR	0x400

#define PIOA	0x1
#define PIOB	0x2
#define PIOC	0x4
#define PIOM	0x7 /// PIOA | PIOB | PIOC

int pio[3] = {PIOA,PIOB,PIOC};

int get_productID(void);
int init_product(void);

int productID = 0x00;

struct at91_gpio_info
{
	unsigned int value[3];
	unsigned int mode[3];
	unsigned int pullup[3];
	unsigned int en_gpio[3];
};

struct at91_gpio_info eddy_gpio;
struct at91_gpio_info tmp_gpio;
	
unsigned char __iomem	*gpio_membase;		/* read/write[bwl] */
	
unsigned long		gpio_mapbase;		/* for ioremap */

int gpio_ioctl(struct inode *inode, struct file *flip, unsigned int command, unsigned long arg);

static int eddy_gpio_open(struct inode *inode, struct file *file_p)
{
	return 0;
}

static int eddy_gpio_close(struct inode *inode, struct file *file_p)
{
	return 0;
}
int get_productID(void)
{
	int gpio_num,val_gpio;
	for(gpio_num = AT91_PIN_PC6,val_gpio = 0 ; gpio_num <= AT91_PIN_PC10 ; gpio_num++)
	{
		at91_set_gpio_input(gpio_num,PULLUP_ENABLE);
		udelay(1);
		val_gpio = at91_get_gpio_value(gpio_num);
		udelay(1);
		productID |= ( val_gpio << ( gpio_num - AT91_PIN_PC6));
	}
	
	return productID;

}
int init_product(void)
{
	return 0;
}

int check_gpio_num(int tmp)
{
	return 0;

}

int get_lowlevel_gpio_info(int pio_info, int register_info)
{
	int i;
	int pio_num = 0;
	for (i = 0 ; i < 3 ; i++)
	{
		pio_num = 0x1 << i;
		if(pio_info & pio_num)
		{

			if(register_info == PIO_OSR)
			{
				tmp_gpio.mode[i] = ioread32(gpio_membase + (pio[i]) + PIO_OSR);
				if(tmp_gpio.mode[i] != eddy_gpio.mode[i])
				{
					ret |= (0x1 << i);
				}
			}
			else if(register_info == PIO_ODSR)
			{
				tmp_gpio.value[i] = ioread32(gpio_membase + (pio[i]) + PIO_ODSR);
				if(tmp_gpio.value[i] != eddy_gpio.value[i])
				{
					ret |= (0x1 << i);
				}
			}
			else if(register_info == PIO_PUSR)
			{
				tmp_gpio.pullup[i] = ioread32(gpio_membase + (pio[i]) + PIO_PUSR);
				if(tmp_gpio.pullup[i] != eddy_gpio.pullup[i])
				{
					ret |= (0x1 << i);
				}
			}
			else
			{
				printk(" error eddy_gpio->ioctl->get_lowlevel_gpio_info wrong register_info 0x%x\n",register_info); 
				return -1;
			}

			return ret;

		}
	}
}
			
int set_lowlevel_gpio_info(int pio_info, int register_info)
{
	int i;
	int pio_num = 0;
	int set_num = 0;
	int clear_num = 0;
	int ret = 0;

	for (i = 0 ; i < 3 ; i++)
	{
		pio_num = 0x1 << i;
		if(pio_info & pio_num)
		{
			gpio_num = tmp.mode[i] & ~(eddy_gpio.en_gpio[i]);

			if(gpio_num)
			{
				printk("error eddy_gpio device initialize first\n");
				return -2;
			}

			gpio_num = tmp.mode[i] & eddy_gpio.en_gpio[i];

			switch(register_info)
			{
				case SETGPIOMOD:
				case SETGPIOVAL		:
				case SETGPIOPUL		:	

				case PIO_IDR:
					iowrite32(gpio_num, gpio_membase + (pio[i]) + PIO_IDR);
					break;

				case PIO_PUDR:
					iowrite32(gpio_num, gpio_membase + (pio[i]) + PIO_PUDR);
					eddy.pullup[i] = eddy.pullup[i] & ( ~gpio_num & eddy_gpio.en_gpio[i]);
					break;

				case PIO_PUER:
					iowrite32(gpio_num, gpio_membase + (pio[i]) + PIO_PUER);
					eddy.pullup[i] = eddy.pullup[i] | ( gpio_num & eddy_gpio.en_gpio[i]);	
					break;

				case PIO_OER:
					iowrite32(gpio_num, gpio_membase + (pio[i]) + PIO_OER);

					gpio_num = ~(gpio_num) & eddy_gpio.en_gpio[i];
					iowrite32(gpio_num, gpio_membase + (pio[i]) + PIO_ODR);
					eddy.mode[i] = gpio_num; 
					break;

				case PIO_PER:
					iowrite32(gpio_num, gpio_membase + (pio[i]) + PIO_PER);
					break;
				
				case PIO_SODR:
					iowrite32(gpio_num, gpio_membase + (pio[i]) + PIO_SODR);
					break;

				default:
					printk(" error eddy_gpio->ioctl->get_lowlevel_gpio_info wrong register_info 0x%x\n",register_info); 
					break;
			}
		}
	}
	return ret;
}

int eddy_gpio_ioctl(struct inode *inode, struct file *flip,
		unsigned int command, unsigned long arg)
{
	int ret=0;
	int tmp = 0;
	int copy_size =0;

	void __user *argp = (void __user *)arg;

	switch (command) {
		case INIT_PRODUCT:
			productID = get_productID();	
			/* all common setting */

			/* ready led set mode out */
			at91_set_gpio_output(AT91_PIN_PC4,GPIO_HIGH);
			/* reset swith read */
			at91_set_gpio_input(AT91_PIN_PC16,PULLUP_ENABLE);
			/* hardware reset */
			/* each product dependent setting */ 
			init_product();
			break;
		case GET_PRODUCTID:
			if(productID == 0x00)
			{
				productID = get_productID();	
			}
			return productID;	
			break;
		case COMRXON:
			/* receive enable */
			at91_set_gpio_value(AT91_PIN_PC16,GPIO_HIGH);
			/* transmit disable */
			at91_set_gpio_value(AT91_PIN_PC17,GPIO_HIGH);
			break;
		case COMTXON:
			/* receive disable */
			at91_set_gpio_value(AT91_PIN_PC16,GPIO_LOW);
			/* transmit enable */
			at91_set_gpio_value(AT91_PIN_PC17,GPIO_LOW);
			break;
		case RDY_LED_ON:
			/* ready LED on */
			//	printk("ready led on\n");
			at91_set_gpio_value(AT91_PIN_PC4,GPIO_HIGH);
			break;
		case RDY_LED_OFF:
			//	printk("ready led off\n");
			/* ready LED off */
			at91_set_gpio_value(AT91_PIN_PC4,GPIO_LOW);
			break;
		case RESET_READ:
			/* reset switch read */	
			//	ret = at91_get_gpio_value(AT91_PIN_PC16);
			ret = 1;

			break;
		case HW_RESET:
			/* Hardware reset */
			at91_set_gpio_value(AT91_PIN_PB19,GPIO_HIGH);
			udelay(100);
			at91_set_gpio_value(AT91_PIN_PB19,GPIO_HIGH);
			udelay(100);
			at91_set_gpio_value(AT91_PIN_PB19,GPIO_HIGH);
			udelay(100);
			break;	
		case SETGPIOMODEIN:
			if(copy_from_user(&tmp, (void *) arg, sizeof(int)))
			{
				return -EFAULT;
			}
			else
			{
				if(productID == EDDY_CPU)
				{
					tmp += AT91_PIN_PC16 ;
				}
				if(productID == EDDY_S1_PIN || productID == EDDY_S1_PIN_C || productID == EDDY_S2_M || productID == EDDY_S2_M_C)
				{
					tmp += AT91_PIN_PC28 ;
				}
				if(productID == EDDY_DK_C)
				{
					tmp += AT91_PIN_PC20 ;
				}

			}
			ret = check_gpio_num(tmp);
			//printk("set gpio mode in ret = %d, tmp = %d \n",ret,tmp);
			if(ret < 0)
			{
				break;
			}
			else
			{
				ret = at91_set_gpio_input(tmp,PULLUP_ENABLE);
				break;
			}
		case SETGPIOMODEOUT:
			if(copy_from_user(&tmp, (void *) arg, sizeof(int)))
			{
				return -EFAULT;
			}
			else
			{
				if(productID == EDDY_CPU)
				{
					//printk("set gpio mode out Eddy_CPU\n");
					tmp += AT91_PIN_PC16 ;
				}
				if(productID == EDDY_S1_PIN || productID == EDDY_S1_PIN_C || productID == EDDY_S2_M || productID == EDDY_S2_M_C)
				{
					//printk("set gpio mode out Eddy_S1_PIN\n");
					tmp += AT91_PIN_PC28 ;
				}
				if(productID == EDDY_DK_C)
				{
					tmp += AT91_PIN_PC20 ;
				}

			}
			ret = check_gpio_num(tmp);
			if(ret < 0)
			{
				break;
			}
			else
			{
				ret = at91_set_gpio_output(tmp,GPIO_HIGH);
				break;
			}
		case GETGPIOMODE:
			if(copy_from_user(&tmp, (void *) arg, sizeof(int)))
			{
				return -EFAULT;
			}
			else
			{
				if(productID == EDDY_CPU)
				{
					tmp += AT91_PIN_PC16 ;
				}
				if(productID == EDDY_S1_PIN || productID == EDDY_S1_PIN_C || productID == EDDY_S2_M || productID == EDDY_S2_M_C)
				{
					tmp += AT91_PIN_PC28 ;
				}
				if(productID == EDDY_DK_C)
				{
					tmp += AT91_PIN_PC20 ;
				}

			}
			ret = check_gpio_num(tmp);
			//printk("get gpio mode ret = %d, tmp = %d \n",ret,tmp);
			if(ret < 0)
			{
				break;
			}
			else
			{
				ret = at91_get_gpio_mode(tmp);
				break;
			}
		case SETGPIOVALUEHIGH:
			if(copy_from_user(&tmp, (void *) arg, sizeof(int)))
			{
				return -EFAULT;
			}
			else
			{
				if(productID == EDDY_CPU)
				{
					tmp += AT91_PIN_PC16 ;
				}
				if(productID == EDDY_S1_PIN || productID == EDDY_S1_PIN_C || productID == EDDY_S2_M || productID == EDDY_S2_M_C)
				{
					tmp += AT91_PIN_PC28 ;
				}
				if(productID == EDDY_DK_C)
				{
					tmp += AT91_PIN_PC20 ;
				}

			}
			ret = check_gpio_num(tmp);
			if(ret < 0)
			{
				break;
			}
			else
			{
				ret = at91_set_gpio_value(tmp,GPIO_HIGH);
				//	printk("set gpio value high = %d \n",tmp);
				break;
			}
		case SETGPIOVALUELOW:
			//printk("\n");
			if(copy_from_user(&tmp, (void *) arg, sizeof(int)))

			{
				return -EFAULT;
			}
			else
			{
				//	printk("set gpio value low 1 = %d \n",tmp);
				if(productID == EDDY_CPU)
				{
					tmp += AT91_PIN_PC16 ;
				}
				if(productID == EDDY_S1_PIN || productID == EDDY_S1_PIN_C || productID == EDDY_S2_M || productID == EDDY_S2_M_C)
				{
					tmp += AT91_PIN_PC28 ;
				}
				if(productID == EDDY_DK_C)
				{
					tmp += AT91_PIN_PC20 ;
				}

			}
			ret = check_gpio_num(tmp);
			//	printk("set gpio value low 2 = %d \n",tmp);
			if(ret < 0)
			{
				break;
			}
			else
			{
				ret = at91_set_gpio_value(tmp,GPIO_LOW);
				//		printk("set gpio value low 3 = %d \n",tmp);
				break;
			}
		case GETGPIOVALUE:
			//printk("\n");
			if(copy_from_user(&tmp, (void *) arg, sizeof(int)))
			{
				return -EFAULT;
			}
			else
			{
				if(productID == EDDY_CPU)
				{
					tmp += AT91_PIN_PC16 ;
				}
				if(productID == EDDY_S1_PIN || productID == EDDY_S1_PIN_C || productID == EDDY_S2_M || productID == EDDY_S2_M_C)
				{
					tmp += AT91_PIN_PC28 ;
				}
				if(productID == EDDY_DK_C)
				{
					tmp += AT91_PIN_PC20 ;
				}

			}
			ret = check_gpio_num(tmp);
			if(ret < 0)
			{
				break;
			}
			else
			{
				ret = at91_get_gpio_value(tmp);
				//		printk("get gpio value = %d \n",tmp);
				break;
			}
		case GPIO_CHIP:
			//	printk("gpio chip check\n");
			ret = system_process();
			break;
	}

	switch (tmp)
	{
		case SETGPIOMOD_LM	:		

			copy_size = sizeof(unsigned int);
			copy_size *= 3;
			if (copy_from_user(&tmp_gpio.mode[0], argp, copy_size))
			{
				return -EFAULT;
			}

			ret = set_lowlevel_gpio_info(PIOM,PIO_IDR);
			ret = set_lowlevel_gpio_info(PIOM,PIO_PUDR);
			ret = set_lowlevel_gpio_info(PIOM,PIO_OER);
			ret = set_lowlevel_gpio_info(PIOM,PIO_PER);

			return ret;


		case GETGPIOMOD_LM	:	

			ret = get_lowlevel_gpio_info(PIOM, PIO_OSR);
			copy_size = sizeof(unsigned int);
			copy_size *= 3;
			if (copy_to_user(argp, &tmp_gpio.mode[0],copy_size))
			{
				return -EFAULT;
			}
			return ret;

		case SETGPIOVAL_LM	:

			copy_size = sizeof(unsigned int);
			copy_size *= 3;
			if (copy_from_user(&tmp_gpio.value[0], argp, copy_size))
			{
				return -EFAULT;
			}

			ret = set_lowlevel_gpio_info(PIOM,PIO_SODR);

			return ret;

		case GETGPIOVAL_LM	:	
			ret = 0;
			ret = get_lowlevel_gpio_info(PIOM, PIO_ODSR);
			copy_size = sizeof(unsigned int);
			copy_size *= 3;
			if (copy_to_user(argp, &tmp_gpio.value[0],copy_size))
			{
				return -EFAULT;
			}
			return 0;

		case SETGPIOPUL_LM	:
			copy_size = sizeof(unsigned int);
			copy_size *= 3;
			if (copy_from_user(&tmp_gpio.pullup[0], argp, copy_size))
			{
				return -EFAULT;
			}

			ret = set_lowlevel_gpio_info(PIOM,PIO_PUER);
			ret = set_lowlevel_gpio_info(PIOM,PIO_PUER);

			return ret;
		case GETGPIOPUL_LM	:
			ret = 0;
			ret = get_lowlevel_gpio_info(PIOM, PIO_PUSR);
			copy_size = sizeof(unsigned int);
			copy_size *= 3;
			if (copy_to_user(argp, &tmp_gpio.pullup[0],copy_size))
			{
				return -EFAULT;
			}
			return ret;

		case SETGPIOMOD_LA	:
			copy_size = sizeof(unsigned int);
			copy_size *= 3;
			if (copy_from_user(&tmp_gpio.mode[0], argp, copy_size))
			{
				return -EFAULT;
			}

			ret = set_lowlevel_gpio_info(PIOM,PIO_IDR);
			ret = set_lowlevel_gpio_info(PIOM,PIO_PUDR);
			ret = set_lowlevel_gpio_info(PIOM,PIO_OER);
			ret = set_lowlevel_gpio_info(PIOM,PIO_PER);

			return ret;

		case GETGPIOMOD_LA	:
			ret = 0;
			ret = get_lowlevel_gpio_info(PIOA, PIO_OSR);
			copy_size = sizeof(unsigned int);
			if (copy_to_user(argp, &tmp_gpio.mode[0],copy_size))
			{
				return -EFAULT;
			}
			return ret;
		case SETGPIOVAL_LA	:
			copy_size = sizeof(unsigned int);
			copy_size *= 3;
			if (copy_from_user(&tmp_gpio.value[0], argp, copy_size))
			{
				return -EFAULT;
			}

			ret = set_lowlevel_gpio_info(PIOM,PIO_SODR);

			return ret;
		case GETGPIOVAL_LA	:
			ret = 0;
			ret = get_lowlevel_gpio_info(PIOA, PIO_ODSR);
			copy_size = sizeof(unsigned int);
			if (copy_to_user(argp, &tmp_gpio.value[0],copy_size))
			{
				return -EFAULT;
			}
			return 0;
		case SETGPIOPUL_LA	:
		case GETGPIOPUL_LA	:
			ret = 0;
			ret = get_lowlevel_gpio_info(PIOA, PIO_PUSR);
			copy_size = sizeof(unsigned int);
			copy_size *= 3;
			if (copy_to_user(argp, &tmp_gpio.pullup[0],copy_size))
			{
				return -EFAULT;
			}
			return ret;
		case SETGPIOMOD_LB	:
			copy_size = sizeof(unsigned int);
			copy_size *= 3;
			if (copy_from_user(&tmp_gpio.mode[0], argp, copy_size))
			{
				return -EFAULT;
			}

			ret = set_lowlevel_gpio_info(PIOM,PIO_IDR);
			ret = set_lowlevel_gpio_info(PIOM,PIO_PUDR);
			ret = set_lowlevel_gpio_info(PIOM,PIO_OER);
			ret = set_lowlevel_gpio_info(PIOM,PIO_PER);

			return ret;

		case GETGPIOMOD_LB	:
			ret = 0;
			ret = get_lowlevel_gpio_info(PIOB, PIO_OSR);
			copy_size = sizeof(unsigned int);
			copy_size *= 3;
			if (copy_to_user(argp, &tmp_gpio.mode[0],copy_size))
			{
				return -EFAULT;
			}
			return ret;
		case SETGPIOVAL_LB	:
			copy_size = sizeof(unsigned int);
			copy_size *= 3;
			if (copy_from_user(&tmp_gpio.value[0], argp, copy_size))
			{
				return -EFAULT;
			}

			ret = set_lowlevel_gpio_info(PIOM,PIO_SODR);

			return ret;
		case GETGPIOVAL_LB	:
			ret = 0;
			ret = get_lowlevel_gpio_info(PIOB, PIO_ODSR);
			copy_size = sizeof(unsigned int);
			copy_size *= 3;
			if (copy_to_user(argp, &tmp_gpio.value[0],copy_size))
			{
				return -EFAULT;
			}
			return 0;
		case SETGPIOPUL_LB	:
		case GETGPIOPUL_LB	:
			ret = 0;
			ret = get_lowlevel_gpio_info(PIOB, PIO_PUSR);
			copy_size = sizeof(unsigned int);
			copy_size *= 3;
			if (copy_to_user(argp, &tmp_gpio.pullup[0],copy_size))
			{
				return -EFAULT;
			}
			return ret;
		case SETGPIOMOD_LC	:
			copy_size = sizeof(unsigned int);
			copy_size *= 3;
			if (copy_from_user(&tmp_gpio.mode[0], argp, copy_size))
			{
				return -EFAULT;
			}

			ret = set_lowlevel_gpio_info(PIOM,PIO_IDR);
			ret = set_lowlevel_gpio_info(PIOM,PIO_PUDR);
			ret = set_lowlevel_gpio_info(PIOM,PIO_OER);
			ret = set_lowlevel_gpio_info(PIOM,PIO_PER);

			return ret;

		case GETGPIOMOD_LC	:
			ret = 0;
			ret = get_lowlevel_gpio_info(PIOC, PIO_OSR);
			copy_size = sizeof(unsigned int);
			copy_size *= 3;
			if (copy_to_user(argp, &tmp_gpio.mode[0],copy_size))
			{
				return -EFAULT;
			}
			return ret;
		case SETGPIOVAL_LC	:
			copy_size = sizeof(unsigned int);
			copy_size *= 3;
			if (copy_from_user(&tmp_gpio.value[0], argp, copy_size))
			{
				return -EFAULT;
			}

			ret = set_lowlevel_gpio_info(PIOM,PIO_SODR);

			return ret;
		case GETGPIOVAL_LC	:
			ret = 0;
			ret = get_lowlevel_gpio_info(PIOC, PIO_ODSR);
			copy_size = sizeof(unsigned int);
			copy_size *= 3;
			if (copy_to_user(argp, &tmp_gpio.value[0],copy_size))
			{
				return -EFAULT;
			}
			return 0;
		case SETGPIOPUL_LC	:
		case GETGPIOPUL_LC	:
			ret = 0;
			ret = get_lowlevel_gpio_info(PIOC, PIO_PUSR);
			copy_size = sizeof(unsigned int);
			copy_size *= 3;
			if (copy_to_user(argp, &tmp_gpio.pullup[0],copy_size))
			{
				return -EFAULT;
			}
			return ret;
		case SETGPIOMOD		:
		case GETGPIOMOD		:
		case SETGPIOVAL		:
		case GETGPIOVAL		:
		case SETGPIOPUL		:	
		case GETGPIOPUL		:
		case SETGPIOMOD_M	:
		case GETGPIOMOD_M	:
		case SETGPIOVAL_M	:
		case GETGPIOVAL_M	:
		case SETGPIOPUL_M	:
		case GETGPIOPUL_M	:
		default : 
			return 0;
	}
	return ret;

}

static const struct file_operations eddy_gpio_fops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.ioctl		= eddy_gpio_ioctl,
	.open		= eddy_gpio_open,
	.release	= eddy_gpio_close,
};

static struct miscdevice eddy_gpio_miscdev = {
	.minor		= AT91GPIO_MINOR,
	.name		= "gpio",
	.fops		= &eddy_gpio_fops,
};

static int __init eddy_gpio_probe(struct platform_device *pdev)
{
	int res;
	int size = pdev->resource[0].end - pdev->resource[0].start + 1;

	gpio_mapbase	= pdev->resource[0].start;

	if (!request_mem_region(gpio_mapbase, size, "at91_gpio"))
		return -EBUSY;

	gpio_membase = ioremap(gpio_mapbase, size);
	if (gpio_membase == NULL) {
		release_mem_region(gpio_mapbase, size);
		return -ENOMEM;
	}

	if (eddy_gpio_miscdev.parent)
		return -EBUSY;
	eddy_gpio_miscdev.parent = &pdev->dev;

	res = misc_register(&eddy_gpio_miscdev);
	if (res)
		return res;

	return 0;
}

static int __exit at91wdt_remove(struct platform_device *pdev)
{
	int res;

	res = misc_deregister(&eddy_gpio_miscdev);
	if (!res)
		eddy_gpio_miscdev.parent = NULL;

	return res;
}
static int eddy_gpio_suspend(struct platform_device *pdev, pm_message_t message)
{
	return 0;
}

static int eddy_gpio_resume(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver eddy_gpio_driver = {
	.probe		= eddy_gpio_probe,
	.remove		= __exit_p(eddy_gpio_remove),
	.suspend	= eddy_gpio_suspend,
	.resume		= eddy_gpio_resume,
	.driver		= {
		.name	= "at91_gpio",
		.owner	= THIS_MODULE,
	},
};

static int __init eddy_gpio_init(void)
{
	int ret = 0;	
	productID = get_productID();	

	ret = platform_driver_register(&eddy_gpio_driver);

	return ret;
}

static void __exit eddy_gpio_exit(void)
{
	platform_driver_unregister(&eddy_gpio_driver);
}

module_init(eddy_gpio_init);

module_exit(eddy_gpio_exit);

MODULE_AUTHOR("John Lee");
MODULE_DESCRIPTION("GPIO driver for Atmel AT91SAM9260 processors");
MODULE_LICENSE("GPL");
MODULE_ALIAS_MISCDEV(AT91GPIO_MINOR);
