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

//--> shlee_define
#define PULLUP_ENABLE 1
#define PULLUP_DISABLE 0
#define GPIO_HIGH 1
#define GPIO_LOW 0

#define MAX_GPIO_NUM 55

#define PIOA_ADDR	0x0
#define PIOB_ADDR	0x200
#define PIOC_ADDR	0x400

#define PIOA	0x1
#define PIOB	0x2
#define PIOC	0x4
#define PIOM	0x7 /// PIOA | PIOB | PIOC

#define CHANGE_VALUE 0x1
#define CHANGE_MODE 0x2
#define CHANGE_PULLUP 0x3

#define SETGPIOMOD			0x00
#define GETGPIOMOD			0x01
#define SETGPIOVAL			0x02
#define GETGPIOVAL			0x03
#define SETGPIOPUL			0x04
#define GETGPIOPUL			0x05
#define SETGPIOENABLE		0x06	

int pio[3] = {PIOA_ADDR,PIOB_ADDR,PIOC_ADDR};
short gpio_tmp[MAX_GPIO_NUM] = {0,}; 
short default_gpio[MAX_GPIO_NUM] = {0,};
int get_productID(void);

int productID = 0x1f;

struct at91_gpio_low_info
{
	unsigned int value[3];
	unsigned int mode[3];
	unsigned int pullup[3];
	unsigned int en_gpio[3];
};

struct at91_gpio_info
{
	unsigned int value[2];
	unsigned int mode[2];
	unsigned int pullup[2];
	unsigned int en_gpio[2];
	int gpio_num;
};

struct at91_gpio_low_info eddy_gpio_l;
struct at91_gpio_low_info tmp_gpio_l;
	
struct at91_gpio_info eddy_gpio;
struct at91_gpio_info tmp_gpio;

void __iomem	*gpio_membase;		/* read/write[bwl] */
	
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
	return productID;
}
int get_lowlevel_gpio_info(int pio_info, int register_info)
{
	int i,ret=0;
	int pio_num = 0;
	unsigned int tmp_value = 0;
	for (i = 0 ; i < 3 ; i++)
	{
		pio_num = 0x1 << i;
		if(pio_info & pio_num)
		{

			if(register_info == PIO_OSR)
			{
				tmp_value = __raw_readl(gpio_membase + (pio[i]) + PIO_OSR);
				tmp_gpio_l.mode[i] = tmp_value & eddy_gpio_l.en_gpio[i];
				if(tmp_gpio_l.mode[i] != eddy_gpio_l.mode[i])
				{
					ret |= (0x1 << i);
				}
			}
			else if(register_info == PIO_PDSR)
			{
				tmp_value = __raw_readl(gpio_membase + (pio[i]) + PIO_PDSR);
				tmp_gpio_l.value[i] = tmp_value & eddy_gpio_l.en_gpio[i];
				if(tmp_gpio_l.value[i] != eddy_gpio_l.value[i])
				{
					ret |= (0x1 << i);
				}
			}
			else if(register_info == PIO_PUSR)
			{
				tmp_value = __raw_readl(gpio_membase + (pio[i]) + PIO_PUSR);
				tmp_gpio_l.pullup[i] = tmp_value & eddy_gpio_l.en_gpio[i];
				if(tmp_gpio_l.pullup[i] != eddy_gpio_l.pullup[i])
				{
					ret |= (0x1 << i);
				}
			}
			else
			{
				printk(" error eddy_gpio_l->ioctl->get_lowlevel_gpio_info wrong register_info 0x%x\n",register_info); 
				return -1;
			}


		}
	}
	return ret;
	
}

int set_lowlevel_gpio_info(int pio_info, int register_info)
{
	int i,gpio_num = 0;
	int pio_num = 0;
	int ret = 0;

	for (i = 0 ; i < 3 ; i++)
	{
		pio_num = 0x1 << i;
		if(pio_info & pio_num)
		{
			gpio_num = tmp_gpio_l.mode[i] & ~(eddy_gpio_l.en_gpio[i]);

			if(gpio_num)
			{
				printk("error 0x%x gpio mode setting is not enabled \n",gpio_num);
			}

			switch(register_info)
			{
				case SETGPIOENABLE:

					gpio_num = tmp_gpio_l.en_gpio[i] & eddy_gpio_l.en_gpio[i];

					eddy_gpio_l.en_gpio[i] = gpio_num; 
					// disable gpio  
					__raw_writel(eddy_gpio_l.en_gpio[i], gpio_membase + (pio[i]) + PIO_PDR);

					// interrupt disable
					__raw_writel(eddy_gpio_l.en_gpio[i], gpio_membase + (pio[i]) + PIO_IDR);

					// enable gpio  
					__raw_writel(eddy_gpio_l.en_gpio[i], gpio_membase + (pio[i]) + PIO_PER);
					break;

				case SETGPIOMOD:
					gpio_num = tmp_gpio_l.mode[i] & eddy_gpio_l.en_gpio[i];

					eddy_gpio_l.mode[i] = gpio_num; 
					// mode output enable 
					__raw_writel(gpio_num, gpio_membase + (pio[i]) + PIO_OER);

					// mode output disable
					gpio_num = ~(gpio_num) & eddy_gpio_l.en_gpio[i];
					__raw_writel(gpio_num, gpio_membase + (pio[i]) + PIO_ODR);

					break;

				case SETGPIOPUL:	
					gpio_num = tmp_gpio_l.pullup[i] & eddy_gpio_l.en_gpio[i];

					eddy_gpio_l.pullup[i] = gpio_num;

					__raw_writel(gpio_num, gpio_membase + (pio[i]) + PIO_PUER);

					gpio_num = ~(gpio_num) & eddy_gpio_l.en_gpio[i];
					__raw_writel(gpio_num, gpio_membase + (pio[i]) + PIO_PUDR);

					break;

				case SETGPIOVAL:
					gpio_num = tmp_gpio_l.value[i] & eddy_gpio_l.en_gpio[i];

					eddy_gpio_l.value[i] = gpio_num; 
					__raw_writel(gpio_num, gpio_membase + (pio[i]) + PIO_SODR);

					gpio_num = ~(gpio_num) & eddy_gpio_l.en_gpio[i];
					__raw_writel(gpio_num, gpio_membase + (pio[i]) + PIO_CODR);

					break;

				default:
					printk(" error eddy_gpio_l->ioctl->set_lowlevel_gpio_info wrong register_info 0x%x\n",register_info); 
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
	int copy_size =0;

	void __user *argp = (void __user *)arg;

	switch (command) {

		case INIT_PRODUCT:
			/* ready led set mode out */
			at91_set_gpio_output(AT91_PIN_PC4,GPIO_HIGH);
			/* reset swith read */
			at91_set_gpio_input(AT91_PIN_PC16,PULLUP_ENABLE);
			break;

		case GET_PRODUCTID:
			productID = get_productID();	
			return productID;	
			break;
		case RDY_LED_ON:
			/* ready LED on */
			at91_set_gpio_value(AT91_PIN_PC4,GPIO_HIGH);
			break;
		case RDY_LED_OFF:
			/* ready LED off */
			at91_set_gpio_value(AT91_PIN_PC4,GPIO_LOW);
			break;
		case RESET_READ:
			/* reset switch read */	
			ret = at91_get_gpio_value(AT91_PIN_PC16);
			break;

		case SETGPIOINIT:		
			copy_size = sizeof(tmp_gpio_l);
			if (copy_from_user(&tmp_gpio_l, argp, copy_size))
			{
				return -EFAULT;
			}
			memcpy(&eddy_gpio_l,&tmp_gpio_l,copy_size);
			
			set_lowlevel_gpio_info(PIOM,SETGPIOENABLE);
			set_lowlevel_gpio_info(PIOM,SETGPIOMOD);
			set_lowlevel_gpio_info(PIOM,SETGPIOPUL);
			set_lowlevel_gpio_info(PIOM,SETGPIOVAL);

			return ret;

		case SETGPIOMOD_LM	:		

			copy_size = sizeof(unsigned int);
			copy_size *= 3;
			if (copy_from_user(&tmp_gpio_l.mode[0], argp, copy_size))
			{
				return -EFAULT;
			}
			ret = set_lowlevel_gpio_info(PIOM,SETGPIOMOD);
			return ret;


		case GETGPIOMOD_LM	:	

			ret = get_lowlevel_gpio_info(PIOM, PIO_OSR);
			copy_size = sizeof(unsigned int);
			copy_size *= 3;
			if (copy_to_user(argp, &tmp_gpio_l.mode[0],copy_size))
			{
				return -EFAULT;
			}
			return ret;

		case SETGPIOVAL_LM	:

			copy_size = sizeof(unsigned int);
			copy_size *= 3;
			if (copy_from_user(&tmp_gpio_l.value[0], argp, copy_size))
			{
				return -EFAULT;
			}
			ret = set_lowlevel_gpio_info(PIOM,SETGPIOVAL);

			return ret;

		case GETGPIOVAL_LM	:	
			ret = 0;
			ret = get_lowlevel_gpio_info(PIOM, PIO_PDSR);
			copy_size = sizeof(unsigned int);
			copy_size *= 3;
			if (copy_to_user(argp, &tmp_gpio_l.value[0],copy_size))
			{
				return -EFAULT;
			}
			return 0;

		case SETGPIOPUL_LM	:
			copy_size = sizeof(unsigned int);
			copy_size *= 3;
			if (copy_from_user(&tmp_gpio_l.pullup[0], argp, copy_size))
			{
				return -EFAULT;
			}

			ret = set_lowlevel_gpio_info(PIOM,SETGPIOPUL);
			return ret;

		case GETGPIOPUL_LM	:
			ret = get_lowlevel_gpio_info(PIOM, PIO_PUSR);
			copy_size = sizeof(unsigned int);
			copy_size *= 3;
			if (copy_to_user(argp, &tmp_gpio_l.pullup[0],copy_size))
			{
				return -EFAULT;
			}
			return ret;

		case SETGPIOMOD_LA	:
			copy_size = sizeof(unsigned int);
			if (copy_from_user(&tmp_gpio_l.mode[0], argp, copy_size))
			{
				return -EFAULT;
			}

			ret = set_lowlevel_gpio_info(PIOA,SETGPIOMOD);
			return ret;

		case GETGPIOMOD_LA	:
			ret = get_lowlevel_gpio_info(PIOA, PIO_OSR);
			copy_size = sizeof(unsigned int);
			if (copy_to_user(argp, &tmp_gpio_l.mode[0],copy_size))
			{
				return -EFAULT;
			}
			return ret;

		case SETGPIOVAL_LA	:
			copy_size = sizeof(unsigned int);
			if (copy_from_user(&tmp_gpio_l.value[0], argp, copy_size))
			{
				return -EFAULT;
			}
			eddy_gpio_l.value[0] = tmp_gpio_l.value[0];
			__raw_writel(tmp_gpio_l.value[0], gpio_membase + PIO_SODR);
			tmp_gpio_l.value[0] = ~(tmp_gpio_l.value[0]) & eddy_gpio_l.en_gpio[0];
			__raw_writel(tmp_gpio_l.value[0], gpio_membase + PIO_CODR);

			return 0;
		case GETGPIOVAL_LA	:
			ret = get_lowlevel_gpio_info(PIOA, PIO_PDSR);
			copy_size = sizeof(unsigned int);
			if (copy_to_user(argp, &tmp_gpio_l.value[0],copy_size))
			{
				return -EFAULT;
			}
			return ret;
		case SETGPIOPUL_LA	:
			copy_size = sizeof(unsigned int);
			if (copy_from_user(&tmp_gpio_l.pullup[0], argp, copy_size))
			{
				return -EFAULT;
			}

			ret = set_lowlevel_gpio_info(PIOA,SETGPIOPUL);
			return ret;
		case GETGPIOPUL_LA	:
			ret = 0;
			ret = get_lowlevel_gpio_info(PIOA, PIO_PUSR);
			copy_size = sizeof(unsigned int);
			copy_size *= 3;
			if (copy_to_user(argp, &tmp_gpio_l.pullup[0],copy_size))
			{
				return -EFAULT;
			}
			return ret;
		case SETGPIOMOD_LB	:
			copy_size = sizeof(unsigned int);
			if (copy_from_user(&tmp_gpio_l.mode[1], argp, copy_size))
			{
				return -EFAULT;
			}

			ret = set_lowlevel_gpio_info(PIOB,SETGPIOMOD);

			return ret;

		case GETGPIOMOD_LB	:
			ret = get_lowlevel_gpio_info(PIOB, PIO_OSR);
			copy_size = sizeof(unsigned int);
			if (copy_to_user(argp, &tmp_gpio_l.mode[1],copy_size))
			{
				return -EFAULT;
			}
			return ret;
		case SETGPIOVAL_LB	:
			copy_size = sizeof(unsigned int);
			if (copy_from_user(&tmp_gpio_l.value[1], argp, copy_size))
			{
				return -EFAULT;
			}

			eddy_gpio_l.value[1] = tmp_gpio_l.value[1];
			__raw_writel(tmp_gpio_l.value[1], gpio_membase + pio[1] + PIO_SODR);
			tmp_gpio_l.value[1] = ~(tmp_gpio_l.value[1]) & eddy_gpio_l.en_gpio[1];
			__raw_writel(tmp_gpio_l.value[1], gpio_membase + pio[1] + PIO_CODR);

			return 0;

		case GETGPIOVAL_LB	:
			ret = get_lowlevel_gpio_info(PIOB, PIO_PDSR);
			copy_size = sizeof(unsigned int);
			if (copy_to_user(argp, &tmp_gpio_l.value[1],copy_size))
			{
				return -EFAULT;
			}
			return ret;
		case SETGPIOPUL_LB	:
			copy_size = sizeof(unsigned int);
			if (copy_from_user(&tmp_gpio_l.pullup[1], argp, copy_size))
			{
				return -EFAULT;
			}

			ret = set_lowlevel_gpio_info(PIOB,SETGPIOPUL);
			return ret;
		case GETGPIOPUL_LB	:
			ret = get_lowlevel_gpio_info(PIOB, PIO_PUSR);
			copy_size = sizeof(unsigned int);
			if (copy_to_user(argp, &tmp_gpio_l.pullup[1],copy_size))
			{
				return -EFAULT;
			}
			return ret;
		case SETGPIOMOD_LC	:
			copy_size = sizeof(unsigned int);
			if (copy_from_user(&tmp_gpio_l.mode[2], argp, copy_size))
			{
				return -EFAULT;
			}

			ret = set_lowlevel_gpio_info(PIOC,SETGPIOMOD);

			return ret;

		case GETGPIOMOD_LC	:
			ret = get_lowlevel_gpio_info(PIOC, PIO_OSR);
			copy_size = sizeof(unsigned int);
			if (copy_to_user(argp, &tmp_gpio_l.mode[2],copy_size))
			{
				return -EFAULT;
			}
			return ret;
		case SETGPIOVAL_LC	:
			copy_size = sizeof(unsigned int);
			if (copy_from_user(&tmp_gpio_l.value[2], argp, copy_size))
			{
				return -EFAULT;
			}

			eddy_gpio_l.value[2] = tmp_gpio_l.value[2];
			__raw_writel(tmp_gpio_l.value[2], gpio_membase + pio[2] + PIO_SODR);
			tmp_gpio_l.value[2] = ~(tmp_gpio_l.value[2]) & eddy_gpio_l.en_gpio[2];
			__raw_writel(tmp_gpio_l.value[2], gpio_membase + pio[2] + PIO_CODR);

			return 0;
		case GETGPIOVAL_LC	:
			ret = get_lowlevel_gpio_info(PIOC, PIO_PDSR);
			copy_size = sizeof(unsigned int);
			if (copy_to_user(argp, &tmp_gpio_l.value[2],copy_size))
			{
				return -EFAULT;
			}
			return ret;
		case SETGPIOPUL_LC	:
			copy_size = sizeof(unsigned int);
			if (copy_from_user(&tmp_gpio_l.pullup[2], argp, copy_size))
			{
				return -EFAULT;
			}

			ret = set_lowlevel_gpio_info(PIOC,SETGPIOPUL);
			return ret;
		case GETGPIOPUL_LC	:
			ret = get_lowlevel_gpio_info(PIOC, PIO_PUSR);
			copy_size = sizeof(unsigned int);
			if (copy_to_user(argp, &tmp_gpio_l.pullup[2],copy_size))
			{
				return -EFAULT;
			}
			return ret;

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

	gpio_membase = (void __iomem *)0xfefff400;

	if (eddy_gpio_miscdev.parent)
		return -EBUSY;
	eddy_gpio_miscdev.parent = &pdev->dev;

	res = misc_register(&eddy_gpio_miscdev);
	if (res)
		return res;

	return 0;
}

static int __exit eddy_gpio_remove(struct platform_device *pdev)
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

	ret = platform_driver_register(&eddy_gpio_driver);

	productID = get_productID();	

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
