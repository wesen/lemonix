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
#include <linux/delay.h>
#include <linux/serial.h>

#include <asm/system.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/bitops.h>

#include <asm/ioctls.h>
#include <asm/arch/gpio.h>
#include "../i2c/chips/alpu_bypass.h"

#define GPIO_MAJOR 99

//-->shlee debug 

#undef SHLEE_DEBUG 
#define GPIO_DEBUG 

//<--
//--> shlee_define
#define PULLUP_ENABLE 1
#define PULLUP_DISABLE 0
#define GPIO_HIGH 1
#define GPIO_LOW 0
/* error return */
#define ERR_PRO 1
#define ERR_CNT 2
//<--
/* interface type */
int interface_type = 0xff ;

int get_productID(void);
int init_product(void);

int productID = 0x00;

int get_interface_type()
{
	return interface_type;
}

EXPORT_SYMBOL(get_interface_type);

int gpio_ioctl(struct inode *inode, struct file *flip, unsigned int command, unsigned long arg);

static ssize_t gpio_read(struct file *file_p, char *data, size_t size, loff_t *loff)
{
	return 0;
}
static ssize_t gpio_write(struct file *file_p, const char *data, size_t size, loff_t *loff)
{
	return 0;
}
static int gpio_open(struct inode *inode, struct file *file_p)
{
	return 0;
}
static int gpio_release(struct inode *inode, struct file *file_p)
{

	return 0;
}

static struct file_operations gpio_ctl_fops = {
llseek: no_llseek,
		read:   gpio_read,
		write:  gpio_write,
		ioctl:  gpio_ioctl,
		open:   gpio_open,
		release: gpio_release,

};
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
	int gpio_num = 0;
	char tmp = 0;
return 0;
	switch(productID)
	{
		case EDDY_CPU:
			/* gpio init */	
			for(gpio_num = AT91_PIN_PC16 ; gpio_num <= AT91_PIN_PC31 ; gpio_num++)
			{
				at91_set_gpio_output(gpio_num,GPIO_HIGH);
			}
			break;
		case EDDY_S1_PIN:
			/* rs232 on */
			for(gpio_num = AT91_PIN_PC16 ; gpio_num <= AT91_PIN_PC31 ; gpio_num++)
			{
				at91_set_gpio_output(gpio_num,GPIO_LOW);
			}
			at91_set_gpio_output(AT91_PIN_PC16,GPIO_LOW);
			at91_set_gpio_output(AT91_PIN_PC17,GPIO_LOW);
			at91_set_gpio_output(AT91_PIN_PC18,GPIO_HIGH);
			at91_set_gpio_output(AT91_PIN_PC19,GPIO_LOW);
			/* gpio init */	
			for(gpio_num = AT91_PIN_PC28 ; gpio_num <= AT91_PIN_PC31 ; gpio_num++)
			{
				at91_set_gpio_output(gpio_num,GPIO_HIGH);
			}
			break;
		case EDDY_S1_PIN_C:
			for(gpio_num = AT91_PIN_PC16 ; gpio_num <= AT91_PIN_PC31 ; gpio_num++)
			{
				at91_set_gpio_output(gpio_num,GPIO_HIGH);
			}
			at91_set_gpio_output(AT91_PIN_PC16,GPIO_HIGH);
			at91_set_gpio_output(AT91_PIN_PC17,GPIO_LOW);
			at91_set_gpio_output(AT91_PIN_PC18,GPIO_HIGH);
			at91_set_gpio_output(AT91_PIN_PC19,GPIO_LOW);
			/* gpio init */	
			for(gpio_num = AT91_PIN_PC28 ; gpio_num <= AT91_PIN_PC31 ; gpio_num++)
			{
				at91_set_gpio_output(gpio_num,GPIO_HIGH);
			}
			break;
		case EDDY_S1_DB9:
			/* rs232 on */
			for(gpio_num = AT91_PIN_PC16 ; gpio_num <= AT91_PIN_PC31 ; gpio_num++)
			{
				at91_set_gpio_output(gpio_num,GPIO_HIGH);
			}
			at91_set_gpio_output(AT91_PIN_PC16,GPIO_LOW);
			at91_set_gpio_output(AT91_PIN_PC17,GPIO_LOW);
			at91_set_gpio_output(AT91_PIN_PC18,GPIO_HIGH);
			at91_set_gpio_output(AT91_PIN_PC19,GPIO_LOW);
			break;
		case EDDY_S1_DB9_C:
			/* rs232 driver Shutdown */
			for(gpio_num = AT91_PIN_PC16 ; gpio_num <= AT91_PIN_PC31 ; gpio_num++)
			{
				at91_set_gpio_output(gpio_num,GPIO_HIGH);
			}
			at91_set_gpio_output(AT91_PIN_PC16,GPIO_HIGH);
			at91_set_gpio_output(AT91_PIN_PC17,GPIO_LOW);
			at91_set_gpio_output(AT91_PIN_PC18,GPIO_HIGH);
			at91_set_gpio_output(AT91_PIN_PC19,GPIO_LOW);
			break;
		case EDDY_S1_POE:
			//init gpio
			for(gpio_num = AT91_PIN_PC16 ; gpio_num <= AT91_PIN_PC31 ; gpio_num++)
			{
				at91_set_gpio_output(gpio_num,GPIO_HIGH);
			}
			/* rs232 on */
			at91_set_gpio_output(AT91_PIN_PC16,GPIO_LOW);
			at91_set_gpio_output(AT91_PIN_PC17,GPIO_LOW);
			at91_set_gpio_output(AT91_PIN_PC18,GPIO_HIGH);
			at91_set_gpio_output(AT91_PIN_PC19,GPIO_LOW);
			break;
		case EDDY_S1_POE_C:
			for(gpio_num = AT91_PIN_PC16 ; gpio_num <= AT91_PIN_PC31 ; gpio_num++)
			{
				at91_set_gpio_output(gpio_num,GPIO_HIGH);
			}
			/* rs232 driver Shutdown */
			at91_set_gpio_output(AT91_PIN_PC16,GPIO_HIGH);
			at91_set_gpio_output(AT91_PIN_PC17,GPIO_LOW);
			at91_set_gpio_output(AT91_PIN_PC18,GPIO_HIGH);
			at91_set_gpio_output(AT91_PIN_PC19,GPIO_LOW);
			break;
		
		
		case EDDY_S2_M:
			/* rs232 on */	
			for(gpio_num = AT91_PIN_PC16 ; gpio_num <= AT91_PIN_PC31 ; gpio_num++)
			{
				at91_set_gpio_output(gpio_num,GPIO_HIGH);
			}
			/* port 0 */
			at91_set_gpio_output(AT91_PIN_PC16,GPIO_LOW);
			at91_set_gpio_output(AT91_PIN_PC17,GPIO_LOW);
			at91_set_gpio_output(AT91_PIN_PC18,GPIO_HIGH);
			at91_set_gpio_output(AT91_PIN_PC19,GPIO_LOW);
			
			/* port 1 */
			at91_set_gpio_output(AT91_PIN_PC20,GPIO_LOW);
			at91_set_gpio_output(AT91_PIN_PC21,GPIO_LOW);
			at91_set_gpio_output(AT91_PIN_PC22,GPIO_HIGH);
			at91_set_gpio_output(AT91_PIN_PC23,GPIO_LOW);
			break;
			
		case EDDY_S2_M_C:
			/* rs232 driver Shutdown */
			for(gpio_num = AT91_PIN_PC16 ; gpio_num <= AT91_PIN_PC31 ; gpio_num++)
			{
				at91_set_gpio_output(gpio_num,GPIO_HIGH);
			}
			
			/* port 0 default 422ptp*/ 
			at91_set_gpio_output(AT91_PIN_PC16,GPIO_HIGH);
			at91_set_gpio_output(AT91_PIN_PC17,GPIO_LOW);
			at91_set_gpio_output(AT91_PIN_PC18,GPIO_HIGH);
			at91_set_gpio_output(AT91_PIN_PC19,GPIO_LOW);
			
			/* port 1 default 422ptp*/ 
			at91_set_gpio_output(AT91_PIN_PC20,GPIO_HIGH);
			at91_set_gpio_output(AT91_PIN_PC21,GPIO_LOW);
			at91_set_gpio_output(AT91_PIN_PC22,GPIO_HIGH);
			at91_set_gpio_output(AT91_PIN_PC23,GPIO_LOW);
			
			break;
			
		case EDDY_WS1_TTL:
			at91_set_gpio_output(AT91_PIN_PC16,GPIO_LOW);
			at91_set_gpio_output(AT91_PIN_PC17,GPIO_LOW);
			at91_set_gpio_output(AT91_PIN_PC19,GPIO_HIGH);
			at91_set_gpio_output(AT91_PIN_PC19,GPIO_LOW);
			break;
		case EDDY_WS1_TTL_C:
			break;
		case EDDY_WS1_PIN:
			at91_set_gpio_output(AT91_PIN_PC16,GPIO_LOW);
			at91_set_gpio_output(AT91_PIN_PC17,GPIO_LOW);
			at91_set_gpio_output(AT91_PIN_PC19,GPIO_HIGH);
			at91_set_gpio_output(AT91_PIN_PC19,GPIO_LOW);
			break;
		case EDDY_WS1_PIN_C:
			break;
		case EDDY_WS1_DB9:
			at91_set_gpio_output(AT91_PIN_PC16,GPIO_LOW);
			at91_set_gpio_output(AT91_PIN_PC17,GPIO_LOW);
			at91_set_gpio_output(AT91_PIN_PC19,GPIO_HIGH);
			at91_set_gpio_output(AT91_PIN_PC19,GPIO_LOW);
			break;
		case EDDY_WS1_DB9_C:
			break;
		
		case EDDY_DK_C:
			for(gpio_num = AT91_PIN_PC16 ; gpio_num <= AT91_PIN_PC31 ; gpio_num++)
			{
				at91_set_gpio_output(gpio_num,GPIO_HIGH);
			}
			/* rs232 driver Shutdown */
			at91_set_gpio_output(AT91_PIN_PC16,GPIO_LOW);
			at91_set_gpio_output(AT91_PIN_PC17,GPIO_LOW);
			at91_set_gpio_output(AT91_PIN_PC18,GPIO_HIGH);
			at91_set_gpio_output(AT91_PIN_PC18,GPIO_HIGH);
			break;
			
		default:
			break;

	}
	return 0;


}
int check_gpio_num(int tmp)
{
	if(productID == EDDY_CPU)
	{
		if(AT91_PIN_PC16 > tmp ||  AT91_PIN_PC31 < tmp )
		{
			return -ERR_CNT;
		}
		else 
		{
			return 0;
		}
	}
	else if(productID == EDDY_S1_PIN || productID== EDDY_S1_PIN_C)
	{
		if((AT91_PIN_PC28) > tmp ||  AT91_PIN_PC31 < tmp )
		{
			return -ERR_CNT;
		}
		else 
		{
			return 0;
		}

	}
	else if(productID == EDDY_S2_M || productID== EDDY_S2_M_C)
	{
		if((AT91_PIN_PC28) > tmp ||  AT91_PIN_PC31 < tmp )
		{
			return -ERR_CNT;
		}
		else 
		{
			return 0;
		}

	}
	else if(productID == EDDY_DK_C)
	{
		
		if((AT91_PIN_PC20) > tmp ||  AT91_PIN_PC31 < tmp )
		{
			return -ERR_CNT;
		}
		else 
		{
			return 0;
		}
	}
	else
	{
		return -ERR_PRO;
	}

}
int gpio_ioctl(struct inode *inode,
		struct file *flip,
		unsigned int command,
		unsigned long arg)
{
	int ret=0;
	int tmp = 0;
	switch (command) {
		case INIT_PRODUCT:
			productID = get_productID();	
//			printk("init_product\n");
			/* all common setting */

			/* ready led set mode out */
			at91_set_gpio_output(AT91_PIN_PC4,GPIO_HIGH);
			/* reset swith read */
			at91_set_gpio_input(AT91_PIN_PC16,PULLUP_ENABLE);
			/* hardware reset */
			//at91_set_gpio_output(AT91_PIN_PB19,GPIO_LOW);
			/* each product dependent setting */ 
			init_product();
			break;
		case GET_PRODUCTID:
//			printk("get_productid\n");
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
#if 0 
case INTERFACESEL:
			if(copy_from_user(&tmp, (void *) arg, sizeof(int)))
			{
				return -EFAULT;
			}
			else
			{
//				printk("interface select = %d \n",tmp);
				switch(tmp)
				{
					case RS232:
						/* rs232 on */
				//		at91_set_gpio_output(AT91_PIN_PC19,GPIO_HIGH);
						break;
					case RS422PTP:
					if(productID == EDDY_DK_C)
					{
						/* receive enable */
						at91_set_gpio_output(AT91_PIN_PC16,GPIO_LOW);
						/* transmit enable */
						at91_set_gpio_output(AT91_PIN_PC17,GPIO_LOW);
						/* rs422 full-Duplex select */
						at91_set_gpio_output(AT91_PIN_PC18,GPIO_HIGH);
						/* rs232 driver Shutdown */
						at91_set_gpio_output(AT91_PIN_PC19,GPIO_LOW);
					
					}
					else
					{
						/* receive enable */
						at91_set_gpio_output(AT91_PIN_PC16,GPIO_HIGH);
						/* transmit enable */
						at91_set_gpio_output(AT91_PIN_PC17,GPIO_LOW);
						/* rs422 full-Duplex select */
						at91_set_gpio_output(AT91_PIN_PC18,GPIO_HIGH);
						/* rs231 driver Shutdown */
						at91_set_gpio_output(AT91_PIN_PC19,GPIO_LOW);
					}
					break;
					case RS422MD:
						break;
					case RS485NE:
					if(productID == EDDY_DK_C)
					{
						/* receive enable */
						at91_set_gpio_output(AT91_PIN_PC16,GPIO_LOW);
						/* transmit enable */
						at91_set_gpio_output(AT91_PIN_PC17,GPIO_LOW);
						/* rs422 full-Duplex select */
						at91_set_gpio_output(AT91_PIN_PC18,GPIO_LOW);
						/* rs232 driver Shutdown */
						at91_set_gpio_output(AT91_PIN_PC19,GPIO_HIGH);
					
					}
					else
					{
						/* receive enable */
						at91_set_gpio_output(AT91_PIN_PC16,GPIO_HIGH);
						/* transmit disable */
						at91_set_gpio_output(AT91_PIN_PC17,GPIO_LOW);
						/* rs422 full-Duplex select */
						at91_set_gpio_output(AT91_PIN_PC18,GPIO_LOW);
						/* rs232 driver Shutdown */
						at91_set_gpio_output(AT91_PIN_PC19,GPIO_HIGH);
					}
					break;
					case RS485E:
						break;
					
					case RS422PTP1:
						/* receive enable */
						at91_set_gpio_output(AT91_PIN_PC20,GPIO_HIGH);
						/* transmit enable */
						at91_set_gpio_output(AT91_PIN_PC21,GPIO_LOW);
						/* rs422 full-Duplex select */
						at91_set_gpio_output(AT91_PIN_PC22,GPIO_HIGH);
						/* rs232 driver Shutdown */
						at91_set_gpio_output(AT91_PIN_PC23,GPIO_LOW);
					
					break;
					case RS422MD1:
						break;
					case RS485NE1:
						/* receive enable */
						at91_set_gpio_output(AT91_PIN_PC20,GPIO_HIGH);
						/* transmit enable */
						at91_set_gpio_output(AT91_PIN_PC21,GPIO_LOW);
						/* rs422 full-Duplex select */
						at91_set_gpio_output(AT91_PIN_PC22,GPIO_LOW);
						/* rs232 driver Shutdown */
						at91_set_gpio_output(AT91_PIN_PC23,GPIO_HIGH);
					
					break;
					case RS485E1:
						break;
					default :
						break;
				}
				interface_type = tmp;
			}
			break;
#endif 
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
			//printk("set gpio mode outret = %d, tmp = %d \n",ret,tmp);
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
	return ret;

}

static int __init gpio_init(void)
{
	//	SetGPIODir(W802_11B_LED, GP_OUTPUT);
	//	SetGPIODir(W802_11G_LED, GP_OUTPUT);
	//	SetGPIODir(RDY_LED, GP_OUTPUT);
	//	SetGPIODir(POLL_RESET, GP_INPUT);

	//	WriteGPIOData(W802_11B_LED, LEDON);
	//	WriteGPIOData(W802_11G_LED, LEDOFF);
	//	WriteGPIOData(RDY_LED, LEDON);

	register_chrdev(GPIO_MAJOR, "eddy_gpio", &gpio_ctl_fops);

	productID = get_productID();	
	
	/* all common setting */

#if 0
	/* ready led set mode out */
	at91_set_gpio_output(AT91_PIN_PC4,GPIO_HIGH);
	/* reset swith read */
	at91_set_gpio_input(AT91_PIN_PC14,PULLUP_ENABLE);
	/* hardware reset */
	at91_set_gpio_output(AT91_PIN_PB19,GPIO_HIGH);
	/* each product dependent setting */ 
	/* rs232 on */
	at91_set_gpio_output(AT91_PIN_PC16,GPIO_HIGH);
	at91_set_gpio_output(AT91_PIN_PC17,GPIO_HIGH);
	at91_set_gpio_output(AT91_PIN_PC18,GPIO_LOW);
	at91_set_gpio_output(AT91_PIN_PC19,GPIO_LOW);

	at91_set_gpio_value(AT91_PIN_PC16,GPIO_HIGH);
	at91_set_gpio_value(AT91_PIN_PC17,GPIO_HIGH);
	at91_set_gpio_value(AT91_PIN_PC18,GPIO_LOW);
	at91_set_gpio_value(AT91_PIN_PC19,GPIO_LOW);

	printk(" gpio value = %d,%d,%d,%d \n",at91_get_gpio_value(AT91_PIN_PC16),
			at91_get_gpio_value(AT91_PIN_PC17),
			at91_get_gpio_value(AT91_PIN_PC18),
			at91_get_gpio_value(AT91_PIN_PC19));
#endif 

//	printk("product id  = 0x%x \n\n",productID);

	return 0;
}

module_init(gpio_init);

