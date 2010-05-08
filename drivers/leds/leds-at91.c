/*
 * AT91 GPIO based LED driver
 *
 * Copyright (C) 2006 David Brownell
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/leds.h>

#include <asm/arch/board.h>
#include <asm/arch/gpio.h>

static LIST_HEAD(at91_led_list);	/* list of AT91 LEDs */

struct at91_led {
	struct led_classdev	cdev;
	struct list_head	list;
	struct at91_gpio_led	*led_data;
};

/*
 * Change the state of the LED.
 */
static void at91_led_set(struct led_classdev *cdev, enum led_brightness value)
{
	struct at91_led	*led = container_of(cdev, struct at91_led, cdev);
	short		active = (value == LED_OFF);

	if (led->led_data->flags & 1)	/* active high/low? */
		active = !active;
	at91_set_gpio_value(led->led_data->gpio, active);
}

static int __devexit at91_led_remove(struct platform_device *pdev)
{
	struct at91_led		*led;

	list_for_each_entry (led, &at91_led_list, list)
		led_classdev_unregister(&led->cdev);

#warning "Free allocated memory"
	// TODO: Free memory.	kfree(led);

	return 0;
}

static int __init at91_led_probe(struct platform_device *pdev)
{
	int			status = 0;
	struct at91_gpio_led	*pdata = pdev->dev.platform_data;
	unsigned		nr_leds;
	struct at91_led		*led;

	if (!pdata)
		return -ENODEV;

	nr_leds = pdata->index;		/* first index stores number of LEDs */

	while (nr_leds--) {
		led = kzalloc(sizeof(struct at91_led), GFP_KERNEL);
		if (!led) {
			dev_err(&pdev->dev, "No memory for device\n");
			status = -ENOMEM;
			goto cleanup;
		}
		led->led_data = pdata;
		led->cdev.name = pdata->name;
		led->cdev.brightness_set = at91_led_set,
		led->cdev.default_trigger = pdata->trigger;

		status = led_classdev_register(&pdev->dev, &led->cdev);
		if (status < 0) {
			dev_err(&pdev->dev, "led_classdev_register failed - %d\n", status);
cleanup:
			at91_led_remove(pdev);
			break;
		}
		list_add(&led->list, &at91_led_list);
		pdata++;
	}
	return status;
}

#ifdef CONFIG_PM
static int at91_led_suspend(struct platform_device *dev, pm_message_t state)
{
	struct at91_led	*led;

	list_for_each_entry (led, &at91_led_list, list)
		led_classdev_suspend(&led->cdev);

	return 0;
}

static int at91_led_resume(struct platform_device *dev)
{
	struct at91_led	*led;

	list_for_each_entry (led, &at91_led_list, list)
		led_classdev_resume(&led->cdev);

	return 0;
}
#else
#define	at91_led_suspend	NULL
#define	at91_led_resume		NULL
#endif

static struct platform_driver at91_led_driver = {
	.probe		= at91_led_probe,
	.remove		= __devexit_p(at91_led_remove),
	.suspend	= at91_led_suspend,
	.resume		= at91_led_resume,
	.driver		= {
		.name	= "at91_leds",
		.owner	= THIS_MODULE,
	},
};

static int __init at91_led_init(void)
{
	return platform_driver_register(&at91_led_driver);
}
module_init(at91_led_init);

static void __exit at91_led_exit(void)
{
	platform_driver_unregister(&at91_led_driver);
}
module_exit(at91_led_exit);

MODULE_DESCRIPTION("AT91 GPIO LED driver");
MODULE_AUTHOR("David Brownell");
MODULE_LICENSE("GPL");
