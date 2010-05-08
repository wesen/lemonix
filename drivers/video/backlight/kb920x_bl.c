/*
 * Backlight Driver for KB9202
 *
 * Copyright (c) 2006 KwikByte
 *
 * Based on Sharp's Corgi Backlight Driver
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/fb.h>
#include <linux/backlight.h>

#include <asm/arch/gpio.h>

/* The backlight is on(1)/off(0) */
#define	KB9202_DEFAULT_INTENSITY	1
#define	KB9202_MAX_INTENSITY		1

static int kb9202bl_suspended;
static int current_intensity = 0;
static DEFINE_SPINLOCK(bl_lock);

static int kb9202bl_set_intensity(struct backlight_device *bd)
{
	unsigned long flags;
	int intensity = bd->props.brightness;

	if (bd->props.power != FB_BLANK_UNBLANK)
		intensity = 0;
	if (bd->props.fb_blank != FB_BLANK_UNBLANK)
		intensity = 0;
	if (kb9202bl_suspended)
		intensity = 0;

	if ((!current_intensity) && (bd->props.power == FB_BLANK_UNBLANK))
		intensity = 1;

	spin_lock_irqsave(&bl_lock, flags);
	if (intensity)
		gpio_set_value(AT91_PIN_PC23, 1);
	else
		gpio_set_value(AT91_PIN_PC23, 0);
	spin_unlock_irqrestore(&bl_lock, flags);

	current_intensity = intensity;

	return 0;
}

static int kb9202bl_get_intensity(struct backlight_device *bd)
{
	return current_intensity;
}

static struct backlight_ops kb9202bl_ops = {
	.get_brightness	= kb9202bl_get_intensity,
	.update_status	= kb9202bl_set_intensity,
};

static int __init kb9202bl_probe(struct platform_device *pdev)
{
	struct backlight_device *bd;

	bd = backlight_device_register ("kb9202-bl", &pdev->dev, NULL, &kb9202bl_ops);
	if (IS_ERR(bd))
		return PTR_ERR(bd);

	platform_set_drvdata(pdev, bd);

	bd->props.max_brightness = KB9202_MAX_INTENSITY;
	bd->props.brightness = KB9202_DEFAULT_INTENSITY;
	(void) kb9202bl_set_intensity(bd);

	return 0;
}

static int kb9202bl_remove(struct platform_device *pdev)
{
	struct backlight_device *bd = platform_get_drvdata(pdev);

	bd->props.brightness = 0;
	bd->props.power = 0;
	(void) kb9202bl_set_intensity(bd);

	backlight_device_unregister(bd);

	return 0;
}

#ifdef CONFIG_PM
static int kb9202bl_suspend(struct platform_device *dev, pm_message_t state)
{
	struct backlight_device *bd = platform_get_drvdata(pdev);

	kb9202bl_suspended = 1;
	(void) kb9202bl_set_intensity(bd);
	return 0;
}

static int kb9202bl_resume(struct platform_device *dev)
{
	struct backlight_device *bd = platform_get_drvdata(pdev);

	kb9202bl_suspended = 0;
	(void) kb9202bl_set_intensity(bd);
	return 0;
}
#else
#define kb9202bl_suspend	NULL
#define kb9202bl_resume		NULL
#endif

static struct platform_driver kb9202bl_driver = {
	.probe		= kb9202bl_probe,
	.remove		= kb9202bl_remove,
	.suspend	= kb9202bl_suspend,
	.resume		= kb9202bl_resume,
	.driver		= {
		.name	= "kb9202-bl",
		.owner	= THIS_MODULE,
	},
};

static struct platform_device *kb9202bl_device;

static int __init kb9202bl_init(void)
{
	int ret;

	ret = platform_driver_register(&kb9202bl_driver);
	if (!ret) {
		kb9202bl_device = platform_device_alloc("kb9202-bl", -1);
		if (!kb9202bl_device)
			return -ENOMEM;

		ret = platform_device_add(kb9202bl_device);
		if (ret) {
			platform_device_put(kb9202bl_device);
			platform_driver_unregister(&kb9202bl_driver);
		}
	}
	return ret;
}

static void __exit kb9202bl_exit(void)
{
	platform_device_unregister(kb9202bl_device);
	platform_driver_unregister(&kb9202bl_driver);
}

module_init(kb9202bl_init);
module_exit(kb9202bl_exit);

MODULE_AUTHOR("KwikByte <kb9200_dev@kwikbyte.com>");
MODULE_DESCRIPTION("KB9202 Backlight Driver");
MODULE_LICENSE("GPL");
