/* leds driver
 * JL
 * 2020.03.28
 */

#include <linux/moduleparam.h>	//module_param
#include <linux/kernel.h>		//container_of
#include <linux/fs.h>			//struct inode, struct file
#include <linux/platform_device.h> //platform_device
#include <linux/slab.h>			//kzalloc
#include <linux/module.h>		//MODULE_LICENSE, MODULE_AUTHOR, MODULE_DESCRIPTION

#include <asm/io.h>				//__raw_readl, __raw_writel

#include "mini6410_leds.h"

struct led_dev *led_devp;

struct class *leds_class;

static int mini6410_leds_major = MINI6410_LEDS_MAJOR;
module_param(mini6410_leds_major, int, S_IRUGO);

//static volatile unsigned long *led_pin_con = NULL;
//static volatile unsigned long *led_pin_dat = NULL;
static void __iomem *led_pin_con = NULL;
static void __iomem *led_pin_dat = NULL;

static int mini6410_led_open(struct inode *inode, struct file *file)
{
	//printk("open success\n");
	struct led_dev *dev = container_of(inode->i_cdev, struct led_dev, cdev);
	file->private_data = dev;

    return 0;
}
 
static long mini6410_led_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	unsigned tmp;
	
	//printk("mini6410_led_ioctl cmd = %d\n", cmd);

	struct led_dev *dev = file->private_data;
	
	//printk("led_num = %u\n", dev->led_num);

	switch(cmd) {
	case 0:
	case 1:
		tmp = __raw_readl(led_pin_dat);
		if (cmd)
			tmp |= (1 << (4 + dev->led_num));
		else
			tmp &= ~(1 << (4 + dev->led_num));
		__raw_writel(tmp, led_pin_dat);
		return 0;
	default:
		return -EINVAL;
	}
}

static const struct file_operations leds_fops = {
	.owner = THIS_MODULE,
	.open = mini6410_led_open,
	.unlocked_ioctl = mini6410_led_ioctl,
};

static void leds_cdev_setup(struct led_dev *led_dev, int index)
{
	int err;
	dev_t devno = MKDEV(mini6410_leds_major, index);

	cdev_init(&led_dev->cdev, &leds_fops);
	led_dev->cdev.owner = THIS_MODULE;
	err = cdev_add(&led_dev->cdev, devno, 1);
	if (err)
		printk(KERN_NOTICE "Error %d adding leds%d\n", err, index);
}

static int __init mini6410_leds_init(void)
{
	int ret;
	int i;

	dev_t devno = MKDEV(mini6410_leds_major, 0);

	if (mini6410_leds_major) {
		ret = register_chrdev_region(mini6410_leds_major, LED_NUM, "mini6410-leds");
	} else {
		ret = alloc_chrdev_region(&devno, 0, LED_NUM, "mini6410-leds");
		mini6410_leds_major = MAJOR(devno);
	}
	if (ret < 0)
		return ret;

	led_devp = kzalloc(sizeof(struct led_dev) * LED_NUM, GFP_KERNEL);
	if (!led_devp) {
		ret = -ENOMEM;
		goto fail_malloc;
	}

	for (i = 0; i < LED_NUM; i++) {
		leds_cdev_setup(led_devp + i, i);
		(led_devp + i)->led_num = i;
	}

	led_pin_con = ioremap(MINI6410_PA_LEDS, 16);
	led_pin_dat = led_pin_con + 8;

	__raw_writel(__raw_readl(led_pin_con) | (1<<16) | (1<<20) | (1<<24) | (1<<28), led_pin_con);

	leds_class = class_create(THIS_MODULE, "mini6410-leds");
	for (i = 0; i < LED_NUM; i++) {
		device_create(leds_class, NULL, MKDEV(mini6410_leds_major, i), NULL, \
				"mini6410-leds%d", i);

		printk("device_create mini6410-leds%d\n", i);
	}
	return 0;

fail_malloc:
	unregister_chrdev_region(devno, LED_NUM);
	return ret;
}
module_init(mini6410_leds_init);

static void __exit mini6410_leds_exit(void)
{
	int i;

	for (i = 0; i < LED_NUM; i++)
		cdev_del(&(led_devp + i)->cdev);
	kfree(led_devp);

	for (i = 0; i < LED_NUM; i++)
		device_destroy(leds_class, MKDEV(mini6410_leds_major,i));
    class_destroy(leds_class);

	iounmap(led_pin_con);
	unregister_chrdev_region(MKDEV(mini6410_leds_major, 0), LED_NUM);
}
module_exit(mini6410_leds_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Jianglei");
MODULE_DESCRIPTION("mini6410 leds drvier");

