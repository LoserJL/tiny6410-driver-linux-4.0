#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <asm/io.h>
#include <plat/map-s3c.h>
#include <plat/map-base.h>

#include "mach/regs-gpio.h"

#include <asm/io.h>
#include <plat/map-s3c.h>
#include <plat/map-base.h>

//#define S3C64XX_GPE_CON                  (S3C64XX_GPE_BASE + 0x00)
//#define S3C64XX_GPE_DAT                  (S3C64XX_GPE_BASE + 0x04)
#define S3C64XX_GPF_CON                  (S3C64XX_GPF_BASE + 0x00)
#define S3C64XX_GPF_DAT                  (S3C64XX_GPF_BASE + 0x04)
#define S3C64XX_GPF_PUD                  (S3C64XX_GPF_BASE + 0x08)

static int backlight_open(struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t backlight_read(struct file *file, char __user *buf, size_t size, loff_t *ppos)
{
	return 0;
}

static ssize_t backlight_write(struct file *file, const char __user *buf, size_t size, loff_t *ppos)
{
	return 0;
}

static struct file_operations backlight_fops = {
	.owner = THIS_MODULE,
	.open = backlight_open,
	.read = backlight_read,
	.write = backlight_write,
};

static struct miscdevice backlight = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "backlight",
	.fops = &backlight_fops,
};

static int __init backlight_init(void)
{
	int ret;
	unsigned long tmp;

	ret = misc_register(&backlight);

	//tmp = __raw_readl(S3C64XX_GPF_CON);
	//tmp &= ~0xf;
	//tmp |= 0x01;
	//__raw_writel(tmp, S3C64XX_GPF_CON);

	//tmp = __raw_readl(S3C64XX_GPF_DAT);
	//tmp = (tmp & ~0x1) | (0x01);
	//__raw_writel(tmp, S3C64XX_GPF_DAT);

	tmp = __raw_readl(S3C64XX_GPF_PUD);
	tmp &= ~(3U <<30);
	tmp |= (2U << 30);
	__raw_writel(tmp, S3C64XX_GPF_PUD);

	tmp = __raw_readl(S3C64XX_GPF_DAT);
	tmp |= (1 << 15);
	__raw_writel(tmp, S3C64XX_GPF_DAT);

	tmp = __raw_readl(S3C64XX_GPF_CON);
	tmp = (tmp & ~(3U << 30)) | (1U << 30);
	__raw_writel(tmp, S3C64XX_GPF_CON);

	return ret;
}

static void __exit backlight_exit(void)
{
	misc_deregister(&backlight);
}

module_init(backlight_init);
module_exit(backlight_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Jianglei");
MODULE_DESCRIPTION("mini6410 backlight drvier");

