#include <linux/module.h>
#include <linux/fb.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>

#include <mach/regs-gpio.h>
#include <mach/map.h>
#include <asm/io.h>

#define MOFPCON		0x7410800C
#define SPCON		0x7F0081A0

#define GPECON		0x7F008180
//#define GPEDAT		0x7F008184
#define GPFCON		0x7F0081A0
//#define GPFDAT		0x7F0081A4
#define GPICON		0x7F008100
#define GPJCON		0x7F008120

static void __iomem *gpecon = NULL;
static void __iomem *gpedat = NULL;
static void __iomem *gpfcon = NULL;
static void __iomem *gpfdat = NULL;
static void __iomem *gpicon = NULL;
static void __iomem *gpjcon = NULL;

static void __iomem *mofpcon = NULL;
static void __iomem *spcon = NULL;

static void __iomem *vidcon0 = NULL;
static void __iomem *vidcon1 = NULL;
static void __iomem *vidtcon0 = NULL;
static void __iomem *vidtcon1 = NULL;
static void __iomem *vidtcon2 = NULL;
static void __iomem *wincon0 = NULL;
static void __iomem *vidosd0a = NULL;
static void __iomem *vidosd0b = NULL;
static void __iomem *vidosd0c = NULL;
static void __iomem *vidw00add0b0 = NULL;
static void __iomem *vidw00add1b0 = NULL;

static volatile unsigned long *HCLK_GATE = NULL;

static u32 s3c_pseudo_palette[16];

struct fb_info *info;

static inline unsigned int chan_to_field(unsigned int chan, struct fb_bitfield *bf)
{
    chan &= 0xffffff;
    chan >>= 24 - bf->length;

    return chan << bf->offset;
}

static int s3c_fb_setcolreg(unsigned int regno, unsigned int red,
                 unsigned int green, unsigned int blue,
                 unsigned int transp, struct fb_info *info)
{
    unsigned int val;
    u32 *pal = info->pseudo_palette;

    if (regno > 24)
        return 1;

    // 用red,green,blue三原色构造出val
    val  = chan_to_field(red,   &info->var.red);
    val |= chan_to_field(green, &info->var.green);
    val |= chan_to_field(blue,  &info->var.blue);
    pal[regno] = val;

    return 0;
}

static struct fb_ops my_fb_ops = {
	.owner		= THIS_MODULE,
	.fb_setcolreg	= s3c_fb_setcolreg,
	.fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
	.fb_imageblit	= cfb_imageblit,
};

static int __init my_framebuffer_init(void)
{
	struct clk *lcd_clk, *clk;
	int hclk, lcd_clkval;

	/* 1 分配一个fb_info */
	/*framebuffer_alloc函数的第一个参数是fb私有数据的大小，可以为0，
	  第二个参数指向拥有这个fb的设备，可以为NULL*/
	info = framebuffer_alloc(0, NULL);

	/* 2 设置 */
	/* 2.1 设置固定参数 即填充fb_fix_screeninfo结构 */
	strcpy(info->fix.id, "mylcd"); //名字
	info->fix.smem_len = 480 * 272 * 2; //xres * yres * byte_per_pixel
	info->fix.type = FB_TYPE_PACKED_PIXELS;
	info->fix.visual = FB_VISUAL_TRUECOLOR; //TFT是真彩色显示
	info->fix.line_length = 480 * 2;

	/* 2.2 设置可变参数 即填充fb_var_screeninfo结构 */
	info->var.xres = 480;			/* visible resolution		*/
	info->var.yres = 272;
	info->var.xres_virtual = 480;
	info->var.yres_virtual = 272;
	//info->var.xoffset = 0; //为0可以不用设置，因为fb_info分配之后都是0
	//info->var.yoffset = 0;
	info->var.bits_per_pixel = 24;

	info->var.red.offset = 16;
	info->var.red.length = 8;
	info->var.green.offset = 8;
	info->var.green.length = 8;
	info->var.blue.offset = 0;
	info->var.blue.length = 8;

	info->var.activate = FB_ACTIVATE_NOW;

	/* 2.3 设置操作函数 */
	info->fbops = &my_fb_ops;

	/* 2.4 其他的一些设置 */
	//info->screen_base = //显存的虚拟地址,在后面分配显存时一起设置
	info->screen_size = 480 * 272 * 2;
	info->pseudo_palette = &s3c_pseudo_palette;

#if 0
	/* 3 硬件设置 */
	/* 3.1 GPIO设置 */
	gpecon = ioremap(GPECON, 8);
	gpedat = gpecon + 1;
	gpfcon = ioremap(GPFCON, 8);
	gpfdat = gpfcon + 1;
	gpicon = ioremap(GPICON, 4);
	gpjcon = ioremap(GPJCON, 4);
	
	__raw_writel(0x00011111, gpecon);
	__raw_writel(0x00000001, gpedat);
	__raw_writel(0x96AAAAAA, gpfcon);
	__raw_writel(0x00002000, gpfdat);
	__raw_writel(0xAAAAAAAA, gpicon);
	__raw_writel(0x00AAAAAA, gpjcon);

	/* 3.2 LCD控制器设置 */
	mofpcon = ioremap(MOFPCON, 4);
	spcon = ioremap(SPCON, 4);
	__raw_writel(__raw_readl(mofpcon) & ~(1<<3), mofpcon);
	__raw_writel((__raw_readl(spcon) & ~(3<<1)) | (1<<0), spcon);

	/*lcd_clk = devm_clk_get(NULL, "lcd");*/
	/*if (IS_ERR(lcd_clk)) {*/
		/*dev_err(NULL, "cannot get lcd clock\n");*/
		/*//retval = PTR_ERR(usb_clk);*/
		/*//goto err_put;*/
	/*}*/
	/*clk_prepare_enable(lcd_clk);*/
	/*mdelay(2);			// let the bus clock stabilise*/
	HCLK_GATE = ioremap(0x7E00F030, 4);
	*HCLK_GATE |= (1<<3);

	//clk = clk_get(NULL, "hclk");
    //hclk = clk_get_rate(clk);
    //lcd_clkval = hclk/9000000 - 1;

	vidcon0 = ioremap(S3C_PA_FB, SZ_16K);
	vidcon1 = vidcon0 + 1;
	vidtcon0 = vidcon0 + 3;
	vidtcon1 = vidcon0 + 4;
	vidtcon2 = vidcon0 + 5;
	wincon0 = vidcon0 + 6;
	vidosd0a = vidcon0 + 11;
	vidosd0b = vidcon0 + 12;
	vidosd0c = vidcon0 + 13;
	vidw00add0b0 = vidcon0 + 27;
	vidw00add1b0 = vidcon0 + 34;
	//__raw_writel(__raw_readl(vidcon0) | (lcd_clkval<<6) | (1<<4), vidcon0);
	__raw_writel((0<<26)|(0<<17)|(0<<16)|(10<<6)|(0<<5)|(1<<4)|(0<<2), vidcon0);
	__raw_writel((0<<7) | (1<<6) | (1<<5) | (0<<4), vidcon1);
	//__raw_writel((2<<16) | (2<<8) | (10<<0), vidtcon0);
	__raw_writel((7<<16) | (8<<8) | (2<<0), vidtcon0);
	//__raw_writel((1<<16) | (1<<8) | (40<<0), vidtcon1);
	__raw_writel((39<<16) | (4<<8) | (2<<0), vidtcon1);
	__raw_writel((271<<11) | (479<<0), vidtcon2);
	__raw_writel(__raw_readl(wincon0) & ~(0xf<<2), wincon0);
	__raw_writel(__raw_readl(wincon0) | (0xb<<2), wincon0);
	__raw_writel((0 << 11) | (0 << 0), vidosd0a);
	__raw_writel((479 << 11) | (271 << 0), vidosd0b);
	__raw_writel(480*272, vidosd0c);
#endif

	/* 4 分配显存 并把地址告诉LCD控制器 */
	//info->screen_base //显存的虚拟地址
	//info->fix.smem_start //显存的物理地址
	info->screen_base = dma_alloc_writecombine(NULL, info->screen_size,
				(dma_addr_t *)&info->fix.smem_start, GFP_KERNEL);
	//__raw_writel(info->fix.smem_start, vidw00add0b0);
	//__raw_writel((info->fix.smem_start + info->fix.smem_len) & 0xffffff, vidw00add1b0);
	/*__raw_writel(((480*4 + 0) * 272) & (0xffffff), vidw00add1b0);*/

	//__raw_writel(__raw_readl(vidcon0) | (3), vidcon0);// 使能LCD控制器输出信号
	//__raw_writel(__raw_readl(wincon0) | (1<<0), wincon0);// 使能窗口0

	/* 5 注册 */
	register_framebuffer(info);

	return 0;
}

static void __exit my_framebuffer_exit(void)
{
	struct clk *lcd_clk; 
      
    unregister_framebuffer(info);  

	//__raw_writel(__raw_readl(vidcon0) & ~(1<<1), vidcon0);// 禁止LCD控制器输出信号
	//__raw_writel(__raw_readl(wincon0) & ~(1<<0), wincon0);// 禁止窗口0

    dma_free_writecombine(NULL, info->screen_size, info->screen_base, 
				info->fix.smem_start);

    /*lcd_clk = clk_get(NULL, "lcd");  */
    /*clk_disable(lcd_clk);  // HCLK_GATE[3]设为0   */

#if 0
    iounmap(vidcon0);  
	iounmap(gpecon);
	iounmap(gpfcon);
    iounmap(gpicon);  
    iounmap(gpjcon);  
    iounmap(mofpcon);  
    iounmap(spcon);  
#endif

    framebuffer_release(info); 
}

module_init(my_framebuffer_init);
module_exit(my_framebuffer_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("JiangLei");
MODULE_DESCRIPTION("my framebuffer driver for tiny6410");

