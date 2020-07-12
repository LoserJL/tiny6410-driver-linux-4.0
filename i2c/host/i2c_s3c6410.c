#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/time.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/cpufreq.h>
#include <linux/slab.h>
#include <linux/io.h>
 
#include <asm/irq.h>
 
#include <linux/gpio.h>
#include <plat/gpio-cfg.h>

#define S3C2410_IICSTAT_ARBITR		(1 << 3)
#define S3C2410_IICSTAT_LASTBIT		(1 << 0)
#define S3C2410_IICCON_IRQPEND		(1 << 4)

static struct clk *clk;
static spinlock_t s3c6410_i2c_lock;
static wait_queue_head_t s3c6410_i2c_wait;

struct s3c6410_i2c_regs {
	unsigned int i2ccon;
	unsigned int i2cstat;
	unsigned int i2cadd;
	unsigned int i2cds;
	unsigned int i2clc;
};

struct s3c6410_i2c_xfer_data {
	struct i2c_msg *msg;
	unsigned int msg_num;
	unsigned int msg_idx;
	unsigned int byte_ptr;
	int state;
};

enum s3c6410_i2c_state {
	STATE_IDLE,
	STATE_START,
	STATE_READ,
	STATE_WRITE,
	STATE_STOP,
};

struct s3c6410_i2c_xfer_data s3c6410_i2c_xfer_data;
static struct s3c6410_i2c_regs *s3c6410_i2c_regs;

static inline void s3c6410_i2c_stop(int ret)
{
	printk("i2c Stop\n");

	/* stop the transfer */
	s3c6410_i2c_regs->i2cstat &= ~(1<<5);

	/* 重新设置状态 */
	s3c6410_i2c_xfer_data.state = STATE_STOP;
	s3c6410_i2c_xfer_data.byte_ptr = 0;
	s3c6410_i2c_xfer_data.msg = NULL;
	s3c6410_i2c_xfer_data.msg_idx++;
	s3c6410_i2c_xfer_data.msg_num = 0;
	if (ret)
		s3c6410_i2c_xfer_data.msg_idx = ret;
	printk("i2c master complete %d\n", ret);

	/* 唤醒应用程序 */
	wake_up(&s3c6410_i2c_wait);

	/* disable irq */
	s3c6410_i2c_regs->i2ccon &= (~(1<<5));
}

static void s3c6410_i2c_message_start(struct i2c_msg *msg)
{
	unsigned int addr = (msg->addr & 0x7f) << 1; //获得地址

	printk("i2c Start\n");

	//Master receive mode
	if (msg->flags & I2C_M_RD) {
		addr |= 1;
		s3c6410_i2c_regs->i2cstat = 0x90;
	} else { //Master transmit mode
		s3c6410_i2c_regs->i2cstat = 0xd0;
	}
	s3c6410_i2c_regs->i2cds = addr;
	printk("i2c i2cadd = 0x%08x,i2ccon = 0x%08x,i2cstat = 0x%08x, %s, %d\n",
			s3c6410_i2c_regs->i2cds, s3c6410_i2c_regs->i2ccon, s3c6410_i2c_regs->i2cstat,
			__FUNCTION__, __LINE__);

	ndelay(50); //为了使地址能传到sda线上
	/*
	 * 重新启动传输 这么写是因为i2c发出stop后会清除中断，ack使能 所以必须重新启动i2c总线
	 * 另外i2ccon的bit4也必须注意，在中断退出时，已经清了中断，此时不需要重新设置该位，否则就会出错
	 * */
	s3c6410_i2c_regs->i2ccon |= (1<<7) | (0<<6) | (1<<5) | (11-1);
	s3c6410_i2c_regs->i2cstat |= (1<<5);
	printk("i2c i2cadd = 0x%08x,i2ccon = 0x%08x,i2cstat = 0x%08x, %s, %d\n",
			s3c6410_i2c_regs->i2cds, s3c6410_i2c_regs->i2ccon, s3c6410_i2c_regs->i2cstat,
			__FUNCTION__, __LINE__);
}

static inline int is_lastmsg(void)
{
	return s3c6410_i2c_xfer_data.msg_idx >= (s3c6410_i2c_xfer_data.msg_num - 1);
}

static inline int is_byteend(void)
{
	return s3c6410_i2c_xfer_data.byte_ptr >= s3c6410_i2c_xfer_data.msg->len;
}

static inline int is_bytelast(void)
{
	return s3c6410_i2c_xfer_data.byte_ptr == s3c6410_i2c_xfer_data.msg->len - 1;
}

static irqreturn_t s3c6410_i2c_irq(int irqno, void *dev_id)
{
	printk("i2c state %d\n", s3c6410_i2c_xfer_data.state);
	printk("i2c %s enter, %d\n", __FUNCTION__, __LINE__);

	if (s3c6410_i2c_regs->i2cstat && S3C2410_IICSTAT_ARBITR) {
		printk("i2c deal with arbitration loss\n");
		return -ENODATA;
	}

	switch (s3c6410_i2c_xfer_data.state) {
	case STATE_IDLE:
		printk("%s: i2c called in STATE_IDLE\n", __FUNCTION__);
		return -ENODEV;
	
	case STATE_START:
		/* 没有ack 返回 错误 */
		if (s3c6410_i2c_regs->i2cstat & S3C2410_IICSTAT_LASTBIT) {
			printk("i2c ack was not received\n");
			s3c6410_i2c_stop(-ENXIO);
			break;
		}

		/* just i2c probe to find devices */
		if (is_lastmsg() && s3c6410_i2c_xfer_data.msg->len == 0) {
			printk("i2c ack was received\n");
			s3c6410_i2c_stop(0);
			break;
		}

		/* 下一个状态 */
		if (s3c6410_i2c_xfer_data.msg->flags & I2C_M_RD)
			s3c6410_i2c_xfer_data.state = STATE_READ;
		else
			s3c6410_i2c_xfer_data.state = STATE_WRITE;

		if (s3c6410_i2c_xfer_data.state == STATE_READ)
			goto prepare_read;

	case STATE_WRITE:
		/* 如果这个消息中还有数据，那么把该数据写入i2c总线 */
		if (!is_byteend()) {
			printk("i2c write next byte\n");
			s3c6410_i2c_regs->i2cds = s3c6410_i2c_xfer_data.msg->buf[s3c6410_i2c_xfer_data.byte_ptr++];
			ndelay(50);
			break;
		} else if (!is_lastmsg()) { /* 还有消息 */
			printk("i2c write next message\n");
			s3c6410_i2c_xfer_data.byte_ptr = 0;
			s3c6410_i2c_xfer_data.msg_idx++;
			s3c6410_i2c_xfer_data.msg++;	//下个msg

			s3c6410_i2c_message_start(s3c6410_i2c_xfer_data.msg);
			s3c6410_i2c_xfer_data.state = STATE_START;
			break;
		} else {
			printk("i2c write no message no byte\n");
			s3c6410_i2c_stop(0);
			break;
		}
	
	case STATE_READ:
		/* 读出数据 */
		s3c6410_i2c_xfer_data.msg->buf[s3c6410_i2c_xfer_data.byte_ptr++] = s3c6410_i2c_regs->i2cds;

//第一次start时，发完地址后并没有数据需要读取
prepare_read:
		/* 消息的最后一个字节 */
		if (is_bytelast()) {
			printk("i2c read last byte\n");
			/* 而且还是最后一个消息，此时不发送ack */
			if (is_lastmsg()) {
				printk("i2c read last message\n");
				s3c6410_i2c_regs->i2ccon &= ~(1<<7);
			}
		} else if (is_byteend()) { /* 否则该消息中没有数据了 */
			printk("i2c read no byte\n");
			/* 而且还是最后一个消息，此时停止 */
			if (is_lastmsg()) {
				printk("i2c read last message no byte\n");
				printk("i2c read send stop");
				s3c6410_i2c_stop(0);
			} else { /* 此时还有消息 */
				printk("i2c read next transfer\n");
				s3c6410_i2c_xfer_data.byte_ptr = 0;
				s3c6410_i2c_xfer_data.msg_idx++;
				s3c6410_i2c_xfer_data.msg++;
			}
		}
		break;
	}

	//清中断
	s3c6410_i2c_regs->i2ccon &= ~(S3C2410_IICCON_IRQPEND);
	return IRQ_HANDLED;
}

static int s3c6410_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num)
{
	static int cnt = 0;
	int ret, timeout;
	printk("s3c6410_i2c_xfer cnt = %d\n", ++cnt);

	spin_lock_irq(&s3c6410_i2c_lock);

	/* 初始化 msg */
	/* 应用程序会调用算法函数，并且会发来几个msg，驱动需要将这些msg读进来或者发出去 */
	s3c6410_i2c_xfer_data.msg = msgs;
	s3c6410_i2c_xfer_data.msg_num = num;
	s3c6410_i2c_xfer_data.byte_ptr = 0;
	s3c6410_i2c_xfer_data.msg_idx = 0;
	s3c6410_i2c_xfer_data.state = STATE_START;

	s3c6410_i2c_message_start(msgs);

	spin_unlock_irq(&s3c6410_i2c_lock);

	timeout = wait_event_timeout(s3c6410_i2c_wait, s3c6410_i2c_xfer_data.msg_num == 0, HZ * 5);

	ret = s3c6410_i2c_xfer_data.msg_idx;

	if (timeout == 0) {
		dev_dbg(NULL, "i2c:timeout\n");
		return -ETIMEDOUT;
	} else if (ret != num)
		dev_dbg(NULL, "i2c:incomplete xfer\n");
	else
		dev_dbg(NULL, "i2c:complete xfer\n");

	udelay(10);
	return ret;
}

static u32 s3c6410_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL | I2C_FUNC_PROTOCOL_MANGLING;
}

static const struct i2c_algorithm s3c6410_i2c_algo = {
	.master_xfer	= s3c6410_i2c_xfer, //i2c传输
	.functionality	= s3c6410_i2c_func,
};

/* 1. 分配/设置i2c_adapter */
static struct i2c_adapter s3c6410_i2c_adapter = {
	.owner	= THIS_MODULE,
	.class	= I2C_CLASS_HWMON | I2C_CLASS_SPD,
	.name	= "s3c6410_i2c",
	.algo	= &s3c6410_i2c_algo,
};

static void s3c6410_i2c_init(void)
{
	/* 3.3.1 配置i2c相关gpio */
	
	/* 初始化i2c相关寄存器 */
	/* bit[7] = 1, 使能ACK
	 * bit[6] = 0, IICCLK = PCLK/16
	 * bit[5] = 1, 使能中断
	 * bit[3:0] = (11-1), Tx clock = IICCLK/16
	 * PCLK = , IICCLK = , Tx Clock = 
	 */
	s3c6410_i2c_regs->i2ccon = (1<<7) | (0<<6) | (1<<5) | (11-1);
	s3c6410_i2c_regs->i2cadd = 0x10;
	s3c6410_i2c_regs->i2clc = (3 << 0) | (1<<2);
	s3c6410_i2c_regs->i2cstat = (1<<4);
}

static int s3c6410_i2c_probe(struct platform_device *pdev)
{
	int retval;

	/* 2. 辅助变量操作 */
	spin_lock_init(&s3c6410_i2c_lock);
	init_waitqueue_head(&s3c6410_i2c_wait);

	/* 3. 硬件操作 */
	/* 3.1 使能相关时钟 */
	clk = devm_clk_get(&pdev->dev, "i2c");
	if (IS_ERR(clk)) {
		dev_err(&pdev->dev, "cannot s3c6410 i2c clock\n");
		retval = PTR_ERR(clk);
		goto err_put;
	}
	clk_prepare_enable(clk);

	/* 3.2 映射相关寄存器 */
	s3c6410_i2c_regs = ioremap(0x7F004000, sizeof(struct s3c6410_i2c_regs));
	printk("ioremap i2c registers %p \n", s3c6410_i2c_regs);

	/* 3.3 初始化i2c */
	s3c6410_i2c_init();

	/* 3.4 注册中断 */
	retval = request_irq(IRQ_IIC, s3c6410_i2c_irq, IRQF_DISABLED, "s3c6410_i2c", NULL);
	if (retval) {
		dev_err(NULL, "cannot request i2c IRQ %d\n", IRQ_IIC);
		free_irq(IRQ_IIC, NULL);
	}

	/* 4. 注册i2_adapter */
	i2c_add_numbered_adapter(&s3c6410_i2c_adapter);
	return 0;

err_put:
	clk_disable_unprepare(clk);
	return retval;
}

static int s3c6410_i2c_remove(struct platform_device *pdev)
{
	i2c_del_adapter(&s3c6410_i2c_adapter);
	free_irq(IRQ_IIC, NULL);
	iounmap(s3c6410_i2c_regs);
	clk_disable_unprepare(clk);
	clk_put(clk);
	
	return 0;
}

//static struct platform_device_id s3c6410_i2c_driver_ids[] = {
	//{
		//.name		= "s3c-i2c",
	//},
	//{ }
//};
//module_device_table(platform,  s3c6410_i2c_driver_ids);

struct platform_driver s3c6410_i2c_driver = {
	.probe		= s3c6410_i2c_probe,
	.remove		= s3c6410_i2c_remove,
	//.id_table	= s3c6410_i2c_driver_ids,
	.driver		= {
		.name	= "s3c-i2c",
		//
	},
};

module_platform_driver(s3c6410_i2c_driver);

MODULE_LICENSE("GPL v2");
