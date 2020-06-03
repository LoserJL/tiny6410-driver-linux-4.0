#include <linux/module.h>
#include <linux/serial_core.h>
#include <linux/pm.h>
#include <linux/io.h>
#include <linux/console.h>
#include <asm/io.h>
#include <mach/map.h>
#include <linux/platform_device.h>
#include <mach/regs-gpio.h>

#define S3C6410_SERIAL_NAME	"ttySAC"
#define S3C6410_SERIAL_MAJOR 204
#define S3C6410_SERIAL_MINOR 64

#define S3C6410_SERIAL_NUM	4

#define MAP_SIZE (0x100)

#define S3C6410_GPACON	(S3C64XX_GPA_BASE + 0x00)
#define S3C6410_GPBCON	(S3C64XX_GPB_BASE + 0x00)
static void __iomem *gpacon;
static void __iomem *gpbcon;

#define S3C6410_UART_START	(0x7F005000)
static void __iomem *uart_membase[4];
static int uart_index = 0;

static int s3c6410_serial_suspend(struct device *dev)
{
	return 0;
}

static int s3c6410_serial_resume(struct device *dev)
{
	return 0;
}

static int s3c6410_serial_resume_noirq(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops s3c6410_serial_pm_ops = {
	.suspend = s3c6410_serial_suspend,
	.resume = s3c6410_serial_resume,
	.resume_noirq = s3c6410_serial_resume_noirq,
};

#define S3C6410_SERIAL_PM_OPS (&s3c6410_serial_pm_ops)

static void uart_putc(struct uart_port *port, unsigned char c)
{
	while( __raw_readl(uart_membase[port->line] + 0x18) & (1 << 14) );
	__raw_writel(c, uart_membase[port->line] + 0x20);
}

static void	s3c6410_serial_start_tx(struct uart_port *port)
{
	struct circ_buf *xmit = &port->state->xmit;

	while(1) {
		uart_putc(port, xmit->buf[xmit->tail]);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);

		port->icount.tx++;

		if (uart_circ_empty(xmit))
			break;
	}
}

static irqreturn_t uart_rx_handler(int irq, void *dev_id)
{
	struct uart_port *port = (struct uart_port *)dev_id;
	unsigned int pend = __raw_readl(uart_membase[port->line] + 0x30);
	unsigned int count;
	struct circ_buf *xmit = &port->state->xmit;

	if (pend & 1)
		__raw_writel(0x1, uart_membase[port->line] + 0x30); //清除中断
	if (pend & (1 << 2)) {
		count = port->fifosize;
		while (!uart_circ_empty(xmit) && count-- > 0) {
			if( __raw_readl(uart_membase[port->line] + 0x18) & (1 << 14) )
				break;

			__raw_writel(xmit->buf[xmit->tail], uart_membase[port->line] + 0x20);
			xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
			port->icount.tx++;
		}
		__raw_writel(1 << 2, uart_membase[port->line] + 0x30); //清除中断
	}

	return IRQ_HANDLED;
}

static int s3c6410_serial_startup(struct uart_port *port)
{
	int ret = 0;
	
	__raw_writel(0xf, uart_membase[port->line] + 0x38); //关闭中断

	if ((ret = request_irq(port->irq, uart_rx_handler, IRQF_DISABLED,
				"s3c6400-uart", (void *)port))) {
		return ret;
	}

	__raw_writel(0x03, uart_membase[port->line] + 0x0);
	__raw_writel(0x05, uart_membase[port->line] + 0x4);
	__raw_writel(0x01, uart_membase[port->line] + 0x8);
	if (port->line < 2)
		__raw_writel(0x00, uart_membase[port->line] + 0x0c);
	__raw_writel(35, uart_membase[port->line] + 0x28);
	__raw_writel(0x01, uart_membase[port->line] + 0x2C);

	__clear_bit(0, uart_membase[port->line] + 0x38); //开启接收中断
	__clear_bit(2, uart_membase[port->line] + 0x38); //开启发送中断

	return ret;
}

static void	s3c6410_serial_shutdown(struct uart_port *port)
{
	free_irq(port->irq, port);
}

static const char *s3c6410_serial_type(struct uart_port *port)
{
	return port->type == PORT_S3C6400 ? "S3C6410_UART" : NULL;
}

static void	s3c6410_serial_release_port(struct uart_port *port)
{
	release_mem_region(port->mapbase, MAP_SIZE);
}

static int s3c6410_serial_request_port(struct uart_port *port)
{
	return request_mem_region(port->mapbase, MAP_SIZE, "s3c6400-uart") ? 0 : -EBUSY;
}

static void	s3c6410_serial_config_port(struct uart_port *port, int flags)
{
	if (flags & UART_CONFIG_TYPE &&
	    s3c6410_serial_request_port(port) == 0)
		port->type = PORT_S3C6400;
}

static void s3c6410_serial_set_termios(struct uart_port *port,
				       struct ktermios *termios,
				       struct ktermios *old)
{

}

static void s3c6410_serial_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
}

#ifdef CONFIG_CONSOLE_POLL
static void	s3c6410_serial_poll_put_char(struct uart_port *port, unsigned char c)
{

}

static int s3c6410_serial_poll_get_char(struct uart_port *port)
{

}
#endif

static struct uart_ops s3c6410_serial_ops = {
	.set_mctrl	= s3c6410_serial_set_mctrl,
	.start_tx	= s3c6410_serial_start_tx,
	.startup	= s3c6410_serial_startup,
	.shutdown	= s3c6410_serial_shutdown,
	.type		= s3c6410_serial_type,
	.release_port	= s3c6410_serial_release_port,
	.request_port	= s3c6410_serial_request_port,
	.config_port	= s3c6410_serial_config_port,
	.set_termios	= s3c6410_serial_set_termios,
#if defined(CONFIG_SERIAL_SAMSUNG_CONSOLE) && defined(CONFIG_CONSOLE_POLL)
	.poll_get_char = s3c6410_serial_get_poll_char,
	.poll_put_char = s3c6410_serial_put_poll_char,
#endif
};

#define __PORT_LOCK_UNLOCKED(i) \
	__SPIN_LOCK_UNLOCKED(s3c6410_uart_port[i].lock)
static struct uart_port s3c6410_uart_port[S3C6410_SERIAL_NUM] = {
	[0] = {
		.lock		= __PORT_LOCK_UNLOCKED(0),
		.mapbase	= (unsigned int)S3C_PA_UART0,
		.iotype		= UPIO_MEM,
		.irq		= IRQ_UART0,
		.uartclk	= 0,
		.fifosize	= 32,
		.ops		= &s3c6410_serial_ops,
		.flags		= UPF_BOOT_AUTOCONF,
		.line		= 0,
	},
	[1] = {
		.lock		= __PORT_LOCK_UNLOCKED(1),
		.mapbase	= (unsigned int)S3C_PA_UART1,
		.iotype		= UPIO_MEM,
		.irq		= IRQ_UART1,
		.uartclk	= 0,
		.fifosize	= 32,
		.ops		= &s3c6410_serial_ops,
		.flags		= UPF_BOOT_AUTOCONF,
		.line		= 1,
	},
	[2] = {
		.lock		= __PORT_LOCK_UNLOCKED(2),
		.mapbase	= (unsigned int)S3C_PA_UART2,
		.iotype		= UPIO_MEM,
		.irq		= IRQ_UART2,
		.uartclk	= 0,
		.fifosize	= 32,
		.ops		= &s3c6410_serial_ops,
		.flags		= UPF_BOOT_AUTOCONF,
		.line		= 2,
	},
	[3] = {
		.lock		= __PORT_LOCK_UNLOCKED(3),
		.mapbase	= (unsigned int)S3C_PA_UART3,
		.iotype		= UPIO_MEM,
		.irq		= IRQ_UART3,
		.uartclk	= 0,
		.fifosize	= 32,
		.ops		= &s3c6410_serial_ops,
		.flags		= UPF_BOOT_AUTOCONF,
		.line		= 3,
	}
};

#ifdef CONFIG_JL_UART_CONSOLE
static struct console s3c6410_console;
#endif
#define S3C6410_SERIAL_CONSOLE (&s3c6410_console)

static struct uart_driver s3c6410_uart_driver = {
	.owner = THIS_MODULE,
	.driver_name = "s3c6400-uart",
	.dev_name = S3C6410_SERIAL_NAME,
	.major = S3C6410_SERIAL_MAJOR,
	.minor = S3C6410_SERIAL_MINOR,
	.nr = S3C6410_SERIAL_NUM,
	.cons = S3C6410_SERIAL_CONSOLE,
};

/* 4个串口，每一个会调用一次，调用4次 */
static int s3c6410_serial_probe(struct platform_device *pdev)
{
	int ret = 0;

	/* 配置引脚为uart模式,这里会调用4次，暂时先这样写 */
	gpacon = ioremap((phys_addr_t)S3C6410_GPACON, 8);
	gpbcon = ioremap((phys_addr_t)S3C6410_GPBCON, 8);
	writel(readl(gpacon) | (2 << 0) | (2 << 4 ) | (2 << 16) | (2 << 20), gpacon);
	writel(readl(gpbcon) | (2 << 0) | (2 << 4 ) | (2 << 8) | (2 << 12), gpbcon);

	uart_membase[uart_index] = devm_ioremap(&pdev->dev, \
									S3C6410_UART_START + uart_index * 0x400, SZ_64);

	if (!uart_index) {
		ret = uart_register_driver(&s3c6410_uart_driver); /* 注册uart驱动 */
		if (ret < 0) {
			pr_err("Failed to register s3c6410 UART driver\n");
			return ret;
		}
	}

	uart_index++;

	uart_add_one_port(&s3c6410_uart_driver, &s3c6410_uart_port[pdev->id]);
	platform_set_drvdata(pdev, &s3c6410_uart_port[pdev->id]);

	return ret;
}

static int s3c6410_serial_remove(struct platform_device *pdev)
{
	platform_set_drvdata(pdev, NULL);

	uart_remove_one_port(&s3c6410_uart_driver, &s3c6410_uart_port[pdev->id]);

	uart_unregister_driver(&s3c6410_uart_driver);

	iounmap(gpacon);
	iounmap(gpbcon);

	return 0;
}

/* 定义platform_driver */
static struct platform_driver s3c6410_serial_driver = {
	.probe = s3c6410_serial_probe,
	.remove = s3c6410_serial_remove,
	.driver = {
		.name = "s3c6400-uart",
		.pm = S3C6410_SERIAL_PM_OPS,
	}
};

/* 该宏声明驱动入口和出口，并注册platform驱动 */
module_platform_driver(s3c6410_serial_driver);

#ifdef CONFIG_JL_UART_CONSOLE

static struct uart_port *cons_uart;

static void s3c6410_console_putchar(struct uart_port *port, int ch)
{
	while( __raw_readl(uart_membase[port->line] + 0x18) & (1 << 14) );
	__raw_writel(ch, uart_membase[port->line] + 0x20);
}

static void s3c6410_console_write(struct console *co, 
				const char *s, unsigned int count)
{
	uart_console_write(cons_uart, s, count, s3c6410_console_putchar);
}

static void __init s3c6410_uart_get_options(struct uart_port *port, int *baud,
			   int *parity, int *bits)
{
	*parity = 'n';
	*bits = 8;
	*baud = 115200;
}

static int __init s3c6410_console_setup(struct console *co, char *options)
{
	struct uart_port *port;
	int baud = 9600;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	/* is this a valid port */

	if (co->index == -1 || co->index >= S3C6410_SERIAL_NUM)
		co->index = 0;

	gpacon = ioremap((phys_addr_t)S3C6410_GPACON, 8);
	writel(readl(gpacon) | (2 << 0) | (2 << 4 ), gpacon);

	uart_membase[co->index] = ioremap((phys_addr_t)S3C6410_UART_START, SZ_64);
	__raw_writel(0x03, uart_membase[co->index] + 0x0);
	__raw_writel(0x05, uart_membase[co->index] + 0x4);
	__raw_writel(0x01, uart_membase[co->index] + 0x8);
	__raw_writel(0x00, uart_membase[co->index] + 0x0c);
	__raw_writel(35, uart_membase[co->index] + 0x28);
	__raw_writel(0x01, uart_membase[co->index] + 0x2C);

	port = &s3c6410_uart_port[co->index];

	/* is the port configured? */

	if (port->mapbase == 0x0)
		return -ENODEV;

	cons_uart = port;

	/*
	 * Check whether an invalid uart number has been specified, and
	 * if so, search for the first available port that does have
	 * console support.
	 */
	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);
	else
		s3c6410_uart_get_options(port, &baud, &parity, &bits);

	return uart_set_options(port, co, baud, parity, bits, flow);
}
static struct console s3c6410_console = {
	.name		= "ttySAC",
	.device		= uart_console_device,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,
	.write		= s3c6410_console_write,
	.setup		= s3c6410_console_setup,
	.data		= &s3c6410_uart_driver,
};

static int __init uart_console_init(void)
{
	register_console(&s3c6410_console);
	return 0;
}

console_initcall(uart_console_init);
#endif

MODULE_LICENSE("GPL v2");
