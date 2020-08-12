/*
 * lpm driver
 *
 */ 

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <asm/atomic.h>
#include <linux/init.h>
#include <linux/gpio.h>

#include <linux/types.h>
#include <linux/errno.h>
#include <linux/netlink.h>
#include <linux/socket.h>

#include <linux/skbuff.h>
#include <net/sock.h>

#define QL_LPM_NETLINK	28
#define MAX_MSGSIZE	24

static int wakeup_in_irq = 0;
static struct sock *nl_sk = NULL;
static int user_pid = -1;
extern struct net init_net;

/*The below number is gpio number on Qcom BaseBand.*/
static int wakeup_in = 75;
module_param(wakeup_in, int, S_IRUGO);
MODULE_PARM_DESC(wakeup_in, "A pin(input) use to wakeup 4G module.");

/*The below number is gpio number on Qcom BaseBand.*/
static int wakeup_out = 24;
module_param(wakeup_out, int, S_IRUGO);
MODULE_PARM_DESC(wakeup_out, "A pin(output) use to wakeup MCU.");

/*The below number, 1 means the wakeupout pin will output high when wakeup module.*/
static int wakeup_out_edge = 1;
module_param(wakeup_out_edge, int, S_IRUGO);
MODULE_PARM_DESC(wakeup_out_edge, "The edge of wakeup_out pin(output) to wakeup mcu, rising by default");

#if 0	/*No need now, maybe useful in the future.*/
static int wakeup_in_edge = 1;
module_param(wakeup_in_edge, int, S_IRUGO);
MODULE_PARM_DESC(wakeup_in_edge, "The edge of wakeup_in pin(input) to wakeup 4G module, rising by default");
#endif

/* 向用户态发送消息 */
int lpm_send_state(char* msg, int len)
{
    struct sk_buff *skb;
    struct nlmsghdr *nlh;

    int ret;

    skb = nlmsg_new(len, GFP_ATOMIC);
    if(!skb)
    {
        pr_err("[ql_lpm]:netlink alloc failure\n");
        return -1;
    }

    nlh = nlmsg_put(skb, 0, 0, QL_LPM_NETLINK, len, 0);
    if(nlh == NULL)
    {
        pr_err("[ql_lpm]:nlmsg_put failaure \n");
        nlmsg_free(skb);
        return -1;
    }
 
    memcpy(nlmsg_data(nlh), msg, len);
    if(user_pid == -1)	//此处应该在netlink_rcv_msg中重新赋值了，不应该是-1
	return -1;
    ret = netlink_unicast(nl_sk, skb, user_pid, MSG_DONTWAIT);	//单播发送
    pr_debug("[ql_lpm]: Send wakeupin state to user space, ret: %d\n", ret);

    return ret;
}

static irqreturn_t quectel_wakeup_irq_func(int irq, void *id)
{
	//TODO: debounce
	int value = gpio_get_value(wakeup_in);

	if(value == 0)
		lpm_send_state("falling", strlen("falling"));
	else if(value == 1)
		lpm_send_state("rising", strlen("rising"));

	return IRQ_HANDLED;
}

static int wakeup_in_init(void)
{
	int err = 0;

	if(wakeup_in == -1)
	{
		pr_err("[ql_lpm][%s]: forgot to assign wakeup_out pin when insmod this kmod\n", __FUNCTION__);
		return -1;
	}
	
	err = gpio_request(wakeup_in, "wakeup_in");	//使用gpio之前必须先申请
	if (err < 0)
	{
		pr_err("[ql_lpm][%s]: request wakeup_in: %d failed, error: %d\n", __FUNCTION__, wakeup_in, err);
		goto err_gpio_request;
	}
	
	err = gpio_direction_input(wakeup_in);	//设置申请的gpio为输入功能
	if (err < 0)
	{
		pr_err("[ql_lpm][%s]: set wakeup_in:  direction input (%d) failed: %d\n", __FUNCTION__, wakeup_in, err);
		goto err_gpio_to_irq;
	}

	err = gpio_to_irq(wakeup_in);	//获取gpio中断编号，该步骤要求gpio为输入功能
	if (err < 0)
	{
		pr_err("[ql_lpm][%s]: wakeup_in: %d to irq failed, err: %d\n", __FUNCTION__, wakeup_in, err);
		goto err_gpio_to_irq;
	}
	
	wakeup_in_irq = err;	//将前面返回的中断编号付给wakeup_in_irq
    
	/* 申请中断，request_any_context_irq()会根据GPIO控制器本身的"上级"中断是否为threaded_irq
	 * 来决定采用request_irq()还是request_threaded_irq()，虽然每个gpio都有独立的中断号，但是
	 * 在硬件上，一组gpio通常是嵌套在上一级中断控制器上的一个中断 */
	/* 中断处理函数quectel_wakeup_irq_func */
	err = request_any_context_irq(wakeup_in_irq, quectel_wakeup_irq_func,	\
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "wakeup_in_irq", NULL);
	if (err < 0)
	{
		pr_err("[ql_lpm][%s]: Can't request %d IRQ for wakeup_in: %d\n", __FUNCTION__, wakeup_in_irq, err);
		goto err_free_irq;
	}

	//wake_lock_init(&wakelock, WAKE_LOCK_SUSPEND, "quectel_wakelock");

	return 0;

err_free_irq:
	free_irq(wakeup_in_irq, NULL);	//释放中断
err_gpio_to_irq:
	gpio_free(wakeup_in);	//释放gpio
err_gpio_request:
	return err;
}

static void wakeup_in_exit(void)
{	
	free_irq(wakeup_in_irq, NULL);	//释放中断
	disable_irq(wakeup_in_irq);	//屏蔽中断

	gpio_free(wakeup_in);	//释放gpio
}

/* netlink接收回调,接收用户态发来的消息 */
static void netlink_rcv_msg(struct sk_buff *__skb)
{

     struct sk_buff *skb;
     struct nlmsghdr *nlh;
     pr_debug("[ql_lpm]: user data come in\n");
     skb = skb_get (__skb);
     if(skb->len >= NLMSG_SPACE(0))
     {
         nlh = nlmsg_hdr(skb);
         user_pid = nlh->nlmsg_pid;	//保存用户态pid，这里是端口
         pr_info("[ql_lpm]: user ID:%d, messages: %s\n",user_pid, (char *)NLMSG_DATA(nlh));
         kfree_skb(skb);
    }
}

static int quectel_low_consume_suspend(struct platform_device *pdev)
{
	pr_debug("[ql_lpm][%s]\n", __FUNCTION__);

	/* enable wakeup_in wakeup function */
	if (enable_irq_wake(wakeup_in_irq) < 0)	//使能wakeup_in_irq将系统从低功耗唤醒的功能
	{
		pr_err("[ql_lpm][%s]: enable wakeup_in wakeup function failed\n", __FUNCTION__);
		return -1;
	}
	
	/* Set wakeup_out to output level, 4G module enter sleep mode, and notify mcu */
	if(wakeup_out_edge == 1)	//高电平唤醒4G模块，所以这里输出低电平以使4G模块进入睡眠状态
	{
		pr_info("[ql_lpm][%s]: output low\n", __FUNCTION__);
		gpio_direction_output(wakeup_out, 0);	//输出低电平
	}
	else if(wakeup_out_edge == 0)	//低电平唤醒4G模块，所以这里输出高电平以使4G模块进入睡眠状态
	{
		pr_info("[ql_lpm][%s]: output high\n", __FUNCTION__);
		gpio_direction_output(wakeup_out, 1); //输出高电平
	}
	
	return 0;
}

static int quectel_low_consume_resume(struct platform_device *pdev)
{
	pr_debug("[ql_lpm][%s]\n", __FUNCTION__);

	/* disable wakeup_in wakeup function */
	if (disable_irq_wake(wakeup_in_irq) < 0)	/* 屏蔽wakeup_in_irq唤醒系统的功能 */
	{
		pr_err("[ql_lpm][%s]: disable wakeup_in wakeup function failed\n", __FUNCTION__);
		return -1;
	}
	
	/* Set wakeup_out to output high level, 4G module enter active mode, and notify mcu */
	//gpio_direction_output(wakeup_out, 1);	//Here, I think should be control in user application.

	return 0;
}

struct netlink_kernel_cfg cfg = { 
	.input  = netlink_rcv_msg, /* set recv callback */
};  

static int quectel_low_consume_probe(struct platform_device *pdev)
{
		int ret = 0;
		if((ret = wakeup_in_init()) < 0)	//wakeup_in gpio初始化，配置为边沿触发中断
		{
			pr_err("[ql_lpm][%s]: wakeup_in init failed\n", __FUNCTION__);
			goto err_wakeup_in_exit;
		}

		nl_sk = netlink_kernel_create(&init_net, QL_LPM_NETLINK, &cfg);	//创建netlink socket，用于内核空间与用户空间通信
                if(!nl_sk){
                    pr_err("[ql_lpm]netlink: create netlink socket error.\n");
                    return -1;
                }
                pr_debug("[ql_lpm]: create netlink socket ok.\n");
		pr_debug("[ql_lpm]: wakeup_in pin: %d, wakeup_out pin: %d\n", wakeup_in, wakeup_out);
	
		pr_info("[ql_lpm][%s]: module probe successfully\n", __FUNCTION__);
		return 0;

err_wakeup_in_exit:
	wakeup_in_exit();

	return ret;
}

static int quectel_low_consume_remove(struct platform_device *pdev)
{
	wakeup_in_exit();

        if(nl_sk != NULL){
            sock_release(nl_sk->sk_socket);	//释放socket
        }
	pr_info("[ql_lpm][%s]: module exit.\n", __FUNCTION__);

	return 0;
}

static const struct of_device_id quectel_low_consume_match[] =
{
	{.compatible = "quec,ql_lpm",},
	{},
};

static struct platform_driver quectel_low_consume_driver = {
	.probe		= quectel_low_consume_probe,
	.remove		= quectel_low_consume_remove,
	.suspend	= quectel_low_consume_suspend,
	.resume		= quectel_low_consume_resume,
	.driver		= {
		.name = "ql_lpm",
		.owner = THIS_MODULE,
		.of_match_table = quectel_low_consume_match,
	},
};

module_platform_driver(quectel_low_consume_driver);

MODULE_LICENSE("GPL v2");
