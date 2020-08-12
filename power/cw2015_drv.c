
/* 在设备树的i2c节点下添加子节点
 * cw2015: cw2015@62{
		compatible = "cw2015";
		cw2015_charge_gpio = <&gpio1 1 GPIO_ACTIVE_LOW>;
		reg = <0x62>;
	};
*/

/* 加载驱动之后，在/sys/class/power_supply/下面会生成一个
 * cw2015-battery目录，在cw2015-battery目录下面有很多文件，
 * 我们读取这些文件就可以获取电池的状态：
 * 如cat capacity，输出100 */

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ide.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/of_gpio.h>
#include <linux/semaphore.h>
#include <linux/i2c.h>
#include <linux/timer.h>
#include <asm/mach/map.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/power_supply.h>


/* CW2015相关寄存器定义 */
#define REG_VERSION				0x0
#define REG_VCELL				0x2
#define REG_SOC					0x4
#define REG_RRT_ALERT			0x6
#define REG_CONFIG				0x8
#define REG_MODE				0xA
#define REG_BATINFO				0x10

/* CW2015结构体 */
struct cw2015_battery {
	struct i2c_client *client;		/* i2c client结构体 */
	struct power_supply *psy;		/* 电源管理结构体 */

	int voltage;			/* 电压 */
	int charge_status; 		/* 充电状态 */
	int capacity;			/* 电量百分比 */
	int charge_gpio;		/* 充电检测gpio */
};

static int cw2015_read_regs(struct cw2015_battery *bat, uint8_t reg, void *val, int len)
{
	int ret;
	struct i2c_msg msg[2];

	/* msg[0]为发送的要读取的首地址 */
	msg[0].addr = bat->client->addr;	/* cw2015地址 */
	msg[0].flag = 0;					/* 标记为发送数据 */
	msg[0].buf = &reg;					/* 要读取的首地址 */
	msg[0].len = 1;						/* reg长度 */

	/* msg[1]读取的数据 */
	msg[1].addr = bat->client->addr;	/* cw2015地址 */
	msg[1].flag = I2C_M_RD;				/* 标记为读取数据 */
	msg[1].bug = val;					/* 读取数据的缓冲区 */
	msg[1].len = len;					/* 要读取数据长度 */

	ret = i2c_transfer(bat->client->adapter, msg, 2);
	if (ret == 2) {
		ret = 0;
	} else {
		printk("i2c rd failed=%d reg=%06x len=%d\n", ret, reg, len);
		ret = -EREMOTEIO;
	}

	return ret;
}

static int cw2015_write_regs(struct cw2015_battery *bat, uint8_t reg, uint8_t *buf, int len)
{
	uint8_t b[256];
	struct i2c_msg msg;

	b[0] = reg;					/* 寄存器首地址 */
	memcpy(&b[1], buf, len);	/* 拷贝要写入的数据 */
	
	msg.addr = bat->client->addr;
	msg.flag = 0;
	msg.buf = b;
	msg.len = len + 1;

	return i2c_transfer(bat->client->adapter, &msg, 1);
}

/* 读取一个寄存器 */
static unsigned char cw2015_read_reg(struct cw2015_battery *bat, uint8_t reg)
{
	uint8_t data = 0;

	cw2015_read_regs(bat, reg, &data, 1);

	return data;
}

static void cw2015_write_reg(struct cw2015_battery *bat, uint8_t reg, uint8_t data)
{
	uint8_t buf = data;

	cw2015_write_regs(bat, reg, &buf, 1);
}

/* 读取cw2015充电状态 */
static void cw2015_readdata(struct cw2015_battery *bat)
{
	uint8_t BATVADC_VALUE_low, BATVADC_VALUE_high;
	uint8_t value;

	/* 唤醒 */
	cw2015_write_reg(bat, REG_MOD, 0x00);

	/* 电压的分辨率是305uV */
	/* 读取电压 */
	BATVADC_VALUE_high = cw2015_read_reg(bat, REG_VCELL);
	BATVADC_VALUE_low = cw2015_read_reg(bat, REG_VCELL+1);

	/* 转换为14位的ADC值 */
	bat->voltage = (BATVADC_VALUE_high << 8) | BATVADC_VALUE_low;

	/* 转换为电压值并乘以1000转为mV，供应用层使用 */
	bat->voltage = bat->voltage * 305 / 1000;

	/* 读取电量 */
	bat->capacity = cw2015_read_reg(bat, REG_SOC);

	/* 读取充电状态 */
	if (bat->charge_gpio != -1) {
		/* 读取充电状态 */
		value = gpio_get_value(bat->charge_gpio);

		if (!value) /* 读取到低电平代表充电 */
			bat->charge_status = POWER_SUPPLY_STATUS_CHARGING;
		else
			bat->charge_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	} else {
		/* 充电检测的gpio为空，充电状态未知 */
		bat->charge_status = POWER_SUPPLY_STATUS_UNKNOWN;
	}
}

/* cw2015初始化 */
static int cw2015_init(struct cw2015_battery *bat)
{
	int ret = 0;
	uint8_t data = 0;

	ret = cw2015_write_regs(bat, REG_MOD, &data, 1);
	if (ret < 0)
		return -1;

	return 0;
}

/* cw2015电源属性数组 */
static enum power_supply_property cw2015_charger_properties[] = {
	/* 相关定义在 power_supply.h */
	POWER_SUPPLY_PROP_STATUS,	//充电状态
	POWER_SUPPLY_PROP_CHARGE_TYPE,	//充电类型
	POWER_SUPPLY_PROP_HEALTH, //电池健康状态
	POWER_SUPPLY_PROP_CAPACITY,	//电量百分比
	POWER_SUPPLY_PROP_VOLTAGE_NOW,	//当前电压	应该是实际值的1000倍
	// POWER_SUPPLY_PROP_VOLTAGE_MAX,	//电压最大值 测量值
	// POWER_SUPPLY_PROP_VOLTAGE_MIN,	//电压最小值 测量值
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,	//电压设计最大值
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,	//电压设计最小值
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,	//电池容量
};

/* cw2015属性值获取函数 */
static int cw2015_get_property(struct power_supply *psy,
							enum power_supply_property psp,
							union power_supply_propval *val)
{
	struct cw2015_battery *cw_bat = power_supply_get_drvdata(psy);

	switch(psp) {
	case POWER_SUPPLY_PROP_STATUS:
		cw2015_readdata(cw_bat);
		val->intval = cw_bat->charge_status;
		/*	
			POWER_SUPPLY_STATUS_UNKNOWN
			POWER_SUPPLY_STATUS_CHARGING
			POWER_SUPPLY_STATUS_DISCHARGING
			POWER_SUPPLY_STATUS_NOT_CHARGING
			POWER_SUPPLY_STATUS_FULL
		*/
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = POWER_SUPPLY_CHARGE_TYPE_NONE;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		cw2015_readdata(cw_bat);
		val->intval = cw_bat->capacity;
		break;
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		val->intval = 4000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		cw2015_readdata(cw_bat);
		val->intval = cw_bat->voltage;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = 4300;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = 3300;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/* 电池信息 */
static const struct power_supply_desc cw2015_psy_desc = {
	.name = "cw2015-battery",
	.typr = POWER_SUPPLY_TYPE_BATTERY,	/* 电源的类型：电池 */
	.get_property = cw2015_get_property,	/* 获取电源属性的接口函数 */
	.properties = cw2015_charger_properties, /* 电源属性，用enum列出 */
	.num_properties = ARRAY_SIZE(cw2015_charger_properties); /* 电源属性的数量 */
};

static int cw2015_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	int ret;
	int loop = 0;

	struct cw2015_battery *cw_bat;
	struct power_supply_config psy_cfg = {};

	cw_bat = devm_kzalloc(&client->dev, sizeof(*cw_bat), GFP_KERNEL);
	if (!cw_bat)
		return -ENOMEM;

	i2c_set_clientdata(client, cw_bat);

	cw_bat->client = client;
	cw_bat->voltage = 0;
	cw_bat->charge_status = POWER_SUPPLY_STATUS_UNKNOWN;
	cw_bat->capacity = 0;
	cw_bat->charge_gpio = -1;

	/* 获取充电检测gpio */
	if (of_get_property(client->dev.of_node, "cw2015_charge_gpio", NULL)) {
		cw_bat->charge_gpio = of_get_named_gpio(client->dev.of_node, "cw2015_charge_gpio", 0);
	} else {
		pr_warning("cw2015 gpio get failed\n");
		cw_bat->charge_gpio = -1;
	}

	if (gpio_is_valid(cw_bat->charge_gpio))
		devm_gpio_request_one(&client->dev, cw_bat->charge_gpio, GPIOF_IN, "cw2015-charge");
	else
		cw_bat->charge_gpio = -1;

	psy_cfg.drv_data = cw_bat;

	/* 检测cw2015是否存在 */
	ret = cw2015_bat_init(cw_bat);
	while ( (loop++ < 200) & (ret != 0) ) {
		msleep(200);
		ret = cw2015_bat_init(cw_bat);
	}
	if (ret) {
		pr_warning("cw2015 is not found\n");
		return ret;
	}

	/* 注册power_supply */
	cw_bat->psy = power_supply_register(&client->dev, &cw2015_psy_desc, &psy_cfg);
	if (IS_ERR(cw_bat->psy)) {
		dev_err(&client->dev, "failed: power supply register\n");
		return PTR_ERR(cw_bat->psy);
	}

	pr_info("cw2015 is found\n");

	return 0;
}

static int cw2015_remove(struct i2c_client *client)
{
	struct cw2015_battery *bat = i2c_get_clientdata(client);

	power_supply_unregister(bat->psy);

	return 0;
}

/* 设备树匹配列表 */
static const struct of_device_id cw2015_of_match[] = {
	{.compatible = "cw2015"},
	{}
}; 

/* 传统匹配方式id列表 */
static const struct i2c_device_id cw2015_id[] = {
	{"cw2015", 0},
	{}
};

/* i2c驱动结构体 */
static struct i2c_driver cw2015_driver = {
	.probe = cw2015_probe,
	.remove = cw2015_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "cw2015",
		.of_match_table = cw2015_of_match,
	},
	.id_table = cw2015_id,
};

static int __init cw2015_init(void)
{
	return i2c_add_driver(&cw2015_driver);
}
module_init(cw2015_init);

static void __exit cw2015_exit(void)
{
	i2c_del_driver(&cw2015_driver);
}
module_exit(cw2015_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("cw2015 driver");
