
/* ���豸����i2c�ڵ�������ӽڵ�
 * cw2015: cw2015@62{
		compatible = "cw2015";
		cw2015_charge_gpio = <&gpio1 1 GPIO_ACTIVE_LOW>;
		reg = <0x62>;
	};
*/

/* ��������֮����/sys/class/power_supply/���������һ��
 * cw2015-batteryĿ¼����cw2015-batteryĿ¼�����кܶ��ļ���
 * ���Ƕ�ȡ��Щ�ļ��Ϳ��Ի�ȡ��ص�״̬��
 * ��cat capacity�����100 */

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


/* CW2015��ؼĴ������� */
#define REG_VERSION				0x0
#define REG_VCELL				0x2
#define REG_SOC					0x4
#define REG_RRT_ALERT			0x6
#define REG_CONFIG				0x8
#define REG_MODE				0xA
#define REG_BATINFO				0x10

/* CW2015�ṹ�� */
struct cw2015_battery {
	struct i2c_client *client;		/* i2c client�ṹ�� */
	struct power_supply *psy;		/* ��Դ����ṹ�� */

	int voltage;			/* ��ѹ */
	int charge_status; 		/* ���״̬ */
	int capacity;			/* �����ٷֱ� */
	int charge_gpio;		/* �����gpio */
};

static int cw2015_read_regs(struct cw2015_battery *bat, uint8_t reg, void *val, int len)
{
	int ret;
	struct i2c_msg msg[2];

	/* msg[0]Ϊ���͵�Ҫ��ȡ���׵�ַ */
	msg[0].addr = bat->client->addr;	/* cw2015��ַ */
	msg[0].flag = 0;					/* ���Ϊ�������� */
	msg[0].buf = &reg;					/* Ҫ��ȡ���׵�ַ */
	msg[0].len = 1;						/* reg���� */

	/* msg[1]��ȡ������ */
	msg[1].addr = bat->client->addr;	/* cw2015��ַ */
	msg[1].flag = I2C_M_RD;				/* ���Ϊ��ȡ���� */
	msg[1].bug = val;					/* ��ȡ���ݵĻ����� */
	msg[1].len = len;					/* Ҫ��ȡ���ݳ��� */

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

	b[0] = reg;					/* �Ĵ����׵�ַ */
	memcpy(&b[1], buf, len);	/* ����Ҫд������� */
	
	msg.addr = bat->client->addr;
	msg.flag = 0;
	msg.buf = b;
	msg.len = len + 1;

	return i2c_transfer(bat->client->adapter, &msg, 1);
}

/* ��ȡһ���Ĵ��� */
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

/* ��ȡcw2015���״̬ */
static void cw2015_readdata(struct cw2015_battery *bat)
{
	uint8_t BATVADC_VALUE_low, BATVADC_VALUE_high;
	uint8_t value;

	/* ���� */
	cw2015_write_reg(bat, REG_MOD, 0x00);

	/* ��ѹ�ķֱ�����305uV */
	/* ��ȡ��ѹ */
	BATVADC_VALUE_high = cw2015_read_reg(bat, REG_VCELL);
	BATVADC_VALUE_low = cw2015_read_reg(bat, REG_VCELL+1);

	/* ת��Ϊ14λ��ADCֵ */
	bat->voltage = (BATVADC_VALUE_high << 8) | BATVADC_VALUE_low;

	/* ת��Ϊ��ѹֵ������1000תΪmV����Ӧ�ò�ʹ�� */
	bat->voltage = bat->voltage * 305 / 1000;

	/* ��ȡ���� */
	bat->capacity = cw2015_read_reg(bat, REG_SOC);

	/* ��ȡ���״̬ */
	if (bat->charge_gpio != -1) {
		/* ��ȡ���״̬ */
		value = gpio_get_value(bat->charge_gpio);

		if (!value) /* ��ȡ���͵�ƽ������ */
			bat->charge_status = POWER_SUPPLY_STATUS_CHARGING;
		else
			bat->charge_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	} else {
		/* ������gpioΪ�գ����״̬δ֪ */
		bat->charge_status = POWER_SUPPLY_STATUS_UNKNOWN;
	}
}

/* cw2015��ʼ�� */
static int cw2015_init(struct cw2015_battery *bat)
{
	int ret = 0;
	uint8_t data = 0;

	ret = cw2015_write_regs(bat, REG_MOD, &data, 1);
	if (ret < 0)
		return -1;

	return 0;
}

/* cw2015��Դ�������� */
static enum power_supply_property cw2015_charger_properties[] = {
	/* ��ض����� power_supply.h */
	POWER_SUPPLY_PROP_STATUS,	//���״̬
	POWER_SUPPLY_PROP_CHARGE_TYPE,	//�������
	POWER_SUPPLY_PROP_HEALTH, //��ؽ���״̬
	POWER_SUPPLY_PROP_CAPACITY,	//�����ٷֱ�
	POWER_SUPPLY_PROP_VOLTAGE_NOW,	//��ǰ��ѹ	Ӧ����ʵ��ֵ��1000��
	// POWER_SUPPLY_PROP_VOLTAGE_MAX,	//��ѹ���ֵ ����ֵ
	// POWER_SUPPLY_PROP_VOLTAGE_MIN,	//��ѹ��Сֵ ����ֵ
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,	//��ѹ������ֵ
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,	//��ѹ�����Сֵ
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,	//�������
};

/* cw2015����ֵ��ȡ���� */
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

/* �����Ϣ */
static const struct power_supply_desc cw2015_psy_desc = {
	.name = "cw2015-battery",
	.typr = POWER_SUPPLY_TYPE_BATTERY,	/* ��Դ�����ͣ���� */
	.get_property = cw2015_get_property,	/* ��ȡ��Դ���ԵĽӿں��� */
	.properties = cw2015_charger_properties, /* ��Դ���ԣ���enum�г� */
	.num_properties = ARRAY_SIZE(cw2015_charger_properties); /* ��Դ���Ե����� */
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

	/* ��ȡ�����gpio */
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

	/* ���cw2015�Ƿ���� */
	ret = cw2015_bat_init(cw_bat);
	while ( (loop++ < 200) & (ret != 0) ) {
		msleep(200);
		ret = cw2015_bat_init(cw_bat);
	}
	if (ret) {
		pr_warning("cw2015 is not found\n");
		return ret;
	}

	/* ע��power_supply */
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

/* �豸��ƥ���б� */
static const struct of_device_id cw2015_of_match[] = {
	{.compatible = "cw2015"},
	{}
}; 

/* ��ͳƥ�䷽ʽid�б� */
static const struct i2c_device_id cw2015_id[] = {
	{"cw2015", 0},
	{}
};

/* i2c�����ṹ�� */
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
