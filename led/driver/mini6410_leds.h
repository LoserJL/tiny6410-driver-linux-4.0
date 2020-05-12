#ifndef __MINI6410_LEDS_H
#define __MINI6410_LEDS_H

#include <linux/cdev.h> //struct cdev

#define MINI6410_PA_LEDS	0x7F008800

#define LED_NUM				4
#define MINI6410_LEDS_MAJOR	0

struct led_dev {
	struct cdev cdev;
	unsigned char led_num;
};

#endif

