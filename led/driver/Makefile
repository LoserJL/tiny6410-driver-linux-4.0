KERN_DIR = /home/coolloser/tiny6410/kernel/linux-4.0/
PWD := $(shell pwd)

obj-m := mini6410_leds.o

all:
	$(MAKE) -C $(KERN_DIR) M=$(PWD) modules

clean:
	rm -rf *.o *.ko *.mod.c *.mod.o *.order *.symvers
