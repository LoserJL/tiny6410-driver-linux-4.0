KERN_DIR := /home/coolloser/tiny6410/kernel/linux-4.0

obj-m := i2c_s3c6410.o

all:
	make -C $(KERN_DIR) M=$(CURDIR) modules

clean:
	make -C $(KERN_DIR) M=$(CURDIR) clean
