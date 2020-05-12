KERNELDIR := /home/coolloser/tiny6410/kernel/linux-4.0/
PWD := $(shell pwd)

obj-m := framebuf.o

all:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules

clean:
	rm -rf *.o *.ko *.mod.c *.mod.o *.order *.symvers
