PWD := $(shell pwd)
obj-m += mpu6050_driver.o

CROSS_ := /hd/linuxlab/bb-kernel/dl/gcc-linaro-6.4.1-2017.11-x86_64_arm-linux-gnueabihf/bin/arm-linux-gnueabihf-
KERNEL_ := /hd/linuxlab/bb-kernel/KERNEL
SUBDIRS_ :=/hd/eworkspace/mpu6050_driver/

all:
	make ARCH=arm CROSS_COMPILE=$(CROSS_) -C $(KERNEL_) SUBDIRS=$(SUBDIRS_) modules
clean:
	make -C $(KERNEL_) SUBDIRS=$(SUBDIRS_) clean

