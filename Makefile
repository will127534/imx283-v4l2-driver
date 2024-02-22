obj-m += v4l2-cci.o
obj-m += v4l2-link-freq.o
obj-m += imx283.o

ccflags-y += -I$(PWD)/include

# Enable devm_cci_regmap_init_i2c
ccflags-y += -DCONFIG_V4L2_CCI_I2C_MODULE

#dtbo-y += imx283.dtbo

KDIR ?= /lib/modules/$(shell uname -r)/build

#targets += $(dtbo-y)    

#always-y := $(dtbo-y)

all:
	make -C $(KDIR) M=$(shell pwd) modules

clean:
	make -C $(KDIR)  M=$(shell pwd) clean

%.dtbo: %.dts
	dtc -@ -I dts -O dtb -o $@ $<

