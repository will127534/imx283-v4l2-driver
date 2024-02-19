obj-m += v4l2-cci.o
obj-m += imx283.o

ccflags-y += -I$(PWD)/include

# Enable devm_cci_regmap_init_i2c
ccflags-y += -DCONFIG_V4L2_CCI_I2C_MODULE

#dtbo-y += imx283.dtbo

RELEASE?=$(shell uname -r)

KDIR ?= /lib/modules/$(RELEASE)/build

#targets += $(dtbo-y)    

#always-y := $(dtbo-y)

all: modules

## Find release options
RELEASES:=$(wildcard /lib/modules/*/build)
RELEASES:=$(patsubst /lib/modules/%/build,%,$(RELEASES))

# Provide make targets for identified releases that can be built against
.PHONY:
$(RELEASES): .PHONY
	$(MAKE) RELEASE=$@ modules

modules clean:
	make -C $(KDIR) M=$(shell pwd) $@

%.dtbo: %.dts
	dtc -@ -I dts -O dtb -o $@ $<

