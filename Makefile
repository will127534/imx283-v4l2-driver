obj-m += imx283.o

#dtbo-y += imx283.dtbo

KDIR ?= /lib/modules/$(shell uname -r)/build

#targets += $(dtbo-y)    

#always-y := $(dtbo-y)

all: modules

modules clean:
	make -C $(KDIR) M=$(shell pwd) $@

%.dtbo: %.dts
	dtc -@ -I dts -O dtb -o $@ $<

