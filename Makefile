EXTRA_CFLAGS += -Ofast -mfpu=vfp -march=armv6zk -mtune=arm1176jzf-s

ifneq ($(KERNELRELEASE),)
	obj-m := nrf905.o
	nrf905-y := nrf905_driver.o nrf905_spi.o

else

default:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

modules_install:
	$(MAKE) -C $(KDIR) M=$(PWD) INSTALL_MOD_PATH=$(MODULES_TEMP) modules_install

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

endif

