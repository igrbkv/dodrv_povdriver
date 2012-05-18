ifneq ($(KERNELRELEASE),)
pov-objs 	:= pov-driver.o
obj-m      	:= pov.o

else
KDIR        := /lib/modules/$(shell uname -r)/build
PWD         := $(shell pwd)

all:
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) modules

install:
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) modules_install
endif


clean:
	-rm *.o *.ko .*.cmd *.mod.c *~
