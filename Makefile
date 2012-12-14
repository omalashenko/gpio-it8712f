obj-m := gpio-it8712f.o

KVERSION := $(shell uname -r)
KDIR     ?= /lib/modules/$(KVERSION)/build/

all:
	$(MAKE) -C $(KDIR) M=$(CURDIR) modules

clean:
	$(MAKE) -C $(KDIR) M=$(CURDIR) clean

