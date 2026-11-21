MODULE_NAME := td3_i2s

ARCH ?= arm
CROSS_COMPILE ?= arm-linux-gnueabihf-
KDIR ?= $(HOME)/bbb-kernel-headers
PWD := $(shell pwd)
OUTPUT_DIR ?= $(PWD)/obj

# Le indica al compilador que busque headers en la carpeta donde está el código fuente real
ccflags-y += -I$(PWD)

obj-m := $(MODULE_NAME).o
$(MODULE_NAME)-objs := mydriver.o utils.o

all:
	@mkdir -p $(OUTPUT_DIR)
	# Si copias o enlazas el .c a obj, asegurate de compilar pasando el include path:
	$(MAKE) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KDIR) M=$(PWD) modules
	@mv -f *.o *.ko *.mod* .*.cmd modules.order Module.symvers $(OUTPUT_DIR)/ 2>/dev/null || true

clean:
	$(MAKE) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KDIR) M=$(PWD) clean
	rm -rf $(OUTPUT_DIR)

deploy:
	scp $(OUTPUT_DIR)/$(MODULE_NAME).ko debian@192.168.7.2:/home/debian/