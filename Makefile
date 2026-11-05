# Nombre del módulo
MODULE_NAME := mydriver
obj-m       := $(MODULE_NAME).o

# Rutas del entorno
KDIR        ?= /home/guido/bbb-kernel-headers
SRC_DIR     := $(shell pwd)
OUTPUT_DIR  := $(SRC_DIR)/obj

# Toolchain de compilación cruzada
ARCH          ?= arm
CROSS_COMPILE ?= arm-linux-gnueabihf-

# Datos de la BeagleBone Black
BBB_IP   ?= 192.168.7.2
BBB_USER ?= debian
BBB_DIR  ?= /home/debian

all: prepare
	$(MAKE) -C $(KDIR) M=$(OUTPUT_DIR) src=$(SRC_DIR) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) modules

prepare:
	@mkdir -p $(OUTPUT_DIR)
	@# Kbuild necesita encontrar o enlazar el fuente y el Makefile de construcción dentro del directorio destino
	@ln -sf $(SRC_DIR)/$(MODULE_NAME).c $(OUTPUT_DIR)/$(MODULE_NAME).c
	@echo "obj-m := $(MODULE_NAME).o" > $(OUTPUT_DIR)/Kbuild

clean:
	@if [ -d "$(OUTPUT_DIR)" ]; then \
		$(MAKE) -C $(KDIR) M=$(OUTPUT_DIR) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) clean; \
		rm -rf $(OUTPUT_DIR); \
	fi
	@rm -f $(SRC_DIR)/*.o $(SRC_DIR)/*.ko $(SRC_DIR)/*.mod* $(SRC_DIR)/*.symvers $(SRC_DIR)/*.order $(SRC_DIR)/.*.cmd

deploy: all
	@echo "Enviando $(OUTPUT_DIR)/$(MODULE_NAME).ko a $(BBB_USER)@$(BBB_IP):$(BBB_DIR)..."
	scp $(OUTPUT_DIR)/$(MODULE_NAME).ko $(BBB_USER)@$(BBB_IP):$(BBB_DIR)/
	@echo "Despliegue finalizado con éxito."

.PHONY: all prepare clean deploy
