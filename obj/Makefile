# Define el nombre de tu módulo
MODULE_NAME = mydriver
obj-m += $(MODULE_NAME).o

# Variables de entorno para la ruta
KDIR ?= /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)
OBJ_DIR := $(PWD)/obj

all:
	# 1. Crea la carpeta obj si no existe
	mkdir -p $(OBJ_DIR)
	# 2. Copia el código fuente y este mismo Makefile a la carpeta obj
	cp *.c $(OBJ_DIR)/
	cp Makefile $(OBJ_DIR)/
	# 3. Ejecuta la compilación apuntando M= al directorio obj
	make -C $(KDIR) M=$(OBJ_DIR) modules

clean:
	# Limpia los archivos generados por Kbuild en obj/
	make -C $(KDIR) M=$(OBJ_DIR) clean
	# Elimina la carpeta obj por completo
	rm -rf $(OBJ_DIR)