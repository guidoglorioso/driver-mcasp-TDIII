Para compilar el archivo de device_tree_overlay.dts usar el siguiente comando:

```Shell
dtc -O dtb -o ./dt/device_tree_overlay.dtbo -b 0 -@ ./dt/device_tree_overlay.dts
```

Para copiar en la BBB.

```Shell
scp ./dt/device_tree_overlay.dtbo debian@192.168.7.2:/tmp/
```

Aplicar dentro de la bbb el device tree overlay. Es necesario rebootear.

```Shell
sudo cp /tmp/device_tree_overlay.dtbo /boot/dtbs/$(uname -r)/
```

NOTA: es necesario que este esta linea en el archivo uEnv.txt:

dtb_overlay=/boot/dtbs/device_tree_overlay.dtbo.
