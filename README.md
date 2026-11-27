# Driver I2S para McASP0 en BeagleBone Black (`td3_i2s`)

## 1. Descripción general del Trabajo Práctico

Este proyecto implementa un **driver de Linux en espacio de kernel** (`td3_i2s.ko`) para el controlador **McASP0** (*Multichannel Audio Serial Port*) del SoC **TI AM335x** en modo **I2S full-duplex**, operando mediante acceso directo a memoria del periférico (sin utilizar el subsistema ALSA).

### Parámetros de funcionamiento I2S
- **Formato**: 2 canales/slots I2S, palabras de **32 bits**, **MSB primero** y retardo de **1 ciclo** respecto al Frame Sync.
- **Sincronismo**: transmisión y recepción síncronas compartiendo bit clock y frame sync, TX en flanco ascendente y RX en flanco descendente.
- **Estado inicial**: arranca por defecto en modo **SLAVE** (líneas de reloj y frame sync en alta impedancia como entradas) por seguridad eléctrica entre placas.
- **Gestión de energía y bloqueo**: manejo de reloj bajo demanda (`clk_prepare_enable` / `clk_disable_unprepare`), E/S dirigida por interrupciones y esperas pasivas bloqueantes en `TASK_INTERRUPTIBLE`.

### Uso de AFIFOs de Hardware
Para alcanzar frecuencias elevadas sin pérdida de tramas, el driver utiliza las **AFIFOs** (*Audio FIFOs*) integradas en el McASP0 junto con su puerto de datos:

- **Problema observado sin AFIFO**: operando directamente contra los buffers simples de transmisión/recepción, el hardware dispone únicamente de **1 palabra (32 bits)** de margen. A `1 Mbps`, cada palabra debe ser atendida en `32 µs`; ante latencias normales del kernel Linux en un único núcleo Cortex-A8 (atención de otras IRQs, consola serie o planificación de tareas), se producían *underruns* en TX y *overruns* en RX, limitando la frecuencia máxima estable a **100 kbps** (`100 kHz`).
- **Solución con AFIFO**: habilitando las FIFOs de hardware de TX y RX de **64 palabras de 32 bits (256 bytes)** cada una, el periférico transfiere automáticamente entre las FIFOs y los serializadores, mientras las rutinas de interrupción vacían o rellenan por lotes según el nivel ocupado. A `1 Mbps` esto otorga **`2.048 ms` de margen** frente a latencias del sistema operativo, permitiendo operar de forma estable a **1 Mbps**.

---

## 2. Entorno de desarrollo y sistema objetivo

- **Host de compilación**: PC con Linux / **WSL2 (Ubuntu)** utilizando el compilador cruzado **`arm-linux-gnueabihf-gcc`** y los headers del kernel de la placa ubicados en `~/bbb-kernel-headers`.
- **Placa destino**: **BeagleBone Black (BBB)** corriendo Debian con kernel **`6.12.100-bone70`**.
- **Conexión PC $\leftrightarrow$ BBB**: interfaz Ethernet sobre USB en `192.168.7.2` (usuario `debian`).

---

## 3. Estructura del repositorio y uso del `Makefile`

```text
Driver-BBB/
├── Makefile                 # Compilación cruzada del módulo, limpieza y deploy por scp
├── README.md                # Documentación principal de uso, entorno y pruebas
├── docs.md                  # Notas técnicas de hardware, pines y registros del McASP0
├── i2s_driver.c / i2s_driver.h  # Implementación principal del driver (platform, cdev, sysfs, IRQ, McASP/AFIFO)
├── utils.c / utils.h        # Buffer circular thread/IRQ-safe y helper de espera de registros (mcasp_wait_reg)
├── dt/                      # Device Tree del overlay y dump del DT de la BBB
│   ├── device_tree_overlay.dts
│   ├── device_tree_overlay.dtbo
│   └── device_tree_bbb.dts
├── obj/                     # Artefactos de compilación generados (.ko, .o, Module.symvers, etc.)
├── test/                    # Herramientas de prueba
│   ├── loopback.c           # Diagnóstico loopback completo (/dev y /sys) en una sola placa
│   └── i2s_tester           # Binario de validación caja negra
└── docs/                    # Bibliografía, hojas de datos y consigna del TP
```

### Comandos del `Makefile`

Las variables por defecto en [Makefile](file:////wsl.localhost/Ubuntu/home/guido/TDIII/Driver-BBB/Makefile) son `ARCH ?= arm`, `CROSS_COMPILE ?= arm-linux-gnueabihf-`, `KDIR ?= $(HOME)/bbb-kernel-headers` y `OUTPUT_DIR ?= $(PWD)/obj`.

- **Compilar el módulo (`td3_i2s.ko`)**:
  ```bash
  make
  ```
  Compila `i2s_driver.o` y `utils.o`, genera `td3_i2s.ko` contra `KDIR` y mueve todos los archivos intermedios y el `.ko` dentro de `obj/`.
- **Limpiar archivos generados**:
  ```bash
  make clean
  ```
  Ejecuta `clean` de kbuild y elimina la carpeta `obj/`.
- **Enviar el `.ko` a la BeagleBone Black**:
  ```bash
  make deploy
  ```
  Copia `obj/td3_i2s.ko` a `debian@192.168.7.2:/home/debian/` mediante `scp`.

> **Nota sobre bitrate de compilación**: en `i2s_driver.h` se encuentra la macro `#define FREQ_TEST 1000000` (actualmente en `1 Mbps`). Si se desea tomar la frecuencia directamente desde el Device Tree (`utn,frecuencia = <1000000>` en `dt/device_tree_overlay.dts`), basta con comentar dicha macro y recompilar con `make`.

---

## 4. Mapeo de pines y conexiones físicas (McASP0)

En el controlador `pinctrl-single` del kernel `6.12.100-bone70`, el receptor del pad (`0x20` = *RX Active*) debe permanecer habilitado en `ACLKX` y `AFSX` incluso en modo `MASTER`, ya que el McASP realimenta su propio reloj desde el pin físico.

| Función I2S | Pin Físico BBB | Señal AM335x | Offset Control Module | Config DT (`<offset conf mux>`) | Bit en `PFUNC` / `PDIR` | Dirección en Master | Dirección en Slave |
| :--- | :---: | :--- | :---: | :---: | :---: | :---: | :---: |
| **BCLK (CLK)** | **P9_31** | `mcasp0_aclkx` | `0x190` (`0x44e10990`) | `0x190 0x28 0x00` (Mode 0) | Bit `26` | Salida (`1`) | Entrada (`0`) |
| **WS / FS** | **P9_29** | `mcasp0_fsx` | `0x194` (`0x44e10994`) | `0x194 0x28 0x00` (Mode 0) | Bit `28` | Salida (`1`) | Entrada (`0`) |
| **DATA_TX** | **P9_30** | `mcasp0_axr0` | `0x198` (`0x44e10998`) | `0x198 0x08 0x00` (Mode 0) | Bit `0` | Salida (`1`) | Salida (`1`) |
| **DATA_RX** | **P9_41** | `mcasp0_axr1` | `0x1a8` (`0x44e109a8`) | `0x1a8 0x28 0x00` (Mode 0) | Bit `1` | Entrada (`0`) | Entrada (`0`) |

### Topologías de conexión
- **Entre dos placas (Master $\leftrightarrow$ Slave)** *(usar resistores serie de 220 $\Omega$ a 330 $\Omega$ para proteger ante colisión de dos Masters)*:
  - `P9_31 (BCLK)` Placa A $\longleftrightarrow$ `P9_31 (BCLK)` Placa B
  - `P9_29 (WS)` Placa A $\longleftrightarrow$ `P9_29 (WS)` Placa B
  - `P9_30 (DATA_TX)` Placa A $\longrightarrow$ `P9_41 (DATA_RX)` Placa B
  - `P9_41 (DATA_RX)` Placa A $\longleftarrow$ `P9_30 (DATA_TX)` Placa B
  - `GND` Placa A $\longleftrightarrow$ `GND` Placa B
- **Modo loopback en una sola placa**: puentear físicamente **`P9_30` (`AXR0`) con `P9_41` (`AXR1`)** (idealmente mediante un resistor serie de 220 $\Omega$–330 $\Omega$).

---

## 5. Compilación e instalación del Device Tree Overlay

En la PC de desarrollo, compilar el overlay con `dtc`:

```bash
dtc -O dtb -o ./dt/device_tree_overlay.dtbo -b 0 -@ ./dt/device_tree_overlay.dts
```

Transferir el `.dtbo` a la BBB:

```bash
scp ./dt/device_tree_overlay.dtbo debian@192.168.7.2:/tmp/
```

Dentro de la BBB, copiarlo al directorio de device trees del kernel en ejecución y reiniciar:

```bash
sudo cp /tmp/device_tree_overlay.dtbo /boot/dtbs/$(uname -r)/
sudo reboot
```

Verificar que `/boot/uEnv.txt` contenga las siguientes líneas habilitadas:

```ini
enable_uboot_overlays=1
dtb_overlay=/boot/dtbs/6.12.100-bone70/device_tree_overlay.dtbo
```

---

## 6. Interfaces y operaciones expuestas por el driver

### A. Dispositivo de caracteres: `/dev/td3_i2s` (permisos `0666`)

| Operación | Función en `i2s_driver.c` | Comportamiento |
| :--- | :--- | :--- |
| **`open()`** | `mymcasp_open` | Apertura exclusiva: retorna `-EBUSY` si el nodo ya está abierto o si sysfs tiene una operación TX/RX en curso. Enciende el reloj e inicializa el periférico y las AFIFOs. |
| **`release()`** | `mymcasp_close` | Si no está saliendo por señal y quedan datos por transmitir, espera hasta 2 s su vaciado; luego detiene el periférico, vacía las FIFOs de hardware, apaga el reloj y libera la exclusividad. |
| **`read()`** | `mymcasp_read` | Lectura bloqueante (`TASK_INTERRUPTIBLE`) con exclusión mutua en recepción (`-EBUSY` si está ocupada). Habilita la interrupción de RX y espera hasta reunir `min(len, BUFF_SIZE)` bytes, manteniéndose activa entre lecturas sucesivas hasta `close()`. Retorna bytes leídos o `-ERESTARTSYS` ante señal. |
| **`write()`** | `mymcasp_write` | Escritura bloqueante (`TASK_INTERRUPTIBLE`) con exclusión mutua en transmisión (`-EBUSY` si está ocupada). Copia los datos del usuario al buffer circular de TX (suspendiéndose si se llena hasta liberar la mitad) y habilita la interrupción de TX para envío continuo por AFIFO. |

### B. Interfaz Sysfs: `/sys/class/td3_i2s/`

| Atributo | Permisos | Función en `i2s_driver.c` | Comportamiento |
| :--- | :---: | :--- | :--- |
| **`master`** | `0644` (R/W) | `master_show` / `master_store` | Lee (`0` o `1`) o configura el modo I2S aceptando `"0"`/`"SLAVE"`/`"ESCLAVO"` o `"1"`/`"MASTER"`/`"MAESTRO"` (`-EINVAL` para otros valores). Retorna `-EBUSY` si `/dev/td3_i2s` está abierto o hay TX/RX activas. |
| **`recibir`** | `0444` (R/O bin) | `recibir_read` | Enciende el hardware bajo demanda, habilita la interrupción de RX, bloquea en `TASK_INTERRUPTIBLE` hasta leer `min(count, BUFF_SIZE)` bytes (offset `0`) y apaga el hardware al finalizar. Retorna `-EBUSY` si `/dev/td3_i2s` u otra lectura están en uso. |
| **`transmitir`** | `0200` (W/O bin) | `transmitir_write` | Enciende el hardware bajo demanda, encola hasta `min(count, BUFF_SIZE)` bytes, habilita la interrupción de TX, espera en `TASK_INTERRUPTIBLE` hasta vaciar el buffer y apaga el hardware al finalizar. Retorna `-EBUSY` si `/dev/td3_i2s` u otra escritura están en uso. |

---

## 7. Transferencia de archivos, carga del módulo y ejecución de tests

### 1. Compilar y transferir archivos desde la PC a la BBB

Desde la raíz del proyecto en la PC/WSL:

```bash
# 1. Compilar el módulo del kernel
make

# 2. (Opcional) Compilar el test de diagnóstico loopback para ARM
arm-linux-gnueabihf-gcc -O2 -Wall ./test/loopback.c -o ./test/loopback

# 3. Transferir módulo y binarios de prueba a la BBB
make deploy
scp ./test/i2s_tester ./test/loopback debian@192.168.7.2:/home/debian/
```

### 2. Conectar por SSH y cargar el módulo en la BBB

```bash
ssh debian@192.168.7.2
```

Ya dentro de la BBB (`/home/debian/`):

```bash
# Cargar el driver (si ya estaba cargado, descargarlo antes con sudo rmmod td3_i2s)
sudo insmod ./td3_i2s.ko

# Verificar carga correcta, nodos creados y logs del kernel
lsmod | grep td3_i2s
ls -l /dev/td3_i2s /sys/class/td3_i2s/
dmesg | tail -n 20
```

Para descargar el módulo luego de las pruebas:

```bash
sudo rmmod td3_i2s
```

### 3. Ejecución del test de caja negra (`i2s_tester`)

Dar permisos de ejecución al binario en la BBB:

```bash
chmod +x ./i2s_tester
```

Sintaxis de invocación:

```bash
./i2s_tester <master|slave> [bitrate_bps]
```

Ejemplos de uso (con `FREQ_TEST = 1000000`, `bitrate_bps = 1000000`):

```bash
# En la placa configurada como MASTER (o en loopback local):
./i2s_tester master 1000000

# En la segunda placa configurada como SLAVE (iniciar antes o en simultáneo):
./i2s_tester slave 1000000
```

### 4. Ejecución del test de diagnóstico (`loopback`)

#### ¿Cómo funciona y para qué sirve?
El programa [test/loopback.c](file:////wsl.localhost/Ubuntu/home/guido/TDIII/Driver-BBB/test/loopback.c) está pensado para depurar la integridad de la trama recibida usando una sola BeagleBone Black (con `P9_30` conectado a `P9_41`):
1. Cambia el driver a modo `MASTER` vía `/sys/class/td3_i2s/master`.
2. Ejecuta **Fase 1 (`/dev/td3_i2s`)**: abre el dispositivo con `O_RDWR`, crea un proceso hijo lector y un padre escritor (`fork()`), y valida tramas pseudoaleatorias/etiquetadas de **8 bytes** (trama corta), **30 bytes** (longitud no múltiplo de 4 bytes) y **32 KB** (desborde del buffer circular interno).
3. Ejecuta **Fase 2 (`/sys/class/td3_i2s/`)**: repite los tres tamaños de prueba usando los atributos binarios `transmitir` y `recibir` en procesos concurrentes.
4. Restaura el modo `SLAVE` al finalizar.

Su utilidad principal de diagnóstico es **localizar automáticamente el desplazamiento inicial por ceros de *idle*** previos al `write()` (`locate_shifted_start`), alinear la secuencia TX dentro de RX y reportar exactamente cuántos bytes se perdieron al inicio, cuántos faltaron al final o cuáles se corrompieron en el medio (mostrando hexdump ASCII en casos cortos y los primeros `[DIFF]` en tramas largas).

#### Comandos para ejecutarlo en la BBB

```bash
chmod +x ./loopback
./loopback
```

*(Si se transfirió `loopback.c` en lugar del binario compilado, puede compilarse directamente en la BBB con `gcc -O2 -Wall loopback.c -o loopback`.)*

---

## 8. Notas de corrección

Durante la ejecución del test de caja negra (`i2s_tester`), en la **Sección 2 (`VALIDAR API, ARGUMENTOS Y ACCESO EXCLUSIVO (-EBUSY)`)** se observan **dos fallos esperados** en las pruebas de exclusividad de `sysfs`:

- `[FAIL] Exclusion en /sys/class/td3_i2s/transmitir -> Se espera EBUSY, obtenido: Device or resource busy`
- `[FAIL] Exclusion en /sys/class/td3_i2s/recibir -> Se espera EBUSY, obtenido: Device or resource busy`

**Motivo técnico**:
El binario `i2s_tester` verifica la exclusividad evaluando que la llamada **`open()`** sobre `/sys/class/td3_i2s/transmitir` y `/sys/class/td3_i2s/recibir` devuelva `-1` con `errno == EBUSY` mientras `/dev/td3_i2s` permanece abierto. Sin embargo, en el kernel Linux la estructura `struct bin_attribute` de `sysfs` (`kernfs`) **no dispone de un callback `.open()`** que pueda implementar o interceptar el driver: la syscall `open()` es resuelta internamente por `kernfs` devolviendo éxito, y el driver recién toma el control en los callbacks `.read()` (`recibir_read`) y `.write()` (`transmitir_write`), donde efectivamente valida el uso exclusivo y retorna `-EBUSY`. El texto `"obtenido: Device or resource busy"` aparece impreso porque `errno` conservó el `EBUSY` de la prueba inmediatamente anterior sobre `/dev/td3_i2s`, pero el descriptor devuelto por `open()` fue válido (`>= 0`) al haber sido manejado íntegramente por el kernel.
