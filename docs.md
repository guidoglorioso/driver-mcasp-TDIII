# Documentación Técnica del Proyecto — Driver I2S (`td3_i2s`)

Este documento registra las definiciones de hardware, mapeo de pines, registros del periférico **McASP0** (AM335x) y detalles críticos descubiertos durante el desarrollo.

---

## 1. Mapeo de Pines (BeagleBone Black - Header P9)

En el controlador `pinctrl-single` del kernel `6.12.100-bone70`, el nodo `pinmux@800` tiene `#pinctrl-cells = <0x02>`, por lo que cada entrada en `pinctrl-single,pins` requiere **3 valores**: `<offset  config  mux_mode>`.

### Flags de Configuración de Pad (`config`)
- **Bit 3 (`0x08`)**: `PULL_DISABLE` (`1` = Pull-up/down deshabilitado, `0` = habilitado).
- **Bit 4 (`0x10`)**: `PULL_UP` (`1` = Pull-up, `0` = Pull-down).
- **Bit 5 (`0x20`)**: `INPUT_EN` / `RX Active` (`1` = receptor del pad habilitado / entrada activa, `0` = solo salida).
  - *Importante*: En los pines de clock (`ACLKX`) y frame sync (`AFSX`) el receptor del pad (`0x20`) **debe estar siempre activo (`0x28`)**, tanto en modo Slave (porque reciben señal externa) como en modo Master (porque el McASP realimenta su propio clock desde el pad físico).

### Tabla de Pines McASP0

| Función I2S | Pin Físico BBB | Señal AM335x | Offset Control Module | Config DT (`<offset conf mux>`) | Bit en `PFUNC` / `PDIR` | Dirección en Master | Dirección en Slave |
| :--- | :---: | :--- | :---: | :---: | :---: | :---: | :---: |
| **BCLK (CLK)** | **P9_31** | `mcasp0_aclkx` | `0x190` (`0x44e10990`) | `0x190 0x28 0x00` (Mode 0) | Bit `26` | Salida (`1`) | Entrada (`0`) |
| **WS / FS** | **P9_29** | `mcasp0_fsx` | `0x194` (`0x44e10994`) | `0x194 0x28 0x00` (Mode 0) | Bit `28` | Salida (`1`) | Entrada (`0`) |
| **DATA_TX** | **P9_30** | `mcasp0_axr0` | `0x198` (`0x44e10998`) | `0x198 0x08 0x00` (Mode 0) | Bit `0` | Salida (`1`) | Salida (`1`) |
| **DATA_RX** | **P9_41** | `mcasp0_axr1` | `0x1a8` (`0x44e109a8`) | `0x1a8 0x28 0x00` (Mode 0) | Bit `1` | Entrada (`0`) | Entrada (`0`) |

### Conexión Física entre las Dos Placas (Topología Punto a Punto Full-Duplex)
> **Protección eléctrica obligatoria**: Colocar resistores limitadores en serie de **220 Ω a 330 Ω** en cada línea compartida (`BCLK`, `WS`, `DATA_TX`, `DATA_RX`) para evitar daños en caso de colisión de dos Masters.

- `P9_31 (BCLK)` Placa A $\longleftrightarrow$ `P9_31 (BCLK)` Placa B
- `P9_29 (WS)` Placa A $\longleftrightarrow$ `P9_29 (WS)` Placa B
- `P9_30 (DATA_TX - AXR0)` Placa A $\longrightarrow$ `P9_41 (DATA_RX - AXR1)` Placa B
- `P9_41 (DATA_RX - AXR1)` Placa A $\longleftarrow$ `P9_30 (DATA_TX - AXR0)` Placa B
- `GND` Placa A $\longleftrightarrow$ `GND` Placa B

---

## 2. Registros Clave del McASP0 (Base `0x48038000`)

| Registro | Offset | Descripción y Bits Relevantes |
| :--- | :---: | :--- |
| **`MCASP_PFUNC_REG`** | `0x10` | Selección de función del pin: `0` = McASP, `1` = GPIO. Bits: `28` (`AFSX`), `26` (`ACLKX`), `0..3` (`AXR0..AXR3`). |
| **`MCASP_PDIR_REG`** | `0x14` | Dirección del pin: `0` = Entrada (alta impedancia), `1` = Salida. |
| **`MCASP_GBLCTL_REG`** | `0x44` | Registro Global de Control (lectura de estado sincronizado de bits de reset de RX `4:0` y TX `12:8`). |
| **`MCASP_RGBLCTL_REG`** | `0x60` | Alias de escritura global exclusivo para **RX** (modifica solo bits `4:0` de `GBLCTL` sin pisar TX). |
| **`MCASP_RMASK_REG`** | `0x64` | Máscara de bits para RX (`0xFFFFFFFF` para 32 bits). En reset vale `0`. |
| **`MCASP_RFMT_REG`** | `0x68` | Formato de flujo de bits RX (`Bit 15 RRVRS`: MSB primero, `Bits 7:4 RSSZ`: tamaño de slot `0xF` = 32 bits, `Bit 3 RBUSEL`: `1` = Configuration Bus). |
| **`MCASP_AFSRCTL_REG`** | `0x6C` | Control de Frame Sync RX (`Bits 15:7 RMOD = 2`: 2 slots I2S, `Bit 4 FSRDUR = 1`: ancho de 1 palabra). |
| **`MCASP_ACLKRCTL_REG`** | `0x70` | Control de clock de recepción (`Bit 7 CLKRP`: polaridad de muestreo RX). |
| **`MCASP_RTDM_REG`** | `0x78` | Slots TDM activos en RX (`0x3` = slots 0 y 1 activos). En reset vale `0`. |
| **`MCASP_RINTCTL_REG`** | `0x7C` | Control de interrupciones RX (`Bit 5 RDATA`: interrupción por dato recibido). |
| **`MCASP_RSTAT_REG`** | `0x80` | Estado del receptor RX (`Bit 5 RDATA`: dato disponible en `RBUF1`). |
| **`MCASP_XGBLCTL_REG`** | `0xA0` | Alias de escritura global exclusivo para **TX** (modifica solo bits `12:8` de `GBLCTL` sin pisar RX). |
| **`MCASP_XMASK_REG`** | `0xA4` | Máscara de bits para TX (`0xFFFFFFFF` para 32 bits). En reset vale `0`. |
| **`MCASP_XFMT_REG`** | `0xA8` | Formato de flujo de bits TX (`Bit 15 XRVRS`: MSB primero, `Bits 7:4 XSSZ`: tamaño de slot `0xF` = 32 bits, `Bit 3 XBUSEL`: `1` = Configuration Bus). |
| **`MCASP_AFSXCTL_REG`** | `0xAC` | Control de Frame Sync (WS) TX. `Bit 1 (FSXM)`: `1` = Interno (Master), `0` = Externo (Slave). `Bits 15:7 (XMOD)`: cantidad de slots (2 para I2S). `Bit 4 (FSXDUR)`: `1` = ancho de 1 palabra. `Bit 0 (FSXP)`: polaridad de frame sync. |
| **`MCASP_ACLKXCTL_REG`** | `0xB0` | Control de Bit Clock TX. `Bit 7 (CLKXP)`: polaridad. **`Bit 6 (ASYNC)`**: `0` = Síncrono (RX usa el mismo CLK y WS que TX), `1` = Asíncrono. `Bit 5 (CLKXM)`: `1` = Interno (Master), `0` = Externo (Slave). `Bits 4:0 (CLKXDIV)`: divisor `1..32`. |
| **`MCASP_AHCLKXCTL_REG`**| `0xB4` | Control de High-Freq Clock TX. `Bit 15 (HCLKXM)`: `1` = Fuente interna (`AUXCLK` 24 MHz). `Bits 11:0 (HCLKXDIV)`: divisor `1..4096`. |
| **`MCASP_XTDM_REG`** | `0xB8` | Slots TDM activos en TX (`0x3` = slots 0 y 1 activos). En reset vale `0`. |
| **`MCASP_XINTCTL_REG`** | `0xBC` | Control de interrupciones TX (`Bit 5 XDATA`: interrupción cuando `XBUF0` está vacío). |
| **`MCASP_XSTAT_REG`** | `0xC0` | Estado del transmisor TX (`Bit 5 XDATA`: buffer vacío, `Bit 0 XUNDRN`: underrun). |
| **`MCASP_SRCTL0_REG`** | `0x180` | Control del Serializer 0 (`AXR0`). `Bits 1:0 (SRMOD)`: `01b` (`1`) = Transmisor (TX), `10b` (`2`) = Receptor (RX). |
| **`MCASP_SRCTL1_REG`** | `0x184` | Control del Serializer 1 (`AXR1`). |
| **`MCASP_SRCTL2_REG`** | `0x188` | Control del Serializer 2 (`AXR2`). |
| **`MCASP_XBUF0_REG`** | `0x200` | Buffer de Transmisión para Serializer 0 (`AXR0`). |
| **`MCASP_RBUF1_REG`** | `0x284` | Buffer de Recepción para Serializer 1 (`AXR1`). |

---

## 3. Hallazgos y Reglas Críticas de Funcionamiento

1. **Sincronización de `GBLCTL` (`XGBLCTL` y `RGBLCTL`)**:
   - Los bits de `GBLCTL` tardan al menos **2 ciclos del reloj serie** en reflejarse tras una escritura.
   - Si se escribe en `XGBLCTL` e inmediatamente se lee y escribe `GBLCTL`, se leen los bits de TX aún en `0` y se vuelven a pisar a `0` (reseteando el transmisor).
   - **Regla**: Usar `MCASP_XGBLCTL_REG` (`0xA0`) para TX, `MCASP_RGBLCTL_REG` (`0x60`) para RX, y esperar con polling acotado (`mcasp_wait_gblctl`) leyendo `MCASP_GBLCTL_REG` (`0x44`) después de levantar cada bit de reset.

2. **Conmutación `MASTER` / `SLAVE` (`I2S_mode`)**:
   - Por seguridad eléctrica (evitar dos salidas enfrentadas al encender), el periférico debe inicializarse siempre en modo **`SLAVE`** (`PDIR[28]=0`, `PDIR[26]=0`, `CLKXM=0`, `FSXM=0`).
   - Al pasar a modo **`MASTER`**, se activan `CLKXM=1`, `FSXM=1`, `PDIR[28]=1`, `PDIR[26]=1` y se re-sincronizan los bits de `XGBLCTL` ahora que el clock interno está corriendo.

3. **Modo Síncrono TX/RX (`ASYNC = 0`)**:
   - Para que el receptor RX utilice exactamente el mismo `BCLK` (`P9_31`) y `WS` (`P9_29`) que el transmisor TX, se debe limpiar el **bit 6 (`ASYNC = 0`)** en `MCASP_ACLKXCTL_REG` (`0xB0`), ya que su valor de reset por defecto es `1` (asíncrono).
