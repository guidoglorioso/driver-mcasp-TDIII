#include "mydriver.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Guido Glorioso");
MODULE_DESCRIPTION("Driver de MCASP TDIII");

static const char __MODULE_NAME__[] = "td3_i2s";

static mymcasp_handler_t *g_handler = NULL;

/////////////////////////
// configuracion de HW
/////////////////////////

static void mcasp_wait_gblctl(mymcasp_handler_t *handler, uint32_t mask) {
    int timeout = 1000;
    while (((ioread32(handler->instance + MCASP_GBLCTL_REG) & mask) != mask) && --timeout > 0) {
        udelay(1);
    }
}

static void mcasp_wait_gblctl_clear(mymcasp_handler_t *handler, uint32_t mask) {
    int timeout = 1000;
    while (((ioread32(handler->instance + MCASP_GBLCTL_REG) & mask) != 0) && --timeout > 0) {
        udelay(1);
    }
}

/////////////////////////
// Handlers IRQ
/////////////////////////

static void mcasp_irq_tx_enable(mymcasp_handler_t *handler, bool enable) {
    uint32_t reg_val;

    if (!handler || !handler->instance)
        return;

    reg_val = ioread32(handler->instance + MCASP_XINTCTL_REG);
    if (enable)
        reg_val |= MCASP_INT_DATA;
    else
        reg_val &= ~MCASP_INT_DATA;
    iowrite32(reg_val, handler->instance + MCASP_XINTCTL_REG);
}

static void mcasp_irq_rx_enable(mymcasp_handler_t *handler, bool enable) {
    uint32_t reg_val;

    if (!handler || !handler->instance)
        return;

    reg_val = ioread32(handler->instance + MCASP_RINTCTL_REG);
    if (enable) {
        if (!(reg_val & MCASP_INT_DATA)) {
            // Descartar palabra vieja latcheada en RBUF1 y limpiar flags de overrun previos
            (void)MCASP_READ_RX_BUF(handler);
            iowrite32(MCASP_STAT_CLEAR_ALL, handler->instance + MCASP_RSTAT_REG);
        }
        reg_val |= MCASP_INT_DATA;
    } else {
        reg_val &= ~MCASP_INT_DATA;
    }
    iowrite32(reg_val, handler->instance + MCASP_RINTCTL_REG);
}

// TODO Revisar registros, hacer clean 
static irqreturn_t isr_tx_mcasp(int irq, void *dev_id) {
    mymcasp_handler_t *handler = (mymcasp_handler_t *)dev_id;
    uint32_t xstat;
    uint32_t word = 0;
    uint32_t reg_val;
    uint8_t byte;

    if (!handler || !handler->instance || !handler->clk_enabled)
        return IRQ_NONE;

    switch (handler->tx_state) {
    case TX_STATE_IDLE:
        word = 0;
        if (!cb_is_empty(&handler->tx_buf))
            handler->tx_state = TX_STATE_SENDING;
        break;

    case TX_STATE_SENDING:
        if (!cb_is_empty(&handler->tx_buf)) {
            if (cb_pop(&handler->tx_buf, &byte) == 0) word |= ((uint32_t)byte << 24);
            if (cb_pop(&handler->tx_buf, &byte) == 0) word |= ((uint32_t)byte << 16);
            if (cb_pop(&handler->tx_buf, &byte) == 0) word |= ((uint32_t)byte << 8);
            if (cb_pop(&handler->tx_buf, &byte) == 0) word |= ((uint32_t)byte << 0);

            if (cb_half_free(&handler->tx_buf))
                wake_up_interruptible(&handler->tx_wq);
        }

        if (cb_is_empty(&handler->tx_buf))
            handler->tx_state = TX_STATE_IDLE;
        break;

    default:
        handler->tx_state = TX_STATE_IDLE;
        word = 0;
        break;
    }

    xstat = ioread32(handler->instance + MCASP_XSTAT_REG);

    /*
     * Recuperación de emergencia solo si ocurrió Underrun (sin tocar XFRST para no cortar AFSX).
     * Se espera explícitamente a que GBLCTL refleje la baja de XSMRST y XSRCLR antes de rearmar.
     */
    if (xstat & MCASP_XSTAT_XUNDRN) {
        reg_val = MCASP_GBL_XFRST | MCASP_GBL_XHCLKRST | MCASP_GBL_XCLKRST;
        iowrite32(reg_val, handler->instance + MCASP_XGBLCTL_REG);
        mcasp_wait_gblctl_clear(handler, MCASP_GBL_XSMRST | MCASP_GBL_XSRCLR);

        iowrite32(MCASP_STAT_CLEAR_ALL, handler->instance + MCASP_XSTAT_REG);

        reg_val |= MCASP_GBL_XSRCLR;
        iowrite32(reg_val, handler->instance + MCASP_XGBLCTL_REG);
        mcasp_wait_gblctl(handler, MCASP_GBL_XSRCLR);

        MCASP_WRITE_TX_BUF(handler, word);

        reg_val |= MCASP_GBL_XSMRST;
        iowrite32(reg_val, handler->instance + MCASP_XGBLCTL_REG);
        mcasp_wait_gblctl(handler, MCASP_GBL_XSMRST);
    } else {
        MCASP_WRITE_TX_BUF(handler, word);
    }

    return IRQ_HANDLED;
}
// TODO Revisar registros, hacer clean
static irqreturn_t isr_rx_mcasp(int irq, void *dev_id) {
    mymcasp_handler_t *handler = (mymcasp_handler_t *)dev_id;
    uint32_t rstat;
    uint32_t word;

    if (!handler || !handler->instance || !handler->clk_enabled)
        return IRQ_NONE;

    rstat = ioread32(handler->instance + MCASP_RSTAT_REG);

    if (rstat & MCASP_INT_DATA) {
        word = MCASP_READ_RX_BUF(handler);

        cb_push(&handler->rx_buf, (uint8_t)((word >> 24) & 0xFF));
        cb_push(&handler->rx_buf, (uint8_t)((word >> 16) & 0xFF));
        cb_push(&handler->rx_buf, (uint8_t)((word >> 8)  & 0xFF));
        cb_push(&handler->rx_buf, (uint8_t)((word >> 0)  & 0xFF));

        wake_up_interruptible(&handler->rx_wq);
    }

    // Limpiar flags de error/estado adicionales en RSTAT si los hubiera
    if (rstat & ~MCASP_INT_DATA) {
        iowrite32(rstat & ~MCASP_INT_DATA, handler->instance + MCASP_RSTAT_REG);
    }

    return IRQ_HANDLED;
}

/////////////////////////
// Rutinas de inicializacion
/////////////////////////

static int init_irq(mymcasp_handler_t *handler) {
    int ret;

    handler->irq_tx = platform_get_irq_byname(handler->pdev, "tx");
    if (handler->irq_tx < 0) {
        pr_err("No se pudo obtener la IRQ de TX desde el DT: %d\n", handler->irq_tx);
        return handler->irq_tx;
    }

    ret = devm_request_irq(&handler->pdev->dev,
                           handler->irq_tx,
                           isr_tx_mcasp,
                           0,
                           "td3_i2s_tx",
                           handler);
    if (ret) {
        pr_err("Error al registrar isr_tx_mcasp (IRQ %d): %d\n", handler->irq_tx, ret);
        return ret;
    }

    handler->irq_rx = platform_get_irq_byname(handler->pdev, "rx");
    if (handler->irq_rx < 0) {
        pr_err("No se pudo obtener la IRQ de RX desde el DT: %d\n", handler->irq_rx);
        return handler->irq_rx;
    }

    ret = devm_request_irq(&handler->pdev->dev,
                           handler->irq_rx,
                           isr_rx_mcasp,
                           0,
                           "td3_i2s_rx",
                           handler);
    if (ret) {
        pr_err("Error al registrar isr_rx_mcasp (IRQ %d): %d\n", handler->irq_rx, ret);
        return ret;
    }
    
    return 0;
}

static void mcasp_reg_config(mymcasp_handler_t *handler, bool enable);

static int set_i2s_mode(mymcasp_handler_t *handler, uint8_t mode) {
    if (!handler || !handler->instance)
        return -ENODEV;

    if (mode != SLAVE && mode != MASTER)
        return -EINVAL;

    handler->i2s_mode = mode;

    // Si el clock ya está encendido, reaplicar la configuración completa del McASP
    if (handler->clk_enabled) {
        mcasp_reg_config(handler, true);
    }

    return 0;
}

static int init_clock(mymcasp_handler_t *handler) {
    int ret;

    if (!handler->fck) {
        handler->fck = devm_clk_get(&handler->pdev->dev, "fck");
        if (IS_ERR(handler->fck)) {
            ret = PTR_ERR(handler->fck);
            pr_err("No se pudo obtener el reloj fck: %d\n", ret);
            handler->fck = NULL;
            return ret;
        }
    }

    if (!handler->clk_enabled) {
        ret = clk_prepare_enable(handler->fck);
        if (ret) {
            pr_err("Error al habilitar el reloj fck: %d\n", ret);
            return ret;
        }
        handler->clk_enabled = true;
    }

    return 0;
}

static void init_gpio(mymcasp_handler_t *handler) {
    uint32_t reg_val;

    if (!handler->instance)
        return;

    // Seteo de pines en modo I2S: AFSX, ACLKX, AXR1 (RX) y AXR0 (TX)
    reg_val = ioread32(handler->instance + MCASP_PFUNC_REG);
    reg_val &= ~(MCASP_PIN_AFSX | MCASP_PIN_ACLKX | MCASP_PIN_AXR1 | MCASP_PIN_AXR0);
    iowrite32(reg_val, handler->instance + MCASP_PFUNC_REG);

    // Seteo de pin de datos TX (AXR0) como salida y RX (AXR1) como entrada.
    // Nota: Los pines de WS (AFSX) y CLK (ACLKX) los maneja mcasp_reg_config().
    reg_val = ioread32(handler->instance + MCASP_PDIR_REG);
    reg_val |= MCASP_PIN_AXR0;
    reg_val &= ~MCASP_PIN_AXR1;
    iowrite32(reg_val, handler->instance + MCASP_PDIR_REG);
}

static void mcasp_calc_clk_divs(uint32_t target_freq, uint32_t *ahclk_div, uint32_t *aclk_div) {
    uint32_t total_div;

    if (!ahclk_div || !aclk_div)
        return;

    if (target_freq == 0)
        target_freq = FAULT_FREQ; // evito division por cero

    total_div = XTAL_FREQ / target_freq;
    if (total_div < 1)
        total_div = 1;

    if (total_div - 1 <= MCASP_AHCLKXDIV_MAX) {
        *ahclk_div = total_div - 1;
        *aclk_div = 0;
    } else {
        *aclk_div = MCASP_ACLKXDIV_MAX;
        if (total_div >= (MCASP_ACLKXDIV_MAX + 1))
            *ahclk_div = (total_div / (MCASP_ACLKXDIV_MAX + 1)) - 1;
        else
            *ahclk_div = 0;

        if (*ahclk_div > MCASP_AHCLKXDIV_MAX)
            *ahclk_div = MCASP_AHCLKXDIV_MAX;
    }
}

// TODO Revisar registros, hacer clean
static void mcasp_reg_config(mymcasp_handler_t *handler, bool enable){
    uint32_t ahclk_div;
    uint32_t aclk_div;
    uint32_t reg_val;
    uint32_t xgbl = 0;
    uint32_t rgbl = 0;

    if(!enable){
        // Dejar AFSX y ACLKX en alta impedancia (entradas) por seguridad eléctrica
        reg_val = ioread32(handler->instance + MCASP_PDIR_REG);
        reg_val &= ~(MCASP_PIN_AFSX | MCASP_PIN_ACLKX);
        iowrite32(reg_val, handler->instance + MCASP_PDIR_REG);

        // Detener clocks y máquinas de estado TX/RX poniendo GBLCTL = 0 y esperar sincronización
        iowrite32(0, handler->instance + MCASP_GBLCTL_REG);
        mcasp_wait_gblctl_clear(handler, MCASP_GBL_TX_MASK | MCASP_GBL_RX_MASK);
        return;
    }
    
    // calculo divisores de clocks.
    mcasp_calc_clk_divs(handler->hertz, &ahclk_div, &aclk_div);

    // Paso 1 Transmit/Receive Section Initialization (22.3.12.2)
    // Resetear McASP a valores por defecto poniendo GBLCTL = 0
    iowrite32(0, handler->instance + MCASP_GBLCTL_REG);
    mcasp_wait_gblctl_clear(handler, MCASP_GBL_TX_MASK | MCASP_GBL_RX_MASK);

    // Paso 2 Transmit/Receive Section Initialization (22.3.12.2)
    // Configurar todos los registros del McASP (excepto GBLCTL) en el orden:
    // 2.a PWRIDLESYSCONFIG (gestionado por el clock/PRCM del SoC)

    // 2.b Registros de Recepción: RMASK, RFMT, AFSRCTL, ACLKRCTL, RTDM, RINTCTL
    iowrite32(0xFFFFFFFF, handler->instance + MCASP_RMASK_REG);

    // RDATDLY = 1 (1 ciclo de retardo - estándar I2S), RRVRS = 1 (MSB primero), RSSZ = 32 bits, RBUSEL = 1 (CFG Bus)
    iowrite32(MCASP_FMT_DATDLY(1) | MCASP_FMT_RVRS_MSB | MCASP_FMT_SSZ_32BIT | MCASP_FMT_BUSEL_CFG,
              handler->instance + MCASP_RFMT_REG);

    // Frame Sync I2S de 2 slots (RMOD = 2) y ancho de 1 palabra (FSRDUR)
    reg_val = ioread32(handler->instance + MCASP_AFSRCTL_REG);
    reg_val &= ~(MCASP_AFS_MOD_MASK | MCASP_AFS_FRWID_WORD);
    reg_val |= MCASP_AFS_MOD(2) | MCASP_AFS_FRWID_WORD;
    iowrite32(reg_val, handler->instance + MCASP_AFSRCTL_REG);

    // CLKRP = 0 (Falling edge en RX)
    reg_val = ioread32(handler->instance + MCASP_ACLKRCTL_REG);
    reg_val &= ~MCASP_ACLK_CLKP;
    iowrite32(reg_val, handler->instance + MCASP_ACLKRCTL_REG);

    // Habilitar los slots TDM 0 y 1 en RX
    iowrite32(MCASP_TDM_2SLOTS, handler->instance + MCASP_RTDM_REG);

    // Mantener RINTCTL deshabilitado hasta el Paso 5
    mcasp_irq_rx_enable(handler, false);

    // 2.c Registros de Transmisión: XMASK, XFMT, AFSXCTL, ACLKXCTL, AHCLKXCTL, XTDM, XINTCTL
    iowrite32(0xFFFFFFFF, handler->instance + MCASP_XMASK_REG);

    // XDATDLY = 1 (1 ciclo de retardo - estándar I2S), XRVRS = 1 (MSB primero), XSSZ = 32 bits, XBUSEL = 1 (CFG Bus)
    iowrite32(MCASP_FMT_DATDLY(1) | MCASP_FMT_RVRS_MSB | MCASP_FMT_SSZ_32BIT | MCASP_FMT_BUSEL_CFG,
              handler->instance + MCASP_XFMT_REG);

    // Frame Sync I2S de 2 slots (XMOD = 2), ancho de 1 palabra (FSXDUR) y FSXM según modo
    reg_val = ioread32(handler->instance + MCASP_AFSXCTL_REG);
    reg_val &= ~(MCASP_AFS_MOD_MASK | MCASP_AFS_FRWID_WORD | MCASP_AFS_FSXM);
    reg_val |= MCASP_AFS_MOD(2) | MCASP_AFS_FRWID_WORD;
    if (handler->i2s_mode == MASTER)
        reg_val |= MCASP_AFS_FSXM; // FSXM = 1 (Interno / Master)
    iowrite32(reg_val, handler->instance + MCASP_AFSXCTL_REG);

    // HCLKXM = 1 (AUXCLK interno) y divisor HCLKXDIV
    reg_val = ioread32(handler->instance + MCASP_AHCLKXCTL_REG);
    reg_val &= ~(MCASP_AHCLKX_DIV_MASK | MCASP_AHCLKX_HCLKXM);
    reg_val |= MCASP_AHCLKX_HCLKXM | (ahclk_div & MCASP_AHCLKX_DIV_MASK);
    iowrite32(reg_val, handler->instance + MCASP_AHCLKXCTL_REG);

    // CLKXP = 0 (Rising edge), ASYNC = 0 (Síncrono TX/RX), CLKXDIV y CLKXM según modo
    reg_val = ioread32(handler->instance + MCASP_ACLKXCTL_REG);
    reg_val &= ~(MCASP_ACLK_CLKP | MCASP_ACLKX_ASYNC | MCASP_ACLKX_CLKXM | MCASP_ACLKX_DIV_MASK);
    reg_val |= (aclk_div & MCASP_ACLKX_DIV_MASK);
    if (handler->i2s_mode == MASTER)
        reg_val |= MCASP_ACLKX_CLKXM; // CLKXM = 1 (Interno / Master)
    iowrite32(reg_val, handler->instance + MCASP_ACLKXCTL_REG);

    // Habilitar los slots TDM 0 y 1 en TX
    iowrite32(MCASP_TDM_2SLOTS, handler->instance + MCASP_XTDM_REG);

    // Mantener XINTCTL deshabilitado hasta que write() cargue datos en tx_buf
    mcasp_irq_tx_enable(handler, false);

    // 2.d Registros de Serializers: SRCTL0 (TX) y SRCTL1 (RX)
    reg_val = ioread32(handler->instance + MCASP_SRCTL0_REG);
    reg_val &= ~MCASP_SRCTL_SRMOD_MASK;
    reg_val |= MCASP_SRCTL_SRMOD_TX; // Serializer 0 (AXR0) como Transmisor
    iowrite32(reg_val, handler->instance + MCASP_SRCTL0_REG);

    reg_val = ioread32(handler->instance + MCASP_SRCTL1_REG);
    reg_val &= ~MCASP_SRCTL_SRMOD_MASK;
    reg_val |= MCASP_SRCTL_SRMOD_RX; // Serializer 1 (AXR1) como Receptor
    iowrite32(reg_val, handler->instance + MCASP_SRCTL1_REG);

    // 2.e Registros Globales: PFUNC y PDIR (programar PDIR recién después de clocks y frames)
    init_gpio(handler);
    reg_val = ioread32(handler->instance + MCASP_PDIR_REG);
    if (handler->i2s_mode == MASTER)
        reg_val |= (MCASP_PIN_AFSX | MCASP_PIN_ACLKX);  // AFSX y ACLKX como salidas
    else
        reg_val &= ~(MCASP_PIN_AFSX | MCASP_PIN_ACLKX); // AFSX y ACLKX como entradas
    iowrite32(reg_val, handler->instance + MCASP_PDIR_REG);

    // 2.f Registros DIT: No aplica (modo TDM/I2S, no DIT)

    // Paso 3 Transmit/Receive Section Initialization (22.3.12.2)
    // Iniciar los relojes serie de alta frecuencia (XHCLKRST y RHCLKRST)
    xgbl |= MCASP_GBL_XHCLKRST;
    iowrite32(xgbl, handler->instance + MCASP_XGBLCTL_REG);
    mcasp_wait_gblctl(handler, MCASP_GBL_XHCLKRST);

    rgbl |= MCASP_GBL_RHCLKRST;
    iowrite32(rgbl, handler->instance + MCASP_RGBLCTL_REG);
    mcasp_wait_gblctl(handler, MCASP_GBL_RHCLKRST);

    // Paso 4 Transmit/Receive Section Initialization (22.3.12.2)
    // Iniciar los relojes serie (XCLKRST y RCLKRST)
    xgbl |= MCASP_GBL_XCLKRST;
    iowrite32(xgbl, handler->instance + MCASP_XGBLCTL_REG);
    mcasp_wait_gblctl(handler, MCASP_GBL_XCLKRST);

    rgbl |= MCASP_GBL_RCLKRST;
    iowrite32(rgbl, handler->instance + MCASP_RGBLCTL_REG);
    mcasp_wait_gblctl(handler, MCASP_GBL_RCLKRST);

    // Paso 5 Transmit/Receive Section Initialization (22.3.12.2)
    // Mantener interrupción de recepción (RDATA) deshabilitada hasta read(); TX (XDATA) se habilita en Paso 7
    (void)MCASP_READ_RX_BUF(handler);
    mcasp_irq_rx_enable(handler, false);

    // Paso 6 Transmit/Receive Section Initialization (22.3.12.2)
    // Limpiar XSTAT y RSTAT (FFFFh) y activar serializers (XSRCLR y RSRCLR)
    iowrite32(MCASP_STAT_CLEAR_ALL, handler->instance + MCASP_XSTAT_REG);
    iowrite32(MCASP_STAT_CLEAR_ALL, handler->instance + MCASP_RSTAT_REG);

    xgbl |= MCASP_GBL_XSRCLR;
    iowrite32(xgbl, handler->instance + MCASP_XGBLCTL_REG);
    mcasp_wait_gblctl(handler, MCASP_GBL_XSRCLR);

    rgbl |= MCASP_GBL_RSRCLR;
    iowrite32(rgbl, handler->instance + MCASP_RGBLCTL_REG);
    mcasp_wait_gblctl(handler, MCASP_GBL_RSRCLR);

    // Paso 7 Transmit/Receive Section Initialization (22.3.12.2)
    // Precargar XBUF0 con 0 antes de soltar XSMRST para arrancar transmitiendo 0 sin caer en Underrun
    handler->tx_state = TX_STATE_IDLE;
    MCASP_WRITE_TX_BUF(handler, 0);
    {
        int timeout = 1000;
        while ((ioread32(handler->instance + MCASP_XSTAT_REG) & MCASP_INT_DATA) && --timeout > 0)
            udelay(1);
    }
    mcasp_irq_tx_enable(handler, true);

    // Paso 8 Transmit/Receive Section Initialization (22.3.12.2)
    // Liberar máquinas de estado de reset (XSMRST y RSMRST)
    xgbl |= MCASP_GBL_XSMRST;
    iowrite32(xgbl, handler->instance + MCASP_XGBLCTL_REG);
    mcasp_wait_gblctl(handler, MCASP_GBL_XSMRST);

    rgbl |= MCASP_GBL_RSMRST;
    iowrite32(rgbl, handler->instance + MCASP_RGBLCTL_REG);
    mcasp_wait_gblctl(handler, MCASP_GBL_RSMRST);

    // Paso 9 Transmit/Receive Section Initialization (22.3.12.2)
    // Liberar generadores de Frame Sync de reset (XFRST y RFRST)
    xgbl |= MCASP_GBL_XFRST;
    iowrite32(xgbl, handler->instance + MCASP_XGBLCTL_REG);
    mcasp_wait_gblctl(handler, MCASP_GBL_XFRST);

    rgbl |= MCASP_GBL_RFRST;
    iowrite32(rgbl, handler->instance + MCASP_RGBLCTL_REG);
    mcasp_wait_gblctl(handler, MCASP_GBL_RFRST);

    // Paso 10 Transmit/Receive Section Initialization (22.3.12.2)
    // Las transferencias comienzan en el primer flanco de Frame Sync
}

static void mymcasp_tx_stop(mymcasp_handler_t *handler) {
    if (!handler)
        return;

    cb_reset(&handler->tx_buf);
    handler->tx_state = TX_STATE_IDLE;
}

static void mymcasp_hw_deinit(mymcasp_handler_t *handler) {
    if (!handler)
        return;

    // Detener transferencias y apagar el reloj del periférico solo cuando no quedan usuarios activos
    mutex_lock(&handler->hw_lock);
    if (handler->hw_users > 0)
        handler->hw_users--;

    if (handler->hw_users == 0 && handler->clk_enabled) {
        // 1. Deshabilitar interrupciones de TX y RX
        if (handler->instance) {
            mcasp_irq_tx_enable(handler, false);
            mcasp_irq_rx_enable(handler, false);
        }
        handler->clk_enabled = false;

        // 2. Esperar a que finalice cualquier ISR en curso antes de tocar registros/clocks
        if (handler->irq_tx >= 0)
            synchronize_irq(handler->irq_tx);
        if (handler->irq_rx >= 0)
            synchronize_irq(handler->irq_rx);

        // 3. Poner GBLCTL = 0 y pines de reloj en entrada
        if (handler->instance) {
            mcasp_reg_config(handler, false);

            // 4. Vaciar buffers HW, limpiar RSTAT/XSTAT y resetear buffers SW/estado TX
            (void)MCASP_READ_RX_BUF(handler);
            iowrite32(0, handler->instance + MCASP_XBUF0_REG);
            iowrite32(MCASP_STAT_CLEAR_ALL, handler->instance + MCASP_RSTAT_REG);
            iowrite32(MCASP_STAT_CLEAR_ALL, handler->instance + MCASP_XSTAT_REG);
        }

        cb_reset(&handler->tx_buf);
        cb_reset(&handler->rx_buf);
        handler->tx_state = TX_STATE_IDLE;

        // 5. Apagar el clock funcional del periférico
        if (handler->fck)
            clk_disable_unprepare(handler->fck);
    }
    mutex_unlock(&handler->hw_lock);
}

static int mymcasp_hw_init(mymcasp_handler_t *handler) {
    int status = 0;

    if (!handler || !handler->instance)
        return -ENODEV;

    mutex_lock(&handler->hw_lock);

    if (handler->hw_users == 0) {
        // Clock Gating
        status = init_clock(handler);
        if (status){
            mutex_unlock(&handler->hw_lock);
            return status;
        }

        // Resetear índices de los buffers circulares y estado TX antes de habilitar el McASP
        cb_reset(&handler->tx_buf);
        cb_reset(&handler->rx_buf);
        handler->tx_state = TX_STATE_IDLE;

        // Inicializa el McASP siguiendo la secuencia 22.3.12.2 (Pasos 1 a 10)
        mcasp_reg_config(handler, true);
    }

    handler->hw_users++;

    mutex_unlock(&handler->hw_lock);
    return status;
}


/////////////////////////
// Interfaz Sysfs - (/sys/class/td3_i2s/)
/////////////////////////

/**
 * @brief Lee el modo de operación I2S actual (`0` = Slave, `1` = Master) en `buf`.
 *        Requiere exclusión mutua sobre TX, RX y `/dev/td3_i2s` abierto.
 */
static ssize_t master_show(const struct class *cls, const struct class_attribute *attr, char *buf) {
    mymcasp_handler_t *handler = g_handler;
    ssize_t ret;

    if (!handler || !handler->instance)
        return -ENODEV;

    if (!mutex_trylock(&handler->tx_lock))
        return -EBUSY;

    if (!mutex_trylock(&handler->rx_lock)) {
        mutex_unlock(&handler->tx_lock);
        return -EBUSY;
    }

    mutex_lock(&handler->hw_lock);
    if (handler->dev_opened) {
        mutex_unlock(&handler->hw_lock);
        mutex_unlock(&handler->rx_lock);
        mutex_unlock(&handler->tx_lock);
        return -EBUSY;
    }
    mutex_unlock(&handler->hw_lock);

    ret = sysfs_emit(buf, "%u\n", handler->i2s_mode);

    mutex_unlock(&handler->rx_lock);
    mutex_unlock(&handler->tx_lock);
    return ret;
}

/**
 * @brief Configura el modo I2S desde `buf` (`0`/`SLAVE` o `1`/`MASTER`) y actualiza el McASP.
 *        Retorna `-EBUSY` si hay transferencias activas o `/dev/td3_i2s` está abierto.
 */
static ssize_t master_store(const struct class *cls, const struct class_attribute *attr, const char *buf, size_t count) {
    mymcasp_handler_t *handler = g_handler;
    char kbuf[16];
    char *arg;
    size_t len;
    uint8_t mode;
    int ret;

    if (!handler || !handler->instance)
        return -ENODEV;

    if (!mutex_trylock(&handler->tx_lock))
        return -EBUSY;

    if (!mutex_trylock(&handler->rx_lock)) {
        mutex_unlock(&handler->tx_lock);
        return -EBUSY;
    }

    mutex_lock(&handler->hw_lock);
    if (handler->dev_opened) {
        mutex_unlock(&handler->hw_lock);
        mutex_unlock(&handler->rx_lock);
        mutex_unlock(&handler->tx_lock);
        return -EBUSY;
    }
    mutex_unlock(&handler->hw_lock);

    if (count == 0) {
        mutex_unlock(&handler->rx_lock);
        mutex_unlock(&handler->tx_lock);
        return -EINVAL;
    }

    len = min(count, sizeof(kbuf) - 1);
    memcpy(kbuf, buf, len);
    kbuf[len] = '\0';
    arg = strim(kbuf);

    if (strcmp(arg, "1") == 0 || strcasecmp(arg, "MASTER") == 0 || strcasecmp(arg, "MAESTRO") == 0) {
        mode = MASTER;
    } else if (strcmp(arg, "0") == 0 || strcasecmp(arg, "SLAVE") == 0 || strcasecmp(arg, "ESCLAVO") == 0) {
        mode = SLAVE;
    } else {
        mutex_unlock(&handler->rx_lock);
        mutex_unlock(&handler->tx_lock);
        return -EINVAL;
    }

    ret = set_i2s_mode(handler, mode);
    mutex_unlock(&handler->rx_lock);
    mutex_unlock(&handler->tx_lock);
    if (ret < 0)
        return ret;

    return count;
}

static CLASS_ATTR_RW(master);

/**
 * @brief Lee de forma bloqueante `count` bytes recibidos por I2S hacia `buf` desde sysfs.
 *        Enciende el HW al iniciar, espera en `TASK_INTERRUPTIBLE` y lo apaga al finalizar.
 */
static ssize_t recibir_read(struct file *filp, struct kobject *kobj,
                            struct bin_attribute *attr, char *buf,
                            loff_t off, size_t count) {
    mymcasp_handler_t *handler = g_handler;
    size_t read_bytes = 0;
    uint8_t byte;
    int ret;

    if (!handler || !handler->instance)
        return -ENODEV;

    if (off > 0)
        return 0;

    if (!mutex_trylock(&handler->rx_lock))
        return -EBUSY;

    mutex_lock(&handler->hw_lock);
    if (handler->dev_opened) {
        mutex_unlock(&handler->hw_lock);
        mutex_unlock(&handler->rx_lock);
        return -EBUSY;
    }
    mutex_unlock(&handler->hw_lock);

    ret = mymcasp_hw_init(handler);
    if (ret < 0) {
        mutex_unlock(&handler->rx_lock);
        return ret;
    }

    if (count == 0) {
        mymcasp_hw_deinit(handler);
        mutex_unlock(&handler->rx_lock);
        return 0;
    }

    count = min_t(size_t, count, BUFF_SIZE);

    mcasp_irq_rx_enable(handler, true);
    if (wait_event_interruptible(handler->rx_wq, cb_count(&handler->rx_buf) >= count)) {
        mcasp_irq_rx_enable(handler, false);
        while (read_bytes < count && cb_pop(&handler->rx_buf, &byte) == 0) {
            buf[read_bytes++] = (char)byte;
        }
        mymcasp_hw_deinit(handler);
        mutex_unlock(&handler->rx_lock);
        return read_bytes > 0 ? read_bytes : -ERESTARTSYS;
    }
    mcasp_irq_rx_enable(handler, false);

    while (read_bytes < count && cb_pop(&handler->rx_buf, &byte) == 0) {
        buf[read_bytes++] = (char)byte;
    }

    mymcasp_hw_deinit(handler);
    mutex_unlock(&handler->rx_lock);
    return read_bytes;
}

static BIN_ATTR_RO(recibir, 0);

/**
 * @brief Transmite de forma bloqueante `count` bytes desde `buf` por I2S desde sysfs.
 *        Enciende el HW, encola en `tx_buf`, espera el vaciado completo y apaga el HW.
 */
static ssize_t transmitir_write(struct file *filp, struct kobject *kobj,
                                struct bin_attribute *attr, char *buf,
                                loff_t off, size_t count) {
    mymcasp_handler_t *handler = g_handler;
    size_t written = 0;
    int ret;

    if (!handler || !handler->instance)
        return -ENODEV;

    if (!mutex_trylock(&handler->tx_lock))
        return -EBUSY;

    mutex_lock(&handler->hw_lock);
    if (handler->dev_opened) {
        mutex_unlock(&handler->hw_lock);
        mutex_unlock(&handler->tx_lock);
        return -EBUSY;
    }
    mutex_unlock(&handler->hw_lock);

    ret = mymcasp_hw_init(handler);
    if (ret < 0) {
        mutex_unlock(&handler->tx_lock);
        return ret;
    }

    if (count == 0) {
        mymcasp_hw_deinit(handler);
        mutex_unlock(&handler->tx_lock);
        return 0;
    }

    count = min_t(size_t, count, BUFF_SIZE);

    while (written < count) {
        if (cb_push(&handler->tx_buf, (uint8_t)buf[written]) == 0) {
            written++;
            continue;
        }

        // Habilitar IRQ de TX para que isr_tx_mcasp libere espacio en tx_buf
        mcasp_irq_tx_enable(handler, true);

        if (wait_event_interruptible(handler->tx_wq, cb_half_free(&handler->tx_buf))) {
            mymcasp_tx_stop(handler);
            mymcasp_hw_deinit(handler);
            mutex_unlock(&handler->tx_lock);
            return written > 0 ? written : -ERESTARTSYS;
        }
    }

    // Habilito la interrupcion del Tx y espero a que se transmita el buffer antes de apagar el HW
    mcasp_irq_tx_enable(handler, true);

    if (wait_event_interruptible(handler->tx_wq, cb_is_empty(&handler->tx_buf))) {
        mymcasp_tx_stop(handler);
        mymcasp_hw_deinit(handler);
        mutex_unlock(&handler->tx_lock);
        return written > 0 ? written : -ERESTARTSYS;
    }

    mymcasp_tx_stop(handler);
    mymcasp_hw_deinit(handler);
    mutex_unlock(&handler->tx_lock);
    return written;
}

static BIN_ATTR_WO(transmitir, 0);

// Defino atributos de la clase /sys/class/td3_i2s/

static struct attribute *td3_i2s_attrs[] = {
    &class_attr_master.attr,
    NULL,
};

static struct bin_attribute *td3_i2s_bin_attrs[] = {
    &bin_attr_recibir,
    &bin_attr_transmitir,
    NULL,
};

static const struct attribute_group td3_i2s_attr_group = {
    .attrs = td3_i2s_attrs,
    .bin_attrs = td3_i2s_bin_attrs,
};

static const struct attribute_group *td3_i2s_class_groups[] = {
    &td3_i2s_attr_group,
    NULL,
};

/**
 * @brief Asigna permisos `0666` al nodo `/dev/td3_i2s` al registrarse en la clase sysfs.
 */
static char *td3_i2s_devnode(const struct device *dev, umode_t *mode) {
    if (mode) {
        *mode = 0666; // Permisos de lectura y escritura para todos
    }
    return NULL;
}

static const struct class td3_i2s_class = {
    .name = "td3_i2s",
    .class_groups = td3_i2s_class_groups,
    .devnode = td3_i2s_devnode,
};


/////////////////////////
// FileOperations
/////////////////////////


/**
 * @brief Abre `/dev/td3_i2s` con acceso exclusivo (`-EBUSY` si ya está abierto o sysfs activo)
 *        e inicializa el reloj y el McASP guardando el handler en `file->private_data`.
 */
static int mymcasp_open(struct inode *inode, struct file *file) {
    int status;
    mymcasp_handler_t *handler = container_of(inode->i_cdev, mymcasp_handler_t, mymcasp_cstm_cdev.mymcasp_cdev);

    // Exclusión mutua de apertura: si el dispositivo ya está abierto o sysfs está usando Tx/Rx, retornar -EBUSY
    mutex_lock(&handler->hw_lock);
    if (handler->dev_opened || mutex_is_locked(&handler->tx_lock) || mutex_is_locked(&handler->rx_lock)) {
        mutex_unlock(&handler->hw_lock);
        pr_warn("%s Dispositivo en uso\n", __MODULE_NAME__);
        return -EBUSY;
    }
    handler->dev_opened = true;
    mutex_unlock(&handler->hw_lock);

    status = mymcasp_hw_init(handler);
    if (status) {
        mutex_lock(&handler->hw_lock);
        handler->dev_opened = false;
        mutex_unlock(&handler->hw_lock);
        return status;
    }

    file->private_data = handler;
    return 0;
}

/**
 * @brief Cierra `/dev/td3_i2s`: espera el vaciado final de `tx_buf` (si el proceso no está
 *        saliendo por señal), detiene el McASP, apaga el reloj y libera la exclusividad.
 */
static int mymcasp_close(struct inode *inode, struct file *file) {
    mymcasp_handler_t *handler = file->private_data;

    if (handler) {
        if (!(current->flags & PF_EXITING) && !cb_is_empty(&handler->tx_buf)) {
            wait_event_interruptible_timeout(handler->tx_wq,
                                             cb_is_empty(&handler->tx_buf),
                                             msecs_to_jiffies(2000));
            usleep_range(1000, 1500);
        }

        mymcasp_tx_stop(handler);
        mymcasp_hw_deinit(handler);

        mutex_lock(&handler->hw_lock);
        handler->dev_opened = false;
        mutex_unlock(&handler->hw_lock);
    }

    file->private_data = NULL;
    return 0;
}

/**
 * @brief Lee de forma bloqueante `len` bytes recibidos hacia el buffer de usuario `buf`.
 *        Habilita la IRQ de RX (sin apagarla al salir para permitir lecturas continuas).
 */
static ssize_t mymcasp_read(struct file *file, char __user *buf, size_t len, loff_t *offset) {
    mymcasp_handler_t *dev = file->private_data;
    size_t total_read = 0;
    uint8_t *kbuff;
    uint8_t byte;

    if (!dev || !dev->instance)
        return -ENODEV;

    if (len == 0)
        return 0;

    if (!mutex_trylock(&dev->rx_lock))
        return -EBUSY;

    len = min_t(size_t, len, BUFF_SIZE);

    kbuff = kmalloc(len, GFP_KERNEL);
    if (kbuff == NULL) {
        pr_err("%s Fallo en Kmalloc\n", __MODULE_NAME__);
        mutex_unlock(&dev->rx_lock);
        return -ENOMEM;
    }

    mcasp_irq_rx_enable(dev, true);
    if (wait_event_interruptible(dev->rx_wq, cb_count(&dev->rx_buf) >= len)) {
        while (total_read < len && cb_pop(&dev->rx_buf, &byte) == 0) {
            kbuff[total_read++] = byte;
        }
        if (total_read == 0) {
            kfree(kbuff);
            mutex_unlock(&dev->rx_lock);
            return -ERESTARTSYS;
        }
        goto out_copy;
    }

    while (total_read < len && cb_pop(&dev->rx_buf, &byte) == 0) {
        kbuff[total_read++] = byte;
    }

out_copy:
    if (copy_to_user(buf, kbuff, total_read)) {
        kfree(kbuff);
        mutex_unlock(&dev->rx_lock);
        return -EFAULT;
    }

    kfree(kbuff);
    mutex_unlock(&dev->rx_lock);
    return total_read;
}


/**
 * @brief Copia `len` bytes desde el buffer de usuario `buf` y los encola en `tx_buf` para TX
 *        por interrupciones; duerme en `TASK_INTERRUPTIBLE` si el buffer se llena.
 */
static ssize_t mymcasp_write(struct file *file, const char __user *buf, size_t len, loff_t *offset) {
    mymcasp_handler_t *dev = file->private_data;
    size_t total_written = 0;
    uint8_t *kbuff;

    if (!dev || !dev->instance)
        return -ENODEV;

    if (len == 0)
        return 0;

    if (!mutex_trylock(&dev->tx_lock))
        return -EBUSY;

    len = min_t(size_t, len, BUFF_SIZE);
    kbuff = kmalloc(len, GFP_KERNEL);
    if (kbuff == NULL) {
        pr_err("%s Fallo en kmalloc\n", __MODULE_NAME__);
        mutex_unlock(&dev->tx_lock);
        return -ENOMEM;
    }

    if (copy_from_user(kbuff, buf, len)) {
        kfree(kbuff);
        mutex_unlock(&dev->tx_lock);
        return -EFAULT;
    }

    while (total_written < len) {
        // Enviar al buffer de tx; suspender en TASK_INTERRUPTIBLE si está lleno
        if (cb_push(&dev->tx_buf, kbuff[total_written]) == 0) {
            total_written++;
            continue;
        }

        // Asegurar IRQ de TX habilitada para que isr_tx_mcasp libere espacio
        mcasp_irq_tx_enable(dev, true);

        if (wait_event_interruptible(dev->tx_wq, cb_half_free(&dev->tx_buf))) {
            kfree(kbuff);
            mutex_unlock(&dev->tx_lock);
            return total_written > 0 ? total_written : -ERESTARTSYS;
        }
    }

    kfree(kbuff);

    // Asegurar interrupción XDATA de TX habilitada; el vaciado final se espera en close()
    if (total_written > 0)
        mcasp_irq_tx_enable(dev, true);

    mutex_unlock(&dev->tx_lock);
    return total_written;
}

static struct file_operations mymcasp_fops = {
    .owner = THIS_MODULE,
    .read = mymcasp_read,
    .write = mymcasp_write,
    .open = mymcasp_open,
    .release = mymcasp_close,
};


/////////////////////////
// Funciones de descubrimiento de HW
/////////////////////////

/**
 * @brief Inicializa el dispositivo de plataforma McASP.
 */
static int mymcasp_probe(struct platform_device *pdev)
{
    int status;
    mymcasp_handler_t *mymcasp_resource;
    struct resource *res;

    pr_info("%s Inicializando dispositivo (probe)\n", __MODULE_NAME__);

    // 1. Reservar memoria para la estructura privada.
    mymcasp_resource = devm_kzalloc(&pdev->dev, sizeof(*mymcasp_resource), GFP_KERNEL);
    if (!mymcasp_resource)
        return -ENOMEM;

    // 2. Inicializar campos, buffers circulares, colas de espera y mutexs.
    mymcasp_resource->pdev = pdev;
    cb_init(&mymcasp_resource->tx_buf, false);
    cb_init(&mymcasp_resource->rx_buf, true);
    init_waitqueue_head(&mymcasp_resource->tx_wq);
    init_waitqueue_head(&mymcasp_resource->rx_wq);
    mymcasp_resource->clk_enabled = false;
    mymcasp_resource->hw_users = 0;
    mymcasp_resource->dev_opened = false;
    mymcasp_resource->i2s_mode = SLAVE;
    mymcasp_resource->irq_tx = -1;
    mymcasp_resource->irq_rx = -1;
    mutex_init(&mymcasp_resource->tx_lock);
    mutex_init(&mymcasp_resource->rx_lock);
    mutex_init(&mymcasp_resource->hw_lock);

    // 3. OBTENER RECURSOS y MAPEAR MEMORIA
    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (!res) {
        pr_err("No se encontró el recurso de memoria en el DT\n");
        status = -ENODEV; // Abortar si no hay memoria
        goto err_mutex;
    }

    mymcasp_resource->instance = devm_ioremap_resource(&(pdev->dev), res);
    if (IS_ERR(mymcasp_resource->instance)) {
        pr_err("%s Error al mapear los recursos de memoria\n", __MODULE_NAME__);
        status = PTR_ERR(mymcasp_resource->instance);
        goto err_mutex;
    }

    // 4. Leer propiedades del Device Tree.
    if (pdev->dev.of_node) {
        status = of_property_read_u32(pdev->dev.of_node, "utn,frecuencia", &mymcasp_resource->hertz);
        if (status < 0) mymcasp_resource->hertz = FAULT_FREQ;
    } else {
        mymcasp_resource->hertz = FAULT_FREQ;
    }
#ifdef FREQ_TEST
    mymcasp_resource->hertz = FREQ_TEST; 
#endif

    // 5. Reservar el número de dispositivo.
    status = alloc_chrdev_region(
        &mymcasp_resource->mymcasp_cstm_cdev.mymcasp_dev,
        CDEV_MINORBASE,
        CDEV_NBR,
        __MODULE_NAME__
    );
    if (status < 0) {
        pr_err("%s Error al registrar la región del dispositivo: %d\n",
               __MODULE_NAME__, status);
        goto err_mutex;
    }

    // 6. Inicializar y registrar el character device.
    cdev_init(&mymcasp_resource->mymcasp_cstm_cdev.mymcasp_cdev, &mymcasp_fops);
    mymcasp_resource->mymcasp_cstm_cdev.mymcasp_cdev.owner = THIS_MODULE;

    status = cdev_add(
        &mymcasp_resource->mymcasp_cstm_cdev.mymcasp_cdev,
        mymcasp_resource->mymcasp_cstm_cdev.mymcasp_dev,
        CDEV_NBR
    );
    if (status < 0) {
        pr_err("%s Error al agregar el cdev: %d\n",
               __MODULE_NAME__, status);
        goto err_chrdev;
    }

    // 7. Crear el device asociado a la clase (/dev/td3_i2s).
    mymcasp_resource->mymcasp_device = device_create(
        &td3_i2s_class,
        &pdev->dev,
        mymcasp_resource->mymcasp_cstm_cdev.mymcasp_dev,
        mymcasp_resource,
        "%s",
        __MODULE_NAME__
    );
    if (IS_ERR(mymcasp_resource->mymcasp_device)) {
        status = PTR_ERR(mymcasp_resource->mymcasp_device);
        pr_err("%s Error al crear el dispositivo: %d\n",
               __MODULE_NAME__, status);
        mymcasp_resource->mymcasp_device = NULL;
        goto err_cdev;
    }

    // 8. Asociar los datos privados al dispositivo de plataforma.
    platform_set_drvdata(pdev, mymcasp_resource);

    // 9. Registrar las interrupciones de TX y RX del McASP.
    status = init_irq(mymcasp_resource);
    if (status < 0) {
        goto err_device;
    }

    // Fix para tener en los metodos de /Sysfs/class/I2S_td3 los datos de la instancia.
    // Nota: lo ideal seria que se cree dentro de esta clase una instancia nueva, no es compatible con el test.
    g_handler = mymcasp_resource;

    pr_info("%s Dispositivo registrado correctamente\n", __MODULE_NAME__);
    pr_info("%s Mayor: %d, Menor: %d\n",
            __MODULE_NAME__,
            MAJOR(mymcasp_resource->mymcasp_cstm_cdev.mymcasp_dev),
            MINOR(mymcasp_resource->mymcasp_cstm_cdev.mymcasp_dev));

    return 0;

err_device:
    platform_set_drvdata(pdev, NULL);
    device_destroy(&td3_i2s_class, mymcasp_resource->mymcasp_cstm_cdev.mymcasp_dev);
    mymcasp_resource->mymcasp_device = NULL;
err_cdev:
    cdev_del(&mymcasp_resource->mymcasp_cstm_cdev.mymcasp_cdev);
err_chrdev:
    unregister_chrdev_region(mymcasp_resource->mymcasp_cstm_cdev.mymcasp_dev, CDEV_NBR);
err_mutex:
    mutex_destroy(&mymcasp_resource->hw_lock);
    mutex_destroy(&mymcasp_resource->rx_lock);
    mutex_destroy(&mymcasp_resource->tx_lock);
    return status;
}


/**
 * @brief Desvincula el dispositivo de plataforma McASP, detiene el hardware si seguía
 *        activo y libera el dispositivo de caracteres, la región y los mutexes.
 */
static void mymcasp_remove(struct platform_device *pdev)
{
    mymcasp_handler_t *mymcasp_resource = platform_get_drvdata(pdev);

    if (!mymcasp_resource) {
        pr_warn("%s No se encontraron los datos del device\n",__MODULE_NAME__);
        return;
    }

    g_handler = NULL;

    // Si quedó abierto/encendido, detener hardware y apagar el reloj
    mutex_lock(&mymcasp_resource->hw_lock);
    if (mymcasp_resource->hw_users > 0)
        mymcasp_resource->hw_users = 1;
    mutex_unlock(&mymcasp_resource->hw_lock);
    mymcasp_hw_deinit(mymcasp_resource);

    // Destruir el dispositivo asociado a la clase.
    device_destroy(&td3_i2s_class, mymcasp_resource->mymcasp_cstm_cdev.mymcasp_dev);
    mymcasp_resource->mymcasp_device = NULL;

    // Desregistrar el character device.
    cdev_del(&mymcasp_resource->mymcasp_cstm_cdev.mymcasp_cdev);

    // Liberar los números de dispositivo.
    unregister_chrdev_region(mymcasp_resource->mymcasp_cstm_cdev.mymcasp_dev, CDEV_NBR);

    // Eliminar la asociación con los datos privados.
    platform_set_drvdata(pdev, NULL);

    // Destruir los mutexes luego de desregistrar todas las interfaces.
    mutex_destroy(&mymcasp_resource->hw_lock);
    mutex_destroy(&mymcasp_resource->rx_lock);
    mutex_destroy(&mymcasp_resource->tx_lock);

    pr_info("%s Dispositivo removido\n", __MODULE_NAME__);
}

// Nombre de match declarado en el Device Tree.
static const struct of_device_id mymcasp_of_match[] = {
    { .compatible = "utn,mcasp-device", },
    { },
};

// Se declara la tabla para que udev pueda cargar el driver automáticamente
MODULE_DEVICE_TABLE(of, mymcasp_of_match); 

// Esta es la estructura que define a mi platform driver.
static struct platform_driver mymcasp_driver = {
    .probe = mymcasp_probe,
    .remove = mymcasp_remove,
    .driver = {
        .name = "utn_mcasp_driver",
        .owner = THIS_MODULE,
        .of_match_table = of_match_ptr(mymcasp_of_match),  
    },
};


/**
 * @brief Función de inicialización del módulo: registra la clase sysfs td3_i2s
 *        y el platform driver asociado al McASP.
 * @return 0 en caso de éxito o código de error negativo si falla el registro.
 */
static int __init mymcasp_init(void){
    int status;

    pr_info("%s Registrando modulo\n", __MODULE_NAME__);

    // Registrar la clase (/sys/class/td3_i2s/ con sus atributos de clase).
    status = class_register(&td3_i2s_class);
    if (status < 0) {
        pr_err("%s Failed to register class: %d\n", __MODULE_NAME__, status);
        return status;
    }

    // Registrar el driver de plataforma.
    status = platform_driver_register(&mymcasp_driver);
    if (status < 0) {
        pr_err("%s Fallo el registro del platform driver: %d\n",__MODULE_NAME__, status);
        class_unregister(&td3_i2s_class);
        return status;
    }

    return 0;
}

/**
 * @brief Función de salida del módulo
 */
static void __exit mymcasp_exit(void){
    platform_driver_unregister(&mymcasp_driver);
    class_unregister(&td3_i2s_class);

    pr_info("%s Modulo sin registrar\n", __MODULE_NAME__);
}

module_init(mymcasp_init);
module_exit(mymcasp_exit);