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

/////////////////////////
// Handlers IRQ
/////////////////////////

// TODO Revisar registros, hacer clean 
static irqreturn_t isr_tx_mcasp(int irq, void *dev_id) {
    mymcasp_handler_t *handler = (mymcasp_handler_t *)dev_id;
    uint32_t xstat;
    uint32_t gbl;
    uint32_t word = 0;
    uint32_t reg_val;
    uint8_t byte;
    static uint32_t cont = 0;
    if (!handler || !handler->instance || !handler->clk_enabled)
        return IRQ_NONE;

    /*
     * Extraer la palabra desde el buffer circular de TX (tx_buf).
     * Si cb_pop devuelve -ENODATA (buffer vacío), deshabilitamos la interrupción
     * XDATA en XINTCTL para evitar un interrupt storm.
     */

    if (cb_pop(&handler->tx_buf, &byte) == -ENODATA) {
        reg_val = ioread32(handler->instance + MCASP_XINTCTL_REG);
        reg_val &= ~MCASP_INT_DATA;
        iowrite32(reg_val, handler->instance + MCASP_XINTCTL_REG);
        //TODO Remover:
        pr_info("Se reseteo la MDE %d veces durante el envio del buffer\n", cont);
        cont = 0;
    
        return IRQ_HANDLED;
    }

    // Formo word para enviar al buff del serializador

    word = ((uint32_t)byte << 24);
    if (cb_pop(&handler->tx_buf, &byte) == 0) word |= ((uint32_t)byte << 16);
    if (cb_pop(&handler->tx_buf, &byte) == 0) word |= ((uint32_t)byte << 8);
    if (cb_pop(&handler->tx_buf, &byte) == 0) word |= ((uint32_t)byte << 0);
   
    // Registros de estado de MDE y modulo MCASP
    xstat = ioread32(handler->instance + MCASP_XSTAT_REG);
    gbl   = ioread32(handler->instance + MCASP_GBLCTL_REG);

    /*
     * Si el transmisor entró en Underrun (bit 0 de XSTAT) mientras estaba inactivo
     * o si la máquina de estados TX (bit 11 XSMRST) aún no arrancó, el McASP exige
     * resetear XSMRST/XSRCLR, limpiar XSTAT, cargar XBUF0 y recién liberar XSMRST/XFRST.
     */
    if ((xstat & 0x1) || !(gbl & (1 << 11))) {
        // Debug
        cont++;

        reg_val = ioread32(handler->instance + MCASP_XGBLCTL_REG);
        reg_val &= ~((1 << 12) | (1 << 11) | (1 << 10));
        iowrite32(reg_val, handler->instance + MCASP_XGBLCTL_REG);
        udelay(5); // TODO Ver de remover delays

        // Limpiar todos los flags de estado/error de TX
        iowrite32(0xFFFF, handler->instance + MCASP_XSTAT_REG);

        // Activar serializer TX (XSRCLR = bit 10)
        reg_val |= (1 << 10);
        iowrite32(reg_val, handler->instance + MCASP_XGBLCTL_REG);
        mcasp_wait_gblctl(handler, (1 << 10));

        // Cargar la palabra inicial en XBUF0 ANTES de soltar XSMRST
        iowrite32(word, handler->instance + MCASP_XBUF0_REG);

        // Liberar máquina de estados TX (XSMRST = 11) y Frame Sync (XFRST = 12)
        reg_val |= (1 << 11);
        iowrite32(reg_val, handler->instance + MCASP_XGBLCTL_REG);
        mcasp_wait_gblctl(handler, (1 << 11));

        reg_val |= (1 << 12);
        iowrite32(reg_val, handler->instance + MCASP_XGBLCTL_REG);
        mcasp_wait_gblctl(handler, (1 << 12));

    } else {
        iowrite32(word, handler->instance + MCASP_XBUF0_REG);
    }

    wake_up_interruptible(&handler->tx_wq);
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

// TODO Revisar registros, hacer clean
static int set_i2s_mode(mymcasp_handler_t *handler, uint8_t mode) {
    uint32_t reg_val;

    if (!handler || !handler->instance)
        return -ENODEV;

    if (mode != SLAVE && mode != MASTER)
        return -EINVAL;

    // Si el clock no está activo, guardar el modo para aplicarlo en la inicialización sin acceder a registros
    if (!handler->clk_enabled) {
        handler->i2s_mode = mode;
        return 0;
    }

    if (mode == MASTER) {
        // CLKXM (bit 5) = 1: Clock de bits (ACLKX) generado internamente
        reg_val = ioread32(handler->instance + MCASP_ACLKXCTL_REG);
        reg_val |= (1 << 5);
        iowrite32(reg_val, handler->instance + MCASP_ACLKXCTL_REG);

        // FSXM (bit 1) = 1: Frame Sync / WS (AFSX) generado internamente
        reg_val = ioread32(handler->instance + MCASP_AFSXCTL_REG);
        reg_val |= (1 << 1);
        iowrite32(reg_val, handler->instance + MCASP_AFSXCTL_REG);

        // PDIR: Bits 28 (AFSX / WS) y 26 (ACLKX / CLK) como salidas (1)
        reg_val = ioread32(handler->instance + MCASP_PDIR_REG);
        reg_val |= ((1 << 28) | (1 << 26));
        iowrite32(reg_val, handler->instance + MCASP_PDIR_REG);

        // Asegurar que los divisores de clock, serializers, MDE y frame sync de TX y RX salgan de reset con el clock interno activo
        reg_val = ioread32(handler->instance + MCASP_XGBLCTL_REG);
        reg_val |= (1 << 8); // XCLKRST
        iowrite32(reg_val, handler->instance + MCASP_XGBLCTL_REG);
        mcasp_wait_gblctl(handler, (1 << 8));

        reg_val = ioread32(handler->instance + MCASP_RGBLCTL_REG);
        reg_val |= (1 << 0); // RCLKRST
        iowrite32(reg_val, handler->instance + MCASP_RGBLCTL_REG);
        mcasp_wait_gblctl(handler, (1 << 0));

        reg_val = ioread32(handler->instance + MCASP_XGBLCTL_REG);
        reg_val |= (1 << 10); // XSRCLR
        iowrite32(reg_val, handler->instance + MCASP_XGBLCTL_REG);
        mcasp_wait_gblctl(handler, (1 << 10));

        reg_val = ioread32(handler->instance + MCASP_RGBLCTL_REG);
        reg_val |= (1 << 2); // RSRCLR
        iowrite32(reg_val, handler->instance + MCASP_RGBLCTL_REG);
        mcasp_wait_gblctl(handler, (1 << 2));

        reg_val = ioread32(handler->instance + MCASP_XGBLCTL_REG);
        reg_val |= (1 << 11); // XSMRST
        iowrite32(reg_val, handler->instance + MCASP_XGBLCTL_REG);
        mcasp_wait_gblctl(handler, (1 << 11));

        reg_val = ioread32(handler->instance + MCASP_RGBLCTL_REG);
        reg_val |= (1 << 3); // RSMRST
        iowrite32(reg_val, handler->instance + MCASP_RGBLCTL_REG);
        mcasp_wait_gblctl(handler, (1 << 3));

        reg_val = ioread32(handler->instance + MCASP_XGBLCTL_REG);
        reg_val |= (1 << 12); // XFRST
        iowrite32(reg_val, handler->instance + MCASP_XGBLCTL_REG);
        mcasp_wait_gblctl(handler, (1 << 12));

        reg_val = ioread32(handler->instance + MCASP_RGBLCTL_REG);
        reg_val |= (1 << 4); // RFRST
        iowrite32(reg_val, handler->instance + MCASP_RGBLCTL_REG);
        mcasp_wait_gblctl(handler, (1 << 4));
    } else {
        // PDIR: Bits 28 (AFSX / WS) y 26 (ACLKX / CLK) como entradas (0 = alta impedancia)
        reg_val = ioread32(handler->instance + MCASP_PDIR_REG);
        reg_val &= ~((1 << 28) | (1 << 26));
        iowrite32(reg_val, handler->instance + MCASP_PDIR_REG);

        // CLKXM (bit 5) = 0: Clock de bits (ACLKX) tomado externamente
        reg_val = ioread32(handler->instance + MCASP_ACLKXCTL_REG);
        reg_val &= ~(1 << 5);
        iowrite32(reg_val, handler->instance + MCASP_ACLKXCTL_REG);

        // FSXM (bit 1) = 0: Frame Sync / WS (AFSX) tomado externamente
        reg_val = ioread32(handler->instance + MCASP_AFSXCTL_REG);
        reg_val &= ~(1 << 1);
        iowrite32(reg_val, handler->instance + MCASP_AFSXCTL_REG);
    }

    handler->i2s_mode = mode;
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

    // Seteo de pines en modo I2S: AFSX (28), ACLKX (26), AXR1 (1 - RX) y AXR0 (0 - TX)
    reg_val = ioread32(handler->instance + MCASP_PFUNC_REG);
    reg_val &= ~((1 << 28) | (1 << 26) | (1 << 1) | (1 << 0));
    iowrite32(reg_val, handler->instance + MCASP_PFUNC_REG);

    // Seteo de pin de datos TX (Bit 0: AXR0) como salida y RX (Bit 1: AXR1) como entrada.
    // Nota: Los pines de WS (Bit 28: AFSX) y CLK (Bit 26: ACLKX) los maneja set_i2s_mode().
    reg_val = ioread32(handler->instance + MCASP_PDIR_REG);
    reg_val |= (1 << 0);
    reg_val &= ~(1 << 1);
    iowrite32(reg_val, handler->instance + MCASP_PDIR_REG);
}

// TODO Revisar registros, hacer clean
static void mcasp_reg_config(mymcasp_handler_t *handler, bool enable){
    
    uint32_t target_freq = handler->hertz;
    uint32_t total_div;
    uint32_t ahclk_div;
    uint32_t aclk_div;
    uint32_t reg_val;

    if(target_freq == 0) target_freq = 1000; // evito division por cero
    
    total_div =  XTAL_FREQ / target_freq;

    if(total_div < 1) total_div = 1;

    if(total_div - 1 <= 4095){
        ahclk_div = total_div - 1;
        aclk_div = 0;
    }else{
        aclk_div= 31;
        if (total_div >= 32)
            ahclk_div = (total_div / 32) - 1;
        else
            ahclk_div = 0;

        if(ahclk_div > 4095) ahclk_div = 4095;
    }

    if(!enable){
        // Apago los clocks de TX (Bits 8 al 12) usando XGBLCTL
        reg_val = ioread32(handler->instance + MCASP_XGBLCTL_REG);
        reg_val &= ~(0x1F << 8);
        iowrite32(reg_val, handler->instance + MCASP_XGBLCTL_REG);

        // Apago los clocks de RX (Bits 0 al 4) usando RGBLCTL
        reg_val = ioread32(handler->instance + MCASP_RGBLCTL_REG);
        reg_val &= ~(0x1F); 
        iowrite32(reg_val, handler->instance + MCASP_RGBLCTL_REG);
        return;
    }

    // Por defecto iniciamos en modo SLAVE (WS y CLK como entradas externas)
    set_i2s_mode(handler, SLAVE);

    // Configurar divisores de reloj

    //HCLKXM (15) Internal transmit high-frequency clock source from output of programmable high clock divider.
    //HCLKXDIV (11-0) Transmit high-frequency clock divide ratio bits determine the dividedown ratio from AUXCLK to AHCLKX.
    reg_val = ioread32(handler->instance + MCASP_AHCLKXCTL_REG);
    reg_val &= ~(0xFFF | (1 << 15));
    reg_val |= (1 << 15)|(ahclk_div & 0xFFF);
    iowrite32(reg_val, handler->instance + MCASP_AHCLKXCTL_REG); 

    // Polaridad y divisor en Tx (sin tocar CLKXM bit 5, que lo maneja set_i2s_mode)
    //CLKXP (7) Transmit bitstream clock polarity select bit | 0h = Rising edge.
    //ASYNC (6) Transmit/receive operation asynchronous enable bit | 0h = Synchronous (RX usa el mismo CLK y WS que TX).
    //CLKXDIV (4-0) Transmit bit clock divide ratio bits determine the divide-down ratio from AHCLKX to ACLKX
    reg_val = ioread32(handler->instance + MCASP_ACLKXCTL_REG);
    reg_val &= ~((1 << 7) | (1 << 6) | 0x1F);
    reg_val |= (aclk_div & 0x1F);
    iowrite32(reg_val, handler->instance + MCASP_ACLKXCTL_REG); 

    /* Relojes de Alta Frecuencia */
    
    //XHCLKRST (9) Transmit high-frequency clock divider reset enable bit.
    reg_val = ioread32(handler->instance + MCASP_XGBLCTL_REG);
    reg_val |= (1 << 9); // XHCLKRST
    iowrite32(reg_val, handler->instance + MCASP_XGBLCTL_REG);
    mcasp_wait_gblctl(handler, (1 << 9));
    
    //RHCLKRST (1) Receive high-frequency clock divider reset enable bit (usando RGBLCTL para no pisar TX).
    reg_val = ioread32(handler->instance + MCASP_RGBLCTL_REG);
    reg_val |= (1 << 1); // RHCLKRST
    iowrite32(reg_val, handler->instance + MCASP_RGBLCTL_REG);
    mcasp_wait_gblctl(handler, (1 << 1));
    
    // Polaridad en Rx
    // CLKRP (7) Receive bitstream clock polarity select bit: 0h = Falling edge.
    reg_val = ioread32(handler->instance + MCASP_ACLKRCTL_REG);
    reg_val &= ~(1 << 7); 
    iowrite32(reg_val, handler->instance + MCASP_ACLKRCTL_REG);

    /* Clock divider */  

    //XCLKRST (8) Transmit clock divider reset enable bit.
    reg_val = ioread32(handler->instance + MCASP_XGBLCTL_REG);
    reg_val |= (1 << 8);
    iowrite32(reg_val, handler->instance + MCASP_XGBLCTL_REG);
    mcasp_wait_gblctl(handler, (1 << 8));

    //RCLKRST (0) Receive clock divider reset enable bit (usando RGBLCTL).
    reg_val = ioread32(handler->instance + MCASP_RGBLCTL_REG);
    reg_val |= (1 << 0);
    iowrite32(reg_val, handler->instance + MCASP_RGBLCTL_REG); 
    mcasp_wait_gblctl(handler, (1 << 0));

    /* Serializer */  

    // Configurar Serializer 0 (AXR0) como Transmisor (Bits 1:0 SRMOD = 01b -> 0x1)
    reg_val = ioread32(handler->instance + MCASP_SRCTL0_REG);
    reg_val &= ~0x3;
    reg_val |= 0x1;
    iowrite32(reg_val, handler->instance + MCASP_SRCTL0_REG);

    // Configurar Serializer 1 (AXR1) como Receptor (Bits 1:0 SRMOD = 10b -> 0x2)
    reg_val = ioread32(handler->instance + MCASP_SRCTL1_REG);
    reg_val &= ~0x3;
    reg_val |= 0x2;
    iowrite32(reg_val, handler->instance + MCASP_SRCTL1_REG);

    // Configurar máscaras de 32 bits (en reset arrancan en 0x00000000 y enmascaran todo)
    iowrite32(0xFFFFFFFF, handler->instance + MCASP_XMASK_REG);
    iowrite32(0xFFFFFFFF, handler->instance + MCASP_RMASK_REG);

    // Configurar formato de palabra TX y RX:
    // Bit 15 (XRVRS/RRVRS) = 1 (MSB primero)
    // Bits 7:4 (XSSZ/RSSZ) = 0xF (Slot de 32 bits)
    // Bit 3 (XBUSEL/RBUSEL) = 1 (Usar Configuration Bus para XBUF0/RBUF1 en 0x200/0x284)
    iowrite32((1 << 15) | (0xF << 4) | (1 << 3), handler->instance + MCASP_XFMT_REG);
    iowrite32((1 << 15) | (0xF << 4) | (1 << 3), handler->instance + MCASP_RFMT_REG);

    // Configurar Frame Sync I2S de 2 slots (XMOD/RMOD = 2 en bits 15:7) y ancho de 1 palabra (FSXDUR/FSRDUR = bit 4)
    reg_val = ioread32(handler->instance + MCASP_AFSXCTL_REG);
    reg_val &= ~((0x1FF << 7) | (1 << 4));
    reg_val |= (2 << 7) | (1 << 4);
    iowrite32(reg_val, handler->instance + MCASP_AFSXCTL_REG);

    reg_val = ioread32(handler->instance + MCASP_AFSRCTL_REG);
    reg_val &= ~((0x1FF << 7) | (1 << 4));
    reg_val |= (2 << 7) | (1 << 4);
    iowrite32(reg_val, handler->instance + MCASP_AFSRCTL_REG);

    // Habilitar los slots TDM 0 y 1 (canal izquierdo y derecho) en TX y RX
    iowrite32(0x00000003, handler->instance + MCASP_XTDM_REG);
    iowrite32(0x00000003, handler->instance + MCASP_RTDM_REG);

    // Limpiar registros de estado XSTAT y RSTAT
    iowrite32(0xFFFF, handler->instance + MCASP_XSTAT_REG);
    iowrite32(0xFFFF, handler->instance + MCASP_RSTAT_REG);

    //XSRCLR (10) Transmit serializer clear enable bit.
    reg_val = ioread32(handler->instance + MCASP_XGBLCTL_REG);
    reg_val |= (1 << 10);
    iowrite32(reg_val, handler->instance + MCASP_XGBLCTL_REG);
    mcasp_wait_gblctl(handler, (1 << 10));

    //RSRCLR (2) Receive serializer clear enable bit (usando RGBLCTL).
    reg_val = ioread32(handler->instance + MCASP_RGBLCTL_REG);
    reg_val |= (1 << 2);
    iowrite32(reg_val, handler->instance + MCASP_RGBLCTL_REG);
    mcasp_wait_gblctl(handler, (1 << 2));

    /* MDE */

    //XSMRST (11) Transmit state machine reset enable bit.
    reg_val = ioread32(handler->instance + MCASP_XGBLCTL_REG);
    reg_val |= (1 << 11);
    iowrite32(reg_val, handler->instance + MCASP_XGBLCTL_REG);
    mcasp_wait_gblctl(handler, (1 << 11));

    //RSMRST (3) Receive state machine reset enable bit (usando RGBLCTL).
    reg_val = ioread32(handler->instance + MCASP_RGBLCTL_REG);
    reg_val |= (1 << 3);
    iowrite32(reg_val, handler->instance + MCASP_RGBLCTL_REG);
    mcasp_wait_gblctl(handler, (1 << 3));

    /* Frame Sync */
    //XFRST (12) Transmit frame sync generator reset enable bit.
    reg_val = ioread32(handler->instance + MCASP_XGBLCTL_REG);
    reg_val |= (1 << 12);
    iowrite32(reg_val, handler->instance + MCASP_XGBLCTL_REG);
    mcasp_wait_gblctl(handler, (1 << 12));

    //RFRST (4) Receive frame sync generator reset enable bit (usando RGBLCTL).
    reg_val = ioread32(handler->instance + MCASP_RGBLCTL_REG);
    reg_val |= (1 << 4);
    iowrite32(reg_val, handler->instance + MCASP_RGBLCTL_REG);
    mcasp_wait_gblctl(handler, (1 << 4));
    
}

static void mymcasp_hw_deinit(mymcasp_handler_t *handler) {
    if (!handler)
        return;

    // Detener transferencias y apagar el reloj del periférico
    if (handler->clk_enabled) {
        // Deshabilitar interrupciones de TX y RX, vaciar los buffers de 32 bits de hardware RX (RBUF1) y TX (XBUF0) y limpiar estados
        if (handler->instance) {
            iowrite32(0, handler->instance + MCASP_XINTCTL_REG);
            iowrite32(0, handler->instance + MCASP_RINTCTL_REG);
            (void)MCASP_READ_RX_BUF(handler);
            iowrite32(0, handler->instance + MCASP_XBUF0_REG);
            iowrite32(0xFFFF, handler->instance + MCASP_RSTAT_REG);
            iowrite32(0xFFFF, handler->instance + MCASP_XSTAT_REG);
        }

        mcasp_reg_config(handler, false);
        if (handler->fck) {
            clk_disable_unprepare(handler->fck);
        }
        handler->clk_enabled = false;
    }

    cb_reset(&handler->tx_buf);
    cb_reset(&handler->rx_buf);
}

static int mymcasp_hw_init(mymcasp_handler_t *handler) {
    int status;
    uint8_t target_mode;

    if (!handler || !handler->instance)
        return -ENODEV;

    if (handler->clk_enabled)
        return 0;

    // Aplicar el modo configurado (SLAVE o MASTER); si es NOT_INIT, usar SLAVE por defecto
    if (handler->i2s_mode == MASTER || handler->i2s_mode == SLAVE)
        target_mode = handler->i2s_mode;
    else
        target_mode = SLAVE;

    // Clock Gating
    status = init_clock(handler);
    if (status)
        return status;

    // Config de pines (PFUNC y PDIR)
    init_gpio(handler);

    // Inicializa el McASP (arranca en SLAVE durante la configuración)
    mcasp_reg_config(handler, true);

    // Aplicar el modo correspondiente (SLAVE por default o MASTER)
    status = set_i2s_mode(handler, target_mode);
    if (status) {
        mymcasp_hw_deinit(handler);
        return status;
    }

    // Resetear índices de los buffers circulares de Tx y Rx
    cb_reset(&handler->tx_buf);
    cb_reset(&handler->rx_buf);

    // Mantener la IRQ de TX deshabilitada hasta que write() cargue datos en tx_buf
    iowrite32(0, handler->instance + MCASP_XINTCTL_REG);

    // Vaciar RBUF1, limpiar RSTAT y habilitar la interrupción de recepción (RDATA)
    (void)MCASP_READ_RX_BUF(handler);
    iowrite32(0xFFFF, handler->instance + MCASP_RSTAT_REG);
    iowrite32(MCASP_INT_DATA, handler->instance + MCASP_RINTCTL_REG);

    return 0;
}


/////////////////////////
// Interfaz Sysfs - (/sys/class/td3_i2s/)
/////////////////////////

static ssize_t master_show(const struct class *cls, const struct class_attribute *attr, char *buf) {
    mymcasp_handler_t *handler = g_handler;
    ssize_t ret;

    if (!handler || !handler->instance)
        return -ENODEV;

    if (!mutex_trylock(&handler->lock))
        return -EBUSY;

    if (handler->i2s_mode == NOT_INIT)
        ret = -ENODEV;
    else
        ret = sysfs_emit(buf, "%u\n", handler->i2s_mode);

    mutex_unlock(&handler->lock);
    return ret;
}

static ssize_t master_store(const struct class *cls, const struct class_attribute *attr, const char *buf, size_t count) {
    mymcasp_handler_t *handler = g_handler;
    char kbuf[16];
    char *arg;
    size_t len;
    uint8_t mode;
    int ret;

    if (!handler || !handler->instance)
        return -ENODEV;

    if (!mutex_trylock(&handler->lock))
        return -EBUSY;

    if (count == 0) {
        mutex_unlock(&handler->lock);
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
        mutex_unlock(&handler->lock);
        return -EINVAL;
    }

    ret = set_i2s_mode(handler, mode);
    mutex_unlock(&handler->lock);
    if (ret < 0)
        return ret;

    return count;
}

static CLASS_ATTR_RW(master);

static ssize_t recibir_read(struct file *filp, struct kobject *kobj,
                            struct bin_attribute *attr, char *buf,
                            loff_t off, size_t count) {
    mymcasp_handler_t *handler = g_handler;
    size_t read_bytes = 0;
    uint8_t byte;
    int ret;

    if (!handler || !handler->instance)
        return -ENODEV;

    if (!mutex_trylock(&handler->lock))
        return -EBUSY;

    ret = mymcasp_hw_init(handler);
    if (ret < 0) {
        mutex_unlock(&handler->lock);
        return ret;
    }

    if (count == 0) {
        mymcasp_hw_deinit(handler);
        mutex_unlock(&handler->lock);
        return 0;
    }

    if (count > BUFF_SIZE)
        count = BUFF_SIZE;

    while (read_bytes < count && cb_pop(&handler->rx_buf, &byte) == 0) {
        buf[read_bytes++] = (char)byte;
    }

    mymcasp_hw_deinit(handler);
    mutex_unlock(&handler->lock);
    return read_bytes;
}

static BIN_ATTR_RO(recibir, 0);

static ssize_t transmitir_write(struct file *filp, struct kobject *kobj,
                                struct bin_attribute *attr, char *buf,
                                loff_t off, size_t count) {
    mymcasp_handler_t *handler = g_handler;
    size_t written = 0;
    uint32_t reg_val;
    int ret;

    if (!handler || !handler->instance)
        return -ENODEV;

    if (!mutex_trylock(&handler->lock))
        return -EBUSY;

    ret = mymcasp_hw_init(handler);
    if (ret < 0) {
        mutex_unlock(&handler->lock);
        return ret;
    }

    if (count == 0) {
        mymcasp_hw_deinit(handler);
        mutex_unlock(&handler->lock);
        return 0;
    }

    while (written < count) {
        if (cb_push(&handler->tx_buf, (uint8_t)buf[written]) != 0)
            break;
        written++;
    }

    if (written == 0) {
        mymcasp_hw_deinit(handler);
        mutex_unlock(&handler->lock);
        return -ENOSPC;
    }

    // Habilito la interrupcion del Tx
    reg_val = ioread32(handler->instance + MCASP_XINTCTL_REG);
    reg_val |= MCASP_INT_DATA;
    iowrite32(reg_val, handler->instance + MCASP_XINTCTL_REG);

    mymcasp_hw_deinit(handler);
    mutex_unlock(&handler->lock);
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


static int mymcasp_open(struct inode *inode, struct file *file) {
    int status;
    
    mymcasp_handler_t *handler = container_of(inode->i_cdev, mymcasp_handler_t, mymcasp_cstm_cdev.mymcasp_cdev);

    // Exclusión mutua: si el dispositivo ya está abierto por otro proceso, retornar -EBUSY
    if (!mutex_trylock(&handler->lock)) {
        pr_warn("%s Dispositivo en uso\n", __MODULE_NAME__);
        return -EBUSY;
    }

    status = mymcasp_hw_init(handler);
    if (status) {
        mutex_unlock(&handler->lock);
        return status;
    }

    // TODO: REMOVER -- (Hardcodeo que arranque en master para prueba de loopback sin depender de sysfs)
    set_i2s_mode(handler, MASTER);

    file->private_data = handler;

    pr_info("%s Dispositivo Abierto\n", __MODULE_NAME__);
    return 0;
}

static int mymcasp_close(struct inode *inode, struct file *file) {
    mymcasp_handler_t *handler = file->private_data;

    if (handler) {
        mymcasp_hw_deinit(handler);
        mutex_unlock(&handler->lock);
    }

    file->private_data = NULL;
    pr_info("%s Dispositivo Abierto\n", __MODULE_NAME__);
    return 0;
}

static ssize_t mymcasp_read(struct file *file, char __user *buf, size_t len, loff_t *offset) {
    mymcasp_handler_t *dev = file->private_data;
    ssize_t ret = 0;
    size_t read_bytes = 0;
    uint8_t *kbuff;
    uint8_t byte;

    pr_info("%s Lectura de dispositivo\n", __MODULE_NAME__);

    if (!dev || !dev->instance)
        return -ENODEV;

    if (len == 0)
        return 0;

    if (len > BUFF_SIZE)
        len = BUFF_SIZE;

    kbuff = kmalloc(len, GFP_KERNEL);
    if (kbuff == NULL) {
        pr_err("%s Fallo en Kmalloc\n", __MODULE_NAME__);
        return -ENOMEM;
    }

    while (read_bytes == 0) {
        if (wait_event_interruptible(dev->rx_wq, !cb_is_empty(&dev->rx_buf))) {
            kfree(kbuff);
            return -ERESTARTSYS;
        }

        while (read_bytes < len && cb_pop(&dev->rx_buf, &byte) == 0) {
            kbuff[read_bytes++] = byte;
        }
    }

    if (copy_to_user(buf, kbuff, read_bytes)) {
        ret = -EFAULT;
    } else {
        ret = read_bytes;
    }

    kfree(kbuff);
    return ret;
}


static ssize_t mymcasp_write(struct file *file, const char __user *buf, size_t len, loff_t *offset) {
    mymcasp_handler_t *dev = file->private_data;
    size_t written = 0;
    uint8_t *kbuff;
    uint32_t reg_val;

    if (!dev || !dev->instance)
        return -ENODEV;

    if (len == 0)
        return 0;

    if (len > BUFF_SIZE)
        len = BUFF_SIZE;

    kbuff = kmalloc(len, GFP_KERNEL);
    if (kbuff == NULL) {
        pr_err("%s Fallo en kmalloc\n", __MODULE_NAME__);
        return -ENOMEM;
    }

    if (copy_from_user(kbuff, buf, len)) {
        kfree(kbuff);
        return -EFAULT;
    }

    // Enviar al buffer de tx; suspender en TASK_INTERRUPTIBLE si está lleno
    while (written < len) {
        if (cb_push(&dev->tx_buf, kbuff[written]) == 0) {
            written++;
            continue;
        }

        // Asegurar IRQ de TX habilitada para que isr_tx_mcasp libere espacio
        reg_val = ioread32(dev->instance + MCASP_XINTCTL_REG);
        reg_val |= MCASP_INT_DATA;
        iowrite32(reg_val, dev->instance + MCASP_XINTCTL_REG);

        if (wait_event_interruptible(dev->tx_wq, !cb_is_full(&dev->tx_buf))) {
            kfree(kbuff);
            return written > 0 ? written : -ERESTARTSYS;
        }
    }

    kfree(kbuff);

    // Si se cargaron datos, habilitar la interrupción XDATA de TX para que isr_tx_mcasp los consuma
    if (written > 0) {
        reg_val = ioread32(dev->instance + MCASP_XINTCTL_REG);
        reg_val |= MCASP_INT_DATA;
        iowrite32(reg_val, dev->instance + MCASP_XINTCTL_REG);
    }

    pr_info("%s Escritura en el dispositivo (%zu bytes)\n", __MODULE_NAME__, written);

    return written;
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

// TODO Revisar bien es un quilombo
static int mymcasp_probe(struct platform_device *pdev)
{
    int status;
    mymcasp_handler_t *mymcasp_resource;
    struct resource *res;

    pr_info("%s Probing device\n", __MODULE_NAME__);

    
    // Reservar memoria para la estructura privada.
    mymcasp_resource = devm_kzalloc(&pdev->dev, sizeof(*mymcasp_resource), GFP_KERNEL);
    if (!mymcasp_resource)
        return -ENOMEM;

    mymcasp_resource->pdev = pdev;
    cb_init(&mymcasp_resource->tx_buf);
    cb_init(&mymcasp_resource->rx_buf);
    init_waitqueue_head(&mymcasp_resource->tx_wq);
    init_waitqueue_head(&mymcasp_resource->rx_wq);
    mymcasp_resource->clk_enabled = false;
    mymcasp_resource->i2s_mode = NOT_INIT;
    mutex_init(&mymcasp_resource->lock);
    
    // OBTENER RECURSOS
    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (!res) {
        pr_err("No se encontró el recurso de memoria en el DT\n");
        return -ENODEV; // Abortar si no hay memoria
    }

    // MAPEAR MEMORIA
    mymcasp_resource->instance = devm_ioremap_resource(&(pdev->dev), res);
    if (IS_ERR(mymcasp_resource->instance)) {
        pr_err("%s:Error mapping resources\n", __MODULE_NAME__);     
        return PTR_ERR(mymcasp_resource->instance);
    }
    
    if (pdev->dev.of_node) {
        status = of_property_read_u32(pdev->dev.of_node, "utn,frecuencia", &mymcasp_resource->hertz);
        // mymcasp_resource->hertz = 10000; // TODO BORRAR
        if (status < 0) mymcasp_resource->hertz = 1000;
        
        status = of_property_read_u32(pdev->dev.of_node, "square_on", &mymcasp_resource->square_on);
        if (status < 0) mymcasp_resource->square_on = 0;
    } else {
        mymcasp_resource->hertz = 1000;
        mymcasp_resource->square_on = 0;
    }

    // Inicializar el character device.
    cdev_init(&mymcasp_resource->mymcasp_cstm_cdev.mymcasp_cdev,&mymcasp_fops);

    mymcasp_resource->mymcasp_cstm_cdev.mymcasp_cdev.owner = THIS_MODULE;

    // Reservar el número de dispositivo.
    status = alloc_chrdev_region(
        &mymcasp_resource->mymcasp_cstm_cdev.mymcasp_dev,
        CDEV_MINORBASE,
        CDEV_NBR,
        __MODULE_NAME__
    );

    if (status < 0) {
        pr_err("%s Failed to register device region: %d\n",
               __MODULE_NAME__, status);
        return status;
    }

    // Registrar el character device.
    status = cdev_add(
        &mymcasp_resource->mymcasp_cstm_cdev.mymcasp_cdev,
        mymcasp_resource->mymcasp_cstm_cdev.mymcasp_dev,
        CDEV_NBR
    );

    if (status < 0) {
        pr_err("%s Failed to add cdev: %d\n",
               __MODULE_NAME__, status);

        unregister_chrdev_region(
            mymcasp_resource->mymcasp_cstm_cdev.mymcasp_dev,
            CDEV_NBR
        );

        return status;
    }

    // Crear el device asociado a nuestra clase (/dev/td3_i2s).
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

        pr_err("%s Failed to create device: %d\n",
               __MODULE_NAME__, status);

        mymcasp_resource->mymcasp_device = NULL;

        cdev_del(&mymcasp_resource->mymcasp_cstm_cdev.mymcasp_cdev);

        unregister_chrdev_region(
            mymcasp_resource->mymcasp_cstm_cdev.mymcasp_dev,
            CDEV_NBR
        );

        return status;
    }

    // Asociar los datos privados al dispositivo de plataforma.
    platform_set_drvdata(pdev, mymcasp_resource);

    // Registrar las interrupciones de TX y RX del McASP
    status = init_irq(mymcasp_resource);
    if (status < 0) {
        platform_set_drvdata(pdev, NULL);
        device_destroy(&td3_i2s_class, mymcasp_resource->mymcasp_cstm_cdev.mymcasp_dev);
        mymcasp_resource->mymcasp_device = NULL;
        cdev_del(&mymcasp_resource->mymcasp_cstm_cdev.mymcasp_cdev);
        unregister_chrdev_region(mymcasp_resource->mymcasp_cstm_cdev.mymcasp_dev, CDEV_NBR);
        return status;
    }

    g_handler = mymcasp_resource;

    pr_info("%s Device registered successfully\n", __MODULE_NAME__);

    pr_info("%s Major: %d, Minor: %d\n",
            __MODULE_NAME__,
            MAJOR(mymcasp_resource->mymcasp_cstm_cdev.mymcasp_dev),
            MINOR(mymcasp_resource->mymcasp_cstm_cdev.mymcasp_dev));
    
    return 0;
}


static void mymcasp_remove(struct platform_device *pdev)
{
    mymcasp_handler_t *mymcasp_resource = platform_get_drvdata(pdev);

    if (!mymcasp_resource) {
        pr_warn("%s No se encontraron los datos del device\n",__MODULE_NAME__);
        return;
    }

    g_handler = NULL;

    // Si quedó abierto/encendido, detener hardware y apagar el reloj
    mymcasp_hw_deinit(mymcasp_resource);

    // Liberar el mutex si quedó tomado por un cierre pendiente
    if (mutex_is_locked(&mymcasp_resource->lock)) {
        mutex_unlock(&mymcasp_resource->lock);
    }
    mutex_destroy(&mymcasp_resource->lock);

    // Destruir el dispositivo asociado a la clase.
    device_destroy(&td3_i2s_class, mymcasp_resource->mymcasp_cstm_cdev.mymcasp_dev);

    mymcasp_resource->mymcasp_device = NULL;

    // Desregistrar el character device.
    cdev_del(&mymcasp_resource->mymcasp_cstm_cdev.mymcasp_cdev);

    // Liberar los números de dispositivo.
    unregister_chrdev_region(mymcasp_resource->mymcasp_cstm_cdev.mymcasp_dev, CDEV_NBR);

    // Eliminar la asociación con los datos privados.
    platform_set_drvdata(pdev, NULL);

    pr_info("%s Device removed\n", __MODULE_NAME__);
}

// nombre que aparece en DT.
static const struct of_device_id mymcasp_of_match[] = {
    { .compatible = "utn,cin-device", },
    { },
};

// Se declara la tabla para que udev pueda cargar el driver automáticamente
MODULE_DEVICE_TABLE(of, mymcasp_of_match); 

// Esta es la estructura que define a mi platform driver.
static struct platform_driver mymcasp_driver = {
    .probe = mymcasp_probe,
    .remove = mymcasp_remove,
    .driver = {
        .name = "cin_platform_driver",
        .owner = THIS_MODULE,
        .of_match_table = of_match_ptr(mymcasp_of_match),  
    },
};


static int __init mymcasp_init(void){
    int status;

    pr_info("%s Registering module\n", __MODULE_NAME__);

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

static void __exit mymcasp_exit(void){
    // Esto provoca remove() para las instancias que sigan asociadas.
    platform_driver_unregister(&mymcasp_driver);

    // Desregistrar la clase cuando ya no quedan dispositivos asociados.
    class_unregister(&td3_i2s_class);

    pr_info("%s Modulo sin registrar\n", __MODULE_NAME__);
}

module_init(mymcasp_init);
module_exit(mymcasp_exit);