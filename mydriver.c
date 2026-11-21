#include "mydriver.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Guido");
MODULE_DESCRIPTION("Un driver de ejemplo");


// TODO
/*

* Funcionamiento
    - Realizar el manejo de buffers del driver.
    - Probar el envio de datos por el pin de datos.

* Interfaz 
    - master (Lectura / Escritura): 
        - Escritura: Permite conmutar el modo de sincronismo de reloj (1 = Maestro, 0 = Esclavo). 
        - Lectura: Retorna el estado operativo actual (1 o 0).
    - recibir (Lectura): 
        - Retorna últimos 4k leídos. Implementar un búfer circular interno de recepción con  tamaño superior a 4k. 
    - transmitir (Escritura): 
        - Permite escribir datos directamente a la cola de transmisión para su envío por la interfaz  I2S. 

* Formalidades y miscelanea 
    - Cambiar nombre del driver a td3_i2s.
    - Impedir el open mientras exista una tarea usando el device. (EBUSY)
    - Al cerrarse el descriptor del proceso activo, el hardware debe detenerse, suspender sus relojes y quedar habilitado para una posterior apertura
    - Iniciar el device en modo slave.

* Interrupciones
    - Chequear su configuracion.
    - Probar si estan activas (con un print).
    - Colocar al proceso consumidor como TASK_INTERRUPTIBLE.


*/
static const char __MODULE_NAME__[] = "cin"; // No poner nombres raros por que no carga el driver sino. (todo junto sin caracter especiales)

static struct class *cinp_class; // Necesario para crear el nodo en /dev

/////////////////////////
// configuracion de HW
/////////////////////////

static void mcasp_wait_gblctl(cinp_handler_t *handler, uint32_t mask) {
    int timeout = 1000;
    while (((ioread32(handler->instance + MCASP_GBLCTL_REG) & mask) != mask) && --timeout > 0) {
        udelay(1);
    }
}

static void mcasp_write_tx_word(cinp_handler_t *handler, uint32_t word) {
    uint32_t xstat = ioread32(handler->instance + MCASP_XSTAT_REG);
    uint32_t gbl   = ioread32(handler->instance + MCASP_GBLCTL_REG);

    /*
     * Si el transmisor entró en Underrun (bit 0 de XSTAT) mientras estaba inactivo
     * o si la máquina de estados TX (bit 11 XSMRST) aún no arrancó, el McASP exige
     * resetear XSMRST/XSRCLR, limpiar XSTAT, cargar XBUF0 y recién liberar XSMRST/XFRST.
     */
    if ((xstat & 0x1) || !(gbl & (1 << 11))) {
        uint32_t reg_val = ioread32(handler->instance + MCASP_XGBLCTL_REG);
        reg_val &= ~((1 << 12) | (1 << 11) | (1 << 10));
        iowrite32(reg_val, handler->instance + MCASP_XGBLCTL_REG);
        udelay(5);

        // Limpiar todos los flags de estado/error de TX
        iowrite32(0xFFFF, handler->instance + MCASP_XSTAT_REG);

        // 1) Activar serializer TX (XSRCLR = bit 10)
        reg_val |= (1 << 10);
        iowrite32(reg_val, handler->instance + MCASP_XGBLCTL_REG);
        mcasp_wait_gblctl(handler, (1 << 10));

        // 2) Cargar la palabra inicial en XBUF0 ANTES de soltar XSMRST
        iowrite32(word, handler->instance + MCASP_XBUF0_REG);

        // 3) Liberar máquina de estados TX (XSMRST = 11) y Frame Sync (XFRST = 12)
        reg_val |= (1 << 11);
        iowrite32(reg_val, handler->instance + MCASP_XGBLCTL_REG);
        mcasp_wait_gblctl(handler, (1 << 11));

        reg_val |= (1 << 12);
        iowrite32(reg_val, handler->instance + MCASP_XGBLCTL_REG);
        mcasp_wait_gblctl(handler, (1 << 12));
    } else {
        iowrite32(word, handler->instance + MCASP_XBUF0_REG);
    }
}

static void I2S_mode(cinp_handler_t *handler, uint8_t mode) {
    uint32_t reg_val;

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
}

static int Init_clock(cinp_handler_t *handler) {
    int ret;

    if (!handler->fck) {
        handler->fck = devm_clk_get(&handler->pdev->dev, "fck");
        if (IS_ERR(handler->fck)) {
            ret = PTR_ERR(handler->fck);
            dev_err(&handler->pdev->dev, "No se pudo obtener el reloj fck: %d\n", ret);
            handler->fck = NULL;
            return ret;
        }
    }

    if (!handler->clk_enabled) {
        ret = clk_prepare_enable(handler->fck);
        if (ret) {
            dev_err(&handler->pdev->dev, "Error al habilitar el reloj fck: %d\n", ret);
            return ret;
        }
        handler->clk_enabled = true;
    }

    return 0;
}

static void Init_GPIO(cinp_handler_t *handler) {
    uint32_t reg_val;

    if (!handler->instance)
        return;

    // Seteo de pines en modo I2S: AFSX (28), ACLKX (26), AXR1 (1 - RX) y AXR0 (0 - TX)
    reg_val = ioread32(handler->instance + MCASP_PFUNC_REG);
    reg_val &= ~((1 << 28) | (1 << 26) | (1 << 1) | (1 << 0));
    iowrite32(reg_val, handler->instance + MCASP_PFUNC_REG);

    // Seteo de pin de datos TX (Bit 0: AXR0) como salida y RX (Bit 1: AXR1) como entrada.
    // Los pines de WS (Bit 28: AFSX) y CLK (Bit 26: ACLKX) los maneja I2S_mode().
    reg_val = ioread32(handler->instance + MCASP_PDIR_REG);
    reg_val |= (1 << 0);
    reg_val &= ~(1 << 1);
    iowrite32(reg_val, handler->instance + MCASP_PDIR_REG);
}

static void mcasp_reg_config(cinp_handler_t *handler, bool enable){
    
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
    I2S_mode(handler, SLAVE);

    // Configurar divisores de reloj

    //HCLKXM (15) Internal transmit high-frequency clock source from output of programmable high clock divider.
    //HCLKXDIV (11-0) Transmit high-frequency clock divide ratio bits determine the dividedown ratio from AUXCLK to AHCLKX.
    reg_val = ioread32(handler->instance + MCASP_AHCLKXCTL_REG);
    reg_val &= ~(0xFFF | (1 << 15));
    reg_val |= (1 << 15)|(ahclk_div & 0xFFF);
    iowrite32(reg_val, handler->instance + MCASP_AHCLKXCTL_REG); 

    // Polaridad y divisor en Tx (sin tocar CLKXM bit 5, que lo maneja I2S_mode)
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


// Funciones para manejar los atributos de sysfs

static ssize_t lectura_show(struct device *dev, struct device_attribute *attr, char *buf) {
    cinp_handler_t *handler = dev_get_drvdata(dev);

    if(!handler)
        return -ENODEV;

    return sysfs_emit(buf, "%.*s\n", (int)handler->buffer_len, handler->buffer);
}

static DEVICE_ATTR_RO(lectura); 

static ssize_t escritura_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {
    cinp_handler_t *handler = dev_get_drvdata(dev);
    size_t length;

    if(!handler)
        return -ENODEV;

    length = (count < BUFF_SIZE) ? count : BUFF_SIZE;

    memcpy(handler->buffer, buf, length);
    handler->buffer_len = length;

    return count;
}

static DEVICE_ATTR_WO(escritura);
// static DEVICE_ATTR_(escritura, 0220, NULL, escritura_store); // Alternativa para definir permisos

static int cinp_open(struct inode *inode, struct file *file) {
    int status;
    // Recuperamos nuestra estructura a través del cdev contenido en el inode
    cinp_handler_t *handler = container_of(inode->i_cdev, cinp_handler_t, cinp_cstm_cdev.cinp_cdev);

    // Exclusión mutua: si el dispositivo ya está abierto por otro proceso, retornar -EBUSY
    if (!mutex_trylock(&handler->lock)) {
        pr_warn("%s Device busy\n", __MODULE_NAME__);
        return -EBUSY;
    }

    file->private_data = handler;

    // ENCENDER EL RELOJ
    status = Init_clock(handler);
    if (status) {
        mutex_unlock(&handler->lock);
        return status;
    }

    // CONFIGURAR PINES DEL MCASP (PFUNC y PDIR)
    Init_GPIO(handler);

    // Inicializa el McASP (default SLAVE)
    mcasp_reg_config(handler, true);

    // TODO: REMOVER -- (Hardcodeo que arranque en master)
    I2S_mode(handler, MASTER);

    // TODO: REMOVER -- Prueba de transmisión en el open: enviar palabra 0xAAFFAAFF por P9_30 (AXR0)
    mcasp_write_tx_word(handler, 0xAAFFAAFF);

    handler->buffer_head = 0; 

    pr_info("%s Device opened\n", __MODULE_NAME__);
    return 0;
}

static int cinp_close(struct inode *inode, struct file *file) {
    cinp_handler_t *handler = file->private_data;

    if (handler) {
        // Detener transferencias y apagar el reloj del periférico
        if (handler->clk_enabled) {
            // Vaciar los buffers de 32 bits de hardware RX (RBUF1) y TX (XBUF0) y limpiar estados
            if (handler->instance) {
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
        handler->buffer_head = 0;
        mutex_unlock(&handler->lock);
    }

    file->private_data = NULL;
    pr_info("%s Device closed\n", __MODULE_NAME__);
    return 0;
}

static ssize_t cinp_read(struct file *file, char __user *buf, size_t len, loff_t *offset) {
    
    pr_info("%s Device read\n", __MODULE_NAME__);

    cinp_handler_t *dev = file->private_data;

    ssize_t ret = 0;
    void *kbuff;

    if(len > dev->buffer_head) {
        len = dev->buffer_head; // Limitar len al tamaño del buffer
    }

    kbuff = kmalloc(len, GFP_KERNEL);
    if (kbuff == NULL) {
        pr_err("%s Failed to allocate memory\n", __MODULE_NAME__);
        return -ENOMEM;
    }

    memcpy(kbuff, &dev->buffer[0], len); // Copiar datos al buffer temporal

    if (copy_to_user(buf, kbuff, len)) {
        ret = -EFAULT;
    } else {
        *offset += len; // Actualizamos la posición real de lectura
        ret = len;
    }

    kfree(kbuff);
    
    return ret;

}



static ssize_t cinp_write(struct file *file, const char __user *buf, size_t len, loff_t *offset) {

    cinp_handler_t *dev = file->private_data;
    ssize_t ret = 0;
    void *kbuff;

    // Si head != 0, todavía hay datos sin vaciar/enviar en el buffer: retornar 0 bytes copiados
    if (dev->buffer_head != 0) {
        return 0;
    }

    if (len > dev->buffer_len) {
        len = dev->buffer_len; // Limitar len al tamaño del buffer
    }

    if (len == 0)
        return 0;

    kbuff = kmalloc(len, GFP_KERNEL);
    if (kbuff == NULL) {
        pr_err("%s Failed to allocate memory\n", __MODULE_NAME__);
        return -ENOMEM;
    }

    if (copy_from_user(kbuff, buf, len)) {
        ret = -EFAULT;
    } else {
        memcpy(dev->buffer, kbuff, len); // Copiar datos del buffer temporal al buffer del dispositivo
        dev->buffer_head = len;
        ret = len;
    }
    kfree(kbuff);

    pr_info("%s Device write\n", __MODULE_NAME__);

    return ret;
}


// Este es el puente que tenemos entre el driver nuestro y el kernel cuando hace los llamados que requiere el user
static struct file_operations cinp_fops = {
    .owner = THIS_MODULE,
    .read = cinp_read,
    .write = cinp_write,
    .open = cinp_open,
    .release = cinp_close,
};

static char *cinp_devnode(const struct device *dev, umode_t *mode) {
    // Esto se usa si queremos usar el /dev, como lo hacemos solo en el /sys no es necesario.
    if (mode) {
        *mode = 0666; // Permisos de lectura y escritura para todos
    }
    return NULL;
}


static int cinp_probe(struct platform_device *pdev)
{
    int status;
    cinp_handler_t *cinp_resource;
    struct resource *res;

    pr_info("%s Probing device\n", __MODULE_NAME__);

    
    // Reservar memoria para la estructura privada.
    cinp_resource = devm_kzalloc(&pdev->dev, sizeof(*cinp_resource), GFP_KERNEL);
    if (!cinp_resource)
        return -ENOMEM;

    cinp_resource->pdev = pdev;
    cinp_resource->buffer_len = BUFF_SIZE;
    cinp_resource->clk_enabled = false;
    mutex_init(&cinp_resource->lock);
    
    // OBTENER RECURSOS
    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (!res) {
        dev_err(&pdev->dev, "No se encontró el recurso de memoria en el DT\n");
        return -ENODEV; // Abortar si no hay memoria
    }

    // MAPEAR MEMORIA
    cinp_resource->instance = devm_ioremap_resource(&(pdev->dev), res);
    if (IS_ERR(cinp_resource->instance)) {
        pr_err("%s:Error mapping resources\n", __MODULE_NAME__);     
        return PTR_ERR(cinp_resource->instance);
    }
    
    if (pdev->dev.of_node) {
        status = of_property_read_u32(pdev->dev.of_node, "utn,frecuencia", &cinp_resource->hertz);
        if (status < 0) cinp_resource->hertz = 1000;
        
        status = of_property_read_u32(pdev->dev.of_node, "square_on", &cinp_resource->square_on);
        if (status < 0) cinp_resource->square_on = 0;
    } else {
        cinp_resource->hertz = 1000;
        cinp_resource->square_on = 0;
    }

    // Inicializar el character device.
    cdev_init(&cinp_resource->cinp_cstm_cdev.cinp_cdev,&cinp_fops);

    cinp_resource->cinp_cstm_cdev.cinp_cdev.owner = THIS_MODULE;

    // Reservar el número de dispositivo.
    status = alloc_chrdev_region(
        &cinp_resource->cinp_cstm_cdev.cinp_dev,
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
        &cinp_resource->cinp_cstm_cdev.cinp_cdev,
        cinp_resource->cinp_cstm_cdev.cinp_dev,
        CDEV_NBR
    );

    if (status < 0) {
        pr_err("%s Failed to add cdev: %d\n",
               __MODULE_NAME__, status);

        unregister_chrdev_region(
            cinp_resource->cinp_cstm_cdev.cinp_dev,
            CDEV_NBR
        );

        return status;
    }

    // Crear el device asociado a nuestra clase.
    cinp_resource->cinp_device = device_create(
        cinp_class,
        &pdev->dev,
        cinp_resource->cinp_cstm_cdev.cinp_dev,
        NULL,
        "cinp"
    );

    if (IS_ERR(cinp_resource->cinp_device)) {
        status = PTR_ERR(cinp_resource->cinp_device);

        pr_err("%s Failed to create device: %d\n",
               __MODULE_NAME__, status);

        cinp_resource->cinp_device = NULL;

        cdev_del(&cinp_resource->cinp_cstm_cdev.cinp_cdev);

        unregister_chrdev_region(
            cinp_resource->cinp_cstm_cdev.cinp_dev,
            CDEV_NBR
        );

        return status;
    }

    // Asociar los datos privados al dispositivo de plataforma.
    // Los callbacks de sysfs necesitan recuperarlos.
    platform_set_drvdata(pdev, cinp_resource);

    // Crear el atributo de lectura.
    status = device_create_file(&pdev->dev, &dev_attr_lectura);

    if (status < 0) {
        pr_err("%s Failed to create lectura attribute: %d\n",__MODULE_NAME__, status);

        platform_set_drvdata(pdev, NULL);

        device_destroy(cinp_class,cinp_resource->cinp_cstm_cdev.cinp_dev);

        cinp_resource->cinp_device = NULL;

        cdev_del(&cinp_resource->cinp_cstm_cdev.cinp_cdev);

        unregister_chrdev_region(
            cinp_resource->cinp_cstm_cdev.cinp_dev,
            CDEV_NBR
        );

        return status;
    }

    // Crear el atributo de escritura.
    status = device_create_file(&pdev->dev, &dev_attr_escritura);

    if (status < 0) {
        pr_err("%s Failed to create escritura attribute: %d\n",
               __MODULE_NAME__, status);

        // lectura se creó correctamente; escritura no.
        device_remove_file(&pdev->dev, &dev_attr_lectura);

        platform_set_drvdata(pdev, NULL);

        device_destroy(
            cinp_class,
            cinp_resource->cinp_cstm_cdev.cinp_dev
        );

        cinp_resource->cinp_device = NULL;

        cdev_del(&cinp_resource->cinp_cstm_cdev.cinp_cdev);

        unregister_chrdev_region(
            cinp_resource->cinp_cstm_cdev.cinp_dev,
            CDEV_NBR
        );

        return status;
    }

    pr_info("%s Device registered successfully\n", __MODULE_NAME__);

    pr_info("%s Major: %d, Minor: %d\n",
            __MODULE_NAME__,
            MAJOR(cinp_resource->cinp_cstm_cdev.cinp_dev),
            MINOR(cinp_resource->cinp_cstm_cdev.cinp_dev));
    
    return 0;
}
static void cinp_remove(struct platform_device *pdev)
{
    cinp_handler_t *cinp_resource = platform_get_drvdata(pdev);

    if (!cinp_resource) {
        pr_warn("%s No se encontraron los datos del device\n",__MODULE_NAME__);
        return;
    }

    // Si quedó abierto/encendido, detener hardware y apagar el reloj
    if (cinp_resource->clk_enabled) {
        mcasp_reg_config(cinp_resource, false);
        if (cinp_resource->fck) {
            clk_disable_unprepare(cinp_resource->fck);
        }
        cinp_resource->clk_enabled = false;
    }

    // Liberar el mutex si quedó tomado por un cierre pendiente
    if (mutex_is_locked(&cinp_resource->lock)) {
        mutex_unlock(&cinp_resource->lock);
    }
    mutex_destroy(&cinp_resource->lock);

    // Eliminar los atributos de sysfs.
    device_remove_file(&pdev->dev, &dev_attr_escritura);
    device_remove_file(&pdev->dev, &dev_attr_lectura);

    
    // Destruir el dispositivo asociado a la clase.
    device_destroy(cinp_class, cinp_resource->cinp_cstm_cdev.cinp_dev);

    cinp_resource->cinp_device = NULL;

    // Desregistrar el character device.
    cdev_del(&cinp_resource->cinp_cstm_cdev.cinp_cdev);

    // Liberar los números de dispositivo.
    unregister_chrdev_region(cinp_resource->cinp_cstm_cdev.cinp_dev, CDEV_NBR);

    // Eliminar la asociación con los datos privados.
    platform_set_drvdata(pdev, NULL);

    pr_info("%s Device removed\n", __MODULE_NAME__);
}

// nombre que aparece en DT.
static const struct of_device_id cinp_of_match[] = {
    { .compatible = "utn,cin-device", },
    { },
};
// Se declara la tabla para que udev pueda cargar el driver automáticamente
MODULE_DEVICE_TABLE(of, cinp_of_match); 

// Esta es la estructura que define a mi platform driver.
static struct platform_driver cinp_driver = {
    .probe = cinp_probe,
    .remove = cinp_remove,
    .driver = {
        .name = "cin_platform_driver",
        .owner = THIS_MODULE,
        .of_match_table = of_match_ptr(cinp_of_match),  
    },
};


static int __init cinp_init(void)
{
    int status;

    pr_info("%s Registering module\n", __MODULE_NAME__);

    // Crear la clase.
    cinp_class = class_create("cinp_class");

    if (IS_ERR(cinp_class)) {
        status = PTR_ERR(cinp_class);
        cinp_class = NULL;
        pr_err("%s Failed to create class: %d\n", __MODULE_NAME__, status);
        return status;
    }

    // Configurar los permisos iniciales de los nodos.
    cinp_class->devnode = cinp_devnode;

    // Registrar el driver de plataforma.
    status = platform_driver_register(&cinp_driver);


    if (status < 0) {
        pr_err("%s Failed to register platform driver: %d\n",__MODULE_NAME__, status);
        class_destroy(cinp_class);
        cinp_class = NULL;
        return status;
    }

    return 0;
}
static void __exit cinp_exit(void)
{
    // Desregistrar el driver.
    // Esto provoca remove() para las instancias que sigan asociadas.
    platform_driver_unregister(&cinp_driver);

    // Destruir la clase cuando ya no quedan dispositivos asociados.
    if (cinp_class) {
        class_destroy(cinp_class);
        cinp_class = NULL;
    }

    pr_info("%s Module unregistered\n", __MODULE_NAME__);
}

module_init(cinp_init);
module_exit(cinp_exit);