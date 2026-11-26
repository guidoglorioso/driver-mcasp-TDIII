#ifndef _CIN_DRIVER_H_
#define _CIN_DRIVER_H_

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h> 
#include <linux/slab.h>    
#include <linux/device.h>  
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/string.h>
#include <linux/wait.h>
#include "utils.h"

#define CDEV_NBR 1
#define CDEV_MINORBASE 0

#define MCASP_PFUNC_REG 0x10    /* Pin Function Register (0: McASP, 1: GPIO) */ 
#define MCASP_PDIR_REG 0x14     /* Pin Direction Register (0: Input, 1: Output) */ 
#define MCASP_GBLCTL_REG 0x44   /* Global Control Register */ 
#define MCASP_RGBLCTL_REG 0x60  /* Receive Global Control Register */
#define MCASP_RMASK_REG   0x64  /* Receive Format Unit Bit Mask Register */
#define MCASP_RFMT_REG    0x68  /* Receive Bit Stream Format Register */
#define MCASP_AFSRCTL_REG 0x6C  /* Receive Frame Sync Control Register */
#define MCASP_ACLKRCTL_REG 0x70 /* Receive Clock Control Register */
#define MCASP_RTDM_REG    0x78  /* Receive TDM Time Slot Register */
#define MCASP_RINTCTL_REG 0x7C  /* Receive Interrupt Control Register */
#define MCASP_RSTAT_REG   0x80  /* Receive Status Register */

#define MCASP_XGBLCTL_REG 0xA0  /* Transmit Global Control Register */ 
#define MCASP_XMASK_REG   0xA4  /* Transmit Format Unit Bit Mask Register */
#define MCASP_XFMT_REG    0xA8  /* Transmit Bit Stream Format Register */
#define MCASP_AFSXCTL_REG 0xAC  /* Transmit Frame Sync Control Register  */
#define MCASP_ACLKXCTL_REG 0xB0 /* Transmit Bit.Clock Control */ 
#define MCASP_AHCLKXCTL_REG 0xB4 /* Transmit High-Frequency Clock Control */
#define MCASP_XTDM_REG    0xB8  /* Transmit TDM Time Slot Register */
#define MCASP_XINTCTL_REG 0xBC  /* Transmit Interrupt Control Register */
#define MCASP_XSTAT_REG   0xC0  /* Transmit Status Register */
#define MCASP_SRCTL0_REG  0x180 /* Serializer 0 Control Register (AXR0) */
#define MCASP_SRCTL1_REG  0x184 /* Serializer 1 Control Register (AXR1) */
#define MCASP_XBUF0_REG   0x200 /* Transmit Buffer Register for Serializer 0 (AXR0) */
#define MCASP_RBUF1_REG   0x284 /* Receive Buffer Register for Serializer 1 (AXR1) */

#define MCASP_INT_DATA    (1 << 5) /* Bit 5 de XINTCTL/XSTAT: XDATA / RDATA */

#define MCASP_WRITE_TX_BUF(handler, word) iowrite32((uint32_t)(word), (handler)->instance + MCASP_XBUF0_REG)
#define MCASP_READ_RX_BUF(handler)        ioread32((handler)->instance + MCASP_RBUF1_REG)

#define XTAL_FREQ 24000000
#define FAULT_FREQ 1000
#define FREQ_TEST 100000
#define SLAVE    0
#define MASTER   1


// Estructura que contiene los datos del device.
typedef struct {
    dev_t mymcasp_dev;
    struct cdev mymcasp_cdev;
} chardev_t;

// Estructura unificada para manejar la data privada del dispositivo
typedef struct {
    
    struct device *mymcasp_device;
    struct platform_device *pdev;
    chardev_t mymcasp_cstm_cdev;

    // Buffers circulares de Tx y Rx
    circ_buf_t tx_buf;
    circ_buf_t rx_buf;

    // Colas de espera (TASK_INTERRUPTIBLE) para Tx y Rx
    wait_queue_head_t tx_wq;
    wait_queue_head_t rx_wq;

    // Memoria de I2s y Clk
    void __iomem *instance;
    struct clk *fck;
    bool clk_enabled;

    // Interrupciones
    int irq_tx;
    int irq_rx;

    // Exclusión mutua independiente para Tx/Rx y serialización de hardware
    struct mutex tx_lock;
    struct mutex rx_lock;
    struct mutex hw_lock;
    uint32_t hw_users;
    bool dev_opened;

    // Parametros de I2S
    uint32_t hertz;
    uint8_t i2s_mode; /* SLAVE (0) o MASTER (1) */
} mymcasp_handler_t;

#endif /* _CIN_DRIVER_H_ */