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

#define CDEV_NBR 1
#define CDEV_MINORBASE 0
#define BUFF_SIZE 1024      

#define MCASP_PFUNC_REG 0x10    /* Pin Function Register (0: McASP, 1: GPIO) */ 
#define MCASP_PDIR_REG 0x14     /* Pin Direction Register (0: Input, 1: Output) */ 
#define MCASP_GBLCTL_REG 0x44   /* Global Control Register */ 
#define MCASP_RGBLCTL_REG 0x60  /* Receive Global Control Register */
#define MCASP_XGBLCTL_REG 0xA0  /* Transmit Global Control Register */ 
#define MCASP_ACLKXCTL_REG 0xB0 /* Transmit Bit.Clock Control */ 
#define MCASP_AHCLKXCTL_REG 0xB4 /* Transmit High-Frequency Clock Control */
#define MCASP_ACLKRCTL_REG 0x70 /* Receive Clock Control Register */

#define MCASP_AFSXCTL_REG  0xAC /* Transmit Frame Sync Control Register  */

#define XTAL_FREQ 24000000


// Estructura que contiene los datos del device.
typedef struct {
    dev_t cinp_dev;
    struct cdev cinp_cdev;
} chardev_t;

// Estructura unificada para manejar la data privada del dispositivo
typedef struct {
    
    struct device *cinp_device;
    struct platform_device *pdev;
    chardev_t cinp_cstm_cdev;

    // Buffer de chardev.
    char buffer[BUFF_SIZE];
    size_t buffer_len;

    // Memoria de I2s y Clk
    void __iomem *instance;
    struct clk *fck;
    // Parametros de I2S
    uint32_t hertz;
    uint32_t square_on;
} cinp_handler_t;

#endif /* _CIN_DRIVER_H_ */