#include "mydriver.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Guido");
MODULE_DESCRIPTION("Un driver de ejemplo");


// TODO
/*
* Temrinar de configurar el DT:
    - Configurar los pines correctamente
    - Establecer la propiedad de HW que se configura (FREQ 1MHZ y modo (master/slave))
    - Subir el archivo a la config.

* Funcionamiento
    - Probar que se genere el clock al hacer el probe y que se apague si se retira el driver.
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

static void mcasp_wait_gblctl(cinp_handler_t *handler, uint32_t mask) {
    int timeout = 1000;
    while (((ioread32(handler->instance + MCASP_GBLCTL_REG) & mask) != mask) && --timeout > 0) {
        udelay(1);
    }
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

    // Configurar divisores de reloj

    //HCLKXM (15) Internal transmit high-frequency clock source from output of programmable high clock divider.
    //HCLKXDIV (11-0) Transmit high-frequency clock divide ratio bits determine the dividedown ratio from AUXCLK to AHCLKX.
    reg_val = ioread32(handler->instance + MCASP_AHCLKXCTL_REG);
    reg_val &= ~(0xFFF | (1 << 15));
    reg_val |= (1 << 15)|(ahclk_div & 0xFFF);
    iowrite32(reg_val, handler->instance + MCASP_AHCLKXCTL_REG); 

    // Polaridad en Tx
    //CLKXP (7) Transmit bitstream clock polarity select bit | 0h = Rising edge. External receiver samples data on the falling edge of the serial clock, so the transmitter must shift data out on the rising edge of the serial clock.
    //CLKXM (5) Transmit bit clock source bit. 1h = Internal transmit clock source from output of programmable bit clock divider
    //CLKXDIV (4-0) Transmit bit clock divide ratio bits determine the divide-down ratio from AHCLKX to ACLKX
    reg_val = ioread32(handler->instance + MCASP_ACLKXCTL_REG);
    reg_val &= ~((1 << 7) | 0x1F);
    reg_val |= (1 << 5) | (aclk_div & 0x1F);
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
    /* FSXM (bit 1): 1h = Internally-generated transmit frame sync */
    reg_val = ioread32(handler->instance + MCASP_AFSXCTL_REG);
    reg_val &= ~(1 << 1);
    reg_val |= (1 << 1);
    iowrite32(reg_val, handler->instance + MCASP_AFSXCTL_REG);

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
    // Recuperamos nuestra estructura a través del cdev contenido en el inode
    cinp_handler_t *handler = container_of(inode->i_cdev, cinp_handler_t, cinp_cstm_cdev.cinp_cdev);
    file->private_data = handler;
    pr_info("%s Device opened\n", __MODULE_NAME__);
    return 0;
}

static int cinp_close(struct inode *inode, struct file *file) {
    pr_info("%s Device closed\n", __MODULE_NAME__);
    file->private_data = NULL; 
    
    return 0;
}

static ssize_t cinp_read(struct file *file, char __user *buf, size_t len, loff_t *offset) {
    
    pr_info("%s Device read\n", __MODULE_NAME__);

    cinp_handler_t *dev = file->private_data;

    ssize_t ret = 0;
    void *kbuff;

    if(len > dev->buffer_len - *offset) {
        len = dev->buffer_len - *offset; // Limitar len al tamaño del buffer
    }

    kbuff = kmalloc(len, GFP_KERNEL);
    if (kbuff == NULL) {
        pr_err("%s Failed to allocate memory\n", __MODULE_NAME__);
        return -ENOMEM;
    }

    memcpy(kbuff, dev->buffer + *offset, len); // Copiar datos al buffer temporal

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
   
    if( len >dev->buffer_len - *offset ) {
        len = dev->buffer_len - *offset; // Limitar len al tamaño del buffer
    }
    
    kbuff = kmalloc(len, GFP_KERNEL);
    if (kbuff == NULL) {
        pr_err("%s Failed to allocate memory\n", __MODULE_NAME__);
        return -ENOMEM;
    }

    if (copy_from_user(kbuff, buf, len)) {
        ret = -EFAULT;
    } else {
        // Pisamos los valores previos
        memcpy(dev->buffer, kbuff, len); // Copiar datos del buffer temporal al buffer del dispositivo
        dev->buffer_len = len; 
        *offset += len; // Actualizamos la posición real de escritura
        ret = len;
    }
    kfree(kbuff);

    pr_info("%s Device write\n", __MODULE_NAME__);

    return len;
}

static long cinp_ioctl(struct file *file, unsigned int cmd, unsigned long arg) {
    pr_info("%s Device ioctl\n", __MODULE_NAME__);
    return 0;
}


// Este es el puente que tenemos entre el driver nuestro y el kernel cuando hace los llamados que requiere el user
static struct file_operations cinp_fops = {
    .owner = THIS_MODULE,
    .read = cinp_read,
    .write = cinp_write,
    .unlocked_ioctl = cinp_ioctl,
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

// Aca se coloca toda mision de configuracion y puesta a punto del driver que no requieran memoria, por ejemplo un seteo de clock.
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

    // ENCENDER EL RELOJ PRIMERO
    int ret;
    cinp_resource->fck = devm_clk_get(&pdev->dev, "fck");
    if (IS_ERR(cinp_resource->fck)) {
        ret = PTR_ERR(cinp_resource->fck);
        dev_err(&pdev->dev, "No se pudo obtener el reloj fck: %d\n", ret);
        cinp_resource->fck = NULL;
        return ret;
    }

    ret = clk_prepare_enable(cinp_resource->fck);
    if (ret) {
        dev_err(&pdev->dev, "Error al habilitar el reloj fck: %d\n", ret);
        return ret;
    }

    // RECIÉN AHORA SE PUEDE ESCRIBIR EN LOS REGISTROS
    if (cinp_resource->instance) {
        uint32_t reg_val;
        
        // Seteo de pines en modo I2S
        reg_val = ioread32(cinp_resource->instance + MCASP_PFUNC_REG);
        reg_val &= ~((1 << 28) | (1 << 26) | (1 << 0));
        iowrite32(reg_val, cinp_resource->instance + MCASP_PFUNC_REG);

        // Seteo de pines como salida.
        /* Bits 28 (AFSX), 26 (ACLKX) y 0 (AXR0)*/
        reg_val = ioread32(cinp_resource->instance + MCASP_PDIR_REG);
        reg_val |= ((1 << 28) | (1 << 26) | (1 << 0));
        iowrite32(reg_val, cinp_resource->instance + MCASP_PDIR_REG);
    }

    // Atención con los argumentos de esta función
    mcasp_reg_config(cinp_resource, true);

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

    mcasp_reg_config(cinp_resource, false);

    if (!cinp_resource) {
        pr_warn("%s No se encontraron los datos del device\n",__MODULE_NAME__);
        return;
    }
    
    // Apagar el reloj  
    if (cinp_resource->fck) {
        clk_disable_unprepare(cinp_resource->fck);
    }

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