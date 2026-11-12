#include "mydriver.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Guido");
MODULE_DESCRIPTION("Un driver de ejemplo");

static const char __MODULE_NAME__[] = "cin"; // No poner nombres raros por que no carga el driver sino. (todo junto sin caracter especiales)

static struct class *cinp_class; // Necesario para crear el nodo en /dev

static int mcasp_set_square_wave(cinp_handler_t *handler, bool enable){

    uint32_t xtal = 24000000 ; // validar si es de 24Mhz o 12Mhz.
    uint32_t target_freq = 1000; // TODO Remplazar por la target freq real.
    uint32_t total_div = xtal / target_freq; 
    uint32_t ahclk_div;
    uint32_t aclk_div;
    uint32_t reg_val;
    if(total_div < 1) total_div = 1;
    if(total_div - 1 <= 4095){
        ahclk_div = total_div - 1;
        aclk_div = 0;
    }else{
        aclk_div= 31;
        ahclk_div = (total_div / 32) - 1;
        if(ahclk_div > 4095) ahclk_div = 4095;
    }

    // Configurar divisores de reloj
    reg_val = ioread32(handler->instance + MCASP_AHCLKXCTL_REG);
    reg_val &= ~(0xFFF | (1 << 15));
    reg_val |= (1 << 15)|(ahclk_div & 0xFFF);
    iowrite32(reg_val, handler->instance + MCASP_AHCLKXCTL_REG); 

    reg_val = ioread32(handler->instance + MCASP_ACLKXCTL_REG);
    reg_val &= ~(0x1F);
    reg_val |= (aclk_div & 0x1F);
    iowrite32(reg_val, handler->instance + MCASP_ACLKXCTL_REG); 

    reg_val = ioread32(handler->instance + MCASP_XGBLCTL_REG);
    reg_val |= 0x03;
    iowrite32(reg_val, handler->instance + MCASP_XGBLCTL_REG);

    reg_val = ioread32(handler->instance + MCASP_GBLCTL_REG);
    reg_val |= 0x03;
    iowrite32(reg_val, handler->instance + MCASP_GBLCTL_REG); 
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
    cinp_resource = devm_kzalloc(&pdev->dev, sizeof(*cinp_resource),GFP_KERNEL);

    if (!cinp_resource)
        return -ENOMEM;

    cinp_resource->pdev = pdev;
    cinp_resource->buffer_len = BUFF_SIZE;

    if(pdev->dev.of_node){
        status = of_property_read_u32(pdev->dev.of_node, "frecuencia", &cinp_resource->hertz);
        if(status < 0){
            cinp_resource->hertz = 1000; // valor default en caso de que falle la lectura del property
        }
        status = of_property_read_u32(pdev->dev.of_node, "square_on", &cinp_resource->square_on);
        if(status < 0){
            cinp_resource->square_on = 0;
        }
    }
    else{
        cinp_resource->hertz = 100;
        cinp_resource->square_on = 0 ;
    }



    // Inicializar el character device.
    cdev_init(
        &cinp_resource->cinp_cstm_cdev.cinp_cdev,
        &cinp_fops
    );

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

    //TODO Revisar si estoy trayendo la memoria adecuada.
    // Obtengo la ubicacion en memoria del recurso (del DT).
    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

    // Mapeo la memoria fisica a virtual.
    if(res){
        cinp_resource->instance = devm_ioremap_resource(&(pdev->dev), res);
        if(IS_ERR(cinp_resource->instance)){
            pr_err("%s:Error mapping resources (%s %s %d)\n", __MODULE_NAME__, __FILE__, __func__, __LINE__);     
            return PTR_ERR(cinp_resource->instance);
        }
    }
    int32_t reg_val;

    // TODO Revisar si las direcciones de memoria, y los pines son correctos
    // Asigno los GPIOs con sus modos al modulo de I2S.
    if(cinp_resource->instance){
        // Seteo de pines en modo I2S
        reg_val = ioread32(cinp_resource->instance + MCASP_PFUNC_REG);

        reg_val &= ~((1 << 28) | (1 << 29));
        iowrite32(reg_val, cinp_resource->instance + MCASP_PFUNC_REG);

        // Seteo de pines como salida.
        reg_val = ioread32(cinp_resource->instance + MCASP_PDIR_REG);
        reg_val |= ((1 << 28) | (1 << 29));
        iowrite32(reg_val, cinp_resource->instance + MCASP_PDIR_REG);
    }

    //seteo el clock del modulo.
    cinp_resource->fck = devm_clk_get(&(pdev->dev), "fck");
     
    if(!IS_ERR(cinp_resource->fck)){
        clk_prepare_enable(cinp_resource->fck);
    }
    else{
        cinp_resource->fck = NULL;
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
        pr_warn("%s No device data found during removal\n",
                __MODULE_NAME__);
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
    device_destroy(
        cinp_class,
        cinp_resource->cinp_cstm_cdev.cinp_dev
    );

    cinp_resource->cinp_device = NULL;

    // Desregistrar el character device.
    cdev_del(&cinp_resource->cinp_cstm_cdev.cinp_cdev);

    // Liberar los números de dispositivo.
    unregister_chrdev_region(
        cinp_resource->cinp_cstm_cdev.cinp_dev,
        CDEV_NBR
    );

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