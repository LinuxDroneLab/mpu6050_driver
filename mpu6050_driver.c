/*
 * Copyright (c) 2016 Zubeen Tolani <ZeekHuge - zeekhuge@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/rpmsg.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/module.h>
#include <linux/kfifo.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/ktime.h>
#include "pru_mylinuxdrone.h"

/*
 * macro to print debug info easily
 */
#define log_debug(msg) printk(KERN_INFO "%s: %s\n", __FILE__, msg);

struct pru_channel {
    struct rpmsg_device *rpdev;
    struct device cntrdev;
};
static const struct device_type pru_channel_type = {
        .name           = "pru_channel",
};

struct device* pru_control_ptr;

/*************************************************************************************
 * PRU IMU DRIVER START
 *************************************************************************************/
struct pru_imu {
    struct pru_channel chst;
};


static struct pru_imu *dev_to_pru_imu(struct device *dev)
{
        return container_of(dev, struct pru_imu, chst.cntrdev);
}
static void pru_imu_release(struct device *dev)
{
    struct pru_imu *pc = dev_to_pru_imu(dev);
    printk(KERN_INFO "pru_imu_release.\n");
    kfree(pc);
}
static ssize_t imu_start_store(struct device *dev,
                            struct device_attribute *attr,
                            const char *buf, size_t len)
{
        struct pru_imu *ch = dev_to_pru_imu(dev);
        unsigned int enable;
        int ret;

        ret = kstrtouint(buf, 0, &enable);
        if (ret < 0)
                return ret;
        if (enable > 1)
                return -EINVAL;

        if(enable == 1) {
            // TODO: inviare messaggio di start
            printk(KERN_INFO "pru_imu_store: started.\n");
        } else {
            // TODO: inviare messaggio di stop
            printk(KERN_INFO "pru_imu_store: stopped.\n");
        }
        return ret ? : len;
}
static DEVICE_ATTR_WO(imu_start);

static struct attribute *pru_imu_attrs[] = {
        &dev_attr_imu_start.attr,
        NULL,
};
ATTRIBUTE_GROUPS(pru_imu);

/* imu_channels - structure that holds information about the
   channels that are present */
static const struct iio_chan_spec imu_channels[] = {
	{// accel, gyro, motors, rc, etc... all in one buffer
		.type = IIO_ACCEL,
		.indexed = 1,
		.channel = 0,
		.info_mask_separate = BIT(IIO_CHAN_INFO_ENABLE) | BIT(IIO_CHAN_INFO_FREQUENCY),
        .scan_index = 0,
        .scan_type = {
                .sign = 's',
                .realbits = sizeof(PrbMessageType),
                .storagebits = sizeof(PrbMessageType),
                .shift = 0,
                .endianness = IIO_LE,
        },
    },
};

/**
 * pru_imu_driver_remove() - function gets invoked when the rpmsg device is
 * removed
 */
static void pru_imu_driver_remove(struct rpmsg_device *rpdev)
{
    struct pru_imu *cntr;

    printk(KERN_INFO "pru_imu_driver_remove.\n");
    cntr = dev_get_drvdata(&rpdev->dev);
    if(cntr != NULL) {
        device_unregister(&cntr->chst.cntrdev);
    }
//    indio_dev = dev_get_drvdata(&rpdev->dev);
//    iio_device_unregister(indio_dev);
//    iio_device_free(indio_dev);
}

static int pru_imu_driver_cb(struct rpmsg_device *rpdev, void *data,
                  int len, void *priv, u32 src)
{
    printk(KERN_INFO "pru_imu_driver_cb.\n");
    return 0;
}
static int pru_imu_driver_probe (struct rpmsg_device *rpdev)
{
    int ret;
    struct pru_imu *st;

    printk(KERN_INFO "pru_imu_driver_probe [%s].\n", rpdev->id.name);
    // FIXME: verificare se device già creato.
    printk(KERN_INFO "pru_imu_driver_probe creating device.\n");

    st = kzalloc(sizeof(*st), GFP_KERNEL);
    if (!st) {
       return -ENOMEM;
    }
    printk(KERN_INFO "pru_imu_driver_probe allocated memory.\n");
    st->chst.rpdev = rpdev;
    st->chst.cntrdev.id = 1;
    st->chst.cntrdev.release = pru_imu_release;
    // TODO: il parent deve essere il pru_control device
    st->chst.cntrdev.parent = pru_control_ptr;
    st->chst.cntrdev.type = &pru_channel_type;
    st->chst.cntrdev.devt = MKDEV(0, 0);
    st->chst.cntrdev.groups = pru_imu_groups;
    dev_set_name(&st->chst.cntrdev, "pru_imu");
    printk(KERN_INFO "pru_imu_driver_probe pru_imu device prepared.\n");

    dev_set_drvdata(&rpdev->dev, st);

    // registra il nuovo device
    printk(KERN_INFO "pru_imu_driver_probe registering pru_imu device... \n");
    ret = device_register(&(st->chst.cntrdev));
    if (ret) {
       printk(KERN_INFO "pru_imu_driver_probe pru_imu device registration failed.\n");
       kfree(st);
       return ret;
    }

    // TODO: creare iio device

    printk(KERN_INFO "pru_imu_driver_probe pru_imu device registered \n");

    return 0;
}

/* pru_imu_id - Structure that holds the channel name for which this driver
   should be probed */
static const struct rpmsg_device_id pru_imu_id[] = {
        { .name = "pru-imu" },
        { },
};
MODULE_DEVICE_TABLE(rpmsg, pru_imu_id);

/* pru_imu_driver - The structure containing the pointers to read/write
   functions to send data to the pru */
static struct rpmsg_driver pru_imu_driver= {
    .drv.name   = "pru_imu_driver",
    .drv.owner  = THIS_MODULE,
    .id_table   = pru_imu_id,
    .probe      = pru_imu_driver_probe,
    .callback   = pru_imu_driver_cb,
    .remove     = pru_imu_driver_remove,
};
/*************************************************************************************
 * PRU IMU DRIVER END
 *************************************************************************************/

/*************************************************************************************
 * PRU CONTROL DRIVER START
 *************************************************************************************/
struct pru_control {
    struct pru_channel chst;
};

static struct class mylinuxdrone_class = {
        .name = "mylinuxdrone",
        .owner = THIS_MODULE,
};

static int enable_imu(struct pru_control *cntrl) {
    unsigned char startMessage[sizeof(PrbMessageType)];
    int ret;
    printk(KERN_INFO "enable_imu\n");
    ((PrbMessageType*)startMessage)->message_type = MPU_CREATE_CHANNEL_MSG_TYPE;

    ret = rpmsg_send(cntrl->chst.rpdev->ept, (void *)startMessage, sizeof(PrbMessageType));
    if (ret) {
        dev_err(&cntrl->chst.cntrdev, "Failed sending start mpu message to PRUs\n");
     }
    printk(KERN_INFO "enable_imu: creation of pru_imu device requested.\n");

    return 0;
}
static int disable_imu(struct pru_control *cntrl) {
    unsigned char startMessage[sizeof(PrbMessageType)];
    int ret;
    printk(KERN_INFO "disable_imu\n");
    ((PrbMessageType*)startMessage)->message_type = MPU_DESTROY_CHANNEL_MSG_TYPE;

    ret = rpmsg_send(cntrl->chst.rpdev->ept, (void *)startMessage, sizeof(PrbMessageType));
    if (ret) {
        dev_err(&cntrl->chst.cntrdev, "Failed sending start mpu message to PRUs\n");
     }
    printk(KERN_WARNING "disable_imu: remove of pru_imu device requested.\n");

    return 0;
}
static struct pru_control *dev_to_pru_control(struct device *dev)
{
        return container_of(dev, struct pru_control, chst.cntrdev);
}
static ssize_t imu_enable_store(struct device *dev,
                            struct device_attribute *attr,
                            const char *buf, size_t len)
{
        struct pru_control *ch = dev_to_pru_control(dev);
        unsigned int enable;
        int ret;

        ret = kstrtouint(buf, 0, &enable);
        if (ret < 0)
                return ret;
        if (enable > 1)
                return -EINVAL;

        if(enable == 1) {
            ret = enable_imu(ch);
        } else {
            ret = disable_imu(ch);
        }
        return ret ? : len;
}
static DEVICE_ATTR_WO(imu_enable);

static struct attribute *pru_control_attrs[] = {
        &dev_attr_imu_enable.attr,
        NULL,
};
ATTRIBUTE_GROUPS(pru_control);

static void pru_control_release(struct device *dev)
{
    struct pru_control *pc = dev_to_pru_control(dev);
    printk(KERN_INFO "pru_control_release.\n");
    kfree(pc);
}

/**
 * pru_control_driver_cb() - function gets invoked each time the pru sends some
 * data.
 */
static int pru_control_driver_cb(struct rpmsg_device *rpdev, void *data,
				  int len, void *priv, u32 src)
{
    printk(KERN_INFO "pru_control_driver_cb [%s].\n", rpdev->id.name);
	return 0;
}

/**
 * pru_control_driver_probe() - function gets invoked when the rpmsg channel
 * as mentioned in the pru_control_id table
 */
static int pru_control_driver_probe(struct rpmsg_device *rpdev)
{
	int ret;
	struct pru_control *st;

    printk(KERN_INFO "pru_control_driver_probe [%s].\n", rpdev->id.name);
    // FIXME: verificare se device già creato.
    printk(KERN_INFO "pru_control_driver_probe device created.\n");

    st = kzalloc(sizeof(*st), GFP_KERNEL);
    if (!st) {
       return -ENOMEM;
    }
    printk(KERN_INFO "pru_control_driver_probe allocated memory.\n");
    st->chst.rpdev = rpdev;
    st->chst.cntrdev.id = 1;
    st->chst.cntrdev.release = pru_control_release;
    st->chst.cntrdev.parent = NULL;
    st->chst.cntrdev.type = &pru_channel_type;
    st->chst.cntrdev.devt = MKDEV(0, 0);
    // FIXME: definire una classe?
//    st->chst.cntrdev.class = &mylinuxdrone_class;
    st->chst.cntrdev.groups = pru_control_groups;
    dev_set_name(&st->chst.cntrdev, "mylinuxdrone");
    printk(KERN_INFO "pru_control_driver_probe pru_control device prepared.\n");

    dev_set_drvdata(&rpdev->dev, st);

    // registra il nuovo device
    printk(KERN_INFO "pru_control_driver_probe registering pru_control device... \n");
    ret = device_register(&(st->chst.cntrdev));
    if (ret) {
       printk(KERN_INFO "pru_control_driver_probe pru_control device registration failed.\n");
       kfree(st);
       return ret;
    }

    pru_control_ptr = &st->chst.cntrdev;
    printk(KERN_INFO "pru_control_driver_probe pru_control device registered \n");

    return 0;

    /*
     * TODO
     * * creare device pru_control_device con attributi per il controllo delle componenti
     * - dividere i driver nei rispettivi file source .c; es. pru_imu_driver.c
     * * il device pru_control_device deve avere il riferimento a rpdev e viceversa
     * * quando viene settato un attributo di pru_control (es. imu-enable=1), lo 'store' deve invocare una funzione 'es. imu_enable(rpdev)'
     * - quando viene invocato una probe(rpdev), dovrà essere invocata una funzione 'es. imu_enabled(imu_control_device)
     * - serve quindi definire una struct di callback (prendere esempio da iio_info)
     * * adeguare pru_control_state a quanto realmente serve (probabilmente diventa pru_control_device con quanto indicato sopra)
     * * i drivers devono essere disaccoppiati ed autonomi
     * * i drivers sono registrati tutti all'init del modulo e deregistrati tutti alla exit del modulo
     * * il messaggio di create del channel rpmsg di un device è inviato da pru_control sul canale di controllo
     * * il messaggio di destroy del channel rpmsg è inviato da pru_control sul canale di controllo
     * * i devices sono creati e distrutti dal relativo driver (es. il device pru_imu_device da pru_imu_driver per effetto di pru_imu_probe o pru_imu_remove etc.)
     * - capire meglio relazione tra 'module', 'driver', 'device'
     *
     */

}

/**
 * pru_control_driver_remove() - function gets invoked when the rpmsg device is
 * removed
 */
static void pru_control_driver_remove(struct rpmsg_device *rpdev)
{
	struct pru_control *cntr;
    printk(KERN_INFO "pru_control_driver_remove.\n");
	cntr = dev_get_drvdata(&rpdev->dev);
	if(cntr != NULL) {
	    device_unregister(&cntr->chst.cntrdev);
	}
    pru_control_ptr = NULL;
}

/* pru_control_id - Structure that holds the channel name for which this driver
   should be probed
 */
static const struct rpmsg_device_id pru_control_id[] = {
		{ .name = "pru-control" },
		{ },
};
MODULE_DEVICE_TABLE(rpmsg, pru_control_id);

/* pru_control_driver - The structure containing the pointers to read/write
   functions to send data to the pru
 */
static struct rpmsg_driver pru_control_driver= {
	.drv.name	= KBUILD_MODNAME,
	.drv.owner	= THIS_MODULE,
	.id_table	= pru_control_id,
	.probe		= pru_control_driver_probe,
	.callback	= pru_control_driver_cb,
	.remove		= pru_control_driver_remove,
};
/*************************************************************************************
 * PRU CONTROL DRIVER END
 *************************************************************************************/

/**
 * mylinuxdrone_module_init() : driver driver registration
 *
 * The initialization function gets invoked when the driver is loaded. The
 * function registers itself on the virtio_rpmsg_bus and it gets invoked when
 * the pru creates a channel named as in the mpu6050_id structure.
 */
static int __init mylinuxdrone_module_init(void)
{
	int ret;
    printk(KERN_INFO "mylinuxdrone_module_init.\n");

    ret = register_rpmsg_driver(&pru_imu_driver);
    if (ret){
        pr_err("Failed to register pru_imu driver on rpmsg_bus\n");
        return ret;
    }
    printk(KERN_INFO "pru_imu_driver registered.\n");

    ret = register_rpmsg_driver(&pru_control_driver);
    if (ret){
        pr_err("Failed to register pru_control driver on rpmsg_bus\n");
        unregister_rpmsg_driver(&pru_imu_driver);
        return ret;
    }
    printk(KERN_INFO "pru_control_driver registered.\n");

	return 0;
}

/**
 * mylinuxdrone_module_exit() - function invoked when the driver is unloaded
 */
static void __exit mylinuxdrone_module_exit(void)
{
    printk(KERN_INFO "mylinuxdrone_module_exit.\n");

    unregister_rpmsg_driver (&pru_imu_driver);
    printk(KERN_INFO "pru_imu_driver unregistered.\n");

	unregister_rpmsg_driver (&pru_control_driver);
    printk(KERN_INFO "pru_control_driver unregistered.\n");
}

module_init(mylinuxdrone_module_init);
module_exit(mylinuxdrone_module_exit);

MODULE_AUTHOR("Andrea Lambruschini <andrea.lambruschini@gmail.com>");
MODULE_DESCRIPTION("mylinuxdrone linux Module");
MODULE_LICENSE("GPL v2");
