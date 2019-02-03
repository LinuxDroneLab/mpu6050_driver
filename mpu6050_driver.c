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

// TODO: convertire in attributi di configurazione
#define SAMPLES_IN_FREQUENCY 1000L
#define SAMPLES_OUT_FREQUENCY 250L
#define GYRO_CALIBRATION_SAMPLES 10000L
#define GYRO_LSB_1000 65500L
#define GYRO_KALMAN_FACTOR_LEFT (30L)
#define GYRO_KALMAN_FACTOR_RIGHT (100L - GYRO_KALMAN_FACTOR_LEFT)
//#define HPF_GYRO_ALFA_100000 24145 // 250Hz
//#define HPF_GYRO_ALFA_100000 76094 // 50Hz
//#define HPF_GYRO_ALFA_100000 72500 // 60Hz
#define HPF_GYRO_ALFA_100000 67970 // 75Hz
//#define HPF_GYRO_ALFA_100000 61413 // 100Hz

/*
 * macro to print debug info easily
 */
#define log_debug(msg) printk(KERN_INFO "%s: %s\n", __FILE__, msg);

struct mpu6050_state {
	struct rpmsg_device *rpdev;
	struct device *dev;
	wait_queue_head_t wait_list;
	ktime_t now;
	ktime_t later;
    int32_t sumCalibrationGyroSamples[3];
    uint16_t autoCalibrationGyroCounter;
    int16_t minCalibrationGyroRaw[3];
    int16_t maxCalibrationGyroRaw[3];
    int16_t deadbandGyro[3];
    int8_t axisMatrixAccel[3];
    int8_t axisMatrixGyro[3];
    int16_t axisOffsetAccel[3];
    int16_t axisOffsetGyro[3];
    int16_t accelRaw[3];
    int16_t gyroRaw[3];
    int32_t sumGyroRaw[3];
    uint8_t outCounter;
};

/* mpu6050_channels - structure that holds information about the
   channels that are present */
static const struct iio_chan_spec mpu6050_channels[] = {
	{
		.type = IIO_ACCEL,
		.indexed = 1,
		.channel = 0,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
        .scan_index = 0,
        .scan_type = {
                .sign = 's',
                .realbits = 160,
                .storagebits = 160,
                .shift = 0,
                .endianness = IIO_LE,
        },
    },
};

/*
 * mpu6050_read_from_pru - To start the PRUs in the required reading mode
 *
 * @indio_dev	pointer to the instance of iio device
 *
 * Description - The function starts the PRUs by sending a start message
 * The message is then dispatched by the
 * rpmsg callback method. This function also checks if the required rpmsg device
 * has been released or not. If it has been released, the driver would return
 * with an error.
 *
 * The rpmsg callback method then pushes the data onto the iio_buffer.
 */
static int mpu6050_read_from_pru(struct iio_dev *indio_dev)
{
	int ret;
	struct mpu6050_state *st;
    unsigned char startMessage[sizeof(PrbMessageType)];
    ((PrbMessageType*)startMessage)->message_type = MPU_ENABLE_MSG_TYPE;
	log_debug("mpu6050_read_from_pru");

	st = iio_priv(indio_dev);

	if (!st->rpdev){
		dev_err(st->dev, "Required rpmsg device has been released\n");
		return -EINVAL;
	}

	ret = rpmsg_send(st->rpdev->ept, (void *)startMessage, sizeof(PrbMessageType));
	if (ret) {
		dev_err(st->dev, "Failed sending start mpu message to PRUs\n");
	}

//    ((PrbMessageType*)startMessage)->message_type = RC_ENABLE_MSG_TYPE;
//    ret = rpmsg_send(st->rpdev->ept, (void *)startMessage, sizeof(PrbMessageType));
//    if (ret) {
//        dev_err(st->dev, "Failed sending start rc message to PRUs\n");
//    }
	return 0;
}

/*
 * mpu6050_stop_sampling - to stop sampling process of the PRUS immediately
 */
static int mpu6050_stop_sampling_pru(struct iio_dev *indio_dev )
{
	int ret;
	unsigned char stop_val[sizeof(PrbMessageType)];
    struct mpu6050_state *st;
	((PrbMessageType*)stop_val)->message_type = MPU_DISABLE_MSG_TYPE;

	st = iio_priv(indio_dev);

	if (!st->rpdev){
		dev_err(st->dev, "Required rpmsg device already released\n");
		return -EINVAL;
	}

	ret = rpmsg_send(st->rpdev->ept, (void *)stop_val, sizeof(PrbMessageType));
	if (ret)
		dev_err(st->dev, "failed to stop mpu6050 sampling\n");

	return ret;
}

/*
 * mpu6050_buffer_postenable - function to do necessay work
 * just after the buffer gets enabled
 */
static int mpu6050_buffer_postenable(struct iio_dev *indio_dev)
{
	int ret;
	struct mpu6050_state *st;

    log_debug("postenable");
	st = iio_priv(indio_dev);
	st->now = ktime_get();
	st->later = ktime_get();
	ret = mpu6050_read_from_pru(indio_dev);
    printk(KERN_INFO "mpu6050_driver postenable result [%d]\n", ret);
	return ret;
}

/*
 * mpu6050_buffer_postenable - function to do necessay work
 * just before the buffer gets disabled
 */
static int mpu6050_buffer_predisable(struct iio_dev *indio_dev)
{
	log_debug("predisable");
	return mpu6050_stop_sampling_pru(indio_dev);
}

static const struct iio_buffer_setup_ops mpu6050_buffer_setup_ops = {
	.postenable = &mpu6050_buffer_postenable,
	.predisable = &mpu6050_buffer_predisable,
};

static int mpu6050_read_raw(struct iio_dev *indio_dev,
               struct iio_chan_spec const *chan,
               int *val,
               int *val2,
               long mask)
{
       struct mpu6050_state *st;
       st = iio_priv(indio_dev);
       log_debug("read_raw");
       printk(KERN_INFO "mpu6050_driver read_raw. Not implemented yet!\n");
       return 0;
}

static int mpu6050_write_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       int val,
			       int val2,
			       long mask)
{
	struct mpu6050_state *st;
	st = iio_priv(indio_dev);

    printk(KERN_INFO "mpu6050_driver write_raw %d. Not implemented yet!\n", val);
	log_debug("write_raw");
	return 0;
}

/* mpu6050_info - Structure contains constant data about the driver */
static const struct iio_info mpu6050_info = {
	.read_raw = mpu6050_read_raw,
	.write_raw = mpu6050_write_raw,
	.driver_module = THIS_MODULE,
};

/**
 * mpu6050_driver_cb() - function gets invoked each time the pru sends some
 * data.
 *
 */
static int mpu6050_driver_cb(struct rpmsg_device *rpdev, void *data,
				  int len, void *priv, u32 src)
{
	struct mpu6050_state *st;
	struct iio_dev *indio_dev;
	u16 *dataw = data;
	PrbMessageType* mpu6050DataStruct = (PrbMessageType*)data;
    uint8_t i = 0;
    int32_t tmp;
    int32_t rpy[3];
    int32_t yawRadians1M;
    uint32_t usec;

	indio_dev = dev_get_drvdata(&rpdev->dev);
	st = iio_priv(indio_dev);

	if(len == sizeof(PrbMessageType)) {
	      st->later = ktime_get();
	      usec = ktime_us_delta(st->later, st->now);
	      if(usec > 1500) {
	          printk(KERN_INFO "mpu6050_driver time exceeds [%d]\n", usec);
	          printk(KERN_INFO "a[%d,%d,%d], g[%d,%d,%d]\n",
	                 mpu6050DataStruct->mpu_accel_gyro.ax,
	                 mpu6050DataStruct->mpu_accel_gyro.ay,
	                 mpu6050DataStruct->mpu_accel_gyro.az,
	                 mpu6050DataStruct->mpu_accel_gyro.gx,
	                 mpu6050DataStruct->mpu_accel_gyro.gy,
	                 mpu6050DataStruct->mpu_accel_gyro.gz
	                             );
	      }
	      if(st->autoCalibrationGyroCounter == 0) {
	          for(i = 0; i < 3; i++) {
//LPF	              tmp = ((int32_t)st->gyroRaw[i] * GYRO_KALMAN_FACTOR_LEFT + (int32_t)(mpu6050DataStruct->mpu_accel_gyro_vect.gyro[i] - st->axisOffsetGyro[i]) * GYRO_KALMAN_FACTOR_RIGHT) / 100L;
                  tmp = (int32_t)st->gyroRaw[i] + HPF_GYRO_ALFA_100000 * (int32_t)(mpu6050DataStruct->mpu_accel_gyro_vect.gyro[i] - st->axisOffsetGyro[i] - (int32_t)st->gyroRaw[i]) / 100000L;
	              // apply offset
	              st->gyroRaw[i] = tmp;
                  st->accelRaw[i] = mpu6050DataStruct->mpu_accel_gyro_vect.accel[i] - st->axisOffsetAccel[i];

//                  // apply dead band
//                  if(abs(st->gyroRaw[i]) < st->deadbandGyro[i]) { // dead band
//                      st->gyroRaw[i] = 0;
//                  }

                  // for integration
                  tmp = st->gyroRaw[i];
                  tmp *= 10000L;
                  tmp /= GYRO_LSB_1000;
                  if(tmp < 0) {
                      tmp -=5;
                  } else {
                      tmp +=5;
                  }

	              st->sumGyroRaw[i] += tmp/10; // deg/sec

                  // 23580000 =  360 * 65.5 (LSB gyro)/dt (dove dt = 1/1000)
//	              if(abs(st->sumGyroRaw[i]) >= 23580000L) {
//	                  st->sumGyroRaw[i] = 0;
//	              }

	          }
//              printk(KERN_INFO "mpu6050_driver %d,%d,%d,%d,%d,%d;\n",
//                     mpu6050DataStruct->mpu_accel_gyro_vect.gyro[0] - st->axisOffsetGyro[0],
//                     mpu6050DataStruct->mpu_accel_gyro_vect.gyro[1] - st->axisOffsetGyro[1],
//                     mpu6050DataStruct->mpu_accel_gyro_vect.gyro[2] - st->axisOffsetGyro[2],
//                     st->gyroRaw[0], st->gyroRaw[1], st->gyroRaw[2]);

	          /* ----------------------------------------------------------------------
	           *                          T. B. D. START
	           * ---------------------------------------------------------------------- */
	          // compensate yaw rotation
              tmp = (int32_t)st->gyroRaw[2];
              tmp *= 1000L;
              tmp *= 314L;
              tmp /= 18L;
              tmp /= GYRO_LSB_1000;
              yawRadians1M = tmp; // * (1000/SAMPLES_IN_FREQUENCY)

              tmp = st->sumGyroRaw[0];
              st->sumGyroRaw[0] += (st->axisMatrixGyro[1]*st->sumGyroRaw[1]*st->axisMatrixGyro[2]*yawRadians1M)/1000000L;
              st->sumGyroRaw[1] += (st->axisMatrixGyro[0]*tmp*st->axisMatrixGyro[2]*yawRadians1M)/1000000L;
              /* ----------------------------------------------------------------------
               *                          T. B. D. END
               * ---------------------------------------------------------------------- */

              for(i = 0; i < 3; i++) {
//                  if(abs(st->sumGyroRaw[i]) >= 360000L) {
//                     st->sumGyroRaw[i] = 0;
//                  }
                  rpy[i] = st->sumGyroRaw[i]/SAMPLES_IN_FREQUENCY;
              }

	          st->outCounter--;
	          if(st->outCounter == 0) {
	              iio_push_to_buffers(indio_dev, dataw); // accel
	              st->outCounter = SAMPLES_IN_FREQUENCY/SAMPLES_OUT_FREQUENCY;
	              // dato dyaw = (gyroRaw[2]/GYROLSB)*(2pi/360/dt), Roll e Pitch devono essere compensati in funzione di dyaw
	              // tmp = rpy[0]
	              // rpy[0] = rpy[0] + axisMatrixGyro[1]*rpy[1]*sin(dyaw)
                  // rpy[1] = rpy[1] + axisMatrixGyro[0]*tmp*sin(dyaw)
	              // sin(dyaw) può essere approssimata con dyaw - 1/6*dyaw^3 o persino con dyaw se è piccolo

	              // TODO: implementing ...
//	              tmp = (int32_t)st->gyroRaw[2];
//	              tmp *= 1000L;
//	              tmp *= 314L;
//	              tmp /= 36L;
//	              tmp /= GYRO_LSB_1000;
//	              yawRadians1M = tmp; // * (1000/SAMPLES_IN_FREQUENCY)
//	              // con alte frequenze è possibile usare direttamente yawRadians invece che sinYawRadians
//	              sinYawRadians1M = (yawRadians1M*1000000L
//	                      - (yawRadians1M * yawRadians1M * yawRadians1M)/6L/1000000L)/1000000L;
//                  tmp = rpy[0];
//
//                  tmp_vect[0] = rpy[0];
//                  tmp_vect[1] = rpy[1];
//                  tmp_vect[2] = rpy[2];
//
//                  // FIXME: qui ci sono numeri troppo grandi per int32
//	              rpy[0] = (rpy[0]*1000L + st->axisMatrixGyro[1]*(rpy[1]*1000L * sinYawRadians1M)/1000000L)/1000L;
//	              rpy[1] = (rpy[1]*1000L + st->axisMatrixGyro[0]*(tmp*1000L * sinYawRadians1M)/1000000L)/1000L;

	              /*
	               * Fare i conti con:
	               *    tmp_vect[-447,431,568],
	               *    rpyC[-446,431,568],
	               *    yawRadians1M[-415],
	               *    sinYawRadians1M[-414],
	               *    gyroRawYaw[-3122]
	               */

//                  printk(KERN_INFO "mpu6050_driver rpy[%d,%d,%d], yawRadians1M[%d], gyroRawYaw[%d], sumGyroRawRoll[%d], sumGyroRawPitch[%d]\n",
//                         rpy[0], rpy[1], rpy[2], (int32_t)yawRadians1M,st->gyroRaw[2],st->sumGyroRaw[0],st->sumGyroRaw[1]);
//	              printk(KERN_INFO "mpu6050_driver %d,%d,%d,%d,%d,%d; yawRadians[%d]\n",rpy[0], rpy[1], rpy[2], st->sumGyroRaw[0], st->sumGyroRaw[1],st->sumGyroRaw[2], yawRadians1M);
	          }
	      } else {
              st->autoCalibrationGyroCounter--;
              if(st->autoCalibrationGyroCounter < GYRO_CALIBRATION_SAMPLES/2) {
                  for(i = 0; i < 3; i ++) {
//LPF                      tmp = ((int32_t)st->gyroRaw[i] * GYRO_KALMAN_FACTOR_LEFT + (int32_t)(mpu6050DataStruct->mpu_accel_gyro_vect.gyro[i]) * GYRO_KALMAN_FACTOR_RIGHT) / 100L;
                      tmp = (int32_t)st->gyroRaw[i] + HPF_GYRO_ALFA_100000 * (int32_t)(mpu6050DataStruct->mpu_accel_gyro_vect.gyro[i] - st->axisOffsetGyro[i] - (int32_t)st->gyroRaw[i]) / 100000L;
                      st->gyroRaw[i] = tmp;
                      st->accelRaw[i] = mpu6050DataStruct->mpu_accel_gyro_vect.accel[i];
                      st->sumCalibrationGyroSamples[i] += st->gyroRaw[i];
                          if(st->minCalibrationGyroRaw[i] > st->gyroRaw[i]) {
                              st->minCalibrationGyroRaw[i] = st->gyroRaw[i];
                          }
                          if(st->maxCalibrationGyroRaw[i] < st->gyroRaw[i]) {
                              st->maxCalibrationGyroRaw[i] = st->gyroRaw[i];
                          }
                          if(abs(st->gyroRaw[i]) > 500) {
                              printk(KERN_INFO "mpu6050_driver MAX/MIN [%d] gyroRaw[%d,%d,%d], minCalibrationGyroRaw[%d], maxCalibrationGyroRaw[%d], raw[%d]\n",
                                     i,
                                     st->gyroRaw[0], st->gyroRaw[1], st->gyroRaw[2],
                                     st->minCalibrationGyroRaw[i],
                                     st->maxCalibrationGyroRaw[i],
                                     mpu6050DataStruct->mpu_accel_gyro_vect.gyro[i]
                                     );
                          }
                  }
                  if(st->autoCalibrationGyroCounter == 0) {
                      for(i = 0; i < 3; i++) {
                          st->axisOffsetGyro[i] = st->sumCalibrationGyroSamples[i]/(GYRO_CALIBRATION_SAMPLES/2);
                          st->sumGyroRaw[i] = 0;
                          st->deadbandGyro[i] = (st->maxCalibrationGyroRaw[i] - st->minCalibrationGyroRaw[i])/2 + 1;
                          printk(KERN_INFO "mpu6050_driver GyroOffset[%d]=[%d] min=[%d], max=[%d]\n", i, st->axisOffsetGyro[i], st->minCalibrationGyroRaw[i], st->maxCalibrationGyroRaw[i]);
                      }
                      st->outCounter = SAMPLES_IN_FREQUENCY/SAMPLES_OUT_FREQUENCY;
                  }
              }
	      }
          st->now = ktime_get();
	} else {
	    printk(KERN_INFO "mpu6050_driver message received [%s]\n", (char *)data);
	}
	return 0;
}

/**
 * mpu6050_driver_probe() - function gets invoked when the rpmsg channel
 * as mentioned in the mpu6050_id table
 *
 * The function
 * - allocates space for the IIO device
 * - registers the device to the IIO subsystem
 * - exposes the sys entries according to the channels info
 */
static int mpu6050_driver_probe (struct rpmsg_device *rpdev)
{
	int ret;
	struct iio_dev *indio_dev;
	struct mpu6050_state *st;
	struct rpmsg_device_id *id;
	struct iio_buffer *buffer;
    uint8_t i = 0;


	log_debug("probe");
    printk(KERN_INFO "mpu6050_driver_probe.\n");

	indio_dev = devm_iio_device_alloc(&rpdev->dev, sizeof(*st));
	if (!indio_dev) {
		return -ENOMEM;
	}

	id = &rpdev->id;
	st = iio_priv(indio_dev);

	st->rpdev = rpdev;
	st->dev = &rpdev->dev;
	st->axisMatrixGyro[0] = 1;
    st->axisMatrixGyro[1] = -1;
    st->axisMatrixGyro[2] = -1;
	st->axisMatrixAccel[0] = 1;
    st->axisMatrixAccel[1] = 1;
    st->axisMatrixAccel[2] = 1;

    for(i = 0; i < 3; i++) {
        st->axisOffsetAccel[i] = 0;
        st->axisOffsetGyro[i] = 0;
        st->sumCalibrationGyroSamples[i] = 0;
        st->minCalibrationGyroRaw[i] = 3000;
        st->maxCalibrationGyroRaw[i] = -3000;
        st->gyroRaw[i] = 0;
    }
    st->autoCalibrationGyroCounter = GYRO_CALIBRATION_SAMPLES;

	dev_set_drvdata(&rpdev->dev, indio_dev);

	indio_dev->dev.parent = &rpdev->dev;
	indio_dev->name = id->name;
	indio_dev->info = &mpu6050_info;
	indio_dev->setup_ops = &mpu6050_buffer_setup_ops;
	indio_dev->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_SOFTWARE;
	indio_dev->channels = mpu6050_channels;
	indio_dev->num_channels = ARRAY_SIZE(mpu6050_channels);
	buffer = devm_iio_kfifo_allocate(&indio_dev->dev);
    if (!buffer) {
        return -ENOMEM;
    }

	ret = buffer->access->set_length(buffer, 8);
    if(ret < 0) {
        pr_err("Failed setting length of kfifo buffer\n");
        return ret;
    }
    ret = buffer->access->set_bytes_per_datum(buffer, sizeof(PrbMessageType));
	if(ret < 0) {
        pr_err("Failed setting bytes_per_datum of kfifo buffer\n");
        return ret;
	}

    ret = buffer->access->request_update(buffer);
    if(ret < 0) {
        pr_err("Failed update of kfifo buffer\n");
        return ret;
    }
	iio_device_attach_buffer(indio_dev, buffer);

	init_waitqueue_head(&st->wait_list);

	ret = iio_device_register(indio_dev);
	if (ret < 0) {
		pr_err("Failed to register with iio\n");
		return ret;
	}

	return 0;

}

/**
 * mpu6050_driver_remove() - function gets invoked when the rpmsg device is
 * removed
 */
static void mpu6050_driver_remove(struct rpmsg_device *rpdev)
{
	struct iio_dev *indio_dev;

    printk(KERN_INFO "mpu6050_driver_remove.\n");
	indio_dev = dev_get_drvdata(&rpdev->dev);

	iio_device_free(indio_dev);
}

/* mpu6050_id - Structure that holds the channel name for which this driver
   should be probed */
static const struct rpmsg_device_id mpu6050_id[] = {
		{ .name = "pru-mylinuxdrone" },
		{ },
};
MODULE_DEVICE_TABLE(rpmsg, mpu6050_id);

/* mpu6050_driver - The structure containing the pointers to read/write
   functions to send data to the pru */
static struct rpmsg_driver mpu6050_driver= {
	.drv.name	= KBUILD_MODNAME,
	.drv.owner	= THIS_MODULE,
	.id_table	= mpu6050_id,
	.probe		= mpu6050_driver_probe,
	.callback	= mpu6050_driver_cb,
	.remove		= mpu6050_driver_remove,
};

/**
 * mpu6050_driver_init() : driver driver registration
 *
 * The initialization function gets invoked when the driver is loaded. The
 * function registers itself on the virtio_rpmsg_bus and it gets invoked when
 * the pru creates a channel named as in the mpu6050_id structure.
 */
static int __init mpu6050_driver_init(void)
{
	int ret;
    printk(KERN_INFO "mpu6050_driver_init.\n");

	ret = register_rpmsg_driver(&mpu6050_driver);
	if (ret){
		pr_err("Failed to register mpu6050 driver on rpmsg_bus\n");
		return ret;
	}

    printk(KERN_INFO "mpu6050_driver registered as rpmsg driver.\n");

	return 0;
}

/**
 * mpu6050_driver_exit() - function invoked when the driver is unloaded
 */
static void __exit mpu6050_driver_exit(void)
{
    printk(KERN_INFO "mpu6050_driver_exit.\n");
	unregister_rpmsg_driver (&mpu6050_driver);
}

module_init(mpu6050_driver_init);
module_exit(mpu6050_driver_exit);

MODULE_AUTHOR("Andrea Lambruschini <andrea.lambruschini@gmail.com>");
MODULE_DESCRIPTION("mpu6050 linux Driver");
MODULE_LICENSE("GPL v2");
