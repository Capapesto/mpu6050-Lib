#ifndef MPU6050_H
#define MPU6050_H

#include <stdio.h>
#include <math.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"



// ================= CONFIG =================

#define MPU6050_ADDR         0x68
#define I2C_MASTER_SCL_IO    22
#define I2C_MASTER_SDA_IO    21
#define I2C_MASTER_NUM       I2C_NUM_0
#define I2C_MASTER_FREQ_HZ   400000

#define QUEUE_SIZE 10



// ==================  MAIN  ====================

void mpu_main();

 




#endif 
