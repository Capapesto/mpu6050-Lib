#ifndef MPU6050_H
#define MPU6050_H

#include "driver/i2c.h"
#include "esp_err.h"

// I2C do MPU6050
#define MPU6050_ADDR         0x68

#define I2C_MASTER_SCL_IO    22
#define I2C_MASTER_SDA_IO    21
#define I2C_MASTER_NUM       I2C_NUM_0
#define I2C_MASTER_FREQ_HZ   400000


// Funções
void i2c_master_init(void);

esp_err_t mpu6050_write(uint8_t reg_addr, uint8_t data);

esp_err_t mpu6050_read(uint8_t reg_addr, uint8_t *data, size_t len);

#endif 
