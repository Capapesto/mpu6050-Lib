#include "driver/i2c.h"
#include "esp_log.h"

#define MPU6050_ADDR         0x68 // Endereço I2C
#define I2C_MASTER_SCL_IO    22
#define I2C_MASTER_SDA_IO    21
#define I2C_MASTER_NUM       I2C_NUM_0
#define I2C_MASTER_FREQ_HZ   400000

// 1. Inicializar I2C
void i2c_master_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

// 2. Escrever num registrador
esp_err_t mpu6050_write(uint8_t reg_addr, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// 3. Ler registradores
esp_err_t mpu6050_read(uint8_t reg_addr, uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

void app_main() {
    i2c_master_init();
    mpu6050_write(0x6B, 0x00); // Acordar MPU6050 (PWR_MGMT_1)

    uint8_t data[14];
    int16_t ax, ay, az, temp, gx, gy, gz;

    while (1) {
        // Ler 14 bytes começando do 0x3B (ACCEL_XOUT_H)
        mpu6050_read(0x3B, data, 14);

        // Converter bytes (Alto << 8 | Baixo)
        ax = (data[0] << 8) | data[1];
        ay = (data[2] << 8) | data[3];
        az = (data[4] << 8) | data[5];
        temp = (data[6] << 8) | data[7];
        gx = (data[8] << 8) | data[9];
        gy = (data[10] << 8) | data[11];
        gz = (data[12] << 8) | data[13];

        printf("Accel: %d, %d, %d\n", ax, ay, az);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
