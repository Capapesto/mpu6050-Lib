#include <stdio.h>
#include <math.h>
#include <string.h>

#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "mpu6050.h"


#define TAG "MPU_SYSTEM"

// ================= CONFIG =================

#define MPU6050_ADDR         0x68
#define I2C_MASTER_SCL_IO    22
#define I2C_MASTER_SDA_IO    21
#define I2C_MASTER_NUM       I2C_NUM_0
#define I2C_MASTER_FREQ_HZ   400000

#define QUEUE_SIZE 10

// ================= STRUCT INTERNA =================

typedef struct {
    float ax, ay, az;
    float gx, gy, gz;
    int64_t timestamp;
} sensor_data_t;

// ================= RTOS =================

static QueueHandle_t sensor_queue;
static SemaphoreHandle_t snapshot_mutex;

// snapshot global 
static mpu_snapshot_t g_snapshot = {0};

// ================= I2C =================

static void i2c_master_init(void) {
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

static esp_err_t mpu6050_write(uint8_t reg, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    return ret;
}

static esp_err_t mpu6050_read(uint8_t reg, uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);

    if (len > 1)
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);

    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    return ret;
}

// ================= TASK 1: SENSOR =================

static void sensor_task(void *arg) {

    uint8_t data[14];
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {

        if (mpu6050_read(0x3B, data, 14) == ESP_OK) {

            sensor_data_t sample;

            int16_t ax_raw = (data[0] << 8) | data[1];
            int16_t ay_raw = (data[2] << 8) | data[3];
            int16_t az_raw = (data[4] << 8) | data[5];

            int16_t gx_raw = (data[8] << 8) | data[9];
            int16_t gy_raw = (data[10] << 8) | data[11];
            int16_t gz_raw = (data[12] << 8) | data[13];

            sample.ax = ax_raw / 16384.0f;
            sample.ay = ay_raw / 16384.0f;
            sample.az = az_raw / 16384.0f;

            sample.gx = gx_raw / 131.0f;
            sample.gy = gy_raw / 131.0f;
            sample.gz = gz_raw / 131.0f;

            sample.timestamp = esp_timer_get_time();

            if (xQueueSend(sensor_queue, &sample, 0) != pdTRUE) {
                ESP_LOGW(TAG, "Fila cheia");
            }
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(50));
    }
}

// ================= TASK 2: PROCESSAMENTO =================

static void processing_task(void *arg) {

    sensor_data_t sample;

    static float pitch = 0;
    static float roll  = 0;

    static float last_magnitude = 0;
    static int64_t last_time = 0;

    while (1) {

        if (xQueueReceive(sensor_queue, &sample, portMAX_DELAY)) {

            float dt = (last_time == 0)
                       ? 0.05f
                       : (sample.timestamp - last_time) / 1000000.0f;

            last_time = sample.timestamp;

            float magnitude = sqrtf(
                sample.ax * sample.ax +
                sample.ay * sample.ay +
                sample.az * sample.az
            );

            float delta = fabsf(magnitude - last_magnitude);
            last_magnitude = magnitude;

            float pitch_acc = atan2f(
                -sample.ax,
                sqrtf(sample.ay * sample.ay + sample.az * sample.az)
            ) * 180.0f / M_PI;

            float roll_acc = atan2f(sample.ay, sample.az) * 180.0f / M_PI;

            // integração gyro
            pitch += sample.gx * dt;
            roll  += sample.gy * dt;

            // filtro complementar
            float alpha = 0.98f;

            pitch = alpha * pitch + (1 - alpha) * pitch_acc;
            roll  = alpha * roll  + (1 - alpha) * roll_acc;

            const char *estado;

            if (delta > 0.5f)
                estado = "IMPACTO";
            else if (magnitude > 1.5f)
                estado = "BRUSCO";
            else if (magnitude > 1.05f)
                estado = "LEVE";
            else
                estado = "PARADO";

            // ================= SNAPSHOT (CRÍTICO) =================
            if (xSemaphoreTake(snapshot_mutex, pdMS_TO_TICKS(5))) {

                g_snapshot.ax = sample.ax;
                g_snapshot.ay = sample.ay;
                g_snapshot.az = sample.az;

                g_snapshot.gx = sample.gx;
                g_snapshot.gy = sample.gy;
                g_snapshot.gz = sample.gz;

                g_snapshot.magnitude = magnitude;
                g_snapshot.delta     = delta;

                g_snapshot.pitch = pitch;
                g_snapshot.roll  = roll;

                g_snapshot.estado = estado;
                g_snapshot.timestamp = sample.timestamp;

                xSemaphoreGive(snapshot_mutex);
            }

            // debug
            printf("PITCH=%.2f ROLL=%.2f | %s\n", pitch, roll, estado);

            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

// ================= API =================

bool mpu_get_snapshot(mpu_snapshot_t *out) {

    if (!out) return false;

    if (xSemaphoreTake(snapshot_mutex, pdMS_TO_TICKS(10))) {

        memcpy(out, &g_snapshot, sizeof(mpu_snapshot_t));

        xSemaphoreGive(snapshot_mutex);
        return true;
    }

    return false;
}

// ================= MAIN =================

void mpu_main(void) {

    i2c_master_init();
    mpu6050_write(0x6B, 0x00);

    sensor_queue = xQueueCreate(QUEUE_SIZE, sizeof(sensor_data_t));
    snapshot_mutex = xSemaphoreCreateMutex();

    if (!sensor_queue || !snapshot_mutex) {
        ESP_LOGE(TAG, "Erro ao iniciar sistema");
        return;
    }

    xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 5, NULL);
    xTaskCreate(processing_task, "processing_task", 4096, NULL, 5, NULL);
}
