#ifndef MPU_SYSTEM_H
#define MPU_SYSTEM_H

#include <stdint.h>
#include <stdbool.h>

// ================= SNAPSHOT =================

typedef struct {

    // Aceleração
    float ax;
    float ay;
    float az;

    // Gyro
    float gx;
    float gy;
    float gz;

    // Processamento
    float magnitude;
    float delta;

    float pitch;
    float roll;

    const char *estado;

    int64_t timestamp;

} mpu_snapshot_t;



void mpu_main(void);

// retorna snapshot completo (seguro)
bool mpu_get_snapshot(mpu_snapshot_t *out);

#endif
