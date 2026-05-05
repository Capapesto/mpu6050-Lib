#ifndef MPU_SYSTEM_H
#define MPU_SYSTEM_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ================= SNAPSHOT =================
// Representa o estado atual do sistema

typedef struct {

    // ----- Aceleração (g) -----
    float ax;
    float ay;
    float az;

    // ----- Gyro (°/s) -----
    float gx;
    float gy;
    float gz;

    // ----- Métricas derivadas -----
    float magnitude;   // |a|
    float delta;       // variação da magnitude

    // ----- Orientação (graus) -----
    float pitch;
    float roll;

    // ----- Estado do movimento -----
    const char *estado;

    // ----- Timestamp (uS) -----
    int64_t timestamp;

} mpu_snapshot_t;




// Inicialização
void mpu_main(void);

// snapshot atual (thread-safe)
// Retorna false se não conseguiu acessar (timeout)
bool mpu_get_snapshot(mpu_snapshot_t *out);


#ifdef __cplusplus
}
#endif

#endif // MPU_SYSTEM_H
