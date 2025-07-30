#ifndef ICM20948_ICM20948_DMP_CONFIG_H
#define ICM20948_ICM20948_DMP_CONFIG_H

#include "icm20948_config.h"
#include <stdint.h>

typedef struct {
    int16_t w;
    int16_t x;
    int16_t y;
    int16_t z;
} quat3_int16_t;

typedef struct {
    float32_t w;
    float32_t x;
    float32_t y;
    float32_t z;
} quat3_float32_t;

#define ICM20948_DMP_MEMORY_BANKS 8
#define ICM20948_DMP_MEMORY_BANK_SIZE 256UL
#define ICM20948_DMP_MEMORY_CHUNK_SIZE 16UL
#define ICM20948_FIFO_DEFAULT_TIMEOUT 11000
#define ICM20948_FIFO_MAX_COUNT 1024UL
#define ICM20948_DMP_SCALE (2.0F / (float32_t)(1 << 31))

#endif // ICM20948_ICM20948_DMP_CONFIG_H