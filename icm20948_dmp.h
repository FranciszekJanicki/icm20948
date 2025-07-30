#ifndef ICM20948_ICM20948_DMP_H
#define ICM20948_ICM20948_DMP_H

#include "icm20948.h"
#include "icm20948_dmp_config.h"
#include "icm20948_dmp_img.h"

typedef struct {
    icm20948_t icm20948;
} icm20948_dmp_t;

icm20948_err_t icm20948_dmp_initialize(icm20948_dmp_t* icm20948_dmp,
                                       icm20948_config_t const* config,
                                       icm20948_interface_t const* interface);
icm20948_err_t icm20948_dmp_deinitialize(icm20948_dmp_t* icm20948_dmp);

#endif // ICM20948_ICM20948_DMP_H