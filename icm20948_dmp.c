#include "icm20948_dmp.h"
#include "icm20948.h"
#include <assert.h>
#include <string.h>

icm20948_err_t icm20948_dmp_initialize(icm20948_dmp_t* icm20948_dmp,
                                       icm20948_config_t const* config,
                                       icm20948_interface_t const* interface)
{
    assert(icm20948_dmp && config && interface);

    return icm20948_initialize(&icm20948_dmp->icm20948, config, interface);
}

icm20948_err_t icm20948_dmp_deinitialize(icm20948_dmp_t* icm20948_dmp)
{
    assert(icm20948_dmp);

    return icm20948_deinitialize(&icm20948_dmp->icm20948);
}