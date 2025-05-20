#ifndef __BARO_DATA_H
#define __BARO_DATA_H

#ifdef __cplusplus
extern "C" {
#endif 

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

typedef struct
{
    uint32_t time_stamp;

    /* set if have */
    int32_t raw_i_pres;
    int16_t raw_i_temp;

    /* set if have */
    float raw_f_pres;
    float raw_f_temp;

    float pressure;
    float temp;
} BaroData_TypeDef;

#ifdef __cplusplus
}
#endif

#endif
