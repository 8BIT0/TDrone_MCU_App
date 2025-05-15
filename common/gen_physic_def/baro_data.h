#ifndef __BARO_DATA_H
#define __BARO_DATA_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

typedef struct
{
    uint32_t time_stamp;

    int32_t raw_pres;
    int16_t raw_temp;

    float pressure;
    float baro_alt;
    float temp;
} BaroData_TypeDef;

#endif
