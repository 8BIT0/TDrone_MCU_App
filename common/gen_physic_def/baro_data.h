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
    float pres;
    float temp;
} BaroData_TypeDef;

#ifdef __cplusplus
}
#endif

#endif
