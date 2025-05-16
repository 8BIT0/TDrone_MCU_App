#ifndef __SRV_BARO_H
#define __SRV_BARO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <string.h>
#include <stdbool.h>

typedef union
{
    uint8_t val;

    struct
    {
        uint8_t init    : 1;
        uint8_t meas    : 1;
        uint8_t unready : 1;
        uint8_t res     : 5;
    } bit;
} SrvBaroErr_TypeDef;

typedef struct
{
    uint32_t time_stamp;
    float pres;             /* unit: pa  */
    float temperature;      /* unit: deg */
    SrvBaroErr_TypeDef err;
} SrvBaro_Data_TypeDef;

typedef struct
{
    bool (*init)(void);
    bool (*sample)(void);
    bool (*get)(SrvBaro_Data_TypeDef *data);
} SrvBaro_TypeDef;

extern SrvBaro_TypeDef SrvBaro;

#ifdef __cplusplus
}
#endif
#endif
