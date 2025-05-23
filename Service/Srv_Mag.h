#ifndef __SRV_MAG_H__
#define __SRV_MAG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "Dev_IST8310.h"

typedef struct
{
    uint32_t time_stamp;

    float raw_mag[Mag_Axis_Num];    /* raw mag data from device */
    float fit_mag[Mag_Axis_Num];    /* after ellipsoid fitting mag data */
} SrvMag_Data_TypeDef;

typedef struct
{
    bool (*init)(void);
    bool (*sample)(void);
    bool (*get)(SrvMag_Data_TypeDef *p_data);
} SrvMag_TypeDef;

extern SrvMag_TypeDef SrvMag;

#ifdef __cplusplus
}
#endif

#endif