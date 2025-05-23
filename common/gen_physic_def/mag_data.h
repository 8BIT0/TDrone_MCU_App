#ifndef __MAG_DATA_H__
#define __MAG_DATA_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

typedef enum
{
    Mag_Axis_X = 0,
    Mag_Axis_Y,
    Mag_Axis_Z,
    Mag_Axis_Sum,
} MagAxis_TypeDef;

typedef struct
{
    uint32_t time_stamp;

    int16_t raw_mag[Mag_Axis_Sum];
    int16_t raw_mag_offset[Mag_Axis_Sum];
    int16_t raw_mag_temp;
          
    float mag[Mag_Axis_Sum];
    float mag_temp;
} MagData_TypeDef;

#ifdef __cplusplus
}
#endif

#endif
