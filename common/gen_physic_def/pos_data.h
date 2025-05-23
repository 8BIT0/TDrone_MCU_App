#ifndef POS_DATA_H
#define POS_DATA_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

typedef enum
{
    POS_Axis_X = 0,
    POS_Axis_Y = 0,
    POS_Axis_Z = 0,
    POS_Axis_Sum,
} POSAxis_TypeDef;

/* relative move on 3 axis */
typedef struct
{
    uint32_t time_stamp;
    float pos[POS_Axis_Sum];
    float vel[POS_Axis_Sum];
} RelMov_TypeDef;

#ifdef __cplusplus
}
#endif

#endif
