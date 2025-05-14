#ifndef POS_DATA_H
#define POS_DATA_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* relative move on 3 axis */
typedef struct
{
    uint32_t time;
    float pos_x;
    float pos_y;
    float pos_z;

    float vel_x;
    float vel_y;
    float vel_z;
} RelMov_TypeDef;

#ifdef __cplusplus
}
#endif

#endif
