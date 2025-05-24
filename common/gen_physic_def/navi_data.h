#ifndef __NAVI_DATA_H
#define __NAVI_DATA_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "pos_data.h"
#include "imu_data.h"
#include "baro_data.h"
#include "mag_data.h"

typedef struct
{
    uint32_t time_stamp;

    float pitch;
    float roll;
    float yaw;
    float heading;

    float q_0;
    float q_1;
    float q_2;
    float q_3;

    float rel_vel[POS_Axis_Sum];    /* relative x y z velocity */
    float rel_pos[POS_Axis_Sum];    /* relative x y z movement */

    float baro_pres;
    float baro_alt;

    float acc[Axis_Sum];
    float gyr[Axis_Sum];
    float mag[Mag_Axis_Sum];

} NaviData_TypeDef;


#ifdef __cplusplus
}
#endif

#endif
