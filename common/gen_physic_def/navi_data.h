#ifndef __NAVI_DATA_H
#define __NAVI_DATA_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

typedef struct
{
    float rel_pos_x;
    float rel_pos_y;
    float rel_pos_z;

    float rel_vel_x;
    float rel_vel_y;
    float rel_vel_z;

    float gyr_x;
    float gyr_y;
    float gyr_z;

    float acc_x;
    float acc_y;
    float acc_z;

    float mag_x;
    float mag_y;
    float mag_z;

    float baro_pressure;
    float baro_temperature;
    float baro_alt;

    float flow_x;
    float flow_y;
    float flow_z;

    float pitch;
    float roll;
    float yaw;

    float q0;
    float q1;
    float q2;
    float q3;

    float heading;

    float tof_alt;
} NaviData_TypeDef;

#endif
