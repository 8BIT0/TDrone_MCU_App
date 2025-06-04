#ifndef __ATTITUDE_EST_H
#define __ATTITUDE_EST_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <Eigen>
#include "imu_data.h"
#include "mag_data.h"

typedef struct {
    uint32_t time_stamp;

    float roll;
    float pitch;
    float yaw;
    float heading;

    float q0;
    float q1;
    float q2;
    float q3;
} AttitudeData_TypeDef;

typedef struct {
    float delta_T;
    Eigen::Matrix<float, 4, 1> q;
} AttitudeObj_TypeDef;

bool AttitudeEstimate_Init(AttitudeObj_TypeDef *obj, uint16_t period);
AttitudeData_TypeDef AttitudeEstimate_Update(AttitudeObj_TypeDef *obj, float gyr_x, float gyr_y, float gyr_z, float acc_x, float acc_y, float acc_z, float mag_x, float mag_y, float mag_z);

#endif
