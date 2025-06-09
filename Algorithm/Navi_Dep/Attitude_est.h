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
    float delta_T;                      /* unit: s */
    float mag_heading;
    Eigen::Matrix<float, 4, 1> q;       /* quaternion data */
    Eigen::Matrix<float, 7, 7> phi;
    Eigen::Matrix<float, 3, 1> gyr_b;   /* gyro bias */
    Eigen::Matrix<float, 4, 7> H;       /* measure jacobi matrix */
    Eigen::Matrix<float, 7, 7> Q;       /* process noise matrix */
    Eigen::Matrix<float, 7, 1> X;       /* state estimate matrix */
    Eigen::Matrix<float, 4, 1> Z;       /* measure matrix */
    Eigen::Matrix<float, 7, 7> P;
    Eigen::Matrix<float, 7, 4> K;
    Eigen::Matrix<float, 4, 4> R;       /* measure covariance matrix (const) */
} AttitudeObj_TypeDef;

bool AttitudeEstimate_Init(AttitudeObj_TypeDef *obj, uint16_t period, float Q_init[7][7], float R_init[4][4], float P_init[7][7]);
AttitudeData_TypeDef AttitudeEstimate_Update(AttitudeObj_TypeDef *obj, float gyr_x, float gyr_y, float gyr_z, float acc_x, float acc_y, float acc_z, float mag_x, float mag_y, float mag_z);

#endif
