#ifndef __TASK_NAVI_H
#define __TASK_NAVI_H

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

typedef union
{
    uint8_t val;
    
    struct
    {
        uint8_t imu  : 1;
        uint8_t baro : 1;
        uint8_t mag  : 1;
        uint8_t flow : 1;
        uint8_t tof  : 1;
        uint8_t res  : 3;
    } bit;
} TaskNaviSensorStatus_TypeDef;

typedef struct
{
    TaskNaviSensorStatus_TypeDef init_state;
    TaskNaviSensorStatus_TypeDef sample_state;

    uint32_t max_sample_period;
    uint32_t imu_sample_period;
    uint32_t baro_sample_period;
    uint32_t mag_sample_period;
    uint32_t tof_sample_period;
    uint32_t flow_sample_period;

    uint32_t imu_sampled_time;
    uint32_t baro_sampled_time;
    uint32_t mag_sampled_time;
    uint32_t tof_sampled_time;
    uint32_t flow_sampled_time;

    float att_pitch;
    float att_roll;
    float att_yaw;
    float att_heading;
    
    uint16_t period;
} TaskNavi_Monitor_TypeDef;

typedef struct
{
    uint32_t time_stamp;

    float pitch;
    float roll;
    float yaw;
    float heading;

    float acc[Axis_Sum];
    float gyr[Axis_Sum];
    float mag[Mag_Axis_Sum];
} NaviData_TypeDef;

void TaskNavi_Init(uint32_t period);
void TaskNavi_Core(void const *arg);

#ifdef __cplusplus
}
#endif

#endif
