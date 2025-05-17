#ifndef __SRV_IMU_H
#define __SRV_IMU_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "Srv_OsCommon.h"
#include "imu_data.h"
#include "util.h"
#include "gen_calib.h"
#include "../FCHW_Config.h"

#define MPU_RANGE_MAX_THRESHOLD 1.2f
#define MPU_RANGE_MIN_THRESHOLD 0.9f
#define MPU_MODULE_OPR_DURATION 50 // duration time 50ms

#define IMU_Commu_TimeOut 1000
#define MPU_MODULE_INIT_RETRY 10 // init retry count 10

#define GYR_STATIC_CALIB_CYCLE 100

#define IMU_DATA_SIZE sizeof(SrvIMU_Data_TypeDef)

typedef enum
{
    SrvIMU_Dev_Detect_Error = -12,
    SrvIMU_IMU_Filter_Init_Error = -9,
    SrvIMU_CSPin_Init_Error = -8,
    SrvIMU_ExtiPin_Init_Error = -7,
    SrvIMU_Bus_Init_Error = -6,
    SrvIMU_Dev_Init_Error = -5,
    SrvIMU_No_Error = 0,
    SrvIMU_AllModule_Init_Error,
} SrvIMU_ErrorCode_List;

typedef enum
{
    SrvIMU_Sample_NoError = 0,
    SrvIMU_Sample_Module_UnReady,
    SrvIMU_Sample_Data_Acc_Blunt,
    SrvIMU_Sample_Data_Gyr_Blunt,
    SrvIMU_Sample_Data_Acc_OverRange,
    SrvIMU_Sample_Data_Gyr_OverRange,
    SrvIMU_Sample_Over_Angular_Accelerate,
} SrvIMU_SampleErrorCode_List;

typedef struct
{
    uint8_t Acc;
    uint16_t Gyr;
}SrvIMU_Range_TypeDef;

typedef struct
{
    uint32_t time_stamp;
    uint32_t cycle_cnt;

    float tempera;

    float flt_gyr[Axis_Sum];
    float flt_acc[Axis_Sum];

    float org_gyr[Axis_Sum];
    float org_acc[Axis_Sum];

    float max_gyr_angular_diff;

    SrvIMU_SampleErrorCode_List error_code;
    float acc_scale;
    float gyr_scale;
    uint16_t chk_sum;
} SrvIMU_Data_TypeDef;

typedef union
{
    uint8_t buff[sizeof(SrvIMU_Data_TypeDef)];
    SrvIMU_Data_TypeDef data;
} SrvIMU_UnionData_TypeDef;

typedef struct
{
    GenCalib_State_TypeList state;
    uint16_t calib_cycle;
    uint16_t cur_cycle;

    float max[Axis_Sum];
    float min[Axis_Sum];
    float avg[Axis_Sum];

    float z_offset[Axis_Sum];
}SrvIMU_CalibMonitor_TypeDef;

typedef struct
{
    SrvIMU_ErrorCode_List (*init)(void);
    bool (*sample)(void);
    bool (*get)(SrvIMU_Data_TypeDef *data);
    bool (*get_range)(SrvIMU_Range_TypeDef *range);
    float (*get_max_angular_speed_diff)(void);
    void (*error_proc)(void);
} SrvIMU_TypeDef;

extern SrvIMU_TypeDef SrvIMU;

#ifdef __cplusplus
}
#endif

#endif
