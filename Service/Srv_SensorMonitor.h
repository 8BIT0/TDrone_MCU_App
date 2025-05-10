#ifndef __SRV_SENSORMONITOR_H
#define __SRV_SENSORMONITOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "util.h"
#include "imu_data.h"

#define BUF_SIZE 128

#define GYRO_CALIB_CYCLE GYR_STATIC_CALIB_CYCLE
#define BARO_CALIB_CYCLE SRVBARO_DEFAULT_CALI_CYCLE

#define IMU_TypeStr(x) SrvIMU_Get_TypeStr(x)
#define BARO_TypeStr(x) SrvBaro_Get_TypeStr(x)
#define BARO_Bus_TypeStr(x) SrvBaro_Get_BusStr(x)

typedef enum
{
    Sensor_Type_IMU = 0,
    Sensor_Type_MAG,
    Sensor_Type_BARO,
    Sensor_Type_ToF,
    Sensor_Type_Flow,
    Sensot_Type_SUM,
} SrvSensorMonitor_Type_List;

typedef union
{
    uint8_t val;
    struct
    {
        uint8_t imu  : 1;
        uint8_t mag  : 1;
        uint8_t baro : 1;
        uint8_t flow : 1;
        uint8_t tof  : 1;

        uint8_t res  : 3;
    }bit;
} SrvSensorReg_TypeDef;

#pragma pack(1)
typedef struct
{
    uint16_t T1;
    int16_t  T2;
    int16_t  T3;

    uint16_t  P1;
    int16_t   P2;
    int16_t   P3;
    int16_t   P4;
    int16_t   P5;
    int16_t   P6;
    int16_t   P7;
    int16_t   P8;
    int16_t   P9;

    int32_t T_fine;
} SrvSensor_BaroCalib_TypeDef;

typedef union
{
    uint8_t val;

    struct
    {
        uint8_t imu  : 1;
        uint8_t mag  : 1;
        uint8_t baro : 1;
        uint8_t tof  : 1;
        uint8_t res  : 4;
    } field;
} SrvSensor_DataReqBit_TypeDef;
#pragma pack()

typedef struct
{
    SrvSensorReg_TypeDef init_state_reg;

    void *sensorboard_cs_obj;
    void *sensorboard_dr_obj;

    void *sensorboard_bus_handle;
    void *flow_bus_handle;

    float acc_factory;
    float gyr_factory;

    SrvSensor_BaroCalib_TypeDef baro_calib;
    void *DR_SemaID;

    uint8_t *sensor_data;
    uint8_t *flow_data;

    uint32_t fpga_req_cnt;
    uint32_t fpga_ack_cnt;

    uint32_t flow_ack_cnt;

    uint8_t sensor_tx_buf[BUF_SIZE];
    uint8_t sensor_rx_buf[BUF_SIZE];

    uint8_t flow_rx_buf[BUF_SIZE];
} SrvSensorMonitorObj_TypeDef;

typedef enum
{
    Sensor_Error_None = 0,
    Sensor_Over_Range,
    Sensor_Blunt,
    Sensor_exceed_Rate_of_change,
} SrvSensor_Error_List;

typedef struct
{
    uint32_t time_stamp;
    float raw_Gyro[Axis_Sum];
    float raw_Acc[Axis_Sum];
    float raw_Mag[Axis_Sum];
    float raw_baro;
    float raw_tof;
    float raw_pos_X;
    float raw_pos_Y;

    float scale_Gyro[Axis_Sum];
    float scale_Acc[Axis_Sum];
    float scale_Mag[Axis_Sum];
    float scale_baro;
    float scale_tof;
    float scale_pos_X;
    float scale_pos_Y;

    uint8_t acc_err;
    uint8_t gyro_err;
    uint8_t baro_err;
    uint8_t tof_err;
    uint8_t flow_err;
} SrvSensorData_TypeDef;

typedef struct
{
    SrvSensorReg_TypeDef (*init)(void);
    void (*sample)(void);
    void (*calib)(uint8_t calib_module);
    bool (*get_data)(SrvSensorData_TypeDef *p_data);
} SrvSensor_TypeDef;

extern SrvSensor_TypeDef SrvSensor;

#ifdef __cplusplus
}
#endif

#endif
