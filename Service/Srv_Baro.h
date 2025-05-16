#ifndef __SRV_BARO_H
#define __SRV_BARO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "Dev_BMP280.h"
#include "../algorithm/Filter_Dep/filter.h"
#include "gen_calib.h"

#define SRVBARO_SAMPLE_RATE_LIMIT SRVBARO_SAMPLE_RATE_100HZ   /* max sample rate 100Hz */
#define SRVBARO_DEFAULT_CALI_CYCLE 100

#define SRVBARO_SAMPLE_RATE_100HZ 100
#define SRVBARO_SAMPLE_RATE_50HZ  50
#define SRVBARO_SAMPLE_RATE_25HZ  25
#define SRVBARO_SAMPLE_RATE_20HZ  20
#define SRVBARO_SAMPLE_RATE_10HZ  10
#define SRVBARO_SAMPLE_RATE_5HZ   5
#define SRVBARO_SAMPLE_RATE_1HZ   1

typedef enum
{
    SrvBaro_Error_None = 0,
    SrvBaro_Error_BusInit,
    SrvBaro_Error_DevInit,
    SrvBaro_Error_BadSensorObj,
    SrvBaro_Error_FilterInit,
}SrvBaro_ErrorCodeList;

typedef struct
{
    uint32_t time_stamp;

    float tempra;
    float pressure;
    float alt;
    float alt_offset;

    uint8_t error_code;
}SrvBaro_Data_TypeDef;

typedef enum
{
    SrvBaro_Bus_None = 0,
    SrvBaro_Bus_IIC,
    SrvBaro_Bus_SPI,
    SrvBaro_BusType_Sum,
}SrvBaroBus_TypeList;

typedef struct
{
    void *sensor_obj;
    void *sensor_api;
    uint8_t init_err;
    uint8_t *sensor_data;
    uint8_t data_size;

    bool ready;
    SrvBaro_Data_TypeDef data;
    uint32_t sample_cnt;
    uint32_t sample_err_cnt;

    uint16_t calib_cycle;
    GenCalib_State_TypeList calib_state;
    float pressure_add_sum;
    float alt_offset;

    SW_Object_Handle smoothwindow_filter_hdl;
}SrvBaroObj_TypeDef;

typedef struct
{
    uint8_t (*init)(void);
    bool (*sample)(void);
    GenCalib_State_TypeList (*set_calib)(uint16_t calib_cyc);
    GenCalib_State_TypeList (*get_calib)(void);
    bool (*get)(SrvBaro_Data_TypeDef *data);
}SrvBaro_TypeDef;

extern SrvBaro_TypeDef SrvBaro;

#ifdef __cplusplus
}
#endif

#endif
