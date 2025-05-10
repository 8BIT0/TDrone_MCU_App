#include "Srv_SensorMonitor.h"
#include "filter.h"
#include "Bsp_SPI.h"
#include "Bsp_Uart.h"
#include "Bsp_GPIO.h"
#include "Srv_OsCommon.h"
#include "debug_util.h"
#include "HW_Def.h"
#include "util.h"

#define MONITOR_TAG "[ SENSOR MONITOR INFO ] "
#define MONITOR_INFO(fmt, ...)  Debug_Print(&DebugPort, MONITOR_TAG, fmt, ##__VA_ARGS__)

#define MONITOR_MALLOC(size)    SrvOsCommon.malloc(size)
#define MONITOR_FREE(ptr)       SrvOsCommon.free(ptr)
 
#define SENSORBOARD_HEADER      0xAA
#define SENSORBOARD_ENDER       0xED

#define SENSORBOARD_DR_TIMEOUT  10  /* unit: 10Ms */

#define COMPONENT_INDEX         2
#define PAYLOAD_INDEX           4
#define GYRO_FACTORY_INDEX      5
#define ACC_FACTORY_INDEX       9

typedef enum
{
    Reg_Sensor_Set  = 0x09,
    Reg_Qur_Setting = 0x10,
    Reg_Data_Req    = 0x11,
    Reg_Data_Ack    = 0x12,
} SrvSensor_Reg_List;

typedef enum
{
    Comp_IMU  = 0x01,
    Comp_Baro = 0x02,
    Comp_Mag  = 0x03,
    Comp_ToF  = 0x04,
} SrvSensor_Component_List;

/************************************************ IMU Configure *************************************************/
typedef enum
{
    IMU_Set_Range  = 0x01,
    IMU_Set_Filter = 0x02,
} SrvSensor_IMUCfg_List;

typedef enum
{
    IMU_Gyro_250dsp  = 0x00,
    IMU_Gyro_500dsp  = 0x01,
    IMU_Gyro_1000dsp = 0x02,
    IMU_Gyro_2000dsp = 0x03,
} SrvSensor_GyroRange_List;

typedef enum
{
    IMU_Acc_2g  = 0x00,
    IMU_Acc_4g  = 0x01,
    IMU_Acc_8g  = 0x02,
    IMU_Acc_16g = 0x03,
} SrvSensor_AccRange_List;

typedef enum
{
    IMU_Filter_BD_258Hz  = 0x00,
    IMU_Filter_BD_536Hz  = 0x01,
    IMU_Filter_BD_997Hz  = 0x02,
    IMU_Filter_BD_1962Hz = 0x03,
} SrvSensor_IMUBandwidth_List;

/************************************************ Baro Configure *************************************************/
typedef enum
{
    Baro_Set_OverSampling = 0x01,
    Baro_Set_Filter       = 0x02,
    Baro_Req_Calib        = 0x03,
} SrvSensor_BaroCfg_List;

typedef enum
{
    Baro_OverSampling_Skip = 0x00,
    Baro_OverSampling_1x   = 0x01,
    Baro_OverSampling_2x   = 0x02,
    Baro_OverSampling_4x   = 0x03,
    Baro_OverSampling_8x   = 0x04,
    Baro_OverSampling_16x  = 0x05,
} SrvSesnor_BaroOverSampling_List;

typedef enum
{
    Baro_Filter_Off      = 0x00,
    Baro_Filter_COEFF_2  = 0x01,
    Baro_Filter_COEFF_4  = 0x02,
    Baro_Filter_COEFF_8  = 0x03,
    Baro_Filter_COEFF_16 = 0x04,
} SrvSensor_BaroFilter_List;

/************************************************ Mag  Configure *************************************************/
/************************************************ ToF  Configure *************************************************/

/* internal vriable */
static SrvSensorMonitorObj_TypeDef SensorMonitor;

/* internal function */
static bool SrvSensor_Board_Init(SrvSensorMonitorObj_TypeDef *obj);
static bool SrvSensor_Flow_Init(SrvSensorMonitorObj_TypeDef *obj);

/* external function */
static SrvSensorReg_TypeDef SrvSensor_Init(void);
static void SrvSensor_Sample(void);
static void SrvSensor_Set_Module_Calib(uint8_t calib_module);

SrvSensor_TypeDef SrvSensor = {
    .init = SrvSensor_Init,
    .sample = SrvSensor_Sample,
    .calib = SrvSensor_Set_Module_Calib,
};

static SrvSensorReg_TypeDef SrvSensor_Init(void)
{
    memset(&SensorMonitor, 0, sizeof(SrvSensorMonitorObj_TypeDef));

    /* configure sensor board */
    SrvSensor_Board_Init(&SensorMonitor);

    /* configure flow sensor */
    SrvSensor_Flow_Init(&SensorMonitor);

    return SensorMonitor.init_state_reg;
}

static uint32_t SrvSensor_Get_InitState(SrvSensorMonitorObj_TypeDef *obj)
{
    if (obj)
        return obj->init_state_reg.val;

    return 0;
}

static void SrvSensor_Board_IMU_Init(SrvSensorMonitorObj_TypeDef *obj)
{
}

static void SrvSensor_Board_Baro_Init(SrvSensorMonitorObj_TypeDef *obj)
{
}

static void SrvSensor_Board_Mag_Init(SrvSensorMonitorObj_TypeDef *obj)
{
}

static void SrvSensor_Board_ToF_Init(SrvSensorMonitorObj_TypeDef *obj)
{
}

static void SrvSensor_Board_DR_Callback(void)
{
}

/******************************************* Flow Section **********************************************/
static bool SrvSensor_Flow_Init(SrvSensorMonitorObj_TypeDef *obj)
{
}

static void SrvSensor_Flow_Sample(SrvSensorMonitorObj_TypeDef *obj)
{

}

/* noticed all sensor sampling and processing must finished in 1ms maximum */
static void SrvSensor_Sample(void)
{
    SrvSensor_Board_Sample(&SensorMonitor);
    SrvSensor_Flow_Sample(&SensorMonitor);
}

static void SrvSensor_Set_Module_Calib(uint8_t calib_module)
{
}
