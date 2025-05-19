/*
 * Auther: 8_B!T0
 * Baro Sensor Sample Service
 * this file still can be optimized in a big way
 * for currently only one baro sensor can be used
 */
#include "Srv_Baro.h"
#include "Srv_OsCommon.h"
#include "../FCHW_Config.h"
#include "error_log.h"
#include "bsp_iic.h"
#include "bsp_gpio.h"
#include "HW_Def.h"
#include <math.h>

#define STANDER_ATMOSPHERIC_PRESSURE (101.325f * 1000)
#define SRVBARO_SMOOTHWINDOW_SIZE 5

/* internal vriable */
SrvBaroObj_TypeDef SrvBaroObj = {
    .init_err = SrvBaro_Error_None,
    .sensor_data = NULL,
};

/* internal function */
static float SrvBaro_PessureCnvToMeter(float pa);

/************************************************************************ Error Tree Item ************************************************************************/
static Error_Handler SrvBaro_Error_Handle = 0;

static void SrvBaro_BusInitError(int16_t code, uint8_t *p_arg, uint16_t size);

static Error_Obj_Typedef SrvBaro_ErrorList[] = {
    {
        .out = true,
        .log = false,
        .prc_callback = SrvBaro_BusInitError,
        .code = SrvBaro_Error_BadSensorObj,
        .desc = "SrvBaro Bad Sensor Object\r\n",
        .proc_type = Error_Proc_Ignore,
        .prc_data_stream = {
            .p_data = NULL,
            .size = 0,
        },
    },
    {
        .out = true,
        .log = false,
        .prc_callback = SrvBaro_BusInitError,
        .code = SrvBaro_Error_BusInit,
        .desc = "SrvBaro Bus Init Failed\r\n",
        .proc_type = Error_Proc_Ignore,
        .prc_data_stream = {
            .p_data = NULL,
            .size = 0,
        },
    },
    {
        .out = true,
        .log = false,
        .prc_callback = SrvBaro_BusInitError,
        .code = SrvBaro_Error_FilterInit,
        .desc = "SrvBaro Sensor Filter Init Failed\r\n",
        .proc_type = Error_Proc_Immd, 
        .prc_data_stream = {
            .p_data = NULL,
            .size = 0,
        },
    },
    {
        .out = true,
        .log = false,
        .prc_callback = SrvBaro_BusInitError,
        .code = SrvBaro_Error_DevInit,
        .desc = "SrvBaro Sensor Device Init Failed\r\n",
        .proc_type = Error_Proc_Immd, 
        .prc_data_stream = {
            .p_data = NULL,
            .size = 0,
        },
    },
};
/************************************************************************ Error Tree Item ************************************************************************/

/* external function */
static uint8_t SrvBaro_Init(void);
static bool SrvBaro_Sample(void);
static bool SrvBaro_Get_Date(SrvBaro_Data_TypeDef *data);
static GenCalib_State_TypeList SrvBaro_Set_Calib(uint16_t cyc);
static GenCalib_State_TypeList SrvBaro_Get_Calib(void);

SrvBaro_TypeDef SrvBaro = {
    .init = SrvBaro_Init,
    .sample = SrvBaro_Sample,
    .get = SrvBaro_Get_Date,
    .set_calib = SrvBaro_Set_Calib,
    .get_calib = SrvBaro_Get_Calib,
};

static bool SrvBaro_BusInit(void)
{
    Baro_BusCfg.handle = SrvOsCommon.malloc(BaroBus_Handle_Size);
    if (Baro_BusCfg.handle == NULL)
        return false;

    Baro_BusCfg.PeriphClkInitStruct = SrvOsCommon.malloc(I2C_PeriphCLKInitType_Size);
    if (Baro_BusCfg.PeriphClkInitStruct == NULL)
    {
        SrvOsCommon.free(Baro_BusCfg.handle);
        return false;
    }

    if (!BaroBus.init(&Baro_BusCfg))
    {
        SrvOsCommon.free(Baro_BusCfg.handle);
        SrvOsCommon.free(Baro_BusCfg.PeriphClkInitStruct);
        return false;
    }
    
    return true;
}

static uint8_t SrvBaro_Init(void)
{
    /* create error log handle */
    SrvBaro_Error_Handle = ErrorLog.create("SrvBaro_Error");

    /* regist all error to the error tree */
    ErrorLog.registe(SrvBaro_Error_Handle, SrvBaro_ErrorList, sizeof(SrvBaro_ErrorList) / sizeof(SrvBaro_ErrorList[0]));

    if (!SrvBaro_BusInit())
    {
        ErrorLog.trigger(SrvBaro_Error_Handle, SrvBaro_Error_BusInit, NULL, 0);
        return SrvBaro_Error_BusInit;
    }

    SrvBaroObj.sensor_obj = SrvOsCommon.malloc(sizeof(DevBMP280Obj_TypeDef));
    SrvBaroObj.sensor_api = &DevBMP280;

    if ((SrvBaroObj.sensor_obj != NULL) && \
        (SrvBaroObj.sensor_api != NULL))
    {
        ToBMP280_OBJ(SrvBaroObj.sensor_obj)->bus_tx = NULL;
        ToBMP280_OBJ(SrvBaroObj.sensor_obj)->bus_rx = NULL;

        /* set sensor object */
        ToBMP280_OBJ(SrvBaroObj.sensor_obj)->bus_obj = (void *)&Baro_BusCfg;
        ToBMP280_OBJ(SrvBaroObj.sensor_obj)->bus_tx = (DevBMP280_IIC_Write)BaroBus.read;
        ToBMP280_OBJ(SrvBaroObj.sensor_obj)->bus_rx = (DevBMP280_IIC_Read)BaroBus.write;

        ToBMP280_OBJ(SrvBaroObj.sensor_obj)->get_tick = SrvOsCommon.get_os_ms;
        ToBMP280_OBJ(SrvBaroObj.sensor_obj)->delay_ms = SrvOsCommon.delay_ms;

        /* device init */
        if (!ToBMP280_API(SrvBaroObj.sensor_api)->init(ToBMP280_OBJ(SrvBaroObj.sensor_obj)))
        {
            ErrorLog.trigger(SrvBaro_Error_Handle, SrvBaro_Error_DevInit, NULL, 0);
            return SrvBaro_Error_DevInit;
        }

        SrvBaroObj.calib_state = Calib_Start;
        SrvBaroObj.calib_cycle = SRVBARO_DEFAULT_CALI_CYCLE;
        
        SrvBaroObj.data_size = BMP280_DataSize;
        SrvBaroObj.sensor_data = SrvOsCommon.malloc(SrvBaroObj.data_size);
    }
    else
    {
        ErrorLog.trigger(SrvBaro_Error_Handle, SrvBaro_Error_BadSensorObj, NULL, 0);
        return SrvBaro_Error_BadSensorObj;
    }

    /* filter init */
    SrvBaroObj.smoothwindow_filter_hdl = SmoothWindow.init(SRVBARO_SMOOTHWINDOW_SIZE);
    if (SrvBaroObj.smoothwindow_filter_hdl == 0)
    {
        ErrorLog.trigger(SrvBaro_Error_Handle, SrvBaro_Error_FilterInit, NULL, 0);
        return SrvBaro_Error_FilterInit;
    }

    return SrvBaro_Error_None;
}

static bool SrvBaro_Sample(void)
{
    if (SrvBaroObj.init_err != SrvBaro_Error_None)
        return false;

    if (!ToBMP280_API(SrvBaroObj.sensor_api)->sample(ToBMP280_OBJ(SrvBaroObj.sensor_obj)))
    {
        SrvBaroObj.sample_err_cnt ++;
        return false;
    }
        
    SrvBaroObj.sample_cnt ++;

    if (((SrvBaroObj.calib_state == Calib_Start) || \
        (SrvBaroObj.calib_state == Calib_InProcess)) && \
        SrvBaroObj.calib_cycle)
    {
        SrvBaroObj.pressure_add_sum += ToBMP280_OBJ(SrvBaroObj.sensor_obj)->pressure;
        SrvBaroObj.pressure_add_sum /= 2;

        SrvBaroObj.calib_cycle --;

        SrvBaroObj.calib_state = Calib_InProcess;
        if (SrvBaroObj.calib_cycle == 0)
        {
            SrvBaroObj.calib_state = Calib_Done;
            SrvBaroObj.alt_offset = SrvBaro_PessureCnvToMeter(SrvBaroObj.pressure_add_sum);
            SrvBaroObj.pressure_add_sum = 0.0f;
        }
    }

    return true;
}

static GenCalib_State_TypeList SrvBaro_Set_Calib(uint16_t cyc)
{
    if (SrvBaroObj.calib_state != Calib_InProcess)
    {
        SrvBaroObj.calib_cycle = cyc;
        SrvBaroObj.calib_state = Calib_Start;
        SrvBaroObj.alt_offset = 0.0f;
        return Calib_InProcess;
    }

    return SrvBaroObj.calib_state;
}

static GenCalib_State_TypeList SrvBaro_Get_Calib(void)
{
    return SrvBaroObj.calib_state;
}

static float SrvBaro_PessureCnvToMeter(float pa)
{
    return ((1 - pow((pa / STANDER_ATMOSPHERIC_PRESSURE), 0.1903))) * 44330;
}

static bool SrvBaro_Get_Date(SrvBaro_Data_TypeDef *data)
{
    float alt = 0.0f;

    if ((SrvBaroObj.init_err != SrvBaro_Error_None) || \
        (data == NULL) || (SrvBaroObj.sensor_data == NULL) || (SrvBaroObj.data_size == 0) || \
        !ToBMP280_API(SrvBaroObj.sensor_api)->ready(ToBMP280_OBJ(SrvBaroObj.sensor_obj)))
        return false;
    
    memset(SrvBaroObj.sensor_data, 0, BMP280_DataSize);
    *ToBMP280_DataPtr(SrvBaroObj.sensor_data) = ToBMP280_API(SrvBaroObj.sensor_api)->get_data(ToBMP280_OBJ(SrvBaroObj.sensor_obj));
    
    /* convert baro pressure to meter */
    alt = SrvBaro_PessureCnvToMeter(ToBMP280_DataPtr(SrvBaroObj.sensor_data)->scaled_press);

    data->time_stamp = ToBMP280_DataPtr(SrvBaroObj.sensor_data)->time_stamp;
    data->alt_offset = SrvBaroObj.alt_offset;
    data->alt = alt - SrvBaroObj.alt_offset;
    data->tempra = ToBMP280_DataPtr(SrvBaroObj.sensor_data)->scaled_tempra;
    data->pressure = ToBMP280_DataPtr(SrvBaroObj.sensor_data)->scaled_press;

    /* doing baro filter */
    data->alt = SmoothWindow.update(SrvBaroObj.smoothwindow_filter_hdl, data->alt);
    return true;
}

/*************************************************************** Error Process Callback *******************************************************************************/
static void SrvBaro_BusInitError(int16_t code, uint8_t *p_arg, uint16_t size)
{
    switch(code)
    {
        case SrvBaro_Error_BadSensorObj: break;
        case SrvBaro_Error_BusInit: break;
        case SrvBaro_Error_DevInit: break;
        case SrvBaro_Error_FilterInit: break;
        default:
            ErrorLog.add_desc("SrvBaro Triggered Unknow ErrorCode: %d\r\n", code);
        break;
    }
}
