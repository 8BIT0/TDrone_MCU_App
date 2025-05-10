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

/*************************************** SensorBoard Section *******************************************/
static bool SrvSensor_Board_Bus_Send(SrvSensorMonitorObj_TypeDef *obj, uint8_t *p_tx, uint16_t size)
{
    bool state = false;

    if ((obj == NULL) || \
        (p_tx == NULL) || \
        (size == 0))
        return false;

    /* CS Low */
    BspGPIO.write(To_GPIO_Obj(obj->sensorboard_cs_obj), false);

    state = BspSPI.trans(obj->sensorboard_bus_handle, p_tx, size, 100);

    /* CS High */
    BspGPIO.write(To_GPIO_Obj(obj->sensorboard_cs_obj), true);

    return state;
}

static bool SrvSensor_Board_Bus_Transmit(SrvSensorMonitorObj_TypeDef *obj, uint8_t *p_tx, uint8_t *p_rx, uint16_t size)
{
    bool state = false;

    if ((obj == NULL) || \
        (obj->sensorboard_bus_handle == NULL) || \
        !obj->sensorboard_bus_init || \
        (p_tx == NULL) || \
        (p_rx == NULL) || \
        (size == 0))
        return false;

    /* CS Low */
    BspGPIO.write(To_GPIO_Obj(obj->sensorboard_cs_obj), false);
    
    state = BspSPI.trans_receive(obj->sensorboard_bus_handle, p_tx, p_rx, size, 100);
    
    /* CS High */
    BspGPIO.write(To_GPIO_Obj(obj->sensorboard_cs_obj), true);

    return state;
}

static bool SrvSensor_Board_CheckTimeOut(void)
{
    if ((SensorMonitor.DR_SemaID == NULL) || \
        osSemaphoreWait(SensorMonitor.DR_SemaID, SENSORBOARD_DR_TIMEOUT) != osOK)
        return true;

    return false;
}

static void SrvSensor_Board_ClearSize(SrvSensorMonitorObj_TypeDef *obj, uint16_t *size)
{
    if ((size == NULL) || (*size == 0))
        return;
    
    if (*size > BUF_SIZE)
        *size = BUF_SIZE;

    memset(obj->sensor_tx_buf, 0, *size);
    memset(obj->sensor_rx_buf, 0, *size);
    *size = 0;
}

static void SrvSensor_Board_IMU_Init(SrvSensorMonitorObj_TypeDef *obj)
{
    bool state = true;
    uint16_t size = 0;
    uint16_t crc16 = 0;

    if ((obj == NULL) || !obj->sensorboard_bus_init)
        return;
    
    obj->init_state_reg.bit.imu = false;

    /* range configure */
    obj->sensor_tx_buf[size ++] = SENSORBOARD_HEADER;
    obj->sensor_tx_buf[size ++] = Reg_Sensor_Set;
    obj->sensor_tx_buf[size ++] = Comp_IMU;
    obj->sensor_tx_buf[size ++] = IMU_Set_Range;
    obj->sensor_tx_buf[size ++] = 2;
    obj->sensor_tx_buf[size ++] = IMU_Gyro_500dsp;
    obj->sensor_tx_buf[size ++] = IMU_Acc_4g;
    crc16 = Common_CRC16(&obj->sensor_tx_buf[COMPONENT_INDEX], 5);
    obj->sensor_tx_buf[size ++] = ((uint8_t *)&crc16)[1];
    obj->sensor_tx_buf[size ++] = ((uint8_t *)&crc16)[0];
    obj->sensor_tx_buf[size ++] = SENSORBOARD_ENDER;

    state = SrvSensor_Board_Bus_Send(obj, obj->sensor_tx_buf, size);
    
    /* wait DR falling edge */
    SrvSensor_Board_ClearSize(obj, &size);
    if (!state || SrvSensor_Board_CheckTimeOut())
        return;
    
    /* get scale factory */
    obj->sensor_tx_buf[size ++] = SENSORBOARD_HEADER;
    obj->sensor_tx_buf[size ++] = Reg_Qur_Setting;
    obj->sensor_tx_buf[size ++] = Comp_IMU;
    obj->sensor_tx_buf[size ++] = IMU_Set_Range;
    obj->sensor_tx_buf[size ++] = 8;
    memset(&obj->sensor_tx_buf[size + 2], 0, 8);
    size += 8;
    crc16 = Common_CRC16(&obj->sensor_tx_buf[COMPONENT_INDEX], 10);
    obj->sensor_tx_buf[size ++] = ((uint8_t *)&crc16)[1];
    obj->sensor_tx_buf[size ++] = ((uint8_t *)&crc16)[0];
    obj->sensor_tx_buf[size ++] = SENSORBOARD_ENDER;
    
    crc16 = 0;
    state &= SrvSensor_Board_Bus_Transmit(obj, obj->sensor_tx_buf, obj->sensor_tx_buf, size);
    if (state)
    {
        /* check received data crc16 */
        crc16 = Common_CRC16(&obj->sensor_rx_buf[COMPONENT_INDEX], 10);

        /* frame check */
        size = 0;
        if ((obj->sensor_rx_buf[size ++] != SENSORBOARD_HEADER) || \
            (obj->sensor_rx_buf[size ++] != Reg_Qur_Setting) || \
            (obj->sensor_rx_buf[size ++]!= Comp_IMU) || \
            (obj->sensor_rx_buf[size ++]!= IMU_Set_Range) || \
            (obj->sensor_rx_buf[size ++]!= 8) || \
            (obj->sensor_rx_buf[size += 8]!= ((uint8_t *)&crc16)[1]) || \
            (obj->sensor_rx_buf[size ++]!= ((uint8_t *)&crc16)[0]) || \
            (obj->sensor_rx_buf[size ++]!= SENSORBOARD_ENDER))
        {
            SrvSensor_Board_ClearSize(obj, &size);
            return;
        }

        /* set factory scale both acc and gyro */
        memcpy(&obj->acc_factory, &obj->sensor_rx_buf[GYRO_FACTORY_INDEX], sizeof(float));
        memcpy(&obj->gyr_factory, &obj->sensor_rx_buf[ACC_FACTORY_INDEX],  sizeof(float));
        SrvSensor_Board_ClearSize(obj, &size);
    }
    else
    {
        SrvSensor_Board_ClearSize(obj, &size);
        return;
    }

    /* filter configure */
    obj->sensor_tx_buf[size ++] = SENSORBOARD_HEADER;
    obj->sensor_tx_buf[size ++] = Reg_Sensor_Set;
    obj->sensor_tx_buf[size ++] = Comp_IMU;
    obj->sensor_tx_buf[size ++] = IMU_Set_Filter;
    obj->sensor_tx_buf[size ++] = 2;
    obj->sensor_tx_buf[size ++] = IMU_Filter_BD_258Hz;
    obj->sensor_tx_buf[size ++] = IMU_Filter_BD_258Hz;
    crc16 = Common_CRC16(&obj->sensor_tx_buf[COMPONENT_INDEX], 5);
    obj->sensor_tx_buf[size ++] = ((uint8_t *)&crc16)[1];
    obj->sensor_tx_buf[size ++] = ((uint8_t *)&crc16)[0];
    obj->sensor_tx_buf[size ++] = SENSORBOARD_ENDER;

    state &= SrvSensor_Board_Bus_Send(obj, obj->sensor_tx_buf, size);
    
    /* wait DR falling edge */
    SrvSensor_Board_ClearSize(obj, &size);
    if (!state || SrvSensor_Board_CheckTimeOut())
        return;

    /* configure IMU */
    obj->init_state_reg.bit.imu = true;
}

static void SrvSensor_Board_Baro_Init(SrvSensorMonitorObj_TypeDef *obj)
{
    uint16_t size = BUF_SIZE;
    uint16_t crc16 = 0;
    bool state = true;

    if ((obj == NULL) || !obj->sensorboard_bus_init)
        return;
    
    obj->init_state_reg.bit.baro = false;
    SrvSensor_Board_ClearSize(obj, &size);

    /* pressure over sampling configure */
    obj->sensor_tx_buf[size ++] = SENSORBOARD_HEADER;
    obj->sensor_tx_buf[size ++] = Reg_Sensor_Set;
    obj->sensor_rx_buf[size ++] = Comp_Baro;
    obj->sensor_tx_buf[size ++] = Baro_Set_OverSampling;
    obj->sensor_tx_buf[size ++] = 2;
    obj->sensor_tx_buf[size ++] = Baro_OverSampling_16x;    /* pressure    over sampling */
    obj->sensor_tx_buf[size ++] = Baro_OverSampling_1x;     /* temperature over sampling */
    crc16 = Common_CRC16(&obj->sensor_tx_buf[COMPONENT_INDEX], 5);
    obj->sensor_tx_buf[size ++] = ((uint8_t *)&crc16)[1];
    obj->sensor_tx_buf[size ++] = ((uint8_t *)&crc16)[0];
    obj->sensor_tx_buf[size ++] = SENSORBOARD_ENDER;

    state = SrvSensor_Board_Bus_Send(obj, obj->sensor_tx_buf, size);

    /* wait DR falling edge */
    SrvSensor_Board_ClearSize(obj, &size);
    if (!state || SrvSensor_Board_CheckTimeOut())
        return;

    /* configure filter */
    obj->sensor_tx_buf[size ++] = SENSORBOARD_HEADER;
    obj->sensor_tx_buf[size ++] = Reg_Sensor_Set;
    obj->sensor_rx_buf[size ++] = Comp_Baro;
    obj->sensor_tx_buf[size ++] = Baro_Set_Filter;
    obj->sensor_tx_buf[size ++] = 0x01;
    obj->sensor_tx_buf[size ++] = Baro_Filter_COEFF_16;
    crc16 = Common_CRC16(&obj->sensor_tx_buf[COMPONENT_INDEX], 4);
    obj->sensor_tx_buf[size ++] = ((uint8_t *)&crc16)[1];
    obj->sensor_tx_buf[size ++] = ((uint8_t *)&crc16)[0];
    obj->sensor_tx_buf[size ++] = SENSORBOARD_ENDER;

    state &= SrvSensor_Board_Bus_Send(obj, obj->sensor_tx_buf, size);
    
    /* wait DR falling edge */
    SrvSensor_Board_ClearSize(obj, &size);
    if (!state || SrvSensor_Board_CheckTimeOut())
        return;
    
    /* get calibration parameter */
    obj->sensor_tx_buf[size ++] = SENSORBOARD_HEADER;
    obj->sensor_tx_buf[size ++] = Reg_Qur_Setting;
    obj->sensor_rx_buf[size ++] = Comp_Baro;
    obj->sensor_tx_buf[size ++] = Baro_Req_Calib;
    obj->sensor_tx_buf[size ++] = sizeof(SrvSensor_BaroCalib_TypeDef);
    memset(&obj->sensor_tx_buf[size], 0x00, sizeof(SrvSensor_BaroCalib_TypeDef));
    size += sizeof(SrvSensor_BaroCalib_TypeDef);
    crc16 = Common_CRC16(&obj->sensor_tx_buf[COMPONENT_INDEX], (sizeof(SrvSensor_BaroCalib_TypeDef) + 2));
    obj->sensor_tx_buf[size ++] = ((uint8_t *)&crc16)[1];
    obj->sensor_tx_buf[size ++] = ((uint8_t *)&crc16)[0];
    obj->sensor_tx_buf[size ++] = SENSORBOARD_ENDER;

    state &= SrvSensor_Board_Bus_Transmit(obj, obj->sensor_tx_buf, obj->sensor_rx_buf, size);
    /* check received data */
    if (state)
    {
        size = 0;
        crc16 = Common_CRC16(&obj->sensor_rx_buf[COMPONENT_INDEX], (sizeof(SrvSensor_BaroCalib_TypeDef) + 2));
        if ((obj->sensor_rx_buf[size ++] != SENSORBOARD_HEADER) || \
            (obj->sensor_rx_buf[size ++] != Reg_Qur_Setting) || \
            (obj->sensor_rx_buf[size ++] != Comp_Baro) || \
            (obj->sensor_rx_buf[size ++] != Baro_Req_Calib) || \
            (obj->sensor_rx_buf[size ++] != sizeof(SrvSensor_BaroCalib_TypeDef)) || \
            (obj->sensor_rx_buf[size += sizeof(SrvSensor_BaroCalib_TypeDef)] != ((uint8_t *)&crc16)[1]) || \
            (obj->sensor_rx_buf[size ++] != ((uint8_t *)&crc16)[0]) || \
            (obj->sensor_rx_buf[size] != SENSORBOARD_ENDER))
        {
            SrvSensor_Board_ClearSize(obj, &size);
            return;
        }

        memcpy(&obj->baro_calib, &obj->sensor_rx_buf[PAYLOAD_INDEX], sizeof(SrvSensor_BaroCalib_TypeDef));
        SrvSensor_Board_ClearSize(obj, &size);
    }
    else
        return;

    /* configure Baro */
    obj->init_state_reg.bit.baro = true;
}

static void SrvSensor_Board_Mag_Init(SrvSensorMonitorObj_TypeDef *obj)
{
    if ((obj == NULL) || !obj->sensorboard_bus_init)
        return;

    obj->init_state_reg.bit.mag = false;

    /* configure Mag */
    // obj->init_state_reg.bit.mag = true;
}

static void SrvSensor_Board_ToF_Init(SrvSensorMonitorObj_TypeDef *obj)
{
    if ((obj == NULL) || !obj->sensorboard_bus_init)
        return;

    obj->init_state_reg.bit.tof = false;
    
    /* configure ToF */
    // obj->init_state_reg.bit.tof = true;
}

static void SrvSensor_Board_DR_Callback(void)
{
    /* release DR semaphore */
    if (SensorMonitor.DR_SemaID)
        osSemaphoreRelease(SensorMonitor.DR_SemaID);
}

static bool SrvSensor_Board_Init(SrvSensorMonitorObj_TypeDef *obj)
{
    if ((obj == NULL) || (obj->sensorboard_bus_handle != NULL))
        return false;

    /* create osSemaphore */
    osSemaphoreDef(DR_Sema);
    obj->DR_SemaID = osSemaphoreCreate(osSemaphore(DR_Sema), 1);
    if (obj->DR_SemaID == NULL)
    {
        MONITOR_INFO("DR semaphore create failed\n");
        return false;
    }

    /* init sensor board DR Pin */
    obj->sensorboard_dr_obj = MONITOR_MALLOC(sizeof(BspGPIO_Obj_TypeDef));
    if (obj->sensorboard_dr_obj == NULL)
    {
        MONITOR_INFO("sensor board dr obj malloc failed\n");
        MONITOR_FREE(obj->sensorboard_cs_obj);
        obj->sensorboard_cs_obj = NULL;
        MONITOR_FREE(obj->sensorboard_bus_handle);
        obj->sensorboard_bus_handle = NULL;
        return false;
    }

    if (!BspGPIO.exti_init(To_GPIO_Obj(obj->sensorboard_dr_obj), SrvSensor_Board_DR_Callback) || \
        !BspGPIO.set_exti_mode(To_GPIO_Obj(obj->sensorboard_dr_obj), GPIO_Exti_Falling))
    {
        MONITOR_INFO("sensor board dr init failed\n");
        MONITOR_FREE(obj->sensorboard_dr_obj);
        obj->sensorboard_dr_obj = NULL;
        MONITOR_FREE(obj->sensorboard_cs_obj);
        obj->sensorboard_cs_obj = NULL;
        MONITOR_FREE(obj->sensorboard_bus_handle);
        obj->sensorboard_bus_handle = NULL;
        return false;
    }
    obj->sensorboard_bus_init = true;

    /* snesor module config */
    SrvSensor_Board_IMU_Init(obj);
    SrvSensor_Board_Baro_Init(obj);
    SrvSensor_Board_Mag_Init(obj);
    SrvSensor_Board_ToF_Init(obj);

    return obj->init_state_reg.bit.imu;
}

static void SrvSensor_Board_Sample(SrvSensorMonitorObj_TypeDef *obj)
{
    uint8_t size = 0;
    SrvSensor_DataReqBit_TypeDef req_bit;

    req_bit.val = 0;
    if ((obj == NULL) || !obj->init_state_reg.bit.imu || !obj->sensorboard_bus_init)
        return;

    obj->fpga_req_cnt ++;

    /* send data request */
    req_bit.field.imu = true;   /* always true */
    req_bit.field.mag = obj->init_state_reg.bit.mag;
    req_bit.field.baro = obj->init_state_reg.bit.baro;
    req_bit.field.tof = obj->init_state_reg.bit.tof;

    obj->sensor_tx_buf[size ++] = SENSORBOARD_HEADER;
    obj->sensor_tx_buf[size ++] = Reg_Data_Req;
    obj->sensor_tx_buf[size ++] = req_bit.val;
    obj->sensor_tx_buf[size ++] = SENSORBOARD_ENDER;

    if (!SrvSensor_Board_Bus_Send(obj, obj->sensor_tx_buf, size))
        return;

    /* wait for DR falling edge */
    if (SrvSensor_Board_CheckTimeOut())
    {
        /* sample time out */
    }
    else
    {
        /* transmit sensor board data */
    }

    /* check data valid */

    /* filt data */
}

/******************************************* Flow Section **********************************************/
static bool SrvSensor_Flow_Init(SrvSensorMonitorObj_TypeDef *obj)
{
    if (obj == NULL)
        return false;

    obj->flow_bus_handle = NULL;

    return false;
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
