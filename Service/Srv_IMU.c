/*
 * CODE: 8_B!T0
 * might need a axis remap function at the sensor init process
 */
#include "Srv_IMU.h"
#include "Dev_ICM426xx.h"
#include "HW_Def.h"
#include "Srv_OsCommon.h"
#include "debug_util.h"
#include "Bsp_SPI.h"
#include "error_log.h"
#include "Dev_Led.h"
#include "../Algorithm/Filter_Dep/filter.h"
#include <math.h>

/* use ENU coordinate */
/* IMU coordinate is x->forward y->right z->down */

/*
 * Angular Speed Over Speed Threshold
 * Angular Speed Per Millscond
 */
#define ANGULAR_ACCECLERATION_THRESHOLD 200 / 1.0f      /* angular speed accelerate from 0 to 100 deg/s in 1 Ms */
#define MOVING_DETECT_CONDITION_1 1.0                   /* unit deg/s */
#define MOVING_DETECT_CONDITION_2 (1000.0f / 3600.0f)   /* unit deg/s */
#define ANGULAR_SPEED_ACCURACY 1000

typedef void (*SrvIMU_SetDataReady_Callback)(void *obj);
typedef bool (*SrvIMU_GetDataReady_Callback)(void *obj);
typedef bool (*SrvIMU_Sample_Callback)(void *obj);
typedef IMUModuleScale_TypeDef (*SrvIMU_GetScale_Callback)(void *obj);
typedef IMU_Error_TypeDef (*SrvIMU_GetError_Callback)(void *obj);

typedef struct
{
    void *obj_ptr;
    IMUData_TypeDef *OriData_ptr;

    uint8_t acc_trip;
    uint16_t gyr_trip;

    SrvIMU_GetDataReady_Callback get_drdy;
    SrvIMU_Sample_Callback sample;
    SrvIMU_GetScale_Callback get_scale;
    SrvIMU_SetDataReady_Callback set_drdy;
    SrvIMU_GetError_Callback get_error;
}SrvIMU_InuseSensorObj_TypeDef;

static uint32_t SrvIMU_Reupdate_Statistics_CNT = 0;


/* internal variable */
/* ICM42688P Instance */
static SPI_HandleTypeDef IMU_Bus_Instance;
static DevICM426xxObj_TypeDef ICM42688PObj;
static Error_Handler SrvMPU_Error_Handle = 0;

/* IMU -> ICM42688P */
static SrvMpu_Reg_TypeDef SrvMpu_Init_Reg;
static SrvMpu_Reg_TypeDef SrvMpu_Update_Reg;

static SrvIMU_Data_TypeDef IMU_Data;
static SrvIMU_Data_TypeDef IMU_Data_Lst;
static SrvIMU_InuseSensorObj_TypeDef IMU_Obj;

/* PriIMU Butterworth filter object handle */
static BWF_Object_Handle IMU_Gyr_LPF_Handle[Axis_Sum] = {0};
static BWF_Object_Handle IMU_Acc_LPF_Handle[Axis_Sum] = {0};

/* Pri Gyro Calibration Monitor */
static SrvIMU_CalibMonitor_TypeDef Gyro_Calib_Monitor;

/************************************************************************ Error Tree Item ************************************************************************/
static void SrvIMU_Dev_Filter_InitError(int16_t code, uint8_t *p_arg, uint16_t size);
static void SrvIMU_Dev_InitError(int16_t code, uint8_t *p_arg, uint16_t size);
static void SrvIMU_AllModule_InitError(int16_t code, uint8_t *p_arg, uint16_t size);
static void SrvIMU_Sample_Undrdy(uint8_t *p_arg, uint16_t size);

static Error_Obj_Typedef SrvIMU_ErrorList[] = {
    {
        .out = true,
        .log = false,
        .prc_callback = NULL,
        .code = SrvIMU_Dev_Detect_Error,
        .desc = "IMU None Detected\r\n",
        .proc_type = Error_Proc_Ignore,
        .prc_data_stream = {
            .p_data = NULL,
            .size = 0,
        },
    },
    {
        .out = true,
        .log = false,
        .prc_callback = SrvIMU_Dev_Filter_InitError,
        .code = SrvIMU_IMU_Filter_Init_Error,
        .desc = "Pri IMU Filter Init Failed\r\n",
        .proc_type = Error_Proc_Ignore,
        .prc_data_stream = {
            .p_data = NULL,
            .size = 0,
        },
    },
    {
        .out = true,
        .log = false,
        .prc_callback = NULL,
        .code = SrvIMU_CSPin_Init_Error,
        .desc = "Pri CS Pin Init Failed\r\n",
        .proc_type = Error_Proc_Ignore,
        .prc_data_stream = {
            .p_data = NULL,
            .size = 0,
        },
    },
    {
        .out = true,
        .log = false,
        .prc_callback = NULL,
        .code = SrvIMU_ExtiPin_Init_Error,
        .desc = "Pri Ext Pin Init Failed\r\n",
        .proc_type = Error_Proc_Ignore,
        .prc_data_stream = {
            .p_data = NULL,
            .size = 0,
        },
    },
    {
        .out = true,
        .log = false,
        .prc_callback = NULL,
        .code = SrvIMU_Bus_Init_Error,
        .desc = "Pri Bus Init Failed\r\n",
        .proc_type = Error_Proc_Ignore,
        .prc_data_stream = {
            .p_data = NULL,
            .size = 0,
        },
    },
    {
        .out = true,
        .log = false,
        .prc_callback = SrvIMU_Dev_InitError,
        .code = SrvIMU_Dev_Init_Error,
        .desc = "Pri Dev Init Failed\r\n",
        .proc_type = Error_Proc_Immd,
        .prc_data_stream = {
            .p_data = NULL,
            .size = 0,
        },
    },
    {
        .out = true,
        .log = false,
        .prc_callback = SrvIMU_AllModule_InitError,
        .code = SrvIMU_AllModule_Init_Error,
        .desc = "All IMU Module Init Failed\r\n",
        .proc_type = Error_Proc_Immd,
        .prc_data_stream = {
            .p_data = NULL,
            .size = 0,
        },
    },
};
/************************************************************************ Error Tree Item ************************************************************************/

/* external function */
static SrvIMU_ErrorCode_List SrvIMU_Init(void);
static bool SrvIMU_Sample(void);
static bool SrvIMU_Get_Data(SrvIMU_Data_TypeDef *data);
static void SrvIMU_ErrorProc(void);
static float SrvIMU_Get_MaxAngularSpeed_Diff(void);
static void SrvIMU_Calib_GyroZeroOffset(SrvIMU_CalibMonitor_TypeDef *cali_monitor, float *gyr);
static bool SrvIMU_Get_Range(SrvIMU_Range_TypeDef *range);

/* internal function */
static int8_t SrvIMU_IMU_Init(void);
static void SrvIMU_IMU_ExtiCallback(void);
static void SrvIMU_IMU_CS_Ctl(bool state);
static bool SrvIMU_IMU_BusTrans_Rec(uint8_t *Tx, uint8_t *Rx, uint16_t size);

static bool SrvIMU_Detect_AngularOverSpeed(float angular_speed, float lst_angular_speed, float ms_diff);

SrvIMU_TypeDef SrvIMU = {
    .init = SrvIMU_Init,
    .sample = SrvIMU_Sample,
    .get_data = SrvIMU_Get_Data,
    .get_range = SrvIMU_Get_Range,
    .error_proc = SrvIMU_ErrorProc,
    .get_max_angular_speed_diff = SrvIMU_Get_MaxAngularSpeed_Diff,
};

static SrvIMU_ErrorCode_List SrvIMU_Init(void)
{
    FilterParam_Obj_TypeDef *Gyr_Filter_Ptr = NULL;
    FilterParam_Obj_TypeDef *Acc_Filter_Ptr = NULL;

    CREATE_FILTER_PARAM_OBJ(Gyr, 3, 50Hz, 1K, Gyr_Filter_Ptr);
    CREATE_FILTER_PARAM_OBJ(Acc, 2, 50Hz, 1K, Acc_Filter_Ptr);

    memset(&IMU_Obj, 0, sizeof(IMU_Obj));

    memset(&IMU_Data, 0, sizeof(IMU_Data));
    memset(&IMU_Data_Lst, 0, sizeof(IMU_Data_Lst));

    /* init gyro calibration monitor */
    memset(&Gyro_Calib_Monitor, 0, sizeof(Gyro_Calib_Monitor));
    Gyro_Calib_Monitor.state = Calib_Ready;
    Gyro_Calib_Monitor.calib_cycle = GYR_STATIC_CALIB_CYCLE;
    Gyro_Calib_Monitor.cur_cycle = 0;
     
    /* create error log handle */
    SrvMPU_Error_Handle = ErrorLog.create("SrvIMU_Error");

    /* regist all error to the error tree */
    ErrorLog.registe(SrvMPU_Error_Handle, SrvIMU_ErrorList, sizeof(SrvIMU_ErrorList) / sizeof(SrvIMU_ErrorList[0]));

    SrvIMU_ErrorCode_List PriIMU_Init_State = SrvIMU_IMU_Init();

    SrvMpu_Init_Reg.val = 0;
    SrvMpu_Update_Reg.val = 0;

    if (PriIMU_Init_State == SrvIMU_No_Error)
    {
        SrvMpu_Init_Reg.sec.Pri_State = true;

        /* init filter */
        IMU_Gyr_LPF_Handle[Axis_X] = Butterworth.init(Gyr_Filter_Ptr);
        IMU_Gyr_LPF_Handle[Axis_Y] = Butterworth.init(Gyr_Filter_Ptr);
        IMU_Gyr_LPF_Handle[Axis_Z] = Butterworth.init(Gyr_Filter_Ptr);

        IMU_Acc_LPF_Handle[Axis_X] = Butterworth.init(Acc_Filter_Ptr);
        IMU_Acc_LPF_Handle[Axis_Y] = Butterworth.init(Acc_Filter_Ptr);
        IMU_Acc_LPF_Handle[Axis_Z] = Butterworth.init(Acc_Filter_Ptr);

        for(uint8_t i = Axis_X; i < Axis_Sum; i++)
        {
            if( (IMU_Gyr_LPF_Handle[i] == 0) || 
                (IMU_Acc_LPF_Handle[i] == 0))
            {
                ErrorLog.trigger(SrvMPU_Error_Handle, SrvIMU_IMU_Filter_Init_Error, NULL, 0);
                return SrvIMU_IMU_Filter_Init_Error;
            }
        }
    }
    else
        ErrorLog.trigger(SrvMPU_Error_Handle, PriIMU_Init_State, (uint8_t *)&IMU_Obj, sizeof(IMU_Obj));

    if(!SrvMpu_Init_Reg.sec.Pri_State)
        return SrvIMU_AllModule_Init_Error;

    return SrvIMU_Dev_Init_Error;
}

/* init primary IMU Device */
/* consider use spi dma sample raw data */
static SrvIMU_ErrorCode_List SrvIMU_IMU_Init(void)
{
    /* primary IMU Pin & Bus Init */
    if (!BspGPIO.out_init(IMU_CSPin))
        return SrvIMU_CSPin_Init_Error;

    if (!BspGPIO.exti_init(IMU_INTPin, SrvIMU_IMU_ExtiCallback))
        return SrvIMU_ExtiPin_Init_Error;

    if (!BspSPI.init(IMU_BusCfg, &IMU_Bus_Instance))
        return SrvIMU_Bus_Init_Error;

    DevICM426xx.pre_init(&ICM42688PObj,
                            SrvIMU_IMU_CS_Ctl,
                            SrvIMU_IMU_BusTrans_Rec,
                            SrvOsCommon.delay_ms,
                            SrvOsCommon.get_os_ms);

    DevICM426xx.config(&ICM42688PObj,
                        ICM426xx_SampleRate_1K,
                        ICM426xx_Acc_4G,
                        ICM426xx_Gyr_500DPS);

    IMU_Obj.obj_ptr = &ICM42688PObj;
    IMU_Obj.OriData_ptr = &(ICM42688PObj.OriData);
    IMU_Obj.acc_trip = ICM42688PObj.PHY_AccTrip_Val;
    IMU_Obj.gyr_trip = ICM42688PObj.PHY_GyrTrip_Val;

    IMU_Obj.set_drdy = (SrvIMU_SetDataReady_Callback)(DevICM426xx.set_ready);
    IMU_Obj.get_drdy = (SrvIMU_GetDataReady_Callback)(DevICM426xx.get_ready);
    IMU_Obj.get_scale = (SrvIMU_GetScale_Callback)(DevICM426xx.get_scale);
    IMU_Obj.sample = (SrvIMU_Sample_Callback)(DevICM426xx.sample);
    IMU_Obj.get_error = (SrvIMU_GetError_Callback)(DevICM426xx.get_error);

    if (!DevICM426xx.init(&ICM42688PObj))
        return SrvIMU_Dev_Init_Error;

    return SrvIMU_No_Error;
}

/* input true selected / false deselected */
static void SrvIMU_IMU_CS_Ctl(bool state)
{
    BspGPIO.write(IMU_CSPin, state);
}

static bool SrvIMU_IMU_BusTrans_Rec(uint8_t *Tx, uint8_t *Rx, uint16_t size)
{
    return BspSPI.trans_receive(&IMU_Bus_Instance, Tx, Rx, size, IMU_Commu_TimeOut);
}

/************************************************************ Module Sample API Function *****************************************************************************/
static SrvIMU_SampleErrorCode_List SrvIMU_DataCheck(IMUData_TypeDef *data, uint8_t acc_range, uint16_t gyr_range)
{
    float Acc_Range_Max = acc_range * MPU_RANGE_MAX_THRESHOLD;
    float Gyr_Range_Max = gyr_range * MPU_RANGE_MAX_THRESHOLD;
    float Acc_Range_Min = acc_range * MPU_RANGE_MIN_THRESHOLD;
    float Gyr_Range_Min = gyr_range * MPU_RANGE_MIN_THRESHOLD;

    for (uint8_t axis = Axis_X; axis < Axis_Sum; axis++)
    {
        /* over range chack */
        {
            /* check acc data range */
            if (fabs(data->acc_flt[axis]) > Acc_Range_Max)
                return SrvIMU_Sample_Data_Acc_OverRange;

            /* check gyr data range */
            if (fabs(data->gyr_flt[axis]) > Gyr_Range_Max)
                return SrvIMU_Sample_Data_Gyr_OverRange;
        }

        /* blunt data check */
        {
            if (data->acc_int_lst[axis] != 0)
            {
                if ((fabs(data->acc_flt[axis]) <= Acc_Range_Min) && (data->acc_int[axis] == data->acc_int_lst[axis]))
                {
                    data->acc_blunt_cnt[axis]++;

                    if (data->acc_blunt_cnt[axis] >= IMU_BLUNT_SAMPLE_CNT)
                    {
                        return SrvIMU_Sample_Data_Acc_Blunt;
                    }
                }
                else
                    data->acc_blunt_cnt[axis] = 0;
            }

            if (data->gyr_int_lst[axis] != 0)
            {
                if ((fabs(data->gyr_flt[axis]) <= Gyr_Range_Min) && (data->gyr_int[axis] == data->gyr_int_lst[axis]))
                {
                    data->gyr_blunt_cnt[axis]++;

                    if (data->gyr_blunt_cnt[axis] >= IMU_BLUNT_SAMPLE_CNT)
                    {
                        return SrvIMU_Sample_Data_Gyr_Blunt;
                    }
                }
                else
                    data->gyr_blunt_cnt[axis] = 0;
            }
        }
    }

    return SrvIMU_Sample_NoError;
}

/* dynamic comput zero offset */
static void SrvIMU_Calib_GyroZeroOffset(SrvIMU_CalibMonitor_TypeDef *cali_monitor, float *gyr)
{
    uint8_t i = 0;

    if ((cali_monitor->state == NULL) || (gyr == NULL))
        return;

    switch (cali_monitor->state)
    {
        case Calib_Ready:
            cali_monitor->state = Calib_InProcess;
            cali_monitor->cur_cycle = 1;
        
            for (i = Axis_X; i < Axis_Sum; i++)
            {
                cali_monitor->avg[i] += gyr[i];
                cali_monitor->max[i] = gyr[i];
                cali_monitor->min[i] = gyr[i];
            }
            break;

        case Calib_InProcess:
            cali_monitor->cur_cycle ++;

            if (cali_monitor->calib_cycle == cali_monitor->cur_cycle)
            {
                cali_monitor->state = Calib_Ready;
                
                for (i = Axis_X; i < Axis_Sum; i++)
                {
                    cali_monitor->avg[i] /= cali_monitor->calib_cycle;

                    if (fabs(cali_monitor->avg[i]) >= MOVING_DETECT_CONDITION_2)
                    {
                        cali_monitor->state = Calib_Failed;
                        return;
                    }
                }

                /* after all axis checked set zero offset */
                for (i = Axis_X; i < Axis_Sum; i ++)
                {
                    cali_monitor->z_offset[i] += cali_monitor->avg[i];
                    cali_monitor->avg[i] = 0.0f;
                    cali_monitor->max[i] = 0.0f;
                    cali_monitor->min[i] = 0.0f;
                }

                /* comput zero offset */
                cali_monitor->state = Calib_Ready;
                cali_monitor->calib_cycle = GYR_STATIC_CALIB_CYCLE;
                break;
            }

            for (i = Axis_X; i < Axis_Sum; i++)
            {
                if (gyr[i] > cali_monitor->max[i])
                    cali_monitor->max[i] = gyr[i];

                if (gyr[i] < cali_monitor->min[i])
                    cali_monitor->min[i] = gyr[i];

                if (fabs(cali_monitor->max[i] - cali_monitor->min[i]) >= MOVING_DETECT_CONDITION_1)
                {
                    cali_monitor->state = Calib_Failed;
                    return;
                }

                cali_monitor->avg[i] += gyr[i];
            }
            break;

        case Calib_Failed:
            for (i = Axis_X; i < Axis_Sum; i ++)
            {
                cali_monitor->avg[i] = 0.0f;
                cali_monitor->max[i] = 0.0f;
                cali_monitor->min[i] = 0.0f;
            }

            cali_monitor->state = Calib_Ready;
            cali_monitor->calib_cycle = GYR_STATIC_CALIB_CYCLE;
            break;

        default: break;
    }
}

static bool SrvIMU_Sample(void)
{
    static uint32_t Sample_Rt_Lst = 0;
    uint8_t i = Axis_X;
    bool sample_state = true;
    IMUModuleScale_TypeDef imu_scale;
    float Sample_MsDiff = 0.0f;

    /* don`t use error tree down below it may decrease code efficient */
    /* trigger error directly when sampling */
    /* lock fus data */
    SrvMpu_Update_Reg.sec.Fus_State = true;

    /* pri imu init successed */
    if (SrvMpu_Init_Reg.sec.Pri_State)
    {
        imu_scale = IMU_Obj.get_scale(IMU_Obj.obj_ptr);
        IMU_Data.acc_scale = imu_scale.acc_scale;
        IMU_Data.gyr_scale = imu_scale.gyr_scale;

        /* pri imu module data ready triggered */
        if (IMU_Obj.get_drdy(IMU_Obj.obj_ptr) && IMU_Obj.sample(IMU_Obj.obj_ptr))
        {
            /* lock */
            SrvMpu_Update_Reg.sec.Pri_State = true;

            IMU_Data.cycle_cnt++;
            IMU_Data.time_stamp = IMU_Obj.OriData_ptr->time_stamp;

            /* check Primary IMU module Sample is correct or not */
            if (Sample_Rt_Lst && (IMU_Data.time_stamp <= Sample_Rt_Lst))
                sample_state = false;

            if (sample_state)
            {
                /* update pri imu data */
                IMU_Data.tempera = IMU_Obj.OriData_ptr->temp_flt;

                /* Pri imu data validation check */
                IMU_Data.error_code = SrvIMU_DataCheck(IMU_Obj.OriData_ptr, IMU_Obj.acc_trip, IMU_Obj.gyr_trip);
                Sample_MsDiff = IMU_Data.time_stamp - Sample_Rt_Lst;

                for (i = Axis_X; i < Axis_Sum; i++)
                {
                    IMU_Data.org_acc[i] = IMU_Obj.OriData_ptr->acc_flt[i];
                    IMU_Data.org_gyr[i] = IMU_Obj.OriData_ptr->gyr_flt[i] - Gyro_Calib_Monitor.z_offset[i];

                    /* filted imu data */
                    IMU_Data.flt_gyr[i] = Butterworth.update(IMU_Gyr_LPF_Handle[i], IMU_Data.org_gyr[i]);
                    IMU_Data.flt_acc[i] = Butterworth.update(IMU_Acc_LPF_Handle[i], IMU_Data.org_acc[i]);

                    /* update last time value */
                    if ((IMU_Data.error_code != SrvIMU_Sample_Data_Acc_OverRange) &&
                        (IMU_Data.error_code != SrvIMU_Sample_Data_Gyr_OverRange))
                    {
                        IMU_Obj.OriData_ptr->acc_int_lst[i] = IMU_Obj.OriData_ptr->acc_int[i];
                        IMU_Obj.OriData_ptr->gyr_int_lst[i] = IMU_Obj.OriData_ptr->gyr_int[i];
                    
                        /* over angular accelerate error detect */
                        if (SrvIMU_Detect_AngularOverSpeed(IMU_Data.org_gyr[i], IMU_Data_Lst.org_gyr[i], Sample_MsDiff) && \
                            (IMU_Data.error_code != SrvIMU_Sample_Data_Gyr_Blunt))
                            IMU_Data.error_code = SrvIMU_Sample_Over_Angular_Accelerate;
                    }
                }

                /* unlock */
                SrvMpu_Update_Reg.sec.Pri_State = false;
                IMU_Data_Lst = IMU_Data;
            }
        }
        else
        {
            SrvIMU_Sample_Undrdy(NULL, 0);
            sample_state = false;
            IMU_Data_Lst.error_code = SrvIMU_Sample_Module_UnReady;
        }

        Sample_Rt_Lst = IMU_Data.time_stamp;
    }
    else
        sample_state = false;
    
    /* update calibration state */
    SrvIMU_Calib_GyroZeroOffset(&Gyro_Calib_Monitor, IMU_Data.org_gyr);
    
    /* unlock fus data */
    SrvMpu_Update_Reg.sec.Fus_State = false;

    return sample_state;
}

static bool SrvIMU_Get_Range(SrvIMU_Range_TypeDef *range)
{
    if(SrvMpu_Init_Reg.sec.Pri_State && range)
    {
        range->Acc = IMU_Obj.acc_trip;
        range->Gyr = IMU_Obj.gyr_trip;
        
        return true; 
    }
    
    return false;
}

static bool SrvIMU_Get_Data(SrvIMU_Data_TypeDef *data)
{
    SrvIMU_Data_TypeDef imu_data_tmp;

    memset(&imu_data_tmp, 0, IMU_DATA_SIZE);

    if(data == NULL)
        return false;

reupdate_imu:
    if (!SrvMpu_Update_Reg.sec.Pri_State)
    {
        memcpy(&imu_data_tmp, &IMU_Data, IMU_DATA_SIZE);
    }
    else
        goto reupdate_imu_statistics;

    memcpy(data, &imu_data_tmp, sizeof(SrvIMU_Data_TypeDef));
    return true;

reupdate_imu_statistics:
    SrvIMU_Reupdate_Statistics_CNT ++;
    goto reupdate_imu;
}

static float SrvIMU_Get_MaxAngularSpeed_Diff(void)
{
    uint16_t angular_diff = 0;

    if (SrvMpu_Init_Reg.sec.Pri_State)
    {
        // angular_diff = (uint16_t)(DevMPU6000.get_gyr_angular_speed_diff(&MPU6000Obj) * ANGULAR_SPEED_ACCURACY);
    }

    return angular_diff / (float)ANGULAR_SPEED_ACCURACY;
}

static bool SrvIMU_Detect_AngularOverSpeed(float angular_speed, float lst_angular_speed, float ms_diff)
{
    uint16_t AngularSpeed_Diff = (uint16_t)(fabs(angular_speed - lst_angular_speed) / ms_diff * ANGULAR_SPEED_ACCURACY);

    if (AngularSpeed_Diff >= (uint16_t)(ANGULAR_ACCECLERATION_THRESHOLD * ANGULAR_SPEED_ACCURACY))
        return true;

    return false;
}

static void SrvIMU_ErrorProc(void)
{
    ErrorLog.proc(SrvMPU_Error_Handle);
}

/************************************************************ DataReady Pin Exti Callback *****************************************************************************/
static void SrvIMU_IMU_ExtiCallback(void)
{
    if (SrvMpu_Init_Reg.sec.Pri_State)
        IMU_Obj.set_drdy(IMU_Obj.obj_ptr);
}

/*************************************************************** Error Process Callback *******************************************************************************/
static void SrvIMU_Dev_Filter_InitError(int16_t code, uint8_t *p_arg, uint16_t size)
{
    ErrorLog.add_desc("IMU Pri Filter Init Error\r\n");
}

static void SrvIMU_Dev_InitError(int16_t code, uint8_t *p_arg, uint16_t size)
{
    SrvIMU_InuseSensorObj_TypeDef *InUseObj_Ptr = NULL;

    if(p_arg && size)
    {
        InUseObj_Ptr = (SrvIMU_InuseSensorObj_TypeDef *)p_arg;
        
        ErrorLog.add_desc("error code:   %d\r\n", InUseObj_Ptr->get_error(InUseObj_Ptr->obj_ptr).code);
        ErrorLog.add_desc("error func:   %s\r\n", InUseObj_Ptr->get_error(InUseObj_Ptr->obj_ptr).function);
        ErrorLog.add_desc("error line:   %d\r\n", InUseObj_Ptr->get_error(InUseObj_Ptr->obj_ptr).line);
        ErrorLog.add_desc("error reg:    %02x\r\n", InUseObj_Ptr->get_error(InUseObj_Ptr->obj_ptr).tar_reg);
        ErrorLog.add_desc("error reg rx: %02x\r\n", InUseObj_Ptr->get_error(InUseObj_Ptr->obj_ptr).reg_r_val);
        ErrorLog.add_desc("error reg tx: %02x\r\n", InUseObj_Ptr->get_error(InUseObj_Ptr->obj_ptr).reg_t_val);

        ErrorLog.add_desc("\r\n");
    }
}

static void SrvIMU_AllModule_InitError(int16_t code, uint8_t *p_arg, uint16_t size)
{
    if(p_arg && size)
        (*(uint32_t *)p_arg)++;
}

static void SrvIMU_Sample_Undrdy(uint8_t *p_arg, uint16_t size)
{
}
