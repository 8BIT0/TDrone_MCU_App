#include "Task_Navi.h"
#include "DataPipe.h"
#include "Srv_OsCommon.h"
#include "Srv_DataHub.h"
#include "Srv_IMU.h"
#include "Srv_Baro.h"
#include "Srv_Mag.h"
#include "DataPipe.h"
#include "MadgwickAHRS.h"
#include "Alt_est.h"
#include "math_util.h"
#include <Eigen>
#include <stdio.h>
#include <iostream>

#include "HW_Def.h"
#include "debug_util.h"

#define NAVI_TAG " "
#define NAVI_INFO(fmt, ...) Debug_Print(&DebugPort, NAVI_TAG, fmt, ##__VA_ARGS__)

#define BARO_SAMPLE_RATE 200    /* unit: Hz */
#define MAG_SAMPLE_RATE  100    /* unit: Hz */
#define FLOW_SAMPLE_RATE 50     /* unit: Hz */
#define TOF_SAMPLE_RATE  100    /* unit: Hz */
#define RATE_To_PERIOD(rate)    ((uint32_t)(1000 / rate))

using namespace std;
using namespace Eigen;

/* IMU coordinate is x->forward y->right z->down */
/*
    x Axis -> Roll  clock wise rotate positive
    y Axis -> Pitch noise up positive
    z Axis -> Yaw   anticlock wise rotate positice
*/

/* internal variable */
DataPipe_CreateDataObj(NaviData_TypeDef, Smp_Navi);
static NaviData_TypeDef NaviData;

/* internal function */
static void TaskNavi_Module_Sample(uint32_t sys_time);
static void TaskNavi_Update_BaroAltEst(float baro, float acc_dif, RelMov_TypeDef *baro_alt);
static Matrix<float, 3, 1> BodyFixAcc_Convert2_GeodeticAcc(float pitch, float roll, float yaw, float *acc);
static bool TaskNavi_ModuleSample_Trigger(uint32_t sys_time, uint32_t *sample_time, uint32_t period);

/* internal vriable */
TaskNavi_Monitor_TypeDef TaskNavi_Monitor;

/* data structure definition */

void TaskNavi_Init(uint32_t period)
{    
    memset(&TaskNavi_Monitor,       0, sizeof(TaskNavi_Monitor_TypeDef));
    memset(&NaviData,               0, sizeof(NaviData_TypeDef));

    TaskNavi_Monitor.max_sample_period = (1000 / period);
    TaskNavi_Monitor.period = period;

    /* data pipe init */
    memset(DataPipe_DataObjAddr(Smp_Navi), 0, DataPipe_DataSize(Smp_Navi));
    Navi_smp_DataPipe.data_addr = (uint32_t)DataPipe_DataObjAddr(Smp_Navi);
    Navi_smp_DataPipe.data_size = DataPipe_DataSize(Smp_Navi);
    DataPipe_Enable(&Navi_smp_DataPipe);

    /* IMU  init */
    /* Max sample rate 1KHz */
    TaskNavi_Monitor.init_state.bit.imu = SrvIMU.init();
    TaskNavi_Monitor.imu_sample_period = TaskNavi_Monitor.max_sample_period;

    /* Baro init */
    /* Max sample rate 100Hz */
    TaskNavi_Monitor.init_state.bit.baro = SrvBaro.init();
    TaskNavi_Monitor.baro_sample_period = RATE_To_PERIOD(BARO_SAMPLE_RATE);

    /* Mag  init */
    /* Max sample rate 100Hz */
    TaskNavi_Monitor.init_state.bit.mag = SrvMag.init();
    TaskNavi_Monitor.mag_sample_period = RATE_To_PERIOD(MAG_SAMPLE_RATE);

    /* Flow init */
    TaskNavi_Monitor.init_state.bit.flow = false;
    TaskNavi_Monitor.flow_sample_period = RATE_To_PERIOD(FLOW_SAMPLE_RATE);

    /* ToF  init */
    TaskNavi_Monitor.init_state.bit.tof = false;
    TaskNavi_Monitor.tof_sample_period = RATE_To_PERIOD(TOF_SAMPLE_RATE);

    /* pipe sensor init state to data hub */

    /* eigen test */
    Matrix<float, 2, 3> matrix_23;
    Matrix<float, 3, 1> vd_3d;
    Matrix<float, 2, 1> result2;

    matrix_23 << 1, 2, 3, 4, 5, 6;
    vd_3d << 3, 2, 1;

    result2 = matrix_23 * vd_3d;
    NAVI_INFO("Eigen Test\r\n");
    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 1; j++)
        {
            NAVI_INFO("%f\t", result2(i, j));
        }
        NAVI_INFO("\r\n");
    }
    /* eigen test */
}

void TaskNavi_Core(void const *arg)
{
    uint32_t prv_time = SrvOsCommon.get_os_ms();
    uint32_t sys_time = 0;
    AlgoAttData_TypeDef algo_att;

    MadgwickAHRSInit(&algo_att);
    Matrix<float, 3, 1> EM_GeoAcc; /* eigen matrix geodetic coordinate Acc */

    while(1)
    {
        sys_time = SrvOsCommon.get_os_ms();

        /* sample sensor */
        TaskNavi_Module_Sample(sys_time);

        /* set navigation data */

        /* pipe data to data hub */
        memcpy(DataPipe_DataObjAddr(Smp_Navi), &NaviData, sizeof(NaviData_TypeDef));
        DataPipe_SendTo(&Navi_smp_DataPipe, &Navi_hub_DataPipe);

        /* check imu data update freq on test */
        SrvOsCommon.precise_delay(&prv_time, TaskNavi_Monitor.period);
    }
}

static void TaskNavi_Update_BaroAltEst(float baro, float acc_dif, RelMov_TypeDef *baro_alt)
{

}

static Matrix<float, 3, 1> BodyFixAcc_Convert2_GeodeticAcc(float pitch, float roll, float yaw, float *acc)
{
    Matrix<float, 3, 1> tmp;
    Matrix<float, 3, 1> Acc_in;
    Matrix<float, 3, 3> EM_Cbn; /* eigen matrix Cbn */
    M_Cbn_TypeDef Cbn;

    /* init matrix */
    tmp.setZero();

    if (acc)
    {
        Cbn = AttConvert2Cbn(Deg2Rad(pitch), Deg2Rad(roll), Deg2Rad(yaw));
        for (uint8_t c, r = 0; c < 3; c ++)
        {
            Acc_in(c, 0) = acc[c];
            for (r = 0; r < 3; r ++)
            {
                EM_Cbn(c, r) = Cbn.matrix[c][r];
            }
        }

        tmp = EM_Cbn * Acc_in;
    }

    return tmp;
}

static void TaskNavi_Module_Sample(uint32_t sys_time)
{
    TaskNavi_Monitor.sample_state.val = 0;
    SrvIMU_Data_TypeDef imu_data;
    SrvBaro_Data_TypeDef baro_data;
    SrvMag_Data_TypeDef mag_data;

    memset(&imu_data,  0, sizeof(SrvIMU_Data_TypeDef));
    memset(&baro_data, 0, sizeof(SrvBaro_Data_TypeDef));
    memset(&mag_data,  0, sizeof(SrvMag_Data_TypeDef));

    /* sample imu */
    if (TaskNavi_ModuleSample_Trigger(sys_time, &TaskNavi_Monitor.imu_sampled_time, TaskNavi_Monitor.imu_sample_period) && SrvIMU.sample())
    {
        TaskNavi_Monitor.sample_state.bit.imu = SrvIMU.get(&imu_data);

        if (TaskNavi_Monitor.sample_state.bit.imu)
        {
            /* pipe raw imu data */
        }
    }

    /* sample baro */
    if (TaskNavi_ModuleSample_Trigger(sys_time, &TaskNavi_Monitor.baro_sampled_time, TaskNavi_Monitor.baro_sample_period) && SrvBaro.sample())
    {
        TaskNavi_Monitor.sample_state.bit.baro = SrvBaro.get(&baro_data);

        if (TaskNavi_Monitor.sample_state.bit.baro)
        {
            /* pipe raw baro data */
        }
    }

    /* sample mag */
    if (TaskNavi_ModuleSample_Trigger(sys_time, &TaskNavi_Monitor.mag_sampled_time, TaskNavi_Monitor.mag_sample_period) && SrvMag.sample())
    {
        TaskNavi_Monitor.sample_state.bit.mag = SrvMag.get(&mag_data);

        if (TaskNavi_Monitor.sample_state.bit.mag)
        {
            /* pipe raw mag data */
        }
    }

    /* sample ToF */

    /* sample Optical Flow */

    /* fill navi data */
    NaviData.time_stamp = sys_time;
    for (uint8_t i = 0; i < Axis_Sum; i++)
    {
        NaviData.acc[i] = imu_data.flt_acc[i];
        NaviData.gyr[i] = imu_data.flt_gyr[i];
        NaviData.mag[i] = mag_data.fit_mag[i];
    }

    NaviData.baro_alt  = baro_data.alt;
    NaviData.baro_pres = baro_data.pressure;
}

static bool TaskNavi_ModuleSample_Trigger(uint32_t sys_time, uint32_t *sample_time, uint32_t period)
{
    if (sample_time == NULL)
        return false;

    if ((*sample_time == 0) || (sys_time >= *sample_time))
    {
        *sample_time = sys_time + period;
        return true;
    }

    return false;
}
