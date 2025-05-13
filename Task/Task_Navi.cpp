#include "Task_Navi.h"
#include "DataPipe.h"
#include "Srv_OsCommon.h"
#include "Srv_DataHub.h"
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

using namespace std;
using namespace Eigen;

/* IMU coordinate is x->forward y->right z->down */
/*
    x Axis -> Roll  clock wise rotate positive
    y Axis -> Pitch noise up positive
    z Axis -> Yaw   anticlock wise rotate positice
*/

/* internal function */
static void TaskNavi_Update_BaroAltEst(float baro, float acc_dif, RelMov_TypeDef *baro_alt);
static Matrix<float, 3, 1> BodyFixAcc_Convert2_GeodeticAcc(float pitch, float roll, float yaw, float *acc);

/* internal vriable */
TaskNavi_Monitor_TypeDef TaskNavi_Monitor;

/* data structure definition */
DataPipe_CreateDataObj(IMUAtt_TypeDef, Navi_Attitude);
DataPipe_CreateDataObj(RelMov_TypeDef, Navi_Altitude);

void TaskNavi_Init(uint32_t period)
{
    memset(&TaskNavi_Monitor, 0, sizeof(TaskNavi_Monitor_TypeDef));

    /* init DataPipe */
    memset(&Attitude_smp_DataPipe, 0, sizeof(Attitude_smp_DataPipe));

    /* init sample */
    SrvSensor.init();
    
    /* pipe sensor init state to data hub */


    memset(DataPipe_DataObjAddr(Navi_Attitude), 0, DataPipe_DataSize(Navi_Attitude));
    memset(DataPipe_DataObjAddr(Navi_Altitude), 0, DataPipe_DataSize(Navi_Altitude));

    Attitude_smp_DataPipe.data_addr = (uint32_t)DataPipe_DataObjAddr(Navi_Attitude);
    Attitude_smp_DataPipe.data_size = DataPipe_DataSize(Navi_Attitude);
    DataPipe_Enable(&Attitude_smp_DataPipe);

    Altitude_smp_DataPipe.data_addr = (uint32_t)DataPipe_DataObjAddr(Navi_Altitude);
    Altitude_smp_DataPipe.data_size = DataPipe_DataSize(Navi_Altitude);
    DataPipe_Enable(&Altitude_smp_DataPipe);

    TaskNavi_Monitor.period = period;

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
    uint32_t sys_time = SrvOsCommon.get_os_ms();
    SrvSensorReg_TypeDef sensor_state;
    IMUAtt_TypeDef attitude;
    RelMov_TypeDef rel_alt;
    AlgoAttData_TypeDef algo_att;
    SrvSensorData_TypeDef sensor_data;

    memset(&sensor_data, 0, sizeof(SrvSensorData_TypeDef));
    memset(&attitude, 0, sizeof(IMUAtt_TypeDef));
    MadgwickAHRSInit(&algo_att);
    Matrix<float, 3, 1> EM_GeoAcc; /* eigen matrix geodetic coordinate Acc */
    sensor_state.val = 0;

    while(1)
    {
        SrvDataHub.get_sensor_init_state(&sensor_state);
        SrvDataHub.get_sensor_data(&sensor_data);
        
        if (!sensor_state.bit.mag)
            memset(sensor_data.scale_Mag, 0, sizeof(sensor_data.scale_Mag));

        if(sensor_state.bit.imu)
        {
            /* update Attitude */
            MadgwickAHRSupdate(&algo_att, \
                               Deg2Rad(sensor_data.scale_Gyro[Axis_X]), Deg2Rad(sensor_data.scale_Gyro[Axis_Y]), Deg2Rad(sensor_data.scale_Gyro[Axis_Z]), \
                               sensor_data.scale_Acc[Axis_X],           sensor_data.scale_Acc[Axis_Y],           sensor_data.scale_Acc[Axis_Z], \
                               sensor_data.scale_Mag[Axis_X],           sensor_data.scale_Mag[Axis_Y],           sensor_data.scale_Mag[Axis_Z]);
            
            attitude.pitch = algo_att.pitch;
            attitude.roll  = algo_att.roll;
            attitude.yaw   = algo_att.yaw;
            attitude.q0    = algo_att.q0;
            attitude.q1    = algo_att.q1;
            attitude.q2    = algo_att.q2;
            attitude.q3    = algo_att.q3;
            
            attitude.time_stamp = SrvOsCommon.get_os_ms();

            DataPipe_DataObj(Navi_Attitude) = attitude;

            /* DataPipe Attitude Data to SrvDataHub */
            DataPipe_SendTo(&Attitude_smp_DataPipe, &Attitude_hub_DataPipe);
            DataPipe_SendTo(&Attitude_smp_DataPipe, &Attitude_log_DataPipe);

            /* convert body fixed coordinate to geidetic coordinate */
            EM_GeoAcc = BodyFixAcc_Convert2_GeodeticAcc(attitude.pitch, attitude.roll, attitude.yaw, sensor_data.scale_Acc);
        }

        /* comput baro altitude */
       if (sensor_state.bit.baro && sensor_state.bit.imu)
        {
            TaskNavi_Update_BaroAltEst(sensor_data.scale_baro, (EM_GeoAcc(2, 0) + 1.0f), &rel_alt);

            DataPipe_DataObj(Navi_Altitude).time = SrvOsCommon.get_os_ms();
            DataPipe_DataObj(Navi_Altitude).pos = rel_alt.pos;
            DataPipe_DataObj(Navi_Altitude).vel = rel_alt.vel;

            DataPipe_SendTo(&Altitude_smp_DataPipe, &Altitude_hub_DataPipe);
            DataPipe_SendTo(&Altitude_smp_DataPipe, &Altitude_log_DataPipe);
        }

        /* check imu data update freq on test */
        SrvOsCommon.precise_delay(&sys_time, TaskNavi_Monitor.period);
    }
}

static void TaskNavi_Update_BaroAltEst(float baro, float acc_dif, RelMov_TypeDef *baro_alt)
{
    static bool init = false;
    RelMov_TypeDef alt;

    if (!init)
    {
        BaroAltEstimate_Init(baro, (TaskNavi_Monitor.period / 1000.0f), 0.28f, 0.00004f);
        init = true;
    }
    else
    {
        alt = BaroAltEstimate_Update(baro, acc_dif);
        if (baro_alt)
            *baro_alt = alt;
    }
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
