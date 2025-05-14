#include "Task_Navi.h"
#include "DataPipe.h"
#include "Srv_OsCommon.h"
#include "Srv_DataHub.h"
#include "Srv_IMU.h"
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

void TaskNavi_Init(uint32_t period)
{
    memset(&TaskNavi_Monitor, 0, sizeof(TaskNavi_Monitor_TypeDef));

    /* init DataPipe */
    memset(&Attitude_smp_DataPipe, 0, sizeof(Attitude_smp_DataPipe));

    /* IMU  init */
    SrvIMU.init();

    /* Baro init */

    /* Mag  init */

    /* Flow init */

    /* ToF  init */

    /* pipe sensor init state to data hub */

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
    RelMov_TypeDef rel_alt;
    AlgoAttData_TypeDef algo_att;

    MadgwickAHRSInit(&algo_att);
    Matrix<float, 3, 1> EM_GeoAcc; /* eigen matrix geodetic coordinate Acc */

    while(1)
    {


        /* check imu data update freq on test */
        SrvOsCommon.precise_delay(&sys_time, TaskNavi_Monitor.period);
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

static void TaskNavi_Module_Sample(void)
{
}
