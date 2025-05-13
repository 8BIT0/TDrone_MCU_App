/*
 *  Auther : 8_B!T0
 *  this file use for moto & servo control
 *  (but currently none servo)
 */
#include "Task_Control.h"
#include "Srv_OsCommon.h"
#include "Task_Telemetry.h"
#include "Srv_DataHub.h"
#include "Srv_Actuator.h"
#include "shell_port.h"

#define CONTROL_STORAGE_SECTION_NAME "Control_Para"

#define DEFAULT_ATTITUDE_CONTROLLER_MODE CtlM_PID
#define DEFAULT_ALTITUDE_CONTROLLER_MODE CtlM_PID

#define DEFAULT_CONTROL_RATE        1.0f

#define ATTITUDE_DISARM_RANGE_MAX   10.0f
#define ATTITUDE_DISARM_RANGE_MIN   -10.0f

#define THROTTLE_CHANGE_RATE        50   /* unit value per ms */

#define DEFAULT_CONTROL_ATT_RANGE   25.0f
#define DEFAULT_CONTROL_GYR_RANGE   500.0f

#define MAX_CONTROL_RATE            1.5f
#define MIN_CONTROL_RATE            0.5f
#define MAX_CONTROL_ATT_RANGE       45.0f
#define MIN_CONTROL_ATT_RANGE       15.0f
#define MAX_CONTROL_GYR_RANGE       800.0f
#define MIN_CONTROL_GYR_RANGE       200.0f

#define Check_Control_Rate(x)       ((x > MAX_CONTROL_RATE) ? MAX_CONTROL_RATE : ((x < MIN_CONTROL_RATE) ? MIN_CONTROL_RATE : x))
#define Check_AttControl_Range(x)   ((x > MAX_CONTROL_ATT_RANGE) ? MAX_CONTROL_ATT_RANGE : ((x < MIN_CONTROL_ATT_RANGE) ? MIN_CONTROL_ATT_RANGE : x))
#define Check_GyrControl_Range(x)   ((x > MAX_CONTROL_GYR_RANGE) ? MAX_CONTROL_GYR_RANGE : ((x < MIN_CONTROL_GYR_RANGE) ? MIN_CONTROL_GYR_RANGE : x))

static uint32_t imu_update_time = 0;
static uint32_t att_update_time = 0;
static bool failsafe = false;
static bool att_update = false;

DataPipe_CreateDataObj(ExpControlData_TypeDef, ExpCtl);

TaskControl_Monitor_TypeDef TaskControl_Monitor = {
    .init_state = false,
    .control_abort = false,

    /* on test mode use angular_speed over rate threshold protect */
    .angular_protect_enable = true,
    .angular_protect = false,

    /* on test mode for throttle control value mutation protect */
    .throttle_protect_enable = true,
    .throttle_protect = false,
    .throttle_percent = 0,

    .dynamic_disarm_enable = false,
    .IMU_Rt = 0,
};

/* internal function */
static void TaskControl_FlightControl_Polling(ControlData_TypeDef *exp_ctl_val, uint32_t sys_ms);
static void TaskControl_Actuator_ControlValue_Update(uint16_t throttle, float GX_Ctl_Out, float GY_Ctl_Out, float GZ_Ctl_Out);
static void TaskControl_CLI_Polling(void);
static void TaskControl_Get_StoreParam(void);

/* internal var */
static uint32_t TaskControl_Period = 0;

void TaskControl_Init(uint32_t period)
{
    if (period == 0)
        return;

    /* init monitor */
    // sensor_init_state.val = 0;
    memset(&TaskControl_Monitor, 0, sizeof(TaskControl_Monitor));
    // SrvDataHub.get_sensor_init_state(&sensor_init_state);

    /* pipe init */
    CtlData_smp_DataPipe.data_addr = (uint32_t)DataPipe_DataObjAddr(ExpCtl);
    CtlData_smp_DataPipe.data_size = DataPipe_DataSize(ExpCtl);
    DataPipe_Enable(&CtlData_smp_DataPipe);
    
    /* Parametet Init */
    TaskControl_Get_StoreParam();
    TaskControl_Monitor.init_state = SrvActuator.init(ESC_Type_DShot300);

    osMessageQDef(MotoCLI_Data, 64, TaskControl_CLIData_TypeDef);
    TaskControl_Monitor.CLIMessage_ID = osMessageCreate(osMessageQ(MotoCLI_Data), NULL);
    
    TaskControl_Period = period;
}

/* read param from storage */
static void TaskControl_Get_StoreParam(void)
{
    TaskControl_CtlPara_TypeDef Ctl_Param;

    /* search storage section first */
    memset(&Ctl_Param, 0, sizeof(TaskControl_CtlPara_TypeDef));
    memset(&TaskControl_Monitor.controller_info, 0, sizeof(Storage_ItemSearchOut_TypeDef));
    memset(&TaskControl_Monitor.actuator_store_info, 0, sizeof(Storage_ItemSearchOut_TypeDef));

    TaskControl_Monitor.controller_info = Storage.search(Para_User, CONTROL_STORAGE_SECTION_NAME);
    TaskControl_Monitor.actuator_store_info = Storage.search(Para_User, ACTUATOR_STORAGE_SECTION_NAME);

    /* set Ctl Parameter as default */
    Ctl_Param.att_mode = DEFAULT_ATTITUDE_CONTROLLER_MODE;
    Ctl_Param.alt_mode = DEFAULT_ATTITUDE_CONTROLLER_MODE;
    Ctl_Param.att_rate = DEFAULT_CONTROL_RATE;
    Ctl_Param.gx_rate = DEFAULT_CONTROL_RATE;
    Ctl_Param.gy_rate = DEFAULT_CONTROL_RATE;
    Ctl_Param.gz_rate = DEFAULT_CONTROL_RATE;
    Ctl_Param.pitch_range = DEFAULT_CONTROL_ATT_RANGE;
    Ctl_Param.roll_range = DEFAULT_CONTROL_ATT_RANGE;
    Ctl_Param.gx_range = DEFAULT_CONTROL_GYR_RANGE;
    Ctl_Param.gy_range = DEFAULT_CONTROL_GYR_RANGE;
    Ctl_Param.gz_range = DEFAULT_CONTROL_GYR_RANGE;
    memcpy(&TaskControl_Monitor.ctl_para, &Ctl_Param, sizeof(TaskControl_CtlPara_TypeDef));

    if (TaskControl_Monitor.controller_info.item_addr == 0)
    {
        /* section create */
        Storage.create(Para_User, CONTROL_STORAGE_SECTION_NAME, (uint8_t *)&Ctl_Param, sizeof(TaskControl_CtlPara_TypeDef));
    }
    else if (Storage.get(Para_User, TaskControl_Monitor.controller_info.item, (uint8_t *)&Ctl_Param, sizeof(TaskControl_CtlPara_TypeDef)) == Storage_Error_None)
    {
        /* check rate parameter validation */
        Ctl_Param.att_rate = Check_Control_Rate(Ctl_Param.att_rate);
        Ctl_Param.gx_rate = Check_Control_Rate(Ctl_Param.gx_rate);
        Ctl_Param.gy_rate = Check_Control_Rate(Ctl_Param.gy_rate);
        Ctl_Param.gz_rate = Check_Control_Rate(Ctl_Param.gz_rate);

        /* check range parameter validation */
        Ctl_Param.pitch_range = Check_AttControl_Range(Ctl_Param.pitch_range);
        Ctl_Param.roll_range = Check_AttControl_Range(Ctl_Param.roll_range);
        Ctl_Param.gx_range = Check_GyrControl_Range(Ctl_Param.gx_range);
        Ctl_Param.gy_range = Check_GyrControl_Range(Ctl_Param.gy_range);
        Ctl_Param.gz_range = Check_GyrControl_Range(Ctl_Param.gz_range);

        memcpy(&TaskControl_Monitor.ctl_para, &Ctl_Param, sizeof(TaskControl_CtlPara_TypeDef));
    }
    
    /* controller load parameter from storage */
    Controller.alt_ctl_init(TaskControl_Monitor.ctl_para.alt_mode);
    Controller.att_ctl_init(TaskControl_Monitor.ctl_para.att_mode);
}

static bool TaskControl_disarm_check(bool telemetry_arm, float pitch, float roll)
{
    if (telemetry_arm == DRONE_ARM)
    {
        TaskControl_Monitor.moto_unlock = Moto_Lock;
        return false;
    }

    if (TaskControl_Monitor.dynamic_disarm_enable)
    {
        TaskControl_Monitor.moto_unlock = Moto_Unlock;
        return true;
    }

    if (TaskControl_Monitor.moto_unlock != Moto_Unlock)
    {
        if (TaskControl_Monitor.moto_unlock == Moto_Unlock_Err)
            return false;

        /* attitude pitch check */
        if ((pitch > ATTITUDE_DISARM_RANGE_MAX) || \
            (pitch < ATTITUDE_DISARM_RANGE_MIN) || \
            (roll > ATTITUDE_DISARM_RANGE_MAX) || \
            (roll < ATTITUDE_DISARM_RANGE_MIN))
        {
            TaskControl_Monitor.moto_unlock = Moto_Unlock_Err;
            return false;
        }

        TaskControl_Monitor.moto_unlock = Moto_Unlock;
        return true;
    }

    return true;
}

void TaskControl_Core(void const *arg)
{
    uint32_t sys_time = SrvOsCommon.get_os_ms();
    uint32_t task_tick = 0;
    bool upgrade_state = false;
    ControlData_TypeDef CtlData;
    memset(&CtlData, 0, sizeof(ControlData_TypeDef));

    while(1)
    {
        task_tick = SrvOsCommon.get_os_ms();

        /* get control data from data hub */
        SrvDataHub.get_rc_control_data(&CtlData);

        if (SrvDataHub.get_upgrade_state(&upgrade_state) && upgrade_state)
        {
            /* lock all actuator when upgrading */
            SrvActuator.lock();
        }
        else
        {
            if (!TaskControl_Monitor.CLI_enable)
            {
                /* debug set control to angular speed control */
                TaskControl_FlightControl_Polling(&CtlData, task_tick);
            }
            else
            {
                if(TaskControl_Monitor.CLI_enable)
                {
                    TaskControl_CLI_Polling();
                }
                else
                    /* lock all moto */
                    SrvActuator.lock();
            }
        }

        SrvOsCommon.precise_delay(&sys_time, TaskControl_Period);
    }
}

static void TaskControl_Actuator_ControlValue_Update(uint16_t throttle, float GX_Ctl_Out, float GY_Ctl_Out, float GZ_Ctl_Out)
{
    int16_t ctl_buf[Actuator_Ctl_Sum] = {0};

    ctl_buf[Actuator_Ctl_Throttle] = throttle;

    ctl_buf[Actuator_Ctl_GyrX] = (int16_t)GX_Ctl_Out;
    ctl_buf[Actuator_Ctl_GyrY] = (int16_t)GY_Ctl_Out;
    ctl_buf[Actuator_Ctl_GyrZ] = (int16_t)GZ_Ctl_Out;

    SrvActuator.mix_control(ctl_buf);
}

/****************************************************** Flight Control Section ********************************************************/
static float TaskControl_Convert_CtlData(uint8_t gimbal_percent, float range, float rate)
{
    float exp = 0.0f;

    exp = (gimbal_percent - 50) / 50.0f;
    exp *= range;

    return exp * rate;
}

static void TaskControl_FlightControl_Polling(ControlData_TypeDef *exp_ctl_val, uint32_t sys_ms)
{
    uint8_t axis = Axis_X;
    bool arm_state = false;
    bool angular_only = false;

    // bool USB_Attach = false;
    AngControl_Out_TypeDef att_ctl_out;
    AttControl_In_TypeDef att_ctl_exp;
    AttControl_In_TypeDef att_ctl_mea;

    memset(&att_ctl_exp, 0, sizeof(AttControl_In_TypeDef));
    memset(&att_ctl_out, 0, sizeof(AngControl_Out_TypeDef));

    if (TaskControl_Monitor.init_state && exp_ctl_val)
    {
        if (exp_ctl_val->update_time_stamp && !exp_ctl_val->fail_safe)
        {
            att_ctl_exp.throttle_percent = exp_ctl_val->throttle_percent;
            att_ctl_exp.pitch  = TaskControl_Convert_CtlData(exp_ctl_val->pitch_percent, TaskControl_Monitor.ctl_para.pitch_range, TaskControl_Monitor.ctl_para.att_rate);
            att_ctl_exp.roll   = TaskControl_Convert_CtlData(exp_ctl_val->roll_percent,  TaskControl_Monitor.ctl_para.roll_range,  TaskControl_Monitor.ctl_para.att_rate);
            att_ctl_exp.gyro_x = TaskControl_Convert_CtlData(exp_ctl_val->roll_percent,  TaskControl_Monitor.ctl_para.gx_range,    TaskControl_Monitor.ctl_para.gx_rate);
            att_ctl_exp.gyro_y = TaskControl_Convert_CtlData(exp_ctl_val->pitch_percent, TaskControl_Monitor.ctl_para.gy_range,    TaskControl_Monitor.ctl_para.gy_rate);
            att_ctl_exp.gyro_z = TaskControl_Convert_CtlData(exp_ctl_val->yaw_percent,   TaskControl_Monitor.ctl_para.gz_range,    TaskControl_Monitor.ctl_para.gz_rate);
        }

        arm_state = exp_ctl_val->arm_state;
        failsafe = exp_ctl_val->fail_safe;
        TaskControl_Monitor.control_abort = false;

        /* pipe converted control data to data hub */
        DataPipe_DataObj(ExpCtl).time_stamp = sys_ms;
        DataPipe_DataObj(ExpCtl).arm = arm_state;
        DataPipe_DataObj(ExpCtl).failsafe = failsafe;
        DataPipe_DataObj(ExpCtl).throttle_percent = exp_ctl_val->throttle_percent;
        DataPipe_DataObj(ExpCtl).pitch = att_ctl_exp.pitch;
        DataPipe_DataObj(ExpCtl).roll  = att_ctl_exp.roll;
        DataPipe_DataObj(ExpCtl).gyr_x = att_ctl_exp.gyro_x;
        DataPipe_DataObj(ExpCtl).gyr_y = att_ctl_exp.gyro_y;
        DataPipe_DataObj(ExpCtl).gyr_z = att_ctl_exp.gyro_z;
        DataPipe_SendTo(&CtlData_smp_DataPipe, &CtlData_hub_DataPipe);
        DataPipe_SendTo(&CtlData_smp_DataPipe, &CtlData_log_DataPipe);

        // check imu filter gyro data update or not
        // if(!SrvDataHub.get_sensor_data(&sensor_data) || !sensor_init_state.bit.imu)
        //     goto lock_moto;

        // for(axis = Axis_X; axis < Axis_Sum; axis ++)
        // {
            // TaskControl_Monitor.gyr[axis] = sensor_data.scale_Gyro[axis];
        // }

        /* if angular speed over ride then lock the moto and set drone as arm */
        /* get attitude */
        att_update = SrvDataHub.get_attitude(&att_update_time,
                                             &TaskControl_Monitor.pitch,
                                             &TaskControl_Monitor.roll,
                                             &TaskControl_Monitor.yaw,
                                             NULL, NULL, NULL, NULL);

        att_ctl_mea.pitch  = TaskControl_Monitor.pitch;
        att_ctl_mea.roll   = TaskControl_Monitor.roll;
        att_ctl_mea.gyro_x = TaskControl_Monitor.gyr[Axis_X];
        att_ctl_mea.gyro_y = TaskControl_Monitor.gyr[Axis_Y];
        att_ctl_mea.gyro_z = TaskControl_Monitor.gyr[Axis_Z];

        TaskControl_disarm_check(arm_state, TaskControl_Monitor.pitch, TaskControl_Monitor.roll);

        /* if armed or usb attached then lock moto */
        if (TaskControl_Monitor.moto_unlock != Moto_Unlock)
            goto lock_moto;

        if (imu_update_time)
        {
            if (imu_update_time > TaskControl_Monitor.IMU_Rt)
            {
                TaskControl_Monitor.imu_update_error_cnt = 0;
                TaskControl_Monitor.IMU_Rt = imu_update_time;
            }
            else if (imu_update_time <= TaskControl_Monitor.IMU_Rt)
            {
                TaskControl_Monitor.imu_update_error_cnt++;
                if (TaskControl_Monitor.imu_update_error_cnt >= IMU_ERROR_UPDATE_MAX_COUNT)
                {
                    TaskControl_Monitor.control_abort = true;
                    goto lock_moto;
                }
            }

            /* check acc error state */
            // switch (sensor_data.acc_err)
            // {
            //     case Sensor_Blunt:
            //     case Sensor_exceed_Rate_of_change:
            //         angular_only = true;
            //         break;

            //     default:
            //         angular_only = false;
            //         break;
            // }

            /* check gyro error state */
            // switch (sensor_data.gyro_err)
            // {
                // case Sensor_Error_None:
                //     for(axis = Axis_X; axis < Axis_Sum; axis ++)
                //     {
                //         TaskControl_Monitor.gyr_lst[axis] = TaskControl_Monitor.gyr[axis];
                //     }
                //     TaskControl_Monitor.control_abort = true;
                //     break;

                // case Sensor_Blunt:
                // case Sensor_Over_Range:
                //     /* totally waste */
                //     /* bye drone see u in another world */
                //     TaskControl_Monitor.control_abort = true;
                //     goto lock_moto;
                //     break;

                // case Sensor_exceed_Rate_of_change:
                //     TaskControl_Monitor.angular_protect = true;
                //     if(TaskControl_Monitor.angular_warning_cnt < OVER_ANGULAR_ACCELERATE_COUNT)
                //     {
                //         TaskControl_Monitor.angular_protect = false;
                //         TaskControl_Monitor.angular_warning_cnt++;

                //         for(axis = Axis_X; axis < Axis_Sum; axis++)
                //         {
                //             TaskControl_Monitor.gyr[axis] = TaskControl_Monitor.gyr_lst[axis];
                //         }
                //     }
                //     break;

            //     default: break;
            // }
        }
        
        if(TaskControl_Monitor.angular_protect_enable && TaskControl_Monitor.angular_protect)
            goto lock_moto;

        /* do drone control algorithm down below */
        TaskControl_Monitor.throttle_percent = exp_ctl_val->throttle_percent;

        /* Controller Update */
        /* altitude control update */
        /* attitude control update */
        Controller.att_ctl(TaskControl_Monitor.ctl_para.att_mode, sys_ms, angular_only, att_ctl_exp, att_ctl_mea, &att_ctl_out);

        /* when usb attached then lock moto */
        // if (SrvDataHub.get_vcp_attach_state(&USB_Attach) || !USB_Attach)
        // {
            TaskControl_Actuator_ControlValue_Update(TaskControl_Monitor.throttle_percent, \
                                                        att_ctl_out.gyro_x, \
                                                        att_ctl_out.gyro_y, \
                                                        att_ctl_out.gyro_z);

            return;
        // }
    }

lock_moto:
    Controller.reset_att_processing(TaskControl_Monitor.ctl_para.att_mode);
    SrvActuator.lock();
}

/****************************************************** CLI Section ******************************************************************/
static void TaskControl_CLI_Polling(void)
{
    osEvent event;
    static TaskControl_CLIData_TypeDef CLIData;
    static uint16_t moto_ctl_buff[8] = {0};
    Shell *shell_obj = Shell_GetInstence();

    if ((shell_obj == NULL) || \
        (SrvActuator.lock == NULL) || \
        (SrvActuator.moto_direct_drive == NULL))
        return;

    if(TaskControl_Monitor.CLIMessage_ID)
    {
        event = osMessageGet(TaskControl_Monitor.CLIMessage_ID, CLI_MESSAGE_OPEARATE_TIMEOUT);
        if (event.status == osEventMessage)
        {
            if (event.value.p == NULL)
            {
                SrvActuator.lock();
                return;
            }

            CLIData = *(TaskControl_CLIData_TypeDef *)(event.value.p);
            switch((uint8_t)CLIData.cli_type)
            {
                case TaskControl_Moto_Set_SpinDir:
                    memset(moto_ctl_buff, 0, sizeof(moto_ctl_buff));
                    if(SrvActuator.set_spin_dir(CLIData.index, (uint8_t)CLIData.value))
                    {
                        shellPrint(shell_obj, "moto spin dir set done\r\n");
                    }
                    else
                        shellPrint(shell_obj, "moto spin dir set error\r\n");
                    break;

                case TaskControl_Moto_Set_Spin:
                    if(CLIData.index < ACTUATOR_MOTO_COUNT)
                    {
                        for(uint8_t i = 0; i < ACTUATOR_MOTO_COUNT; i++)
                            moto_ctl_buff[i] = 0;

                        moto_ctl_buff[CLIData.index] = CLIData.value;
                        break;
                    }
                    
                    for(uint8_t i = 0; i < ACTUATOR_MOTO_COUNT; i++)
                        moto_ctl_buff[i] = CLIData.value;
                    break;

                case TaskControl_Moto_CliDisable:
                    memset(moto_ctl_buff, 0, sizeof(moto_ctl_buff));
                    TaskControl_Monitor.CLI_enable = false;
                    break;

                default: break;
            }

            SrvOsCommon.free(event.value.p);
        }
    }

    if(CLIData.cli_type == TaskControl_Moto_Set_Spin)
    {
        /* set moto spin */
        for (uint8_t i = 0; i < ACTUATOR_MOTO_COUNT; i++)
            SrvActuator.moto_direct_drive(i, moto_ctl_buff[i]);
    }
    else
        SrvActuator.lock();
}

static void TaskControl_CLI_MotoSpinTest(uint8_t moto_index, uint16_t test_val)
{
    bool arm_state = false;
    Shell *shell_obj = Shell_GetInstence();
    TaskControl_CLIData_TypeDef *p_CLIData = NULL;
    bool ctl_enable = true;

    if(shell_obj == NULL)
        return;

    SrvDataHub.get_arm_state(&arm_state);

    if(TaskControl_Monitor.CLIMessage_ID == NULL)
    {
        shellPrint(shell_obj, "TaskControl semaphore create failed\r\n");
        return;
    }

    if(arm_state == DRONE_DISARM)
    {
        TaskControl_Monitor.CLI_enable = false;
        shellPrint(shell_obj, "Set drone in ARM state first\r\n");
        return;
    }

    TaskControl_Monitor.CLI_enable = true;
    shellPrint(shell_obj, "make sure all propeller is already disassmabled\r\n");

    if(moto_index >= ACTUATOR_MOTO_COUNT)
    {
        shellPrint(shell_obj, "all moto selected\r\n");
        for (uint8_t i = 0; i < ACTUATOR_MOTO_COUNT; i ++)
        {
            shellPrint(shell_obj, "moto %d control range down below\r\n", i);
            shellPrint(shell_obj, "\tmax  : %d\r\n", DSHOT_CTL_MAX);
            shellPrint(shell_obj, "\tidle : %d\r\n", DSHOT_CTL_IDLE);
            shellPrint(shell_obj, "\tmin  : %d\r\n", DSHOT_CTL_MIN);
        }
    }
    else
    {
        shellPrint(shell_obj, "moto %d is selected\r\n", moto_index);
        shellPrint(shell_obj, "control range down below\r\n");
        shellPrint(shell_obj, "\tmax  : %d\r\n", DSHOT_CTL_MAX);
        shellPrint(shell_obj, "\tidle : %d\r\n", DSHOT_CTL_IDLE);
        shellPrint(shell_obj, "\tmin  : %d\r\n", DSHOT_CTL_MIN);
    }

    if (!ctl_enable)
    {
        shellPrint(shell_obj, "Get moto control data range failed\r\n");
        return;
    }

    if(test_val > DSHOT_CTL_MAX)
    {
        shellPrint(shell_obj, "input value [%d] is bigger than max [%d] value", test_val, DSHOT_CTL_MAX);
    }
    else if(test_val < DSHOT_CTL_MIN)
    {
        shellPrint(shell_obj, "input value [%d] is lower than min [%d] value", test_val, DSHOT_CTL_MIN);
    }
    else
    {
        shellPrint(shell_obj, "current control value %d\r\n", test_val);

        p_CLIData = SrvOsCommon.malloc(sizeof(TaskControl_CLIData_TypeDef));
        if(p_CLIData)
        {
            p_CLIData->cli_type = TaskControl_Moto_Set_Spin;
            p_CLIData->index = moto_index;
            p_CLIData->timestamp = SrvOsCommon.get_os_ms();
            p_CLIData->value = test_val;

            if(osMessagePut(TaskControl_Monitor.CLIMessage_ID, (uint32_t)p_CLIData, CLI_MESSAGE_OPEARATE_TIMEOUT) != osOK)
            {
                SrvOsCommon.free(p_CLIData);
                shellPrint(shell_obj, "TaskControl CLI set failed\r\n");
            }
        }
        else
        {
            shellPrint(shell_obj, "Moto control test data malloc failed\r\n");
            SrvOsCommon.free(p_CLIData);
        }
    }
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC) | SHELL_CMD_DISABLE_RETURN, moto_spin, TaskControl_CLI_MotoSpinTest, Single Moto Spin);

static void TaskControl_Reverse_SpinDir(uint8_t moto_index, uint8_t dir)
{
    Shell *shell_obj = Shell_GetInstence();
    TaskControl_CLIData_TypeDef *p_CLIData = NULL;
    bool arm_state = false;

    if (shell_obj == NULL)
        return;
    
    SrvDataHub.get_arm_state(&arm_state);

    if(TaskControl_Monitor.CLIMessage_ID == NULL)
    {
        shellPrint(shell_obj, "TaskControl semaphore create failed\r\n");
        return;
    }

    if(arm_state == DRONE_DISARM)
    {
        TaskControl_Monitor.CLI_enable = false;
        shellPrint(shell_obj, "Set drone in ARM state first\r\n");
        return;
    }

    TaskControl_Monitor.CLI_enable = true;
    shellPrint(shell_obj, "make sure all propeller is already disassmabled\r\n");

    if((moto_index >= ACTUATOR_MOTO_COUNT) || (dir > 2))
    {
        shellPrint(shell_obj, "parameter error\r\n");
        return;
    }

    shellPrint(shell_obj, "moto %d is selected dir %d\r\n", moto_index, dir);
    p_CLIData = SrvOsCommon.malloc(sizeof(TaskControl_CLIData_TypeDef));
    if(p_CLIData)
    {
        p_CLIData->cli_type = TaskControl_Moto_Set_SpinDir;
        p_CLIData->index = moto_index;
        p_CLIData->timestamp = SrvOsCommon.get_os_ms();
        p_CLIData->value = dir;

        if(osMessagePut(TaskControl_Monitor.CLIMessage_ID, (uint32_t)p_CLIData, CLI_MESSAGE_OPEARATE_TIMEOUT) != osOK)
        {
            SrvOsCommon.free(p_CLIData);
            shellPrint(shell_obj, "TaskControl CLI set failed\r\n");
        }
    }
    else
    {
        shellPrint(shell_obj, "Moto control test data malloc failed\r\n");
        SrvOsCommon.free(p_CLIData);
    }
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC) | SHELL_CMD_DISABLE_RETURN, rev_spin, TaskControl_Reverse_SpinDir, reverse moto Spin);

static void TaskControl_Close_CLI(void)
{
    Shell *shell_obj = Shell_GetInstence();
    TaskControl_CLIData_TypeDef *p_CLIData = NULL;
    uint32_t time_stamp = SrvOsCommon.get_os_ms();

    if(shell_obj == NULL)
        return;

    if(TaskControl_Monitor.CLIMessage_ID == NULL)
    {
        shellPrint(shell_obj, "TaskControl semaphore create failed\r\n");
        return;
    }

    shellPrint(shell_obj, "TaskControl CLI disable\r\n");
    p_CLIData = SrvOsCommon.malloc(sizeof(TaskControl_CLIData_TypeDef));
    if (p_CLIData == NULL)
    {
        shellPrint(shell_obj, "Moto control test data malloc failed\r\n");
        SrvOsCommon.free(p_CLIData);
        return;
    }

    p_CLIData->cli_type = TaskControl_Moto_CliDisable;
    p_CLIData->timestamp = time_stamp;
    p_CLIData->index = 0;
    p_CLIData->value = 0;

    if(osMessagePut(TaskControl_Monitor.CLIMessage_ID, (uint32_t)p_CLIData, CLI_MESSAGE_OPEARATE_TIMEOUT) != osOK)
    {
        shellPrint(shell_obj, "TaskControl CLI set failed\r\n");
        SrvOsCommon.free(p_CLIData);
    }
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC) | SHELL_CMD_DISABLE_RETURN, lock_moto, TaskControl_Close_CLI, disable actuator test);

