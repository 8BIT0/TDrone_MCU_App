#include "Srv_DataHub.h"

#define To_Pipe_TransFinish_Callback(x) ((Pipe_TransFinish_Callback)x)

/* internal variable */
SrvDataHub_Monitor_TypeDef SrvDataHub_Monitor = {
    .init_state = false,
};

/* Pipe Object */
DataPipe_CreateDataObj(NaviData_TypeDef, Hub_Navi);
DataPipe_CreateDataObj(BaroData_TypeDef, Hub_Baro);
DataPipe_CreateDataObj(MagData_TypeDef, Hub_Mag);
DataPipe_CreateDataObj(IMUData_TypeDef, Hub_IMU);
DataPipe_CreateDataObj(SrvActuatorPipeData_TypeDef, Hub_Actuator);
DataPipe_CreateDataObj(ControlData_TypeDef, Hub_Telemetry_Rc);
DataPipe_CreateDataObj(ExpControlData_TypeDef, Hub_Cnv_CtlData);
DataPipe_CreateDataObj(bool, Hub_VCP_Attach_State);

/* internal function */
static void SrvDataHub_PipeRcTelemtryDataFinish_Callback(DataPipeObj_TypeDef *obj);
static void SrvDataHub_Actuator_DataPipe_Finish_Callback(DataPipeObj_TypeDef *obj);
static void SrvDataHub_VCPAttach_DataPipe_Finish_Callback(DataPipeObj_TypeDef *obj);
static void SrvDataHub_NavData_DataPipe_Finish_Callback(DataPipeObj_TypeDef *obj);
static void SrvDataHub_RawIMUData_DataPipe_Finish_Callback(DataPipeObj_TypeDef *obj);
static void SrvDataHub_RawMagData_DataPipe_Finish_Callback(DataPipeObj_TypeDef *obj);
static void SrvDataHub_RawBaroData_DataPipe_Finish_Callback(DataPipeObj_TypeDef *obj);
static void SrvDataHub_PipeConvertControlDataFinish_Callback(DataPipeObj_TypeDef *obj);

/* external function */
static void SrvDataHub_Init(void);
static bool SrvDataHub_Get_Arm(bool *arm);
static bool SrvDataHub_Get_Failsafe(bool *failsafe);
static bool SrvDataHub_Get_Telemetry_ControlData(ControlData_TypeDef *data);
static bool SrvDataHub_Get_Attitude(uint32_t *time_stamp, float *pitch, float *roll, float *yaw, float *heading);
static bool SrvDataHub_Get_VCPAttach_State(bool *state);
static bool SrvDataHub_Get_CLI_State(bool *state);
static bool SrvDataHub_Get_Upgrade_State(bool *state);
static bool SrvDataHub_Get_RelativeAlt(uint32_t *time_stamp, float *alt, float *alt_speed);
static bool SrvDataHub_Get_Convert_ControlData(uint32_t *time_stamp, bool *arm, bool *failsafe, float *pitch, float *roll, float *gx, float *gy, float *gz);
static bool SrvDataHub_Get_Actuator(uint32_t *time_stamp, int16_t *moto_ch, int16_t *servo_ch);

static bool SrvDataHub_Get_IMU(uint32_t *time_stamp, float *gx, float *gy, float *gz, float *ax, float *ay, float *az);
static bool SrvDataHub_Get_Baro(uint32_t *time_stamp, float *press, float *temp);
static bool SrvDatHub_Get_Mag(uint32_t *time_stamp, float *mx, float *my, float *mz);

static bool SrvDataHub_Set_CLI_State(bool state);
static bool SrvDataHub_Set_Upgrade_State(bool state);

/* external variable */
SrvDataHub_TypeDef SrvDataHub = {
    .init = SrvDataHub_Init,
    .get_attitude = SrvDataHub_Get_Attitude,
    .get_arm_state = SrvDataHub_Get_Arm,
    .get_failsafe = SrvDataHub_Get_Failsafe,
    .get_rc_control_data = SrvDataHub_Get_Telemetry_ControlData,
    .get_actuator = SrvDataHub_Get_Actuator,
    .get_vcp_attach_state = SrvDataHub_Get_VCPAttach_State,
    .get_cli_state = SrvDataHub_Get_CLI_State,
    .get_upgrade_state = SrvDataHub_Get_Upgrade_State,
    .get_relative_alt = SrvDataHub_Get_RelativeAlt,
    .get_cnv_control_data = SrvDataHub_Get_Convert_ControlData,

    /* sensor data */
    .get_imu = SrvDataHub_Get_IMU,
    .get_baro = SrvDataHub_Get_Baro,
    .get_mag = SrvDatHub_Get_Mag,

    .set_cli_state = SrvDataHub_Set_CLI_State,
    .set_upgrade_state = SrvDataHub_Set_Upgrade_State,
};

static void SrvDataHub_Init(void)
{
    if (SrvDataHub_Monitor.init_state)
        return;

    memset(&SrvDataHub_Monitor, 0, sizeof(SrvDataHub_Monitor));

    /* init pipe object */
    memset(DataPipe_DataObjAddr(Hub_Telemetry_Rc), 0, DataPipe_DataSize(Hub_Telemetry_Rc));
    Receiver_hub_DataPipe.data_addr = (uint32_t)DataPipe_DataObjAddr(Hub_Telemetry_Rc);
    Receiver_hub_DataPipe.data_size = DataPipe_DataSize(Hub_Telemetry_Rc);
    Receiver_hub_DataPipe.trans_finish_cb = To_Pipe_TransFinish_Callback(SrvDataHub_PipeRcTelemtryDataFinish_Callback);
    DataPipe_Enable(&Receiver_hub_DataPipe);

    memset(DataPipe_DataObjAddr(Hub_Actuator), 0, DataPipe_DataSize(Hub_Actuator));
    Actuator_hub_DataPipe.data_addr = (uint32_t)DataPipe_DataObjAddr(Hub_Actuator);
    Actuator_hub_DataPipe.data_size = DataPipe_DataSize(Hub_Actuator);
    Actuator_hub_DataPipe.trans_finish_cb = To_Pipe_TransFinish_Callback(SrvDataHub_Actuator_DataPipe_Finish_Callback);
    DataPipe_Enable(&Actuator_hub_DataPipe);

    memset(DataPipe_DataObjAddr(Hub_Navi), 0, DataPipe_DataSize(Hub_Navi));
    Navi_hub_DataPipe.data_addr = (uint32_t)DataPipe_DataObjAddr(Hub_Navi);
    Navi_hub_DataPipe.data_size = DataPipe_DataSize(Hub_Navi);
    Navi_hub_DataPipe.trans_finish_cb = To_Pipe_TransFinish_Callback(SrvDataHub_NavData_DataPipe_Finish_Callback);
    DataPipe_Enable(&Navi_hub_DataPipe);

    memset(DataPipe_DataObjAddr(Hub_IMU), 0, DataPipe_DataSize(Hub_IMU));
    RawIMU_hub_DataPipe.data_addr = (uint32_t)DataPipe_DataObjAddr(Hub_IMU);
    RawIMU_hub_DataPipe.data_size = DataPipe_DataSize(Hub_IMU);
    RawIMU_hub_DataPipe.trans_finish_cb = To_Pipe_TransFinish_Callback(SrvDataHub_RawIMUData_DataPipe_Finish_Callback);
    DataPipe_Enable(&RawIMU_hub_DataPipe);

    memset(DataPipe_DataObjAddr(Hub_Baro), 0, DataPipe_DataSize(Hub_Baro));
    RawBaro_hub_DataPipe.data_addr = (uint32_t)DataPipe_DataObjAddr(Hub_Baro);
    RawBaro_hub_DataPipe.data_size = DataPipe_DataSize(Hub_Baro);
    RawBaro_hub_DataPipe.trans_finish_cb = To_Pipe_TransFinish_Callback(SrvDataHub_RawBaroData_DataPipe_Finish_Callback);
    DataPipe_Enable(&RawBaro_hub_DataPipe);

    memset(DataPipe_DataObjAddr(Hub_Mag), 0, DataPipe_DataSize(Hub_Mag));
    RawMag_hub_DataPipe.data_addr = (uint32_t)DataPipe_DataObjAddr(Hub_Mag);
    RawMag_hub_DataPipe.data_size = DataPipe_DataSize(Hub_Mag);
    RawMag_hub_DataPipe.trans_finish_cb = To_Pipe_TransFinish_Callback(SrvDataHub_RawMagData_DataPipe_Finish_Callback);
    DataPipe_Enable(&RawMag_hub_DataPipe);

    memset(DataPipe_DataObjAddr(Hub_VCP_Attach_State), 0, DataPipe_DataSize(Hub_VCP_Attach_State));
    VCP_Connect_hub_DataPipe.data_addr = (uint32_t)DataPipe_DataObjAddr(Hub_VCP_Attach_State);
    VCP_Connect_hub_DataPipe.data_size = DataPipe_DataSize(Hub_VCP_Attach_State);
    VCP_Connect_hub_DataPipe.trans_finish_cb = To_Pipe_TransFinish_Callback(SrvDataHub_VCPAttach_DataPipe_Finish_Callback);
    DataPipe_Enable(&VCP_Connect_hub_DataPipe);
    
    memset(DataPipe_DataObjAddr(Hub_Cnv_CtlData), 0, DataPipe_DataSize(Hub_Cnv_CtlData));
    CtlData_hub_DataPipe.data_addr = (uint32_t)DataPipe_DataObjAddr(Hub_Cnv_CtlData);
    CtlData_hub_DataPipe.data_size = DataPipe_DataSize(Hub_Cnv_CtlData);
    CtlData_hub_DataPipe.trans_finish_cb = To_Pipe_TransFinish_Callback(SrvDataHub_PipeConvertControlDataFinish_Callback);
    DataPipe_Enable(&CtlData_hub_DataPipe);

    memset(&SrvDataHub_Monitor, 0, sizeof(SrvDataHub_Monitor));
    SrvDataHub_Monitor.init_state = true;
    SrvDataHub_Monitor.data.failsafe = true;
    SrvDataHub_Monitor.data.arm = DRONE_ARM;
}

static bool SrvDataHub_Set_CLI_State(bool state)
{
    if (SrvDataHub_Monitor.inuse_reg.bit.cli)
        SrvDataHub_Monitor.inuse_reg.bit.cli = false;

    SrvDataHub_Monitor.update_reg.bit.cli = true;
    SrvDataHub_Monitor.data.CLI_state = state;
    SrvDataHub_Monitor.update_reg.bit.cli = false;

    return true;
}

static void SrvDataHub_VCPAttach_DataPipe_Finish_Callback(DataPipeObj_TypeDef *obj)
{
    if ((!SrvDataHub_Monitor.init_state) || (obj == NULL) || (obj != &VCP_Connect_hub_DataPipe))
        return;

    SrvDataHub_Monitor.update_reg.bit.USB_VCP_attach = true;

    if (SrvDataHub_Monitor.inuse_reg.bit.USB_VCP_attach)
        SrvDataHub_Monitor.inuse_reg.bit.USB_VCP_attach = false;

    SrvDataHub_Monitor.data.VCP_Attach = DataPipe_DataObj(Hub_VCP_Attach_State);

    SrvDataHub_Monitor.update_reg.bit.USB_VCP_attach = false;
}

static void SrvDataHub_Actuator_DataPipe_Finish_Callback(DataPipeObj_TypeDef *obj)
{
    if ((!SrvDataHub_Monitor.init_state) || (obj == NULL) || (obj != &Actuator_hub_DataPipe))
        return;
        
    SrvDataHub_Monitor.update_reg.bit.actuator = true;

    if (SrvDataHub_Monitor.inuse_reg.bit.actuator)
        SrvDataHub_Monitor.inuse_reg.bit.actuator = false;

    SrvDataHub_Monitor.data.actuator_update_time = DataPipe_DataObj(Hub_Actuator).time_stamp;
    memcpy(SrvDataHub_Monitor.data.moto, DataPipe_DataObj(Hub_Actuator).moto, sizeof(SrvDataHub_Monitor.data.moto));
    memcpy(SrvDataHub_Monitor.data.servo, DataPipe_DataObj(Hub_Actuator).servo, sizeof(SrvDataHub_Monitor.data.servo));

    SrvDataHub_Monitor.update_reg.bit.actuator = false;
}

static void SrvDataHub_PipeRcTelemtryDataFinish_Callback(DataPipeObj_TypeDef *obj)
{
    if ((!SrvDataHub_Monitor.init_state) || (obj == NULL) || (obj != &Receiver_hub_DataPipe))
        return;

    SrvDataHub_Monitor.update_reg.bit.rc_control_data = true;

    if (SrvDataHub_Monitor.inuse_reg.bit.rc_control_data)
        SrvDataHub_Monitor.inuse_reg.bit.rc_control_data = false;

    SrvDataHub_Monitor.data.RC_Control_Data = DataPipe_DataObj(Hub_Telemetry_Rc);

    SrvDataHub_Monitor.update_reg.bit.rc_control_data = false;
}

static void SrvDataHub_PipeConvertControlDataFinish_Callback(DataPipeObj_TypeDef *obj)
{
    if ((!SrvDataHub_Monitor.init_state) || (obj == NULL) || (obj != &CtlData_hub_DataPipe))
        return;

    SrvDataHub_Monitor.update_reg.bit.cnv_control_data = true;

    if (SrvDataHub_Monitor.inuse_reg.bit.cnv_control_data)
        SrvDataHub_Monitor.inuse_reg.bit.cnv_control_data = false;

    SrvDataHub_Monitor.data.cnvctl_data_time = DataPipe_DataObj(Hub_Cnv_CtlData).time_stamp;
    SrvDataHub_Monitor.data.arm = DataPipe_DataObj(Hub_Cnv_CtlData).arm;
    SrvDataHub_Monitor.data.throttle_percent = DataPipe_DataObj(Hub_Cnv_CtlData).throttle_percent;
    SrvDataHub_Monitor.data.failsafe = DataPipe_DataObj(Hub_Cnv_CtlData).failsafe;
    SrvDataHub_Monitor.data.exp_gyr_x = DataPipe_DataObj(Hub_Cnv_CtlData).gyr_x;
    SrvDataHub_Monitor.data.exp_gyr_y = DataPipe_DataObj(Hub_Cnv_CtlData).gyr_y;
    SrvDataHub_Monitor.data.exp_gyr_z = DataPipe_DataObj(Hub_Cnv_CtlData).gyr_z;
    SrvDataHub_Monitor.data.exp_pitch = DataPipe_DataObj(Hub_Cnv_CtlData).pitch;
    SrvDataHub_Monitor.data.exp_roll = DataPipe_DataObj(Hub_Cnv_CtlData).roll;

    SrvDataHub_Monitor.update_reg.bit.cnv_control_data = false;
}

static void SrvDataHub_RawIMUData_DataPipe_Finish_Callback(DataPipeObj_TypeDef *obj)
{
    if (!SrvDataHub_Monitor.init_state || (obj == NULL) || (obj != &RawIMU_hub_DataPipe))
        return;

    SrvDataHub_Monitor.update_reg.bit.imu_data = true;

    SrvDataHub_Monitor.data.imu_update_time = DataPipe_DataObj(Hub_IMU).time_stamp;
    memcpy(SrvDataHub_Monitor.data.raw_acc, DataPipe_DataObj(Hub_IMU).acc_flt, sizeof(SrvDataHub_Monitor.data.raw_acc));
    memcpy(SrvDataHub_Monitor.data.raw_gyr, DataPipe_DataObj(Hub_IMU).gyr_flt, sizeof(SrvDataHub_Monitor.data.raw_gyr));

    if (SrvDataHub_Monitor.inuse_reg.bit.imu_data)
        SrvDataHub_Monitor.inuse_reg.bit.imu_data = false;

    SrvDataHub_Monitor.update_reg.bit.imu_data = false;
}

static void SrvDataHub_RawMagData_DataPipe_Finish_Callback(DataPipeObj_TypeDef *obj)
{
    if (!SrvDataHub_Monitor.init_state || (obj == NULL) || (obj != &RawMag_hub_DataPipe))
        return;

    SrvDataHub_Monitor.update_reg.bit.mag_data = true;

    if (SrvDataHub_Monitor.inuse_reg.bit.mag_data)
        SrvDataHub_Monitor.inuse_reg.bit.mag_data = false;

    SrvDataHub_Monitor.data.mag_update_time = DataPipe_DataObj(Hub_Mag).time_stamp;
    memcpy(SrvDataHub_Monitor.data.raw_mag, DataPipe_DataObj(Hub_Mag).mag, sizeof(SrvDataHub_Monitor.data.raw_mag));

    SrvDataHub_Monitor.update_reg.bit.mag_data = false;
}

static void SrvDataHub_RawBaroData_DataPipe_Finish_Callback(DataPipeObj_TypeDef *obj)
{
    if (!SrvDataHub_Monitor.init_state || (obj == NULL) || (obj != &RawBaro_hub_DataPipe))
        return;

    SrvDataHub_Monitor.update_reg.bit.baro_data = true;

    if (SrvDataHub_Monitor.inuse_reg.bit.baro_data)
        SrvDataHub_Monitor.inuse_reg.bit.baro_data = false;

    SrvDataHub_Monitor.data.baro_update_time = DataPipe_DataObj(Hub_Baro).time_stamp;
    SrvDataHub_Monitor.data.baro_pres = DataPipe_DataObj(Hub_Baro).pres;
    SrvDataHub_Monitor.data.baro_temp = DataPipe_DataObj(Hub_Baro).temp;

    SrvDataHub_Monitor.update_reg.bit.baro_data = false;
}

static void SrvDataHub_NavData_DataPipe_Finish_Callback(DataPipeObj_TypeDef *obj)
{
    if ((!SrvDataHub_Monitor.init_state) || (obj == NULL) || (obj != &Navi_hub_DataPipe))
        return;

    SrvDataHub_Monitor.update_reg.bit.navi_data  = true;

    if (SrvDataHub_Monitor.inuse_reg.bit.navi_data)
        SrvDataHub_Monitor.inuse_reg.bit.navi_data = false;

    /* set data */
    SrvDataHub_Monitor.data.navi_update_time = DataPipe_DataObj(Hub_Navi).time_stamp;
    SrvDataHub_Monitor.data.navi_roll = DataPipe_DataObj(Hub_Navi).roll;
    SrvDataHub_Monitor.data.navi_pitch = DataPipe_DataObj(Hub_Navi).pitch;
    SrvDataHub_Monitor.data.navi_heading = DataPipe_DataObj(Hub_Navi).yaw;
    SrvDataHub_Monitor.data.navi_q0 = DataPipe_DataObj(Hub_Navi).q0;
    SrvDataHub_Monitor.data.navi_q1 = DataPipe_DataObj(Hub_Navi).q1;
    SrvDataHub_Monitor.data.navi_q2 = DataPipe_DataObj(Hub_Navi).q2;
    SrvDataHub_Monitor.data.navi_q3 = DataPipe_DataObj(Hub_Navi).q3;
    memcpy(SrvDataHub_Monitor.data.navi_acc, DataPipe_DataObj(Hub_Navi).acc, sizeof(float) * Axis_Sum);
    memcpy(SrvDataHub_Monitor.data.navi_gyr, DataPipe_DataObj(Hub_Navi).gyr, sizeof(float) * Axis_Sum);
    memcpy(SrvDataHub_Monitor.data.navi_mag, DataPipe_DataObj(Hub_Navi).mag, sizeof(float) * Mag_Axis_Sum);
    SrvDataHub_Monitor.data.navi_pres = DataPipe_DataObj(Hub_Navi).baro_pres;
    SrvDataHub_Monitor.data.navi_alt = DataPipe_DataObj(Hub_Navi).baro_alt;
    SrvDataHub_Monitor.data.navi_tof_alt = DataPipe_DataObj(Hub_Navi).tof_alt;
    memcpy(SrvDataHub_Monitor.data.navi_rel_vel, DataPipe_DataObj(Hub_Navi).rel_vel, sizeof(float) * POS_Axis_Sum);
    memcpy(SrvDataHub_Monitor.data.navi_rel_pos, DataPipe_DataObj(Hub_Navi).rel_pos, sizeof(float) * POS_Axis_Sum);

    SrvDataHub_Monitor.update_reg.bit.navi_data = false;
}

static bool SrvDataHub_Get_RelativeAlt(uint32_t *time_stamp, float *alt, float *alt_speed)
{
    SrvDataHub_Monitor.inuse_reg.bit.navi_data = true;

reupdate_relative_alt:
    if (time_stamp)
        *time_stamp = SrvDataHub_Monitor.data.relative_alt_time;
    
    if (alt)
        *alt = SrvDataHub_Monitor.data.relative_alt;

    if (alt_speed)
        *alt_speed = SrvDataHub_Monitor.data.relative_vertical_speed;

    if (!SrvDataHub_Monitor.inuse_reg.bit.navi_data)
        goto reupdate_relative_alt;

    SrvDataHub_Monitor.inuse_reg.bit.navi_data = false;

    return true;
}

static bool SrvDataHub_Get_Attitude(uint32_t *time_stamp, float *pitch, float *roll, float *yaw, float *heading)
{
reupdate_attitude:
    SrvDataHub_Monitor.inuse_reg.bit.navi_data = true; 
    
    if (time_stamp)
        (*time_stamp) = SrvDataHub_Monitor.data.navi_update_time;
    
    if (pitch)
        (*pitch) = SrvDataHub_Monitor.data.navi_pitch;
    
    if (roll)
        (*roll) = SrvDataHub_Monitor.data.navi_roll;
    
    if (yaw)
        (*yaw) = SrvDataHub_Monitor.data.navi_yaw;

    if (heading)
        (*heading) = SrvDataHub_Monitor.data.navi_heading;

    if(!SrvDataHub_Monitor.inuse_reg.bit.navi_data)
        goto reupdate_attitude;

    SrvDataHub_Monitor.inuse_reg.bit.navi_data = false;

    return true;
}

static bool SrvDataHub_Get_Arm(bool *arm)
{
    if (arm == NULL)
        return false;

reupdate_arm:
    SrvDataHub_Monitor.inuse_reg.bit.cnv_control_data = true;
    *arm = SrvDataHub_Monitor.data.arm;

    if (!SrvDataHub_Monitor.inuse_reg.bit.cnv_control_data)
        goto reupdate_arm;

    SrvDataHub_Monitor.inuse_reg.bit.cnv_control_data = false;

    return true;
}

static bool SrvDataHub_Get_Failsafe(bool *failsafe)
{
reupdate_failsafe:
    SrvDataHub_Monitor.inuse_reg.bit.cnv_control_data = true;
    
    if (failsafe)
        *failsafe = SrvDataHub_Monitor.data.failsafe;

    if (!SrvDataHub_Monitor.inuse_reg.bit.cnv_control_data)
        goto reupdate_failsafe;

    SrvDataHub_Monitor.inuse_reg.bit.cnv_control_data = false;

    return true;
}

static bool SrvDataHub_Get_Convert_ControlData(uint32_t *time_stamp, bool *arm, bool *failsafe, float *pitch, float *roll, float *gx, float *gy, float *gz)
{
reupdate_convertdata:
    SrvDataHub_Monitor.inuse_reg.bit.cnv_control_data = true;

    if (time_stamp)
        *time_stamp = SrvDataHub_Monitor.data.cnvctl_data_time;
    
    if (arm)
        *arm = SrvDataHub_Monitor.data.arm;
        
    if (failsafe)
        *failsafe = SrvDataHub_Monitor.data.failsafe;
        
    if (pitch)
        *pitch = SrvDataHub_Monitor.data.exp_pitch;
        
    if (roll)
        *roll = SrvDataHub_Monitor.data.exp_roll;
        
    if (gx)
        *gx = SrvDataHub_Monitor.data.exp_gyr_x;
        
    if (gy)
        *gy = SrvDataHub_Monitor.data.exp_gyr_y;

    if (gz)
        *gz = SrvDataHub_Monitor.data.exp_gyr_z;

    if (!SrvDataHub_Monitor.inuse_reg.bit.cnv_control_data)
        goto reupdate_convertdata;

    SrvDataHub_Monitor.inuse_reg.bit.cnv_control_data = false;
    
    return true;
}

static bool SrvDataHub_Get_Telemetry_ControlData(ControlData_TypeDef *data)
{
    if(data == NULL)
        return false;

reupdate_telemetry_control_data:
    SrvDataHub_Monitor.inuse_reg.bit.rc_control_data = true;

    (*data) = SrvDataHub_Monitor.data.RC_Control_Data;

    if(!SrvDataHub_Monitor.inuse_reg.bit.rc_control_data)
        goto reupdate_telemetry_control_data;

    SrvDataHub_Monitor.inuse_reg.bit.rc_control_data = false;

    return true;
}

static bool SrvDataHub_Get_Actuator(uint32_t *time_stamp, int16_t *moto_ch, int16_t *servo_ch)
{
reupdate_actuator:
    SrvDataHub_Monitor.inuse_reg.bit.actuator = true;

    *time_stamp = SrvDataHub_Monitor.data.actuator_update_time;

    if (moto_ch)
        memcpy(moto_ch, SrvDataHub_Monitor.data.moto, sizeof(SrvDataHub_Monitor.data.moto));
    
    if (servo_ch)
        memcpy(servo_ch, SrvDataHub_Monitor.data.servo, sizeof(SrvDataHub_Monitor.data.servo));

    if (!SrvDataHub_Monitor.inuse_reg.bit.actuator)
        goto reupdate_actuator;

    SrvDataHub_Monitor.inuse_reg.bit.actuator = false;

    return true;
}

static bool SrvDataHub_Get_CLI_State(bool *state)
{
    if(state == NULL)
        return false;

reupdate_cli_state:
    SrvDataHub_Monitor.inuse_reg.bit.cli = true;

    (*state) = SrvDataHub_Monitor.data.CLI_state;

    if(!SrvDataHub_Monitor.inuse_reg.bit.cli)
        goto reupdate_cli_state;

    SrvDataHub_Monitor.inuse_reg.bit.cli = false;

    return true;
}

static bool SrvDataHub_Set_Upgrade_State(bool state)
{
    if (SrvDataHub_Monitor.inuse_reg.bit.upgrade)
        SrvDataHub_Monitor.inuse_reg.bit.upgrade = false;

    SrvDataHub_Monitor.update_reg.bit.upgrade = true;
    SrvDataHub_Monitor.data.upgrading = state;
    SrvDataHub_Monitor.update_reg.bit.upgrade = false;
    
    return true; 
}

static bool SrvDataHub_Get_Upgrade_State(bool *state)
{
    if(state == NULL)
        return false;

reupdate_upgrade_state:
    SrvDataHub_Monitor.inuse_reg.bit.upgrade = true;

    (*state) = SrvDataHub_Monitor.data.upgrading;

    if(!SrvDataHub_Monitor.inuse_reg.bit.upgrade)
        goto reupdate_upgrade_state;

    SrvDataHub_Monitor.inuse_reg.bit.upgrade = false;
    
    return true;
}

static bool SrvDataHub_Get_VCPAttach_State(bool *state)
{
    if(state == NULL)
        return false;

reupdate_vcp_attach_state:
    SrvDataHub_Monitor.inuse_reg.bit.USB_VCP_attach = true;

    (*state) = SrvDataHub_Monitor.data.VCP_Attach;

    if(!SrvDataHub_Monitor.inuse_reg.bit.USB_VCP_attach)
        goto reupdate_vcp_attach_state;

    SrvDataHub_Monitor.inuse_reg.bit.USB_VCP_attach = false;
    
    return true;
}


static bool SrvDataHub_Get_IMU(uint32_t *time_stamp, float *gx, float *gy, float *gz, float *ax, float *ay, float *az)
{
reupdate_raw_imu:
    SrvDataHub_Monitor.inuse_reg.bit.imu_data = true;

    if (time_stamp)
        *time_stamp = SrvDataHub_Monitor.data.imu_update_time;

    if (gx)
        *gx = SrvDataHub_Monitor.data.raw_gyr[Axis_X];

    if (gy)
        *gy = SrvDataHub_Monitor.data.raw_gyr[Axis_Y];

    if (gz)
        *gz = SrvDataHub_Monitor.data.raw_gyr[Axis_Z];

    if (ax)
        *ax = SrvDataHub_Monitor.data.raw_acc[Axis_X];

    if (ay)
        *ay = SrvDataHub_Monitor.data.raw_acc[Axis_Y];

    if (az)
        *az = SrvDataHub_Monitor.data.raw_acc[Axis_Z];

    if (!SrvDataHub_Monitor.inuse_reg.bit.imu_data)
        goto reupdate_raw_imu;

    SrvDataHub_Monitor.inuse_reg.bit.imu_data = false;
    return true;
}

static bool SrvDataHub_Get_Baro(uint32_t *time_stamp, float *press, float *temp)
{
reupdate_raw_baro:
    SrvDataHub_Monitor.inuse_reg.bit.baro_data = true;

    if (time_stamp)
        *time_stamp = SrvDataHub_Monitor.data.baro_update_time;

    if (press)
        *press = SrvDataHub_Monitor.data.baro_pres;

    if (temp)
        *temp = SrvDataHub_Monitor.data.baro_temp;

    if (!SrvDataHub_Monitor.inuse_reg.bit.baro_data)
        goto reupdate_raw_baro;

    SrvDataHub_Monitor.inuse_reg.bit.baro_data = false;

    return true;
}

static bool SrvDatHub_Get_Mag(uint32_t *time_stamp, float *mx, float *my, float *mz)
{
reupdate_raw_mag:
    SrvDataHub_Monitor.inuse_reg.bit.mag_data = true;

    if (time_stamp)
        *time_stamp = SrvDataHub_Monitor.data.mag_update_time;

    if (mx)
        *mx = SrvDataHub_Monitor.data.raw_mag[Mag_Axis_X];

    if (my)
        *my = SrvDataHub_Monitor.data.raw_mag[Mag_Axis_Y];

    if (mz)
        *mz = SrvDataHub_Monitor.data.raw_mag[Mag_Axis_Z];

    if (!SrvDataHub_Monitor.inuse_reg.bit.mag_data)
        goto reupdate_raw_mag;

    SrvDataHub_Monitor.inuse_reg.bit.mag_data = false;
    return true;
}

