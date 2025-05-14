#include "Srv_DataHub.h"

#define To_Pipe_TransFinish_Callback(x) ((Pipe_TransFinish_Callback)x)

/* internal variable */
SrvDataHub_Monitor_TypeDef SrvDataHub_Monitor = {
    .init_state = false,
};

/* Pipe Object */
DataPipe_CreateDataObj(SrvActuatorPipeData_TypeDef, Hub_Actuator);
DataPipe_CreateDataObj(ControlData_TypeDef, Hub_Telemetry_Rc);
DataPipe_CreateDataObj(IMUAtt_TypeDef, Hub_Attitude);
DataPipe_CreateDataObj(RelMov_TypeDef, Hub_Alt);
DataPipe_CreateDataObj(ExpControlData_TypeDef, Hub_Cnv_CtlData);
DataPipe_CreateDataObj(bool, Hub_VCP_Attach_State);

/* internal function */
static void SrvDataHub_PipeRcTelemtryDataFinish_Callback(DataPipeObj_TypeDef *obj);
static void SrvDataHub_Actuator_DataPipe_Finish_Callback(DataPipeObj_TypeDef *obj);
static void SrvDataHub_Attitude_DataPipe_Finish_Callback(DataPipeObj_TypeDef *obj);
// static void SrvDataHub_Pos_DataPipe_Finish_Callback(DataPipeObj_TypeDef *obj);
static void SrvDatHub_Alt_DataPipe_Finish_Callback(DataPipeObj_TypeDef *obj);
static void SrvDataHub_VCPAttach_dataPipe_Finish_Callback(DataPipeObj_TypeDef *obj);
static void SrvDataHub_PipeConvertControlDataFinish_Callback(DataPipeObj_TypeDef *obj);

/* external function */
static void SrvDataHub_Init(void);
static bool SrvDataHub_Get_Arm(bool *arm);
static bool SrvDataHub_Get_Failsafe(bool *failsafe);
static bool SrvDataHub_Get_Telemetry_ControlData(ControlData_TypeDef *data);
static bool SrvDataHub_Get_Attitude(uint32_t *time_stamp, float *pitch, float *roll, float *yaw, float *q0, float *q1, float *q2, float *q3);
static bool SrvDataHub_Get_VCPAttach_State(bool *state);
static bool SrvDataHub_Get_CLI_State(bool *state);
static bool SrvDataHub_Get_Upgrade_State(bool *state);
static bool SrvDataHub_Get_RelativeAlt(uint32_t *time_stamp, float *alt, float *alt_speed);
static bool SrvDataHub_Get_Convert_ControlData(uint32_t *time_stamp, bool *arm, bool *failsafe, float *pitch, float *roll, float *gx, float *gy, float *gz);
static bool SrvDataHub_Get_Actuator(uint32_t *time_stamp, int16_t *moto_ch, int16_t *servo_ch);

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
    
    memset(DataPipe_DataObjAddr(Hub_Attitude), 0, DataPipe_DataSize(Hub_Attitude));
    Attitude_hub_DataPipe.data_addr = (uint32_t)DataPipe_DataObjAddr(Hub_Attitude);
    Attitude_hub_DataPipe.data_size = DataPipe_DataSize(Hub_Attitude);
    Attitude_hub_DataPipe.trans_finish_cb = To_Pipe_TransFinish_Callback(SrvDataHub_Attitude_DataPipe_Finish_Callback);
    DataPipe_Enable(&Attitude_hub_DataPipe);

    memset(DataPipe_DataObjAddr(Hub_Alt), 0, DataPipe_DataSize(Hub_Alt));
    Altitude_hub_DataPipe.data_addr = (uint32_t)DataPipe_DataObjAddr(Hub_Alt);
    Altitude_hub_DataPipe.data_size = DataPipe_DataSize(Hub_Alt);
    Altitude_hub_DataPipe.trans_finish_cb = To_Pipe_TransFinish_Callback(SrvDatHub_Alt_DataPipe_Finish_Callback);
    DataPipe_Enable(&Altitude_hub_DataPipe);

    memset(DataPipe_DataObjAddr(Hub_VCP_Attach_State), 0, DataPipe_DataSize(Hub_VCP_Attach_State));
    VCP_Connect_hub_DataPipe.data_addr = (uint32_t)DataPipe_DataObjAddr(Hub_VCP_Attach_State);
    VCP_Connect_hub_DataPipe.data_size = DataPipe_DataSize(Hub_VCP_Attach_State);
    VCP_Connect_hub_DataPipe.trans_finish_cb = To_Pipe_TransFinish_Callback(SrvDataHub_VCPAttach_dataPipe_Finish_Callback);
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

static void SrvDataHub_VCPAttach_dataPipe_Finish_Callback(DataPipeObj_TypeDef *obj)
{
    if (obj == &VCP_Connect_hub_DataPipe)
    {
        SrvDataHub_Monitor.update_reg.bit.USB_VCP_attach = true;

        if (SrvDataHub_Monitor.inuse_reg.bit.USB_VCP_attach)
            SrvDataHub_Monitor.inuse_reg.bit.USB_VCP_attach = false;

        SrvDataHub_Monitor.data.VCP_Attach = DataPipe_DataObj(Hub_VCP_Attach_State);

        SrvDataHub_Monitor.update_reg.bit.USB_VCP_attach = false;
    }
}

static void SrvDatHub_Alt_DataPipe_Finish_Callback(DataPipeObj_TypeDef *obj)
{
    if (obj == &Altitude_hub_DataPipe)
    {
        SrvDataHub_Monitor.update_reg.bit.relative_alt = true;

        if (SrvDataHub_Monitor.inuse_reg.bit.relative_alt)
            SrvDataHub_Monitor.inuse_reg.bit.relative_alt = false;
            
        SrvDataHub_Monitor.data.relative_alt_time = DataPipe_DataObj(Hub_Alt).time;
        SrvDataHub_Monitor.data.relative_alt = DataPipe_DataObj(Hub_Alt).pos;
        SrvDataHub_Monitor.data.relative_vertical_speed = DataPipe_DataObj(Hub_Alt).vel;

        SrvDataHub_Monitor.update_reg.bit.relative_alt = false;
    }
}

static void SrvDataHub_Attitude_DataPipe_Finish_Callback(DataPipeObj_TypeDef *obj)
{
    if(obj == &Attitude_hub_DataPipe)
    {
        SrvDataHub_Monitor.update_reg.bit.attitude = true;

        if(SrvDataHub_Monitor.inuse_reg.bit.attitude)
            SrvDataHub_Monitor.inuse_reg.bit.attitude = false;

        SrvDataHub_Monitor.data.att_update_time = DataPipe_DataObj(Hub_Attitude).time_stamp;
        SrvDataHub_Monitor.data.att_pitch = DataPipe_DataObj(Hub_Attitude).pitch;
        SrvDataHub_Monitor.data.att_roll = DataPipe_DataObj(Hub_Attitude).roll;
        SrvDataHub_Monitor.data.att_yaw = DataPipe_DataObj(Hub_Attitude).yaw;
        SrvDataHub_Monitor.data.att_q0 = DataPipe_DataObj(Hub_Attitude).q0;
        SrvDataHub_Monitor.data.att_q1 = DataPipe_DataObj(Hub_Attitude).q1;
        SrvDataHub_Monitor.data.att_q2 = DataPipe_DataObj(Hub_Attitude).q2;
        SrvDataHub_Monitor.data.att_q3 = DataPipe_DataObj(Hub_Attitude).q3;
        SrvDataHub_Monitor.data.att_error_code = DataPipe_DataObj(Hub_Attitude).err_code;
    
        SrvDataHub_Monitor.update_reg.bit.attitude = false;
    }
}

static void SrvDataHub_Actuator_DataPipe_Finish_Callback(DataPipeObj_TypeDef *obj)
{
    if (obj == &Actuator_hub_DataPipe)
    {
        SrvDataHub_Monitor.update_reg.bit.actuator = true;

        if (SrvDataHub_Monitor.inuse_reg.bit.actuator)
            SrvDataHub_Monitor.inuse_reg.bit.actuator = false;

        SrvDataHub_Monitor.data.actuator_update_time = DataPipe_DataObj(Hub_Actuator).time_stamp;
        memcpy(SrvDataHub_Monitor.data.moto, DataPipe_DataObj(Hub_Actuator).moto, sizeof(SrvDataHub_Monitor.data.moto));
        memcpy(SrvDataHub_Monitor.data.servo, DataPipe_DataObj(Hub_Actuator).servo, sizeof(SrvDataHub_Monitor.data.servo));

        SrvDataHub_Monitor.update_reg.bit.actuator = false;
    }
}

static void SrvDataHub_PipeRcTelemtryDataFinish_Callback(DataPipeObj_TypeDef *obj)
{
    if ((!SrvDataHub_Monitor.init_state) || (obj == NULL))
        return;

    if (obj == &Receiver_hub_DataPipe)
    {
        SrvDataHub_Monitor.update_reg.bit.rc_control_data = true;

        if (SrvDataHub_Monitor.inuse_reg.bit.rc_control_data)
            SrvDataHub_Monitor.inuse_reg.bit.rc_control_data = false;

        SrvDataHub_Monitor.data.RC_Control_Data = DataPipe_DataObj(Hub_Telemetry_Rc);

        SrvDataHub_Monitor.update_reg.bit.rc_control_data = false;
    }
}

static void SrvDataHub_PipeConvertControlDataFinish_Callback(DataPipeObj_TypeDef *obj)
{
    if ((!SrvDataHub_Monitor.init_state) || (obj == NULL))
        return;

    if (obj == &CtlData_hub_DataPipe)
    {
        SrvDataHub_Monitor.update_reg.bit.cnv_control_data = true;

        if (SrvDataHub_Monitor.inuse_reg.bit.cnv_control_data)
            SrvDataHub_Monitor.inuse_reg.bit.cnv_control_data = false;

        SrvDataHub_Monitor.data.cnvctl_data_time = SrvOsCommon.get_os_ms();
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
}

static bool SrvDataHub_Get_RelativeAlt(uint32_t *time_stamp, float *alt, float *alt_speed)
{
    SrvDataHub_Monitor.inuse_reg.bit.relative_alt = true;

reupdate_relative_alt:
    if (time_stamp)
        *time_stamp = SrvDataHub_Monitor.data.relative_alt_time;
    
    if (alt)
        *alt = SrvDataHub_Monitor.data.relative_alt;

    if (alt_speed)
        *alt_speed = SrvDataHub_Monitor.data.relative_vertical_speed;

    if (!SrvDataHub_Monitor.inuse_reg.bit.relative_alt)
        goto reupdate_relative_alt;

    SrvDataHub_Monitor.inuse_reg.bit.relative_alt = false;

    return true;
}

static bool SrvDataHub_Get_Attitude(uint32_t *time_stamp, float *pitch, float *roll, float *yaw, float *q0, float *q1, float *q2, float *q3)
{
reupdate_attitude:
    SrvDataHub_Monitor.inuse_reg.bit.attitude = true; 
    
    if (time_stamp)
        (*time_stamp) = SrvDataHub_Monitor.data.att_update_time;
    
    if (pitch)
        (*pitch) = SrvDataHub_Monitor.data.att_pitch;
    
    if (roll)
        (*roll) = SrvDataHub_Monitor.data.att_roll;
    
    if (yaw)
        (*yaw) = SrvDataHub_Monitor.data.att_yaw;

    if (q0)
        (*q0) = SrvDataHub_Monitor.data.att_q0;

    if (q1)
        (*q1) = SrvDataHub_Monitor.data.att_q1;

    if (q2)
        (*q2) = SrvDataHub_Monitor.data.att_q2;
    
    if (q3)
        (*q3) = SrvDataHub_Monitor.data.att_q3;

    if(!SrvDataHub_Monitor.inuse_reg.bit.attitude)
        goto reupdate_attitude;

    SrvDataHub_Monitor.inuse_reg.bit.attitude = false;

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
    if(data)
    {
reupdate_telemetry_control_data:
        SrvDataHub_Monitor.inuse_reg.bit.rc_control_data = true;

        (*data) = SrvDataHub_Monitor.data.RC_Control_Data;

        if(!SrvDataHub_Monitor.inuse_reg.bit.rc_control_data)
            goto reupdate_telemetry_control_data;

        SrvDataHub_Monitor.inuse_reg.bit.rc_control_data = false;

        return true;
    }

    return false;
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
    if(state)
    {
reupdate_cli_state:
        SrvDataHub_Monitor.inuse_reg.bit.cli = true;

        (*state) = SrvDataHub_Monitor.data.CLI_state;

        if(!SrvDataHub_Monitor.inuse_reg.bit.cli)
            goto reupdate_cli_state;

        SrvDataHub_Monitor.inuse_reg.bit.cli = false;

        return true;
    }

    return false;
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
    if(state)
    {
reupdate_upgrade_state:
        SrvDataHub_Monitor.inuse_reg.bit.upgrade = true;

        (*state) = SrvDataHub_Monitor.data.upgrading;

        if(!SrvDataHub_Monitor.inuse_reg.bit.upgrade)
            goto reupdate_upgrade_state;

        SrvDataHub_Monitor.inuse_reg.bit.upgrade = false;
        
        return true;
    }

    return false;
}

static bool SrvDataHub_Get_VCPAttach_State(bool *state)
{
    if(state)
    {
reupdate_vcp_attach_state:
        SrvDataHub_Monitor.inuse_reg.bit.USB_VCP_attach = true;

        (*state) = SrvDataHub_Monitor.data.VCP_Attach;

        if(!SrvDataHub_Monitor.inuse_reg.bit.USB_VCP_attach)
            goto reupdate_vcp_attach_state;

        SrvDataHub_Monitor.inuse_reg.bit.USB_VCP_attach = false;
        
        return true;
    }

    return false;
}

