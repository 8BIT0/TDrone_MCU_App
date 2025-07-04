#include "Srv_ComProto.h"
#include "Srv_DataHub.h"
#include "DataPipe.h"
#include "Srv_OsCommon.h"
#include "Bsp_Uart.h"

#define To_DataPack_Callback(x) (DataPack_Callback)x

/* only can use one hardware port at one time */
/* still can be optmize / use multi port proto mavlink frame */

SrvComProto_Monitor_TypeDef SrvComProto_monitor = {
    .Proto_Type = SrvComProto_Type_None,
    .init_state = false,
};

/* internal function */
static uint16_t SrvComProto_MavMsg_Raw_IMU(SrvComProto_MsgInfo_TypeDef *pck);
static uint16_t SrvComProto_MavMsg_Attitude(SrvComProto_MsgInfo_TypeDef *pck);
static uint16_t SrvConProto_MavMsg_RC(SrvComProto_MsgInfo_TypeDef *pck);
static uint16_t SrvComProto_MavMsg_Altitude(SrvComProto_MsgInfo_TypeDef *pck);
static uint16_t SrvComProto_MavMsg_MotoActuator(SrvComProto_MsgInfo_TypeDef *pck);

/* just fot temporary will create custom message in the next */
static uint16_t SrvComProto_MavMsg_Exp_Attitude(SrvComProto_MsgInfo_TypeDef *pck);

/* external function */
static bool Srv_ComProto_Init(SrvComProto_Type_List type, uint8_t *arg);
static bool Srv_ComProto_MsgObj_Init(SrvComProto_MsgInfo_TypeDef *msg, SrvComProto_MavPackInfo_TypeDef pck_info, uint32_t period);
static void SrvComProto_MsgToStream(SrvComProto_MsgInfo_TypeDef *msg, SrvComProto_Stream_TypeDef *com_stream, void *arg, ComProto_Callback tx_cb);
static bool SrvComProto_MsgEnable_Control(SrvComProto_MsgInfo_TypeDef *msg, bool state);
static SrvComProto_Type_List Srv_ComProto_GetType(void);
static SrvComProto_Msg_StreamIn_TypeDef SrvComProto_MavMsg_Input_DecodeAll(SrvComProto_MsgObj_TypeDef *obj, uint8_t *p_data, uint16_t size);
static void SrvComProto_Set_MavMsgIn_Callback(SrvComProto_MsgObj_TypeDef *obj, SrvComProto_MavInMsgType_List type, SrvComProto_MavMsgIn_Callback callback, void *p_cus_data);

SrvComProto_TypeDef SrvComProto = {
    .init = Srv_ComProto_Init,
    .mav_msg_obj_init = Srv_ComProto_MsgObj_Init,
    .get_msg_type = Srv_ComProto_GetType,
    .mav_msg_stream = SrvComProto_MsgToStream,
    .mav_msg_enable_ctl = SrvComProto_MsgEnable_Control,
    .msg_decode = SrvComProto_MavMsg_Input_DecodeAll,
    .mav_msg_set_input_callback = SrvComProto_Set_MavMsgIn_Callback,
};

static bool Srv_ComProto_Init(SrvComProto_Type_List type, uint8_t *arg)
{
    UNUSED(arg);

    /* only init one time */
    if (SrvComProto_monitor.init_state)
        return true;

    SrvDataHub.init();

    memset(&SrvComProto_monitor, 0, sizeof(SrvComProto_monitor));
    SrvComProto_monitor.Proto_Type = type;
    SrvComProto_monitor.init_state = true;

    return true;
}

static SrvComProto_Type_List Srv_ComProto_GetType(void)
{
    return SrvComProto_monitor.Proto_Type;
}

static bool Srv_ComProto_MsgObj_Init(SrvComProto_MsgInfo_TypeDef *msg, SrvComProto_MavPackInfo_TypeDef pck_info, uint32_t period)
{
    if ((msg == NULL) || \
        (period == 0))
        return false;

    msg->in_proto = false;
    msg->lock_proto = true;

    msg->pck_info = pck_info;
    msg->period = period;
    msg->proto_time = 0;

    /* create mavlink message object */
    msg->msg_obj = (mavlink_message_t *)SrvOsCommon.malloc(sizeof(mavlink_message_t));

    if (msg->msg_obj == NULL)
    {
        SrvOsCommon.free(msg->msg_obj);
        return false;
    }

    memset(msg->msg_obj, 0, sizeof(mavlink_message_t));

    /* set mavlink data structure value set function */
    switch ((uint8_t)pck_info.component_id)
    {
    case MAV_CompoID_Attitude:
        msg->pack_callback = To_DataPack_Callback(SrvComProto_MavMsg_Attitude);
        break;

    case MAV_CompoID_Exp_Attitude:
        msg->pack_callback = To_DataPack_Callback(SrvComProto_MavMsg_Exp_Attitude);
        break;
    
    case MAV_CompoID_Altitude:
        msg->pack_callback = To_DataPack_Callback(SrvComProto_MavMsg_Altitude);
        break;

    case MAV_CompoID_RC_Channel:
        msg->pack_callback = To_DataPack_Callback(SrvConProto_MavMsg_RC);
        break;

    case MAV_CompoID_Raw_IMU:
        msg->pack_callback = To_DataPack_Callback(SrvComProto_MavMsg_Raw_IMU);
        break;

    case MAV_CompoID_MotoCtl:
        msg->pack_callback = To_DataPack_Callback(SrvComProto_MavMsg_MotoActuator);
        break;

    default:
        SrvOsCommon.free(msg->msg_obj);
        msg->lock_proto = false;
        return false;
    }

    msg->lock_proto = false;

    return true;
}

static void SrvComProto_MsgToStream(SrvComProto_MsgInfo_TypeDef *msg, SrvComProto_Stream_TypeDef *com_stream, void *arg, ComProto_Callback tx_cb)
{
    uint32_t sys_time = SrvOsCommon.get_os_ms();

    if (msg->enable && com_stream && com_stream->p_buf && msg->pack_callback)
    {
        msg->in_proto = true;

        if ((msg->proto_time) && (sys_time - msg->proto_time < msg->period))
        {
            msg->in_proto = false;
            return;
        }

        com_stream->size = msg->pack_callback((uint8_t *)msg);

        if (com_stream->size && ((com_stream->size + MAVLINK_NUM_NON_PAYLOAD_BYTES) <= com_stream->max_size))
        {
            com_stream->size = mavlink_msg_to_send_buffer(com_stream->p_buf, msg->msg_obj);

            if (tx_cb)
            {
                tx_cb(arg, com_stream->p_buf, com_stream->size);
                msg->proto_cnt ++;
                memset(com_stream->p_buf, 0, com_stream->size);
            }

            msg->proto_time = sys_time;
        }

        msg->in_proto = false;
    }
}

static bool SrvComProto_MsgEnable_Control(SrvComProto_MsgInfo_TypeDef *msg, bool state)
{
    if (msg == NULL)
        return false;

    msg->enable = state;
    return true;
}

/******************************************* Frame Out ****************************************/
static uint16_t SrvComProto_MavMsg_Raw_IMU(SrvComProto_MsgInfo_TypeDef *pck)
{
    uint32_t imu_time = 0;
    uint32_t mag_time = 0;
    float acc_x = 0.0f;
    float acc_y = 0.0f;
    float acc_z = 0.0f;
    float gyr_x = 0.0f;
    float gyr_y = 0.0f;
    float gyr_z = 0.0f;
    float mag_x = 0.0f;
    float mag_y = 0.0f;
    float mag_z = 0.0f;

    SrvDataHub.get_imu(&imu_time, &gyr_x, &gyr_y, &gyr_z, &acc_x, &acc_y, &acc_z);
    SrvDataHub.get_mag(&mag_time, &mag_x, &mag_y, &mag_z);

    int16_t i_acc_x = (int16_t)(acc_x * 1000.0f);
    int16_t i_acc_y = (int16_t)(acc_y * 1000.0f);
    int16_t i_acc_z = (int16_t)(acc_z * 1000.0f);

    int16_t i_gyr_x = (int16_t)(gyr_x * 1000.0f);
    int16_t i_gyr_y = (int16_t)(gyr_y * 1000.0f);
    int16_t i_gyr_z = (int16_t)(gyr_z * 1000.0f);

    int16_t i_mag_x = (int16_t)(mag_x * 10.0f);
    int16_t i_mag_y = (int16_t)(mag_y * 10.0f);
    int16_t i_mag_z = (int16_t)(mag_z * 10.0f);

    /* we dont have any mag sensor currently */
    return mavlink_msg_scaled_imu_pack_chan(pck->pck_info.system_id,
                                            pck->pck_info.component_id,
                                            pck->pck_info.chan, pck->msg_obj,
                                            /* imu_time */mag_time,
                                            i_acc_x, i_acc_y, i_acc_z,
                                            i_gyr_x, i_gyr_y, i_gyr_z,
                                            i_mag_x, i_mag_y, i_mag_z);
}

/* just fot temporary will create custom message in the next */
static uint16_t SrvComProto_MavMsg_Exp_Attitude(SrvComProto_MsgInfo_TypeDef *pck)
{
    uint32_t time = 0;
    float pitch = 0.0f;
    float roll = 0.0f;
    float gx = 0.0f;
    float gy = 0.0f;
    float gz = 0.0f;

    SrvDataHub.get_cnv_control_data(&time, NULL, NULL, &pitch, &roll, &gx, &gy, &gz);
    return mavlink_msg_attitude_pack_chan(pck->pck_info.system_id,
                                          pck->pck_info.component_id,
                                          pck->pck_info.chan, pck->msg_obj,
                                          time, roll, pitch, 0.0f, gx, gy, gz);
}

static uint16_t SrvComProto_MavMsg_Attitude(SrvComProto_MsgInfo_TypeDef *pck)
{
    uint32_t time_stamp = 0;
    
    float pitch = 0.0f;
    float roll = 0.0f;
    float yaw = 0.0f;
    float heading = 0.0f;

    SrvDataHub.get_attitude(&time_stamp, &pitch, &roll, &yaw, &heading);
    return mavlink_msg_attitude_pack_chan(pck->pck_info.system_id,
                                          pck->pck_info.component_id,
                                          pck->pck_info.chan, pck->msg_obj,
                                          time_stamp,
                                          roll, pitch, yaw, 
                                          0.0f, 0.0f, 0.0f);
}

static uint16_t SrvConProto_MavMsg_RC(SrvComProto_MsgInfo_TypeDef *pck)
{
    ControlData_TypeDef inuse_ctldata;

    memset(&inuse_ctldata, 0, sizeof(ControlData_TypeDef));
    SrvDataHub.get_rc_control_data(&inuse_ctldata);

    return mavlink_msg_rc_channels_pack_chan(pck->pck_info.system_id,
                                             pck->pck_info.component_id,
                                             pck->pck_info.chan, pck->msg_obj,
                                             inuse_ctldata.update_time_stamp, 
                                             inuse_ctldata.channel_sum,
                                             inuse_ctldata.all_ch[0],  inuse_ctldata.all_ch[1],  inuse_ctldata.all_ch[2],  inuse_ctldata.all_ch[3],
                                             inuse_ctldata.all_ch[4],  inuse_ctldata.all_ch[5],  inuse_ctldata.all_ch[6],  inuse_ctldata.all_ch[7],
                                             inuse_ctldata.all_ch[8],  inuse_ctldata.all_ch[9],  inuse_ctldata.all_ch[10], inuse_ctldata.all_ch[11],
                                             inuse_ctldata.all_ch[12], inuse_ctldata.all_ch[13], inuse_ctldata.all_ch[14], inuse_ctldata.all_ch[15],
                                             inuse_ctldata.all_ch[16], inuse_ctldata.all_ch[17], inuse_ctldata.rssi);
}

static uint16_t SrvComProto_MavMsg_Altitude(SrvComProto_MsgInfo_TypeDef *pck)
{
    // uint32_t time_stamp = 0;
    // uint8_t error = 0;
    // float baro_pressure = 0.0f;
    // float baro_alt = 0.0f;
    // float baro_alt_offset = 0.0f;
    // float baro_tempra = 0.0f;

    // uint32_t relalt_time_stamp = 0;
    // float rel_alt = 0.0f;
    // float rel_alt_speed = 0.0f;

    // float acc_z = 0.0f;

    // SrvDataHub.get_baro_altitude(&time_stamp, &baro_pressure, &baro_alt, &baro_alt_offset, &baro_tempra, &error);
    // SrvDataHub.get_relative_alt(&relalt_time_stamp, &rel_alt, &rel_alt_speed);
    // SrvDataHub.get_scaled_imu(NULL, NULL, NULL, NULL, NULL, &acc_z, NULL, NULL, NULL, NULL, NULL);

    // return mavlink_msg_altitude_pack_chan(pck->pck_info.system_id,
    //                                       pck->pck_info.component_id,
    //                                       pck->pck_info.chan, pck->msg_obj,
    //                                       time_stamp,
    //                                       baro_alt, baro_pressure, acc_z, rel_alt, rel_alt_speed, 0);
    return 0;
}

static uint16_t SrvComProto_MavMsg_MotoActuator(SrvComProto_MsgInfo_TypeDef *pck)
{
    // uint32_t time_stamp = 0;
    // uint8_t moto_num = 0;
    // int16_t m_b[ACTUATOR_MOTO_COUNT];

    // memset(m_b, 0, sizeof(m_b));
    // SrvDataHub.get_moto(&time_stamp, m_b);

    // return mavlink_msg_servo_output_raw_pack_chan(pck->pck_info.system_id,
    //                                               pck->pck_info.component_id,
    //                                               pck->pck_info.chan, pck->msg_obj,
    //                                               time_stamp, moto_num,
    //                                               m_b[0], m_b[1], m_b[2], m_b[3],
    //                                               m_b[4], m_b[5], m_b[6], m_b[7]);

    return 0;
}

/******************************************* Frame Out ****************************************/
/******************************************* Frame In  ****************************************/
static void SrvComProto_Set_MavMsgIn_Callback(SrvComProto_MsgObj_TypeDef *obj, SrvComProto_MavInMsgType_List type, SrvComProto_MavMsgIn_Callback callback, void *p_cus_data)
{
    if (obj && p_cus_data)
    {
        switch ((uint8_t)type)
        {
            case MavIn_Msg_Gyr:         obj->MavMsg_Gyr_Callback         = callback; obj->cus_p_gyr         = p_cus_data; break;
            case MavIn_Msg_Att:         obj->MavMsg_Att_Callback         = callback; obj->cus_p_att         = p_cus_data; break;
            case MavIn_Msg_Alt:         obj->MavMsg_Alt_Callback         = callback; obj->cus_p_alt         = p_cus_data; break;
            case MavIn_Msg_RC:          obj->MavMsg_RC_Callback          = callback; obj->cus_p_RC          = p_cus_data; break;
            case MavIn_Msg_MotoCtl:     obj->MavMsg_MotoCtl_Callback     = callback; obj->cus_p_moto        = p_cus_data; break;
            default: break;
        }
    }
}

static bool SrvComProto_MavMsg_Decode_ExpAttiude(SrvComProto_MsgObj_TypeDef *obj, const mavlink_message_t msg)
{
    mavlink_attitude_t msg_att;

    if (obj)
    {
        obj->Att_Cnt ++;

        memset(&msg_att, 0, sizeof(mavlink_attitude_t));
        mavlink_msg_attitude_decode(&msg, &msg_att);

        if (obj->MavMsg_Att_Callback)
            return obj->MavMsg_Att_Callback(&msg_att, obj->cus_p_att);
    }

    return false;
}

static bool SrvComProto_MavMsg_Decode_ExpGyro(SrvComProto_MsgObj_TypeDef *obj, const mavlink_message_t msg)
{
    mavlink_raw_imu_t msg_imu;

    if (obj)
    {
        obj->Gyr_Cnt ++;

        memset(&msg_imu, 0, sizeof(mavlink_raw_imu_t));
        mavlink_msg_raw_imu_decode(&msg, &msg_imu);

        if (obj->MavMsg_Gyr_Callback)
            return obj->MavMsg_Gyr_Callback(&msg_imu, obj->cus_p_gyr);
    }

    return false;
}

static bool SrvComProto_MavMsg_Decode_ExpAlt(SrvComProto_MsgObj_TypeDef *obj, const mavlink_message_t msg)
{
    mavlink_altitude_t msg_alt;

    if (obj)
    {
        obj->Alt_Cnt ++;

        memset(&msg_alt, 0, sizeof(mavlink_altitude_t));
        mavlink_msg_altitude_decode(&msg, &msg_alt);

        if (obj->MavMsg_Alt_Callback)
            return obj->MavMsg_Alt_Callback(&msg_alt, obj->cus_p_alt);
    }

    return false;
}

static bool SrvComProto_MavMsg_Decode_ExpMoto(SrvComProto_MsgObj_TypeDef *obj, const mavlink_message_t msg)
{
    mavlink_servo_output_raw_t msg_moto;

    if (obj)
    {
        obj->Moto_Cnt ++;

        memset(&msg_moto, 0, sizeof(mavlink_servo_output_raw_t));
        mavlink_msg_servo_output_raw_decode(&msg, &msg_moto);

        if (obj->MavMsg_MotoCtl_Callback)
            return obj->MavMsg_MotoCtl_Callback(&msg_moto, obj->cus_p_moto);
    }

    return false;
}

static bool SrvComProto_MavMsg_Decode_ExpRC(SrvComProto_MsgObj_TypeDef *obj, const mavlink_message_t msg)
{
    mavlink_rc_channels_t msg_rc;

    if (obj)
    {
        obj->RC_Cnt ++;

        memset(&msg_rc, 0, sizeof(mavlink_rc_channels_t));
        mavlink_msg_rc_channels_decode(&msg, &msg_rc);

        if (obj->MavMsg_RC_Callback)
            return obj->MavMsg_RC_Callback(&msg_rc, obj->cus_p_RC);
    }

    return false;
}

static bool SrvComProto_MavMsg_Decode_PIDPara_GyroX(SrvComProto_MsgObj_TypeDef *obj, const mavlink_message_t msg)
{
    mavlink_para_gyrox_pid_t msg_pid_para_gyrox;

    if (obj)
    {
        obj->PID_Para_GyrX_Cnt ++;

        memset(&msg_pid_para_gyrox, 0, sizeof(mavlink_para_gyrox_pid_t));
        mavlink_msg_para_gyrox_pid_decode(&msg, &msg_pid_para_gyrox);
    
        if (obj->MavMsg_PIDPara_GyroX_Callback)
            return obj->MavMsg_PIDPara_GyroX_Callback(&msg_pid_para_gyrox, obj->cus_p_pidpara_gyrox);
    }

    return false;
}

static bool SrvComProto_MavMsg_Decode_PIDPara_GyroY(SrvComProto_MsgObj_TypeDef *obj, const mavlink_message_t msg)
{
    mavlink_para_gyroy_pid_t msg_pid_para_gyroy;
    
    if (obj)
    {
        obj->PID_Para_GyrY_Cnt ++;

        memset(&msg_pid_para_gyroy, 0, sizeof(mavlink_para_gyroy_pid_t));
        mavlink_msg_para_gyroy_pid_decode(&msg, &msg_pid_para_gyroy);
    
        if (obj->MavMsg_PIDPara_GyroY_Callback)
            return obj->MavMsg_PIDPara_GyroY_Callback(&msg_pid_para_gyroy, obj->cus_p_pidpara_gyroy);
    }

    return false;
}

static bool SrvComProto_MavMsg_Decode_PIDPara_GyroZ(SrvComProto_MsgObj_TypeDef *obj, const mavlink_message_t msg)
{
    mavlink_para_gyroz_pid_t msg_pid_para_gyroz;
    
    if (obj)
    {
        obj->PID_Para_GyrZ_Cnt ++;

        memset(&msg_pid_para_gyroz, 0, sizeof(mavlink_para_gyroz_pid_t));
        mavlink_msg_para_gyroz_pid_decode(&msg, &msg_pid_para_gyroz);

        if (obj->MavMsg_PIDPara_GyroZ_Callback)
            return obj->MavMsg_PIDPara_GyroZ_Callback(&msg_pid_para_gyroz, obj->cus_p_pidpara_gyroz);
    }

    return false;
}

static bool SrvComProto_MavMsg_Decode_PIDPara_Roll(SrvComProto_MsgObj_TypeDef *obj, const mavlink_message_t msg)
{
    mavlink_para_roll_pid_t msg_pid_para_roll;

    if (obj)
    {
        obj->PID_Para_Roll_Cnt ++;

        memset(&msg_pid_para_roll, 0, sizeof(mavlink_para_roll_pid_t));
        mavlink_msg_para_roll_pid_decode(&msg, &msg_pid_para_roll);
    
        if (obj->MavMsg_PIDPara_Roll_Callback)
            return obj->MavMsg_PIDPara_Roll_Callback(&msg_pid_para_roll, obj->cus_p_pidpara_roll);
    }

    return false;
}

static bool SrvComProto_MavMsg_Decode_PIDPara_Pitch(SrvComProto_MsgObj_TypeDef *obj, const mavlink_message_t msg)
{
    mavlink_para_pitch_pid_t msg_pid_para_pitch;

    if (obj)
    {
        obj->PID_Para_Pitch_Cnt ++;

        memset(&msg_pid_para_pitch, 0, sizeof(mavlink_para_pitch_pid_t));
        mavlink_msg_para_pitch_pid_decode(&msg, &msg_pid_para_pitch);

        if (obj->MavMsg_PIDPara_Pitch_Callback)
            return obj->MavMsg_PIDPara_Pitch_Callback(&msg_pid_para_pitch, obj->cus_p_pidpara_pitch);
    }

    return false;
}

/******************************************* Frame In  ****************************************/
static SrvComProto_Msg_StreamIn_TypeDef SrvComProto_MavMsg_Input_DecodeAll(SrvComProto_MsgObj_TypeDef *obj, uint8_t *p_data, uint16_t size)
{
    SrvComProto_Msg_StreamIn_TypeDef stream_in; 
    uint8_t default_channel = 0;
    mavlink_message_t mav_msg;
    mavlink_status_t mav_sta;
    volatile uint8_t mav_decode = 0;
    
    memset(&stream_in, 0, sizeof(SrvComProto_Msg_StreamIn_TypeDef));

    /* match cli */
    if((p_data[size - 1] == '\n') && (p_data[size - 2] == '\r'))
    {
        stream_in.pac_type = ComFrame_CLI;
        stream_in.valid = true;
        stream_in.size = size;
        stream_in.p_buf = p_data;
    
        goto input_stream_valid;
    }
    
    memset(&mav_msg, 0, sizeof(mavlink_message_t));
    memset(&mav_sta, 0, sizeof(mavlink_status_t));
    
    /* match mavlink message */
    for(uint16_t i = 0; i < size; i++)
    {
        mav_decode = mavlink_frame_char(default_channel, p_data[i], &mav_msg, &mav_sta);

        if (mav_decode == MAVLINK_FRAMING_OK)
        {
            /* get multi protocol frame in one transmition and first protocol frame is mavlink message */
            /* only decode first pack */
            if (obj && (mav_msg.sysid == MAV_SysID_Radio))
            {
                switch ((uint8_t)mav_msg.compid)
                {
                    case MAV_CompoID_Ctl_Gyro:          SrvComProto_MavMsg_Decode_ExpGyro(obj, mav_msg);        break;
                    case MAV_CompoID_Ctl_Attitude:      SrvComProto_MavMsg_Decode_ExpAttiude(obj, mav_msg);     break;
                    case MAV_CompoID_Ctl_Altitude:      SrvComProto_MavMsg_Decode_ExpAlt(obj, mav_msg);         break;
                    case MAV_CompoID_Ctl_RC_Channel:    SrvComProto_MavMsg_Decode_ExpRC(obj, mav_msg);          break;
                    case MAV_CompoID_Ctl_MotoCtl:       SrvComProto_MavMsg_Decode_ExpMoto(obj, mav_msg);        break;
                    case MAV_CompoID_Ctl_PIDPara_GyrX:  SrvComProto_MavMsg_Decode_PIDPara_GyroX(obj, mav_msg);  break;
                    case MAV_CompoID_Ctl_PIDPara_GyrY:  SrvComProto_MavMsg_Decode_PIDPara_GyroY(obj, mav_msg);  break;
                    case MAV_CompoID_Ctl_PIDPara_GyrZ:  SrvComProto_MavMsg_Decode_PIDPara_GyroZ(obj, mav_msg);  break;
                    case MAV_CompoID_Ctl_PIDPara_Roll:  SrvComProto_MavMsg_Decode_PIDPara_Roll(obj, mav_msg);   break;
                    case MAV_CompoID_Ctl_PIDPata_Pitch: SrvComProto_MavMsg_Decode_PIDPara_Pitch(obj, mav_msg);  break;
                    default: break;
                }
            }
            
            /* mavlink frame ack */
            // SrvComProto_MavMsg_Ack(obj, MAV_SysID_Drone, uint16_t compo_id, decode_state);

            memset(&mav_msg, 0, sizeof(mavlink_message_t));
            memset(&mav_sta, 0, sizeof(mavlink_status_t));
        }
        else if (mav_decode == MAVLINK_FRAMING_BAD_CRC)
        {
            memset(&mav_msg, 0, sizeof(mavlink_message_t));
            memset(&mav_sta, 0, sizeof(mavlink_status_t));
        }
    }

    /* custom frame input check */
    stream_in.pac_type = ComFrame_MavMsg;
    stream_in.valid = true;
    stream_in.size = size;
    stream_in.p_buf = p_data;

input_stream_valid:
    return stream_in;
}
