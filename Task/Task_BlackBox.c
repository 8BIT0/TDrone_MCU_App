/*
 * Auther: 8_B!T0
 * this file will replace task_log
 * drone black box task
 */
#include "FreeRTOS.h"
#include "task.h"
#include "Task_BlackBox.h"
#include "Dev_Led.h"
#include "shell_port.h"
#include "debug_util.h"
#include "../DataStructure/CusQueue.h"
#include "error_log.h"
#include "../DataPipe/DataPipe.h"
#include "../System/storage/Storage.h"
#include "HW_Def.h"
#include "minilzo.h"
#include "Srv_OsCommon.h"
#include "Srv_BlackBox_Def.h"

#define PeriphSec __attribute__((section(".Perph_Section")))
#define BlackBox_Buff_Size (8 Kb)
#define BlackBox_Storage_Name "BlackBox"

typedef enum
{
    BlackBox_Log_Idle = 0,
    BlackBox_Log_Enable,
    BlackBox_Log_Disable,
} BlackBox_LogState_List;

typedef enum
{
    BlackBox_Cnv_None_Error = 0,
    BlackBox_Cnv_Para_Error,
    BlackBox_Cnv_Size_Error,
    BlackBox_Cnv_Type_Error,
    BlackBox_Cnv_Header_Error,
    BlackBox_Cnv_Ender_Error,
} BlackBox_ConvertError_List;

typedef struct
{
    uint32_t cnt;
    uint32_t byte_size;
} BlackBox_LogStatistic_TypeDef;

#pragma pack(1)
typedef struct
{
    BlackBox_LogType_List log_type;
    uint32_t log_size;
} BlackBox_LogInfo_TypeDef;
#pragma pack()

typedef struct
{
    BlackBox_LogState_List state;
    bool write_thread_state;
    
    Storage_ItemSearchOut_TypeDef storage_search;
    BlackBox_LogType_List log_type;
    uint32_t expection_log_size;

    uint32_t log_unit;
    uint8_t *p_log_buf;
    uint32_t log_byte_size;
    uint32_t log_cnt;
    BlackBox_LogStatistic_TypeDef imu_log;
    BlackBox_LogStatistic_TypeDef alt_att_log; 
    BlackBox_LogStatistic_TypeDef ctl_ang_log;
    BlackBox_LogStatistic_TypeDef ctl_att_log;
} BlackBox_Monitor_TypeDef;

/* internal vriable */
static uint8_t BlackBox_Buff[BlackBox_Buff_Size] PeriphSec;
static QueueObj_TypeDef BlackBox_Queue;
static BlackBox_Monitor_TypeDef Monitor;
static osSemaphoreId BlackBox_Sem = 0;
static SrvBlackBox_TypeDef *p_blackbox = NULL;
DataPipe_CreateDataObj(IMUAtt_TypeDef,            LogAtt_Data);
DataPipe_CreateDataObj(RelMov_TypeDef,            LogAlt_Data);
DataPipe_CreateDataObj(ExpControlData_TypeDef,    LogCtl_Data);

/* internal function */
static void TaskBlackBox_PipeTransFinish_Callback(DataPipeObj_TypeDef *obj);
static void TaskBlackBox_PushFinish_Callback(void);

void TaskBlackBox_Init(void)
{
    uint32_t unit = 0;
    StorageDevObj_TypeDef StorageDevInfo; 
    SrvBlackBox_DevInfo_TypeDef BlackBoxDevInfo;
    BlackBox_LogInfo_TypeDef BlackBoxInfo;
    /* medium init */
    /* medium auto detect */
    /*
        --- TF Card
        --- W25Qxx Storage Chip
        --- Com Output
     */
    memset(&StorageDevInfo, 0, sizeof(StorageDevObj_TypeDef));
    memset(&BlackBoxDevInfo, 0, sizeof(SrvBlackBox_DevInfo_TypeDef));
    memset(&BlackBoxInfo, 0, sizeof(BlackBox_LogInfo_TypeDef));
    memset(&Monitor, 0, sizeof(BlackBox_Monitor_TypeDef));
    Monitor.state = BlackBox_Log_Idle;

    /* read blackbox info from storage */
    Monitor.storage_search = Storage.search(Para_User, BlackBox_Storage_Name);
    if (Monitor.storage_search.item_addr == 0)
    {
        BlackBoxInfo.log_size = 0;

        BlackBoxInfo.log_type = BlackBox_AngularPID_Tune;
        if (Storage.create(Para_User, BlackBox_Storage_Name, (uint8_t *)&BlackBoxInfo, sizeof(BlackBox_LogInfo_TypeDef)) != Storage_Error_None)
            return;
    }
    else
    {
        if (Storage.get(Para_User, Monitor.storage_search.item, (uint8_t *)&BlackBoxInfo, sizeof(BlackBox_LogInfo_TypeDef)) != Storage_Error_None)
            return;

        Monitor.log_type = BlackBoxInfo.log_type;
        Monitor.expection_log_size = BlackBoxInfo.log_size;
    }

    Monitor.write_thread_state = false;
    Storage.get_dev_info(&StorageDevInfo);
    BlackBoxDevInfo.phy_start_addr = BlackBox_Storage_Start_Addr;
    BlackBoxDevInfo.str_start_addr = StorageDevInfo.start_addr;
    BlackBoxDevInfo.total_str_size = StorageDevInfo.total_size;
    BlackBoxDevInfo.sector_size = StorageDevInfo.sector_size;

    unit = SrvChip_BlackBox.init(TaskBlackBox_PushFinish_Callback, BlackBoxDevInfo);
    p_blackbox = &SrvChip_BlackBox;
    
    if (unit == 0)
    {
        p_blackbox = NULL;
        return;
    }

    switch ((uint8_t) Monitor.log_type)
    {
        case BlackBox_Imu_Filted:
        case BlackBox_Log_Alt_Att:
        case BlackBox_AngularPID_Tune:
        case BlackBox_AttitudePID_Tune:
            break;

        case BlackBox_Log_None:
        default: return;
    }

    Monitor.p_log_buf = SrvOsCommon.malloc(unit);
    if (Monitor.p_log_buf == NULL)
    {
        SrvOsCommon.free(Monitor.p_log_buf);
        return;
    }

    Monitor.log_unit = unit;
    memset(&Attitude_log_DataPipe, 0, sizeof(DataPipeObj_TypeDef));
    memset(&Altitude_log_DataPipe, 0, sizeof(DataPipeObj_TypeDef));
    memset(&CtlData_log_DataPipe,  0, sizeof(DataPipeObj_TypeDef));

    memset(DataPipe_DataObjAddr(LogAtt_Data),  0, DataPipe_DataSize(LogAtt_Data));
    memset(DataPipe_DataObjAddr(LogAlt_Data),  0, DataPipe_DataSize(LogAlt_Data));
    memset(DataPipe_DataObjAddr(LogCtl_Data),  0, DataPipe_DataSize(LogCtl_Data));

    if (!Queue.create_with_buf(&BlackBox_Queue, "BlackBox_Queue", BlackBox_Buff, BlackBox_Buff_Size))
        return;

    osSemaphoreDef(Log);
    BlackBox_Sem = osSemaphoreCreate(osSemaphore(Log), 32);

    if (BlackBox_Sem == 0)
        return;

    /* pipe object init */
    CtlData_log_DataPipe.data_addr = (uint32_t)DataPipe_DataObjAddr(LogCtl_Data);
    CtlData_log_DataPipe.data_size = DataPipe_DataSize(LogCtl_Data);
    CtlData_log_DataPipe.trans_finish_cb = (Pipe_TransFinish_Callback)TaskBlackBox_PipeTransFinish_Callback;

    Attitude_log_DataPipe.data_addr = (uint32_t)DataPipe_DataObjAddr(LogAtt_Data);
    Attitude_log_DataPipe.data_size = DataPipe_DataSize(LogAtt_Data);
    Attitude_log_DataPipe.trans_finish_cb = (Pipe_TransFinish_Callback)TaskBlackBox_PipeTransFinish_Callback;

    Altitude_log_DataPipe.data_addr = (uint32_t)DataPipe_DataObjAddr(LogAlt_Data);
    Altitude_log_DataPipe.data_size = DataPipe_DataSize(LogAlt_Data);
    Altitude_log_DataPipe.trans_finish_cb = (Pipe_TransFinish_Callback)TaskBlackBox_PipeTransFinish_Callback;

    DataPipe_Enable(&CtlData_log_DataPipe);
    DataPipe_Enable(&Attitude_log_DataPipe);
    DataPipe_Enable(&Altitude_log_DataPipe);

    /* disable log first */
    Monitor.state = BlackBox_Log_Idle;
}

void TaskBlackBox_Core(void const *arg)
{
    Monitor.write_thread_state = true;

    while (true)
    {
        /* task create failed */
        if ((BlackBox_Sem == NULL) || (p_blackbox == NULL))
            vTaskDelete(NULL);

        osSemaphoreWait(BlackBox_Sem, osWaitForever);
        if ((Monitor.state == BlackBox_Log_Disable) && p_blackbox->disable && p_blackbox->disable(Storage.write_section))
        {
            Monitor.state = BlackBox_Log_Idle;
            Queue.reset(&BlackBox_Queue);
        }
        else if (p_blackbox->push && Monitor.log_unit)
        {
            if(p_blackbox->push(Storage.write_section, Monitor.p_log_buf, Monitor.log_unit))
            {
                Monitor.log_cnt ++;
                Monitor.log_byte_size += Monitor.log_unit;
            }
        }
    }
}

static uint8_t TaskBlackBox_Get_CheckSum(uint8_t *p_data, uint16_t len)
{
    uint8_t check_sum = 0;

    if (p_data && len)
        for (uint16_t i = 0; i < len; i++)
            check_sum += p_data[i];

    return check_sum;
}

/* BlackBox Tansmit Finished Callback */
static void TaskBlackBox_PushFinish_Callback(void)
{

}

/* PIPE Callback */
static void TaskBlackBox_UpdateQueue(uint8_t *p_data, uint16_t size)
{
    uint32_t offset = 0;
    uint32_t Queue_remain = Queue.remain(BlackBox_Queue);
    
    if (size > Queue_remain)
    {
        offset = Queue_remain;
        Queue.push(&BlackBox_Queue, p_data, Queue_remain);
    }
    else
        Queue.push(&BlackBox_Queue, p_data, size);

    if (Queue.size(BlackBox_Queue) >= Monitor.log_unit)
    {
        Queue.pop(&BlackBox_Queue, Monitor.p_log_buf, Monitor.log_unit);
        osSemaphoreRelease(BlackBox_Sem);
    }

    if (offset)
        Queue.push(&BlackBox_Queue, p_data + offset, size - offset);
}

static void TaskBlackBox_PipeTransFinish_Callback(DataPipeObj_TypeDef *obj)
{
    static BlackBox_IMUData_TypeDef  imu_data;
    static BlackBox_BaroData_TypeDef baro_data;
    static BlackBox_AttAltData_TypeDef att_alt_data;
    static BlackBox_AngCtlData_TypeDef ang_ctl_data;
    static BlackBox_AttCtlData_TypeDef att_ctl_data;
    static BlackBox_LogState_List lst_state;
    static bool alt_update = false;
    static bool att_update = false;
    static bool imu_update = false;
    static bool ctl_update = false;
    BlackBox_DataHeader_TypeDef blackbox_header;
    BlackBox_DataEnder_TypeDef blackbox_ender;
    uint8_t check_sum = 0;

    memset(&blackbox_header, 0, BLACKBOX_HEADER_SIZE);
    memset(&blackbox_ender,  0, BLACKBOX_ENDER_SIZE);

    blackbox_header.header = BLACKBOX_LOG_HEADER;
    blackbox_ender.ender   = BLACKBOX_LOG_ENDER;

    if ((Monitor.log_type == BlackBox_Log_None) || !Monitor.write_thread_state)
        return;

    if (lst_state == BlackBox_Log_Enable)
    {
        /* pipe data update */
        /* log raw and filted sensor data */
        if ((obj == &Attitude_log_DataPipe) || (obj == &Altitude_log_DataPipe))
        {
            /* set imu data */
            att_alt_data.gyr_scale = imu_data.gyr_scale;
            att_alt_data.acc_scale = imu_data.acc_scale;
            memcpy(att_alt_data.acc, imu_data.flt_acc, sizeof(imu_data.flt_acc));
            memcpy(att_alt_data.gyr, imu_data.flt_gyr, sizeof(imu_data.flt_gyr));

            /* set baro data */
            att_alt_data.baro = baro_data.press;

            /* log data when attitude update */
            if (obj == &Attitude_log_DataPipe)
            {
                /* set attitude */
                att_alt_data.time = DataPipe_DataObj(LogAtt_Data).time_stamp;
                att_alt_data.pitch = DataPipe_DataObj(LogAtt_Data).pitch;
                att_alt_data.roll = DataPipe_DataObj(LogAtt_Data).roll;
                att_alt_data.yaw = DataPipe_DataObj(LogAtt_Data).yaw;
                att_update = true;
            }
            else if (obj == &Altitude_log_DataPipe)
            {
                /* set altitude */
                att_alt_data.alt = DataPipe_DataObj(LogAlt_Data).pos;
                alt_update = true;
            }
        }
        else if (obj == &CtlData_log_DataPipe)
        {
            imu_data.throttle_percent = DataPipe_DataObj(LogCtl_Data).throttle_percent;
            baro_data.throttle_percent = DataPipe_DataObj(LogCtl_Data).throttle_percent;

            /* test code */
            // if (DataPipe_DataObj(LogCtl_Data).control_mode == Attitude_Control)
            // {
            //     att_ctl_data.time = DataPipe_DataObj(LogCtl_Data).time_stamp;
            //     att_ctl_data.throttle_percent = DataPipe_DataObj(LogCtl_Data).throttle_percent;
            //     att_ctl_data.exp_pitch = DataPipe_DataObj(LogCtl_Data).pitch;
            //     att_ctl_data.exp_roll = DataPipe_DataObj(LogCtl_Data).roll;
            //     att_ctl_data.exp_gyr_z = DataPipe_DataObj(LogCtl_Data).gyr_z;
            //     att_ctl_data.pitch = att_alt_data.pitch;
            //     att_ctl_data.roll = att_alt_data.roll;
            //     att_ctl_data.gyr_z = imu_data.flt_gyr[Axis_Z];
            // }
            // else
            /* test code */
            {
                ang_ctl_data.time = DataPipe_DataObj(LogCtl_Data).time_stamp;
                ang_ctl_data.throttle_percent = DataPipe_DataObj(LogCtl_Data).throttle_percent;
                ang_ctl_data.e_gyr[Axis_X] = DataPipe_DataObj(LogCtl_Data).gyr_x;
                ang_ctl_data.e_gyr[Axis_Y] = DataPipe_DataObj(LogCtl_Data).gyr_y;
                ang_ctl_data.e_gyr[Axis_Z] = DataPipe_DataObj(LogCtl_Data).gyr_z;
                ang_ctl_data.m_gyr[Axis_X] = imu_data.flt_gyr[Axis_X] / imu_data.gyr_scale;
                ang_ctl_data.m_gyr[Axis_Y] = imu_data.flt_gyr[Axis_Y] / imu_data.gyr_scale;
                ang_ctl_data.m_gyr[Axis_Z] = imu_data.flt_gyr[Axis_Z] / imu_data.gyr_scale;
            }

            ctl_update = true;
        }

        /* blackbox log control */
        if ((Monitor.log_type == BlackBox_Imu_Filted) && imu_update)
        {
            /* log data when imu update */
            Monitor.imu_log.cnt ++;
            Monitor.imu_log.byte_size += BLACKBOX_HEADER_SIZE;
            blackbox_header.type = BlackBox_Imu_Filted;
            blackbox_header.size = sizeof(BlackBox_IMUData_TypeDef);
            TaskBlackBox_UpdateQueue((uint8_t *)&blackbox_header, BLACKBOX_HEADER_SIZE);
            
            TaskBlackBox_UpdateQueue((uint8_t *)&imu_data, sizeof(BlackBox_IMUData_TypeDef));
            Monitor.imu_log.byte_size += sizeof(BlackBox_IMUData_TypeDef);

            check_sum = TaskBlackBox_Get_CheckSum((uint8_t *)&imu_data, sizeof(BlackBox_IMUData_TypeDef));
            blackbox_ender.check_sum = check_sum;
            Monitor.imu_log.byte_size += BLACKBOX_ENDER_SIZE;
            TaskBlackBox_UpdateQueue((uint8_t *)&blackbox_ender, BLACKBOX_ENDER_SIZE);
            imu_update = false;
        }
        else if ((Monitor.log_type == BlackBox_Log_Alt_Att) && (alt_update & att_update & imu_update))
        {
            Monitor.alt_att_log.cnt ++;
            Monitor.alt_att_log.byte_size += BLACKBOX_HEADER_SIZE;
            blackbox_header.type = BlackBox_Log_Alt_Att;
            blackbox_header.size = sizeof(BlackBox_AttAltData_TypeDef);
            TaskBlackBox_UpdateQueue((uint8_t *)&blackbox_header, BLACKBOX_HEADER_SIZE);
            
            TaskBlackBox_UpdateQueue((uint8_t *)&att_alt_data, sizeof(BlackBox_AttAltData_TypeDef));
            Monitor.alt_att_log.byte_size += sizeof(BlackBox_AttAltData_TypeDef);

            check_sum = TaskBlackBox_Get_CheckSum((uint8_t *)&att_alt_data, sizeof(BlackBox_AttAltData_TypeDef));
            blackbox_ender.check_sum = check_sum;
            Monitor.imu_log.byte_size += BLACKBOX_ENDER_SIZE;
            TaskBlackBox_UpdateQueue((uint8_t *)&blackbox_ender, BLACKBOX_ENDER_SIZE);
            alt_update = false;
            att_update = false;
            imu_update = false;
        }
        else if ((Monitor.log_type == BlackBox_AngularPID_Tune) && (imu_update & ctl_update))
        {
            Monitor.ctl_ang_log.cnt ++;
            Monitor.ctl_ang_log.byte_size += BLACKBOX_HEADER_SIZE;
            blackbox_header.type = BlackBox_AngularPID_Tune;
            blackbox_header.size = sizeof(BlackBox_AngCtlData_TypeDef);
            TaskBlackBox_UpdateQueue((uint8_t *)&blackbox_header, BLACKBOX_HEADER_SIZE);

            TaskBlackBox_UpdateQueue((uint8_t *)&ang_ctl_data, sizeof(BlackBox_AngCtlData_TypeDef));
            Monitor.ctl_ang_log.byte_size += sizeof(BlackBox_AngCtlData_TypeDef);
        
            check_sum = TaskBlackBox_Get_CheckSum((uint8_t *)&ang_ctl_data, sizeof(BlackBox_AngCtlData_TypeDef));
            blackbox_ender.check_sum = check_sum;
            Monitor.ctl_ang_log.byte_size += BLACKBOX_ENDER_SIZE;
            TaskBlackBox_UpdateQueue((uint8_t *)&blackbox_ender, BLACKBOX_ENDER_SIZE);
            imu_update = false;
            ctl_update = false;
        }
        else if ((Monitor.log_type == BlackBox_AttitudePID_Tune) && (imu_update & ctl_update & att_update))
        {
            Monitor.ctl_att_log.cnt ++;
            Monitor.ctl_att_log.byte_size += BLACKBOX_HEADER_SIZE;
            blackbox_header.type = BlackBox_AttitudePID_Tune;
            blackbox_header.size = sizeof(BlackBox_AttCtlData_TypeDef);
            TaskBlackBox_UpdateQueue((uint8_t *)&blackbox_header, BLACKBOX_HEADER_SIZE);

            TaskBlackBox_UpdateQueue((uint8_t *)&att_ctl_data, sizeof(BlackBox_AttCtlData_TypeDef));
            Monitor.ctl_att_log.byte_size += sizeof(BlackBox_AttCtlData_TypeDef);

            check_sum = TaskBlackBox_Get_CheckSum((uint8_t *)&att_ctl_data, sizeof(BlackBox_AttCtlData_TypeDef));
            blackbox_ender.check_sum = check_sum;
            Monitor.ctl_att_log.byte_size += BLACKBOX_ENDER_SIZE;
            TaskBlackBox_UpdateQueue((uint8_t *)&blackbox_ender, BLACKBOX_ENDER_SIZE);
            imu_update = false;
            ctl_update = false;
            att_update = false;
        }
        /* use minilzo compress data if have to */
    }
    else
    {
        /* log disable */
        if (Monitor.state == BlackBox_Log_Disable)
            osSemaphoreRelease(BlackBox_Sem);
    }

    lst_state = Monitor.state;
}

bool TaskBlackBox_LogControl(void)
{
    uint8_t time_out = 100; /* unit: ms */
    DevLedObj_TypeDef BlackBox_Noti;

    memset(&BlackBox_Noti, 0, sizeof(DevLedObj_TypeDef));
    if (BlackBox_Noti_Ptr)
        memcpy(&BlackBox_Noti, BlackBox_Noti_Ptr, sizeof(DevLedObj_TypeDef));

    if (Monitor.state != BlackBox_Log_Enable)
    {
        Monitor.log_cnt = 0;
        Monitor.log_byte_size = 0;
        Queue.reset(&BlackBox_Queue);

        if (p_blackbox && p_blackbox->enable)
        {
            while (!p_blackbox->enable())
            {
                if (time_out == 0)
                    return false;

                SrvOsCommon.delay_ms(1);
                time_out --;
            }

            Monitor.state = BlackBox_Log_Enable;

            if (BlackBox_Noti_Ptr)
                DevLED.ctl(BlackBox_Noti, true);
        }
    }
    else
    {
        if (BlackBox_Noti_Ptr)
            DevLED.ctl(BlackBox_Noti, false);

        Monitor.state = BlackBox_Log_Disable;
    }

    return true;
}

/****************************************************************** shell api implement ***************************************************** */
static void TaskBlackBox_Set_Log(uint8_t medium, uint8_t type, uint32_t size)
{
    Shell *shell_obj = Shell_GetInstence();
    Storage_ItemSearchOut_TypeDef search_out;
    BlackBox_LogInfo_TypeDef info;

    if (shell_obj == NULL)
        return;
    
    shellPrint(shell_obj, "[ BlackBox ] ----- Log Type List\r\n");
    shellPrint(shell_obj, "\tLog None --------- 0\r\n");
    shellPrint(shell_obj, "\tLog Imu Only ----- 1\r\n");
    shellPrint(shell_obj, "\tLog Att & Alt ---- 2\r\n");
    shellPrint(shell_obj, "\tLog PID Angular -- 3\r\n");
    shellPrint(shell_obj, "\tLog PID Attitude - 4\r\n");
    shellPrint(shell_obj, "\r\n");

    if ((type > (uint8_t)BlackBox_AttitudePID_Tune))
    {
        shellPrint(shell_obj, "\tBAD LOG DATA TYPE INPUT\r\n");
        return;
    }

    info.log_type = type;
    info.log_size = size;   /* only when if blackbox avaliable */

    search_out = Storage.search(Para_User, BlackBox_Storage_Name);
    if ((search_out.item_addr == 0) || \
        (Storage.update(Para_User, search_out.item.data_addr, (uint8_t *)&info, sizeof(BlackBox_LogInfo_TypeDef)) != Storage_Error_None))
    {
        shellPrint(shell_obj, "\tLog info storage failed\r\n");
        return;
    }

    shellPrint(shell_obj, "\tLog info storage saved\r\n");
    shellPrint(shell_obj, "\treboot is required\r\n");
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC) | SHELL_CMD_DISABLE_RETURN, blackbox_set, TaskBlackBox_Set_Log, set blackbox log);

static void TaskBlackBox_EnableLog(void)
{
    Shell *shell_obj = Shell_GetInstence();

    if ((shell_obj == NULL) || (p_blackbox == NULL))
        return;

    if (p_blackbox->enable && p_blackbox->enable())
    {
        shellPrint(shell_obj, "[ BalckBox ] Enable\r\r");
        Monitor.state = BlackBox_Log_Enable;
        Monitor.log_cnt = 0;
        Monitor.log_byte_size = 0;
        Queue.reset(&BlackBox_Queue);
    }
    else
        shellPrint(shell_obj, "[ BlackBox ] Enable Failed\r\n");
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC) | SHELL_CMD_DISABLE_RETURN, blackbox_enable, TaskBlackBox_EnableLog, blackbox start log);

static void TaskBlackBox_DisableLog(void)
{
    Shell *shell_obj = Shell_GetInstence();

    if ((shell_obj == NULL) || (p_blackbox == NULL))
        return;

    Monitor.state = BlackBox_Log_Disable;
    shellPrint(shell_obj, "[ BalckBox ] Disable\r\r");
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC) | SHELL_CMD_DISABLE_RETURN, blackbox_disable, TaskBlackBox_DisableLog, blackbox end log);

static BlackBox_ConvertError_List TaskBlackBox_ConvertLogData_To_Header(Shell *p_shell, BlackBox_DataHeader_TypeDef *p_header, uint8_t **p_data, uint32_t *len)
{
    if ((p_header == NULL) || \
        (p_data == NULL) || \
        (len == NULL) || \
        (*p_data == NULL))
        return BlackBox_Cnv_Para_Error;
    
    if (*len <= BLACKBOX_HEADER_SIZE)
        return BlackBox_Cnv_Size_Error;

    memcpy(p_header, *p_data, BLACKBOX_HEADER_SIZE);
    if (p_header->header != BLACKBOX_LOG_HEADER)
    {
        memset(p_header, 0, BLACKBOX_HEADER_SIZE);
        shellPrint(p_shell, "[ BlackBox ] header tag error\r\n");
        return BlackBox_Cnv_Header_Error;
    }

    switch ((uint8_t) p_header->type)
    {
        case BlackBox_Imu_Filted:
            if (*len < (BLACKBOX_HEADER_SIZE + BLACKBOX_ENDER_SIZE + sizeof(BlackBox_IMUData_TypeDef)))
                return BlackBox_Cnv_Size_Error;
            break;

        case BlackBox_Log_Alt_Att:
            if (*len < (BLACKBOX_HEADER_SIZE + BLACKBOX_ENDER_SIZE + sizeof(BlackBox_AttAltData_TypeDef)))
                return BlackBox_Cnv_Size_Error;
            break;

        case BlackBox_AngularPID_Tune:
            if (*len < (BLACKBOX_HEADER_SIZE + BLACKBOX_ENDER_SIZE + sizeof(BlackBox_AngCtlData_TypeDef)))
                return BlackBox_Cnv_Size_Error;
            break;

        default:
            if (p_shell)
                shellPrint(p_shell, "[ BlackBox ] type error\r\n");
            return BlackBox_Cnv_Type_Error;
    }

    *len -= BLACKBOX_HEADER_SIZE;
    *p_data += BLACKBOX_HEADER_SIZE;

    return BlackBox_Cnv_None_Error;
}

static BlackBox_ConvertError_List TaskBlackBox_ConvertLogData_To_Ender(Shell *p_shell, BlackBox_DataEnder_TypeDef *p_ender, uint8_t **p_data, uint32_t *len)
{
    if ((p_ender == NULL) || \
        (p_data == NULL) || \
        (*p_data == NULL) || \
        (len == NULL) || \
        (*len < BLACKBOX_ENDER_SIZE))
        return BlackBox_Cnv_Para_Error;

    memcpy(p_ender, *p_data, BLACKBOX_ENDER_SIZE);
    if (p_ender->ender != BLACKBOX_LOG_ENDER)
    {
        memset(p_ender, 0, BLACKBOX_ENDER_SIZE);
        if (p_shell)
            shellPrint(p_shell, "[ BlackBox ] ender error\r\n");
        return BlackBox_Cnv_Ender_Error;
    }

    *p_data += BLACKBOX_ENDER_SIZE;
    *len -= BLACKBOX_ENDER_SIZE;

    return BlackBox_Cnv_None_Error;
}

/* filted imu data log */
static BlackBox_ConvertError_List TaskBlackBox_ConvertLogData_To_IMU(Shell *p_shell, BlackBox_IMUData_TypeDef *p_imu, uint8_t **p_data, uint32_t *len)
{
    if ((p_imu == NULL) || \
        (p_data == NULL) || \
        (*p_data == NULL) || \
        (len == NULL) || \
        (*len < sizeof(BlackBox_IMUData_TypeDef)))
        return BlackBox_Cnv_Size_Error;

    memcpy(p_imu, *p_data, sizeof(BlackBox_IMUData_TypeDef));
    if (p_shell)
    {
        shellPrint(p_shell, "%d ", p_imu->time);
        shellPrint(p_shell, "%d ", p_imu->cyc);
        shellPrint(p_shell, "%d ", p_imu->throttle_percent);
        shellPrint(p_shell, "%f ", p_imu->org_acc[Axis_X] / p_imu->acc_scale);
        shellPrint(p_shell, "%f ", p_imu->org_acc[Axis_Y] / p_imu->acc_scale);
        shellPrint(p_shell, "%f ", p_imu->org_acc[Axis_Z] / p_imu->acc_scale);
        shellPrint(p_shell, "%f ", p_imu->flt_acc[Axis_X] / p_imu->acc_scale);
        shellPrint(p_shell, "%f ", p_imu->flt_acc[Axis_Y] / p_imu->acc_scale);
        shellPrint(p_shell, "%f ", p_imu->flt_acc[Axis_Z] / p_imu->acc_scale);
        shellPrint(p_shell, "%f ", p_imu->org_gyr[Axis_X] / p_imu->gyr_scale);
        shellPrint(p_shell, "%f ", p_imu->org_gyr[Axis_Y] / p_imu->gyr_scale);
        shellPrint(p_shell, "%f ", p_imu->org_gyr[Axis_Z] / p_imu->gyr_scale);
        shellPrint(p_shell, "%f ", p_imu->flt_gyr[Axis_X] / p_imu->gyr_scale);
        shellPrint(p_shell, "%f ", p_imu->flt_gyr[Axis_Y] / p_imu->gyr_scale);
        shellPrint(p_shell, "%f\r\n", p_imu->flt_gyr[Axis_Z] / p_imu->gyr_scale);
    }

    *p_data += sizeof(BlackBox_IMUData_TypeDef);
    *len -= sizeof(BlackBox_IMUData_TypeDef);
    return BlackBox_Cnv_None_Error;
}

/* attitude altitude log data */
static BlackBox_ConvertError_List TaskBlackBox_ConvertLogData_To_Alt_Att(Shell *p_shell, BlackBox_AttAltData_TypeDef *p_att_alt, uint8_t **p_data, uint32_t *len)
{
    if ((p_att_alt == NULL) || \
        (p_data == NULL) || \
        (*p_data == NULL) || \
        (len == NULL) || \
        (*len < sizeof(BlackBox_AttAltData_TypeDef)))
        return BlackBox_Cnv_Size_Error;

    memcpy(p_att_alt, *p_data, sizeof(BlackBox_AttAltData_TypeDef));
    if (p_shell)
    {
        shellPrint(p_shell, "%d ", p_att_alt->time);
        shellPrint(p_shell, "%f ", p_att_alt->baro);
        shellPrint(p_shell, "%f ", p_att_alt->acc[Axis_X] / p_att_alt->acc_scale);
        shellPrint(p_shell, "%f ", p_att_alt->acc[Axis_Y] / p_att_alt->acc_scale);
        shellPrint(p_shell, "%f ", p_att_alt->acc[Axis_Z] / p_att_alt->acc_scale);
        shellPrint(p_shell, "%f ", p_att_alt->gyr[Axis_X] / p_att_alt->gyr_scale);
        shellPrint(p_shell, "%f ", p_att_alt->gyr[Axis_Y] / p_att_alt->gyr_scale);
        shellPrint(p_shell, "%f ", p_att_alt->gyr[Axis_Z] / p_att_alt->gyr_scale);
        shellPrint(p_shell, "%f ", p_att_alt->pitch);
        shellPrint(p_shell, "%f ", p_att_alt->roll);
        shellPrint(p_shell, "%f ", p_att_alt->yaw);
        shellPrint(p_shell, "%f\r\n", p_att_alt->alt);
    }

    *p_data += sizeof(BlackBox_AttAltData_TypeDef);
    *len -= sizeof(BlackBox_AttAltData_TypeDef);
    return BlackBox_Cnv_None_Error;
}

/* angular control log data */
static BlackBox_ConvertError_List TaskBlackBox_ConvertLogData_To_CtlAng(Shell *p_shell, BlackBox_AngCtlData_TypeDef *p_ang_ctl, uint8_t **p_data, uint32_t *len)
{
    if ((p_ang_ctl == NULL) || \
        (p_data == NULL) || \
        (*p_data == NULL) || \
        (len == NULL) || \
        (*len < sizeof(BlackBox_AngCtlData_TypeDef)))
        return BlackBox_Cnv_Size_Error;
    
    memcpy(p_ang_ctl, *p_data, sizeof(BlackBox_AngCtlData_TypeDef));
    if (p_shell)
    {
        shellPrint(p_shell, "%d ", p_ang_ctl->time);
        shellPrint(p_shell, "%d ", p_ang_ctl->throttle_percent);
        shellPrint(p_shell, "%f ", p_ang_ctl->e_gyr[Axis_X]);
        shellPrint(p_shell, "%f ", p_ang_ctl->e_gyr[Axis_Y]);
        shellPrint(p_shell, "%f ", p_ang_ctl->e_gyr[Axis_Z]);
        shellPrint(p_shell, "%f ", p_ang_ctl->m_gyr[Axis_X]);
        shellPrint(p_shell, "%f ", p_ang_ctl->m_gyr[Axis_Y]);
        shellPrint(p_shell, "%f\r\n", p_ang_ctl->m_gyr[Axis_Z]);
    }

    *p_data += sizeof(BlackBox_AngCtlData_TypeDef);
    *len -= sizeof(BlackBox_AngCtlData_TypeDef);
    return BlackBox_Cnv_None_Error;
}

/* attitude control log data */
static BlackBox_ConvertError_List TaskBlackBox_ConvertLogData_To_CtlAtt(Shell *p_shell, BlackBox_AttCtlData_TypeDef *p_att_ctl, uint8_t **p_data, uint32_t *len)
{
    if ((p_att_ctl == NULL) || \
        (p_data == NULL) || \
        (*p_data == NULL) || \
        (len == NULL) || \
        (*len < sizeof(BlackBox_AttCtlData_TypeDef)))
        return BlackBox_Cnv_Size_Error;

    memcpy(p_att_ctl, *p_data, sizeof(BlackBox_AttCtlData_TypeDef));
    if (p_shell)
    {
        shellPrint(p_shell, "%d ", p_att_ctl->time);
        shellPrint(p_shell, "%f ", p_att_ctl->exp_pitch);
        shellPrint(p_shell, "%f ", p_att_ctl->exp_roll);
        shellPrint(p_shell, "%f ", p_att_ctl->exp_gyr_z);
        shellPrint(p_shell, "%f ", p_att_ctl->pitch);
        shellPrint(p_shell, "%f ", p_att_ctl->roll);
        shellPrint(p_shell, "%f\r\n", p_att_ctl->gyr_z);
    }

    *p_data += sizeof(BlackBox_AttCtlData_TypeDef);
    *len -= sizeof(BlackBox_AttCtlData_TypeDef);
    return BlackBox_Cnv_None_Error;
}

static const char* TaskBlackBox_Get_LogType_Str(void)
{
    switch (Monitor.log_type)
    {
        case BlackBox_Log_None:         return "Log_None";
        case BlackBox_Imu_Filted:       return "Filted_IMU";
        case BlackBox_Log_Alt_Att:      return "Alt_Att";
        case BlackBox_AngularPID_Tune:  return "AngularPID";
        case BlackBox_AttitudePID_Tune: return "AttitudePID";
        default: return "unknow log type";
    }
}

static void TaskBlackBox_GetLogType(void)
{
    Shell *shell_obj = Shell_GetInstence();
    if (shell_obj == NULL)
        return;
    
    shellPrint(shell_obj, "[ BlackBox ] log type: %s\r\n", TaskBlackBox_Get_LogType_Str());
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC) | SHELL_CMD_DISABLE_RETURN, blackbox_type, TaskBlackBox_GetLogType, blackbox log type);

static void TaskBlackBox_GetLogInfo(void)
{
    Shell *shell_obj = Shell_GetInstence();
    uint32_t log_cnt = 0;
    uint32_t log_size = 0;
    bool log_enable = false;
    uint32_t addr = 0;
    volatile BlackBox_ConvertError_List err = BlackBox_Cnv_None_Error;
    BlackBox_DataHeader_TypeDef header;
    BlackBox_DataEnder_TypeDef ender;
    BlackBox_IMUData_TypeDef log_imu;
    BlackBox_AttAltData_TypeDef log_att_alt;
    BlackBox_AngCtlData_TypeDef log_AngCtl;
    BlackBox_AttCtlData_TypeDef log_AttCtl;
    uint8_t *p_log_data = NULL;
    uint32_t uncomplete_size = 0;
    uint8_t remain_buf[128];
    uint8_t *p_remain_buf = NULL;
    uint8_t check_sum = 0;

    if ((shell_obj == NULL) || (p_blackbox == NULL))
        return;

    if (p_blackbox->get_info == NULL)
    {
        shellPrint(shell_obj, "[ BlackBox ] get info api error\r\n");
        return;
    }

    p_blackbox->get_info(Storage.read_section, &log_cnt, &log_size, &log_enable);
    if (log_enable)
    {
        shellPrint(shell_obj, "[ BlackBox ] is logging\r\n");
        return;
    }

    shellPrint(shell_obj, "[ BlackBox ] log type:               %s\r\n", TaskBlackBox_Get_LogType_Str());
    shellPrint(shell_obj, "[ BlackBox ] task log success count: %d\r\n", Monitor.log_cnt);
    shellPrint(shell_obj, "[ BlackBox ] service log count:      %d\r\n", log_cnt);
    shellPrint(shell_obj, "[ BlackBox ] service log size:       %d\r\n", log_size);

    /* dispaly data */
    if ((p_blackbox->read == NULL) || (Monitor.p_log_buf == NULL))
    {
        shellPrint(shell_obj, "[ BlackBox ] read api error\r\n");
        return;
    }

    memset(&header, 0, BLACKBOX_HEADER_SIZE);
    for (uint16_t i = 0; i < log_cnt; i++)
    {
        if (p_blackbox->read(Storage.read_section, addr, Monitor.p_log_buf, Monitor.log_unit))
        {
            log_size = Monitor.log_unit;
            p_log_data = Monitor.p_log_buf;
            addr += Monitor.log_unit;

            /* convert data */
            while (log_size)
            {
                if (uncomplete_size)
                {
                    p_remain_buf = remain_buf;

                    if (uncomplete_size <= BLACKBOX_HEADER_SIZE)
                    {
                        /* get header first */
                        if (uncomplete_size < BLACKBOX_HEADER_SIZE)
                        {
                            memcpy(p_remain_buf + uncomplete_size, p_log_data, BLACKBOX_HEADER_SIZE - uncomplete_size);
                            p_log_data += (BLACKBOX_HEADER_SIZE - uncomplete_size);
                            log_size -= (BLACKBOX_HEADER_SIZE - uncomplete_size);
                            uncomplete_size = BLACKBOX_HEADER_SIZE;
                        }

                        err = TaskBlackBox_ConvertLogData_To_Header(shell_obj, &header, &p_remain_buf, &uncomplete_size);
                        if (err == BlackBox_Cnv_Size_Error)
                        {
                            err = BlackBox_Cnv_None_Error;
                            uncomplete_size = 0;
                        }
                    }
                    else if (uncomplete_size > BLACKBOX_HEADER_SIZE)
                    {
                        err = TaskBlackBox_ConvertLogData_To_Header(shell_obj, &header, &p_remain_buf, &uncomplete_size);
                        if ((err != BlackBox_Cnv_None_Error) && (err != BlackBox_Cnv_Size_Error))
                        {
                            /* convert bug */
                            shellPrint(shell_obj, "[ BlackBox ] uncomplete decode error\r\n");
                            return;
                        }
                        else if (err == BlackBox_Cnv_Size_Error)
                            err = BlackBox_Cnv_None_Error;

                        uncomplete_size -= BLACKBOX_HEADER_SIZE;
                        p_remain_buf += BLACKBOX_HEADER_SIZE;
                    }

                    if (uncomplete_size)
                    {
                        uint32_t decode_size = 0;
                        switch ((uint8_t)header.type)
                        {
                            case BlackBox_Imu_Filted:
                                if (uncomplete_size <= sizeof(BlackBox_IMUData_TypeDef))
                                {
                                    memcpy(p_remain_buf + uncomplete_size, p_log_data, sizeof(BlackBox_IMUData_TypeDef) - uncomplete_size);
                                    p_log_data += sizeof(BlackBox_IMUData_TypeDef) - uncomplete_size;
                                    log_size -= sizeof(BlackBox_IMUData_TypeDef) - uncomplete_size;
                                    uncomplete_size = 0;
                                }
                                else
                                {
                                    memmove(p_remain_buf, p_remain_buf + uncomplete_size, uncomplete_size - sizeof(BlackBox_IMUData_TypeDef));
                                    uncomplete_size -= sizeof(BlackBox_IMUData_TypeDef);
                                }

                                decode_size = sizeof(BlackBox_IMUData_TypeDef);
                                TaskBlackBox_ConvertLogData_To_IMU(shell_obj, &log_imu, &p_remain_buf, &decode_size);
                                break;

                            case BlackBox_Log_Alt_Att:
                                if (uncomplete_size <= sizeof(BlackBox_AttAltData_TypeDef))
                                {
                                    memcpy(p_remain_buf + uncomplete_size, p_log_data, sizeof(BlackBox_AttAltData_TypeDef) - uncomplete_size);
                                    p_log_data += sizeof(BlackBox_AttAltData_TypeDef) - uncomplete_size;
                                    log_size -= sizeof(BlackBox_AttAltData_TypeDef) - uncomplete_size;
                                    uncomplete_size = 0;
                                }
                                else
                                {
                                    memmove(p_remain_buf, p_remain_buf + uncomplete_size, uncomplete_size - sizeof(BlackBox_AttAltData_TypeDef));
                                    uncomplete_size -= sizeof(BlackBox_AttAltData_TypeDef);
                                }

                                decode_size = sizeof(BlackBox_AttAltData_TypeDef);
                                TaskBlackBox_ConvertLogData_To_Alt_Att(shell_obj, &log_att_alt, &p_remain_buf, &decode_size);
                                break;

                            case BlackBox_AngularPID_Tune:
                                if (uncomplete_size <= sizeof(BlackBox_AngCtlData_TypeDef))
                                {
                                    memcpy(p_remain_buf + uncomplete_size, p_log_data, sizeof(BlackBox_AngCtlData_TypeDef) - uncomplete_size);
                                    p_log_data += sizeof(BlackBox_AngCtlData_TypeDef) - uncomplete_size;
                                    log_size -= sizeof(BlackBox_AngCtlData_TypeDef) - uncomplete_size;
                                    uncomplete_size = 0;
                                }
                                else
                                {
                                    memmove(p_remain_buf, p_remain_buf + uncomplete_size, uncomplete_size - sizeof(BlackBox_AngCtlData_TypeDef));
                                    uncomplete_size -= sizeof(BlackBox_AngCtlData_TypeDef);
                                }

                                decode_size = sizeof(BlackBox_AngCtlData_TypeDef);
                                TaskBlackBox_ConvertLogData_To_CtlAng(shell_obj, &log_AngCtl, &p_remain_buf, &decode_size);
                                break;

                            case BlackBox_AttitudePID_Tune:
                                if (uncomplete_size <= sizeof(BlackBox_AttCtlData_TypeDef))
                                {
                                    memcpy(p_remain_buf + uncomplete_size, p_log_data, sizeof(BlackBox_AttCtlData_TypeDef) - uncomplete_size);
                                    p_log_data += sizeof(BlackBox_AttCtlData_TypeDef) - uncomplete_size;
                                    log_size -= sizeof(BlackBox_AttCtlData_TypeDef) - uncomplete_size;
                                    uncomplete_size = 0;
                                }
                                else
                                {
                                    memmove(p_remain_buf, p_remain_buf + uncomplete_size, uncomplete_size - sizeof(BlackBox_AttCtlData_TypeDef));
                                    uncomplete_size -= sizeof(BlackBox_AttCtlData_TypeDef);
                                }

                                decode_size = sizeof(BlackBox_AttCtlData_TypeDef);
                                TaskBlackBox_ConvertLogData_To_CtlAtt(shell_obj, &log_AttCtl, &p_remain_buf, &decode_size);
                                break;

                            default: return;
                        }

                        /* get ender */
                        if (uncomplete_size)
                        {
                            memcpy(remain_buf + uncomplete_size, p_log_data, BLACKBOX_ENDER_SIZE - uncomplete_size);
                            decode_size = BLACKBOX_ENDER_SIZE;
                            p_log_data += BLACKBOX_ENDER_SIZE - uncomplete_size;
                            log_size -= BLACKBOX_ENDER_SIZE - uncomplete_size;
                            TaskBlackBox_ConvertLogData_To_Ender(shell_obj, &ender, &p_remain_buf, &decode_size);
                        }
                        else
                            TaskBlackBox_ConvertLogData_To_Ender(shell_obj, &ender, &p_log_data, &log_size);

                        /* get next header */
                        err = TaskBlackBox_ConvertLogData_To_Header(shell_obj, &header, &p_log_data, &log_size);
                    }
                }
                else
                    err = TaskBlackBox_ConvertLogData_To_Header(shell_obj, &header, &p_log_data, &log_size);

                if (err == BlackBox_Cnv_None_Error)
                {
                    uncomplete_size = 0;
                    switch ((uint8_t)header.type)
                    {
                        case BlackBox_Imu_Filted:
                            err = TaskBlackBox_ConvertLogData_To_IMU(shell_obj, &log_imu, &p_log_data, &log_size);
                            check_sum = TaskBlackBox_Get_CheckSum((uint8_t *)&log_imu, sizeof(BlackBox_IMUData_TypeDef));
                            break;

                        case BlackBox_Log_Alt_Att:
                            err = TaskBlackBox_ConvertLogData_To_Alt_Att(shell_obj, &log_att_alt, &p_log_data, &log_size);
                            check_sum = TaskBlackBox_Get_CheckSum((uint8_t *)&log_att_alt, sizeof(BlackBox_AttAltData_TypeDef));
                            break;

                        case BlackBox_AngularPID_Tune:
                            err = TaskBlackBox_ConvertLogData_To_CtlAng(shell_obj, &log_AngCtl, &p_log_data, &log_size);
                            check_sum = TaskBlackBox_Get_CheckSum((uint8_t *)&log_AngCtl, sizeof(BlackBox_AngCtlData_TypeDef));
                            break;

                        case BlackBox_AttitudePID_Tune:
                            err = TaskBlackBox_ConvertLogData_To_CtlAtt(shell_obj, &log_AttCtl, &p_log_data, &log_size);
                            check_sum = TaskBlackBox_Get_CheckSum((uint8_t *)&log_AttCtl, sizeof(BlackBox_AttCtlData_TypeDef));
                            break;

                        default: break;
                    }

                    if (err == BlackBox_Cnv_Size_Error)
                    {
                        memset(remain_buf, 0, sizeof(remain_buf));
                        uncomplete_size = log_size;
                        memcpy(remain_buf, p_log_data, uncomplete_size);
                        log_size = 0;
                        break;
                    }
                    else
                    {
                        TaskBlackBox_ConvertLogData_To_Ender(shell_obj, &ender, &p_log_data, &log_size);

                        if (check_sum != ender.check_sum)
                        {
                            shellPrint(shell_obj, "[ BlackBox ] CheckSum error\r\n");
                            shellPrint(shell_obj, "[ BlackBox ] addr: %d remian: %d\r\n", addr, log_size);
                        }
                    }
                }
                else if (err == BlackBox_Cnv_Size_Error)
                {
                    memset(remain_buf, 0, sizeof(remain_buf));
                    uncomplete_size = log_size;
                    memcpy(remain_buf, p_log_data, uncomplete_size);
                    log_size = 0;
                }
                else
                    break;
            }
        }
        else
        {
            shellPrint(shell_obj, "[ BlackBox ] Data read failed\r\n");
            shellPrint(shell_obj, "[ BlackBox ] read address %d\r\n", addr);
            break;
        }
    }
    shellPrint(shell_obj, "[ BlackBox ] Log End\r\n");
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC) | SHELL_CMD_DISABLE_RETURN, blackbox_info, TaskBlackBox_GetLogInfo, blackbox log info);
