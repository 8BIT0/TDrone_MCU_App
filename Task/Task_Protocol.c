#include "Task_Protocol.h"
#include "Srv_OsCommon.h"
#include "HW_Def.h"
#include "Srv_ComProto.h"
#include "Srv_DataHub.h"
#include "DataPipe.h"
#include "Storage.h"
#include "debug_util.h"
#include "pid.h"

#define PROTO_STREAM_BUF_SIZE (1024 + 128)
#define SLIENT_TIMEOUT 200              /* unit: ms 200ms */
#define TUNNING_TIMEOUT 2000            /* unit: ms 1S */

#define CLI_FUNC_BUF_SIZE 512
#define RADIO_BUFF_SIZE PROTO_STREAM_BUF_SIZE

#if (RADIO_UART_NUM > 0)
static uint8_t RadioRxBuff[RADIO_UART_NUM][RADIO_BUFF_SIZE];

static BspUARTObj_TypeDef Radio_Port1_UartObj = {
    .instance = RADIO_PORT,
    .baudrate = RADIO_PORT_BAUD,
    .tx_io = {
        .init_state = RADIO_TX_PIN_INIT_STATE,
        .pin = RADIO_TX_PIN,
        .port = RADIO_TX_PORT,
        .alternate = RADIO_TX_PIN_ALT,
    }, 
    .rx_io = {
        .init_state = RADIO_RX_PIN_INIT_STATE,
        .pin = RADIO_RX_PIN,
        .port = RADIO_RX_PORT,
        .alternate = RADIO_RX_PIN_ALT,
    }, 
    .pin_swap = false,
    .rx_dma = RADIO_RX_DMA,
    .rx_stream = RADIO_RX_DMA_STREAM,
    .tx_dma = RADIO_TX_DMA,
    .tx_stream = RADIO_TX_DMA_STREAM,
    .rx_buf = RadioRxBuff[0],
    .rx_size = RADIO_BUFF_SIZE,
};

static FrameCTL_UartPortMonitor_TypeDef Radio_UartPort_List[RADIO_UART_NUM] = {
    [0] = {.init_state = false,
           .Obj = &Radio_Port1_UartObj},
};
#endif

/* MAVLink message List */
SrvComProto_MsgInfo_TypeDef TaskProto_MAV_RawIMU;

SrvComProto_MsgInfo_TypeDef RadioProto_MAV_RawIMU;

static SrvComProto_MsgObj_TypeDef *InUsePort_MavMsgInput_Obj;
static SrvComProto_MsgObj_TypeDef DefaultPort_MavMsgInput_Obj;
static SrvComProto_MsgObj_TypeDef RadioPort_MavMsgInput_Obj;

static bool FrameCTL_MavProto_Enable = false;
static FrameCTL_PortMonitor_TypeDef PortMonitor = {.init = false};
static uint32_t FrameCTL_Period = 0;
static __attribute__((section(".Perph_Section"))) uint8_t MavShareBuf[1024];
static __attribute__((section(".Perph_Section"))) uint8_t CLIRxBuf[CLI_FUNC_BUF_SIZE];
static uint8_t CLIProcBuf[CLI_FUNC_BUF_SIZE];
static uint8_t Uart_RxBuf_Tmp[PROTO_STREAM_BUF_SIZE];
static uint8_t USB_RxBuf_Tmp[PROTO_STREAM_BUF_SIZE];
static uint32_t Radio_Addr = 0;
static uint32_t USB_VCP_Addr = 0;
DataPipe_CreateDataObj(bool, VCP_Attach_State);

static SrvComProto_Stream_TypeDef UartRx_Stream = {
    .p_buf = Uart_RxBuf_Tmp,
    .size = 0,
    .max_size = sizeof(Uart_RxBuf_Tmp),
};

static SrvComProto_Stream_TypeDef USBRx_Stream = {
    .p_buf = USB_RxBuf_Tmp,
    .size = 0,
    .max_size = sizeof(USB_RxBuf_Tmp),
};

static SrvComProto_Stream_TypeDef MavStream = {
    .p_buf = MavShareBuf,
    .size = 0,
    .max_size = sizeof(MavShareBuf),
};

static SrvComProto_Stream_TypeDef CLI_RX_Stream = {
    .p_buf = CLIRxBuf,
    .size = 0,
    .max_size = sizeof(CLIRxBuf),
};

static SrvComProto_Stream_TypeDef CLI_Proc_Stream = {
    .p_buf = CLIProcBuf,
    .size = 0,
    .max_size = sizeof(CLIProcBuf),
};

static FrameCTL_CLIMonitor_TypeDef CLI_Monitor = {
    .port_addr = 0,
    .p_rx_stream = &CLI_RX_Stream,
    .p_proc_stream = &CLI_Proc_Stream,
    .slient_type = Slient_Disable,
    .slient_timeout = 0,
};

static FrameCTL_UpgradeMonitor_TypeDef Upgrade_Monitor = {
    .is_enable = false,
    .flash_enable = false,
};

/* upgrade section */

/* frame section */
static void TaskFrameCTL_PortFrameOut_Process(void);
static void TaskFrameCTL_MavMsg_Trans(FrameCTL_Monitor_TypeDef *Obj, uint8_t *p_data, uint16_t size);

/* default vcp port section */
static void TaskFrameCTL_DefaultPort_Init(FrameCTL_PortMonitor_TypeDef *monitor);
static void TaskFrameCTL_RadioPort_Init(FrameCTL_PortMonitor_TypeDef *monitor);
static bool TaskFrameCTL_MAV_Msg_Init(void);
static void TaskFrameCTL_Port_Rx_Callback(uint32_t RecObj_addr, uint8_t *p_data, uint16_t size);
static void TaskFrameCTL_Port_TxCplt_Callback(uint32_t RecObj_addr, uint8_t *p_data, uint32_t *size);
static void TaskFrameCTL_USB_VCP_Connect_Callback(uint32_t Obj_addr, uint32_t *time_stamp);
static uint32_t TaskFrameCTL_Set_RadioPort(FrameCTL_PortType_List port_type, uint16_t index);
static bool TaskFrameCTL_Port_Tx(uint32_t obj_addr, uint8_t *p_data, uint16_t size);
static bool TaskFrameCTL_DefaultPort_Trans(uint8_t *p_data, uint16_t size);
static void TaskFrameCTL_ConfigureStateCheck(void);
static void TaskFrameCTL_CLI_Proc(void);
static int TaskFrameCTL_CLI_Trans(const uint8_t *p_data, uint16_t size);

void TaskFrameCTL_Init(uint32_t period)
{
    FrameCTL_Period = FrameCTL_MAX_Period;

    /* USB VCP as defaut port to tune parameter and frame porotcol */
    memset(&PortMonitor, 0, sizeof(PortMonitor));
    memset(&DefaultPort_MavMsgInput_Obj, 0, sizeof(SrvComProto_MsgObj_TypeDef));
    memset(&RadioPort_MavMsgInput_Obj, 0, sizeof(SrvComProto_MsgObj_TypeDef));

    TaskFrameCTL_DefaultPort_Init(&PortMonitor);
    TaskFrameCTL_RadioPort_Init(&PortMonitor);
    Radio_Addr = TaskFrameCTL_Set_RadioPort(Port_Uart, 0);

    PortMonitor.init = true;
    
    /* init radio protocol*/
    FrameCTL_MavProto_Enable = TaskFrameCTL_MAV_Msg_Init();

    if(period && (period <= FrameCTL_MAX_Period))
    {
        FrameCTL_Period = period;
    }

    /* Shell Init */
    Shell_Init(TaskFrameCTL_CLI_Trans, CLI_Monitor.p_proc_stream->p_buf, CLI_Monitor.p_proc_stream->max_size);

    memset(&VCP_Connect_smp_DataPipe, 0, sizeof(VCP_Connect_smp_DataPipe));
    memset(DataPipe_DataObjAddr(VCP_Attach_State), 0, sizeof(DataPipe_DataObj(VCP_Attach_State)));

    VCP_Connect_smp_DataPipe.data_addr = (uint32_t)DataPipe_DataObjAddr(VCP_Attach_State);
    VCP_Connect_smp_DataPipe.data_size = sizeof(DataPipe_DataObj(VCP_Attach_State));

    SrvOsCommon.delay_ms(50);
    DataPipe_Enable(&VCP_Connect_smp_DataPipe);
}

void TaskFrameCTL_Core(void const *arg)
{
    uint32_t per_time = SrvOsCommon.get_os_ms();
    bool cli_state = false;

    while(1)
    {
        TaskFrameCTL_ConfigureStateCheck();
        SrvDataHub.get_cli_state(&cli_state);

        if (!Upgrade_Monitor.is_enable)
        {
            if (!cli_state)
            {
                /* frame protocol process */
                TaskFrameCTL_PortFrameOut_Process();
            }
            else
                /* command line process */
                TaskFrameCTL_CLI_Proc();
        }
        else
        {
            /* upgrade mode process */
        }

        SrvOsCommon.precise_delay(&per_time, FrameCTL_Period);
    }
}

/*************************** ByPass Mode is still in developping *********************************/
/************************************** radio section ********************************************/
static void TaskFrameCTL_DefaultPort_Init(FrameCTL_PortMonitor_TypeDef *monitor)
{
    if(monitor)
    {
        if(BspUSB_VCP.init((uint32_t)&(monitor->VCP_Port.RecObj)) != BspUSB_Error_None)
        {
            /* init default port VCP first */
            monitor->VCP_Port.init_state = false;
            return;
        }
        else
            monitor->VCP_Port.init_state = true;

        /* create USB VCP Tx semaphore */
        osSemaphoreDef(DefaultPort_Tx);
        monitor->VCP_Port.p_tx_semphr = osSemaphoreCreate(osSemaphore(DefaultPort_Tx), 1);

        if(monitor->VCP_Port.p_tx_semphr == NULL)
        {
            monitor->VCP_Port.init_state = false;
            return;
        }

        BspUSB_VCP.set_tx_cpl_callback(TaskFrameCTL_Port_TxCplt_Callback);
        BspUSB_VCP.set_rx_callback(TaskFrameCTL_Port_Rx_Callback);
        BspUSB_VCP.set_connect_callback(TaskFrameCTL_USB_VCP_Connect_Callback);

        monitor->VCP_Port.RecObj.PortObj_addr = (uint32_t)&(monitor->VCP_Port);
        monitor->VCP_Port.RecObj.type = Port_USB;

        USB_VCP_Addr = (uint32_t)&(monitor->VCP_Port);
    }
}

static bool TaskFrameCTL_DefaultPort_Trans(uint8_t *p_data, uint16_t size)
{
    bool state = false;

    /* when attach to host device then send data */
    if (PortMonitor.VCP_Port.init_state && \
        PortMonitor.vcp_connect_state && \
        PortMonitor.VCP_Port.p_tx_semphr && \
        p_data && size)
    {
        state = true;
        osSemaphoreWait(PortMonitor.VCP_Port.p_tx_semphr, 0);
        BspUSB_VCP.send(p_data, size);
        if (osSemaphoreWait(PortMonitor.VCP_Port.p_tx_semphr, FrameCTL_Port_Tx_TimeOut) != osOK)
            state = false;
    }

    return state;
}

/************************************** radio port section *************************/
static void TaskFrameCTL_RadioPort_Init(FrameCTL_PortMonitor_TypeDef *monitor)
{
    if(monitor)
    {
#if (RADIO_UART_NUM > 0)
        monitor->uart_port_num = RADIO_UART_NUM;
        monitor->Uart_Port = Radio_UartPort_List;
        
        for(uint8_t i = 0; i < monitor->uart_port_num; i++)
        {
            /* create port obj element */
            monitor->Uart_Port[i].Obj->hdl = SrvOsCommon.malloc(UART_HandleType_Size);
            if(monitor->Uart_Port[i].Obj->hdl == NULL)
            {
                SrvOsCommon.free(monitor->Uart_Port[i].Obj->hdl);
                return;
            }

            monitor->Uart_Port[i].Obj->rx_dma_hdl = SrvOsCommon.malloc(UART_DMA_Handle_Size);
            if(monitor->Uart_Port[i].Obj->rx_dma_hdl == NULL)
            {
                SrvOsCommon.free(monitor->Uart_Port[i].Obj->rx_dma_hdl);
                SrvOsCommon.free(monitor->Uart_Port[i].Obj->hdl);
                return;
            }

            monitor->Uart_Port[i].Obj->tx_dma_hdl = SrvOsCommon.malloc(UART_DMA_Handle_Size);
            if(monitor->Uart_Port[i].Obj->tx_dma_hdl == NULL)
            {
                SrvOsCommon.free(monitor->Uart_Port[i].Obj->rx_dma_hdl);
                SrvOsCommon.free(monitor->Uart_Port[i].Obj->tx_dma_hdl);
                SrvOsCommon.free(monitor->Uart_Port[i].Obj->hdl);
                return;
            }

            if(BspUart.init(monitor->Uart_Port[i].Obj))
            {
                monitor->Uart_Port[i].init_state = true;
                memset(&monitor->Uart_Port[i].RecObj, 0, sizeof(FrameCTL_PortProtoObj_TypeDef));
                memset(&monitor->Uart_Port[i].ByPass_Mode, 0, sizeof(Port_Bypass_TypeDef));
                
                monitor->Uart_Port[i].RecObj.type = Port_Uart;
                monitor->Uart_Port[i].RecObj.port_index = i;
                
                monitor->Uart_Port[i].Obj->cust_data_addr = (uint32_t)&(monitor->Uart_Port[i].RecObj);
            
                /* create semaphore for send */
                osSemaphoreDef(Uart_Port_Tmp);
                monitor->Uart_Port[i].p_tx_semphr = osSemaphoreCreate(osSemaphore(Uart_Port_Tmp), 1);

                if(monitor->Uart_Port[i].p_tx_semphr)
                {
                    /* set callback */
                    BspUart.set_rx_callback(monitor->Uart_Port[i].Obj, (BspUART_Callback)TaskFrameCTL_Port_Rx_Callback);
                    BspUart.set_tx_callback(monitor->Uart_Port[i].Obj, (BspUART_Callback)TaskFrameCTL_Port_TxCplt_Callback);

                    monitor->Uart_Port[i].RecObj.PortObj_addr = (uint32_t)&(monitor->Uart_Port[i]);
                    monitor->Uart_Port[i].init_state = true;
                }
                else
                {
                    monitor->Uart_Port[i].init_state = false;
                    monitor->uart_port_num --;
                }
            }
            else
            {
                monitor->Uart_Port[i].init_state = false;
                monitor->uart_port_num --;
            }
        }
#else
        monitor->uart_port_num = 0;
#endif
    }
}

static uint32_t TaskFrameCTL_Set_RadioPort(FrameCTL_PortType_List port_type, uint16_t index)
{
    uint32_t port_hdl = 0;

    switch((uint8_t) port_type)
    {
        case Port_Uart:
            if((index < PortMonitor.uart_port_num) && PortMonitor.Uart_Port[index].init_state)
            {
                port_hdl = (uint32_t)&(PortMonitor.Uart_Port[index]);
            }
            break;

        default: break;
    }

    return port_hdl;
}

/************************************** receive process callback section *************************/
static bool TaskFrameCTL_Port_Tx(uint32_t obj_addr, uint8_t *p_data, uint16_t size)
{
    bool state = false;

    FrameCTL_UartPortMonitor_TypeDef *p_UartPort = NULL;

    if(obj_addr && p_data && size)
    {
        state = true;
        p_UartPort = (FrameCTL_UartPortMonitor_TypeDef *)obj_addr;

        /* when FC attach to some host usb device then send nothing through the radio */
        if(p_UartPort->init_state && p_UartPort->Obj && p_UartPort->p_tx_semphr && !PortMonitor.vcp_connect_state)
        {
            osSemaphoreWait(p_UartPort->p_tx_semphr, 0);
            if ((!BspUart.send(p_UartPort->Obj, p_data, size)) || \
                (osSemaphoreWait(p_UartPort->p_tx_semphr, FrameCTL_Port_Tx_TimeOut) != osOK))
                state = false;
        }
        else
            state = false;
    }

    return state;
}

static void TaskFrameCTL_Port_Rx_Callback(uint32_t RecObj_addr, uint8_t *p_data, uint16_t size)
{
    SrvComProto_Msg_StreamIn_TypeDef stream_in;
    SrvComProto_Stream_TypeDef *p_stream = NULL;
    FrameCTL_PortProtoObj_TypeDef *p_RecObj = NULL;
    InUsePort_MavMsgInput_Obj = NULL;
    bool cli_state = false;

    /* use mavlink protocol tuning the flight parameter */
    if(p_data && size && RecObj_addr)
    {
        SrvDataHub.get_cli_state(&cli_state);

        p_RecObj = (FrameCTL_PortProtoObj_TypeDef *)RecObj_addr;
        p_RecObj->time_stamp = SrvOsCommon.get_os_ms();

        switch((uint8_t) p_RecObj->type)
        {
            case Port_USB:
                p_stream = &USBRx_Stream;
                InUsePort_MavMsgInput_Obj = &DefaultPort_MavMsgInput_Obj;

                if (cli_state && !Upgrade_Monitor.is_enable)
                    TaskFrameCTL_DefaultPort_Trans(p_data, size);
                break;

            case Port_Uart:
                p_stream = &UartRx_Stream;
                InUsePort_MavMsgInput_Obj = &RadioPort_MavMsgInput_Obj;
                
                if (cli_state && !Upgrade_Monitor.is_enable)
                    TaskFrameCTL_Port_Tx(p_RecObj->PortObj_addr, p_data, size);
                break;

            default:
                return;
        }

        if ((p_stream->size + size) <= p_stream->max_size)
        {
            memcpy(p_stream->p_buf + p_stream->size, p_data, size);
            p_stream->size += size;
        }
        else
        {
            memset(p_stream->p_buf, 0, p_stream->size);
            p_stream->size = 0;
            return;
        }

        /* when file adapter is enable then halt cli and mavlink frame receive */
        /* and check for upgrade data incoming */
        if (Upgrade_Monitor.is_enable && (Upgrade_Monitor.port_addr == p_RecObj->PortObj_addr))
        {
            // if (SrvUpgrade.DealRec(, p_stream->p_buf, p_stream->size))
            // {
            //     memset(p_stream->p_buf, 0, p_stream->max_size);
            //     p_stream->size = 0;
            // }

            return;
        }

        stream_in = SrvComProto.msg_decode(InUsePort_MavMsgInput_Obj, p_stream->p_buf, p_stream->size);
    
        /* noticed when drone is under disarmed state we can`t tune or send cli to drone for safety */
        if(stream_in.valid)
        {
            /* tag on recive time stamp */
            /* first come first serve */
            /* in case two different port tuning the same function or same parameter at the same time */
            /* if attach to configrator or in tunning then lock moto */
            if(!cli_state)
            {
                if (stream_in.pac_type == ComFrame_MavMsg)
                {
                    /* check mavlink message frame type */
                    /* only process mavlink message when cli is disabled */

                    /* after mavlink message processed */
                    /* deal with stream buffer */
                    if (p_stream->size > stream_in.size)
                    {
                        memmove(p_stream->p_buf, (stream_in.p_buf + stream_in.size), (p_stream->size - stream_in.size));
                    }
                    else
                        memset(p_stream->p_buf, 0, p_stream->size);

                    p_stream->size -= stream_in.size;
                }
            }
            
            if(stream_in.pac_type == ComFrame_CLI)
            {
                /* set current mode as cli mode */
                /* all command line end up with "\r\n" */
                /* push string into cli shared stream */
                if(((CLI_Monitor.port_addr == 0) || (CLI_Monitor.port_addr == p_RecObj->PortObj_addr)) && \
                   (CLI_Monitor.p_rx_stream->size + p_stream->size) <= CLI_Monitor.p_rx_stream->max_size)
                {
                    SrvDataHub.set_cli_state(true);

                    CLI_Monitor.type = p_RecObj->type;
                    CLI_Monitor.port_addr = p_RecObj->PortObj_addr;
                    memcpy(CLI_Monitor.p_rx_stream->p_buf + CLI_Monitor.p_rx_stream->size, stream_in.p_buf, stream_in.size);
                    CLI_Monitor.p_rx_stream->size += p_stream->size;

                    /* clear CLI command */
                    p_stream->size = 0;
                    memset(p_stream->p_buf, 0, p_stream->size);
                }
            }
        }
    }
}

static void TaskFrameCTL_Port_TxCplt_Callback(uint32_t Obj_addr, uint8_t *p_data, uint32_t *size)
{
    UNUSED(p_data);
    UNUSED(size);

    FrameCTL_PortProtoObj_TypeDef *p_Obj = NULL;
    FrameCTL_UartPortMonitor_TypeDef *p_UartPortObj = NULL;
    FrameCTL_VCPPortMonitor_TypeDef *p_USBPortObj = NULL;
    osSemaphoreId semID = NULL;
    uint32_t *p_rls_err_cnt = NULL;

    if(Obj_addr)
    {
        p_Obj = (FrameCTL_PortProtoObj_TypeDef *)Obj_addr;

        if(p_Obj->PortObj_addr)
        {
            switch((uint8_t) p_Obj->type)
            {
                case Port_USB:
                    p_USBPortObj = (FrameCTL_VCPPortMonitor_TypeDef *)(p_Obj->PortObj_addr);

                    if(p_USBPortObj->init_state && p_USBPortObj->p_tx_semphr)
                    {
                        semID = p_USBPortObj->p_tx_semphr;
                        p_rls_err_cnt = &p_USBPortObj->tx_semphr_rls_err;
                    }
                    break;

                case Port_Uart:
                    p_UartPortObj = (FrameCTL_UartPortMonitor_TypeDef *)(p_Obj->PortObj_addr);

                    if(p_UartPortObj->init_state && p_UartPortObj->p_tx_semphr)
                    {
                        semID = p_UartPortObj->p_tx_semphr;
                        p_rls_err_cnt = &p_UartPortObj->tx_semphr_rls_err;
                    }
                    break;

                default:
                    return;
            }
            
            if(semID && p_rls_err_cnt && (osSemaphoreRelease(semID) != osOK))
                (*p_rls_err_cnt) ++;
        }
    }
}

/************************************** USB Only Callback section ********************************************/
/*
 * use usb sof interrupt
 * as long as usb port attach to computer or other host device
 * flight controller will receive sof interrupt
 * if flight controller continue to receive this interrupt it must be attach to computer
 */
static void TaskFrameCTL_USB_VCP_Connect_Callback(uint32_t Obj_addr, uint32_t *time_stamp)
{
    FrameCTL_PortProtoObj_TypeDef *p_Obj = NULL;

    if (Obj_addr)
    {
        p_Obj = (FrameCTL_PortProtoObj_TypeDef *)Obj_addr;

        if (p_Obj->PortObj_addr && (p_Obj->type == Port_USB))
            *time_stamp = SrvOsCommon.get_os_ms();
    }
}

/************************************** upgrade protocol section ********************************************/

/************************************** frame protocol section ********************************************/
static bool TaskFrameCTL_MAV_Msg_Init(void)
{
    if (SrvComProto.get_msg_type() == SrvComProto_Type_MAV)
    {
        SrvComProto_MavPackInfo_TypeDef PckInfo;
        /* create mavlink output message object */
        memset(&PckInfo, 0, sizeof(PckInfo));
        memset(&TaskProto_MAV_RawIMU, 0, sizeof(TaskProto_MAV_RawIMU));
 
        memset(&PckInfo, 0, sizeof(PckInfo));
        memset(&RadioProto_MAV_RawIMU, 0, sizeof(TaskProto_MAV_RawIMU));
 
        // period 10Ms 100Hz
        PckInfo.system_id = MAV_SysID_Drone;
        PckInfo.component_id = MAV_CompoID_Raw_IMU;
        PckInfo.chan = 0;
        SrvComProto.mav_msg_obj_init(&TaskProto_MAV_RawIMU, PckInfo, 10);
        SrvComProto.mav_msg_enable_ctl(&TaskProto_MAV_RawIMU, true);
 
        SrvComProto.mav_msg_obj_init(&RadioProto_MAV_RawIMU, PckInfo, 10);
        SrvComProto.mav_msg_enable_ctl(&RadioProto_MAV_RawIMU, true);
        return true;
    }

    return false;
}

static void TaskFrameCTL_PortFrameOut_Process(void)
{
    FrameCTL_Monitor_TypeDef proto_monitor;
    void *proto_arg = (void *)&proto_monitor;
    bool arm_state = false;
    bool CLI_state = false;

    proto_monitor.frame_type = ComFrame_MavMsg;
    SrvDataHub.get_cli_state(&CLI_state);

    if (!FrameCTL_MavProto_Enable)
        return;

    /* when attach to configrator then disable radio port trans use default port trans mav data */
    /* check other port init state */

    /* if in tunning than halt general frame protocol */
    SrvDataHub.get_arm_state(&arm_state);

    if (((FrameCTL_UartPortMonitor_TypeDef *)Radio_Addr)->init_state)
    {
        /* Proto mavlink message through Radio */
        proto_monitor.port_type = Port_Uart;
        proto_monitor.port_addr = Radio_Addr;
        SrvComProto.mav_msg_stream(&RadioProto_MAV_RawIMU,    &MavStream, proto_arg, (ComProto_Callback)TaskFrameCTL_MavMsg_Trans);
    }

    if (PortMonitor.VCP_Port.init_state)
    {
        /* Proto mavlink message through default port */
        proto_monitor.port_type = Port_USB;
        proto_monitor.port_addr = USB_VCP_Addr;
        SrvComProto.mav_msg_stream(&TaskProto_MAV_RawIMU,       &MavStream, proto_arg, (ComProto_Callback)TaskFrameCTL_MavMsg_Trans);
    }
}

static void TaskFrameCTL_CLI_Proc(void)
{
    uint16_t rx_stream_size = 0;
    bool arm_state;
    Shell *shell_obj = Shell_GetInstence();

    /* check CLI stream */
    if(shell_obj && CLI_Monitor.p_rx_stream->p_buf && CLI_Monitor.p_rx_stream->size)
    {
        rx_stream_size = CLI_Monitor.p_rx_stream->size;

        /* if drone is under disarm state then CLI disable */
        SrvDataHub.get_arm_state(&arm_state);
        if (arm_state == DRONE_DISARM)
            return;

        for(uint16_t i = 0; i < rx_stream_size; i++)
        {
            shellHandler(shell_obj, CLI_Monitor.p_rx_stream->p_buf[i]);
            CLI_Monitor.p_rx_stream->p_buf[i] = 0;
            CLI_Monitor.p_rx_stream->size --;
        }
    }
}

static void TaskFrameCTL_MavMsg_Trans(FrameCTL_Monitor_TypeDef *Obj, uint8_t *p_data, uint16_t size)
{
    if((Obj == NULL) || (Obj->frame_type != ComFrame_MavMsg) || (Obj->port_addr == 0) || (p_data == NULL) || (size == 0))
        return;

    switch((uint8_t)(Obj->port_type))
    {
        case Port_Uart: TaskFrameCTL_Port_Tx(Obj->port_addr, p_data, size); break;
        case Port_USB: TaskFrameCTL_DefaultPort_Trans(p_data, size); break;
        default: return;
    }
}

static void TaskFrameCTL_ConfigureStateCheck(void)
{
    uint32_t cur_time = SrvOsCommon.get_os_ms();
    bool lst_vcp_state = false;

    PortMonitor.vcp_connect_state = false;

    /* check usb vcp attach state */
    if (BspUSB_VCP.check_connect)
        PortMonitor.vcp_connect_state = BspUSB_VCP.check_connect(cur_time, FrameCTL_Period);

    SrvDataHub.get_vcp_attach_state(&lst_vcp_state);

    /* if vcp connect state is change update vcp state to data hub */
    if (lst_vcp_state != PortMonitor.vcp_connect_state)
    {
        DataPipe_DataObj(VCP_Attach_State) = PortMonitor.vcp_connect_state;
        DataPipe_SendTo(&VCP_Connect_smp_DataPipe, &VCP_Connect_hub_DataPipe);
    }
}

/***************************************** Frame mavlink Receive Callback ************************************/

/***************************************** CLI Section ***********************************************/
static int TaskFrameCTL_CLI_Trans(const uint8_t *p_data, uint16_t size)
{
    if((p_data == NULL) || (size == 0))
        return 0;

    switch ((uint8_t) CLI_Monitor.type)
    {
        case Port_Uart: TaskFrameCTL_Port_Tx(CLI_Monitor.port_addr, (uint8_t *)p_data, size); break;
        case Port_USB: TaskFrameCTL_DefaultPort_Trans((uint8_t *)p_data, size); break; 
        default: break;
    }
    return 0;
}

static void TaskFermeCTL_CLI_DisableControl(void)
{
    Shell *shell_obj = Shell_GetInstence();
    
    if (shell_obj == NULL)
        return;

    shellPrint(shell_obj, "\r\n\r\n");
    
    SrvOsCommon.enter_critical();
    SrvDataHub.set_cli_state(false);
    SrvOsCommon.exit_critical();

    shellPrint(shell_obj, "[ CLI Disabled ]\r\n");
    CLI_Monitor.port_addr = 0;
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC) | SHELL_CMD_DISABLE_RETURN, CLI_Disable,  TaskFermeCTL_CLI_DisableControl, CLI Enable Control);

