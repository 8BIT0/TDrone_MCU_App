/*
 * Auther: 8_B!T0
 *
 * WARNING:
 *  SBUS Mode is none tested
 * 
 */
#include "Srv_Receiver.h"
#include "Bsp_Uart.h"
#include "Bsp_SPI.h"
#include "Bsp_GPIO.h"
#include "error_log.h"
#include "HW_Def.h"
#include "Srv_OsCommon.h"
#include "../debug/debug_util.h"

static uint8_t SrvReceiver_Buff[SRV_RECEIVER_BUFF_SIZE] __attribute__((section(".Perph_Section")));

static Error_Handler SrvReceiver_Error_Handle = 0;
static SrvReceiver_Monitor_TypeDef SrvReceiver_Monitor;

/* internal function */
static void SrvReceiver_SerialDecode_Callback(SrvReceiverObj_TypeDef *obj, uint8_t *p_data, uint16_t size);
static bool SrvReceiver_Check(SrvReceiverObj_TypeDef *receiver_obj);

/* external function */
static uint8_t *SrvReceiver_Create_UartObj(uint32_t serial_instance,
                                           uint32_t rx_dma,
                                           uint32_t rx_dma_stream,
                                           uint32_t tx_dma,
                                           uint32_t tx_dma_stream,
                                           bool swap,
                                           const BspGPIO_Obj_TypeDef tx_pin,
                                           const BspGPIO_Obj_TypeDef rx_pin);
static bool SrvReceiver_Init(SrvReceiverObj_TypeDef *obj, uint8_t *port_obj);
static SrvReceiverData_TypeDef SrvReceiver_Get_Value(SrvReceiverObj_TypeDef *receiver_obj);
static bool SrvReceiver_Get_Scope(SrvReceiverObj_TypeDef *receiver_obj, int16_t *max, int16_t *mid, int16_t *min);

SrvReceiver_TypeDef SrvReceiver = {
    .create_serial_obj = SrvReceiver_Create_UartObj,
    .init = SrvReceiver_Init,
    .get = SrvReceiver_Get_Value,
    .get_scope = SrvReceiver_Get_Scope,
};

static Error_Obj_Typedef SrvReceiver_ErrorList[] = {
    {
        .out = false,
        .log = false,
        .prc_callback = NULL,
        .code = Receiver_Obj_Error,
        .desc = "Receiver Object Input Error\r\n",
        .proc_type = Error_Proc_Ignore,
        .prc_data_stream = {
            .p_data = NULL,
            .size = 0,
        },
    },
    {
        .out = false,
        .log = false,
        .prc_callback = NULL,
        .code = Receiver_Port_Error,
        .desc = "Receiver Port Error\r\n",
        .proc_type = Error_Proc_Ignore,
        .prc_data_stream = {
            .p_data = NULL,
            .size = 0,
        },
    },
    {
        .out = false,
        .log = false,
        .prc_callback = NULL,
        .code = Receiver_Port_Init_Error,
        .desc = "Receiver Port Init Error\r\n",
        .proc_type = Error_Proc_Ignore,
        .prc_data_stream = {
            .p_data = NULL,
            .size = 0,
        },
    },
    {
        .out = false,
        .log = false,
        .prc_callback = NULL,
        .code = Receiver_FrameType_Error,
        .desc = "Receiver Frame Type Error\r\n",
        .proc_type = Error_Proc_Ignore,
        .prc_data_stream = {
            .p_data = NULL,
            .size = 0,
        },
    },
};

static uint8_t *SrvReceiver_Create_UartObj(uint32_t serial_instance,
                                           uint32_t rx_dma,
                                           uint32_t rx_dma_stream,
                                           uint32_t tx_dma,
                                           uint32_t tx_dma_stream,
                                           bool swap,
                                           const BspGPIO_Obj_TypeDef tx_pin,
                                           const BspGPIO_Obj_TypeDef rx_pin)
{
    BspUARTObj_TypeDef *Uart_Receiver_Obj = NULL;

    Uart_Receiver_Obj = (BspUARTObj_TypeDef *)SrvOsCommon.malloc(sizeof(BspUARTObj_TypeDef));

    if (Uart_Receiver_Obj == NULL)
    {
        SrvOsCommon.free(Uart_Receiver_Obj);
        return NULL;
    }

    memset(Uart_Receiver_Obj, 0, sizeof(BspUARTObj_TypeDef));

    Uart_Receiver_Obj->instance = (void *)serial_instance;
    Uart_Receiver_Obj->tx_io = tx_pin;
    Uart_Receiver_Obj->rx_io = rx_pin;
    Uart_Receiver_Obj->pin_swap = swap;
    Uart_Receiver_Obj->rx_dma = rx_dma;
    Uart_Receiver_Obj->rx_stream = rx_dma_stream;
    Uart_Receiver_Obj->tx_dma = tx_dma;
    Uart_Receiver_Obj->tx_stream = tx_dma_stream;

    Uart_Receiver_Obj->hdl = SrvOsCommon.malloc(UART_HandleType_Size);
    if(Uart_Receiver_Obj->hdl == NULL)
    {
        SrvOsCommon.free(Uart_Receiver_Obj->hdl);
        return false;
    }

    Uart_Receiver_Obj->tx_dma_hdl = SrvOsCommon.malloc(UART_DMA_Handle_Size);
    if(Uart_Receiver_Obj->tx_dma_hdl)
    {
        SrvOsCommon.free(Uart_Receiver_Obj->tx_dma_hdl);
        SrvOsCommon.free(Uart_Receiver_Obj->hdl);
        return false;
    }

    Uart_Receiver_Obj->rx_dma_hdl = SrvOsCommon.malloc(UART_DMA_Handle_Size);
    if(Uart_Receiver_Obj->rx_dma_hdl)
    {
        SrvOsCommon.free(Uart_Receiver_Obj->rx_dma_hdl);
        SrvOsCommon.free(Uart_Receiver_Obj->tx_dma_hdl);
        SrvOsCommon.free(Uart_Receiver_Obj->hdl);
        return false;
    }

    return (uint8_t *)Uart_Receiver_Obj;
}

static bool SrvReceiver_Init(SrvReceiverObj_TypeDef *obj, uint8_t *port_obj)
{
    bool data_obj_error = false;

    if ((obj == NULL) || (port_obj == NULL))
        return false;

    memset(SrvReceiver_Buff, 0, SRV_RECEIVER_BUFF_SIZE);

    /* create error log handle */
    SrvReceiver_Error_Handle = ErrorLog.create("SrvReceiver_Error");

    /* regist all error to the error tree */
    ErrorLog.registe(SrvReceiver_Error_Handle, SrvReceiver_ErrorList, sizeof(SrvReceiver_ErrorList) / sizeof(SrvReceiver_ErrorList[0]));
    
    memset(&SrvReceiver_Monitor, 0, SRVRECEIVER_SIZE);

    To_BspUart_ObjPtr(port_obj)->baudrate = CRSF_BAUDRATE;

    /* create data obj */
    obj->frame_data_obj = SrvOsCommon.malloc(sizeof(DevCRSFObj_TypeDef));
    if (obj->frame_data_obj == NULL)
    {
        data_obj_error = true;
        return false;
    }

    /* set max channel num */
    obj->channel_num = CRSF_MAX_CHANNEL;

    /* set receiver object frame object */
    obj->frame_api = &DevCRSF;

    if (!((DevCRSF_TypeDef *)(obj->frame_api))->init((DevCRSFObj_TypeDef *)(obj->frame_data_obj)))
        return false;

    obj->data.val_list = (uint16_t *)SrvOsCommon.malloc(sizeof(uint16_t) * obj->channel_num);
    if (!obj->data.val_list)
    {
        SrvOsCommon.free(obj->data.val_list);
        return false;
    }

    obj->port = (SrvReceiver_Port_TypeDef *)SrvOsCommon.malloc(sizeof(SrvReceiver_Port_TypeDef));
    if (!obj->port)
    {
        SrvOsCommon.free(obj->data.val_list);
        return false;
    }

    if (data_obj_error)
    {
        ErrorLog.trigger(SrvReceiver_Error_Handle, Receiver_Obj_Error, (uint8_t *)&SrvReceiver_Monitor, SRVRECEIVER_SIZE);
        return false;
    }

    /* set uart init parameter */
    To_BspUart_ObjPtr(port_obj)->rx_buf = SrvReceiver_Buff;
    To_BspUart_ObjPtr(port_obj)->rx_size = SRV_RECEIVER_BUFF_SIZE;
    To_BspUart_ObjPtr(port_obj)->cust_data_addr = (uint32_t)obj;

    /* set uart callback */
    To_BspUart_ObjPtr(port_obj)->RxCallback = (BspUART_Callback)SrvReceiver_SerialDecode_Callback;

    /* serial port init */
    if (!BspUart.init(port_obj))
    {
        ErrorLog.trigger(SrvReceiver_Error_Handle, Receiver_Port_Init_Error, (uint8_t *)&SrvReceiver_Monitor, SRVRECEIVER_SIZE);
        return false;
    }

    obj->port->api = (void *)&BspUart;
    obj->port->cfg = (void *)port_obj;

    return true;
}

static void SrvReceiver_SerialDecode_Callback(SrvReceiverObj_TypeDef *receiver_obj, uint8_t *p_data, uint16_t size)
{
    BspUARTObj_TypeDef *uart_obj = NULL;
    uint8_t *rx_buff_ptr = NULL;
    uint8_t decode_out = 0xFF;
    bool sig_update = false;

    if (receiver_obj && receiver_obj->frame_api && receiver_obj->port && receiver_obj->frame_data_obj)
    {
        /* do serial decode funtion */
        decode_out = ((DevCRSF_TypeDef *)(receiver_obj->frame_api))->decode(To_CRSF_Obj(receiver_obj->frame_data_obj), p_data, size);
        sig_update = true;

        switch (decode_out)
        {
            case CRSF_FRAMETYPE_LINK_STATISTICS:
                receiver_obj->data.rssi = ((DevCRSF_TypeDef *)(receiver_obj->frame_api))->get_statistics(To_CRSF_Obj(receiver_obj->frame_data_obj)).downlink_RSSI;
                receiver_obj->data.link_quality = ((DevCRSF_TypeDef *)(receiver_obj->frame_api))->get_statistics(To_CRSF_Obj(receiver_obj->frame_data_obj)).downlink_Link_quality;
                receiver_obj->data.active_antenna = ((DevCRSF_TypeDef *)(receiver_obj->frame_api))->get_statistics(To_CRSF_Obj(receiver_obj->frame_data_obj)).active_antenna;
                break;

            case CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
                // DebugPin.ctl(Debug_PC8, false);
                ((DevCRSF_TypeDef *)(receiver_obj->frame_api))->get_channel(To_CRSF_Obj(receiver_obj->frame_data_obj), receiver_obj->data.val_list);

                for (uint8_t i = 0; i < receiver_obj->channel_num; i++)
                {
                    if (receiver_obj->data.val_list[i] < CRSF_DIGITAL_CHANNEL_MIN)
                    {
                        receiver_obj->data.val_list[i] = CRSF_DIGITAL_CHANNEL_MIN;
                    }
                    else if (receiver_obj->data.val_list[i] > CRSF_DIGITAL_CHANNEL_MAX)
                    {
                        receiver_obj->data.val_list[i] = CRSF_DIGITAL_CHANNEL_MAX;
                    }
                }

                receiver_obj->data.failsafe = false;
                // DebugPin.ctl(Debug_PC8, true);

                /* set decode time stamp */
                receiver_obj->data.time_stamp = SrvOsCommon.get_os_ms();
                break;

            default:
                sig_update = false;
                return;
        }

        receiver_obj->re_update = false;            
        if (sig_update && receiver_obj->in_use)
            receiver_obj->re_update = true;

        /* clear serial obj received data */
        if (receiver_obj->port->cfg)
        {
            uart_obj = ((BspUARTObj_TypeDef *)(receiver_obj->port->cfg));

            rx_buff_ptr = uart_obj->rx_buf;

            if (rx_buff_ptr)
                memset(rx_buff_ptr, 0, size);
        }
    }
}

static bool SrvReceiver_Check(SrvReceiverObj_TypeDef *receiver_obj)
{
    volatile uint32_t sys_time = SrvOsCommon.get_os_ms();

    /* update check */
    /* update frequence less than 10hz switch into failsafe */
    if (receiver_obj->data.time_stamp && \
        ((sys_time - receiver_obj->data.time_stamp) > SRV_RECEIVER_UPDATE_TIMEOUT_MS))
    {
        volatile uint8_t test;
        test ++;
        return false;
    }

    /* range check */
    for (uint8_t i = 0; i < receiver_obj->channel_num; i++)
    {
        if ((receiver_obj->data.val_list[i] > CHANNEL_RANGE_MAX) || (receiver_obj->data.val_list[i] < CHANNEL_RANGE_MIN))
            return false;
    }

    /* check RSSI Or Statistics Data */
    /* check CRSF Statistics Data */
    // if ((receiver_obj->data.rssi < SRV_RECEIVER_MIN_RSSI) ||
    //     (receiver_obj->data.link_quality < SRV_RECEIVER_MIN_LINKQUALITY) ||
    //     (receiver_obj->data.active_antenna < SRV_RECEIVER_MIN_ANTENNA_VALUE))
    //    return false;

    return true;
}

static SrvReceiverData_TypeDef SrvReceiver_Get_Value(SrvReceiverObj_TypeDef *receiver_obj)
{
    SrvReceiverData_TypeDef receiver_data_tmp;

    memset(&receiver_data_tmp, 0, sizeof(receiver_data_tmp));
    receiver_data_tmp.failsafe = true;

    if (receiver_obj == NULL)
        return receiver_data_tmp;

re_do:
    receiver_obj->in_use = true;

    if (!SrvReceiver_Check(receiver_obj))
        return receiver_data_tmp;

    receiver_data_tmp.failsafe = false;
    receiver_data_tmp = receiver_obj->data;

    receiver_obj->in_use = false;

    if (receiver_obj->re_update)
    {
        receiver_obj->re_update = false;
        goto re_do;
    }

    return receiver_data_tmp;
}

static bool SrvReceiver_Get_Scope(SrvReceiverObj_TypeDef *receiver_obj, int16_t *max, int16_t *mid, int16_t *min)
{
    if(receiver_obj && max && mid && min)
    {
        (*max) = CRSF_DIGITAL_CHANNEL_MAX;
        (*mid) = CRSF_DIGITAL_CHANNEL_MID;
        (*min) = CRSF_DIGITAL_CHANNEL_MIN;
        return true;
    }

    return false;
}

/*************************************************************** Error Process Tree Callback *******************************************************************************/
// static void SrvReceiver_Obj_Error(int16_t code, uint8_t *p_arg, uint16_t size)
// {
// }

// static void SrvReceiver_FrameType_Error(int16_t code, uint8_t *p_arg, uint16_t size)
// {
// }

// static void SrvReceiver_Port_Error(int16_t code, uint8_t *p_arg, uint16_t size)
// {
// }

// static void SrvReceiver_Port_Init_Error(int16_t code, uint8_t *p_arg, uint16_t size)
// {
// }

/*************************************************************** Directly Error Process *******************************************************************************/
// static void SrvReceiver_UpdateTimeOut_Error()
// {
// }

// static void SrvReceiver_ValueOverRange_Error()
// {
// }

// static void SrvReceiver_ValueStepBump_Error()
// {
// }

// static void SrvReceiver_NoneSignal_Error()
// {
// }
