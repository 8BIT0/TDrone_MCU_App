#include "Srv_Flow.h"
#include "Srv_DataHub.h"
#include "Dev_Flow_3901U.h"
#include "Bsp_Uart.h"
#include "HW_Def.h"
#include "../FCHW_Config.h"
#include "../common/util.h"
#include <math.h>

#define SrvFlow_TaskPeriod_Base 50  /* 50Hz */

/* internal vriable */
static SrvFlowObj_TypeDef FlowMonitor = {
    .init = false,
    .api = NULL,
    .obj = NULL,
    .time_stamp = 0,
    .lst_time_stamp = 0,
    .flt_x_i = 0.0f,
    .flt_y_i = 0.0f,
    .att_x_bias = 0.0f,
    .att_y_bias = 0.0f,
    .lst_out_x = 0.0f,
    .lst_out_y = 0.0f,
    .out_x = 0.0f,
    .out_y = 0.0f,
    .x_v = 0.0f,
    .y_v = 0.0f,
};

/* internal function */
static bool SrvFlow_3901U_PortInit(BspUART_Port_List port, void *p_cus_data);
static void SrvFlow_3901U_RawComput(uint32_t time_stamp, int16_t raw_x, int16_t raw_y);

/* external function */
static bool SrvFlow_Init(SrvFlow_SensorType_List type);
static bool SrvFlow_Get_Data(SrvFlowData_TypeDef *p_data);
static bool SrvFlow_Sample(SrvFlow_SensorType_List type);

SrvFlow_TypeDef SrvFlow = {
    .init =  SrvFlow_Init,
    .get_data = SrvFlow_Get_Data,
    .sample = SrvFlow_Sample,
};

static bool SrvFlow_Init(SrvFlow_SensorType_List type)
{
    void *p_obj = NULL;

    FlowMonitor.type = type;
    switch ((uint8_t)FlowMonitor.type)
    {
        case Flow_3901U:    /* Uart port */
            /* use uart6 connect with flow sensor */
            p_obj = SrvOsCommon.malloc(DevFlow3901U_Size);
            if (p_obj == NULL)
                return false;

            To_DevFlow3901U_Obj(p_obj)->get_sys_time = SrvOsCommon.get_os_ms;
            FlowMonitor.obj = p_obj;
            FlowMonitor.api = &DevFlow3901;

            if (DevFlow3901.init(To_DevFlow3901U_Obj(p_obj)) == 0)
                return false;
            
            FlowMonitor.init = true;
            
            return true;

        default: return false;
    }

    return false;
}

static bool SrvFlow_Sample(SrvFlow_SensorType_List type)
{
    return false;
}

static bool SrvFlow_Get_Data(SrvFlowData_TypeDef *p_data)
{
    uint32_t time_stamp = 0;
    int16_t raw_x = 0;
    int16_t raw_y = 0;

    if (p_data)
    {
        switch ((uint8_t)FlowMonitor.type)
        {
            case Flow_3901U:
                To_DevFlow3901U_Api(FlowMonitor.api)->get(To_DevFlow3901U_Obj(FlowMonitor.obj), \
                                    &time_stamp, &raw_x, &raw_y, NULL);
                SrvFlow_3901U_RawComput(time_stamp, raw_x, raw_y);
                break;

            default: return false;
        }
    }

    return false;
}

/************************************************************ distance process impliment **********************************************************/
static bool SrvFlow_3901U_PortInit(BspUART_Port_List port, void *p_cus_data)
{
    void *port_obj = NULL;
    void *port_instance = NULL;

    port_obj = SrvOsCommon.malloc(BspUartObj_Size);
    if (port_obj == NULL)
        return false;

    To_BspUart_Obj(port_obj)->rx_size = FLOW_TEMP_BUFF_SIZE;
    To_BspUart_Obj(port_obj)->baudrate = FLOW_3901U_BAUDRATE;
    To_BspUart_Obj(port_obj)->cust_data_addr = p_cus_data;
    To_BspUart_Obj(port_obj)->irq_type = BspUart_IRQ_Type_Idle;
    To_BspUart_Obj(port_obj)->instance = port_instance;
    To_BspUart_Obj(port_obj)->pin_swap = false;
    // To_BspUart_Obj(port_obj)->rx_io = ;
    // To_BspUart_Obj(port_obj)->tx_io = ;
    // To_BspUart_Obj(port_obj)->rx_dma = FLOW_RX_DMA;
    // To_BspUart_Obj(port_obj)->rx_stream = FLOW_RX_DMA_STREAM;
    To_BspUart_Obj(port_obj)->tx_dma = Bsp_DMA_None;
    To_BspUart_Obj(port_obj)->tx_stream = Bsp_DMA_Stream_None;
    To_BspUart_Obj(port_obj)->rx_buf = SrvOsCommon.malloc(To_BspUart_Obj(port_obj)->rx_size);
    if (To_BspUart_Obj(port_obj)->rx_buf == NULL)
        return false;

    BspUart.init(port_obj);

    /* set port parse callback */
    BspUart.set_rx_callback(To_BspUart_Obj(port_obj), (BspUART_Callback *)DevFlow3901.recv);

    /* set tx pin as yaw mode control pin */

    return true;
}

/* comput raw data */
static void SrvFlow_3901U_RawComput(uint32_t time_stamp, int16_t raw_x, int16_t raw_y)
{
    uint32_t time = 0;
    float pitch = 0.0f;
    float roll = 0.0f;
    float yaw = 0.0f;
    float pos_x_diff = 0.0f;
    float pos_y_diff = 0.0f;
    float dt = 0.0f;
    float cpi = 0.0f;
    float alt = 0.0f;

    SrvDataHub.get_baro_altitude(NULL, NULL, &alt, NULL, NULL, NULL);
    SrvDataHub.get_attitude(&time, &pitch, &roll, &yaw, NULL, NULL, NULL, NULL, NULL);
    pitch = DEG_2_RAD(pitch);
    roll = DEG_2_RAD(roll);
    yaw = DEG_2_RAD(yaw);

    FlowMonitor.time_stamp = time_stamp;

    /* simple low pass filter */
    FlowMonitor.flt_x_i += (raw_x - FlowMonitor.flt_x_i) * FLOW_3901U_POSFILT_GAIN;
    FlowMonitor.flt_y_i += (raw_y - FlowMonitor.flt_y_i) * FLOW_3901U_POSFILT_GAIN;

    /* get attitude offset */
    FlowMonitor.att_x_bias = (FLOW_3901U_ATTOFFSET_GAIN * tan(roll)  - FlowMonitor.att_x_bias) * FLOW_3901U_ATTFILT_GAIN;
    FlowMonitor.att_y_bias = (FLOW_3901U_ATTOFFSET_GAIN * tan(pitch) - FlowMonitor.att_y_bias) * FLOW_3901U_ATTFILT_GAIN;

    FlowMonitor.out_x = FlowMonitor.flt_x_i - FlowMonitor.att_x_bias;
    FlowMonitor.out_y = FlowMonitor.flt_y_i - FlowMonitor.att_y_bias;

    /* comput velocity */
    if (FlowMonitor.lst_time_stamp)
    {
        dt = (time_stamp - FlowMonitor.lst_time_stamp) / 1000.0f;
        pos_x_diff = (FlowMonitor.out_x - FlowMonitor.lst_out_x) / dt;
        pos_y_diff = (FlowMonitor.out_y - FlowMonitor.lst_out_y) / dt;

        FlowMonitor.x_v += (pos_x_diff - FlowMonitor.x_v) * 0.1f;
        FlowMonitor.y_v += (pos_y_diff - FlowMonitor.y_v) * 0.1f;
    }

    /* set history data */
    FlowMonitor.lst_time_stamp = time_stamp;
    FlowMonitor.lst_out_x = FlowMonitor.out_x;
    FlowMonitor.lst_out_y = FlowMonitor.out_y;

    /* convert unit */
    cpi = (alt / 11.914f) * 2.54f;
    FlowMonitor.out_x *= cpi;
    FlowMonitor.out_y *= cpi;
    FlowMonitor.x_v *= cpi;
    FlowMonitor.y_v *= cpi;
}
