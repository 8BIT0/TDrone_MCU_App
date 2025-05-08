#ifndef __SRV_RECEIVER_H
#define __SRV_RECEIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "Srv_OsCommon.h"
#include "Dev_CRSF.h"
#include "Bsp_GPIO.h"
#include "Bsp_Uart.h"
#include "control_data.h"

#define SRV_RECEIVER_UPDATE_TIMEOUT_MS 100
#define SRV_RECEIVER_BUFF_SIZE 256

#define CHANNEL_RANGE_MIN 100
#define CHANNEL_RANGE_MID 1000
#define CHANNEL_RANGE_MAX 1900

#define SRV_RECEIVER_MIN_RSSI 10
#define SRV_RECEIVER_MIN_LINKQUALITY 10
#define SRV_RECEIVER_MIN_ANTENNA_VALUE 5

typedef enum
{
    Receiver_No_Error = 0,
    Receiver_Obj_Error,
    Receiver_Port_Error,
    Receiver_Port_Init_Error,
    Receiver_FrameType_Error,
} SrvReceiver_ErrorCode_List;

typedef struct
{
    bool FailSafe : 1;
    bool Update_TimeOut : 1;

    uint8_t init_error_code;

    uint32_t cur_rt;
    uint32_t lst_update_rt;

    uint32_t success_decode_cnt;
    uint32_t error_decode_cnt;
    uint32_t total_decode_cnt;

    uint32_t over_rang_cnt;
    uint32_t value_step_bump_cnt;
} SrvReceiver_Monitor_TypeDef;

#define SRVRECEIVER_SIZE sizeof(SrvReceiver_Monitor_TypeDef)

typedef struct
{
    uint32_t time_stamp;

    uint16_t *val_list;
    uint16_t rssi;
    uint16_t link_quality;
    uint8_t active_antenna;

    bool failsafe;
} SrvReceiverData_TypeDef;

typedef struct
{
    void *api;
    void *cfg;
} SrvReceiver_Port_TypeDef;

typedef struct
{
    bool arm_state;
    bool buzz_state;
    bool cali_state;
    bool blackbox;
    bool failsafe;

    uint16_t rssi;
    uint16_t link_quality;

    uint16_t gimbal_percent[4];
    uint16_t module_enable;

    uint32_t update_interval;
    uint32_t time_stamp;

    uint8_t channel_sum;
    int16_t channel[Channel_Max];
} SrvRecever_RCSig_TypeDef;

typedef struct
{
    uint8_t channel_num;
    uint8_t *frame_data_obj;
    SrvReceiverData_TypeDef data;
    void *frame_api;
    SrvReceiver_Port_TypeDef *port;

    bool re_update;
    bool in_use;
} SrvReceiverObj_TypeDef;

typedef struct
{
    uint8_t *(*create_serial_obj)(uint32_t serial_instance,
                                  uint32_t rx_dma,
                                  uint32_t rx_dma_stream,
                                  uint32_t tx_dma,
                                  uint32_t tx_dma_stream,
                                  bool swap,
                                  const BspGPIO_Obj_TypeDef tx_pin,
                                  const BspGPIO_Obj_TypeDef rx_pin);
    bool (*init)(SrvReceiverObj_TypeDef *obj, uint8_t *port_obj);
    SrvReceiverData_TypeDef (*get)(SrvReceiverObj_TypeDef *obj);
    bool (*get_scope)(SrvReceiverObj_TypeDef *obj, int16_t *max, int16_t *mid, int16_t *min);
} SrvReceiver_TypeDef;

extern SrvReceiver_TypeDef SrvReceiver;

#ifdef __cplusplus
}
#endif

#endif
