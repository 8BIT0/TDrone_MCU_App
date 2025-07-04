#ifndef __DEV_BMP280_H
#define __DEV_BMP280_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "baro_data.h"

#define ToBMP280_OBJ(x)     ((DevBMP280Obj_TypeDef *)x)
#define ToBMP280_API(x)     ((DevBMP280_TypeDef *)x)
#define ToBMP280_DataPtr(x) ((DevBMP280_Data_TypeDef *)x)
#define BMP280_DataSize     sizeof(DevBMP280_Data_TypeDef)

#define BMP280_NORMAL_MODE             3

#define BMP280_ADDR_1                  0x76
#define BMP280_ADDR_2                  0x77

#define BMP280_REG_NVM_PAR_T1_L        0x88        /**< NVM PAR T1 low register */
#define BMP280_REG_NVM_PAR_T1_H        0x89        /**< NVM PAR T1 high register */
#define BMP280_REG_NVM_PAR_T2_L        0x8A        /**< NVM PAR T2 low register */
#define BMP280_REG_NVM_PAR_T2_H        0x8B        /**< NVM PAR T2 high register */
#define BMP280_REG_NVM_PAR_T3_L        0x8C        /**< NVM PAR T3 low register */
#define BMP280_REG_NVM_PAR_T3_H        0x8D        /**< NVM PAR T3 high register */
#define BMP280_REG_NVM_PAR_P1_L        0x8E        /**< NVM PAR P1 low register */
#define BMP280_REG_NVM_PAR_P1_H        0x8F        /**< NVM PAR P1 high register */
#define BMP280_REG_NVM_PAR_P2_L        0x90        /**< NVM PAR P2 low register */
#define BMP280_REG_NVM_PAR_P2_H        0x91        /**< NVM PAR P2 high register */
#define BMP280_REG_NVM_PAR_P3_L        0x92        /**< NVM PAR P3 low register */
#define BMP280_REG_NVM_PAR_P3_H        0x93        /**< NVM PAR P3 high register */
#define BMP280_REG_NVM_PAR_P4_L        0x94        /**< NVM PAR P4 low register */
#define BMP280_REG_NVM_PAR_P4_H        0x95        /**< NVM PAR P4 high register */
#define BMP280_REG_NVM_PAR_P5_L        0x96        /**< NVM PAR P5 low register */
#define BMP280_REG_NVM_PAR_P5_H        0x97        /**< NVM PAR P5 high register */
#define BMP280_REG_NVM_PAR_P6_L        0x98        /**< NVM PAR P6 low register */
#define BMP280_REG_NVM_PAR_P6_H        0x99        /**< NVM PAR P6 high register */
#define BMP280_REG_NVM_PAR_P7_L        0x9A        /**< NVM PAR P7 low register */
#define BMP280_REG_NVM_PAR_P7_H        0x9B        /**< NVM PAR P7 high register */
#define BMP280_REG_NVM_PAR_P8_L        0x9C        /**< NVM PAR P8 low register */
#define BMP280_REG_NVM_PAR_P8_H        0x9D        /**< NVM PAR P8 high register */
#define BMP280_REG_NVM_PAR_P9_L        0x9E        /**< NVM PAR P9 low register */
#define BMP280_REG_NVM_PAR_P9_H        0x9F        /**< NVM PAR P9 high register */
#define BMP280_REG_TEMP_XLSB           0xFC        /**< temp xlsb register */
#define BMP280_REG_TEMP_LSB            0xFB        /**< temp lsb register */
#define BMP280_REG_TEMP_MSB            0xFA        /**< temp msb register */
#define BMP280_REG_PRESS_XLSB          0xF9        /**< press xlsb register */
#define BMP280_REG_PRESS_LSB           0xF8        /**< press lsb register */
#define BMP280_REG_PRESS_MSB           0xF7        /**< press msb register */
#define BMP280_REG_CONFIG              0xF5        /**< config register */
#define BMP280_REG_CTRL_MEAS           0xF4        /**< ctrl meas register */
#define BMP280_REG_STATUS              0xF3        /**< status register */
#define BMP280_REG_RESET               0xE0        /**< soft reset register */
#define BMP280_REG_ID                  0xD0        /**< chip id register */
#define BMP280_DEVICE_ID               0x58        /**< chip id */

typedef enum
{
    DevBMP280_Error_None = 0,
    DevBMP280_Para_Error,
    DevBMP280_Init_Error,
    DevBMP280_Reset_Error,
    DevBMP280_Set_Mode_Error,
    DevBMP280_Get_CalibParam_Error,
    DevBMP280_Set_Temperature_OverSampling_Error,
    DevBMP280_Set_Pressure_OverSampling_Error,
    DevBMP280_Set_Filter_Error,
    DevBMP280_ID_Error,
} DevBMP280_ErrorCode_List;

typedef enum
{
    DevBMP280_OVERSAMPLING_SKIP = 0x00,
    DevBMP280_OVERSAMPLING_x1   = 0x01,
    DevBMP280_OVERSAMPLING_x2   = 0x02,
    DevBMP280_OVERSAMPLING_x4   = 0x03,
    DevBMP280_OVERSAMPLING_x8   = 0x04,
    DevBMP280_OVERSAMPLING_x16  = 0x05,
} DevBMP280_OverSampling_List;

typedef enum
{
    DevBMP280_FILTER_OFF      = 0x00,
    DevBMP280_FILTER_COEFF_2  = 0x01,
    DevBMP280_FILTER_COEFF_4  = 0x02,
    DevBMP280_FILTER_COEFF_8  = 0x03,
    DevBMP280_FILTER_COEFF_16 = 0x04,
} DevBMP280_Filter_List;

typedef uint32_t (*DevBMP280_Get_Tick)(void);
typedef void (*DevBMP280_Delay_Ms)(uint32_t ms);

typedef bool (*DevBMP280_IIC_Write)(void *bus_obj, uint16_t dev_addr, uint16_t reg_addr, uint8_t *p_data, uint16_t len);
typedef bool (*DevBMP280_IIC_Read)(void *bus_obj, uint16_t dev_addr, uint16_t reg_addr, uint8_t *p_data, uint16_t len);

typedef struct
{
    uint16_t t1;
    int16_t t2;
    int16_t t3;
    uint16_t p1;
    int16_t p2;
    int16_t p3;
    int16_t p4;
    int16_t p5;
    int16_t p6;
    int16_t p7;
    int16_t p8;
    int16_t p9;
    int32_t t_fine;
} DevBMP280_Calib_TypeDef;

typedef struct
{
    uint8_t id;

    DevBMP280_ErrorCode_List ErrorCode;

    DevBMP280_Get_Tick get_tick;
    DevBMP280_Delay_Ms delay_ms;

    DevBMP280_Calib_TypeDef calib;

    uint32_t sys_tick;
    uint32_t cyc;
    uint32_t lst_sys_tick;

    float raw_temperature;  /* without compensate temperature */
    float raw_pressure;     /* without compensate pressure */

    float temperature;
    float pressure;

    /* IIC section */
    uint8_t DevAddr;
    void *bus_obj;
    DevBMP280_IIC_Write bus_tx;
    DevBMP280_IIC_Read bus_rx;
} DevBMP280Obj_TypeDef;

typedef struct
{
    uint32_t time_stamp;
    uint32_t cyc;

    float scaled_press;
    float scaled_tempra;
}DevBMP280_Data_TypeDef;

typedef union
{
    uint8_t val;

    struct
    {
        uint8_t im_update : 1;
        uint8_t res_1     : 2;
        uint8_t measuring : 1;
        uint8_t res_2     : 4;
    } bit;
} DevBMP280_Status_TypeDef;

typedef struct
{
    bool (*init)(DevBMP280Obj_TypeDef *obj);
    bool (*sample)(DevBMP280Obj_TypeDef *obj);
    bool (*ready)(DevBMP280Obj_TypeDef *obj);
    DevBMP280_Data_TypeDef (*get_data)(DevBMP280Obj_TypeDef *obj);
} DevBMP280_TypeDef;

extern DevBMP280_TypeDef DevBMP280;

#ifdef __cplusplus
}
#endif

#endif
