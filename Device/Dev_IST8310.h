#ifndef __DEV_IST8310_H__
#define __DEV_IST8310_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "mag_data.h"

typedef bool (*IST8310_Bus_Read)(void *bus_obj, uint8_t dev_addr, uint8_t reg, uint8_t *p_data, uint16_t len);
typedef bool (*IST8310_Bus_Write)(void *bus_obj, uint8_t dev_addr, uint8_t reg, uint8_t *p_data, uint16_t len);
typedef uint32_t (*IST8310_Get_Tick)(void);
typedef void (*IST8310_Delay_ms)(uint32_t ms);

#define IST8310_ADDRESS_SUM     4
#define IST8310_MAG_FACTORY     0.3f

#define IST8310_I2C_ADDR_1      0x0C
#define IST8310_I2C_ADDR_2      0x0D
#define IST8310_I2C_ADDR_3      0x0E
#define IST8310_I2C_ADDR_4      0x0F

#define IST8310_REG_WHO_AM_I    0x00
#define IST8310_REG_STATUS_1    0x02
#define IST8310_REG_MAG_X_L     0x03
#define IST8310_REG_MAG_X_H     0x04
#define IST8310_REG_MAG_Y_L     0x05
#define IST8310_REG_MAG_Y_H     0x06
#define IST8310_REG_MAG_Z_L     0x07
#define IST8310_REG_MAG_Z_H     0x08
#define IST8310_REG_STATUS_2    0x09
#define IST8310_REG_CTRL_1      0x0A
#define IST8310_REG_CTRL_2      0x0B
#define IST8310_REG_SELFTEST    0x0C
#define IST8310_REG_TEMP_L      0x1C
#define IST8310_REG_TEMP_H      0x1D
#define IST8310_REG_AVGCNTL     0x41
#define IST8310_REG_PDCNTL      0x42

#define IST8310_DEV_ID          0x10

typedef enum
{
    IST8310_PulseDuration_Long = 0b01,
    IST8310_PulseDuration_Normal = 0b11,
} IST8310_PulseDuration_TypeDef;

typedef enum
{
    Stand_By = 0,
    Single_Measurement,
} IST8310_Control1_Mode_TypeDef;

typedef enum
{
    Drdy_Act_L = 0, /* drdy pin active low */
    Drdy_Act_H,     /* drdy pin active high */
} IST8310_Drdy_PinPolarity_TypeDef;

typedef enum
{
    Avg_None = 0,
    Avg_2_Times = 1,
    Avg_4_Times = 2,    /* default */
    Avg_8_Times = 3,
    Avg_16_Times = 4,
} IST8310_AverageTime_TypeDef;

typedef union
{
    uint8_t val;

    struct
    {
        uint8_t drdy : 1;   /* bit0. read only 0: data not ready  / 1: data ready */
        uint8_t dor  : 1;   /* bit1. read only 0: no data overrun / 1: data overrun. Bit is released after any output data register read  */
        uint8_t res  : 6;
    } bit;
} IST8310_Status1_TypeDef;

typedef union
{
    uint8_t val;

    struct
    {
        uint8_t res1 : 3;   /* bit0 ~ bit2 */
        uint8_t INT  : 1;   /* bit3 read only, When interrupt event occurs, this bit will be set to 1 */
        uint8_t res2 : 4;   /* bit4 ~ bit7 */
    } bit;
} IST8310_Status2_TypeDef;

typedef union
{
    uint8_t val;

    struct
    {
        uint8_t mode : 4;   /* bit0 ~ bit3 mode read & write */
        uint8_t res  : 4;   /* bit4 ~ bit7 */
    } bit;
} IST8310_Control1_TypeDef;

typedef union
{
    uint8_t val;

    struct
    {
        uint8_t soft_reset          : 1;    /* bit0 read & write default 0 */
        uint8_t res_1               : 1;    /* bit1 */
        uint8_t drdy_pin_polarity   : 1;    /* bit2 read & write drdy pin active level default 1 */
        uint8_t drdy_enable         : 1;    /* bit3 read & write drdy output pin enable default 1 */
        uint8_t res_2               : 4;    /* bit4 ~ bit7 */
    } bit;
} IST8310_Control2_TypeDef;

typedef union
{
    uint8_t val;

    struct
    {
        uint8_t res_1       : 6;    /* bit0 ~ bit5 */
        uint8_t self_test   : 1;    /* bit6 read & write self test enable */
        uint8_t res_2       : 1;    /* bit7 */
    } bit;
} IST8310_SelfTest_TypeDef;

typedef union
{
    uint8_t val;

    struct
    {
        uint8_t xz_avg_time : 3;    /* bit0 ~ bit2 read & write Average times for x & z sensor data */
        uint8_t y_avg_time  : 3;    /* bit3 ~ bit5 read & write Average times for y sensor data */
        uint8_t res         : 2;    /* bit6 ~ bit7 */
    } bit;
} IST8310_AvgControl_TypeDef;

typedef union
{
    uint8_t val;

    struct
    {
        uint8_t res             : 6;    /* bit0 ~ bit5 */
        uint8_t pulse_duration  : 2;    /* bit6 ~ bit7 duration selection */
    } bit;
} IST8310_PDControl_TypeDef;

typedef struct
{
    bool init_state;
    uint8_t dev_addr;
    uint8_t id;

    IST8310_Get_Tick get_tick;
    IST8310_Delay_ms delay;

    void *bus_obj;
    IST8310_Bus_Read bus_read;
    IST8310_Bus_Write bus_write;

    uint32_t update_cnt;
    MagData_TypeDef data;
} DevIST8310Obj_TypeDef;

typedef struct
{
    bool (*init)(DevIST8310Obj_TypeDef *obj);
    bool (*sample)(DevIST8310Obj_TypeDef *obj);
    bool (*get)(DevIST8310Obj_TypeDef *obj, MagData_TypeDef *p_data);
    bool (*reset)(DevIST8310Obj_TypeDef *obj);
    bool (*set_drdy)(DevIST8310Obj_TypeDef *obj);
} DevIST8310_TypeDef;

extern DevIST8310_TypeDef DevIST8310;

#ifdef __cplusplus
}
#endif

#endif
