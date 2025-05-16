/*
 *  Author: 8_B!T0
 *  Baro pressure data unit: pa
 */
#include "Srv_Baro.h"
#include "Srv_OsCommon.h"
#include "HW_Def.h"
#include "Dev_BMP280.h"

#define BARO_UNREADY_THRESHOLD          50
#define BARO_DATA_ACCURACY              100.0f
#define BARO_PRESSURE_BLUNT_THRESHOLD   20
#define MAX_BARO_VERTICAL_VELOCITY      1           /* unit: m/s */
#define STANDER_BARO_PRESSURE           101325.0f   /* unit: pa */

typedef struct
{
    bool init_state;
    uint32_t unready_cnt;
    uint32_t blunt_cnt;
    SrvBaro_Data_TypeDef cur_data;
    SrvBaro_Data_TypeDef lst_data;
} SrvBaroObj_TypeDef;

/* internal variable */
static SrvBaroObj_TypeDef SrvBaroObj = {
    .init_state  = false,
    .unready_cnt = 0,
    .blunt_cnt   = 0,
};
static DevBMP280Obj_TypeDef DevBMP280Obj;

/* internal function */
static void SrvBaro_DataCheck(void);

/* external function */
static bool SrvBaro_Init(void);
static bool SrvBaro_Sample(void);
static bool SrvBaro_GetData(SrvBaro_Data_TypeDef *data);

SrvBaro_TypeDef SrvBaro = {
    .init   = SrvBaro_Init,
    .sample = SrvBaro_Sample,
    .get    = SrvBaro_GetData,
};

static bool SrvBaro_Init(void)
{
    bool state = false;

    /* bus init */
    state = BaroBus.init(&Baro_BusCfg);

    /* module init */
    DevBMP280Obj.delay_ms = SrvOsCommon.delay_ms;
    DevBMP280Obj.get_tick = SrvOsCommon.get_os_ms;
    DevBMP280Obj.bus_obj  = (void *)&Baro_BusCfg;
    DevBMP280Obj.bus_rx   = (DevBMP280_IIC_Read)BaroBus.read;
    DevBMP280Obj.bus_tx   = (DevBMP280_IIC_Write)BaroBus.write;
    state &= DevBMP280.init(&DevBMP280Obj);

    SrvBaroObj.init_state = state;
    SrvBaroObj.cur_data.err.val = 0;
    if (!state)
        SrvBaroObj.cur_data.err.bit.init = true;
    return state;
}

static bool SrvBaro_Sample(void)
{
    bool state = false;

    if (!SrvBaroObj.init_state)
        return false;

    /* check baro data ready and sample data */
    if (DevBMP280.ready(&DevBMP280Obj) && DevBMP280.sample(&DevBMP280Obj))
    {
        SrvBaroObj.unready_cnt = 0;
        /* update data */
        SrvBaroObj.lst_data = SrvBaroObj.cur_data;

        /* unit convert */

        state = true;
    }
    else
    {
        SrvBaroObj.unready_cnt ++;
        SrvBaroObj.cur_data = SrvBaroObj.lst_data;
    }

    SrvBaro_DataCheck();
    return state;
}

static void SrvBaro_DataCheck(void)
{                       
    int32_t lst_press = lrintf((int32_t)(SrvBaroObj.lst_data.pres * BARO_DATA_ACCURACY));
    int32_t cur_press = lrintf((int32_t)(SrvBaroObj.cur_data.pres * BARO_DATA_ACCURACY));

    /* blunt check */
    if (lst_press == cur_press)
    {
        SrvBaroObj.blunt_cnt ++;
        if (SrvBaroObj.blunt_cnt >= BARO_PRESSURE_BLUNT_THRESHOLD)
            SrvBaroObj.cur_data.err.bit.meas = true;
    }
    else
        SrvBaroObj.blunt_cnt = 0;

    /* exceed change rate check */

    /* unready check */
    if (SrvBaroObj.unready_cnt >= BARO_UNREADY_THRESHOLD)
        SrvBaroObj.cur_data.err.bit.unready = true;
}

static bool SrvBaro_GetData(SrvBaro_Data_TypeDef *data)
{
    if ((SrvBaroObj.init_state == false) || (data == NULL))
        return false;

    memcpy(data, &SrvBaroObj.cur_data, sizeof(SrvBaro_Data_TypeDef));
    return true;
}
