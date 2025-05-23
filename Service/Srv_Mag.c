#include "Srv_Mag.h"
#include "Srv_OsCommon.h"
#include "Storage.h"
#include "Dev_IST8310.h"
#include "error_log.h"
#include "HW_Def.h"

#define MagModule_Reinit_Max_Cnt    5

typedef struct
{
    bool init_state;
    uint8_t reinit_cnt;

    void *dev_obj;
    bool (*dev_sample)(void *dev_obj);
    bool (*dev_get)(void *dev_obj, MagData_TypeDef *p_data);
} SrvMagMonitor_TypeDef;

/* internal variable */
static SrvMagMonitor_TypeDef Monitor = {
    .init_state = false,
    .reinit_cnt = MagModule_Reinit_Max_Cnt,
};
static DevIST8310Obj_TypeDef IST8310Obj;

/* internal function */
static bool SrvMag_BusInit(void);
static void SrvMag_EllipsoidFitting_Comput(float *p_mag_in, float *p_mag_out);

/* external function */
static bool SrvMag_Init(void);
static bool SrvMag_Sample(void);

SrvMag_TypeDef SrvMag = {
    .init = SrvMag_Init,
    .sample = SrvMag_Sample,
};

static bool SrvMag_BusInit(void)
{
    if (IIC_BusCfg.handle == NULL)
    {
        IIC_BusCfg.handle = SrvOsCommon.malloc(MagBus_Handle_Size);
        if (Mag_BusCfg.handle == NULL)
            return false;
    }

    if (Mag_BusCfg.PeriphClkInitStruct == NULL)
    {
        Mag_BusCfg.PeriphClkInitStruct = SrvOsCommon.malloc(MagBus_PeriphClk_Size);
        if (Mag_BusCfg.PeriphClkInitStruct == NULL)
        {
            SrvOsCommon.free(Mag_BusCfg.handle);
            return false;
        }
    }

    if (!BaroBus.init(&Mag_BusCfg))
    {
        SrvOsCommon.free(Mag_BusCfg.handle);
        SrvOsCommon.free(Mag_BusCfg.PeriphClkInitStruct);
        return false;
    }
    
    return true;
}

static bool SrvMag_Init(void)
{
    Monitor.init_state = false;

    /* bus init */
    if (!SrvMag_BusInit())
        return false;

    /* load ellipsoid fitting parameter */

    /* device init */
    IST8310Obj.init_state = false;
    IST8310Obj.bus_obj = (void *)&Mag_BusCfg;
    IST8310Obj.bus_write = (IST8310_Bus_Write)MagBus.write;
    IST8310Obj.bus_read = (IST8310_Bus_Read)MagBus.read;
    IST8310Obj.delay = SrvOsCommon.delay_ms;
    IST8310Obj.get_tick = SrvOsCommon.get_os_ms;
    IST8310Obj.hw_drdy_en = false;
    
    if (!DevIST8310.init(&IST8310Obj))
        return false;

    Monitor.dev_obj = (void *)&IST8310Obj;
    Monitor.dev_sample = (bool (*)(void *))DevIST8310.sample;
    Monitor.dev_get = (bool (*)(void *, MagData_TypeDef *))DevIST8310.get;

    Monitor.init_state = true;
    return true;
}

static bool SrvMag_Sample(void)
{
    float raw_mag[Mag_Axis_Num];
    float elli_fix_mag[Mag_Axis_Num];
    MagData_TypeDef mag_data;

    memset(&mag_data, 0, sizeof(MagData_TypeDef));
    memset(raw_mag, 0, sizeof(raw_mag));
    memset(elli_fix_mag, 0, sizeof(elli_fix_mag));

    if (!Monitor.init_state || \
        (Monitor.dev_obj == NULL) || \
        (Monitor.dev_sample == NULL))
        return false;

    /* sample and get data */
    if (!Monitor.dev_sample(Monitor.dev_obj) || \
        !Monitor.dev_get(Monitor.dev_obj, &mag_data))
        return false;

    /* doing ellipsoid fitting */
    SrvMag_EllipsoidFitting_Comput(raw_mag, elli_fix_mag);

    return true;
}

static void SrvMag_EllipsoidFitting_Comput(float *p_mag_in, float *p_mag_out)
{
    if (!Monitor.init_state || (p_mag_in == NULL) || (p_mag_out == NULL))
    {
        memcpy(p_mag_out, p_mag_in, sizeof(float) * Mag_Axis_Num);
        return;
    }
}
