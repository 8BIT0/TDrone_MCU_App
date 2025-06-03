#include "Srv_Mag.h"
#include "Srv_OsCommon.h"
#include "Storage.h"
#include "error_log.h"
#include "HW_Def.h"

typedef struct
{
    float C[Mag_Axis_Sum];                  /* center point */
    float W[Mag_Axis_Sum][Mag_Axis_Sum];    /* soft iron fitting matrix */
} SrvMag_CalibMatrix_TypeDef;

typedef struct
{
    bool init_state;
    SrvMag_Data_TypeDef data;

    void *dev_obj;
    bool (*dev_sample)(void *dev_obj);
    bool (*dev_get)(void *dev_obj, MagData_TypeDef *p_data);
} SrvMagMonitor_TypeDef;

/* internal variable */
static SrvMagMonitor_TypeDef Monitor = {
    .init_state = false,

    .dev_obj = NULL,
    .dev_sample = NULL,
    .dev_get = NULL,
};
static DevIST8310Obj_TypeDef IST8310Obj;

/* internal function */
static bool SrvMag_BusInit(void);
static void SrvMag_EllipsoidFitting_Comput(float *p_mag_in, float *p_mag_out);

/* external function */
static bool SrvMag_Init(void);
static bool SrvMag_Sample(void);
static bool SrvMag_GetData(SrvMag_Data_TypeDef *p_data);

SrvMag_TypeDef SrvMag = {
    .init   = SrvMag_Init,
    .sample = SrvMag_Sample,
    .get    = SrvMag_GetData,
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
    memset(&IST8310Obj, 0, sizeof(DevIST8310Obj_TypeDef));
    IST8310Obj.init_state = false;
    IST8310Obj.bus_obj = (void *)&Mag_BusCfg;
    IST8310Obj.bus_write = (IST8310_Bus_Write)MagBus.write;
    IST8310Obj.bus_read = (IST8310_Bus_Read)MagBus.read;
    IST8310Obj.delay = SrvOsCommon.delay_ms;
    IST8310Obj.get_tick = SrvOsCommon.get_os_ms;
    
    if (!DevIST8310.init(&IST8310Obj))
        return false;

    Monitor.dev_obj = (void *)&IST8310Obj;
    Monitor.dev_sample = (bool (*)(void *))DevIST8310.sample;
    Monitor.dev_get = (bool (*)(void *, MagData_TypeDef *))DevIST8310.get;

    Monitor.init_state = true;
    memset(&Monitor.data, 0, sizeof(SrvMag_Data_TypeDef));
    return true;
}

static bool SrvMag_Sample(void)
{
    float raw_mag[Mag_Axis_Sum];
    float elli_fix_mag[Mag_Axis_Sum];
    MagData_TypeDef dev_mag_data;

    memset(&dev_mag_data, 0, sizeof(MagData_TypeDef));
    memset(raw_mag, 0, sizeof(raw_mag));
    memset(elli_fix_mag, 0, sizeof(elli_fix_mag));

    if (!Monitor.init_state || \
        (Monitor.dev_obj == NULL) || \
        (Monitor.dev_sample == NULL))
        return false;

    /* sample and get data */
    if (!Monitor.dev_sample(Monitor.dev_obj) || \
        !Monitor.dev_get(Monitor.dev_obj, &dev_mag_data))
        return false;

    memcpy(raw_mag, dev_mag_data.mag, sizeof(raw_mag));

    /* doing ellipsoid fitting */
    SrvMag_EllipsoidFitting_Comput(raw_mag, elli_fix_mag);

    /* save data */
    Monitor.data.time_stamp = dev_mag_data.time_stamp ;
    memcpy(Monitor.data.raw_mag, raw_mag, sizeof(raw_mag));
    memcpy(Monitor.data.fit_mag, elli_fix_mag, sizeof(elli_fix_mag));

    return true;
}

static void SrvMag_EllipsoidFitting_Comput(float *p_mag_in, float *p_mag_out)
{
    if (!Monitor.init_state || (p_mag_in == NULL) || (p_mag_out == NULL))
    {
        memcpy(p_mag_out, p_mag_in, sizeof(float) * Mag_Axis_Sum);
        return;
    }
}

static bool SrvMag_GetData(SrvMag_Data_TypeDef *p_data)
{
    if (!Monitor.init_state || (p_data == NULL))
        return false;

    memcpy(p_data, &Monitor.data, sizeof(SrvMag_Data_TypeDef));
    return true;
}
