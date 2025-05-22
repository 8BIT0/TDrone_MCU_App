#include "Srv_Mag.h"
#include "Srv_OsCommon.h"
#include "Dev_IST8310.h"
#include "error_log.h"
#include "HW_Def.h"

#define MagModule_Reinit_Max_Cnt    5

typedef struct
{
    bool init_state;
    uint8_t reinit_cnt;
} SrvMagMonitor_TypeDef;

/* internal variable */
static SrvMagMonitor_TypeDef Monitor = {
    .init_state = false,
    .reinit_cnt = MagModule_Reinit_Max_Cnt,
};

/* internal function */
static DevIST8310Obj_TypeDef IST8310Obj;

/* external function */
static bool SrvMag_Init(void);

SrvMag_TypeDef SrvMag = {
    .init = SrvMag_Init,
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

    /* device init */
    IST8310Obj.init_state = false;
    IST8310Obj.bus_obj = (void *)&Mag_BusCfg;
    IST8310Obj.bus_write = (IST8310_Bus_Write)MagBus.write;
    IST8310Obj.bus_read = (IST8310_Bus_Read)MagBus.read;
    IST8310Obj.delay = SrvOsCommon.delay_ms;
    IST8310Obj.get_tick = SrvOsCommon.get_os_ms;
    
    if (!DevIST8310.init(&IST8310Obj))
        return false;

    Monitor.init_state = true;
    return true;
}
