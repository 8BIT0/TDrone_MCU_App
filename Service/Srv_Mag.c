#include "Srv_Mag.h"
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

/* external function */
static bool SrvMag_Init(void);

SrvMag_TypeDef SrvMag = {
    .init = SrvMag_Init,
};

static bool SrvMag_Init(void)
{
    /* bus init */
    if (!MagBus.init(&Mag_BusCfg))
        return false;

    /* device init */
    for (;;)
    {
        // if ()
    }

    return true;
}
