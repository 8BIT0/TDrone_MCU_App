#include "Srv_Mag.h"
#include "Dev_IST8310.h"
#include "HW_Def.h"

/* internal function */

/* external function */
static bool SrvMag_Init(void);

SrvMag_TypeDef SrvMag = {
    .init = SrvMag_Init,
};

static bool SrvMag_Init(void)
{
    /* bus init */

    /* device init */

    return true;
}
