#include "Srv_Baro.h"
#include "HW_Def.h"
#include "Dev_BMP280.h"

typedef struct
{
    bool init;
} SrvBaroObj_TypeDef;

/* external funtion */
static bool SrvBaro_Init(void);

SrvBaro_TypeDef SrvBaro = {
    .init = SrvBaro_Init,
};


static bool SrvBaro_Init(void)
{
    /* bus init */

    /* module init */

    return false;
}

