#include "Srv_Baro.h"
#include "Dev_BMP280.h"

/* external funtion */
static bool SrvBaro_Init(void);

SrvBaro_TypeDef SrvBaro = {
    .init = SrvBaro_Init,
};


static bool SrvBaro_Init(void)
{
    return false;
}

