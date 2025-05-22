#include "Dev_IST8310.h"

/* internal variable */
static const uint8_t IST8310_Address_List[IST8310_ADDRESS_SUM] = {
   IST8310_I2C_ADDR_1,
   IST8310_I2C_ADDR_2,
   IST8310_I2C_ADDR_3,
   IST8310_I2C_ADDR_4,
};

/* internal function */
static bool DevIST8310_Get_ID(DevIST8310Obj_TypeDef *obj);

/* external function */
static bool DevIST8310_Init(DevIST8310Obj_TypeDef *obj);

DevIST8310_TypeDef DevIST8310 = {
    .init = DevIST8310_Init,
};

static bool DevIST8310_Init(DevIST8310Obj_TypeDef *obj)
{
    if (obj == NULL)
        return false;

    /* check device id */
    for (uint8_t i = 0; i < IST8310_ADDRESS_SUM; i++)
    {
        obj->dev_addr = (uint8_t)(IST8310_Address_List[i] << 1);
        DevIST8310_Get_ID(obj);
        if (obj->id == IST8310_DEV_ID)
            break;    
    }

    /* final id check */
    if (obj->id != IST8310_DEV_ID)
        return false;

    /* soft reset */

    return true;
}

static bool DevIST8310_Get_ID(DevIST8310Obj_TypeDef *obj)
{
    if ((obj == NULL) || (obj->bus_obj == NULL) || (obj->bus_read == NULL))
        return false;

    if ((!obj->bus_read(obj->bus_obj, obj->dev_addr, IST8310_REG_WHO_AM_I, &obj->id, (uint16_t)1)))
        return false;

    return true;
}
