#include "Dev_IST8310.h"

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
    if (!DevIST8310_Get_ID(obj))
        return false;

    return true;
}

static bool DevIST8310_Get_ID(DevIST8310Obj_TypeDef *obj)
{
    if ((obj == NULL) || (obj->bus_obj == NULL) || (obj->bus_obj->read == NULL))
        return false;

    if ((!obj->bus_read(obj->bus_obj, IST8310_DEV_ID, &obj->id, 1)) || (obj->id != IST8310_DEV_ID))
        return false;

    return true;
}
