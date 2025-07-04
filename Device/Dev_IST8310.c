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
static bool DevIST8310_Prepare_Measure(DevIST8310Obj_TypeDef *obj);

/* external function */
static bool DevIST8310_Init(DevIST8310Obj_TypeDef *obj);
static bool DevIST8310_SoftReset(DevIST8310Obj_TypeDef *obj);
static bool DevIST8310_Set_Drdy(DevIST8310Obj_TypeDef *obj);    /* when int pin is attach to mcu */
static bool DevIST8310_Sample(DevIST8310Obj_TypeDef *obj);
static bool DevIST8310_GetData(DevIST8310Obj_TypeDef *obj, MagData_TypeDef *p_data);

DevIST8310_TypeDef DevIST8310 = {
    .init       = DevIST8310_Init,
    .sample     = DevIST8310_Sample,
    .reset      = DevIST8310_SoftReset,
    .set_drdy   = DevIST8310_Set_Drdy,
    .get        = DevIST8310_GetData,
};

static bool DevIST8310_Init(DevIST8310Obj_TypeDef *obj)
{
    IST8310_Control2_TypeDef ctl2;
    IST8310_AvgControl_TypeDef avg;
    IST8310_PDControl_TypeDef pdctl;
    uint8_t read_tmp = 0;

    if ((obj == NULL) || \
        (obj->bus_obj == NULL) || \
        (obj->bus_read == NULL) || \
        (obj->bus_write == NULL))
        return false;

    obj->init_state = false;
    obj->update_cnt = 0;

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
    {
        obj->id = 0;
        obj->dev_addr = 0;
        return false;
    }

    /* soft reset */
    if (!DevIST8310_SoftReset(obj))
        return false;

    /* drdy pin setting */
    ctl2.val = 0;
    ctl2.bit.drdy_pin_polarity = Drdy_Act_H;
    ctl2.bit.drdy_enable = true;

    if (!obj->bus_write(obj->bus_obj, obj->dev_addr, IST8310_REG_CTRL_2, &ctl2.val, (uint16_t)1) || \
        !obj->bus_read(obj->bus_obj, obj->dev_addr, IST8310_REG_CTRL_2, &read_tmp, (uint16_t)1) || \
        (read_tmp != ctl2.val))
        return false;

    /* set pulse duration */
    pdctl.val = 0;
    pdctl.bit.pulse_duration = IST8310_PulseDuration_Normal;

    if (!obj->bus_write(obj->bus_obj, obj->dev_addr, IST8310_REG_PDCNTL, &pdctl.val, (uint16_t)1) || \
        !obj->bus_read(obj->bus_obj, obj->dev_addr, IST8310_REG_PDCNTL, &read_tmp, (uint16_t)1) || \
        (read_tmp != pdctl.val))
        return false;

    /* sample average setting */
    avg.val = 0;
    avg.bit.xz_avg_time = Avg_16_Times;
    avg.bit.y_avg_time = Avg_16_Times;

    if (!obj->bus_write(obj->bus_obj, obj->dev_addr, IST8310_REG_AVGCNTL, &avg.val, (uint16_t)1) || \
        !obj->bus_read(obj->bus_obj, obj->dev_addr, IST8310_REG_AVGCNTL, &read_tmp, (uint16_t)1) || \
        (read_tmp != avg.val))
        return false;

    obj->init_state = true;

    DevIST8310_Prepare_Measure(obj);
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

static bool DevIST8310_SoftReset(DevIST8310Obj_TypeDef *obj)
{
    IST8310_Control2_TypeDef ctl2;

    if ((obj == NULL) || (obj->delay == NULL) || (obj->dev_addr == 0) || (obj->bus_obj == NULL) || (obj->bus_write == NULL))
        return false;

    ctl2.val = 0;
    ctl2.bit.soft_reset = 1;

    if (!obj->bus_write(obj->bus_obj, obj->dev_addr, IST8310_REG_CTRL_2, &ctl2.val, (uint16_t)1))
        return false;

    obj->delay(100);
    return true;
}

static bool DevIST8310_Set_Drdy(DevIST8310Obj_TypeDef *obj)
{
    if ((obj == NULL) || !obj->init_state)
        return false;

    return true;
}

static bool DevIST8310_Sample(DevIST8310Obj_TypeDef *obj)
{
    IST8310_Status1_TypeDef status;
    uint8_t mag_tmp[Mag_Axis_Sum * 2];
    uint8_t temp_tmp[2];
    volatile uint32_t sys_time = 0;
    uint8_t read_tmp = 0;

    if ((obj == NULL) || \
        !obj->init_state || \
        (obj->bus_obj == NULL) || \
        (obj->bus_read == NULL) || \
        (obj->get_tick == NULL) || \
        (obj->get_tick == NULL))
        return false;

    /* check data ready register */
    sys_time = obj->get_tick();
    status.val = 0;
    while (1)
    {
        if ((!obj->bus_read(obj->bus_obj, obj->dev_addr, IST8310_REG_STATUS_1, &status.val, (uint16_t)1)) || \
            (obj->get_tick() - sys_time > 10))
        {
            obj->bus_read(obj->bus_obj, obj->dev_addr, IST8310_REG_STATUS_2, &read_tmp, (uint16_t)1);
            DevIST8310_Prepare_Measure(obj);
            return false;
        }

        if (status.bit.drdy)
            break;
    }

    memset(mag_tmp, 0, sizeof(mag_tmp));
    memset(temp_tmp, 0, sizeof(temp_tmp));

    /* read mag data */
    obj->data.time_stamp = obj->get_tick();
    if (!obj->bus_read(obj->bus_obj, obj->dev_addr, IST8310_REG_MAG_X_L, mag_tmp, (uint16_t)(Mag_Axis_Sum * 2)) || \
        !obj->bus_read(obj->bus_obj, obj->dev_addr, IST8310_REG_TEMP_L, temp_tmp, (uint16_t)2))
    {
        obj->data.time_stamp = 0;
        DevIST8310_Prepare_Measure(obj);
        return false;
    }

    DevIST8310_Prepare_Measure(obj);
    /* set mag axis data */
    for (uint8_t i = Mag_Axis_X; i < Mag_Axis_Sum; i++)
    {
        /* convert unit */
        obj->data.mag[i] = (float)(((int16_t)(mag_tmp[i * 2 + 1] << 8) | mag_tmp[i * 2]) * IST8310_MAG_FACTORY);
    }

    /* set mag temperature */
    obj->data.mag_temp = (int16_t)(temp_tmp[1] << 8) | temp_tmp[0];
    obj->update_cnt ++;
    return true;
}

static bool DevIST8310_GetData(DevIST8310Obj_TypeDef *obj, MagData_TypeDef *p_data)
{
    if ((obj == NULL) || !obj->init_state || (p_data == NULL))
        return false;

    memcpy(p_data, &(obj->data), sizeof(MagData_TypeDef));
    return true;
}

static bool DevIST8310_Prepare_Measure(DevIST8310Obj_TypeDef *obj)
{
    IST8310_Control1_TypeDef ctl1;

    /* sample mode setting */
    ctl1.val = 0;
    ctl1.bit.mode = Single_Measurement;

    if (!obj->bus_write(obj->bus_obj, obj->dev_addr, IST8310_REG_CTRL_1, &ctl1.val, (uint16_t)1))
        return false;

    return true;
}
