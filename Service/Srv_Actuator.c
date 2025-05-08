#include "Srv_Actuator.h"
#include "Srv_OsCommon.h"
#include "../System/DataPipe/DataPipe.h"

#define SRV_ACTUATOR_MAX_THROTTLE_PERCENT 80
#define Actuator_Malloc(size) SrvOsCommon.malloc(size)
#define Actuator_Free(ptr) SrvOsCommon.free(ptr)

/* internal variable */
DataPipe_CreateDataObj(SrvActuatorPipeData_TypeDef, Actuator);

/* internal function */
static void SrvActuator_PipeData(void);

/* external function */
static bool SrvActuator_Init(SrvActuator_ESCType_List ESC_type);
static void SrvActuator_MotoControl(int16_t *p_val);
static bool SrvActuator_Lock(void);
static bool SrvActuator_Moto_DirectDrive(uint8_t index, int16_t value);
static bool SrvActuator_Servo_DirectDrive(uint8_t index, uint16_t value);
static bool SrvActuator_SetSpin_Dir(uint8_t component_index, uint8_t dir);

/* external variable */
SrvActuator_TypeDef SrvActuator = {
    .init = SrvActuator_Init,
    .lock = SrvActuator_Lock,
    .mix_control = SrvActuator_MotoControl,
    .set_spin_dir = SrvActuator_SetSpin_Dir,
    .moto_direct_drive = SrvActuator_Moto_DirectDrive,
    .servo_direct_drive = SrvActuator_Servo_DirectDrive,
};

static bool SrvActuator_Init(SrvActuator_ESCType_List ESC_type)
{
    /* malloc dshot esc driver obj for using */
        
    /* default init */
    switch (ESC_type)
    {
        case ESC_Type_DShot150:
        case ESC_Type_DShot300:
        case ESC_Type_DShot600:
            /* set moto info */
            break;

        default: return false;
    }

    /* set servo object */


    /* data pipe init */
    Actuator_smp_DataPipe.data_addr = (uint32_t)DataPipe_DataObjAddr(Actuator);
    Actuator_smp_DataPipe.data_size = DataPipe_DataSize(Actuator);
    DataPipe_Enable(&Actuator_smp_DataPipe);

    /* check value remap relationship */
    /* we can read this info from storage module */
    SrvActuator_Lock();

    return true;
}

static bool SrvActuator_Lock(void)
{
    return true;
}

static void SrvActuator_PipeData(void)
{
    uint8_t i = 0;

    /* pipe actuator control data to data hub */
    DataPipe_DataObj(Actuator).time_stamp = SrvOsCommon.get_os_ms();

    /* set moto and servo control valuse */
    for (i = 0; i < ACTUATOR_MOTO_COUNT; i ++)
    {
        // DataPipe_DataObj(Actuator).moto[i] = ;
    }

    for (i = 0; i < ACTUATOR_SERVO_COUNT; i ++)
    {
        // DataPipe_DataObj(Actuator).servo[i] = ;
    }

    DataPipe_SendTo(&Actuator_smp_DataPipe, &Actuator_hub_DataPipe);
}

static void SrvActuator_MotoControl(int16_t *p_val)
{
    // uint8_t i = 0;
    // uint8_t offset = 0;

    // if ((p_val == NULL) || !SrvActuator_Obj.init)
    //     return;

    /* send control value through SPI bus */

    SrvActuator_PipeData();
}

static bool SrvActuator_SetSpin_Dir(uint8_t component_index, uint8_t dir)
{
    return false;
}

static bool SrvActuator_Moto_DirectDrive(uint8_t index, int16_t value)
{
    if (index >= ACTUATOR_MOTO_COUNT)
        return false;

    return false;
}

static bool SrvActuator_Servo_DirectDrive(uint8_t index, uint16_t value)
{
    if (index >= ACTUATOR_SERVO_COUNT)
        return false;

    return false;
}

