#ifndef __GEN_CALIB_H
#define __GEN_CALIB_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    Calib_Ready = 0,
    Calib_Start,
    Calib_InProcess,
    Calib_Done,
    Calib_Failed,
} GenCalib_State_TypeList;

typedef union
{
    uint16_t val;

    struct
    {
        uint16_t imu : 1;
        uint16_t mag : 1;
        uint16_t baro : 1;
        uint16_t flow : 1;

        uint16_t reserve : 12;
    } bit;
} GenCalib_Reg_TypeDef;

#ifdef __cplusplus
}
#endif

#endif
