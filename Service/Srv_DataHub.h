#ifndef __SRV_DATAHUB_H
#define __SRV_DATAHUB_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "DataPipe.h"
#include "navi_data.h"
#include "Srv_Actuator.h"
#include "Srv_Receiver.h"

#define SRVDATAHUB_TUNNING_HEARTBEAT_TIMEOUT 3000  /* unit: ms 3S timeout */
#define SRVDATAHUB_CONFIGRATOR_ATTACH_TIMEOUT 2000 /* unit: ms 2S timeout */

#define DRONE_ARM 1
#define DRONE_DISARM 0

typedef union
{
    struct
    {
        uint32_t sensor_init        : 1;    // bit : 0
        uint32_t navi_data          : 1;    // bit : 1
        uint32_t imu_data           : 1;    // bit : 2
        uint32_t baro_data          : 1;    // bit : 3
        uint32_t mag_data           : 1;    // bit : 4
        uint32_t range_imu          : 1;    // bit : 5
        uint32_t range_mag          : 1;    // bit : 6
        uint32_t range_baro         : 1;    // bit : 7
        uint32_t range_tof          : 1;    // bit : 8
        uint32_t range_flow         : 1;    // bit : 9
        uint32_t cnv_control_data   : 1;    // bit : 10
        uint32_t rc_control_data    : 1;    // bit : 11
        uint32_t actuator           : 1;    // bit : 12
        uint32_t USB_VCP_attach     : 1;    // bit : 13
        uint32_t cli                : 1;    // bit : 14
        uint32_t upgrade            : 1;    // bit : 15
        uint32_t res                : 16;   // bit : 16 ~ 32
    } bit;

    uint32_t val;
} SrvDataHub_UpdateReg_TypeDef;

typedef struct
{
    int16_t gyro_range[2];
    int16_t acc_range[2];

    uint32_t navi_update_time;
    float navi_roll;
    float navi_pitch;
    float navi_yaw;
    float navi_heading;
    float navi_q0;
    float navi_q1;
    float navi_q2;
    float navi_q3;
    float navi_acc[Axis_Sum];
    float navi_gyr[Axis_Sum];
    float navi_mag[Mag_Axis_Sum];
    float navi_pres;
    float navi_alt;
    float navi_tof_alt;
    float navi_rel_vel[POS_Axis_Sum];
    float navi_rel_pos[POS_Axis_Sum];

    /* altitude relative to lift off place */
    uint32_t relative_alt_time;
    float relative_alt;
    float relative_vertical_speed;

    ControlData_TypeDef RC_Control_Data;

    uint32_t cnvctl_data_time;
    bool arm;
    bool failsafe;
    uint8_t throttle_percent;
    float exp_pitch;
    float exp_roll;
    float exp_gyr_x;
    float exp_gyr_y;
    float exp_gyr_z;

    uint32_t baro_update_time;
    float baro_pres;
    float baro_temp;

    uint32_t imu_update_time;
    float raw_acc[Axis_Sum];
    float raw_gyr[Axis_Sum];

    uint32_t mag_update_time;
    float raw_mag[Mag_Axis_Sum];

    uint32_t pos_update_time;
    double raw_pos[POS_Axis_Sum];
    double raw_vel[POS_Axis_Sum];

    uint32_t tof_update_time;
    float tof;
    bool tof_valid;

    uint32_t actuator_update_time;
    int16_t moto[2];
    int16_t servo[2];

    bool upgrading;

    bool CLI_state;
    bool VCP_Attach;
} SrvDataHubObj_TypeDef;

typedef struct
{
    bool init_state;
    SrvDataHub_UpdateReg_TypeDef inuse_reg;
    SrvDataHub_UpdateReg_TypeDef update_reg;
    SrvDataHubObj_TypeDef data;
} SrvDataHub_Monitor_TypeDef;

typedef struct
{
    void (*init)(void);
    bool (*set_vcp_attach_state)(bool state);
    bool (*set_upgrade_state)(bool state);
    bool (*set_cli_state)(bool state);

    bool (*get_upgrade_state)(bool *state);
    bool (*get_cli_state)(bool *state);
    bool (*get_vcp_attach_state)(bool *state);

    /* sensor data */
    bool (*get_imu)(uint32_t *time_stamp, float *gx, float *gy, float *gz, float *ax, float *ay, float *az);
    bool (*get_mag)(uint32_t *time_stamp, float *mx, float *my, float *mz);
    bool (*get_baro)(uint32_t *time_stamp, float *press, float *temp);

    bool (*get_attitude)(uint32_t *time_stamp, float *pitch, float *roll, float *yaw, float *heading);
    bool (*get_rc_control_data)(ControlData_TypeDef *data);
    bool (*get_cnv_control_data)(uint32_t *time_stamp, bool *arm, bool *failsafe, float *pitch, float *roll, float *gx, float *gy, float *gz);
    bool (*get_relative_alt)(uint32_t *time_stamp, float *alt, float *alt_speed);
    bool (*get_arm_state)(bool *arm);
    bool (*get_failsafe)(bool *failsafe);
    bool (*get_actuator)(uint32_t *time_stamp, int16_t *moto_ch, int16_t *servo_ch);
} SrvDataHub_TypeDef;

extern SrvDataHub_TypeDef SrvDataHub;

#ifdef __cplusplus
}
#endif

#endif
