#ifndef __SRV_DATAHUB_H
#define __SRV_DATAHUB_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "DataPipe.h"
#include "pos_data.h"
#include "imu_data.h"
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
        uint32_t sensor_init : 1;           // bit : 0

        uint32_t sensor_data : 1;           // bit : 1

        uint32_t range_imu : 1;             // bit : 2
        uint32_t range_mag : 1;             // bit : 3
        uint32_t range_baro : 1;            // bit : 4
        uint32_t range_tof : 1;             // bit : 5
        uint32_t range_flow : 1;            // bit : 6
        
        uint32_t cnv_control_data : 1;      // bit : 7
        uint32_t rc_control_data : 1;       // bit : 8
        
        uint32_t actuator : 1;              // bit : 9
        uint32_t attitude : 1;              // bit : 10
        uint32_t relative_alt : 1;          // bit : 11

        uint32_t USB_VCP_attach : 1;        // bit : 12

        uint32_t cli : 1;                   // bit : 13
        uint32_t upgrade : 1;               // bit : 14
    } bit;

    uint32_t val;
} SrvDataHub_UpdateReg_TypeDef;

typedef struct
{
    int16_t gyro_range[2];
    int16_t acc_range[2];
    int16_t mag_range[2];
    int16_t baro_range[2];
    int16_t tof_range[2];

    uint32_t att_update_time;
    float att_roll;
    float att_pitch;
    float att_yaw;
    float att_q0;
    float att_q1;
    float att_q2;
    float att_q3;
    uint8_t att_error_code;

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

    uint32_t pos_update_time;
    double pos_x;
    double pos_y;
    double pos_z;

    double x_vel;
    double y_vel;
    double z_vel;

    uint32_t tof_update_time;
    float tof;

    uint32_t flow_update_time;
    float relative_pos_x;
    float relative_pos_y;
    float relative_vel_x;
    float relative_vel_y;

    uint32_t actuator_update_time;
    int16_t moto[2];
    int16_t servo[2];

    /* reserved */
    uint32_t tunning_heartbeat_timestamp;
    bool in_tunning;
    uint32_t tunning_port_addr;

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

    bool (*get_attitude)(uint32_t *time_stamp, float *pitch, float *roll, float *yaw, float *q0, float *q1, float *q2, float *q3);
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
