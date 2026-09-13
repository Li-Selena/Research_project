#ifndef __INS_TASK_H
#define __INS_TASK_H

#include <stdint.h>
#include "imu.h"

/* Robot-frame attitude: FLU, degrees; vectors are body-frame components. */
typedef struct
{
    float Pitch_Angle;
    float Yaw_Angle;
    float Yaw_TolAngle;
    float Roll_Angle;

    float Pitch_Gyro;
    float Yaw_Gyro;
    float Roll_Gyro;

    float Angle[3]; /* yaw, roll, pitch (deg), preserved public array order */
    float Gyro[3];
    float Accel[3];

    float Last_Yaw_Angle;
    int16_t YawRoundCount;
} INS_Info_Typedef;

typedef struct
{
    float x_m; /* world-frame X; yaw=0 aligns body +X/forward with world +X */
    float y_m;
    float yaw_rad;
    float yaw_total_rad;

    float vx_mps; /* world-frame velocity, not body-frame cmd_vel */
    float vy_mps;
    float wz_radps;

    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float yaw_total_deg;

    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
    float acc_x_g;
    float acc_y_g;
    float acc_z_g;

    uint8_t imu_online;
    uint32_t update_tick;
    uint32_t imu_update_tick;
    uint32_t odom_update_tick;
} INS_NavState_t;

typedef struct
{
    float x_m; /* world-frame X; yaw=0 aligns body +X/forward with world +X */
    float y_m;
    float vx_mps; /* world-frame velocity, not body-frame cmd_vel */
    float vy_mps;
    float wz_radps;
    uint32_t update_tick;
} INS_OdometryState_t;

typedef struct
{
    uint32_t tick_ms;
    uint32_t task_loop_count;
    uint32_t imu_snapshot_count;
    uint32_t imu_miss_count;

    uint8_t imu_online;
    uint8_t imu_data_online;
    uint8_t imu_update_flag;
    uint8_t has_new_imu;
    uint8_t yaw_total_initialized;
    uint8_t reserved[3];

    uint32_t imu_last_update_tick;
    uint32_t imu_age_ms;
    uint32_t nav_update_tick;
    uint32_t imu_update_tick;
    uint32_t odom_update_tick;
    uint32_t odom_age_ms;

    /* imu_* below is raw sensor-frame debug; nav_* is converted FLU. */
    float imu_acc_x_g;
    float imu_acc_y_g;
    float imu_acc_z_g;
    float imu_gyro_x_dps;
    float imu_gyro_y_dps;
    float imu_gyro_z_dps;
    float imu_roll_deg;
    float imu_pitch_deg;
    float imu_yaw_deg;

    float nav_x_m;
    float nav_y_m;
    float nav_yaw_rad;
    float nav_yaw_total_rad;
    float nav_vx_mps;
    float nav_vy_mps;
    float nav_wz_radps;
    float nav_roll_deg;
    float nav_pitch_deg;
    float nav_yaw_deg;
    float nav_yaw_total_deg;
    float nav_gyro_x_dps;
    float nav_gyro_y_dps;
    float nav_gyro_z_dps;
    float nav_acc_x_g;
    float nav_acc_y_g;
    float nav_acc_z_g;

    float odom_x_m;
    float odom_y_m;
    float odom_vx_mps;
    float odom_vy_mps;
    float odom_wz_radps;

    float ins_last_yaw_deg;
    int16_t yaw_round_count;
} INS_DebugState_t;

extern INS_Info_Typedef INS_Info;
extern INS_NavState_t g_ins_nav_state;
extern INS_OdometryState_t g_ins_odom_state;
extern INS_DebugState_t g_ins_debug_state;

void INS_Task(void const *argument);
void INS_Update_From_IMU(const IMU_Data_t *imu);
void INS_Update_Yaw_TotalAngle(void);
void INS_Update_NavState(void);
void INS_GetState(INS_NavState_t *out);
void INS_SetOdometry(float x_m,
                     float y_m,
                     float vx_mps,
                     float vy_mps,
                     float wz_radps);

#endif /* __INS_TASK_H */
