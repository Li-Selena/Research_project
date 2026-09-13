#ifndef __R2_YAW_AUTOTUNE_H
#define __R2_YAW_AUTOTUNE_H

#include <stdint.h>
#include "R2_move.h"

typedef enum
{
    R2_YAW_AUTOTUNE_IDLE    = 0,
    R2_YAW_AUTOTUNE_RUNNING = 1,
    R2_YAW_AUTOTUNE_DONE    = 2,
    R2_YAW_AUTOTUNE_FAILED  = 3,
    R2_YAW_AUTOTUNE_STOPPED = 4,
} R2_YawAutoTuneState_t;

typedef enum
{
    R2_YAW_AUTOTUNE_FAIL_NONE         = 0,
    R2_YAW_AUTOTUNE_FAIL_IMU_OFFLINE  = 1,
    R2_YAW_AUTOTUNE_FAIL_SOURCE       = 2,
    R2_YAW_AUTOTUNE_FAIL_MOTOR        = 3,
    R2_YAW_AUTOTUNE_FAIL_SETDIST      = 4,
    R2_YAW_AUTOTUNE_FAIL_SAFETY       = 5,
    R2_YAW_AUTOTUNE_FAIL_TIMEOUT      = 6,
    R2_YAW_AUTOTUNE_FAIL_BAD_ARG      = 7,
} R2_YawAutoTuneFail_t;

typedef struct
{
    uint8_t state;
    uint8_t segment_index;
    uint8_t segment_count;
    uint8_t pass_index;
    uint8_t pass_count;
    uint8_t fail_reason;
    uint8_t active_mode;
    uint8_t phase;

    uint32_t tick_ms;
    uint32_t segment_elapsed_ms;

    float yaw_error_deg;
    float yaw_error_abs_max_deg;
    float yaw_rate_error_rms_dps;
    float gyro_z_abs_max_dps;
    float wheel_rpm_abs_max;
    float score;
    float last_adjust;

    float angle_kp;
    float angle_ki;
    float angle_kd;
    float rate_kp;
    float rate_ki;
    float rate_kd;
    float pos_kp_yaw;
} R2_YawAutoTuneStatus_t;

void R2_YawAutoTune_Init(void);
uint8_t R2_YawAutoTune_Start(R2_Move_Ctrl_t *ctrl, uint8_t pass_count);
void R2_YawAutoTune_Stop(void);
void R2_YawAutoTune_Step(R2_Move_Ctrl_t *ctrl, uint32_t now_ms);
uint8_t R2_YawAutoTune_IsRunning(void);
void R2_YawAutoTune_GetStatus(R2_YawAutoTuneStatus_t *out);

#endif
