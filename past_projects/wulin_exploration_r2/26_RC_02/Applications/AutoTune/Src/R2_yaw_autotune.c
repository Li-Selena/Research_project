#include "R2_yaw_autotune.h"
#include "INS_Task.h"
#include "fdcan_receive.h"
#include "pid_user.h"
#include <math.h>
#include <string.h>

#define TUNE_RAD_TO_DEG             57.2957795f
#define TUNE_DEG_TO_RAD             0.0174532925f

#define TUNE_MAX_PASS_COUNT         3U
#define TUNE_SETTLE_MS              260U
#define TUNE_MOTOR_ONLINE_MIN_CNT   5U
#define TUNE_ROLL_PITCH_LIMIT_DEG   25.0f
#define TUNE_GYRO_LIMIT_DPS         500.0f
#define TUNE_YAW_ERR_LIMIT_DEG      90.0f
#define TUNE_NO_YAW_ERR_LIMIT_DEG   45.0f
#define TUNE_POS_TIMEOUT_EXTRA_MS   2500U

#define TUNE_ANGLE_KP_MIN           0.40f
#define TUNE_ANGLE_KP_MAX           5.00f
#define TUNE_ANGLE_KI_MIN           0.00f
#define TUNE_ANGLE_KI_MAX           0.05f
#define TUNE_ANGLE_KD_MIN           0.00f
#define TUNE_ANGLE_KD_MAX           0.50f
#define TUNE_RATE_KP_MIN            0.15f
#define TUNE_RATE_KP_MAX            2.50f
#define TUNE_RATE_KI_MIN            0.00f
#define TUNE_RATE_KI_MAX            0.10f
#define TUNE_RATE_KD_MIN            0.00f
#define TUNE_RATE_KD_MAX            1.00f
#define TUNE_POS_KP_YAW_MIN         0.60f
#define TUNE_POS_KP_YAW_MAX         4.00f

typedef enum
{
    TUNE_PHASE_IDLE   = 0,
    TUNE_PHASE_START  = 1,
    TUNE_PHASE_RUN    = 2,
    TUNE_PHASE_SETTLE = 3,
} TunePhase_t;

typedef struct
{
    R2_MoveMode_t mode;
    float vx;
    float vy;
    float vw;
    float dx;
    float dy;
    float dyaw;
    uint32_t run_ms;
    uint32_t timeout_ms;
} TuneSegment_t;

typedef struct
{
    uint8_t running;
    R2_YawAutoTuneStatus_t status;
    R2_Move_Ctrl_t *ctrl;
    TunePhase_t phase;
    uint32_t segment_start_ms;
    uint32_t settle_start_ms;
    float start_yaw_deg;
    float target_yaw_deg;
    float yaw_err_sum_abs;
    float rate_err_sum_sq;
    uint32_t sample_count;
    int8_t last_err_sign;
    uint8_t sign_changes;
} TuneContext_t;

/*
 * Closed nominal path inside a 2.4m x 3.0m observation area.
 * Segment X/Y values use the project FLU frame: X forward and Y left.
 * All eight R2 motion modes are visited once:
 *   - VEL modes exercise yaw hold/rate PID and wheel velocity PID.
 *   - POS modes exercise XY position loop, yaw position loop, yaw PID, and wheel PID.
 * Nominal end pose returns to the start: x=0, y=0, yaw=0.
 */
static const TuneSegment_t s_segments[] =
{
    {R2_MODE_ROBOT_NO_YAW_VEL, 0.35f, 0.00f, 0.00f, 0.00f, 0.00f, 0.000000f, 1800U, 3300U},
    {R2_MODE_WORLD_NO_YAW_VEL, 0.00f, -0.35f, 0.00f, 0.00f, 0.00f, 0.000000f, 1800U, 3300U},
    {R2_MODE_ROBOT_VEL, 0.00f, 0.00f, 0.45f, 0.00f, 0.00f, 0.000000f, 1400U, 3000U},
    {R2_MODE_WORLD_VEL, 0.25f, 0.25f, -0.45f, 0.00f, 0.00f, 0.000000f, 1400U, 3000U},
    {R2_MODE_ROBOT_NO_YAW_POS, 0.00f, 0.00f, 0.00f, 0.00f, 0.28f, 0.000000f, 0U, 6500U},
    {R2_MODE_WORLD_NO_YAW_POS, 0.00f, 0.00f, 0.00f, -0.68f, 0.00f, 0.000000f, 0U, 6500U},
    {R2_MODE_ROBOT_POS, 0.00f, 0.00f, 0.00f, -0.30f, -0.30f, 0.523599f, 0U, 7000U},
    {R2_MODE_WORLD_POS, 0.00f, 0.00f, 0.00f, 0.00f, 0.30f, -0.523599f, 0U, 7000U},
};

static TuneContext_t s_tune;

extern motor_measure_t motor_fdcan1[8];

static float TuneAbs(float x)
{
    return (x < 0.0f) ? -x : x;
}

static float TuneClamp(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static float TuneWrapDeg(float deg)
{
    while (deg > 180.0f) {
        deg -= 360.0f;
    }
    while (deg < -180.0f) {
        deg += 360.0f;
    }
    return deg;
}

static uint8_t TuneMotorOnline(void)
{
    uint8_t i;

    for (i = 0U; i < CHASSIS_MOTOR_COUNT; i++) {
        if (motor_fdcan1[i].msg_cnt < TUNE_MOTOR_ONLINE_MIN_CNT) {
            return 0U;
        }
    }

    return 1U;
}

static float TuneMaxWheelRpm(void)
{
    uint8_t i;
    float max_rpm = 0.0f;

    for (i = 0U; i < CHASSIS_MOTOR_COUNT; i++) {
        float rpm = TuneAbs((float)motor_fdcan1[i].speed_rpm);
        if (rpm > max_rpm) {
            max_rpm = rpm;
        }
    }

    return max_rpm;
}

static void TuneSnapshotParams(R2_Move_Ctrl_t *ctrl)
{
    ChassisYawPIDParam_t p;

    Chassis_Yaw_GetPIDParam(&p);
    s_tune.status.angle_kp = p.angle_kp;
    s_tune.status.angle_ki = p.angle_ki;
    s_tune.status.angle_kd = p.angle_kd;
    s_tune.status.rate_kp = p.rate_kp;
    s_tune.status.rate_ki = p.rate_ki;
    s_tune.status.rate_kd = p.rate_kd;
    s_tune.status.pos_kp_yaw = (ctrl != NULL) ? ctrl->pos_kp_yaw : 0.0f;
}

static void TuneStartFail(R2_Move_Ctrl_t *ctrl, R2_YawAutoTuneFail_t reason)
{
    if (ctrl != NULL) {
        R2_Move_Stop(ctrl);
    }

    memset(&s_tune, 0, sizeof(s_tune));
    s_tune.ctrl = ctrl;
    s_tune.phase = TUNE_PHASE_IDLE;
    s_tune.status.state = (uint8_t)R2_YAW_AUTOTUNE_FAILED;
    s_tune.status.fail_reason = (uint8_t)reason;
    s_tune.status.phase = (uint8_t)TUNE_PHASE_IDLE;
    s_tune.status.segment_count = (uint8_t)(sizeof(s_segments) / sizeof(s_segments[0]));
    TuneSnapshotParams(ctrl);
}

static void TuneFail(R2_YawAutoTuneFail_t reason)
{
    if (s_tune.ctrl != NULL) {
        R2_Move_Stop(s_tune.ctrl);
    }

    s_tune.running = 0U;
    s_tune.phase = TUNE_PHASE_IDLE;
    s_tune.status.state = (uint8_t)R2_YAW_AUTOTUNE_FAILED;
    s_tune.status.fail_reason = (uint8_t)reason;
    s_tune.status.phase = (uint8_t)TUNE_PHASE_IDLE;
    TuneSnapshotParams(s_tune.ctrl);
}

static void TuneResetMetrics(void)
{
    s_tune.yaw_err_sum_abs = 0.0f;
    s_tune.rate_err_sum_sq = 0.0f;
    s_tune.sample_count = 0U;
    s_tune.last_err_sign = 0;
    s_tune.sign_changes = 0U;
    s_tune.status.yaw_error_deg = 0.0f;
    s_tune.status.yaw_error_abs_max_deg = 0.0f;
    s_tune.status.yaw_rate_error_rms_dps = 0.0f;
    s_tune.status.gyro_z_abs_max_dps = 0.0f;
    s_tune.status.wheel_rpm_abs_max = 0.0f;
    s_tune.status.score = 0.0f;
    s_tune.status.last_adjust = 0.0f;
}

static uint32_t TuneElapsed(uint32_t now_ms, uint32_t start_ms)
{
    return now_ms - start_ms;
}

static const TuneSegment_t *TuneCurrentSegment(void)
{
    if (s_tune.status.segment_index >= s_tune.status.segment_count) {
        return NULL;
    }

    return &s_segments[s_tune.status.segment_index];
}

static void TuneSetTargetYaw(const TuneSegment_t *seg, const INS_NavState_t *nav)
{
    s_tune.start_yaw_deg = nav->yaw_total_deg;

    if (R2_Move_IsNoYawMode(seg->mode) != 0U) {
        s_tune.target_yaw_deg = s_tune.start_yaw_deg;
    } else if (R2_Move_IsPosMode(seg->mode) != 0U) {
        s_tune.target_yaw_deg = s_tune.start_yaw_deg + (seg->dyaw * TUNE_RAD_TO_DEG);
    } else {
        s_tune.target_yaw_deg = s_tune.start_yaw_deg;
    }
}

static uint8_t TuneStartSegment(R2_Move_Ctrl_t *ctrl, uint32_t now_ms)
{
    const TuneSegment_t *seg = TuneCurrentSegment();
    INS_NavState_t nav;

    if ((seg == NULL) || (ctrl == NULL)) {
        TuneFail(R2_YAW_AUTOTUNE_FAIL_BAD_ARG);
        return 0U;
    }

    INS_GetState(&nav);
    if (nav.imu_online == 0U) {
        TuneFail(R2_YAW_AUTOTUNE_FAIL_IMU_OFFLINE);
        return 0U;
    }
    if (TuneMotorOnline() == 0U) {
        TuneFail(R2_YAW_AUTOTUNE_FAIL_MOTOR);
        return 0U;
    }

    TuneResetMetrics();
    TuneSetTargetYaw(seg, &nav);
    R2_Move_Stop(ctrl);
    R2_Move_SetMode(ctrl, seg->mode);

    if (R2_Move_IsNoYawMode(seg->mode) != 0U) {
        if (R2_Move_IsWorldMode(seg->mode) != 0U) {
            R2_Move_SetWorldLockYaw(ctrl,
                                    s_tune.target_yaw_deg * TUNE_DEG_TO_RAD);
        } else {
            R2_Move_SetRobotLockYaw(ctrl,
                                    TuneWrapDeg(s_tune.target_yaw_deg -
                                                s_tune.start_yaw_deg) *
                                    TUNE_DEG_TO_RAD);
        }
    }

    if (R2_Move_IsVelMode(seg->mode) != 0U) {
        R2_Move_SetVel(ctrl, seg->vx, seg->vy, seg->vw);
    } else {
        if (R2_Move_SetDist(ctrl, seg->dx, seg->dy, seg->dyaw) != 0) {
            TuneFail(R2_YAW_AUTOTUNE_FAIL_SETDIST);
            return 0U;
        }
    }

    s_tune.segment_start_ms = now_ms;
    s_tune.settle_start_ms = 0U;
    s_tune.phase = TUNE_PHASE_RUN;
    s_tune.status.phase = (uint8_t)s_tune.phase;
    s_tune.status.active_mode = (uint8_t)seg->mode;
    s_tune.status.segment_elapsed_ms = 0U;
    TuneSnapshotParams(ctrl);

    return 1U;
}

static void TuneSample(const TuneSegment_t *seg, const INS_NavState_t *nav)
{
    float yaw_error;
    float abs_yaw_error;
    float target_rate_dps;
    float actual_rate_dps;
    float rate_error;
    float abs_gyro;
    float wheel_rpm;
    int8_t err_sign = 0;

    if ((seg == NULL) || (nav == NULL)) {
        return;
    }

    if ((R2_Move_IsNoYawMode(seg->mode) != 0U) || (R2_Move_IsPosMode(seg->mode) != 0U)) {
        yaw_error = TuneWrapDeg(nav->yaw_total_deg - s_tune.target_yaw_deg);
    } else {
        yaw_error = TuneWrapDeg(nav->yaw_total_deg - s_tune.start_yaw_deg);
    }

    target_rate_dps = (R2_Move_IsVelMode(seg->mode) != 0U) ? (seg->vw * TUNE_RAD_TO_DEG) : 0.0f;
    actual_rate_dps = nav->gyro_z_dps;
    rate_error = target_rate_dps - actual_rate_dps;

    abs_yaw_error = TuneAbs(yaw_error);
    abs_gyro = TuneAbs(nav->gyro_z_dps);
    wheel_rpm = TuneMaxWheelRpm();

    s_tune.yaw_err_sum_abs += abs_yaw_error;
    s_tune.rate_err_sum_sq += rate_error * rate_error;
    s_tune.sample_count++;

    if (yaw_error > 0.4f) {
        err_sign = 1;
    } else if (yaw_error < -0.4f) {
        err_sign = -1;
    }

    if ((err_sign != 0) && (s_tune.last_err_sign != 0) &&
        (err_sign != s_tune.last_err_sign) && (s_tune.sign_changes < 255U)) {
        s_tune.sign_changes++;
    }
    if (err_sign != 0) {
        s_tune.last_err_sign = err_sign;
    }

    s_tune.status.yaw_error_deg = yaw_error;
    if (abs_yaw_error > s_tune.status.yaw_error_abs_max_deg) {
        s_tune.status.yaw_error_abs_max_deg = abs_yaw_error;
    }
    if (abs_gyro > s_tune.status.gyro_z_abs_max_dps) {
        s_tune.status.gyro_z_abs_max_dps = abs_gyro;
    }
    if (wheel_rpm > s_tune.status.wheel_rpm_abs_max) {
        s_tune.status.wheel_rpm_abs_max = wheel_rpm;
    }
    if (s_tune.sample_count != 0U) {
        s_tune.status.yaw_rate_error_rms_dps =
            sqrtf(s_tune.rate_err_sum_sq / (float)s_tune.sample_count);
    }
}

static uint8_t TuneSafetyOk(const TuneSegment_t *seg, const INS_NavState_t *nav)
{
    float yaw_limit = TUNE_YAW_ERR_LIMIT_DEG;

    if (nav->imu_online == 0U) {
        TuneFail(R2_YAW_AUTOTUNE_FAIL_IMU_OFFLINE);
        return 0U;
    }
    if (TuneMotorOnline() == 0U) {
        TuneFail(R2_YAW_AUTOTUNE_FAIL_MOTOR);
        return 0U;
    }
    if ((TuneAbs(nav->roll_deg) > TUNE_ROLL_PITCH_LIMIT_DEG) ||
        (TuneAbs(nav->pitch_deg) > TUNE_ROLL_PITCH_LIMIT_DEG) ||
        (TuneAbs(nav->gyro_z_dps) > TUNE_GYRO_LIMIT_DPS)) {
        TuneFail(R2_YAW_AUTOTUNE_FAIL_SAFETY);
        return 0U;
    }

    if ((seg != NULL) && (R2_Move_IsNoYawMode(seg->mode) != 0U)) {
        yaw_limit = TUNE_NO_YAW_ERR_LIMIT_DEG;
    }
    if (s_tune.status.yaw_error_abs_max_deg > yaw_limit) {
        TuneFail(R2_YAW_AUTOTUNE_FAIL_SAFETY);
        return 0U;
    }

    return 1U;
}

static void TuneApplyAdjustment(R2_Move_Ctrl_t *ctrl, const TuneSegment_t *seg)
{
    ChassisYawPIDParam_t p;
    float final_abs;
    float max_abs;
    float rate_rms;
    float adjust = 0.0f;

    if ((ctrl == NULL) || (seg == NULL)) {
        return;
    }

    Chassis_Yaw_GetPIDParam(&p);

    final_abs = TuneAbs(s_tune.status.yaw_error_deg);
    max_abs = s_tune.status.yaw_error_abs_max_deg;
    rate_rms = s_tune.status.yaw_rate_error_rms_dps;

    if (R2_Move_IsNoYawMode(seg->mode) != 0U) {
        if ((max_abs > 3.0f) || (final_abs > 1.5f)) {
            p.angle_kp += 0.05f;
            p.rate_kp += 0.015f;
            adjust += 0.065f;
        }
        if ((s_tune.sign_changes > 3U) && (max_abs > 2.0f)) {
            p.angle_kp -= 0.04f;
            p.rate_kd += 0.02f;
            adjust -= 0.02f;
        }
        if ((final_abs > 1.0f) && (s_tune.sign_changes <= 1U)) {
            p.rate_ki += 0.001f;
            adjust += 0.001f;
        }
    } else if (R2_Move_IsVelMode(seg->mode) != 0U) {
        if (rate_rms > 12.0f) {
            p.rate_kp += 0.03f;
            adjust += 0.03f;
        }
        if ((s_tune.sign_changes > 4U) ||
            (s_tune.status.gyro_z_abs_max_dps > (TuneAbs(seg->vw * TUNE_RAD_TO_DEG) + 120.0f))) {
            p.rate_kp -= 0.02f;
            p.rate_kd += 0.02f;
        }
    } else {
        if (final_abs > 2.5f) {
            ctrl->pos_kp_yaw += 0.05f;
            p.rate_kp += 0.01f;
            adjust += 0.06f;
        }
        if ((max_abs > 8.0f) || (s_tune.sign_changes > 2U)) {
            ctrl->pos_kp_yaw -= 0.04f;
            p.rate_kd += 0.02f;
            adjust -= 0.02f;
        }
    }

    p.angle_kp = TuneClamp(p.angle_kp, TUNE_ANGLE_KP_MIN, TUNE_ANGLE_KP_MAX);
    p.angle_ki = TuneClamp(p.angle_ki, TUNE_ANGLE_KI_MIN, TUNE_ANGLE_KI_MAX);
    p.angle_kd = TuneClamp(p.angle_kd, TUNE_ANGLE_KD_MIN, TUNE_ANGLE_KD_MAX);
    p.rate_kp  = TuneClamp(p.rate_kp,  TUNE_RATE_KP_MIN,  TUNE_RATE_KP_MAX);
    p.rate_ki  = TuneClamp(p.rate_ki,  TUNE_RATE_KI_MIN,  TUNE_RATE_KI_MAX);
    p.rate_kd  = TuneClamp(p.rate_kd,  TUNE_RATE_KD_MIN,  TUNE_RATE_KD_MAX);
    ctrl->pos_kp_yaw = TuneClamp(ctrl->pos_kp_yaw, TUNE_POS_KP_YAW_MIN, TUNE_POS_KP_YAW_MAX);

    Chassis_Yaw_SetPIDParam(&p);
    s_tune.status.score = max_abs + 0.05f * rate_rms + (0.5f * (float)s_tune.sign_changes);
    s_tune.status.last_adjust = adjust;
    TuneSnapshotParams(ctrl);
}

static void TuneAdvance(R2_Move_Ctrl_t *ctrl, uint32_t now_ms)
{
    const TuneSegment_t *seg = TuneCurrentSegment();

    TuneApplyAdjustment(ctrl, seg);
    R2_Move_Stop(ctrl);

    s_tune.status.segment_index++;
    if (s_tune.status.segment_index >= s_tune.status.segment_count) {
        s_tune.status.segment_index = 0U;
        s_tune.status.pass_index++;
    }

    if (s_tune.status.pass_index >= s_tune.status.pass_count) {
        s_tune.running = 0U;
        s_tune.phase = TUNE_PHASE_IDLE;
        s_tune.status.state = (uint8_t)R2_YAW_AUTOTUNE_DONE;
        s_tune.status.phase = (uint8_t)TUNE_PHASE_IDLE;
        s_tune.status.tick_ms = now_ms;
        return;
    }

    s_tune.phase = TUNE_PHASE_START;
    s_tune.status.phase = (uint8_t)s_tune.phase;
}

void R2_YawAutoTune_Init(void)
{
    memset(&s_tune, 0, sizeof(s_tune));
    s_tune.status.state = (uint8_t)R2_YAW_AUTOTUNE_IDLE;
    s_tune.status.segment_count = (uint8_t)(sizeof(s_segments) / sizeof(s_segments[0]));
}

uint8_t R2_YawAutoTune_Start(R2_Move_Ctrl_t *ctrl, uint8_t pass_count)
{
    INS_NavState_t nav;

    if (ctrl == NULL) {
        TuneStartFail(NULL, R2_YAW_AUTOTUNE_FAIL_BAD_ARG);
        return 0U;
    }

    INS_GetState(&nav);
    if (nav.imu_online == 0U) {
        TuneStartFail(ctrl, R2_YAW_AUTOTUNE_FAIL_IMU_OFFLINE);
        return 0U;
    }
    if (TuneMotorOnline() == 0U) {
        TuneStartFail(ctrl, R2_YAW_AUTOTUNE_FAIL_MOTOR);
        return 0U;
    }

    if (pass_count == 0U) {
        pass_count = 1U;
    } else if (pass_count > TUNE_MAX_PASS_COUNT) {
        pass_count = TUNE_MAX_PASS_COUNT;
    }

    memset(&s_tune, 0, sizeof(s_tune));
    s_tune.running = 1U;
    s_tune.ctrl = ctrl;
    s_tune.phase = TUNE_PHASE_START;
    s_tune.status.state = (uint8_t)R2_YAW_AUTOTUNE_RUNNING;
    s_tune.status.segment_count = (uint8_t)(sizeof(s_segments) / sizeof(s_segments[0]));
    s_tune.status.pass_count = pass_count;
    s_tune.status.fail_reason = (uint8_t)R2_YAW_AUTOTUNE_FAIL_NONE;
    s_tune.status.phase = (uint8_t)s_tune.phase;
    TuneSnapshotParams(ctrl);
    R2_Move_Stop(ctrl);

    return 1U;
}

void R2_YawAutoTune_Stop(void)
{
    if (s_tune.ctrl != NULL) {
        R2_Move_Stop(s_tune.ctrl);
    }

    s_tune.running = 0U;
    s_tune.phase = TUNE_PHASE_IDLE;
    s_tune.status.state = (uint8_t)R2_YAW_AUTOTUNE_STOPPED;
    s_tune.status.phase = (uint8_t)TUNE_PHASE_IDLE;
    TuneSnapshotParams(s_tune.ctrl);
}

void R2_YawAutoTune_Step(R2_Move_Ctrl_t *ctrl, uint32_t now_ms)
{
    const TuneSegment_t *seg;
    INS_NavState_t nav;
    uint32_t elapsed;

    if ((s_tune.running == 0U) || (s_tune.status.state != (uint8_t)R2_YAW_AUTOTUNE_RUNNING)) {
        return;
    }

    if ((ctrl == NULL) || (ctrl != s_tune.ctrl)) {
        TuneFail(R2_YAW_AUTOTUNE_FAIL_SOURCE);
        return;
    }

    s_tune.status.tick_ms = now_ms;

    if (s_tune.phase == TUNE_PHASE_START) {
        (void)TuneStartSegment(ctrl, now_ms);
        return;
    }

    seg = TuneCurrentSegment();
    if (seg == NULL) {
        TuneFail(R2_YAW_AUTOTUNE_FAIL_BAD_ARG);
        return;
    }

    INS_GetState(&nav);
    TuneSample(seg, &nav);
    if (TuneSafetyOk(seg, &nav) == 0U) {
        return;
    }

    elapsed = TuneElapsed(now_ms, s_tune.segment_start_ms);
    s_tune.status.segment_elapsed_ms = elapsed;

    if (s_tune.phase == TUNE_PHASE_RUN) {
        if (R2_Move_IsVelMode(seg->mode) != 0U) {
            if (elapsed >= seg->run_ms) {
                R2_Move_SetVel(ctrl, 0.0f, 0.0f, 0.0f);
                s_tune.settle_start_ms = now_ms;
                s_tune.phase = TUNE_PHASE_SETTLE;
                s_tune.status.phase = (uint8_t)s_tune.phase;
            }
        } else {
            if (R2_Move_GetPosState(ctrl) == R2_POS_DONE) {
                s_tune.settle_start_ms = now_ms;
                s_tune.phase = TUNE_PHASE_SETTLE;
                s_tune.status.phase = (uint8_t)s_tune.phase;
            } else if (elapsed > (seg->timeout_ms + TUNE_POS_TIMEOUT_EXTRA_MS)) {
                TuneFail(R2_YAW_AUTOTUNE_FAIL_TIMEOUT);
            }
        }
    } else if (s_tune.phase == TUNE_PHASE_SETTLE) {
        if (TuneElapsed(now_ms, s_tune.settle_start_ms) >= TUNE_SETTLE_MS) {
            TuneAdvance(ctrl, now_ms);
        }
    }

    if (elapsed > seg->timeout_ms) {
        if ((R2_Move_IsVelMode(seg->mode) != 0U) && (s_tune.phase == TUNE_PHASE_RUN)) {
            TuneFail(R2_YAW_AUTOTUNE_FAIL_TIMEOUT);
        }
    }
}

uint8_t R2_YawAutoTune_IsRunning(void)
{
    return s_tune.running;
}

void R2_YawAutoTune_GetStatus(R2_YawAutoTuneStatus_t *out)
{
    if (out == NULL) {
        return;
    }

    *out = s_tune.status;
}
