#include "R2_climb.h"
#include "fdcan_receive.h"
#include "pid_user.h"
#include "R2_laser_user.h"
#include <string.h>

extern motor_measure_t motor_fdcan2[8];
extern motor_measure_t motor_fdcan1[8];

#define LEG_FRONT_RIGHT_IDX 0U
#define LEG_REAR_RIGHT_IDX  1U
#define LEG_REAR_LEFT_IDX   2U
#define LEG_FRONT_LEFT_IDX  3U

#define R2_CLIMB_FRONT_LEG_MASK \
    ((uint8_t)((1U << LEG_FRONT_RIGHT_IDX) | (1U << LEG_FRONT_LEFT_IDX)))
#define R2_CLIMB_REAR_LEG_MASK \
    ((uint8_t)((1U << LEG_REAR_RIGHT_IDX) | (1U << LEG_REAR_LEFT_IDX)))

#define DRIVE_LEFT_IDX      0U
#define DRIVE_RIGHT_IDX     1U

#define R2_CLIMB_DRIVE_GROUP_MASK \
    ((uint8_t)(R2_CLIMB_DRIVE_GROUP_FRONT | R2_CLIMB_DRIVE_GROUP_REAR))

static const float s_leg_dir[4] = {
    R2_CLIMB_LEG1_DIR,
    R2_CLIMB_LEG2_DIR,
    R2_CLIMB_LEG3_DIR,
    R2_CLIMB_LEG4_DIR,
};

static const float s_drive_dir[2] = {
    R2_CLIMB_DRIVE_LEFT_DIR,
    R2_CLIMB_DRIVE_RIGHT_DIR,
};

typedef enum
{
    R2_CLIMB_MAIN_STEP_LEG_TARGET = 0,
    R2_CLIMB_MAIN_STEP_DRIVE_DELTA,
    R2_CLIMB_MAIN_STEP_CHASSIS_DELTA,
} R2_ClimbMainStepKind_t;

typedef struct
{
    const char *name;
    R2_ClimbMainStepKind_t kind;
    uint8_t leg_mask;
    uint8_t drive_group_mask;
    float leg_target_mm[4];
    float delta;
    uint32_t timeout_ms;
} R2_ClimbMainStep_t;

/* Drive deltas below are wheel travel in mm. The four climb drive wheels are
 * 1:1 from motor output shaft to wheel and share the same left/right targets.
 */
static const R2_ClimbMainStep_t s_main_steps[R2_CLIMB_MAIN_STEP_COUNT] = {
    {"STEP_01_ALL_LEGS_220", R2_CLIMB_MAIN_STEP_LEG_TARGET, 0x0FU, R2_CLIMB_DRIVE_GROUP_ALL, {220.0f, 220.0f, 220.0f, 220.0f}, 0.0f, R2_CLIMB_STEP_LONG_LEG_TIMEOUT_MS},
    {"STEP_02_ALL_DRIVE_FORWARD_30_X3", R2_CLIMB_MAIN_STEP_DRIVE_DELTA, 0x00U, R2_CLIMB_DRIVE_GROUP_ALL, {220.0f, 220.0f, 220.0f, 220.0f}, 90.0f, R2_CLIMB_DRIVE_TIMEOUT_MS(90.0f)},
    {"STEP_03_ALL_DRIVE_FORWARD_10_X3", R2_CLIMB_MAIN_STEP_DRIVE_DELTA, 0x00U, R2_CLIMB_DRIVE_GROUP_ALL, {220.0f, 220.0f, 220.0f, 220.0f}, 30.0f, R2_CLIMB_DRIVE_TIMEOUT_MS(30.0f)},
    {"STEP_04_ALL_LEGS_DOWN_10_X5", R2_CLIMB_MAIN_STEP_LEG_TARGET, 0x0FU, R2_CLIMB_DRIVE_GROUP_ALL, {170.0f, 170.0f, 170.0f, 170.0f}, 0.0f, R2_CLIMB_LEG_TIMEOUT_MS(50.0f)},
    {"STEP_05_FRONT_MINUS_30", R2_CLIMB_MAIN_STEP_LEG_TARGET, R2_CLIMB_FRONT_LEG_MASK, R2_CLIMB_DRIVE_GROUP_ALL, {-30.0f, 170.0f, 170.0f, -30.0f}, 0.0f, R2_CLIMB_LEG_TIMEOUT_MS(200.0f)},
    {"STEP_06_REAR_DRIVE_FORWARD_30_X4", R2_CLIMB_MAIN_STEP_DRIVE_DELTA, 0x00U, R2_CLIMB_DRIVE_GROUP_REAR, {-30.0f, 170.0f, 170.0f, -30.0f}, 120.0f, R2_CLIMB_DRIVE_TIMEOUT_MS(120.0f)},
    {"STEP_07_FRONT_UP_10_X2", R2_CLIMB_MAIN_STEP_LEG_TARGET, R2_CLIMB_FRONT_LEG_MASK, R2_CLIMB_DRIVE_GROUP_REAR, {-10.0f, 170.0f, 170.0f, -10.0f}, 0.0f, R2_CLIMB_STEP_LEG_20_TIMEOUT_MS},
    {"STEP_08_ALL_LEGS_UP_10_X6", R2_CLIMB_MAIN_STEP_LEG_TARGET, 0x0FU, R2_CLIMB_DRIVE_GROUP_REAR, {50.0f, 230.0f, 230.0f, 50.0f}, 0.0f, R2_CLIMB_LEG_TIMEOUT_MS(60.0f)},
    {"STEP_09_ALL_DRIVE_FORWARD_200_PAUSE", R2_CLIMB_MAIN_STEP_DRIVE_DELTA, 0x00U, R2_CLIMB_DRIVE_GROUP_ALL, {50.0f, 230.0f, 230.0f, 50.0f}, 200.0f, R2_CLIMB_DRIVE_TIMEOUT_MS(200.0f)},
    {"STEP_10_ALL_DRIVE_FORWARD_300_RESUME", R2_CLIMB_MAIN_STEP_DRIVE_DELTA, 0x00U, R2_CLIMB_DRIVE_GROUP_ALL, {50.0f, 230.0f, 230.0f, 50.0f}, 300.0f, R2_CLIMB_DRIVE_TIMEOUT_MS(300.0f)},
    {"STEP_11_ALL_DRIVE_FORWARD_30_X9", R2_CLIMB_MAIN_STEP_DRIVE_DELTA, 0x00U, R2_CLIMB_DRIVE_GROUP_ALL, {50.0f, 230.0f, 230.0f, 50.0f}, 270.0f, R2_CLIMB_DRIVE_TIMEOUT_MS(270.0f)},
    {"STEP_12_ALL_LEGS_ZERO", R2_CLIMB_MAIN_STEP_LEG_TARGET, 0x0FU, R2_CLIMB_DRIVE_GROUP_ALL, {0.0f, 0.0f, 0.0f, 0.0f}, 0.0f, R2_CLIMB_STEP_LONG_LEG_TIMEOUT_MS},
    {"STEP_13_ALL_LEGS_DOWN_10_X2", R2_CLIMB_MAIN_STEP_LEG_TARGET, 0x0FU, R2_CLIMB_DRIVE_GROUP_ALL, {-20.0f, -20.0f, -20.0f, -20.0f}, 0.0f, R2_CLIMB_STEP_LEG_20_TIMEOUT_MS},
    {"STEP_14_CHASSIS_FORWARD_100_X2", R2_CLIMB_MAIN_STEP_CHASSIS_DELTA, 0x00U, R2_CLIMB_DRIVE_GROUP_ALL, {-20.0f, -20.0f, -20.0f, -20.0f}, 0.20f, R2_CLIMB_STEP_CHASSIS_200_TIMEOUT_MS},
};

static const R2_ClimbMainStep_t s_downstairs_steps[R2_CLIMB_DOWNSTAIRS_STEP_COUNT] = {
    {"DOWN_01_FRONT_UP_10_X23", R2_CLIMB_MAIN_STEP_LEG_TARGET, R2_CLIMB_FRONT_LEG_MASK, R2_CLIMB_DRIVE_GROUP_ALL, {200.0f, -30.0f, -30.0f, 200.0f}, 0.0f, R2_CLIMB_LEG_TIMEOUT_MS(230.0f)},
    {"DOWN_02_REAR_ZERO", R2_CLIMB_MAIN_STEP_LEG_TARGET, R2_CLIMB_REAR_LEG_MASK, R2_CLIMB_DRIVE_GROUP_ALL, {200.0f, 0.0f, 0.0f, 200.0f}, 0.0f, R2_CLIMB_STEP_LEG_30_TIMEOUT_MS},
    {"DOWN_03_REAR_UP_10", R2_CLIMB_MAIN_STEP_LEG_TARGET, R2_CLIMB_REAR_LEG_MASK, R2_CLIMB_DRIVE_GROUP_ALL, {200.0f, 10.0f, 10.0f, 200.0f}, 0.0f, R2_CLIMB_STEP_LEG_10_TIMEOUT_MS},
    {"DOWN_04_ALL_LEGS_UP_10", R2_CLIMB_MAIN_STEP_LEG_TARGET, 0x0FU, R2_CLIMB_DRIVE_GROUP_ALL, {210.0f, 20.0f, 20.0f, 210.0f}, 0.0f, R2_CLIMB_STEP_LEG_10_TIMEOUT_MS},
    {"DOWN_05_ALL_DRIVE_FORWARD_500", R2_CLIMB_MAIN_STEP_DRIVE_DELTA, 0x00U, R2_CLIMB_DRIVE_GROUP_ALL, {210.0f, 20.0f, 20.0f, 210.0f}, 500.0f, R2_CLIMB_DRIVE_TIMEOUT_MS(500.0f)},
    {"DOWN_06_ALL_DRIVE_FORWARD_30_X11", R2_CLIMB_MAIN_STEP_DRIVE_DELTA, 0x00U, R2_CLIMB_DRIVE_GROUP_ALL, {210.0f, 20.0f, 20.0f, 210.0f}, 330.0f, R2_CLIMB_DRIVE_TIMEOUT_MS(330.0f)},
    {"DOWN_07_ALL_DRIVE_FORWARD_10_X2", R2_CLIMB_MAIN_STEP_DRIVE_DELTA, 0x00U, R2_CLIMB_DRIVE_GROUP_ALL, {210.0f, 20.0f, 20.0f, 210.0f}, 20.0f, R2_CLIMB_DRIVE_TIMEOUT_MS(20.0f)},
    {"DOWN_08_ALL_LEGS_DOWN_10_X6", R2_CLIMB_MAIN_STEP_LEG_TARGET, 0x0FU, R2_CLIMB_DRIVE_GROUP_ALL, {150.0f, -30.0f, -30.0f, 150.0f}, 0.0f, R2_CLIMB_LEG_TIMEOUT_MS(60.0f)},
    {"DOWN_09_ALL_DRIVE_FORWARD_30_X3", R2_CLIMB_MAIN_STEP_DRIVE_DELTA, 0x00U, R2_CLIMB_DRIVE_GROUP_ALL, {150.0f, -30.0f, -30.0f, 150.0f}, 90.0f, R2_CLIMB_DRIVE_TIMEOUT_MS(90.0f)},
    {"DOWN_10_ALL_DRIVE_FORWARD_10", R2_CLIMB_MAIN_STEP_DRIVE_DELTA, 0x00U, R2_CLIMB_DRIVE_GROUP_ALL, {150.0f, -30.0f, -30.0f, 150.0f}, 10.0f, R2_CLIMB_DRIVE_TIMEOUT_MS(10.0f)},
    {"DOWN_11_REAR_UP_10_X19", R2_CLIMB_MAIN_STEP_LEG_TARGET, R2_CLIMB_REAR_LEG_MASK, R2_CLIMB_DRIVE_GROUP_ALL, {150.0f, 160.0f, 160.0f, 150.0f}, 0.0f, R2_CLIMB_LEG_TIMEOUT_MS(190.0f)},
    {"DOWN_12_ALL_DRIVE_FORWARD_30_X4", R2_CLIMB_MAIN_STEP_DRIVE_DELTA, 0x00U, R2_CLIMB_DRIVE_GROUP_ALL, {150.0f, 160.0f, 160.0f, 150.0f}, 120.0f, R2_CLIMB_DRIVE_TIMEOUT_MS(120.0f)},
    {"DOWN_13_ALL_LEGS_ZERO", R2_CLIMB_MAIN_STEP_LEG_TARGET, 0x0FU, R2_CLIMB_DRIVE_GROUP_ALL, {0.0f, 0.0f, 0.0f, 0.0f}, 0.0f, R2_CLIMB_STEP_LONG_LEG_TIMEOUT_MS},
    {"DOWN_14_ALL_LEGS_DOWN_10_X3", R2_CLIMB_MAIN_STEP_LEG_TARGET, 0x0FU, R2_CLIMB_DRIVE_GROUP_ALL, {-30.0f, -30.0f, -30.0f, -30.0f}, 0.0f, R2_CLIMB_STEP_LEG_30_TIMEOUT_MS},
    {"DOWN_15_CHASSIS_FORWARD_100", R2_CLIMB_MAIN_STEP_CHASSIS_DELTA, 0x00U, R2_CLIMB_DRIVE_GROUP_ALL, {-30.0f, -30.0f, -30.0f, -30.0f}, 0.10f, R2_CLIMB_STEP_CHASSIS_100_TIMEOUT_MS},
};

static uint8_t NormalizeFlow(uint8_t flow)
{
    return (flow == (uint8_t)R2_CLIMB_FLOW_DOWNSTAIRS) ?
           (uint8_t)R2_CLIMB_FLOW_DOWNSTAIRS :
           (uint8_t)R2_CLIMB_FLOW_UPSTAIRS;
}

static const R2_ClimbMainStep_t *FlowSteps(uint8_t flow)
{
    if (NormalizeFlow(flow) == (uint8_t)R2_CLIMB_FLOW_DOWNSTAIRS) {
        return s_downstairs_steps;
    }

    return s_main_steps;
}

static uint32_t FlowStepCount(uint8_t flow)
{
    if (NormalizeFlow(flow) == (uint8_t)R2_CLIMB_FLOW_DOWNSTAIRS) {
        return R2_CLIMB_DOWNSTAIRS_STEP_COUNT;
    }

    return R2_CLIMB_MAIN_STEP_COUNT;
}

static const R2_ClimbMainStep_t *CtrlSteps(const R2_Climb_Ctrl_t *ctrl)
{
    return FlowSteps((ctrl != 0) ? ctrl->flow : (uint8_t)R2_CLIMB_FLOW_UPSTAIRS);
}

static uint32_t CtrlStepCount(const R2_Climb_Ctrl_t *ctrl)
{
    return FlowStepCount((ctrl != 0) ? ctrl->flow : (uint8_t)R2_CLIMB_FLOW_UPSTAIRS);
}

volatile R2_ClimbDebug_t g_r2_climb_debug_usart;
volatile R2_ClimbDebug_t g_r2_climb_debug_usb;
volatile R2_ClimbDebug_t g_r2_climb_debug_active;

static void UpdatePositions(R2_Climb_Ctrl_t *ctrl);
static const char *StateName(uint8_t flow, R2_ClimbState_t state);
static void BuildDebugView(const R2_Climb_Ctrl_t *ctrl,
                           uint8_t source,
                           const volatile R2_ClimbDebug_t *prev,
                           R2_ClimbDebug_t *view);
static void StoreDebugCurrent(volatile R2_ClimbDebug_t *view,
                              const R2_ClimbMotorCmd_t *cmd);
static uint8_t LegsReached(const R2_Climb_Ctrl_t *ctrl, uint8_t mask);
static uint8_t DrivesReached(const R2_Climb_Ctrl_t *ctrl);
static uint8_t StateUsesChassis(const R2_Climb_Ctrl_t *ctrl,
                                R2_ClimbState_t state);
static uint8_t ShouldPauseAfterState(const R2_Climb_Ctrl_t *ctrl,
                                     R2_ClimbState_t state);
static uint8_t UpLaserXReady(const R2_LaserMeasure_t *laser);
static uint8_t UpLaserTriggerReached(void);
static uint8_t UpLaserXInvalidTimedOut(R2_Climb_Ctrl_t *ctrl,
                                       uint32_t now_ms);
static uint8_t DownLaserHeightReady(const R2_LaserMeasure_t *laser);
static uint8_t DownLaserTriggerReached(void);
static uint8_t DownLaserHeightInvalidTimedOut(R2_Climb_Ctrl_t *ctrl,
                                              uint32_t now_ms);

static float AbsF(float x)
{
    return (x < 0.0f) ? -x : x;
}

static float SafeScale(float scale)
{
    return (scale > 0.001f) ? scale : 1.0f;
}

static uint8_t ParamsReady(void)
{
    if (R2_CLIMB_PARAM_CONFIGURED == 0U) {
        return 0U;
    }

    if (R2_CLIMB_LEG_COUNT_PER_MM <= 0.001f) {
        return 0U;
    }

    if (R2_CLIMB_DRIVE_COUNT_PER_MM <= 0.001f) {
        return 0U;
    }

    return 1U;
}

static uint8_t UpLaserXReady(const R2_LaserMeasure_t *laser)
{
    if (laser == 0) {
        return 0U;
    }

    if ((laser->x_pos_valid == 0U) || (laser->x_pos_online == 0U)) {
        return 0U;
    }

    if (laser->x_pos_mm < R2_CLIMB_UP_TRIGGER_X_MIN_MM) {
        return 0U;
    }

    return 1U;
}

static uint8_t UpLaserTriggerReached(void)
{
    R2_LaserMeasure_t laser;

    R2_LaserUser_GetMeasure(&laser);
    if (UpLaserXReady(&laser) == 0U) {
        return 0U;
    }

    if (laser.x_pos_mm >= R2_CLIMB_UP_TRIGGER_X_MAX_MM) {
        return 0U;
    }

    return 1U;
}

static uint8_t UpLaserXInvalidTimedOut(R2_Climb_Ctrl_t *ctrl,
                                       uint32_t now_ms)
{
    R2_LaserMeasure_t laser;

    if (ctrl == 0) {
        return 1U;
    }

    R2_LaserUser_GetMeasure(&laser);
    if (UpLaserXReady(&laser) != 0U) {
        ctrl->up_laser_invalid_active = 0U;
        ctrl->up_laser_invalid_start_ms = 0U;
        return 0U;
    }

    if (ctrl->up_laser_invalid_active == 0U) {
        ctrl->up_laser_invalid_active = 1U;
        ctrl->up_laser_invalid_start_ms = now_ms;
        return 0U;
    }

    return ((now_ms - ctrl->up_laser_invalid_start_ms) >
            R2_CLIMB_UP_LASER_INVALID_TIMEOUT_MS) ? 1U : 0U;
}

static uint8_t DownLaserHeightReady(const R2_LaserMeasure_t *laser)
{
    if (laser == 0) {
        return 0U;
    }

    if ((laser->height_valid == 0U) || (laser->height_online == 0U)) {
        return 0U;
    }

    if (laser->height_mm < 0) {
        return 0U;
    }

    return 1U;
}

static uint8_t DownLaserTriggerReached(void)
{
    R2_LaserMeasure_t laser;

    R2_LaserUser_GetMeasure(&laser);
    if (DownLaserHeightReady(&laser) == 0U) {
        return 0U;
    }

    return (laser.height_mm > R2_CLIMB_DOWN_TRIGGER_HEIGHT_MIN_MM) ? 1U : 0U;
}

static uint8_t DownLaserHeightInvalidTimedOut(R2_Climb_Ctrl_t *ctrl,
                                              uint32_t now_ms)
{
    R2_LaserMeasure_t laser;

    if (ctrl == 0) {
        return 1U;
    }

    R2_LaserUser_GetMeasure(&laser);
    if (DownLaserHeightReady(&laser) != 0U) {
        ctrl->down_laser_invalid_active = 0U;
        ctrl->down_laser_invalid_start_ms = 0U;
        return 0U;
    }

    if (ctrl->down_laser_invalid_active == 0U) {
        ctrl->down_laser_invalid_active = 1U;
        ctrl->down_laser_invalid_start_ms = now_ms;
        return 0U;
    }

    return ((now_ms - ctrl->down_laser_invalid_start_ms) >
            R2_CLIMB_DOWN_LASER_INVALID_TIMEOUT_MS) ? 1U : 0U;
}

static uint32_t StateTimeoutMs(const R2_Climb_Ctrl_t *ctrl,
                               R2_ClimbState_t state)
{
    uint32_t step = (uint32_t)state;
    const R2_ClimbMainStep_t *steps = CtrlSteps(ctrl);
    uint32_t count = CtrlStepCount(ctrl);

    if (state == R2_CLIMB_STATE_PREPARE) {
        return R2_CLIMB_PREPARE_TIMEOUT_MS;
    }

    if (state == R2_CLIMB_STATE_UP_APPROACH) {
        return R2_CLIMB_UP_APPROACH_TIMEOUT_MS;
    }

    if (state == R2_CLIMB_STATE_UP_LASER_APPROACH) {
        return R2_CLIMB_UP_LASER_APPROACH_TIMEOUT_MS;
    }

    if (state == R2_CLIMB_STATE_DOWN_LASER_APPROACH) {
        return R2_CLIMB_DOWN_LASER_APPROACH_TIMEOUT_MS;
    }

    if (state == R2_CLIMB_STATE_DOWN_APPROACH) {
        return R2_CLIMB_DOWN_APPROACH_TIMEOUT_MS;
    }

    if ((step >= (uint32_t)R2_CLIMB_STATE_STEP_1) &&
        (step <= count)) {
        return steps[step - 1U].timeout_ms;
    }

    return 0U;
}

static R2_ClimbState_t NextState(const R2_Climb_Ctrl_t *ctrl,
                                 R2_ClimbState_t state)
{
    uint32_t step = (uint32_t)state;
    uint32_t count = CtrlStepCount(ctrl);

    if (state == R2_CLIMB_STATE_UP_LASER_APPROACH) {
        return R2_CLIMB_STATE_PREPARE;
    }

    if (state == R2_CLIMB_STATE_DOWN_LASER_APPROACH) {
        return R2_CLIMB_STATE_DOWN_APPROACH;
    }

    if (state == R2_CLIMB_STATE_DOWN_APPROACH) {
        return R2_CLIMB_STATE_PREPARE;
    }

    if (state == R2_CLIMB_STATE_PREPARE) {
        if ((ctrl != 0) &&
            (NormalizeFlow(ctrl->flow) == (uint8_t)R2_CLIMB_FLOW_UPSTAIRS)) {
            return R2_CLIMB_STATE_UP_APPROACH;
        }
        return R2_CLIMB_STATE_STEP_1;
    }

    if (state == R2_CLIMB_STATE_UP_APPROACH) {
        return R2_CLIMB_STATE_STEP_1;
    }

    if ((step >= (uint32_t)R2_CLIMB_STATE_STEP_1) &&
        (step < count)) {
        return (R2_ClimbState_t)(step + 1U);
    }

    return R2_CLIMB_STATE_DONE;
}

static const char *StateName(uint8_t flow, R2_ClimbState_t state)
{
    uint32_t step = (uint32_t)state;
    const R2_ClimbMainStep_t *steps = FlowSteps(flow);
    uint32_t count = FlowStepCount(flow);

    if (state == R2_CLIMB_STATE_IDLE) {
        return "IDLE";
    }

    if (state == R2_CLIMB_STATE_DONE) {
        return "DONE";
    }

    if (state == R2_CLIMB_STATE_ERROR) {
        return "ERROR";
    }

    if (state == R2_CLIMB_STATE_PREPARE) {
        return "PREPARE_ALL_LEGS_MINUS_30";
    }

    if (state == R2_CLIMB_STATE_UP_APPROACH) {
        return "UP_PREPARE_CHASSIS_FORWARD_30";
    }

    if (state == R2_CLIMB_STATE_UP_LASER_APPROACH) {
        return "UP_LASER_APPROACH_X_LT_35";
    }

    if (state == R2_CLIMB_STATE_DOWN_LASER_APPROACH) {
        return "DOWN_LASER_APPROACH_H_GT_65";
    }

    if (state == R2_CLIMB_STATE_DOWN_APPROACH) {
        return "DOWN_PREPARE_CHASSIS_FORWARD_5";
    }

    if ((step >= (uint32_t)R2_CLIMB_STATE_STEP_1) &&
        (step <= count)) {
        return steps[step - 1U].name;
    }

    return "UNKNOWN";
}

uint32_t R2_Climb_GetDownstairsStepCount(void)
{
    return R2_CLIMB_DOWNSTAIRS_STEP_COUNT;
}

const char *R2_Climb_GetDownstairsStepName(uint8_t step)
{
    if ((step == 0U) || (step > R2_CLIMB_DOWNSTAIRS_STEP_COUNT)) {
        return "UNKNOWN";
    }

    return s_downstairs_steps[step - 1U].name;
}

static void CaptureLegZero(R2_Climb_Ctrl_t *ctrl)
{
    uint8_t i;

    for (i = 0U; i < 4U; i++) {
        ctrl->leg_zero[i] = motor_fdcan2[i].total_angle;
    }
}

static void CaptureDriveSegmentStart(R2_Climb_Ctrl_t *ctrl)
{
    ctrl->front_drive_segment_start[DRIVE_LEFT_IDX] =
        motor_fdcan1[4U].total_angle;
    ctrl->front_drive_segment_start[DRIVE_RIGHT_IDX] =
        motor_fdcan1[5U].total_angle;
    ctrl->drive_segment_start[DRIVE_LEFT_IDX] =
        motor_fdcan2[4U].total_angle;
    ctrl->drive_segment_start[DRIVE_RIGHT_IDX] =
        motor_fdcan2[5U].total_angle;
}

static float ClampLegTarget(float target_mm)
{
    if (target_mm < R2_CLIMB_LEG_MIN_MM) {
        return R2_CLIMB_LEG_MIN_MM;
    }

    if (target_mm > R2_CLIMB_LEG_MAX_MM) {
        return R2_CLIMB_LEG_MAX_MM;
    }

    return target_mm;
}

static void SetAllLegTarget(R2_Climb_Ctrl_t *ctrl, float target_mm)
{
    uint8_t i;
    float limited_target_mm = ClampLegTarget(target_mm);

    for (i = 0U; i < 4U; i++) {
        ctrl->leg_target_mm[i] = limited_target_mm;
    }
}

static void SetFrontLegTarget(R2_Climb_Ctrl_t *ctrl, float target_mm)
{
    float limited_target_mm = ClampLegTarget(target_mm);

    ctrl->leg_target_mm[LEG_FRONT_RIGHT_IDX] = limited_target_mm;
    ctrl->leg_target_mm[LEG_FRONT_LEFT_IDX] = limited_target_mm;
}

static void SetRearLegTarget(R2_Climb_Ctrl_t *ctrl, float target_mm)
{
    float limited_target_mm = ClampLegTarget(target_mm);

    ctrl->leg_target_mm[LEG_REAR_RIGHT_IDX] = limited_target_mm;
    ctrl->leg_target_mm[LEG_REAR_LEFT_IDX] = limited_target_mm;
}

static void SetLegTargets(R2_Climb_Ctrl_t *ctrl,
                          float front_right_mm,
                          float rear_right_mm,
                          float rear_left_mm,
                          float front_left_mm)
{
    ctrl->leg_target_mm[LEG_FRONT_RIGHT_IDX] =
        ClampLegTarget(front_right_mm);
    ctrl->leg_target_mm[LEG_REAR_RIGHT_IDX] =
        ClampLegTarget(rear_right_mm);
    ctrl->leg_target_mm[LEG_REAR_LEFT_IDX] =
        ClampLegTarget(rear_left_mm);
    ctrl->leg_target_mm[LEG_FRONT_LEFT_IDX] =
        ClampLegTarget(front_left_mm);
}

static void SetDriveTarget(R2_Climb_Ctrl_t *ctrl, float target_mm)
{
    ctrl->drive_target_mm[DRIVE_LEFT_IDX] = target_mm;
    ctrl->drive_target_mm[DRIVE_RIGHT_IDX] = target_mm;
}

static uint8_t NormalizeDriveGroupMask(uint8_t mask)
{
    mask &= R2_CLIMB_DRIVE_GROUP_MASK;
    return (mask != 0U) ? mask : R2_CLIMB_DRIVE_GROUP_ALL;
}

static uint8_t ActiveDriveGroupMask(const R2_Climb_Ctrl_t *ctrl)
{
    if (ctrl == 0) {
        return R2_CLIMB_DRIVE_GROUP_ALL;
    }

    return NormalizeDriveGroupMask(ctrl->drive_group_mask);
}

static void SetDriveGroup(R2_Climb_Ctrl_t *ctrl, uint8_t mask)
{
    if (ctrl == 0) {
        return;
    }

    ctrl->drive_group_mask = NormalizeDriveGroupMask(mask);
}

static void SetDriveTargetForGroup(R2_Climb_Ctrl_t *ctrl,
                                   float target_mm,
                                   uint8_t mask)
{
    SetDriveGroup(ctrl, mask);
    SetDriveTarget(ctrl, target_mm);
}

static void AddLegTargetDelta(R2_Climb_Ctrl_t *ctrl, uint8_t mask, float delta_mm)
{
    uint8_t i;

    for (i = 0U; i < 4U; i++) {
        if ((mask & (uint8_t)(1U << i)) == 0U) {
            continue;
        }
        ctrl->leg_target_mm[i] = ClampLegTarget(ctrl->leg_pos_mm[i] + delta_mm);
    }
}

static void ClearLegPidByMask(uint8_t mask)
{
    uint8_t i;

    for (i = 0U; i < 4U; i++) {
        if ((mask & (uint8_t)(1U << i)) != 0U) {
            PID_FDCAN2_Clear(i + 1U);
        }
    }
}

static void ClearDrivePid(void)
{
    PID_FDCAN1_Clear(5U);
    PID_FDCAN1_Clear(6U);
    PID_FDCAN2_Clear(5U);
    PID_FDCAN2_Clear(6U);
}

static void ClearDrivePidByGroup(uint8_t mask)
{
    mask = NormalizeDriveGroupMask(mask);

    if ((mask & R2_CLIMB_DRIVE_GROUP_FRONT) != 0U) {
        PID_FDCAN1_Clear(5U);
        PID_FDCAN1_Clear(6U);
    }

    if ((mask & R2_CLIMB_DRIVE_GROUP_REAR) != 0U) {
        PID_FDCAN2_Clear(5U);
        PID_FDCAN2_Clear(6U);
    }
}

static uint8_t IsTestDriveAction(uint8_t action)
{
    switch ((R2_ClimbTestAction_t)action) {
    case R2_CLIMB_TEST_REAR_DRIVE_FORWARD_30:
    case R2_CLIMB_TEST_REAR_DRIVE_FORWARD_10:
    case R2_CLIMB_TEST_REAR_DRIVE_BACKWARD_10:
    case R2_CLIMB_TEST_REAR_DRIVE_BACKWARD_30:
    case R2_CLIMB_TEST_REAR_DRIVE_FORWARD_500:
    case R2_CLIMB_TEST_REAR_DRIVE_BACKWARD_500:
    case R2_CLIMB_TEST_FRONT_DRIVE_FORWARD_30:
    case R2_CLIMB_TEST_FRONT_DRIVE_FORWARD_10:
    case R2_CLIMB_TEST_FRONT_DRIVE_BACKWARD_10:
    case R2_CLIMB_TEST_FRONT_DRIVE_BACKWARD_30:
    case R2_CLIMB_TEST_FRONT_DRIVE_FORWARD_500:
    case R2_CLIMB_TEST_FRONT_DRIVE_BACKWARD_500:
    case R2_CLIMB_TEST_ALL_DRIVE_FORWARD_30:
    case R2_CLIMB_TEST_ALL_DRIVE_FORWARD_10:
    case R2_CLIMB_TEST_ALL_DRIVE_BACKWARD_10:
    case R2_CLIMB_TEST_ALL_DRIVE_BACKWARD_30:
    case R2_CLIMB_TEST_ALL_DRIVE_FORWARD_500:
    case R2_CLIMB_TEST_ALL_DRIVE_BACKWARD_500:
        return 1U;
    default:
        return 0U;
    }
}

static uint8_t TestDriveGroupMask(uint8_t action)
{
    switch ((R2_ClimbTestAction_t)action) {
    case R2_CLIMB_TEST_FRONT_DRIVE_FORWARD_30:
    case R2_CLIMB_TEST_FRONT_DRIVE_FORWARD_10:
    case R2_CLIMB_TEST_FRONT_DRIVE_BACKWARD_10:
    case R2_CLIMB_TEST_FRONT_DRIVE_BACKWARD_30:
    case R2_CLIMB_TEST_FRONT_DRIVE_FORWARD_500:
    case R2_CLIMB_TEST_FRONT_DRIVE_BACKWARD_500:
        return R2_CLIMB_DRIVE_GROUP_FRONT;

    case R2_CLIMB_TEST_REAR_DRIVE_FORWARD_30:
    case R2_CLIMB_TEST_REAR_DRIVE_FORWARD_10:
    case R2_CLIMB_TEST_REAR_DRIVE_BACKWARD_10:
    case R2_CLIMB_TEST_REAR_DRIVE_BACKWARD_30:
    case R2_CLIMB_TEST_REAR_DRIVE_FORWARD_500:
    case R2_CLIMB_TEST_REAR_DRIVE_BACKWARD_500:
        return R2_CLIMB_DRIVE_GROUP_REAR;

    case R2_CLIMB_TEST_ALL_DRIVE_FORWARD_30:
    case R2_CLIMB_TEST_ALL_DRIVE_FORWARD_10:
    case R2_CLIMB_TEST_ALL_DRIVE_BACKWARD_10:
    case R2_CLIMB_TEST_ALL_DRIVE_BACKWARD_30:
    case R2_CLIMB_TEST_ALL_DRIVE_FORWARD_500:
    case R2_CLIMB_TEST_ALL_DRIVE_BACKWARD_500:
        return R2_CLIMB_DRIVE_GROUP_ALL;

    default:
        return R2_CLIMB_DRIVE_GROUP_ALL;
    }
}

static float TestDriveTargetMm(uint8_t action)
{
    switch ((R2_ClimbTestAction_t)action) {
    case R2_CLIMB_TEST_REAR_DRIVE_FORWARD_30:
    case R2_CLIMB_TEST_FRONT_DRIVE_FORWARD_30:
    case R2_CLIMB_TEST_ALL_DRIVE_FORWARD_30:
        return R2_CLIMB_TEST_DRIVE_30_MM;

    case R2_CLIMB_TEST_REAR_DRIVE_FORWARD_10:
    case R2_CLIMB_TEST_FRONT_DRIVE_FORWARD_10:
    case R2_CLIMB_TEST_ALL_DRIVE_FORWARD_10:
        return R2_CLIMB_TEST_DRIVE_10_MM;

    case R2_CLIMB_TEST_REAR_DRIVE_BACKWARD_10:
    case R2_CLIMB_TEST_FRONT_DRIVE_BACKWARD_10:
    case R2_CLIMB_TEST_ALL_DRIVE_BACKWARD_10:
        return -R2_CLIMB_TEST_DRIVE_10_MM;

    case R2_CLIMB_TEST_REAR_DRIVE_BACKWARD_30:
    case R2_CLIMB_TEST_FRONT_DRIVE_BACKWARD_30:
    case R2_CLIMB_TEST_ALL_DRIVE_BACKWARD_30:
        return -R2_CLIMB_TEST_DRIVE_30_MM;

    case R2_CLIMB_TEST_REAR_DRIVE_FORWARD_500:
    case R2_CLIMB_TEST_FRONT_DRIVE_FORWARD_500:
    case R2_CLIMB_TEST_ALL_DRIVE_FORWARD_500:
        return R2_CLIMB_TEST_DRIVE_500_MM;

    case R2_CLIMB_TEST_REAR_DRIVE_BACKWARD_500:
    case R2_CLIMB_TEST_FRONT_DRIVE_BACKWARD_500:
    case R2_CLIMB_TEST_ALL_DRIVE_BACKWARD_500:
        return -R2_CLIMB_TEST_DRIVE_500_MM;

    default:
        return 0.0f;
    }
}

static void BeginDriveTestAction(R2_Climb_Ctrl_t *ctrl, uint8_t action)
{
    uint8_t mask = TestDriveGroupMask(action);

    ClearDrivePidByGroup(mask);
    CaptureDriveSegmentStart(ctrl);
    SetDriveTargetForGroup(ctrl, TestDriveTargetMm(action), mask);
}

uint8_t R2_Climb_TestActionUsesChassis(uint8_t action)
{
    switch ((R2_ClimbTestAction_t)action) {
    case R2_CLIMB_TEST_CHASSIS_FORWARD_100:
    case R2_CLIMB_TEST_CHASSIS_FORWARD_50:
    case R2_CLIMB_TEST_CHASSIS_BACKWARD_50:
    case R2_CLIMB_TEST_CHASSIS_BACKWARD_100:
    case R2_CLIMB_TEST_CHASSIS_FORWARD_300:
    case R2_CLIMB_TEST_CHASSIS_BACKWARD_300:
        return 1U;
    default:
        return 0U;
    }
}

static uint32_t TestActionTimeoutMs(uint8_t action)
{
    switch ((R2_ClimbTestAction_t)action) {
    case R2_CLIMB_TEST_REAR_DRIVE_FORWARD_500:
    case R2_CLIMB_TEST_REAR_DRIVE_BACKWARD_500:
    case R2_CLIMB_TEST_FRONT_DRIVE_FORWARD_500:
    case R2_CLIMB_TEST_FRONT_DRIVE_BACKWARD_500:
    case R2_CLIMB_TEST_ALL_DRIVE_FORWARD_500:
    case R2_CLIMB_TEST_ALL_DRIVE_BACKWARD_500:
        return R2_CLIMB_TEST_DRIVE_500_TIMEOUT_MS;

    case R2_CLIMB_TEST_CHASSIS_FORWARD_300:
    case R2_CLIMB_TEST_CHASSIS_BACKWARD_300:
        return R2_CLIMB_TEST_CHASSIS_300_TIMEOUT_MS;

    case R2_CLIMB_TEST_FRONT_220:
    case R2_CLIMB_TEST_FRONT_MINUS_30:
    case R2_CLIMB_TEST_REAR_220:
    case R2_CLIMB_TEST_REAR_MINUS_30:
        return R2_CLIMB_STEP_LONG_LEG_TIMEOUT_MS;

    default:
        break;
    }

    if (R2_Climb_TestActionUsesChassis(action) != 0U) {
        return R2_CLIMB_TEST_CHASSIS_TIMEOUT_MS;
    }

    if (IsTestDriveAction(action) != 0U) {
        return R2_CLIMB_TEST_DRIVE_TIMEOUT_MS;
    }

    return R2_CLIMB_TEST_LEG_TIMEOUT_MS;
}

static uint8_t StateUsesChassis(const R2_Climb_Ctrl_t *ctrl,
                                R2_ClimbState_t state)
{
    uint32_t step = (uint32_t)state;
    const R2_ClimbMainStep_t *steps = CtrlSteps(ctrl);
    uint32_t count = CtrlStepCount(ctrl);

    if ((state == R2_CLIMB_STATE_UP_APPROACH) ||
        (state == R2_CLIMB_STATE_UP_LASER_APPROACH) ||
        (state == R2_CLIMB_STATE_DOWN_LASER_APPROACH) ||
        (state == R2_CLIMB_STATE_DOWN_APPROACH)) {
        return 1U;
    }

    if ((step >= (uint32_t)R2_CLIMB_STATE_STEP_1) &&
        (step <= count) &&
        (steps[step - 1U].kind == R2_CLIMB_MAIN_STEP_CHASSIS_DELTA)) {
        return 1U;
    }

    return 0U;
}

static uint8_t ShouldPauseAfterState(const R2_Climb_Ctrl_t *ctrl,
                                     R2_ClimbState_t state)
{
    uint32_t step = (uint32_t)state;

    if (ctrl == 0) {
        return 0U;
    }

    if (ctrl->auto_pause_enabled == 0U) {
        return 0U;
    }

    if ((step < (uint32_t)R2_CLIMB_STATE_STEP_1) ||
        (step > CtrlStepCount(ctrl))) {
        return 0U;
    }

    return (((uint8_t)step) == ctrl->auto_pause_step) ? 1U : 0U;
}

static void ClearAutoPause(R2_Climb_Ctrl_t *ctrl)
{
    if (ctrl == 0) {
        return;
    }

    ctrl->pending_resume = 0U;
    ctrl->auto_pause_enabled = 0U;
    ctrl->auto_pause_active = 0U;
    ctrl->auto_pause_step = 0U;
    ctrl->auto_pause_resume_state = R2_CLIMB_STATE_IDLE;
}

static void BeginState(R2_Climb_Ctrl_t *ctrl,
                       R2_Move_Ctrl_t *move_ctrl,
                       R2_ClimbState_t state,
                       uint32_t now_ms)
{
    if (ctrl == 0) {
        return;
    }

    if ((ctrl->test_chassis_active != 0U) &&
        (StateUsesChassis(ctrl, state) == 0U)) {
        if (move_ctrl != 0) {
            R2_Move_Stop(move_ctrl);
        }
        ctrl->test_chassis_active = 0U;
    }

    ctrl->state = state;
    ctrl->state_done = 0U;
    ctrl->state_start_ms = now_ms;
    ctrl->test_active = 0U;
    ctrl->test_action = R2_CLIMB_TEST_NONE;
    ctrl->test_chassis_active = 0U;
    ctrl->up_laser_invalid_active = 0U;
    ctrl->up_laser_invalid_start_ms = 0U;
    ctrl->down_laser_invalid_active = 0U;
    ctrl->down_laser_invalid_start_ms = 0U;
    SetDriveGroup(ctrl, R2_CLIMB_DRIVE_GROUP_ALL);

    if (state == R2_CLIMB_STATE_PREPARE) {
        ClearLegPidByMask(0x0FU);
        SetAllLegTarget(ctrl, R2_CLIMB_STANDBY_MM);
        CaptureDriveSegmentStart(ctrl);
        SetDriveTarget(ctrl, 0.0f);
        UpdatePositions(ctrl);
        return;
    }

    if (state == R2_CLIMB_STATE_UP_LASER_APPROACH) {
        SetAllLegTarget(ctrl, R2_CLIMB_STANDBY_MM);
        CaptureDriveSegmentStart(ctrl);
        SetDriveTarget(ctrl, 0.0f);
        if (move_ctrl != 0) {
            R2_Move_Stop(move_ctrl);
        }

        if (UpLaserTriggerReached() == 0U) {
            if (move_ctrl == 0) {
                ctrl->error_flags |= R2_CLIMB_ERR_TEST_ACTION;
                ctrl->auto_run = 0U;
                ctrl->state = R2_CLIMB_STATE_ERROR;
            } else {
                R2_Move_Resume(move_ctrl);
                R2_Move_SetMode(move_ctrl, R2_MODE_ROBOT_NO_YAW_VEL);
                R2_Move_SetVel(move_ctrl,
                               R2_CLIMB_UP_LASER_APPROACH_SPEED_MPS,
                               0.0f,
                               0.0f);
                ctrl->test_chassis_active = 1U;
                (void)UpLaserXInvalidTimedOut(ctrl, now_ms);
            }
        }
        UpdatePositions(ctrl);
        return;
    }

    if (state == R2_CLIMB_STATE_DOWN_LASER_APPROACH) {
        SetAllLegTarget(ctrl, R2_CLIMB_STANDBY_MM);
        CaptureDriveSegmentStart(ctrl);
        SetDriveTarget(ctrl, 0.0f);
        if (move_ctrl != 0) {
            R2_Move_Stop(move_ctrl);
        }

        if (DownLaserTriggerReached() == 0U) {
            if (move_ctrl == 0) {
                ctrl->error_flags |= R2_CLIMB_ERR_TEST_ACTION;
                ctrl->auto_run = 0U;
                ctrl->state = R2_CLIMB_STATE_ERROR;
            } else {
                R2_Move_Resume(move_ctrl);
                R2_Move_SetMode(move_ctrl, R2_MODE_ROBOT_NO_YAW_VEL);
                R2_Move_SetVel(move_ctrl,
                               R2_CLIMB_DOWN_LASER_APPROACH_SPEED_MPS,
                               0.0f,
                               0.0f);
                ctrl->test_chassis_active = 1U;
                (void)DownLaserHeightInvalidTimedOut(ctrl, now_ms);
            }
        }
        UpdatePositions(ctrl);
        return;
    }

    if (state == R2_CLIMB_STATE_DOWN_APPROACH) {
        SetAllLegTarget(ctrl, R2_CLIMB_STANDBY_MM);
        CaptureDriveSegmentStart(ctrl);
        SetDriveTarget(ctrl, 0.0f);
        if (move_ctrl == 0) {
            ctrl->error_flags |= R2_CLIMB_ERR_TEST_ACTION;
            ctrl->auto_run = 0U;
            ctrl->state = R2_CLIMB_STATE_ERROR;
        } else {
            R2_Move_Stop(move_ctrl);
            R2_Move_Resume(move_ctrl);
            R2_Move_SetMode(move_ctrl, R2_MODE_ROBOT_NO_YAW_POS);
            if (R2_Move_SetDist(move_ctrl,
                                R2_CLIMB_DOWN_APPROACH_FORWARD_M,
                                0.0f,
                                0.0f) == 0) {
                ctrl->test_chassis_active = 1U;
            } else {
                ctrl->error_flags |= R2_CLIMB_ERR_TEST_ACTION;
                ctrl->auto_run = 0U;
                ctrl->state = R2_CLIMB_STATE_ERROR;
            }
        }
        UpdatePositions(ctrl);
        return;
    }

    if (state == R2_CLIMB_STATE_UP_APPROACH) {
        SetAllLegTarget(ctrl, R2_CLIMB_STANDBY_MM);
        CaptureDriveSegmentStart(ctrl);
        SetDriveTarget(ctrl, 0.0f);
        if (move_ctrl == 0) {
            ctrl->error_flags |= R2_CLIMB_ERR_TEST_ACTION;
            ctrl->auto_run = 0U;
            ctrl->state = R2_CLIMB_STATE_ERROR;
        } else {
            R2_Move_Stop(move_ctrl);
            R2_Move_Resume(move_ctrl);
            R2_Move_SetMode(move_ctrl, R2_MODE_ROBOT_NO_YAW_POS);
            if (R2_Move_SetDist(move_ctrl,
                                R2_CLIMB_UP_APPROACH_FORWARD_M,
                                0.0f,
                                0.0f) == 0) {
                ctrl->test_chassis_active = 1U;
            } else {
                ctrl->error_flags |= R2_CLIMB_ERR_TEST_ACTION;
                ctrl->auto_run = 0U;
                ctrl->state = R2_CLIMB_STATE_ERROR;
            }
        }
        UpdatePositions(ctrl);
        return;
    }

    if (((uint32_t)state >= (uint32_t)R2_CLIMB_STATE_STEP_1) &&
        ((uint32_t)state <= CtrlStepCount(ctrl))) {
        const R2_ClimbMainStep_t *step =
            &CtrlSteps(ctrl)[(uint32_t)state - 1U];

        SetDriveGroup(ctrl, step->drive_group_mask);
        SetLegTargets(ctrl,
                      step->leg_target_mm[LEG_FRONT_RIGHT_IDX],
                      step->leg_target_mm[LEG_REAR_RIGHT_IDX],
                      step->leg_target_mm[LEG_REAR_LEFT_IDX],
                      step->leg_target_mm[LEG_FRONT_LEFT_IDX]);

        if (step->kind == R2_CLIMB_MAIN_STEP_LEG_TARGET) {
            ClearLegPidByMask(step->leg_mask);
        } else if (step->kind == R2_CLIMB_MAIN_STEP_DRIVE_DELTA) {
            ClearDrivePidByGroup(step->drive_group_mask);
            CaptureDriveSegmentStart(ctrl);
            SetDriveTargetForGroup(ctrl, step->delta, step->drive_group_mask);
        } else if (step->kind == R2_CLIMB_MAIN_STEP_CHASSIS_DELTA) {
            if (move_ctrl == 0) {
                ctrl->error_flags |= R2_CLIMB_ERR_TEST_ACTION;
                ctrl->auto_run = 0U;
                ctrl->state = R2_CLIMB_STATE_ERROR;
            } else {
                R2_Move_Stop(move_ctrl);
                R2_Move_Resume(move_ctrl);
                R2_Move_SetMode(move_ctrl, R2_MODE_ROBOT_NO_YAW_POS);
                if (R2_Move_SetDist(move_ctrl, step->delta, 0.0f, 0.0f) == 0) {
                    ctrl->test_chassis_active = 1U;
                } else {
                    ctrl->error_flags |= R2_CLIMB_ERR_TEST_ACTION;
                    ctrl->auto_run = 0U;
                    ctrl->state = R2_CLIMB_STATE_ERROR;
                }
            }
        }

        UpdatePositions(ctrl);
        return;
    }

    if (state == R2_CLIMB_STATE_DONE) {
        if (move_ctrl != 0) {
            R2_Move_SetMode(move_ctrl, R2_MODE_ROBOT_VEL);
        }
        ctrl->auto_run = 0U;
        ctrl->state_done = 1U;
        SetAllLegTarget(ctrl, R2_CLIMB_STANDBY_MM);
        CaptureDriveSegmentStart(ctrl);
        SetDriveTarget(ctrl, 0.0f);
        UpdatePositions(ctrl);
        return;
    }

    UpdatePositions(ctrl);
    return;

}

static void BeginTestAction(R2_Climb_Ctrl_t *ctrl,
                            R2_Move_Ctrl_t *move_ctrl,
                            uint8_t action,
                            uint32_t now_ms)
{
    if (ctrl == 0) {
        return;
    }

    UpdatePositions(ctrl);

    if (ctrl->test_chassis_active != 0U) {
        if (move_ctrl != 0) {
            R2_Move_Stop(move_ctrl);
        }
        ctrl->test_chassis_active = 0U;
    }

    ctrl->enabled = 1U;
    ctrl->auto_run = 0U;
    ctrl->gate_active = 0U;
    ctrl->pending_step = 0U;
    ctrl->pending_auto = 0U;
    ctrl->pending_gate = 0U;
    if (ctrl->auto_pause_active != 0U) {
        ctrl->pending_resume = 0U;
        ctrl->auto_pause_enabled = 0U;
    } else {
        ClearAutoPause(ctrl);
    }
    ctrl->state = R2_CLIMB_STATE_IDLE;
    ctrl->state_done = 0U;
    ctrl->state_start_ms = now_ms;
    ctrl->test_action = action;
    ctrl->test_active = 1U;
    ctrl->test_chassis_active = 0U;
    ctrl->error_flags = 0U;

    switch ((R2_ClimbTestAction_t)action) {
    case R2_CLIMB_TEST_ALL_LEGS_220:
        ClearLegPidByMask(0x0FU);
        SetAllLegTarget(ctrl, R2_CLIMB_LIFT_HIGH_MM);
        break;

    case R2_CLIMB_TEST_ALL_LEGS_UP_10:
        ClearLegPidByMask(0x0FU);
        AddLegTargetDelta(ctrl, 0x0FU, R2_CLIMB_TEST_LEG_DELTA_MM);
        break;

    case R2_CLIMB_TEST_ALL_LEGS_DOWN_10:
        ClearLegPidByMask(0x0FU);
        AddLegTargetDelta(ctrl, 0x0FU, -R2_CLIMB_TEST_LEG_DELTA_MM);
        break;

    case R2_CLIMB_TEST_REAR_DRIVE_FORWARD_30:
    case R2_CLIMB_TEST_REAR_DRIVE_FORWARD_10:
    case R2_CLIMB_TEST_REAR_DRIVE_BACKWARD_10:
    case R2_CLIMB_TEST_REAR_DRIVE_BACKWARD_30:
    case R2_CLIMB_TEST_REAR_DRIVE_FORWARD_500:
    case R2_CLIMB_TEST_REAR_DRIVE_BACKWARD_500:
    case R2_CLIMB_TEST_FRONT_DRIVE_FORWARD_30:
    case R2_CLIMB_TEST_FRONT_DRIVE_FORWARD_10:
    case R2_CLIMB_TEST_FRONT_DRIVE_BACKWARD_10:
    case R2_CLIMB_TEST_FRONT_DRIVE_BACKWARD_30:
    case R2_CLIMB_TEST_FRONT_DRIVE_FORWARD_500:
    case R2_CLIMB_TEST_FRONT_DRIVE_BACKWARD_500:
    case R2_CLIMB_TEST_ALL_DRIVE_FORWARD_30:
    case R2_CLIMB_TEST_ALL_DRIVE_FORWARD_10:
    case R2_CLIMB_TEST_ALL_DRIVE_BACKWARD_10:
    case R2_CLIMB_TEST_ALL_DRIVE_BACKWARD_30:
    case R2_CLIMB_TEST_ALL_DRIVE_FORWARD_500:
    case R2_CLIMB_TEST_ALL_DRIVE_BACKWARD_500:
        BeginDriveTestAction(ctrl, action);
        break;

    case R2_CLIMB_TEST_FRONT_ZERO:
        ClearLegPidByMask((uint8_t)((1U << LEG_FRONT_RIGHT_IDX) |
                                    (1U << LEG_FRONT_LEFT_IDX)));
        SetFrontLegTarget(ctrl, R2_CLIMB_HOME_MM);
        break;

    case R2_CLIMB_TEST_FRONT_220:
        ClearLegPidByMask((uint8_t)((1U << LEG_FRONT_RIGHT_IDX) |
                                    (1U << LEG_FRONT_LEFT_IDX)));
        SetFrontLegTarget(ctrl, R2_CLIMB_LIFT_HIGH_MM);
        break;

    case R2_CLIMB_TEST_FRONT_MINUS_30:
        ClearLegPidByMask((uint8_t)((1U << LEG_FRONT_RIGHT_IDX) |
                                    (1U << LEG_FRONT_LEFT_IDX)));
        SetFrontLegTarget(ctrl, R2_CLIMB_STANDBY_MM);
        break;

    case R2_CLIMB_TEST_FRONT_UP_10:
        ClearLegPidByMask((uint8_t)((1U << LEG_FRONT_RIGHT_IDX) |
                                    (1U << LEG_FRONT_LEFT_IDX)));
        AddLegTargetDelta(ctrl,
                          (uint8_t)((1U << LEG_FRONT_RIGHT_IDX) |
                                    (1U << LEG_FRONT_LEFT_IDX)),
                          R2_CLIMB_TEST_LEG_DELTA_MM);
        break;

    case R2_CLIMB_TEST_FRONT_DOWN_10:
        ClearLegPidByMask((uint8_t)((1U << LEG_FRONT_RIGHT_IDX) |
                                    (1U << LEG_FRONT_LEFT_IDX)));
        AddLegTargetDelta(ctrl,
                          (uint8_t)((1U << LEG_FRONT_RIGHT_IDX) |
                                    (1U << LEG_FRONT_LEFT_IDX)),
                          -R2_CLIMB_TEST_LEG_DELTA_MM);
        break;

    case R2_CLIMB_TEST_CHASSIS_FORWARD_100:
    case R2_CLIMB_TEST_CHASSIS_FORWARD_50:
    case R2_CLIMB_TEST_CHASSIS_BACKWARD_50:
    case R2_CLIMB_TEST_CHASSIS_BACKWARD_100:
    case R2_CLIMB_TEST_CHASSIS_FORWARD_300:
    case R2_CLIMB_TEST_CHASSIS_BACKWARD_300:
    {
        float chassis_delta_m = 0.0f;

        if (move_ctrl == 0) {
            ctrl->error_flags |= R2_CLIMB_ERR_TEST_ACTION;
            ctrl->test_active = 0U;
            ctrl->state = R2_CLIMB_STATE_ERROR;
            break;
        }
        R2_Move_Stop(move_ctrl);
        R2_Move_Resume(move_ctrl);
        R2_Move_SetMode(move_ctrl, R2_MODE_ROBOT_NO_YAW_POS);

        if (action == (uint8_t)R2_CLIMB_TEST_CHASSIS_FORWARD_100) {
            chassis_delta_m = R2_CLIMB_TEST_CHASSIS_100_M;
        } else if (action == (uint8_t)R2_CLIMB_TEST_CHASSIS_FORWARD_50) {
            chassis_delta_m = R2_CLIMB_TEST_CHASSIS_50_M;
        } else if (action == (uint8_t)R2_CLIMB_TEST_CHASSIS_BACKWARD_50) {
            chassis_delta_m = -R2_CLIMB_TEST_CHASSIS_50_M;
        } else if (action == (uint8_t)R2_CLIMB_TEST_CHASSIS_BACKWARD_100) {
            chassis_delta_m = -R2_CLIMB_TEST_CHASSIS_100_M;
        } else if (action == (uint8_t)R2_CLIMB_TEST_CHASSIS_FORWARD_300) {
            chassis_delta_m = R2_CLIMB_TEST_CHASSIS_300_M;
        } else if (action == (uint8_t)R2_CLIMB_TEST_CHASSIS_BACKWARD_300) {
            chassis_delta_m = -R2_CLIMB_TEST_CHASSIS_300_M;
        }

        if (R2_Move_SetDist(move_ctrl, chassis_delta_m, 0.0f, 0.0f) == 0) {
            ctrl->test_chassis_active = 1U;
        }

        if (ctrl->test_chassis_active == 0U) {
            ctrl->error_flags |= R2_CLIMB_ERR_TEST_ACTION;
            ctrl->test_active = 0U;
            ctrl->state = R2_CLIMB_STATE_ERROR;
        }
        break;
    }

    case R2_CLIMB_TEST_REAR_ZERO:
        ClearLegPidByMask((uint8_t)((1U << LEG_REAR_RIGHT_IDX) |
                                    (1U << LEG_REAR_LEFT_IDX)));
        SetRearLegTarget(ctrl, R2_CLIMB_HOME_MM);
        break;

    case R2_CLIMB_TEST_REAR_220:
        ClearLegPidByMask((uint8_t)((1U << LEG_REAR_RIGHT_IDX) |
                                    (1U << LEG_REAR_LEFT_IDX)));
        SetRearLegTarget(ctrl, R2_CLIMB_LIFT_HIGH_MM);
        break;

    case R2_CLIMB_TEST_REAR_MINUS_30:
        ClearLegPidByMask((uint8_t)((1U << LEG_REAR_RIGHT_IDX) |
                                    (1U << LEG_REAR_LEFT_IDX)));
        SetRearLegTarget(ctrl, R2_CLIMB_STANDBY_MM);
        break;

    case R2_CLIMB_TEST_REAR_UP_10:
        ClearLegPidByMask((uint8_t)((1U << LEG_REAR_RIGHT_IDX) |
                                    (1U << LEG_REAR_LEFT_IDX)));
        AddLegTargetDelta(ctrl,
                          (uint8_t)((1U << LEG_REAR_RIGHT_IDX) |
                                    (1U << LEG_REAR_LEFT_IDX)),
                          R2_CLIMB_TEST_LEG_DELTA_MM);
        break;

    case R2_CLIMB_TEST_REAR_DOWN_10:
        ClearLegPidByMask((uint8_t)((1U << LEG_REAR_RIGHT_IDX) |
                                    (1U << LEG_REAR_LEFT_IDX)));
        AddLegTargetDelta(ctrl,
                          (uint8_t)((1U << LEG_REAR_RIGHT_IDX) |
                                    (1U << LEG_REAR_LEFT_IDX)),
                          -R2_CLIMB_TEST_LEG_DELTA_MM);
        break;

    case R2_CLIMB_TEST_ALL_LEGS_ZERO:
        ClearLegPidByMask(0x0FU);
        SetAllLegTarget(ctrl, R2_CLIMB_HOME_MM);
        break;

    default:
        ctrl->error_flags |= R2_CLIMB_ERR_TEST_ACTION;
        ctrl->test_active = 0U;
        ctrl->test_action = R2_CLIMB_TEST_NONE;
        ctrl->state = R2_CLIMB_STATE_ERROR;
        break;
    }

    UpdatePositions(ctrl);
}

static uint8_t TestActionReached(const R2_Climb_Ctrl_t *ctrl,
                                 const R2_Move_Ctrl_t *move_ctrl)
{
    uint8_t front_mask = (uint8_t)((1U << LEG_FRONT_RIGHT_IDX) |
                                  (1U << LEG_FRONT_LEFT_IDX));
    uint8_t rear_mask = (uint8_t)((1U << LEG_REAR_RIGHT_IDX) |
                                 (1U << LEG_REAR_LEFT_IDX));

    switch ((R2_ClimbTestAction_t)ctrl->test_action) {
    case R2_CLIMB_TEST_ALL_LEGS_220:
    case R2_CLIMB_TEST_ALL_LEGS_UP_10:
    case R2_CLIMB_TEST_ALL_LEGS_DOWN_10:
    case R2_CLIMB_TEST_ALL_LEGS_ZERO:
        return LegsReached(ctrl, 0x0FU);

    case R2_CLIMB_TEST_REAR_DRIVE_FORWARD_30:
    case R2_CLIMB_TEST_REAR_DRIVE_FORWARD_10:
    case R2_CLIMB_TEST_REAR_DRIVE_BACKWARD_10:
    case R2_CLIMB_TEST_REAR_DRIVE_BACKWARD_30:
    case R2_CLIMB_TEST_REAR_DRIVE_FORWARD_500:
    case R2_CLIMB_TEST_REAR_DRIVE_BACKWARD_500:
    case R2_CLIMB_TEST_FRONT_DRIVE_FORWARD_30:
    case R2_CLIMB_TEST_FRONT_DRIVE_FORWARD_10:
    case R2_CLIMB_TEST_FRONT_DRIVE_BACKWARD_10:
    case R2_CLIMB_TEST_FRONT_DRIVE_BACKWARD_30:
    case R2_CLIMB_TEST_FRONT_DRIVE_FORWARD_500:
    case R2_CLIMB_TEST_FRONT_DRIVE_BACKWARD_500:
    case R2_CLIMB_TEST_ALL_DRIVE_FORWARD_30:
    case R2_CLIMB_TEST_ALL_DRIVE_FORWARD_10:
    case R2_CLIMB_TEST_ALL_DRIVE_BACKWARD_10:
    case R2_CLIMB_TEST_ALL_DRIVE_BACKWARD_30:
    case R2_CLIMB_TEST_ALL_DRIVE_FORWARD_500:
    case R2_CLIMB_TEST_ALL_DRIVE_BACKWARD_500:
        return DrivesReached(ctrl);

    case R2_CLIMB_TEST_FRONT_ZERO:
    case R2_CLIMB_TEST_FRONT_220:
    case R2_CLIMB_TEST_FRONT_MINUS_30:
    case R2_CLIMB_TEST_FRONT_UP_10:
    case R2_CLIMB_TEST_FRONT_DOWN_10:
        return LegsReached(ctrl, front_mask);

    case R2_CLIMB_TEST_CHASSIS_FORWARD_100:
    case R2_CLIMB_TEST_CHASSIS_FORWARD_50:
    case R2_CLIMB_TEST_CHASSIS_BACKWARD_50:
    case R2_CLIMB_TEST_CHASSIS_BACKWARD_100:
    case R2_CLIMB_TEST_CHASSIS_FORWARD_300:
    case R2_CLIMB_TEST_CHASSIS_BACKWARD_300:
        if ((move_ctrl != 0) &&
            (R2_Move_GetPosState(move_ctrl) == R2_POS_DONE)) {
            return 1U;
        }
        return 0U;

    case R2_CLIMB_TEST_REAR_ZERO:
    case R2_CLIMB_TEST_REAR_220:
    case R2_CLIMB_TEST_REAR_MINUS_30:
    case R2_CLIMB_TEST_REAR_UP_10:
    case R2_CLIMB_TEST_REAR_DOWN_10:
        return LegsReached(ctrl, rear_mask);

    default:
        return 0U;
    }
}

static void UpdatePositions(R2_Climb_Ctrl_t *ctrl)
{
    uint8_t i;
    float leg_scale = SafeScale(R2_CLIMB_LEG_COUNT_PER_MM);
    float drive_scale = SafeScale(R2_CLIMB_DRIVE_COUNT_PER_MM);

    for (i = 0U; i < 4U; i++) {
        ctrl->leg_pos_mm[i] =
            s_leg_dir[i] *
            ((float)(motor_fdcan2[i].total_angle - ctrl->leg_zero[i])) /
            leg_scale;
    }

    ctrl->front_drive_pos_mm[DRIVE_LEFT_IDX] =
        s_drive_dir[DRIVE_LEFT_IDX] *
        ((float)(motor_fdcan1[4U].total_angle -
                 ctrl->front_drive_segment_start[DRIVE_LEFT_IDX])) /
        drive_scale;

    ctrl->front_drive_pos_mm[DRIVE_RIGHT_IDX] =
        s_drive_dir[DRIVE_RIGHT_IDX] *
        ((float)(motor_fdcan1[5U].total_angle -
                 ctrl->front_drive_segment_start[DRIVE_RIGHT_IDX])) /
        drive_scale;

    ctrl->rear_drive_pos_mm[DRIVE_LEFT_IDX] =
        s_drive_dir[DRIVE_LEFT_IDX] *
        ((float)(motor_fdcan2[4U].total_angle -
                 ctrl->drive_segment_start[DRIVE_LEFT_IDX])) /
        drive_scale;

    ctrl->rear_drive_pos_mm[DRIVE_RIGHT_IDX] =
        s_drive_dir[DRIVE_RIGHT_IDX] *
        ((float)(motor_fdcan2[5U].total_angle -
                 ctrl->drive_segment_start[DRIVE_RIGHT_IDX])) /
        drive_scale;

    {
        uint8_t mask = ActiveDriveGroupMask(ctrl);
        uint8_t index;

        for (index = 0U; index < 2U; index++) {
            float pos_sum = 0.0f;
            float pos_count = 0.0f;

            if ((mask & R2_CLIMB_DRIVE_GROUP_FRONT) != 0U) {
                pos_sum += ctrl->front_drive_pos_mm[index];
                pos_count += 1.0f;
            }

            if ((mask & R2_CLIMB_DRIVE_GROUP_REAR) != 0U) {
                pos_sum += ctrl->rear_drive_pos_mm[index];
                pos_count += 1.0f;
            }

            ctrl->drive_pos_mm[index] =
                (pos_count > 0.5f) ? (pos_sum / pos_count) : 0.0f;
        }
    }
}

static uint8_t LegsReached(const R2_Climb_Ctrl_t *ctrl, uint8_t mask)
{
    uint8_t i;

    for (i = 0U; i < 4U; i++) {
        if ((mask & (uint8_t)(1U << i)) == 0U) {
            continue;
        }
        if (AbsF(ctrl->leg_pos_mm[i] - ctrl->leg_target_mm[i]) >
            R2_CLIMB_LEG_TOL_MM) {
            return 0U;
        }
    }

    return 1U;
}

static uint8_t DrivesReached(const R2_Climb_Ctrl_t *ctrl)
{
    uint8_t index;
    uint8_t mask = ActiveDriveGroupMask(ctrl);

    for (index = 0U; index < 2U; index++) {
        if (((mask & R2_CLIMB_DRIVE_GROUP_FRONT) != 0U) &&
            (AbsF(ctrl->front_drive_pos_mm[index] -
                  ctrl->drive_target_mm[index]) >
             R2_CLIMB_DRIVE_TOL_MM)) {
            return 0U;
        }

        if (((mask & R2_CLIMB_DRIVE_GROUP_REAR) != 0U) &&
            (AbsF(ctrl->rear_drive_pos_mm[index] -
                  ctrl->drive_target_mm[index]) >
             R2_CLIMB_DRIVE_TOL_MM)) {
            return 0U;
        }
    }

    return 1U;
}

static uint8_t CurrentStateReached(const R2_Climb_Ctrl_t *ctrl,
                                   const R2_Move_Ctrl_t *move_ctrl)
{
    uint32_t step_index = (uint32_t)ctrl->state;
    const R2_ClimbMainStep_t *step;
    const R2_ClimbMainStep_t *steps = CtrlSteps(ctrl);
    uint32_t count = CtrlStepCount(ctrl);

    if (ctrl->state == R2_CLIMB_STATE_PREPARE) {
        return LegsReached(ctrl, 0x0FU);
    }

    if (ctrl->state == R2_CLIMB_STATE_UP_LASER_APPROACH) {
        return UpLaserTriggerReached();
    }

    if (ctrl->state == R2_CLIMB_STATE_DOWN_LASER_APPROACH) {
        return DownLaserTriggerReached();
    }

    if (ctrl->state == R2_CLIMB_STATE_DOWN_APPROACH) {
        if ((move_ctrl != 0) &&
            (R2_Move_GetPosState(move_ctrl) == R2_POS_DONE)) {
            return 1U;
        }
        return 0U;
    }

    if (ctrl->state == R2_CLIMB_STATE_UP_APPROACH) {
        if ((move_ctrl != 0) &&
            (R2_Move_GetPosState(move_ctrl) == R2_POS_DONE)) {
            return 1U;
        }
        return 0U;
    }

    if ((step_index < (uint32_t)R2_CLIMB_STATE_STEP_1) ||
        (step_index > count)) {
        return 0U;
    }

    step = &steps[step_index - 1U];
    if (step->kind == R2_CLIMB_MAIN_STEP_LEG_TARGET) {
        return LegsReached(ctrl, step->leg_mask);
    }

    if (step->kind == R2_CLIMB_MAIN_STEP_DRIVE_DELTA) {
        return DrivesReached(ctrl);
    }

    if (step->kind == R2_CLIMB_MAIN_STEP_CHASSIS_DELTA) {
        if ((move_ctrl != 0) &&
            (R2_Move_GetPosState(move_ctrl) == R2_POS_DONE)) {
            return 1U;
        }
        return 0U;
    }

    return 0U;
}

static uint8_t DriveHoldActive(const R2_Climb_Ctrl_t *ctrl,
                               R2_ClimbState_t state)
{
    uint32_t step_index = (uint32_t)state;

    if ((step_index >= 2U) && (step_index <= CtrlStepCount(ctrl))) {
        return 1U;
    }

    return 0U;
}

static int32_t LegTargetCount(const R2_Climb_Ctrl_t *ctrl, uint8_t index)
{
    float scale = SafeScale(R2_CLIMB_LEG_COUNT_PER_MM);
    float counts = (float)ctrl->leg_zero[index] +
                   s_leg_dir[index] * ctrl->leg_target_mm[index] * scale;

    return (int32_t)counts;
}

static int32_t DriveTargetCount(const R2_Climb_Ctrl_t *ctrl, uint8_t index)
{
    float scale = SafeScale(R2_CLIMB_DRIVE_COUNT_PER_MM);
    float counts = (float)ctrl->drive_segment_start[index] +
                   s_drive_dir[index] * ctrl->drive_target_mm[index] * scale;

    return (int32_t)counts;
}

static int32_t FrontDriveTargetCount(const R2_Climb_Ctrl_t *ctrl, uint8_t index)
{
    float scale = SafeScale(R2_CLIMB_DRIVE_COUNT_PER_MM);
    float counts = (float)ctrl->front_drive_segment_start[index] +
                   s_drive_dir[index] * ctrl->drive_target_mm[index] * scale;

    return (int32_t)counts;
}

static void BuildDebugView(const R2_Climb_Ctrl_t *ctrl,
                           uint8_t source,
                           const volatile R2_ClimbDebug_t *prev,
                           R2_ClimbDebug_t *view)
{
    uint8_t i;

    if (view == 0) {
        return;
    }

    memset(view, 0, sizeof(*view));
    view->source = source;
    view->flow = R2_CLIMB_FLOW_UPSTAIRS;
    view->state_name = StateName(R2_CLIMB_FLOW_UPSTAIRS, R2_CLIMB_STATE_IDLE);
    view->param_ready = ParamsReady();
    view->leg_count_per_mm = R2_CLIMB_LEG_COUNT_PER_MM;
    view->drive_count_per_mm = R2_CLIMB_DRIVE_COUNT_PER_MM;
    for (i = 0U; i < 4U; i++) {
        view->leg_dir[i] = s_leg_dir[i];
    }
    for (i = 0U; i < 2U; i++) {
        view->drive_dir[i] = s_drive_dir[i];
    }

    if (prev != 0) {
        for (i = 0U; i < 4U; i++) {
            view->leg_current[i] = prev->leg_current[i];
        }
        for (i = 0U; i < 2U; i++) {
            view->front_drive_current[i] = prev->front_drive_current[i];
            view->drive_current[i] = prev->drive_current[i];
        }
    }

    if (ctrl == 0) {
        return;
    }

    view->state = (uint8_t)ctrl->state;
    view->enabled = ctrl->enabled;
    view->auto_run = ctrl->auto_run;
    view->state_done = ctrl->state_done;
    view->error_flags = ctrl->error_flags;
    view->flow = ctrl->flow;
    view->pending_step = ctrl->pending_step;
    view->pending_auto = ctrl->pending_auto;
    view->auto_pause_active = ctrl->auto_pause_active;
    view->state_start_ms = ctrl->state_start_ms;
    view->last_update_ms = ctrl->last_update_ms;
    view->state_name = StateName(ctrl->flow, ctrl->state);
    view->is_motor_active = R2_Climb_IsMotorActive(ctrl);
    view->drive_group_mask = ActiveDriveGroupMask(ctrl);

    if (((ctrl->state == R2_CLIMB_STATE_IDLE) ||
         (ctrl->state == R2_CLIMB_STATE_DONE)) &&
        (ctrl->test_active == 0U)) {
        view->elapsed_ms = 0U;
    } else {
        view->elapsed_ms = ctrl->last_update_ms - ctrl->state_start_ms;
    }

    for (i = 0U; i < 4U; i++) {
        view->leg_pos_mm[i] = ctrl->leg_pos_mm[i];
        view->leg_target_mm[i] = ctrl->leg_target_mm[i];
        view->leg_zero[i] = ctrl->leg_zero[i];
    }

    for (i = 0U; i < 2U; i++) {
        view->front_drive_pos_mm[i] = ctrl->front_drive_pos_mm[i];
        view->rear_drive_pos_mm[i] = ctrl->rear_drive_pos_mm[i];
        view->drive_pos_mm[i] = ctrl->drive_pos_mm[i];
        view->drive_target_mm[i] = ctrl->drive_target_mm[i];
        view->front_drive_segment_start[i] =
            ctrl->front_drive_segment_start[i];
        view->drive_segment_start[i] = ctrl->drive_segment_start[i];
    }
}

static void StoreDebugCurrent(volatile R2_ClimbDebug_t *view,
                              const R2_ClimbMotorCmd_t *cmd)
{
    uint8_t i;

    if ((view == 0) || (cmd == 0)) {
        return;
    }

    for (i = 0U; i < 4U; i++) {
        view->leg_current[i] = cmd->leg[i];
    }
    for (i = 0U; i < 2U; i++) {
        view->front_drive_current[i] = cmd->front_drive[i];
        view->drive_current[i] = cmd->drive[i];
    }
}

void R2_Climb_Init(R2_Climb_Ctrl_t *ctrl)
{
    if (ctrl == 0) {
        return;
    }

    memset(ctrl, 0, sizeof(*ctrl));
    ctrl->state = R2_CLIMB_STATE_IDLE;
    ctrl->flow = R2_CLIMB_FLOW_UPSTAIRS;
    ctrl->pending_flow = R2_CLIMB_FLOW_UPSTAIRS;
    SetDriveGroup(ctrl, R2_CLIMB_DRIVE_GROUP_ALL);
    CaptureLegZero(ctrl);
    CaptureDriveSegmentStart(ctrl);
    ctrl->zero_captured = 1U;
    SetAllLegTarget(ctrl, R2_CLIMB_STANDBY_MM);
    SetDriveTarget(ctrl, 0.0f);
    UpdatePositions(ctrl);
}

void R2_Climb_Stop(R2_Climb_Ctrl_t *ctrl)
{
    int32_t leg_zero[4];
    uint8_t zero_captured;
    uint8_t i;

    if (ctrl == 0) {
        return;
    }

    zero_captured = ctrl->zero_captured;
    for (i = 0U; i < 4U; i++) {
        leg_zero[i] = ctrl->leg_zero[i];
    }

    R2_Climb_Init(ctrl);
    if (zero_captured != 0U) {
        for (i = 0U; i < 4U; i++) {
            ctrl->leg_zero[i] = leg_zero[i];
        }
        ctrl->zero_captured = 1U;
        UpdatePositions(ctrl);
    }
}

void R2_Climb_RequestStep(R2_Climb_Ctrl_t *ctrl)
{
    R2_Climb_RequestFlowStep(ctrl, R2_CLIMB_FLOW_UPSTAIRS);
}

void R2_Climb_RequestAuto(R2_Climb_Ctrl_t *ctrl)
{
    R2_Climb_RequestFlowAuto(ctrl, R2_CLIMB_FLOW_UPSTAIRS);
}

static uint8_t FlowCanSwitch(const R2_Climb_Ctrl_t *ctrl, uint8_t flow)
{
    if (ctrl == 0) {
        return 0U;
    }

    if (ctrl->flow == flow) {
        return 1U;
    }

    if ((ctrl->state == R2_CLIMB_STATE_IDLE) ||
        (ctrl->state == R2_CLIMB_STATE_DONE) ||
        (ctrl->state == R2_CLIMB_STATE_ERROR)) {
        return 1U;
    }

    return 0U;
}

static void RequestFlow(R2_Climb_Ctrl_t *ctrl, uint8_t flow, uint8_t auto_run)
{
    if (ctrl == 0) {
        return;
    }

    flow = NormalizeFlow(flow);
    if (ctrl->enabled == 0U) {
        ctrl->enabled = 1U;
    }

    if (FlowCanSwitch(ctrl, flow) == 0U) {
        ctrl->error_flags |= R2_CLIMB_ERR_FLOW_SWITCH;
        ctrl->auto_run = 0U;
        ctrl->gate_active = 0U;
        ctrl->pending_step = 0U;
        ctrl->pending_auto = 0U;
        ctrl->pending_gate = 0U;
        ClearAutoPause(ctrl);
        ctrl->state_done = 0U;
        ctrl->state = R2_CLIMB_STATE_ERROR;
        return;
    }

    if (ctrl->flow != flow) {
        ctrl->flow = flow;
        ctrl->state = R2_CLIMB_STATE_IDLE;
        ctrl->state_done = 0U;
        ctrl->auto_run = 0U;
        ctrl->gate_active = 0U;
        ClearAutoPause(ctrl);
    }

    ctrl->pending_flow = flow;
    ctrl->gate_active = 0U;
    ctrl->pending_gate = 0U;
    ClearAutoPause(ctrl);
    if (auto_run != 0U) {
        ctrl->pending_auto = 1U;
    } else {
        ctrl->pending_step = 1U;
    }
}

void R2_Climb_RequestFlowGate(R2_Climb_Ctrl_t *ctrl, uint8_t flow)
{
    if (ctrl == 0) {
        return;
    }

    flow = NormalizeFlow(flow);
    if (ctrl->enabled == 0U) {
        ctrl->enabled = 1U;
    }

    if (FlowCanSwitch(ctrl, flow) == 0U) {
        ctrl->error_flags |= R2_CLIMB_ERR_FLOW_SWITCH;
        ctrl->auto_run = 0U;
        ctrl->gate_active = 0U;
        ctrl->pending_step = 0U;
        ctrl->pending_auto = 0U;
        ctrl->pending_gate = 0U;
        ClearAutoPause(ctrl);
        ctrl->state_done = 0U;
        ctrl->state = R2_CLIMB_STATE_ERROR;
        return;
    }

    if (ctrl->flow != flow) {
        ctrl->flow = flow;
        ctrl->state = R2_CLIMB_STATE_IDLE;
        ctrl->state_done = 0U;
        ctrl->auto_run = 0U;
        ctrl->gate_active = 0U;
        ClearAutoPause(ctrl);
    }

    ctrl->pending_flow = flow;
    ctrl->pending_step = 0U;
    ctrl->pending_auto = 0U;
    ctrl->pending_gate = 1U;
    ClearAutoPause(ctrl);
}

void R2_Climb_RequestFlowStep(R2_Climb_Ctrl_t *ctrl, uint8_t flow)
{
    RequestFlow(ctrl, flow, 0U);
}

void R2_Climb_RequestFlowAuto(R2_Climb_Ctrl_t *ctrl, uint8_t flow)
{
    RequestFlow(ctrl, flow, 1U);
}

void R2_Climb_RequestFlowAutoPause(R2_Climb_Ctrl_t *ctrl, uint8_t flow)
{
    if (ctrl == 0) {
        return;
    }

    flow = NormalizeFlow(flow);
    RequestFlow(ctrl, flow, 1U);
    if (ctrl->state == R2_CLIMB_STATE_ERROR) {
        return;
    }

    ctrl->auto_pause_enabled = 1U;
    ctrl->auto_pause_active = 0U;
    ctrl->auto_pause_step =
        (flow == (uint8_t)R2_CLIMB_FLOW_DOWNSTAIRS) ?
        R2_CLIMB_DOWN_INTERRUPT_PAUSE_STEP :
        R2_CLIMB_UP_INTERRUPT_PAUSE_STEP;
    ctrl->auto_pause_resume_state = R2_CLIMB_STATE_IDLE;
    ctrl->pending_resume = 0U;
}

void R2_Climb_RequestAutoResume(R2_Climb_Ctrl_t *ctrl)
{
    if (ctrl == 0) {
        return;
    }

    if (ctrl->enabled == 0U) {
        ctrl->enabled = 1U;
    }

    ctrl->pending_resume = 1U;
}

void R2_Climb_RequestTestAction(R2_Climb_Ctrl_t *ctrl, uint8_t action)
{
    if (ctrl == 0) {
        return;
    }

    if (ctrl->enabled == 0U) {
        ctrl->enabled = 1U;
    }
    ctrl->pending_test_action = action;
}

void R2_Climb_SetInput(R2_Climb_Ctrl_t *ctrl,
                       uint8_t enable_level,
                       uint8_t step_level,
                       uint8_t auto_level)
{
    uint8_t step_edge;
    uint8_t auto_edge;

    if (ctrl == 0) {
        return;
    }

    if (enable_level == 0U) {
        R2_Climb_Stop(ctrl);
        ctrl->last_step_level = step_level;
        ctrl->last_auto_level = auto_level;
        return;
    }

    if (ctrl->enabled == 0U) {
        ctrl->enabled = 1U;
        ctrl->state = R2_CLIMB_STATE_IDLE;
        ctrl->error_flags = 0U;
    }

    step_edge = ((step_level != 0U) && (ctrl->last_step_level == 0U)) ? 1U : 0U;
    auto_edge = ((auto_level != 0U) && (ctrl->last_auto_level == 0U)) ? 1U : 0U;

    if (step_edge != 0U) {
        R2_Climb_RequestStep(ctrl);
    }

    if (auto_edge != 0U) {
        R2_Climb_RequestAuto(ctrl);
    }

    ctrl->last_step_level = step_level;
    ctrl->last_auto_level = auto_level;
}

void R2_Climb_Update(R2_Climb_Ctrl_t *ctrl,
                     R2_Move_Ctrl_t *move_ctrl,
                     uint32_t now_ms)
{
    uint8_t reached;
    uint8_t step;
    uint8_t auto_req;
    uint8_t gate_req;
    uint8_t resume_req;
    uint8_t test_req;
    R2_ClimbState_t resume_state;

    if (ctrl == 0) {
        return;
    }

    ctrl->last_update_ms = now_ms;
    UpdatePositions(ctrl);

    if (ctrl->enabled == 0U) {
        return;
    }

    if (ParamsReady() == 0U) {
        ctrl->error_flags |= R2_CLIMB_ERR_PARAM_NOT_CONFIGURED;
        ctrl->auto_run = 0U;
        ctrl->state_done = 0U;
        ctrl->pending_step = 0U;
        ctrl->pending_auto = 0U;
        ctrl->pending_gate = 0U;
        ClearAutoPause(ctrl);
        ctrl->gate_active = 0U;
        ctrl->pending_test_action = 0U;
        if (ctrl->test_chassis_active != 0U) {
            if (move_ctrl != 0) {
                R2_Move_Stop(move_ctrl);
            }
            ctrl->test_chassis_active = 0U;
        }
        ctrl->test_active = 0U;
        ctrl->state = R2_CLIMB_STATE_ERROR;
        return;
    }

    test_req = ctrl->pending_test_action;
    ctrl->pending_test_action = 0U;
    if (test_req != 0U) {
        BeginTestAction(ctrl, move_ctrl, test_req, now_ms);
        return;
    }

    if (ctrl->test_active != 0U) {
        ctrl->pending_step = 0U;
        ctrl->pending_auto = 0U;
        ctrl->pending_resume = 0U;

        if ((now_ms - ctrl->state_start_ms) >
            TestActionTimeoutMs(ctrl->test_action)) {
            ctrl->error_flags |= R2_CLIMB_ERR_TIMEOUT;
            if (ctrl->test_chassis_active != 0U) {
                if (move_ctrl != 0) {
                    R2_Move_Stop(move_ctrl);
                }
                ctrl->test_chassis_active = 0U;
            }
            ctrl->test_active = 0U;
            ctrl->state_done = 0U;
            ctrl->state = R2_CLIMB_STATE_ERROR;
            return;
        }

        if (TestActionReached(ctrl, move_ctrl) != 0U) {
            ctrl->test_active = 0U;
            ctrl->test_chassis_active = 0U;
            ctrl->state_done = 1U;
        }
        return;
    }

    step = ctrl->pending_step;
    auto_req = ctrl->pending_auto;
    gate_req = ctrl->pending_gate;
    resume_req = ctrl->pending_resume;
    ctrl->pending_step = 0U;
    ctrl->pending_auto = 0U;
    ctrl->pending_gate = 0U;
    ctrl->pending_resume = 0U;

    if (resume_req != 0U) {
        if ((ctrl->auto_pause_active != 0U) &&
            (ctrl->state_done != 0U) &&
            (ctrl->state != R2_CLIMB_STATE_DONE) &&
            (ctrl->state != R2_CLIMB_STATE_ERROR)) {
            resume_state = ctrl->auto_pause_resume_state;
            if (resume_state == R2_CLIMB_STATE_IDLE) {
                resume_state = ctrl->state;
            }
            ClearAutoPause(ctrl);
            ctrl->auto_run = 1U;
            ctrl->gate_active = 0U;
            BeginState(ctrl, move_ctrl, NextState(ctrl, resume_state), now_ms);
        } else {
            ctrl->error_flags |= R2_CLIMB_ERR_FLOW_SWITCH;
            ctrl->auto_run = 0U;
            ctrl->gate_active = 0U;
            ClearAutoPause(ctrl);
            ctrl->state_done = 0U;
            ctrl->state = R2_CLIMB_STATE_ERROR;
        }
        step = 0U;
        auto_req = 0U;
        gate_req = 0U;
    }

    if (gate_req != 0U) {
        ctrl->auto_run = 0U;
        ctrl->gate_active = 1U;
        if ((ctrl->state == R2_CLIMB_STATE_IDLE) ||
            (ctrl->state == R2_CLIMB_STATE_DONE) ||
            (ctrl->state_done != 0U)) {
            if (NormalizeFlow(ctrl->flow) ==
                (uint8_t)R2_CLIMB_FLOW_UPSTAIRS) {
                BeginState(ctrl,
                           move_ctrl,
                           R2_CLIMB_STATE_UP_LASER_APPROACH,
                           now_ms);
            } else {
                BeginState(ctrl,
                           move_ctrl,
                           R2_CLIMB_STATE_DOWN_LASER_APPROACH,
                           now_ms);
            }
        } else {
            ctrl->error_flags |= R2_CLIMB_ERR_FLOW_SWITCH;
            ctrl->gate_active = 0U;
            ctrl->state_done = 0U;
            ctrl->state = R2_CLIMB_STATE_ERROR;
        }
        step = 0U;
        auto_req = 0U;
    }

    if (auto_req != 0U) {
        if (ctrl->state == R2_CLIMB_STATE_ERROR) {
            ctrl->auto_run = 0U;
        } else {
            ctrl->auto_run = 1U;
            if ((ctrl->state == R2_CLIMB_STATE_IDLE) ||
                (ctrl->state == R2_CLIMB_STATE_DONE)) {
                if (NormalizeFlow(ctrl->flow) ==
                    (uint8_t)R2_CLIMB_FLOW_UPSTAIRS) {
                    BeginState(ctrl,
                               move_ctrl,
                               R2_CLIMB_STATE_UP_LASER_APPROACH,
                               now_ms);
                } else {
                    BeginState(ctrl,
                               move_ctrl,
                               R2_CLIMB_STATE_DOWN_LASER_APPROACH,
                               now_ms);
                }
            } else if (ctrl->state_done != 0U) {
                BeginState(ctrl, move_ctrl, NextState(ctrl, ctrl->state), now_ms);
            }
        }
        step = 0U;
    }

    if (step != 0U) {
        ctrl->auto_run = 0U;
        if (ctrl->state == R2_CLIMB_STATE_IDLE) {
            BeginState(ctrl, move_ctrl, R2_CLIMB_STATE_PREPARE, now_ms);
        } else if ((ctrl->state != R2_CLIMB_STATE_DONE) &&
                   (ctrl->state != R2_CLIMB_STATE_ERROR)) {
            BeginState(ctrl, move_ctrl, NextState(ctrl, ctrl->state), now_ms);
        }
    }

    if ((ctrl->state == R2_CLIMB_STATE_IDLE) ||
        (ctrl->state == R2_CLIMB_STATE_DONE) ||
        (ctrl->state == R2_CLIMB_STATE_ERROR) ||
        (ctrl->state_done != 0U)) {
        return;
    }

    if ((ctrl->state == R2_CLIMB_STATE_UP_LASER_APPROACH) &&
        (UpLaserXInvalidTimedOut(ctrl, now_ms) != 0U)) {
        ctrl->error_flags |= R2_CLIMB_ERR_TIMEOUT;
        if (ctrl->test_chassis_active != 0U) {
            if (move_ctrl != 0) {
                R2_Move_Stop(move_ctrl);
            }
            ctrl->test_chassis_active = 0U;
        }
        ctrl->auto_run = 0U;
        ctrl->gate_active = 0U;
        ctrl->state_done = 0U;
        ctrl->state = R2_CLIMB_STATE_ERROR;
        return;
    }

    if ((ctrl->state == R2_CLIMB_STATE_DOWN_LASER_APPROACH) &&
        (DownLaserHeightInvalidTimedOut(ctrl, now_ms) != 0U)) {
        ctrl->error_flags |= R2_CLIMB_ERR_TIMEOUT;
        if (ctrl->test_chassis_active != 0U) {
            if (move_ctrl != 0) {
                R2_Move_Stop(move_ctrl);
            }
            ctrl->test_chassis_active = 0U;
        }
        ctrl->auto_run = 0U;
        ctrl->gate_active = 0U;
        ctrl->state_done = 0U;
        ctrl->state = R2_CLIMB_STATE_ERROR;
        return;
    }

    if (StateTimeoutMs(ctrl, ctrl->state) != 0U) {
        if ((now_ms - ctrl->state_start_ms) > StateTimeoutMs(ctrl, ctrl->state)) {
            ctrl->error_flags |= R2_CLIMB_ERR_TIMEOUT;
            if ((StateUsesChassis(ctrl, ctrl->state) != 0U) &&
                (ctrl->test_chassis_active != 0U)) {
                if (move_ctrl != 0) {
                    R2_Move_Stop(move_ctrl);
                }
                ctrl->test_chassis_active = 0U;
            }
            ctrl->auto_run = 0U;
            ctrl->gate_active = 0U;
            ctrl->state_done = 0U;
            ctrl->state = R2_CLIMB_STATE_ERROR;
            return;
        }
    }

    reached = CurrentStateReached(ctrl, move_ctrl);
    if (reached == 0U) {
        return;
    }

    if ((StateUsesChassis(ctrl, ctrl->state) != 0U) &&
        (ctrl->test_chassis_active != 0U)) {
        if (move_ctrl != 0) {
            R2_Move_Stop(move_ctrl);
        }
        ctrl->test_chassis_active = 0U;
    }

    if (ctrl->gate_active != 0U) {
        if (ctrl->state == R2_CLIMB_STATE_DOWN_LASER_APPROACH) {
            BeginState(ctrl, move_ctrl, R2_CLIMB_STATE_DOWN_APPROACH, now_ms);
        } else {
            ctrl->gate_active = 0U;
            ctrl->auto_run = 0U;
            ctrl->state_done = 1U;
        }
        return;
    }

    if (ctrl->auto_run != 0U) {
        if (ShouldPauseAfterState(ctrl, ctrl->state) != 0U) {
            ctrl->auto_run = 0U;
            ctrl->auto_pause_enabled = 0U;
            ctrl->auto_pause_active = 1U;
            ctrl->auto_pause_resume_state = ctrl->state;
            ctrl->state_done = 1U;
            return;
        }
        BeginState(ctrl, move_ctrl, NextState(ctrl, ctrl->state), now_ms);
    } else {
        if ((NextState(ctrl, ctrl->state) == R2_CLIMB_STATE_DONE) &&
            (move_ctrl != 0)) {
            R2_Move_SetMode(move_ctrl, R2_MODE_ROBOT_VEL);
        }
        ctrl->state_done = 1U;
    }
}

uint8_t R2_Climb_IsMotorActive(const R2_Climb_Ctrl_t *ctrl)
{
    if ((ctrl == 0) || (ctrl->enabled == 0U)) {
        return 0U;
    }

    if (ctrl->state == R2_CLIMB_STATE_ERROR) {
        return 0U;
    }

    return 1U;
}

void R2_Climb_GetMotorCurrent(const R2_Climb_Ctrl_t *ctrl,
                              R2_ClimbMotorCmd_t *cmd)
{
    uint8_t i;
    uint8_t active;

    if (cmd == 0) {
        return;
    }

    for (i = 0U; i < 4U; i++) {
        cmd->leg[i] = 0;
        cmd->front_drive[i] = 0;
        cmd->drive[i] = 0;
    }

    active = R2_Climb_IsMotorActive(ctrl);

    if (active == 0U) {
        for (i = 0U; i < 4U; i++) {
            cmd->leg[i] = (int16_t)PID_velocity_realize_2(0.0f, (int)i + 1);
        }
        cmd->front_drive[0] = (int16_t)PID_velocity_realize_1(0.0f, 5);
        cmd->front_drive[1] = (int16_t)PID_velocity_realize_1(0.0f, 6);
        cmd->drive[0] = (int16_t)PID_velocity_realize_2(0.0f, 5);
        cmd->drive[1] = (int16_t)PID_velocity_realize_2(0.0f, 6);
        return;
    }

    for (i = 0U; i < 4U; i++) {
        cmd->leg[i] = (int16_t)pid_call_2((float)LegTargetCount(ctrl, i),
                                          (int)i + 1);
    }

    if ((DriveHoldActive(ctrl, ctrl->state) != 0U) ||
        (IsTestDriveAction(ctrl->test_action) != 0U)) {
        uint8_t drive_mask = ActiveDriveGroupMask(ctrl);

        if ((drive_mask & R2_CLIMB_DRIVE_GROUP_FRONT) != 0U) {
            cmd->front_drive[0] =
                (int16_t)pid_call_1((float)FrontDriveTargetCount(ctrl, DRIVE_LEFT_IDX), 5);
            cmd->front_drive[1] =
                (int16_t)pid_call_1((float)FrontDriveTargetCount(ctrl, DRIVE_RIGHT_IDX), 6);
        } else {
            cmd->front_drive[0] = (int16_t)PID_velocity_realize_1(0.0f, 5);
            cmd->front_drive[1] = (int16_t)PID_velocity_realize_1(0.0f, 6);
        }

        if ((drive_mask & R2_CLIMB_DRIVE_GROUP_REAR) != 0U) {
            cmd->drive[0] =
                (int16_t)pid_call_2((float)DriveTargetCount(ctrl, DRIVE_LEFT_IDX), 5);
            cmd->drive[1] =
                (int16_t)pid_call_2((float)DriveTargetCount(ctrl, DRIVE_RIGHT_IDX), 6);
        } else {
            cmd->drive[0] = (int16_t)PID_velocity_realize_2(0.0f, 5);
            cmd->drive[1] = (int16_t)PID_velocity_realize_2(0.0f, 6);
        }
    } else {
        cmd->front_drive[0] = (int16_t)PID_velocity_realize_1(0.0f, 5);
        cmd->front_drive[1] = (int16_t)PID_velocity_realize_1(0.0f, 6);
        cmd->drive[0] = (int16_t)PID_velocity_realize_2(0.0f, 5);
        cmd->drive[1] = (int16_t)PID_velocity_realize_2(0.0f, 6);
    }
}

void R2_Climb_UpdateDebugViews(const R2_Climb_Ctrl_t *usart_ctrl,
                               const R2_Climb_Ctrl_t *usb_ctrl,
                               uint8_t active_source)
{
    R2_ClimbDebug_t usart_view;
    R2_ClimbDebug_t usb_view;
    R2_ClimbDebug_t active_view;

    BuildDebugView(usart_ctrl,
                   R2_CLIMB_DEBUG_SOURCE_USART,
                   &g_r2_climb_debug_usart,
                   &usart_view);
    BuildDebugView(usb_ctrl,
                   R2_CLIMB_DEBUG_SOURCE_USB,
                   &g_r2_climb_debug_usb,
                   &usb_view);

    g_r2_climb_debug_usart = usart_view;
    g_r2_climb_debug_usb = usb_view;

    if (active_source == R2_CLIMB_DEBUG_SOURCE_USB) {
        active_view = usb_view;
    } else if (active_source == R2_CLIMB_DEBUG_SOURCE_USART) {
        active_view = usart_view;
    } else {
        BuildDebugView(0,
                       R2_CLIMB_DEBUG_SOURCE_NONE,
                       &g_r2_climb_debug_active,
                       &active_view);
    }

    g_r2_climb_debug_active = active_view;
}

void R2_Climb_SetDebugMotorCurrent(uint8_t source,
                                   const R2_ClimbMotorCmd_t *cmd)
{
    R2_ClimbMotorCmd_t zero_cmd;

    if (cmd == 0) {
        return;
    }

    if (source == R2_CLIMB_DEBUG_SOURCE_USART) {
        StoreDebugCurrent(&g_r2_climb_debug_usart, cmd);
        if (g_r2_climb_debug_active.source == R2_CLIMB_DEBUG_SOURCE_USART) {
            StoreDebugCurrent(&g_r2_climb_debug_active, cmd);
        }
    } else if (source == R2_CLIMB_DEBUG_SOURCE_USB) {
        StoreDebugCurrent(&g_r2_climb_debug_usb, cmd);
        if (g_r2_climb_debug_active.source == R2_CLIMB_DEBUG_SOURCE_USB) {
            StoreDebugCurrent(&g_r2_climb_debug_active, cmd);
        }
    } else {
        memset(&zero_cmd, 0, sizeof(zero_cmd));
        StoreDebugCurrent(&g_r2_climb_debug_usart, &zero_cmd);
        StoreDebugCurrent(&g_r2_climb_debug_usb, &zero_cmd);
        StoreDebugCurrent(&g_r2_climb_debug_active, &zero_cmd);
    }
}
