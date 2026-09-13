#ifndef __R2_CLIMB_H
#define __R2_CLIMB_H

#include <stdint.h>
#include "R2_move.h"

/*
 * Climb motor map, viewed from above with the vehicle heading forward:
 *   Leg lift motors are numbered from the right-top corner clockwise.
 *   Leg 1: front right
 *   Leg 2: rear  right
 *   Leg 3: rear  left
 *   Leg 4: front left
 *   FDCAN1 drive wheel 5: front left, under the front-left post
 *   FDCAN1 drive wheel 6: front right, under the front-right post
 *   FDCAN2 drive wheel 5: rear left
 *   FDCAN2 drive wheel 6: rear right
 *
 * Leg-down/support rotation direction:
 *   Leg 1 motor: counter-clockwise
 *   Leg 2 motor: clockwise
 *   Leg 3 motor: counter-clockwise
 *   Leg 4 motor: clockwise
 * A positive support target therefore commands count signs +, -, +, - for
 * legs 1..4. Confirm this at low output before running full travel.
 *
 * Drive-forward rotation direction, viewed at the motor output shaft:
 *   FDCAN1/FDCAN2 drive wheel 5: clockwise
 *   FDCAN1/FDCAN2 drive wheel 6: counter-clockwise
 * A positive forward target therefore commands count signs -, + for
 * drive wheels 5..6.
 */

/*
 * Scale factors from the actual climb mechanism.
 *
 * Position convention:
 *   0 mm  : power-on contact-ground position.
 *   >0 mm : legs extend to lift the chassis.
 *   <0 mm : legs retract above ground.
 *   negative travel is limited to 30 mm above ground.
 *   positive support travel is limited to 260 mm.
 *   IDLE/DONE hold the standby position, 30 mm above the power-on zero.
 *
 * Leg rack:   count/mm = encoder_count_per_motor_rev * total_reduction /
 *                        pinion_pitch_circumference_mm
 * Drive wheel count/mm = encoder_count_per_motor_rev * motor_internal_reduction *
 *                        output_to_wheel_reduction / wheel_circumference_mm
 *
 * The four drive wheels are 1:1 from motor output shaft to wheel.
 */
#ifndef R2_CLIMB_PARAM_CONFIGURED
#define R2_CLIMB_PARAM_CONFIGURED        1U
#endif

#ifndef R2_CLIMB_ENCODER_COUNT_PER_REV
#define R2_CLIMB_ENCODER_COUNT_PER_REV   8192.0f
#endif

#ifndef R2_CLIMB_LEG_REDUCTION
#define R2_CLIMB_LEG_REDUCTION           (3591.0f / 187.0f)
#endif

#ifndef R2_CLIMB_LEG_PINION_TOOTH_COUNT
#define R2_CLIMB_LEG_PINION_TOOTH_COUNT  18.0f
#endif

#ifndef R2_CLIMB_LEG_RACK_PITCH_MM
#define R2_CLIMB_LEG_RACK_PITCH_MM       4.71f
#endif

#ifndef R2_CLIMB_LEG_PINION_TRAVEL_PER_REV_MM
#define R2_CLIMB_LEG_PINION_TRAVEL_PER_REV_MM \
    (R2_CLIMB_LEG_PINION_TOOTH_COUNT * R2_CLIMB_LEG_RACK_PITCH_MM)
#endif

#ifndef R2_CLIMB_DRIVE_REDUCTION
#define R2_CLIMB_DRIVE_REDUCTION         36.0f
#endif

#ifndef R2_CLIMB_DRIVE_EXTERNAL_REDUCTION
#define R2_CLIMB_DRIVE_EXTERNAL_REDUCTION 1.0f
#endif

#ifndef R2_CLIMB_DRIVE_TOTAL_REDUCTION
#define R2_CLIMB_DRIVE_TOTAL_REDUCTION \
    (R2_CLIMB_DRIVE_REDUCTION * R2_CLIMB_DRIVE_EXTERNAL_REDUCTION)
#endif

#ifndef R2_CLIMB_DRIVE_WHEEL_DIAMETER_MM
#define R2_CLIMB_DRIVE_WHEEL_DIAMETER_MM 64.0f
#endif

#ifndef R2_CLIMB_PI
#define R2_CLIMB_PI                      3.14159265358979323846f
#endif

#ifndef R2_CLIMB_DRIVE_WHEEL_CIRCUMFERENCE_MM
#define R2_CLIMB_DRIVE_WHEEL_CIRCUMFERENCE_MM \
    (R2_CLIMB_PI * R2_CLIMB_DRIVE_WHEEL_DIAMETER_MM)
#endif

#ifndef R2_CLIMB_LEG_COUNT_PER_MM
#define R2_CLIMB_LEG_COUNT_PER_MM \
    ((R2_CLIMB_ENCODER_COUNT_PER_REV * R2_CLIMB_LEG_REDUCTION) / \
     R2_CLIMB_LEG_PINION_TRAVEL_PER_REV_MM)
#endif

#ifndef R2_CLIMB_DRIVE_COUNT_PER_MM
#define R2_CLIMB_DRIVE_COUNT_PER_MM \
    ((R2_CLIMB_ENCODER_COUNT_PER_REV * R2_CLIMB_DRIVE_TOTAL_REDUCTION) / \
     R2_CLIMB_DRIVE_WHEEL_CIRCUMFERENCE_MM)
#endif

#ifndef R2_CLIMB_LEG1_DIR
#define R2_CLIMB_LEG1_DIR                1.0f
#endif
#ifndef R2_CLIMB_LEG2_DIR
#define R2_CLIMB_LEG2_DIR              (-1.0f)
#endif
#ifndef R2_CLIMB_LEG3_DIR
#define R2_CLIMB_LEG3_DIR                1.0f
#endif
#ifndef R2_CLIMB_LEG4_DIR
#define R2_CLIMB_LEG4_DIR              (-1.0f)
#endif
#ifndef R2_CLIMB_DRIVE_LEFT_DIR
#define R2_CLIMB_DRIVE_LEFT_DIR        (-1.0f)
#endif
#ifndef R2_CLIMB_DRIVE_RIGHT_DIR
#define R2_CLIMB_DRIVE_RIGHT_DIR         1.0f
#endif

#define R2_CLIMB_AIR_CLEARANCE_MAX_MM   30.0f
#define R2_CLIMB_LEG_MIN_MM            (-R2_CLIMB_AIR_CLEARANCE_MAX_MM)
#define R2_CLIMB_LEG_MAX_MM            260.0f

#define R2_CLIMB_HOME_MM                 0.0f
#define R2_CLIMB_STANDBY_MM            (-R2_CLIMB_AIR_CLEARANCE_MAX_MM)
#define R2_CLIMB_LIFT_HIGH_MM          220.0f

#define R2_CLIMB_TEST_LEG_DELTA_MM       10.0f
#define R2_CLIMB_TEST_DRIVE_30_MM        30.0f
#define R2_CLIMB_TEST_DRIVE_10_MM        10.0f
#define R2_CLIMB_TEST_DRIVE_500_MM      500.0f
#define R2_CLIMB_TEST_CHASSIS_100_M       0.10f
#define R2_CLIMB_TEST_CHASSIS_50_M        0.05f
#define R2_CLIMB_TEST_CHASSIS_300_M       0.30f

#define R2_CLIMB_LEG_TOL_MM              3.0f
#define R2_CLIMB_DRIVE_TOL_MM            5.0f

/* Main flows are compacted from debug actions into target/delta states. */
#define R2_CLIMB_MAIN_STEP_COUNT          14U
#define R2_CLIMB_DOWNSTAIRS_STEP_COUNT    15U
#define R2_CLIMB_UP_INTERRUPT_PAUSE_STEP  9U
#define R2_CLIMB_DOWN_INTERRUPT_PAUSE_STEP 5U
#define R2_CLIMB_STATE_DONE_ID            22U
#define R2_CLIMB_STATE_ERROR_ID           23U
#define R2_CLIMB_STATE_PREPARE_ID         24U
#define R2_CLIMB_STATE_UP_APPROACH_ID     25U
#define R2_CLIMB_STATE_UP_LASER_APPROACH_ID 26U
#define R2_CLIMB_STATE_DOWN_LASER_APPROACH_ID 27U
#define R2_CLIMB_STATE_DOWN_APPROACH_ID   28U
#define R2_CLIMB_UP_TRIGGER_X_MIN_MM       0
#define R2_CLIMB_UP_TRIGGER_X_MAX_MM       35
#define R2_CLIMB_UP_LASER_APPROACH_SPEED_MPS 0.08f
#define R2_CLIMB_UP_LASER_APPROACH_TIMEOUT_MS 8000U
#define R2_CLIMB_UP_LASER_INVALID_TIMEOUT_MS 1000U
#define R2_CLIMB_UP_APPROACH_FORWARD_M     0.03f
#define R2_CLIMB_UP_APPROACH_TIMEOUT_MS 4000U
#define R2_CLIMB_DOWN_TRIGGER_HEIGHT_MIN_MM 65
#define R2_CLIMB_DOWN_LASER_APPROACH_SPEED_MPS 0.08f
#define R2_CLIMB_DOWN_LASER_APPROACH_TIMEOUT_MS 8000U
#define R2_CLIMB_DOWN_LASER_INVALID_TIMEOUT_MS 1000U
#define R2_CLIMB_DOWN_APPROACH_FORWARD_M   0.005f
#define R2_CLIMB_DOWN_APPROACH_TIMEOUT_MS 4000U
#define R2_CLIMB_LEG_TIMEOUT_SPEED_MM_S   40.0f
#define R2_CLIMB_LEG_TIMEOUT_MARGIN_MS    3000U
#define R2_CLIMB_LEG_TIMEOUT_MS(distance_mm) \
    ((uint32_t)((((distance_mm) * 1000.0f) / \
                 R2_CLIMB_LEG_TIMEOUT_SPEED_MM_S) + \
                (float)R2_CLIMB_LEG_TIMEOUT_MARGIN_MS))
#define R2_CLIMB_STEP_LONG_LEG_TIMEOUT_MS \
    R2_CLIMB_LEG_TIMEOUT_MS(R2_CLIMB_LEG_MAX_MM)
#define R2_CLIMB_PREPARE_TIMEOUT_MS \
    R2_CLIMB_LEG_TIMEOUT_MS(R2_CLIMB_LEG_MAX_MM + \
                            R2_CLIMB_AIR_CLEARANCE_MAX_MM)
#define R2_CLIMB_STEP_FRONT_ZERO_TIMEOUT_MS \
    R2_CLIMB_LEG_TIMEOUT_MS(200.0f)
#define R2_CLIMB_STEP_LEG_10_TIMEOUT_MS   R2_CLIMB_LEG_TIMEOUT_MS(10.0f)
#define R2_CLIMB_STEP_LEG_20_TIMEOUT_MS   R2_CLIMB_LEG_TIMEOUT_MS(20.0f)
#define R2_CLIMB_STEP_LEG_30_TIMEOUT_MS   R2_CLIMB_LEG_TIMEOUT_MS(30.0f)
#define R2_CLIMB_STEP_LEG_40_TIMEOUT_MS   R2_CLIMB_LEG_TIMEOUT_MS(40.0f)
#define R2_CLIMB_DRIVE_TIMEOUT_SPEED_MM_S 80.0f
#define R2_CLIMB_DRIVE_TIMEOUT_MARGIN_MS 1500U
#define R2_CLIMB_DRIVE_TIMEOUT_MS(distance_mm) \
    ((uint32_t)((((distance_mm) * 1000.0f) / \
                 R2_CLIMB_DRIVE_TIMEOUT_SPEED_MM_S) + \
                (float)R2_CLIMB_DRIVE_TIMEOUT_MARGIN_MS))
#define R2_CLIMB_STEP_CHASSIS_100_TIMEOUT_MS 5000U
#define R2_CLIMB_STEP_CHASSIS_200_TIMEOUT_MS 8000U
#define R2_CLIMB_TEST_LEG_TIMEOUT_MS      5000U
#define R2_CLIMB_TEST_DRIVE_TIMEOUT_MS    4000U
#define R2_CLIMB_TEST_DRIVE_500_TIMEOUT_MS \
    R2_CLIMB_DRIVE_TIMEOUT_MS(R2_CLIMB_TEST_DRIVE_500_MM)
#define R2_CLIMB_TEST_CHASSIS_TIMEOUT_MS  5000U
#define R2_CLIMB_TEST_CHASSIS_300_TIMEOUT_MS 10000U

#define R2_CLIMB_ERR_TIMEOUT             0x01U
#define R2_CLIMB_ERR_PARAM_NOT_CONFIGURED 0x02U
#define R2_CLIMB_ERR_TEST_ACTION         0x04U
#define R2_CLIMB_ERR_FLOW_SWITCH         0x08U

#define R2_CLIMB_DEBUG_SOURCE_USART   0U
#define R2_CLIMB_DEBUG_SOURCE_USB     1U
#define R2_CLIMB_DEBUG_SOURCE_NONE    2U

#define R2_CLIMB_DRIVE_GROUP_FRONT    0x01U
#define R2_CLIMB_DRIVE_GROUP_REAR     0x02U
#define R2_CLIMB_DRIVE_GROUP_ALL \
    ((uint8_t)(R2_CLIMB_DRIVE_GROUP_FRONT | R2_CLIMB_DRIVE_GROUP_REAR))

typedef enum
{
    R2_CLIMB_STATE_IDLE = 0,
    R2_CLIMB_STATE_STEP_1 = 1,
    R2_CLIMB_STATE_DONE = R2_CLIMB_STATE_DONE_ID,
    R2_CLIMB_STATE_ERROR = R2_CLIMB_STATE_ERROR_ID,
    R2_CLIMB_STATE_PREPARE = R2_CLIMB_STATE_PREPARE_ID,
    R2_CLIMB_STATE_UP_APPROACH = R2_CLIMB_STATE_UP_APPROACH_ID,
    R2_CLIMB_STATE_UP_LASER_APPROACH =
        R2_CLIMB_STATE_UP_LASER_APPROACH_ID,
    R2_CLIMB_STATE_DOWN_LASER_APPROACH =
        R2_CLIMB_STATE_DOWN_LASER_APPROACH_ID,
    R2_CLIMB_STATE_DOWN_APPROACH =
        R2_CLIMB_STATE_DOWN_APPROACH_ID,
} R2_ClimbState_t;

typedef enum
{
    R2_CLIMB_FLOW_UPSTAIRS = 0,
    R2_CLIMB_FLOW_DOWNSTAIRS = 1,
} R2_ClimbFlow_t;

typedef enum
{
    R2_CLIMB_TEST_NONE = 0,
    R2_CLIMB_TEST_ALL_LEGS_220,
    R2_CLIMB_TEST_ALL_LEGS_UP_10,
    R2_CLIMB_TEST_ALL_LEGS_DOWN_10,
    R2_CLIMB_TEST_REAR_DRIVE_FORWARD_30,
    R2_CLIMB_TEST_REAR_DRIVE_FORWARD_10,
    R2_CLIMB_TEST_REAR_DRIVE_BACKWARD_10,
    R2_CLIMB_TEST_FRONT_ZERO,
    R2_CLIMB_TEST_FRONT_UP_10,
    R2_CLIMB_TEST_FRONT_DOWN_10,
    R2_CLIMB_TEST_CHASSIS_FORWARD_100,
    R2_CLIMB_TEST_CHASSIS_FORWARD_50,
    R2_CLIMB_TEST_CHASSIS_BACKWARD_50,
    R2_CLIMB_TEST_REAR_ZERO,
    R2_CLIMB_TEST_REAR_UP_10,
    R2_CLIMB_TEST_REAR_DOWN_10,
    R2_CLIMB_TEST_ALL_LEGS_ZERO,
    R2_CLIMB_TEST_REAR_DRIVE_BACKWARD_30,
    R2_CLIMB_TEST_REAR_DRIVE_FORWARD_500,
    R2_CLIMB_TEST_REAR_DRIVE_BACKWARD_500,
    R2_CLIMB_TEST_CHASSIS_BACKWARD_100,
    R2_CLIMB_TEST_CHASSIS_FORWARD_300,
    R2_CLIMB_TEST_CHASSIS_BACKWARD_300,
    R2_CLIMB_TEST_FRONT_220,
    R2_CLIMB_TEST_FRONT_MINUS_30,
    R2_CLIMB_TEST_REAR_220,
    R2_CLIMB_TEST_REAR_MINUS_30,
    R2_CLIMB_TEST_FRONT_DRIVE_FORWARD_30,
    R2_CLIMB_TEST_FRONT_DRIVE_FORWARD_10,
    R2_CLIMB_TEST_FRONT_DRIVE_BACKWARD_10,
    R2_CLIMB_TEST_FRONT_DRIVE_BACKWARD_30,
    R2_CLIMB_TEST_FRONT_DRIVE_FORWARD_500,
    R2_CLIMB_TEST_FRONT_DRIVE_BACKWARD_500,
    R2_CLIMB_TEST_ALL_DRIVE_FORWARD_30,
    R2_CLIMB_TEST_ALL_DRIVE_FORWARD_10,
    R2_CLIMB_TEST_ALL_DRIVE_BACKWARD_10,
    R2_CLIMB_TEST_ALL_DRIVE_BACKWARD_30,
    R2_CLIMB_TEST_ALL_DRIVE_FORWARD_500,
    R2_CLIMB_TEST_ALL_DRIVE_BACKWARD_500,
} R2_ClimbTestAction_t;

typedef struct
{
    int16_t leg[4];          /* FDCAN2 ID 1..4 */
    int16_t front_drive[4];  /* FDCAN1 ID 5..8, only 5..6 are used */
    int16_t drive[4];        /* FDCAN2 ID 5..8, only 5..6 are used */
} R2_ClimbMotorCmd_t;

typedef struct
{
    R2_ClimbState_t state;
    R2_ClimbState_t auto_pause_resume_state;
    uint8_t enabled;
    uint8_t auto_run;
    uint8_t state_done;
    uint8_t error_flags;
    uint8_t flow;
    uint8_t pending_flow;
    uint8_t pending_step;
    uint8_t pending_auto;
    uint8_t pending_gate;
    uint8_t pending_resume;
    uint8_t gate_active;
    uint8_t auto_pause_enabled;
    uint8_t auto_pause_active;
    uint8_t auto_pause_step;
    uint8_t zero_captured;
    uint8_t pending_test_action;
    uint8_t test_action;
    uint8_t test_active;
    uint8_t test_chassis_active;
    uint8_t last_step_level;
    uint8_t last_auto_level;
    uint8_t up_laser_invalid_active;
    uint8_t down_laser_invalid_active;
    uint8_t drive_group_mask;

    uint32_t state_start_ms;
    uint32_t last_update_ms;
    uint32_t up_laser_invalid_start_ms;
    uint32_t down_laser_invalid_start_ms;

    int32_t leg_zero[4];
    int32_t front_drive_segment_start[2];
    int32_t drive_segment_start[2];

    float leg_target_mm[4];
    float drive_target_mm[2];
    float leg_pos_mm[4];
    float front_drive_pos_mm[2];
    float rear_drive_pos_mm[2];
    float drive_pos_mm[2];
} R2_Climb_Ctrl_t;

typedef struct
{
    uint8_t source;
    uint8_t state;
    uint8_t enabled;
    uint8_t auto_run;
    uint8_t state_done;
    uint8_t error_flags;
    uint8_t flow;
    uint8_t is_motor_active;
    uint8_t pending_step;
    uint8_t pending_auto;
    uint8_t auto_pause_active;
    uint8_t param_ready;
    uint8_t drive_group_mask;

    uint32_t state_start_ms;
    uint32_t last_update_ms;
    uint32_t elapsed_ms;
    const char *state_name;

    float leg_pos_mm[4];
    float leg_target_mm[4];
    float front_drive_pos_mm[2];
    float rear_drive_pos_mm[2];
    float drive_pos_mm[2];
    float drive_target_mm[2];
    float leg_count_per_mm;
    float drive_count_per_mm;
    float leg_dir[4];
    float drive_dir[2];

    int32_t leg_zero[4];
    int32_t front_drive_segment_start[2];
    int32_t drive_segment_start[2];

    int16_t leg_current[4];
    int16_t front_drive_current[2];
    int16_t drive_current[2];
} R2_ClimbDebug_t;

extern volatile R2_ClimbDebug_t g_r2_climb_debug_usart;
extern volatile R2_ClimbDebug_t g_r2_climb_debug_usb;
extern volatile R2_ClimbDebug_t g_r2_climb_debug_active;

void R2_Climb_Init(R2_Climb_Ctrl_t *ctrl);
void R2_Climb_Stop(R2_Climb_Ctrl_t *ctrl);
void R2_Climb_SetInput(R2_Climb_Ctrl_t *ctrl,
                       uint8_t enable_level,
                       uint8_t step_level,
                       uint8_t auto_level);
void R2_Climb_RequestStep(R2_Climb_Ctrl_t *ctrl);
void R2_Climb_RequestAuto(R2_Climb_Ctrl_t *ctrl);
void R2_Climb_RequestFlowGate(R2_Climb_Ctrl_t *ctrl, uint8_t flow);
void R2_Climb_RequestFlowStep(R2_Climb_Ctrl_t *ctrl, uint8_t flow);
void R2_Climb_RequestFlowAuto(R2_Climb_Ctrl_t *ctrl, uint8_t flow);
void R2_Climb_RequestFlowAutoPause(R2_Climb_Ctrl_t *ctrl, uint8_t flow);
void R2_Climb_RequestAutoResume(R2_Climb_Ctrl_t *ctrl);
void R2_Climb_RequestTestAction(R2_Climb_Ctrl_t *ctrl, uint8_t action);
void R2_Climb_Update(R2_Climb_Ctrl_t *ctrl,
                     R2_Move_Ctrl_t *move_ctrl,
                     uint32_t now_ms);
uint8_t R2_Climb_IsMotorActive(const R2_Climb_Ctrl_t *ctrl);
uint8_t R2_Climb_TestActionUsesChassis(uint8_t action);
uint32_t R2_Climb_GetDownstairsStepCount(void);
const char *R2_Climb_GetDownstairsStepName(uint8_t step);
void R2_Climb_GetMotorCurrent(const R2_Climb_Ctrl_t *ctrl,
                              R2_ClimbMotorCmd_t *cmd);
void R2_Climb_UpdateDebugViews(const R2_Climb_Ctrl_t *usart_ctrl,
                               const R2_Climb_Ctrl_t *usb_ctrl,
                               uint8_t active_source);
void R2_Climb_SetDebugMotorCurrent(uint8_t source,
                                   const R2_ClimbMotorCmd_t *cmd);

#endif
