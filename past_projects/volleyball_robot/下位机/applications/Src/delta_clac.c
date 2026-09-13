#include "include.h"

float delta_position[3];
float D_theta[3];
volatile uint8_t delta_mode = DELTA_TRACK_CARTESIAN;
volatile uint8_t delta_zero_requested;
volatile delta_status_t delta_status = DELTA_STATUS_WAITING_FEEDBACK;
volatile delta_state_t delta_state = DELTA_STATE_WAITING_FEEDBACK;
volatile uint32_t delta_target_last_ms;

static float commanded_joint_deg[3];
static float speed_integral[3];
static float previous_speed_error[3];
static uint8_t delta_control_active;
static uint32_t strike_started_ms;
static uint16_t strike_timeout_ms = DELTA_STRIKE_TIMEOUT_MS;
static const float motor_direction[3] = DELTA_MOTOR_DIRECTIONS;
static const float calibration_angle_deg[3] = DELTA_CALIBRATION_ANGLES_DEG;

static float clampf(float value, float low, float high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static void clear_pid(void)
{
    memset(speed_integral, 0, sizeof(speed_integral));
    memset(previous_speed_error, 0, sizeof(previous_speed_error));
}

static void set_home_target(void)
{
    delta_position[0] = DELTA_HOME_X_M;
    delta_position[1] = DELTA_HOME_Y_M;
    delta_position[2] = DELTA_HOME_Z_M;
}

void delta_init(void)
{
    set_home_target();
    memset(D_theta, 0, sizeof(D_theta));
    memset(commanded_joint_deg, 0, sizeof(commanded_joint_deg));
    clear_pid();
    delta_control_active = 0;
    delta_mode = DELTA_TRACK_CARTESIAN;
    delta_state = DELTA_STATE_WAITING_FEEDBACK;
    delta_status = DELTA_STATUS_WAITING_FEEDBACK;
    delta_target_last_ms = 0;
}

void delta_set_cartesian_target(float x, float y, float z)
{
    uint32_t irq = __get_PRIMASK();
    __disable_irq();
    delta_position[0] = x;
    delta_position[1] = y;
    delta_position[2] = z;
    delta_target_last_ms = HAL_GetTick();
    __set_PRIMASK(irq);
}

void delta_jog_z(float dz)
{
    float target[3];
    float joint[3];
    uint32_t irq = __get_PRIMASK();
    __disable_irq();
    memcpy(target, delta_position, sizeof(target));
    __set_PRIMASK(irq);
    target[2] += dz;
    if (target[2] > 0.0f && DeltaInverseKinematics(target[0], target[1], target[2], joint))
        delta_set_cartesian_target(target[0], target[1], target[2]);
}

void delta_request_zero(void) { delta_zero_requested = 1; }

uint8_t delta_feedback_ready(void)
{
    uint32_t now = HAL_GetTick();
    for (uint8_t i = 0; i < 3; ++i)
        if (!motor_feedback_online(&motor_can1[i + 3], now)) return 0;
    return 1;
}

void delta_get_joint_angles(float joint_deg[3])
{
    motor_measure_t feedback;
    for (uint8_t i = 0; i < 3; ++i) {
        if (!motor_feedback_snapshot(&motor_can1[i + 3], &feedback)) joint_deg[i] = 0.0f;
        else joint_deg[i] = calibration_angle_deg[i] + motor_direction[i] *
            (float)feedback.total_angle * 360.0f / (8192.0f * DELTA_GEAR_RATIO);
    }
}

uint8_t delta_start_strike(float x_m, float y_m, float z_m, uint16_t timeout_ms)
{
    float joint[3];
    if (delta_state != DELTA_STATE_HOLD_HOME || !delta_feedback_ready()) return 0;
    if (!DeltaInverseKinematics(x_m, y_m, z_m, joint)) return 0;
    if (timeout_ms < 100U || timeout_ms > 1000U) return 0;
    delta_set_cartesian_target(x_m, y_m, z_m);
    strike_timeout_ms = timeout_ms;
    strike_started_ms = HAL_GetTick();
    delta_state = DELTA_STATE_STRIKING;
    delta_mode = DELTA_TRACK_CARTESIAN;
    return 1;
}

uint8_t delta_start_home(void)
{
    if (!delta_feedback_ready() || delta_state == DELTA_STATE_STOPPED) return 0;
    set_home_target();
    delta_state = DELTA_STATE_RETURNING;
    delta_mode = DELTA_TRACK_CARTESIAN;
    return 1;
}

void delta_force_home(void)
{
    if (delta_state != DELTA_STATE_STOPPED && delta_state != DELTA_STATE_HOLD_HOME) {
        set_home_target();
        delta_state = delta_feedback_ready() ? DELTA_STATE_RETURNING : DELTA_STATE_WAITING_FEEDBACK;
        delta_mode = DELTA_TRACK_CARTESIAN;
    }
}

uint8_t delta_is_busy(void)
{
    return delta_state == DELTA_STATE_STRIKING || delta_state == DELTA_STATE_RETURNING;
}

uint8_t delta_at_home(void) { return delta_state == DELTA_STATE_HOLD_HOME; }

void delta_emergency_stop(void)
{
    delta_mode = DELTA_DISABLED;
    delta_state = DELTA_STATE_STOPPED;
    delta_status = DELTA_STATUS_DISABLED;
    clear_pid();
    delta_control_active = 0;
    for (uint8_t id = 4; id <= 6; ++id) CAN1_SetMotorCurrent(id, 0.0f);
}

void delta_disable(void) { delta_emergency_stop(); }

void delta_control(void)
{
    motor_measure_t feedback[3];
    float target_joint[3];
    float target_position[3];
    float slew_deg_s = DELTA_JOINT_SLEW_DEG_S;
    float max_motor_rpm = DELTA_MAX_MOTOR_RPM;
    float max_joint_error = 0.0f;
    float max_feedback_rpm = 0.0f;
    uint32_t now = HAL_GetTick();

    if (delta_mode == DELTA_DISABLED || delta_state == DELTA_STATE_STOPPED) {
        for (uint8_t id = 4; id <= 6; ++id) CAN1_SetMotorCurrent(id, 0.0f);
        return;
    }

    for (uint8_t i = 0; i < 3; ++i) {
        if (!motor_feedback_snapshot(&motor_can1[i + 3], &feedback[i]) ||
            (uint32_t)(now - feedback[i].last_rx_ms) > MOTOR_TIMEOUT_MS) {
            delta_status = feedback[i].initialized ? DELTA_STATUS_FEEDBACK_LOST : DELTA_STATUS_WAITING_FEEDBACK;
            delta_state = feedback[i].initialized ? DELTA_STATE_FAULT : DELTA_STATE_WAITING_FEEDBACK;
            clear_pid();
            delta_control_active = 0;
            for (uint8_t id = 4; id <= 6; ++id) CAN1_SetMotorCurrent(id, 0.0f);
            return;
        }
    }

    if (delta_zero_requested) {
        reset_motor_position(&motor_can1[3], 3);
        memset(commanded_joint_deg, 0, sizeof(commanded_joint_deg));
        clear_pid();
        delta_zero_requested = 0;
        delta_control_active = 1;
        set_home_target();
        delta_state = DELTA_STATE_RETURNING;
    }

    if (!delta_control_active) {
        delta_get_joint_angles(commanded_joint_deg);
        clear_pid();
        delta_control_active = 1;
        if (delta_state == DELTA_STATE_WAITING_FEEDBACK || delta_state == DELTA_STATE_FAULT) {
            set_home_target();
            delta_state = DELTA_STATE_RETURNING;
        }
    }

    if (delta_state == DELTA_STATE_STRIKING) {
        slew_deg_s = DELTA_STRIKE_SLEW_DEG_S;
        max_motor_rpm = DELTA_STRIKE_MAX_MOTOR_RPM;
    } else if (delta_state == DELTA_STATE_RETURNING) {
        slew_deg_s = DELTA_RETURN_SLEW_DEG_S;
        max_motor_rpm = DELTA_RETURN_MAX_MOTOR_RPM;
    } else if (delta_state == DELTA_STATE_HOLD_HOME) {
        set_home_target();
    }

    uint32_t irq = __get_PRIMASK();
    __disable_irq();
    memcpy(target_position, delta_position, sizeof(target_position));
    __set_PRIMASK(irq);
    if (!DeltaInverseKinematics(target_position[0], target_position[1],
                                target_position[2], target_joint)) {
        delta_status = DELTA_STATUS_UNREACHABLE;
        delta_state = DELTA_STATE_FAULT;
        clear_pid();
        for (uint8_t id = 4; id <= 6; ++id) CAN1_SetMotorCurrent(id, 0.0f);
        return;
    }

    for (uint8_t i = 0; i < 3; ++i) {
        float step = slew_deg_s * CONTROL_DT_S;
        float target_delta = clampf(target_joint[i] - commanded_joint_deg[i], -step, step);
        float current_joint_deg;
        float target_motor_rpm;
        float rpm_error;
        float derivative;
        float unsaturated;
        float current;
        float joint_error;

        commanded_joint_deg[i] += target_delta;
        current_joint_deg = calibration_angle_deg[i] + motor_direction[i] *
            (float)feedback[i].total_angle * 360.0f / (8192.0f * DELTA_GEAR_RATIO);
        target_motor_rpm = motor_direction[i] *
            ((commanded_joint_deg[i] - current_joint_deg) * DELTA_POSITION_KP +
             (target_delta / CONTROL_DT_S) * DELTA_GEAR_RATIO / 6.0f);
        target_motor_rpm = clampf(target_motor_rpm, -max_motor_rpm, max_motor_rpm);
        rpm_error = target_motor_rpm - feedback[i].speed_rpm;
        speed_integral[i] = clampf(speed_integral[i] + DELTA_SPEED_KI * rpm_error,
                                   -DELTA_PID_INTEGRAL_LIMIT, DELTA_PID_INTEGRAL_LIMIT);
        derivative = rpm_error - previous_speed_error[i];
        unsaturated = DELTA_SPEED_KP * rpm_error + speed_integral[i] + DELTA_SPEED_KD * derivative;
        current = clampf(unsaturated, -DELTA_PID_OUTPUT_LIMIT, DELTA_PID_OUTPUT_LIMIT);
        if (current != unsaturated)
            speed_integral[i] = clampf(current - DELTA_SPEED_KP * rpm_error -
                                       DELTA_SPEED_KD * derivative,
                                       -DELTA_PID_INTEGRAL_LIMIT, DELTA_PID_INTEGRAL_LIMIT);
        previous_speed_error[i] = rpm_error;
        CAN1_SetMotorCurrent(i + 4, current);
        D_theta[i] = target_joint[i];
        joint_error = fabsf(target_joint[i] - current_joint_deg);
        if (joint_error > max_joint_error) max_joint_error = joint_error;
        if (fabsf((float)feedback[i].speed_rpm) > max_feedback_rpm)
            max_feedback_rpm = fabsf((float)feedback[i].speed_rpm);
    }

    if (delta_state == DELTA_STATE_STRIKING &&
        ((max_joint_error <= DELTA_STRIKE_TOLERANCE_DEG &&
          max_feedback_rpm <= 2.0f * DELTA_STATIONARY_RPM) ||
         (uint32_t)(now - strike_started_ms) >= strike_timeout_ms)) {
        set_home_target();
        delta_state = DELTA_STATE_RETURNING;
    } else if (delta_state == DELTA_STATE_RETURNING &&
               max_joint_error <= DELTA_HOME_TOLERANCE_DEG &&
               max_feedback_rpm <= DELTA_STATIONARY_RPM) {
        delta_state = DELTA_STATE_HOLD_HOME;
    }
    delta_status = DELTA_STATUS_READY;
}
