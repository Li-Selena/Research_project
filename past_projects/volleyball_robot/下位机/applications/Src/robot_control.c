#include "include.h"
#include "can.h"

#define HOLDING_COUNT 0x27U
#define INPUT_COUNT 0x32U
#define PROTOCOL_VERSION 0x0100U

volatile uint8_t run_mode = CHASSIS_STOP;
volatile uint8_t change_mode_flag;
volatile uint32_t host_last_rx_ms;
float receive_data[6];

static uint16_t holding[HOLDING_COUNT];
static volatile striker_t active_striker;
static volatile rod_state_t rod_state;
static volatile robot_result_t last_result;
static volatile uint16_t last_command;
static uint32_t rod_state_started_ms;
static uint32_t rod_last_command_ms;
static uint8_t host_was_online;

static uint8_t finite3(const float value[3])
{
    return isfinite(value[0]) && isfinite(value[1]) && isfinite(value[2]);
}

static void float_to_registers(float value, uint16_t out[2])
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    out[0] = (uint16_t)(bits >> 16);
    out[1] = (uint16_t)bits;
}

static float registers_to_float(const uint16_t in[2])
{
    uint32_t bits = ((uint32_t)in[0] << 16) | in[1];
    float value;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

static uint8_t rod_online(uint32_t now)
{
    return MI_Motor[2].feedback_valid &&
           (uint32_t)(now - MI_Motor[2].last_rx_ms) <= MOTOR_TIMEOUT_MS;
}

static uint8_t rod_reached(float target)
{
    return fabsf(MI_Motor[2].RxCAN_info.angle - target) <= ROD_POSITION_TOLERANCE_RAD &&
           fabsf(MI_Motor[2].RxCAN_info.speed) <= ROD_STATIONARY_RAD_S;
}

static void rod_send_hold(float target)
{
    MI_motor_Control(&MI_Motor[2], 0.0f, target, 0.0f, ROD_HOLD_KP, ROD_HOLD_KD);
}

static void stop_chassis(void)
{
    memset(receive_data, 0, 3U * sizeof(float));
    if (run_mode != CHASSIS_STOP) {
        run_mode = CHASSIS_STOP;
        change_mode_flag = 1;
    }
}

static void rod_begin(rod_state_t state)
{
    rod_state = state;
    rod_state_started_ms = HAL_GetTick();
    rod_last_command_ms = 0;
    active_striker = STRIKER_ROD;
}

static void rod_force_prepare(void)
{
    if (rod_state != ROD_STOPPED && rod_state != ROD_WAITING_FEEDBACK && rod_state != ROD_ZEROING)
        rod_begin(ROD_RECOVERING);
}

uint8_t robot_host_online(void)
{
    return host_last_rx_ms != 0U &&
           (uint32_t)(HAL_GetTick() - host_last_rx_ms) <= HOST_TIMEOUT_MS;
}

uint8_t robot_chassis_locked(void) { return active_striker != STRIKER_NONE; }

void robot_control_init(void)
{
    uint16_t encoded[2];
    memset(holding, 0, sizeof(holding));
    memset(receive_data, 0, sizeof(receive_data));
    float_to_registers(DELTA_DEFAULT_STRIKE_X_M * 1000.0f, encoded);
    holding[0x20] = encoded[0]; holding[0x21] = encoded[1];
    float_to_registers(DELTA_DEFAULT_STRIKE_Y_M * 1000.0f, encoded);
    holding[0x22] = encoded[0]; holding[0x23] = encoded[1];
    float_to_registers(DELTA_DEFAULT_STRIKE_Z_M * 1000.0f, encoded);
    holding[0x24] = encoded[0]; holding[0x25] = encoded[1];
    holding[ROBOT_HOLDING_DELTA_TIMEOUT] = DELTA_STRIKE_TIMEOUT_MS;
    run_mode = CHASSIS_STOP;
    active_striker = STRIKER_NONE;
    rod_state = ROD_WAITING_FEEDBACK;
    last_result = ROBOT_RESULT_OK;
    last_command = ROBOT_CMD_NONE;
    host_last_rx_ms = 0;
    host_was_online = 0;
    MI_motor_Init(&MI_Motor[2], &MI_CAN_2, 2);
    MI_motor_Enable(&MI_Motor[2]);
}

static void rod_service(void)
{
    uint32_t now = HAL_GetTick();
    if (!rod_online(now)) {
        if (rod_state != ROD_WAITING_FEEDBACK && rod_state != ROD_ZEROING && rod_state != ROD_STOPPED)
            rod_state = ROD_FAULT;
        if (active_striker == STRIKER_ROD) active_striker = STRIKER_NONE;
        return;
    }

    if (rod_state == ROD_WAITING_FEEDBACK) {
        MI_motor_SetMechPositionToZero(&MI_Motor[2]);
        rod_state = ROD_ZEROING;
        rod_state_started_ms = now;
        return;
    }
    if (rod_state == ROD_ZEROING) {
        if ((uint32_t)(now - rod_state_started_ms) >= ROD_ZERO_SETTLE_MS)
            rod_begin(ROD_PREPARING);
        return;
    }

    if ((uint32_t)(now - rod_last_command_ms) >= 20U) {
        if (rod_state == ROD_PREPARING || rod_state == ROD_READY || rod_state == ROD_RECOVERING)
            rod_send_hold(ROD_PREPARE_RAD);
        else if (rod_state == ROD_HOMING || rod_state == ROD_AT_HOME)
            rod_send_hold(ROD_HOME_RAD);
        else if (rod_state == ROD_STRIKING)
            MI_motor_Control(&MI_Motor[2], ROD_STRIKE_TORQUE_NM, ROD_STRIKE_RAD,
                             ROD_STRIKE_SPEED_RAD_S, ROD_STRIKE_KP, ROD_STRIKE_KD);
        rod_last_command_ms = now;
    }

    if (rod_state == ROD_PREPARING || rod_state == ROD_RECOVERING) {
        if (rod_reached(ROD_PREPARE_RAD)) {
            rod_state = ROD_READY;
            if (active_striker == STRIKER_ROD) active_striker = STRIKER_NONE;
        } else if ((uint32_t)(now - rod_state_started_ms) > ROD_MOVE_TIMEOUT_MS) {
            MI_motor_Stop(&MI_Motor[2]);
            rod_state = ROD_FAULT;
            active_striker = STRIKER_NONE;
            last_result = ROBOT_RESULT_TIMEOUT;
        }
    } else if (rod_state == ROD_HOMING) {
        if (rod_reached(ROD_HOME_RAD)) {
            rod_state = ROD_AT_HOME;
            active_striker = STRIKER_NONE;
        } else if ((uint32_t)(now - rod_state_started_ms) > ROD_MOVE_TIMEOUT_MS) {
            MI_motor_Stop(&MI_Motor[2]);
            rod_state = ROD_FAULT;
            active_striker = STRIKER_NONE;
            last_result = ROBOT_RESULT_TIMEOUT;
        }
    } else if (rod_state == ROD_STRIKING &&
               (rod_reached(ROD_STRIKE_RAD) ||
                (uint32_t)(now - rod_state_started_ms) >= ROD_STRIKE_TIMEOUT_MS)) {
        rod_begin(ROD_RECOVERING);
    }
}

void robot_control_service(void)
{
    uint8_t online = robot_host_online();
    if (host_was_online && !online) {
        stop_chassis();
        if (active_striker == STRIKER_DELTA) {
            delta_force_home();
        } else if (active_striker == STRIKER_ROD) {
            rod_force_prepare();
        } else {
            delta_force_home();
            if (!delta_is_busy()) rod_force_prepare();
        }
        last_result = ROBOT_RESULT_TIMEOUT;
    }
    host_was_online = online;
    rod_service();
    if (active_striker == STRIKER_DELTA && !delta_is_busy()) {
        if (delta_at_home()) active_striker = STRIKER_NONE;
        else if (delta_state == DELTA_STATE_FAULT) {
            active_striker = STRIKER_NONE;
            last_result = ROBOT_RESULT_FAULT;
        }
    }
}

robot_result_t robot_execute_command(uint16_t command)
{
    robot_result_t result = ROBOT_RESULT_OK;
    float strike_mm[3];
    last_command = command;

    switch (command) {
    case ROBOT_CMD_NONE:
        break;
    case ROBOT_CMD_ODOMETRY_RESET:
        stop_chassis();
        reset_motor_position(motor_can1, 3);
        rnd_count_and_diaplacement_reset();
        break;
    case ROBOT_CMD_DELTA_STRIKE:
        if (active_striker != STRIKER_NONE) result = ROBOT_RESULT_BUSY;
        else if (!delta_feedback_ready() || !delta_at_home()) result = ROBOT_RESULT_NOT_READY;
        else {
            for (uint8_t i = 0; i < 3; ++i)
                strike_mm[i] = registers_to_float(&holding[ROBOT_HOLDING_DELTA_TARGET + 2U * i]);
            if (!delta_start_strike(strike_mm[0] / 1000.0f, strike_mm[1] / 1000.0f,
                                    strike_mm[2] / 1000.0f,
                                    holding[ROBOT_HOLDING_DELTA_TIMEOUT]))
                result = ROBOT_RESULT_INVALID;
            else {
                stop_chassis();
                active_striker = STRIKER_DELTA;
            }
        }
        break;
    case ROBOT_CMD_DELTA_HOME:
        if (active_striker != STRIKER_NONE) result = ROBOT_RESULT_BUSY;
        else if (!delta_start_home()) result = ROBOT_RESULT_NOT_READY;
        else {
            active_striker = STRIKER_DELTA;
            stop_chassis();
        }
        break;
    case ROBOT_CMD_ROD_PREPARE:
        if (active_striker != STRIKER_NONE) result = ROBOT_RESULT_BUSY;
        else if (!rod_online(HAL_GetTick()) || rod_state == ROD_STOPPED) result = ROBOT_RESULT_NOT_READY;
        else if (rod_state != ROD_READY) rod_begin(ROD_PREPARING);
        break;
    case ROBOT_CMD_ROD_HOME:
        if (active_striker != STRIKER_NONE) result = ROBOT_RESULT_BUSY;
        else if (!rod_online(HAL_GetTick()) || rod_state == ROD_STOPPED) result = ROBOT_RESULT_NOT_READY;
        else if (rod_state != ROD_AT_HOME) rod_begin(ROD_HOMING);
        break;
    case ROBOT_CMD_ROD_STRIKE:
        if (active_striker != STRIKER_NONE) result = ROBOT_RESULT_BUSY;
        else if (rod_state != ROD_READY || !rod_online(HAL_GetTick())) result = ROBOT_RESULT_NOT_READY;
        else {
            stop_chassis();
            rod_begin(ROD_STRIKING);
        }
        break;
    case ROBOT_CMD_EMERGENCY_STOP:
        stop_chassis();
        delta_emergency_stop();
        MI_motor_Stop(&MI_Motor[2]);
        rod_state = ROD_STOPPED;
        active_striker = STRIKER_NONE;
        break;
    default:
        result = ROBOT_RESULT_INVALID;
        break;
    }
    last_result = result;
    return result;
}

static uint8_t holding_address_valid(uint16_t address)
{
    return address <= 0x0001U ||
           (address >= ROBOT_HOLDING_CHASSIS_MODE && address <= 0x0017U) ||
           (address >= ROBOT_HOLDING_DELTA_TARGET && address <= ROBOT_HOLDING_DELTA_TIMEOUT);
}

uint8_t robot_holding_read(uint16_t address, uint16_t count, uint16_t *values)
{
    if (count == 0U || (uint32_t)address + count > HOLDING_COUNT) return 0;
    for (uint16_t i = 0; i < count; ++i) {
        if (!holding_address_valid((uint16_t)(address + i))) return 0;
        values[i] = holding[address + i];
    }
    return 1;
}

static uint8_t apply_chassis_block(void)
{
    float target[3];
    uint16_t mode = holding[ROBOT_HOLDING_CHASSIS_MODE];
    for (uint8_t i = 0; i < 3; ++i)
        target[i] = registers_to_float(&holding[ROBOT_HOLDING_CHASSIS_TARGET + 2U * i]);
    if (!finite3(target) || mode > CHASSIS_POSITION_WORLD) return 0;
    if (mode == CHASSIS_SPEED_LOCAL &&
        (fabsf(target[0]) > 4000.0f || fabsf(target[1]) > 4000.0f || fabsf(target[2]) > 720.0f))
        return 0;
    if (mode == CHASSIS_POSITION_WORLD &&
        (fabsf(target[0]) > 100000.0f || fabsf(target[1]) > 100000.0f || fabsf(target[2]) > 36000.0f))
        return 0;
    if (robot_chassis_locked() && mode != CHASSIS_STOP) return 0;
    memcpy(receive_data, target, sizeof(target));
    if (run_mode != mode) {
        run_mode = (uint8_t)mode;
        change_mode_flag = 1;
    }
    return 1;
}

static uint8_t apply_delta_block(void)
{
    float target_mm[3];
    float joint[3];
    uint16_t timeout = holding[ROBOT_HOLDING_DELTA_TIMEOUT];
    for (uint8_t i = 0; i < 3; ++i)
        target_mm[i] = registers_to_float(&holding[ROBOT_HOLDING_DELTA_TARGET + 2U * i]);
    if (!finite3(target_mm) || timeout < 100U || timeout > 1000U) return 0;
    return DeltaInverseKinematics(target_mm[0] / 1000.0f, target_mm[1] / 1000.0f,
                                  target_mm[2] / 1000.0f, joint);
}

uint8_t robot_holding_write(uint16_t address, uint16_t count, const uint16_t *values)
{
    uint16_t backup[8];
    if (count == 0U || count > 8U || (uint32_t)address + count > HOLDING_COUNT) return 0;
    for (uint16_t i = 0; i < count; ++i)
        if (!holding_address_valid((uint16_t)(address + i))) return 0;
    memcpy(backup, &holding[address], count * sizeof(uint16_t));
    memcpy(&holding[address], values, count * sizeof(uint16_t));

    if (address == ROBOT_HOLDING_HEARTBEAT && count == 1U) {
        host_last_rx_ms = HAL_GetTick();
        return 1;
    }
    if (address == ROBOT_HOLDING_COMMAND && count == 1U) {
        robot_execute_command(values[0]);
        holding[ROBOT_HOLDING_COMMAND] = 0;
        return 1;
    }
    if (address == ROBOT_HOLDING_CHASSIS_MODE && count == 8U) {
        if (apply_chassis_block()) return 1;
    } else if (address == ROBOT_HOLDING_DELTA_TARGET && count == 7U) {
        if (apply_delta_block()) return 1;
    } else {
        memcpy(&holding[address], backup, count * sizeof(uint16_t));
        return 0;
    }
    memcpy(&holding[address], backup, count * sizeof(uint16_t));
    return 0;
}

static void measured_chassis_speed(float speed[3])
{
    float wheel[3];
    float body[3];
    motor_measure_t feedback;
    for (uint8_t i = 0; i < 3; ++i) {
        wheel[i] = motor_feedback_snapshot(&motor_can1[i], &feedback) ? feedback.speed_rpm : 0.0f;
    }
    matrix_multiply(solution_matrix, wheel, body);
    speed[0] = body[0] / CHASSIS_RPM_PER_MM_S;
    speed[1] = body[1] / CHASSIS_RPM_PER_MM_S;
    speed[2] = body[2] / CHASSIS_RPM_PER_RAD_S * 180.0f / CONTROL_PI;
}

static uint16_t input_register(uint16_t address)
{
    uint16_t flags = 0;
    uint32_t now = HAL_GetTick();

    if (address == ROBOT_INPUT_PROTOCOL_VERSION) return PROTOCOL_VERSION;
    if (address == ROBOT_INPUT_STATUS_FLAGS) {
        if (robot_host_online()) flags |= 1U << 0;
        if (imu_input_fresh()) flags |= 1U << 1;
        if (motor_feedback_online(&motor_can1[0], now) && motor_feedback_online(&motor_can1[1], now) &&
            motor_feedback_online(&motor_can1[2], now)) flags |= 1U << 2;
        if (delta_feedback_ready()) flags |= 1U << 3;
        if (rod_online(now)) flags |= 1U << 4;
        if (delta_state == DELTA_STATE_FAULT || rod_state == ROD_FAULT) flags |= 1U << 5;
        return flags;
    }
    if (address == ROBOT_INPUT_ACTIVE_STRIKER) return (uint16_t)active_striker;
    if (address == ROBOT_INPUT_LAST_COMMAND) return last_command;
    if (address == ROBOT_INPUT_LAST_RESULT) return (uint16_t)last_result;
    if (address == ROBOT_INPUT_DELTA_STATE) return (uint16_t)delta_state;
    if (address == ROBOT_INPUT_ROD_STATE) return (uint16_t)rod_state;
    if (address == ROBOT_INPUT_CHASSIS_STATE) return run_mode;
    return 0;
}

static uint8_t input_address_valid(uint16_t address)
{
    return address <= ROBOT_INPUT_CHASSIS_STATE ||
           (address >= ROBOT_INPUT_ODOMETRY && address < ROBOT_INPUT_ODOMETRY + 12U) ||
           (address >= ROBOT_INPUT_DELTA_JOINTS && address < ROBOT_INPUT_DELTA_JOINTS + 6U) ||
           (address >= ROBOT_INPUT_ROD_ANGLE && address < ROBOT_INPUT_ROD_ANGLE + 2U);
}

uint8_t robot_input_read(uint16_t address, uint16_t count, uint16_t *values)
{
    float odometry[3];
    float speed[3];
    float joints[3];
    float rod_angle;
    uint16_t pair[2];
    if (count == 0U || count > 125U || (uint32_t)address + count > INPUT_COUNT) return 0;
    for (uint16_t i = 0; i < count; ++i)
        if (!input_address_valid((uint16_t)(address + i))) return 0;

    memcpy(odometry, displace_buffer, sizeof(odometry));
    measured_chassis_speed(speed);
    delta_get_joint_angles(joints);
    rod_angle = MI_Motor[2].RxCAN_info.angle * 180.0f / CONTROL_PI;

    for (uint16_t i = 0; i < count; ++i) {
        uint16_t current = (uint16_t)(address + i);
        if (current <= ROBOT_INPUT_CHASSIS_STATE) {
            values[i] = input_register(current);
        } else if (current >= ROBOT_INPUT_ODOMETRY && current < ROBOT_INPUT_ODOMETRY + 6U) {
            uint16_t offset = (uint16_t)(current - ROBOT_INPUT_ODOMETRY);
            float_to_registers(odometry[offset / 2U], pair);
            values[i] = pair[offset & 1U];
        } else if (current >= ROBOT_INPUT_CHASSIS_SPEED && current < ROBOT_INPUT_CHASSIS_SPEED + 6U) {
            uint16_t offset = (uint16_t)(current - ROBOT_INPUT_CHASSIS_SPEED);
            float_to_registers(speed[offset / 2U], pair);
            values[i] = pair[offset & 1U];
        } else if (current >= ROBOT_INPUT_DELTA_JOINTS && current < ROBOT_INPUT_DELTA_JOINTS + 6U) {
            uint16_t offset = (uint16_t)(current - ROBOT_INPUT_DELTA_JOINTS);
            float_to_registers(joints[offset / 2U], pair);
            values[i] = pair[offset & 1U];
        } else {
            uint16_t offset = (uint16_t)(current - ROBOT_INPUT_ROD_ANGLE);
            float_to_registers(rod_angle, pair);
            values[i] = pair[offset];
        }
    }
    return 1;
}
