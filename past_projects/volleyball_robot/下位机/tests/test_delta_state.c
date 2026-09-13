#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "include.h"

motor_measure_t motor_can1[6];
static uint32_t tick_ms = 100U;
static float motor_current[7];

uint32_t HAL_GetTick(void) { return tick_ms; }

uint8_t motor_feedback_online(const motor_measure_t *motor, uint32_t now)
{
    return motor->initialized && (uint32_t)(now - motor->last_rx_ms) <= MOTOR_TIMEOUT_MS;
}

uint8_t motor_feedback_snapshot(const motor_measure_t *motor, motor_measure_t *snapshot)
{
    *snapshot = *motor;
    return motor->initialized;
}

void reset_motor_position(motor_measure_t *motor, uint8_t count)
{
    for (uint8_t i = 0; i < count; ++i) motor[i].total_angle = 0;
}

void CAN1_SetMotorCurrent(uint8_t id, float current)
{
    if (id < 7U) motor_current[id] = current;
}

static void set_feedback(float joint_deg, float rpm)
{
    int32_t total = (int32_t)lroundf(joint_deg / 360.0f * 8192.0f * DELTA_GEAR_RATIO);
    for (uint8_t i = 3; i < 6; ++i) {
        motor_can1[i].initialized = 1U;
        motor_can1[i].last_rx_ms = tick_ms;
        motor_can1[i].total_angle = total;
        motor_can1[i].speed_rpm = (int16_t)rpm;
    }
}

int main(void)
{
    float strike_joint[3];

    delta_init();
    delta_control();
    assert(delta_state == DELTA_STATE_WAITING_FEEDBACK);

    set_feedback(0.0f, 0.0f);
    delta_control();
    assert(delta_state == DELTA_STATE_HOLD_HOME);
    assert(delta_at_home());

    assert(!delta_start_strike(0.0f, 0.0f, 0.6f, 300U));
    assert(!delta_start_strike(0.0f, 0.0f, 0.4f, 99U));
    assert(DeltaInverseKinematics(0.0f, 0.0f, 0.4f, strike_joint));
    assert(delta_start_strike(0.0f, 0.0f, 0.4f, 300U));
    assert(delta_state == DELTA_STATE_STRIKING);

    tick_ms += 300U;
    set_feedback(0.0f, 0.0f);
    delta_control();
    assert(delta_state == DELTA_STATE_RETURNING);
    delta_control();
    assert(delta_state == DELTA_STATE_HOLD_HOME);

    assert(delta_start_strike(0.0f, 0.0f, 0.4f, 300U));
    set_feedback(strike_joint[0], 0.0f);
    delta_control();
    assert(delta_state == DELTA_STATE_RETURNING);

    set_feedback(0.0f, 0.0f);
    delta_control();
    assert(delta_state == DELTA_STATE_HOLD_HOME);

    assert(delta_start_strike(0.0f, 0.0f, 0.4f, 300U));
    motor_can1[4].initialized = 0U;
    delta_control();
    assert(delta_state == DELTA_STATE_WAITING_FEEDBACK || delta_state == DELTA_STATE_FAULT);
    assert(motor_current[4] == 0.0f && motor_current[5] == 0.0f && motor_current[6] == 0.0f);

    delta_emergency_stop();
    assert(delta_state == DELTA_STATE_STOPPED);
    puts("delta state tests passed");
    return 0;
}
