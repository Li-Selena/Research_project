#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "include.h"

MI_Motor_s MI_Motor[9];
int MI_CAN_2;
motor_measure_t motor_can1[8];
float solution_matrix[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
float displace_buffer[3];
volatile delta_state_t delta_state = DELTA_STATE_HOLD_HOME;
static uint32_t tick_ms = 100U;
static uint8_t delta_busy;
static uint8_t motor_stopped;

uint32_t HAL_GetTick(void) { return tick_ms; }
void MI_motor_Init(MI_Motor_s *motor, int *bus, uint8_t id) {(void)motor;(void)bus;(void)id;}
void MI_motor_Enable(MI_Motor_s *motor) {(void)motor;}
void MI_motor_SetMechPositionToZero(MI_Motor_s *motor) { motor->RxCAN_info.angle = 0.0f; }
void MI_motor_Control(MI_Motor_s *motor, float torque, float position,
                      float speed, float kp, float kd)
{
    (void)torque;(void)speed;(void)kp;(void)kd;
    motor->RxCAN_info.angle = position;
    motor->RxCAN_info.speed = 0.0f;
}
void MI_motor_Stop(MI_Motor_s *motor) {(void)motor; motor_stopped = 1U;}
void reset_motor_position(motor_measure_t *motor, uint8_t count) {(void)motor;(void)count;}
void rnd_count_and_diaplacement_reset(void) { memset(displace_buffer, 0, sizeof(displace_buffer)); }
uint8_t DeltaInverseKinematics(float x,float y,float z,float joint[3])
{(void)x;(void)y;if(z<0.15f||z>0.5f)return 0;memset(joint,0,3*sizeof(float));return 1;}
uint8_t delta_start_strike(float x,float y,float z,uint16_t timeout)
{(void)x;(void)y;(void)z;(void)timeout;delta_busy=1;delta_state=DELTA_STATE_STRIKING;return 1;}
uint8_t delta_start_home(void) {delta_busy=1;delta_state=DELTA_STATE_RETURNING;return 1;}
void delta_force_home(void) {if(delta_state!=DELTA_STATE_HOLD_HOME){delta_busy=1;delta_state=DELTA_STATE_RETURNING;}}
void delta_emergency_stop(void) {delta_busy=0;delta_state=DELTA_STATE_STOPPED;}
uint8_t delta_is_busy(void) {return delta_busy;}
uint8_t delta_at_home(void) {return delta_state==DELTA_STATE_HOLD_HOME;}
uint8_t delta_feedback_ready(void) {return 1;}
void delta_get_joint_angles(float joint[3]) {memset(joint,0,3*sizeof(float));}
uint8_t imu_input_fresh(void) {return 1;}
uint8_t motor_feedback_online(const motor_measure_t *motor,uint32_t now)
{return motor->initialized && now-motor->last_rx_ms<=MOTOR_TIMEOUT_MS;}
uint8_t motor_feedback_snapshot(const motor_measure_t *motor,motor_measure_t *snapshot)
{*snapshot=*motor;return motor->initialized;}
void matrix_multiply(float matrix[3][3],float input[3],float output[3])
{for(uint8_t i=0;i<3;++i)output[i]=matrix[i][0]*input[0]+matrix[i][1]*input[1]+matrix[i][2]*input[2];}

static uint16_t input_value(uint16_t address)
{
    uint16_t value;
    assert(robot_input_read(address, 1U, &value));
    return value;
}

static void make_rod_online(void)
{
    MI_Motor[2].feedback_valid = 1U;
    MI_Motor[2].last_rx_ms = tick_ms;
    MI_Motor[2].RxCAN_info.speed = 0.0f;
}

static void reach_rod_ready(void)
{
    make_rod_online();
    robot_control_service();
    assert(input_value(ROBOT_INPUT_ROD_STATE) == ROD_ZEROING);
    tick_ms += ROD_ZERO_SETTLE_MS;
    make_rod_online();
    robot_control_service();
    assert(input_value(ROBOT_INPUT_ROD_STATE) == ROD_PREPARING);
    robot_control_service();
    assert(input_value(ROBOT_INPUT_ROD_STATE) == ROD_READY);
    assert(input_value(ROBOT_INPUT_ACTIVE_STRIKER) == STRIKER_NONE);
}

int main(void)
{
    uint16_t heartbeat = 1U;

    for (uint8_t i=0;i<3;++i) {motor_can1[i].initialized=1;motor_can1[i].last_rx_ms=tick_ms;}
    robot_control_init();
    reach_rod_ready();

    assert(robot_execute_command(ROBOT_CMD_ROD_STRIKE) == ROBOT_RESULT_OK);
    assert(input_value(ROBOT_INPUT_ACTIVE_STRIKER) == STRIKER_ROD);
    assert(robot_execute_command(ROBOT_CMD_DELTA_STRIKE) == ROBOT_RESULT_BUSY);
    robot_control_service();
    assert(input_value(ROBOT_INPUT_ROD_STATE) == ROD_RECOVERING);
    robot_control_service();
    assert(input_value(ROBOT_INPUT_ROD_STATE) == ROD_READY);

    assert(robot_execute_command(ROBOT_CMD_DELTA_STRIKE) == ROBOT_RESULT_OK);
    assert(robot_execute_command(ROBOT_CMD_ROD_STRIKE) == ROBOT_RESULT_BUSY);
    delta_busy = 0U;
    delta_state = DELTA_STATE_HOLD_HOME;
    robot_control_service();
    assert(input_value(ROBOT_INPUT_ACTIVE_STRIKER) == STRIKER_NONE);

    assert(robot_holding_write(ROBOT_HOLDING_HEARTBEAT, 1U, &heartbeat));
    robot_control_service();
    assert(robot_host_online());
    assert(robot_execute_command(ROBOT_CMD_ROD_STRIKE) == ROBOT_RESULT_OK);
    tick_ms += HOST_TIMEOUT_MS + 1U;
    make_rod_online();
    robot_control_service();
    assert(run_mode == CHASSIS_STOP);
    assert(input_value(ROBOT_INPUT_ROD_STATE) == ROD_READY);
    assert(input_value(ROBOT_INPUT_LAST_RESULT) == ROBOT_RESULT_TIMEOUT);

    assert(robot_execute_command(ROBOT_CMD_EMERGENCY_STOP) == ROBOT_RESULT_OK);
    assert(motor_stopped);
    assert(input_value(ROBOT_INPUT_ROD_STATE) == ROD_STOPPED);
    assert(delta_state == DELTA_STATE_STOPPED);

    puts("robot mutual exclusion and watchdog tests passed");
    return 0;
}
