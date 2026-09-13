#ifndef ROBOT_TEST_INCLUDE_H
#define ROBOT_TEST_INCLUDE_H

#include <math.h>
#include <stdint.h>
#include <string.h>
#include "control_config.h"
#include "delta_clac.h"
#include "robot_control.h"

typedef struct {
    int32_t total_angle;
    int16_t speed_rpm;
    uint32_t last_rx_ms;
    uint8_t initialized;
} motor_measure_t;

typedef struct {
    float angle;
    float speed;
} mi_feedback_t;

typedef struct {
    mi_feedback_t RxCAN_info;
    uint32_t last_rx_ms;
    uint8_t feedback_valid;
} MI_Motor_s;

extern MI_Motor_s MI_Motor[9];
extern int MI_CAN_2;
extern motor_measure_t motor_can1[8];
extern float solution_matrix[3][3];
extern float displace_buffer[3];
extern volatile delta_state_t delta_state;

uint32_t HAL_GetTick(void);
void MI_motor_Init(MI_Motor_s *motor, int *bus, uint8_t id);
void MI_motor_Enable(MI_Motor_s *motor);
void MI_motor_SetMechPositionToZero(MI_Motor_s *motor);
void MI_motor_Control(MI_Motor_s *motor, float torque, float position,
                      float speed, float kp, float kd);
void MI_motor_Stop(MI_Motor_s *motor);
void reset_motor_position(motor_measure_t *motor, uint8_t count);
void rnd_count_and_diaplacement_reset(void);
uint8_t DeltaInverseKinematics(float x, float y, float z, float joint_deg[3]);
uint8_t delta_start_strike(float x, float y, float z, uint16_t timeout);
uint8_t delta_start_home(void);
void delta_force_home(void);
void delta_emergency_stop(void);
uint8_t delta_is_busy(void);
uint8_t delta_at_home(void);
uint8_t delta_feedback_ready(void);
void delta_get_joint_angles(float joint_deg[3]);
uint8_t imu_input_fresh(void);
uint8_t motor_feedback_online(const motor_measure_t *motor, uint32_t now);
uint8_t motor_feedback_snapshot(const motor_measure_t *motor, motor_measure_t *snapshot);
void matrix_multiply(float matrix[3][3], float input[3], float output[3]);

#endif
