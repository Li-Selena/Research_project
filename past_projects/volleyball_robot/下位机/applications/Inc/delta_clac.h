#ifndef DELTA_CALC_H
#define DELTA_CALC_H

#include <stdint.h>

typedef enum { DELTA_DISABLED = 0, DELTA_TRACK_CARTESIAN = 1 } delta_mode_t;
typedef enum {
    DELTA_STATUS_DISABLED = 0, DELTA_STATUS_WAITING_FEEDBACK,
    DELTA_STATUS_READY, DELTA_STATUS_UNREACHABLE,
    DELTA_STATUS_FEEDBACK_LOST, DELTA_STATUS_COMMAND_TIMEOUT
} delta_status_t;
typedef enum {
    DELTA_STATE_WAITING_FEEDBACK = 0, DELTA_STATE_HOLD_HOME,
    DELTA_STATE_STRIKING, DELTA_STATE_RETURNING,
    DELTA_STATE_STOPPED, DELTA_STATE_FAULT
} delta_state_t;

extern float delta_position[3];
extern float D_theta[3];
extern volatile uint8_t delta_mode;
extern volatile uint8_t delta_zero_requested;
extern volatile delta_status_t delta_status;
extern volatile delta_state_t delta_state;
extern volatile uint32_t delta_target_last_ms;

uint8_t DeltaInverseKinematics(float x, float y, float z, float joint_deg[3]);
void delta_init(void);
void delta_set_cartesian_target(float x, float y, float z);
void delta_jog_z(float dz);
void delta_request_zero(void);
void delta_control(void);
void delta_disable(void);
uint8_t delta_start_strike(float x_m, float y_m, float z_m, uint16_t timeout_ms);
uint8_t delta_start_home(void);
void delta_force_home(void);
void delta_emergency_stop(void);
uint8_t delta_is_busy(void);
uint8_t delta_at_home(void);
uint8_t delta_feedback_ready(void);
void delta_get_joint_angles(float joint_deg[3]);

#endif
