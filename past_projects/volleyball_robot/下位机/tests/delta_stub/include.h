#ifndef DELTA_TEST_INCLUDE_H
#define DELTA_TEST_INCLUDE_H

#include <math.h>
#include <stdint.h>
#include <string.h>
#include "control_config.h"
#include "delta_clac.h"

typedef struct {
    int32_t total_angle;
    int16_t speed_rpm;
    uint32_t last_rx_ms;
    uint8_t initialized;
} motor_measure_t;

extern motor_measure_t motor_can1[6];
uint32_t HAL_GetTick(void);
uint8_t motor_feedback_online(const motor_measure_t *motor, uint32_t now);
uint8_t motor_feedback_snapshot(const motor_measure_t *motor, motor_measure_t *snapshot);
void reset_motor_position(motor_measure_t *motor, uint8_t count);
void CAN1_SetMotorCurrent(uint8_t id, float current);

static inline uint32_t __get_PRIMASK(void) { return 0U; }
static inline void __disable_irq(void) {}
static inline void __set_PRIMASK(uint32_t value) { (void)value; }

#endif
