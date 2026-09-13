#ifndef MODBUS_TEST_INCLUDE_H
#define MODBUS_TEST_INCLUDE_H

#include <stdint.h>
#include <string.h>
#include "modbus_rtu.h"

#define MODBUS_SLAVE_ADDRESS 1U
#define MODBUS_FRAME_TIMEOUT_MS 10U

uint8_t robot_holding_read(uint16_t address, uint16_t count, uint16_t *values);
uint8_t robot_holding_write(uint16_t address, uint16_t count, const uint16_t *values);
uint8_t robot_input_read(uint16_t address, uint16_t count, uint16_t *values);
uint32_t HAL_GetTick(void);

static inline uint32_t __get_PRIMASK(void) { return 0U; }
static inline void __disable_irq(void) {}
static inline void __set_PRIMASK(uint32_t value) { (void)value; }

#endif
