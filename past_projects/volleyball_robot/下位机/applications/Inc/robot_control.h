#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include <stdint.h>

typedef enum {
    CHASSIS_STOP = 0,
    CHASSIS_SPEED_LOCAL = 1,
    CHASSIS_POSITION_WORLD = 2
} chassis_mode_t;

typedef enum {
    ROBOT_CMD_NONE = 0x0000,
    ROBOT_CMD_ODOMETRY_RESET = 0x0004,
    ROBOT_CMD_DELTA_STRIKE = 0x0010,
    ROBOT_CMD_DELTA_HOME = 0x0011,
    ROBOT_CMD_ROD_PREPARE = 0x0020,
    ROBOT_CMD_ROD_HOME = 0x0021,
    ROBOT_CMD_ROD_STRIKE = 0x0022,
    ROBOT_CMD_EMERGENCY_STOP = 0x00ff
} robot_command_t;

typedef enum {
    ROBOT_RESULT_OK = 0,
    ROBOT_RESULT_BUSY = 1,
    ROBOT_RESULT_INVALID = 2,
    ROBOT_RESULT_NOT_READY = 3,
    ROBOT_RESULT_FAULT = 4,
    ROBOT_RESULT_TIMEOUT = 5
} robot_result_t;

typedef enum {
    STRIKER_NONE = 0,
    STRIKER_DELTA = 1,
    STRIKER_ROD = 2
} striker_t;

typedef enum {
    ROD_WAITING_FEEDBACK = 0,
    ROD_ZEROING,
    ROD_PREPARING,
    ROD_READY,
    ROD_HOMING,
    ROD_AT_HOME,
    ROD_STRIKING,
    ROD_RECOVERING,
    ROD_STOPPED,
    ROD_FAULT
} rod_state_t;

#define ROBOT_HOLDING_HEARTBEAT 0x0000U
#define ROBOT_HOLDING_COMMAND 0x0001U
#define ROBOT_HOLDING_CHASSIS_MODE 0x0010U
#define ROBOT_HOLDING_CHASSIS_TARGET 0x0012U
#define ROBOT_HOLDING_DELTA_TARGET 0x0020U
#define ROBOT_HOLDING_DELTA_TIMEOUT 0x0026U

#define ROBOT_INPUT_PROTOCOL_VERSION 0x0000U
#define ROBOT_INPUT_STATUS_FLAGS 0x0001U
#define ROBOT_INPUT_ACTIVE_STRIKER 0x0002U
#define ROBOT_INPUT_LAST_COMMAND 0x0003U
#define ROBOT_INPUT_LAST_RESULT 0x0004U
#define ROBOT_INPUT_DELTA_STATE 0x0005U
#define ROBOT_INPUT_ROD_STATE 0x0006U
#define ROBOT_INPUT_CHASSIS_STATE 0x0007U
#define ROBOT_INPUT_ODOMETRY 0x0010U
#define ROBOT_INPUT_CHASSIS_SPEED 0x0016U
#define ROBOT_INPUT_DELTA_JOINTS 0x0020U
#define ROBOT_INPUT_ROD_ANGLE 0x0030U

extern volatile uint8_t run_mode;
extern volatile uint8_t change_mode_flag;
extern volatile uint32_t host_last_rx_ms;
extern float receive_data[6];

void robot_control_init(void);
void robot_control_service(void);
uint8_t robot_chassis_locked(void);
uint8_t robot_host_online(void);
robot_result_t robot_execute_command(uint16_t command);
uint8_t robot_holding_read(uint16_t address, uint16_t count, uint16_t *values);
uint8_t robot_holding_write(uint16_t address, uint16_t count, const uint16_t *values);
uint8_t robot_input_read(uint16_t address, uint16_t count, uint16_t *values);

#endif
