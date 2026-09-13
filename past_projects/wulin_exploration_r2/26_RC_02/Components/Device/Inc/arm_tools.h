#ifndef __ARM_TOOLS_H
#define __ARM_TOOLS_H

#include "include.h"
#include "arm_user.h"
#include <stdint.h>

//控制信号源
#define TOOL_USART_SOURCE 0U
#define TOOL_USB_SOURCE   1U

#define TOOL_DEV_CLAMP    0U
#define TOOL_DEV_CHUCK    1U

/* FDCAN3 id4 M2006 switches tools.
 * Motor encoder → 36:1 gearbox output shaft (330°) → 18T:33T external → tool gear (180°)
 * TOOL_MOTOR_REDUCTION only needs the M2006 internal 36:1; the 33/18 external ratio
 * is already baked into TOOL_CHUCK_OUTPUT_DEG (330° at gearbox output = 180° at tool). */
#define TOOL_MOTOR_ENCODER_CPR       8192.0f
#define TOOL_MOTOR_REDUCTION         36.0f
#define TOOL_MOTOR_DEG_TO_COUNT      (TOOL_MOTOR_ENCODER_CPR * TOOL_MOTOR_REDUCTION / 360.0f)
#define TOOL_CHUCK_OUTPUT_DEG        330.0f
#define TOOL_SWITCH_TOL_OUTPUT_DEG   2.0f
#define TOOL_SWITCH_TIMEOUT_MS       2000U

//工具切换角度与电机实际转动角度
// 工具旋转：夹爪位为零，吸盘位为减速箱输出 330 度；开合由独立执行器负责。
#define CLAMP_TARGET_ANGLE           0.0f
#define CHUCK_TARGET_ANGLE           (TOOL_CHUCK_OUTPUT_DEG * TOOL_MOTOR_DEG_TO_COUNT)
#define TOOL_SWITCH_TOLERANCE        (TOOL_SWITCH_TOL_OUTPUT_DEG * TOOL_MOTOR_DEG_TO_COUNT)

/* TIM1 is configured to 1 MHz, so CCR equals pulse width in microseconds.
 * Clamp close is the initial 0 deg position. Clamp open is 180 deg clockwise.
 */
#define CLAMP_SERVO_MIN_DEG          0.0f
#define CLAMP_SERVO_MAX_DEG          180.0f
#define CLAMP_SERVO_0DEG_PULSE_US    2500U
#define CLAMP_SERVO_180DEG_PULSE_US  500U
#define CLAMP_SERVO_CLOSE_DEG        0.0f
#define CLAMP_SERVO_OPEN_DEG         180.0f

//夹爪坐标偏置
#define CLAMP_X_OFFEST 0.0f
#define CLAMP_Y_OFFEST 0.0f
#define CLAMP_Z_OFFEXT 0.0f

//夹爪控制状态
#define CLAMP_OPEN     1U
#define CLAMP_CLOSE    0U

//吸盘坐标偏置
#define CHUCK_X_OFFEST 0.0f
#define CHUCK_Y_OFFEST 0.0f
#define CHUCK_Z_OFFEXT 0.0f

//吸盘控制状态
#define CHUCK_OPEN     1U
#define CHUCK_CLOSE    0U

//安全标志
#define SAFE_NO        0U
#define SAFE_YES       1U

// 定义工作状态枚举
typedef enum {
    TOOL_STATUS_IDLE = 0,    // 空闲/待机状态
    TOOL_STATUS_MOVING,      // 正在运动中
    TOOL_STATUS_ERROR        // 发生错误（如超时卡堵）
} Tool_RunStatus_t;

typedef struct{
    uint8_t state;//0: close, 1: open
    uint8_t control_source[2]; //0: USART, 1: USB
    float real_angle;
    float target_angle;
    uint8_t safe_flag; //0: unsafe, 1: safe

    Tool_RunStatus_t run_status; // 状态机的当前运行状态
    uint8_t pending_state;       // 正在前往的目标状态 (OPEN或CLOSE)
    uint32_t start_tick;         // 记录动作开始的系统节拍，用于计算超时
}clamp_Handle_t;

typedef struct{
    uint8_t state;//0: close, 1: open
    uint8_t control_source[2]; //0: USART, 1: USB
    float real_angle;
    float target_angle;
    uint8_t safe_flag; //0: unsafe, 1: safe

    Tool_RunStatus_t run_status; // 状态机的当前运行状态
    uint8_t pending_state;       // 正在前往的目标状态 (OPEN或CLOSE)
    uint32_t start_tick;         // 记录动作开始的系统节拍，用于计算超时
}chuck_Handle_t;

extern clamp_Handle_t clamp_usart;
extern clamp_Handle_t clamp_usb;
extern chuck_Handle_t chuck_usart;
extern chuck_Handle_t chuck_usb;
extern uint8_t tool_dev_usart;
extern uint8_t tool_dev_usb;

extern void clamp_init(clamp_Handle_t *clamp);
extern void chuck_init(chuck_Handle_t *chuck);
extern void set_clamp_controlSource(clamp_Handle_t *clamp, uint8_t source);
extern void set_chuck_controlSource(chuck_Handle_t *chuck, uint8_t source);
extern int update_clamp_real_angle(clamp_Handle_t *clamp, float angle);
extern int update_chuck_real_angle(chuck_Handle_t *chuck, float angle);
extern void set_clamp_target_angle(clamp_Handle_t *clamp, float angle);
extern void set_chuck_target_angle(chuck_Handle_t *chuck, float angle);
extern int get_clamp_safe_flag(clamp_Handle_t *clamp);
extern int get_chuck_safe_flag(chuck_Handle_t *chuck);
extern void Tool_SetClampActuator(clamp_Handle_t *clamp, uint8_t target_state);
extern void Tool_SetChuckActuator(chuck_Handle_t *chuck, uint8_t target_state);
extern void trigger_clamp_action(clamp_Handle_t *clamp, uint8_t target_state);
extern void trigger_chuck_action(chuck_Handle_t *chuck, uint8_t target_state);
extern void clamp_state_machine_run(clamp_Handle_t *clamp);
extern void chuck_state_machine_run(chuck_Handle_t *chuck);

void Tool_InitAll(void);
void Tool_SetActiveSource(uint8_t source);
uint8_t Tool_GetActiveSource(void);
clamp_Handle_t *Tool_GetClamp(uint8_t source);
chuck_Handle_t *Tool_GetChuck(uint8_t source);
clamp_Handle_t *Tool_GetActiveClamp(void);
chuck_Handle_t *Tool_GetActiveChuck(void);
void Tool_SetSelectedDev(uint8_t source, uint8_t dev);
uint8_t Tool_GetSelectedDev(uint8_t source);
uint8_t Tool_GetActiveSelectedDev(void);
void Tool_RunActiveStateMachine(void);
void Tool_StopSource(uint8_t source);
void Tool_HoldSource(uint8_t source);
float Tool_GetActiveTargetAngle(void);



#endif /* __ARM_TOOLS_H */
