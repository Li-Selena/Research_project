#include "arm_tools.h"
#include "bsp_tick.h"
#include "fdcan_receive.h"
#include "tim.h"
#include <stdint.h>
#include <math.h>
#include "FreeRTOS.h"
#include "task.h"

//工具切换电机，只有电机motor_fdcan3[4]，两个工具固定在电机上
extern motor_measure_t motor_fdcan3[8];
extern TIM_HandleTypeDef htim1;

//工具句柄：USART 与 USB 各自保存独立状态，当前控制源只决定哪套目标驱动物理电机
clamp_Handle_t clamp_usart;
clamp_Handle_t clamp_usb;
chuck_Handle_t chuck_usart;
chuck_Handle_t chuck_usb;
uint8_t tool_dev_usart = TOOL_DEV_CLAMP;
uint8_t tool_dev_usb = TOOL_DEV_CLAMP;

static uint8_t s_tool_active_source = TOOL_USART_SOURCE;

static void Tool_HoldClamp(clamp_Handle_t *clamp);
static void Tool_HoldChuck(chuck_Handle_t *chuck);
static uint32_t Tool_ClampServoDegToPulse(float deg);
static void Tool_SetClampServo(uint8_t state);
static void Tool_SetChuckSucker(uint8_t state);
static void Tool_StartSwitch(uint8_t source, uint8_t dev);

static uint32_t Tool_ClampServoDegToPulse(float deg)
{
    float ratio;
    int32_t pulse_delta;
    int32_t pulse;

    if (deg < CLAMP_SERVO_MIN_DEG) {
        deg = CLAMP_SERVO_MIN_DEG;
    } else if (deg > CLAMP_SERVO_MAX_DEG) {
        deg = CLAMP_SERVO_MAX_DEG;
    }

    ratio = (deg - CLAMP_SERVO_MIN_DEG) /
            (CLAMP_SERVO_MAX_DEG - CLAMP_SERVO_MIN_DEG);
    pulse_delta = (int32_t)CLAMP_SERVO_180DEG_PULSE_US -
                  (int32_t)CLAMP_SERVO_0DEG_PULSE_US;
    pulse = (int32_t)CLAMP_SERVO_0DEG_PULSE_US +
            (int32_t)(ratio * (float)pulse_delta + 0.5f);

    if (pulse < 0) {
        pulse = 0;
    }

    return (uint32_t)pulse;
}

static void Tool_SetClampServo(uint8_t state)
{
    float deg = (state == CLAMP_OPEN) ?
                CLAMP_SERVO_OPEN_DEG :
                CLAMP_SERVO_CLOSE_DEG;
    uint32_t pulse = Tool_ClampServoDegToPulse(deg);

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse);
}

static void Tool_SetChuckSucker(uint8_t state)
{
    HAL_GPIO_WritePin(GPIOE,
                      GPIO_PIN_13,
                      (state == CHUCK_OPEN) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

//工具初始化
void clamp_init(clamp_Handle_t *clamp)
{
    if (clamp == 0) {
        return;
    }
    clamp->state = CLAMP_CLOSE;
    clamp->control_source[0] = 0U;          //USART
    clamp->control_source[1] = 0U;          //USB
    clamp->real_angle = motor_fdcan3[3].total_angle;
    clamp->target_angle = CLAMP_TARGET_ANGLE;
    clamp->safe_flag = 0U;
    clamp->run_status = TOOL_STATUS_IDLE;
    clamp->pending_state = CLAMP_CLOSE;
    clamp->start_tick = 0U;
}

void chuck_init(chuck_Handle_t *chuck)
{
    if (chuck == 0) {
        return;
    }
    chuck->state = CHUCK_CLOSE;
    chuck->control_source[0] = 0U;          //USART
    chuck->control_source[1] = 0U;          //USB
    chuck->real_angle = motor_fdcan3[3].total_angle;
    chuck->target_angle = CHUCK_TARGET_ANGLE;
    chuck->safe_flag = 0U;
    chuck->run_status = TOOL_STATUS_IDLE;
    chuck->pending_state = CHUCK_CLOSE;
    chuck->start_tick = 0U;
}
//设置工具控制信号源
void set_clamp_controlSource(clamp_Handle_t *clamp, uint8_t source)
{
    if (clamp == 0) {
        return;
    }
    if(source == TOOL_USART_SOURCE)
    {
        clamp->control_source[0] = 1U;
        clamp->control_source[1] = 0U;
    }
    else if(source == TOOL_USB_SOURCE)
    {
        clamp->control_source[0] = 0U;
        clamp->control_source[1] = 1U;
    }
}

void set_chuck_controlSource(chuck_Handle_t *chuck, uint8_t source)
{
    if (chuck == 0) {
        return;
    }
    if(source == TOOL_USART_SOURCE)
    {
        chuck->control_source[0] = 1U;
        chuck->control_source[1] = 0U;
    }
    else if(source == TOOL_USB_SOURCE)
    {
        chuck->control_source[0] = 0U;
        chuck->control_source[1] = 1U;
    }
}
//获取电机实际角度
int update_clamp_real_angle(clamp_Handle_t *clamp, float angle)
{
    if (clamp == 0) {
        return 0;
    }
    clamp->real_angle = angle;
    return 1;
}

int update_chuck_real_angle(chuck_Handle_t *chuck, float angle)
{
    if (chuck == 0) {
        return 0;
    }
    chuck->real_angle = angle;
    return 1;
}
//获取安全标志（是否达到目标角度）
int get_clamp_safe_flag(clamp_Handle_t *clamp)
{
    if (clamp == 0) {
        return SAFE_NO;
    }
    update_clamp_real_angle(clamp, motor_fdcan3[3].total_angle);
    if(fabsf(clamp->real_angle - clamp->target_angle) < TOOL_SWITCH_TOLERANCE)
    {
        clamp->safe_flag = 1U;
        return clamp->safe_flag;
    }
    else
    {
        clamp->safe_flag = 0U;
        return clamp->safe_flag;
    }
}

int get_chuck_safe_flag(chuck_Handle_t *chuck)
{
    if (chuck == 0) {
        return SAFE_NO;
    }
    update_chuck_real_angle(chuck, motor_fdcan3[3].total_angle);
    if(fabsf(chuck->real_angle - chuck->target_angle) < TOOL_SWITCH_TOLERANCE)
    {
        chuck->safe_flag = 1U;
        return chuck->safe_flag;
    }
    else
    {
        chuck->safe_flag = 0U;
        return chuck->safe_flag;
    }
}
//更新控制目标角度
void set_clamp_target_angle(clamp_Handle_t *clamp, float angle)
{
    if (clamp == 0) {
        return;
    }
    clamp->target_angle = angle;
}

void set_chuck_target_angle(chuck_Handle_t *chuck, float angle)
{
    if (chuck == 0) {
        return;
    }
    chuck->target_angle = angle;
}

static void Tool_HoldClamp(clamp_Handle_t *clamp)
{
    if (clamp == 0) {
        return;
    }

    update_clamp_real_angle(clamp, motor_fdcan3[3].total_angle);
    clamp->target_angle = clamp->real_angle;
}

static void Tool_HoldChuck(chuck_Handle_t *chuck)
{
    if (chuck == 0) {
        return;
    }

    update_chuck_real_angle(chuck, motor_fdcan3[3].total_angle);
    chuck->target_angle = chuck->real_angle;
}

void Tool_SetClampActuator(clamp_Handle_t *clamp, uint8_t target_state)
{
    if (clamp == 0) {
        return;
    }
    if ((target_state != CLAMP_OPEN) && (target_state != CLAMP_CLOSE)) {
        return;
    }

    Tool_SetClampServo(target_state);
    clamp->state = target_state;
    clamp->pending_state = target_state;

    if (clamp->run_status == TOOL_STATUS_ERROR) {
        return;
    }
    if (clamp->run_status != TOOL_STATUS_MOVING) {
        clamp->run_status = TOOL_STATUS_IDLE;
    }
}

void Tool_SetChuckActuator(chuck_Handle_t *chuck, uint8_t target_state)
{
    if (chuck == 0) {
        return;
    }
    if ((target_state != CHUCK_OPEN) && (target_state != CHUCK_CLOSE)) {
        return;
    }

    Tool_SetChuckSucker(target_state);
    chuck->state = target_state;
    chuck->pending_state = target_state;

    if (chuck->run_status == TOOL_STATUS_ERROR) {
        return;
    }
    if (chuck->run_status != TOOL_STATUS_MOVING) {
        chuck->run_status = TOOL_STATUS_IDLE;
    }
}
//更新使用控制状态
// int update_clamp_control_state(clamp_Handle_t *clamp,uint8_t state)
// {
//     if(state == CLAMP_OPEN)
//         set_clamp_target_angle(clamp, CLAMP_TARGET_ANGLE);
//     else if(state == CLAMP_CLOSE)
//         set_clamp_target_angle(clamp, 0.0f);
//     if(get_clamp_safe_flag(clamp) == SAFE_YES)
//     {
//         clamp->state = state;
//         return 1;
//     }
//     else
//         return 0;
// }
// int update_chuck_control_state(chuck_Handle_t *chuck,uint8_t state)
// {
//     if(state == CHUCK_OPEN)
//         set_chuck_target_angle(chuck, CHUCK_TARGET_ANGLE);
//     else if(state == CHUCK_CLOSE)
//         set_chuck_target_angle(chuck, 0.0f);

//     Delay_ms(1000);

//     if(get_chuck_safe_flag(chuck) == SAFE_YES)
//     {
//         chuck->state = state;
//         return 1;
//     }
//     else
//         return 0;
// }

void trigger_clamp_action(clamp_Handle_t *clamp, uint8_t target_state)
{
    if (clamp == 0) {
        return;
    }
    // 防御性编程：如果正在运动中，可以拒绝新指令，或者重置超时时间
    if(clamp->run_status == TOOL_STATUS_MOVING) {
        return; 
    }

    // 1. 下发目标角度
    if ((target_state != CLAMP_OPEN) && (target_state != CLAMP_CLOSE)) {
        return;
    }

    set_clamp_target_angle(clamp, CLAMP_TARGET_ANGLE);
    if (get_clamp_safe_flag(clamp) != SAFE_YES) {
        clamp->pending_state = clamp->state;
        clamp->start_tick = xTaskGetTickCount();
        clamp->run_status = TOOL_STATUS_MOVING;
        return;
    }

    Tool_SetClampServo(target_state);

    // 2. 更新状态机变量
    clamp->pending_state = target_state;             // 暂存目标状态
    clamp->start_tick = xTaskGetTickCount();         // 获取当前 FreeRTOS 系统 Tick
    clamp->run_status = TOOL_STATUS_MOVING;          // 引擎启动，切入运动状态
}

// 状态机步进函数（放在主循环或周期任务中高频调用）
void clamp_state_machine_run(clamp_Handle_t *clamp)
{
    if (clamp == 0) {
        return;
    }
    switch(clamp->run_status)
    {
        case TOOL_STATUS_IDLE:
            // 空闲状态，电机已到位，无需任何处理
            break;

        case TOOL_STATUS_MOVING:
            // 1. 检查是否已经安全到达目标位置
            if(get_clamp_safe_flag(clamp) == SAFE_YES)
            {
                clamp->state = clamp->pending_state;  // 真正确认状态已改变
                clamp->run_status = TOOL_STATUS_IDLE; // 回归空闲状态
                
                // 【拓展】可以在这里发送一帧串口数据给上位机，告知“执行完毕”
            }
            // 2. 超时检测：如果超过 1000ms 还没到位（比如夹到硬物卡死）
            else if((xTaskGetTickCount() - clamp->start_tick) > pdMS_TO_TICKS(TOOL_SWITCH_TIMEOUT_MS))
            {
                Tool_HoldClamp(clamp);
                clamp->run_status = TOOL_STATUS_ERROR; // 切入错误状态
                // 【拓展】可以在这里将 target_angle 设回当前真实角度，让电机卸力
            }
            break;

        case TOOL_STATUS_ERROR:
            // 发生错误，等待系统复位或新的干预指令
            // 如果收到复位指令，可以将 run_status 重新设为 IDLE
            break;
            
        default:
            clamp->run_status = TOOL_STATUS_IDLE;
            break;
    }
}

void trigger_chuck_action(chuck_Handle_t *chuck, uint8_t target_state)
{
    if (chuck == 0) {
        return;
    }
    // 防御性编程：如果正在运动中，可以拒绝新指令，或者重置超时时间
    if(chuck->run_status == TOOL_STATUS_MOVING) {
        return; 
    }

    // 1. 下发目标角度
    if ((target_state != CHUCK_OPEN) && (target_state != CHUCK_CLOSE)) {
        return;
    }

    set_chuck_target_angle(chuck, CHUCK_TARGET_ANGLE);
    if (get_chuck_safe_flag(chuck) != SAFE_YES) {
        chuck->pending_state = chuck->state;
        chuck->start_tick = xTaskGetTickCount();
        chuck->run_status = TOOL_STATUS_MOVING;
        return;
    }

    Tool_SetChuckSucker(target_state);

    // 2. 更新状态机变量
    chuck->pending_state = target_state;             // 暂存目标状态
    chuck->start_tick = xTaskGetTickCount();         // 获取当前 FreeRTOS 系统 Tick
    chuck->run_status = TOOL_STATUS_MOVING;          // 引擎启动，切入运动状态
}

// 状态机步进函数（放在主循环或周期任务中高频调用）
void chuck_state_machine_run(chuck_Handle_t *chuck)
{
    if (chuck == 0) {
        return;
    }
    switch(chuck->run_status)
    {
        case TOOL_STATUS_IDLE:
            // 空闲状态，电机已到位，无需任何处理
            break;

        case TOOL_STATUS_MOVING:
            // 1. 检查是否已经安全到达目标位置
            if(get_chuck_safe_flag(chuck) == SAFE_YES)
            {
                chuck->state = chuck->pending_state;  // 真正确认状态已改变
                chuck->run_status = TOOL_STATUS_IDLE; // 回归空闲状态
                
                // 【拓展】可以在这里发送一帧串口数据给上位机，告知“执行完毕”
            }
            // 2. 超时检测：如果超过 1000ms 还没到位（比如夹到硬物卡死）
            else if((xTaskGetTickCount() - chuck->start_tick) > pdMS_TO_TICKS(TOOL_SWITCH_TIMEOUT_MS))
            {
                Tool_HoldChuck(chuck);
                chuck->run_status = TOOL_STATUS_ERROR; // 切入错误状态
                // 【拓展】可以在这里将 target_angle 设回当前真实角度，让电机卸力
            }
            break;

        case TOOL_STATUS_ERROR:
            // 发生错误，等待系统复位或新的干预指令
            // 如果收到复位指令，可以将 run_status 重新设为 IDLE
            break;
            
        default:
            chuck->run_status = TOOL_STATUS_IDLE;
            break;
    }
}

static void Tool_StartSwitch(uint8_t source, uint8_t dev)
{
    clamp_Handle_t *tool_clamp;
    chuck_Handle_t *tool_chuck;

    if (source != TOOL_USB_SOURCE) {
        source = TOOL_USART_SOURCE;
    }

    tool_clamp = Tool_GetClamp(source);
    tool_chuck = Tool_GetChuck(source);

    set_clamp_target_angle(tool_clamp, CLAMP_TARGET_ANGLE);
    set_chuck_target_angle(tool_chuck, CHUCK_TARGET_ANGLE);

    if (dev == TOOL_DEV_CHUCK) {
        tool_clamp->run_status = TOOL_STATUS_IDLE;
        if (tool_chuck->run_status == TOOL_STATUS_MOVING) {
            return;
        }
        tool_chuck->pending_state = tool_chuck->state;
        tool_chuck->start_tick = xTaskGetTickCount();
        tool_chuck->run_status = (get_chuck_safe_flag(tool_chuck) == SAFE_YES) ?
                                 TOOL_STATUS_IDLE : TOOL_STATUS_MOVING;
    } else {
        tool_chuck->run_status = TOOL_STATUS_IDLE;
        if (tool_clamp->run_status == TOOL_STATUS_MOVING) {
            return;
        }
        tool_clamp->pending_state = tool_clamp->state;
        tool_clamp->start_tick = xTaskGetTickCount();
        tool_clamp->run_status = (get_clamp_safe_flag(tool_clamp) == SAFE_YES) ?
                                 TOOL_STATUS_IDLE : TOOL_STATUS_MOVING;
    }
}

void Tool_InitAll(void)
{
    clamp_init(&clamp_usart);
    clamp_init(&clamp_usb);
    chuck_init(&chuck_usart);
    chuck_init(&chuck_usb);

    (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    Tool_SetClampServo(CLAMP_CLOSE);
    Tool_SetChuckSucker(CHUCK_CLOSE);

    tool_dev_usart = TOOL_DEV_CLAMP;
    tool_dev_usb = TOOL_DEV_CLAMP;
    Tool_StartSwitch(TOOL_USART_SOURCE, TOOL_DEV_CLAMP);
    Tool_StartSwitch(TOOL_USB_SOURCE, TOOL_DEV_CLAMP);
    Tool_SetActiveSource(TOOL_USART_SOURCE);
}

void Tool_SetActiveSource(uint8_t source)
{
    uint8_t source_changed;

    if (source != TOOL_USB_SOURCE) {
        source = TOOL_USART_SOURCE;
    }

    source_changed = (s_tool_active_source != source) ? 1U : 0U;
    s_tool_active_source = source;

    clamp_usart.control_source[0] = (source == TOOL_USART_SOURCE) ? 1U : 0U;
    clamp_usart.control_source[1] = 0U;
    chuck_usart.control_source[0] = (source == TOOL_USART_SOURCE) ? 1U : 0U;
    chuck_usart.control_source[1] = 0U;

    clamp_usb.control_source[0] = 0U;
    clamp_usb.control_source[1] = (source == TOOL_USB_SOURCE) ? 1U : 0U;
    chuck_usb.control_source[0] = 0U;
    chuck_usb.control_source[1] = (source == TOOL_USB_SOURCE) ? 1U : 0U;

    if (source_changed != 0U) {
        Tool_StartSwitch(source, Tool_GetSelectedDev(source));
    }
}

uint8_t Tool_GetActiveSource(void)
{
    return s_tool_active_source;
}

clamp_Handle_t *Tool_GetClamp(uint8_t source)
{
    return (source == TOOL_USB_SOURCE) ? &clamp_usb : &clamp_usart;
}

chuck_Handle_t *Tool_GetChuck(uint8_t source)
{
    return (source == TOOL_USB_SOURCE) ? &chuck_usb : &chuck_usart;
}

clamp_Handle_t *Tool_GetActiveClamp(void)
{
    return Tool_GetClamp(s_tool_active_source);
}

chuck_Handle_t *Tool_GetActiveChuck(void)
{
    return Tool_GetChuck(s_tool_active_source);
}

void Tool_SetSelectedDev(uint8_t source, uint8_t dev)
{
    uint8_t clean_dev = (dev == 0U) ? 0U : 1U;
    uint8_t old_dev = Tool_GetSelectedDev(source);

    if (old_dev != clean_dev) {
        if (old_dev == TOOL_DEV_CHUCK) {
            Tool_GetChuck(source)->run_status = TOOL_STATUS_IDLE;
        } else {
            Tool_GetClamp(source)->run_status = TOOL_STATUS_IDLE;
        }
    }

    if (source == TOOL_USB_SOURCE) {
        tool_dev_usb = clean_dev;
    } else {
        tool_dev_usart = clean_dev;
    }

    Tool_StartSwitch(source, clean_dev);
}

uint8_t Tool_GetSelectedDev(uint8_t source)
{
    return (source == TOOL_USB_SOURCE) ? tool_dev_usb : tool_dev_usart;
}

uint8_t Tool_GetActiveSelectedDev(void)
{
    return Tool_GetSelectedDev(s_tool_active_source);
}

void Tool_RunActiveStateMachine(void)
{
    if (Tool_GetActiveSelectedDev() == TOOL_DEV_CHUCK) {
        chuck_state_machine_run(Tool_GetActiveChuck());
    } else {
        clamp_state_machine_run(Tool_GetActiveClamp());
    }
}

void Tool_StopSource(uint8_t source)
{
    if (Tool_GetSelectedDev(source) == TOOL_DEV_CHUCK) {
        chuck_Handle_t *tool_chuck = Tool_GetChuck(source);
        trigger_chuck_action(tool_chuck, CHUCK_CLOSE);
    } else {
        clamp_Handle_t *tool_clamp = Tool_GetClamp(source);
        trigger_clamp_action(tool_clamp, CLAMP_OPEN);
    }
}

void Tool_HoldSource(uint8_t source)
{
    if (Tool_GetSelectedDev(source) == TOOL_DEV_CHUCK) {
        chuck_Handle_t *tool_chuck = Tool_GetChuck(source);
        Tool_HoldChuck(tool_chuck);
        tool_chuck->pending_state = tool_chuck->state;
        tool_chuck->run_status = TOOL_STATUS_IDLE;
    } else {
        clamp_Handle_t *tool_clamp = Tool_GetClamp(source);
        Tool_HoldClamp(tool_clamp);
        tool_clamp->pending_state = tool_clamp->state;
        tool_clamp->run_status = TOOL_STATUS_IDLE;
    }
}

float Tool_GetActiveTargetAngle(void)
{
    if (Tool_GetActiveSelectedDev() == TOOL_DEV_CHUCK) {
        return Tool_GetActiveChuck()->target_angle;
    }

    return Tool_GetActiveClamp()->target_angle;
}
