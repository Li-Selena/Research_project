#ifndef __CONTROL_TASK_H
#define __CONTROL_TASK_H

#include "bsp_mcu.h"
#include "include.h"
#include "bsp_tick.h"
#include "mecanum_classic.h"
#include "CRC.h"
#include "bsp_uart.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <stdint.h>
#include "bsp_usb.h"
#include "arm_ik_3r_safe_stm32h7.h"
#include "arm_user.h"
#include "arm_echo_uart10.h"
#include "arm_tools.h"
#include "R2_move.h"
#include "R2_climb.h"
#include "INS_Task.h"



extern float ctrl_j1,ctrl_j2,ctrl_j3;
extern float model_theta1,model_theta2,model_theta3;
extern float ctrl_J_USB[4];
extern float model_J_USB[4];
extern float ctrl_J_USART[4];
extern float model_J_USART[4];

typedef struct
{
    uint32_t tick_ms;
    uint8_t enc_inited;
    uint8_t imu_yaw_valid;
    uint8_t active_source;
    uint8_t reserved;

    int32_t current_enc[CHASSIS_MOTOR_COUNT];
    int32_t last_enc[CHASSIS_MOTOR_COUNT];
    int32_t enc_delta[CHASSIS_MOTOR_COUNT];
    float wheel_delta_m[CHASSIS_MOTOR_COUNT];

    float robot_dx_m;
    float robot_dy_m;
    float robot_dyaw_rad;
    float odom_vx_mps;
    float odom_vy_mps;
    float odom_wz_radps;

    float imu_yaw_rad;
    float active_odom_x;
    float active_odom_y;
    float active_odom_yaw;
} R2_DebugOdom_t;

/* R2 底盘运动控制器（在 Control_Task.c 定义，由 TIM3 通知控制任务 1ms 更新） */
extern R2_Move_Ctrl_t g_r2_ctrl_usart;  /* 串口遥控 */
extern R2_Move_Ctrl_t g_r2_ctrl_usb;    /* USB 调试 */
extern R2_Climb_Ctrl_t g_r2_climb_usart;
extern R2_Climb_Ctrl_t g_r2_climb_usb;

void Mecanum_task_USB(ChassisVel_t *chassis_user, MecanumParam_t *param_user, WheelSpeed_t *speed_user);    //麦克纳姆轮底盘控制处理，专门给USB数据解析调用的接口
extern R2_DebugOdom_t g_r2_debug_odom;
extern uint32_t g_r2_tick_ms;
extern int32_t g_r2_last_enc[CHASSIS_MOTOR_COUNT];
extern uint8_t g_r2_enc_inited;

void Arm_task_USB(float x,float y,float z);
void Arm_HoldCurrentPosition(uint8_t source);

void control_tim1mscallback(void);



#endif /* __CONTROL_TASK_H */
