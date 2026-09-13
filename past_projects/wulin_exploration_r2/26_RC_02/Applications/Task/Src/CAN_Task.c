#include "Control_Task.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "pid_user.h"
#include "CAN_Task.h"

extern float ctrl_j1,ctrl_j2,ctrl_j3;
extern float model_theta1,model_theta2,model_theta3;
extern float ctrl_J_USB[4];
extern float model_J_USB[4];
extern float ctrl_J_USART[4];
extern float model_J_USART[4];

extern WheelSpeed_t total_speed;

static float AngleClampJ1(float deg);


void CAN_Task(void const * argument){
    osDelay(5000);

    /* 关节减速比常量 */
    float Tnum23 = 360.0f / 8192.0f / 3591.0f * 187.0f / 5.0f;
    float Tnum1  = 360.0f / 8192.0f / 36.0f   / 94.0f  * 19.0f;
    float mecNum = MOTOR_IN2OUT * RPM_TO_MS;

    for(;;)
    {
        WheelSpeed_t wheel_cmd = {0};
        float arm_cmd[3] = {0.0f, 0.0f, 0.0f};
        float tool_target = 0.0f;
        R2_Climb_Ctrl_t climb_snapshot;
        R2_ClimbMotorCmd_t climb_cmd;
        uint8_t has_source = 0U;
        uint8_t has_climb_source = 0U;
        uint8_t climb_debug_source = R2_CLIMB_DEBUG_SOURCE_NONE;

        /*
         * FDCAN1: 底盘四麦克纳姆轮
         *
         * 轮速 m/s -> 电机 RPM: motor_rpm = wheel_speed / mecNum
         * fr/br 取反: 对齐电机安装方向。
         * USART/USB 两个 R2 控制器根据 Task_flag 自动切换。
         */
        /*
         * FDCAN1 电机映射（物理顺时针编号, O 型麦克纳姆轮）:
         *   CAN1=Motor1(FR,右上) ← fr   (安装反向, 取负)
         *   CAN2=Motor2(BR,右下) ← br   (安装反向, 取负)
         *   CAN3=Motor3(BL,左下) ← bl
         *   CAN4=Motor4(FL,左上) ← fl
         */
        taskENTER_CRITICAL();
        if (USART_Task_flag == 1U) {
            wheel_cmd = g_r2_ctrl_usart.wheel_speed;
            climb_snapshot = g_r2_climb_usart;
            arm_cmd[0] = ctrl_J_USART[0];
            arm_cmd[1] = ctrl_J_USART[1];
            arm_cmd[2] = ctrl_J_USART[2];
            has_source = 1U;
            has_climb_source = 1U;
            climb_debug_source = R2_CLIMB_DEBUG_SOURCE_USART;
        } else if (USB_Task_flag == 1U) {
            wheel_cmd = g_r2_ctrl_usb.wheel_speed;
            climb_snapshot = g_r2_climb_usb;
            arm_cmd[0] = ctrl_J_USB[0];
            arm_cmd[1] = ctrl_J_USB[1];
            arm_cmd[2] = ctrl_J_USB[2];
            has_source = 1U;
            has_climb_source = 1U;
            climb_debug_source = R2_CLIMB_DEBUG_SOURCE_USB;
        }
        tool_target = Tool_GetActiveTargetAngle();
        taskEXIT_CRITICAL();

        if (has_source != 0U) {
            FDCAN1_CMD_1(
                PID_velocity_realize_1(-wheel_cmd.fr / mecNum, 1),
                PID_velocity_realize_1(-wheel_cmd.br / mecNum, 2),
                PID_velocity_realize_1( wheel_cmd.bl / mecNum, 3),
                PID_velocity_realize_1( wheel_cmd.fl / mecNum, 4)
            );
        } else {
            FDCAN1_CMD_1(0, 0, 0, 0);
        }

        if (has_climb_source != 0U) {
            R2_Climb_GetMotorCurrent(&climb_snapshot, &climb_cmd);
        } else {
            climb_cmd.leg[0] = (int16_t)PID_velocity_realize_2(0.0f, 1);
            climb_cmd.leg[1] = (int16_t)PID_velocity_realize_2(0.0f, 2);
            climb_cmd.leg[2] = (int16_t)PID_velocity_realize_2(0.0f, 3);
            climb_cmd.leg[3] = (int16_t)PID_velocity_realize_2(0.0f, 4);
            climb_cmd.front_drive[0] = (int16_t)PID_velocity_realize_1(0.0f, 5);
            climb_cmd.front_drive[1] = (int16_t)PID_velocity_realize_1(0.0f, 6);
            climb_cmd.front_drive[2] = 0;
            climb_cmd.front_drive[3] = 0;
            climb_cmd.drive[0] = (int16_t)PID_velocity_realize_2(0.0f, 5);
            climb_cmd.drive[1] = (int16_t)PID_velocity_realize_2(0.0f, 6);
            climb_cmd.drive[2] = 0;
            climb_cmd.drive[3] = 0;
        }

        R2_Climb_SetDebugMotorCurrent(climb_debug_source, &climb_cmd);

        /*
         * FDCAN1: front climb drive wheels 5..6
         * FDCAN2: climbing legs 1..4 and rear drive wheels 5..6
         */
        FDCAN1_CMD_2(
            climb_cmd.front_drive[0],
            climb_cmd.front_drive[1],
            0,
            0
        );

        FDCAN2_CMD_1(
            climb_cmd.leg[0],
            climb_cmd.leg[1],
            climb_cmd.leg[2],
            climb_cmd.leg[3]
        );

        FDCAN2_CMD_2(
            climb_cmd.drive[0],
            climb_cmd.drive[1],
            0,
            0
        );

        /*
         * FDCAN3: 机械臂四电机
         *
         * pid_call_3 = 位置外环 + 速度内环 (级联 PID)
         * J1 +/-180 deg limit
         */
        FDCAN3_CMD_1(
            pid_call_3(-AngleClampJ1(arm_cmd[0]) / Tnum1, 1),
            pid_call_3( arm_cmd[1] / Tnum23, 2),
            pid_call_3( arm_cmd[2] / Tnum23, 3),
            pid_call_3( tool_target, 4)
        );

        osDelay(1);
    }
}


static float AngleClampJ1(float deg)
{
    if (deg > ARM_IK_J1_LIMIT_DEG)       return ARM_IK_J1_LIMIT_DEG;
    else if (deg < -ARM_IK_J1_LIMIT_DEG) return -ARM_IK_J1_LIMIT_DEG;
    else                   return deg;
}
