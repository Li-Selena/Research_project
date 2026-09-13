#include "cmsis_os.h"
#include "Control_Task.h"
#include "Data_Analysis.h"
#include "R2_yaw_autotune.h"
#include "CRC.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;


extern volatile uint8_t USB_Task_flag;
extern volatile uint8_t USART_Task_flag ;


// 机械臂模型与两种控制源的关节目标。
extern Arm3R_Handle_t g_arm_ik;
float ctrl_j1,ctrl_j2,ctrl_j3;
float model_theta1,model_theta2,model_theta3;
float ctrl_J_USB[4];
float model_J_USB[4];
float ctrl_J_USART[4];
float model_J_USART[4];

float start_X = 0.0f;//启动保护
float start_Y = 0.0f;
float start_Z = 0.0f;

float start_X_USB = 0.0f;//USB启动保护
float start_Y_USB = 0.0f;
float start_Z_USB = 0.0f;

#define ARM_HOLD_TNUM1   0.0002464f
#define ARM_HOLD_TNUM23  0.0004577f




//麦克纳姆轮底盘控制
extern ChassisVel_t total_vel ;
extern WheelSpeed_t total_speed;
extern MecanumParam_t mecParam;

/* ── R2 底盘运动控制器 ── */
R2_Move_Ctrl_t g_r2_ctrl_usart;
R2_Move_Ctrl_t g_r2_ctrl_usb;
R2_Climb_Ctrl_t g_r2_climb_usart;
R2_Climb_Ctrl_t g_r2_climb_usb;
R2_DebugOdom_t g_r2_debug_odom;

/* Logical 1 ms counter incremented by the control task. */
uint32_t g_r2_tick_ms = 0U;

/* 1ms 里程计：记录上一周期编码器值，计算增量 */
int32_t g_r2_last_enc[CHASSIS_MOTOR_COUNT]; /* [0..3] = FR, BR, BL, FL physical motors 1..4 */
uint8_t g_r2_enc_inited = 0U;


// 遥控命令状态。
extern int8_t control_cmd ;

//串口控制切换工具
extern uint8_t tool_flag;
extern uint8_t clampuse_flag;
extern uint8_t chuckuse_flag;



static void Arm_task(void);    // USART 机械臂目标处理。
static void R2_Control_1msStep(void);

static TaskHandle_t s_control_task_handle = NULL;
static uint8_t s_usart_arm_hold_active = 0U;

void Mecanum_task_USB(ChassisVel_t *chassis_user, MecanumParam_t *param_user, WheelSpeed_t *speed_user);    //ķֵ̿ƴרŸUSBݽõĽӿ
void Arm_task_USB(float x,float y,float z);    //еۿƴרŸUSBݽõĽӿ

void Control_Task(void const * argument){
	osDelay(5000);

	MX_USB_DEVICE_Init();
    HAL_UART_Receive_IT(&huart10, &btReceiveData, 1);

    MCU_Init();

    /* R2 底盘运动控制器初始化 */
    R2_Move_Init(&g_r2_ctrl_usart, &mecParam, 0.001f);
    R2_Move_Init(&g_r2_ctrl_usb,   &mecParam, 0.001f);   /* 1ms 控制周期 */
    R2_Climb_Init(&g_r2_climb_usart);
    R2_Climb_Init(&g_r2_climb_usb);
    R2_YawAutoTune_Init();



	// ArmEchoUart10_Init();
	ArmIK_ComponentInit();

    Tool_InitAll();

    s_control_task_handle = xTaskGetCurrentTaskHandle();

    /* 启动 TIM3 1ms 中断：ISR 只负责通知本任务执行控制步进 */
    HAL_TIM_Base_Start_IT(&htim3);


  for(;;)
  {
    uint32_t pending_ticks = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1));

    if (pending_ticks == 0U) {
        pending_ticks = 1U;
    }

    while (pending_ticks != 0U) {
        R2_Control_1msStep();
        pending_ticks--;
    }

    BT_Data_MAC_Process(&total_vel.vx,&total_vel.vy,&total_vel.vw,NULL);

    // USB_Task_flag = 1U;
    // USART_Task_flag = 0U;

    Tool_RunActiveStateMachine();

    if(USART_Task_flag == 1U)
    {
        Arm_task();
    }
  }


}




static void Arm_task()
{
    const ArmIK_AppState_t *app;

    if (arm_flag != 1)
    {
        if (s_usart_arm_hold_active == 0U) {
            Arm_HoldCurrentPosition(TOOL_USART_SOURCE);
            s_usart_arm_hold_active = 1U;
        }
        arm_input_valid = 0U;
        return;
    }

    if (arm_input_valid == 0U)
    {
        return;
    }

    if(start_X == arm_X && start_Y == arm_Y && start_Z == arm_Z)
    {
        if (s_usart_arm_hold_active == 0U) {
            Arm_HoldCurrentPosition(TOOL_USART_SOURCE);
            s_usart_arm_hold_active = 1U;
        }
    }
    else
    {
        s_usart_arm_hold_active = 0U;
        ArmIK_ComponentStep(arm_X, arm_Y, arm_Z);

        /* 读取最近一次通过安全检查的目标。 */
        app = ArmIK_GetAppState();

        if (app->has_last_valid != 0U)
        {
            model_J_USART[0] = app->active_model.theta1;
            model_J_USART[1] = app->active_model.theta2;
            model_J_USART[2] = app->active_model.theta3;

            ctrl_J_USART[0] = app->active_motor_deg.j1_deg;
            ctrl_J_USART[1] = app->active_motor_deg.j2_deg;
            ctrl_J_USART[2] = app->active_motor_deg.j3_deg;

	        if (ArmEchoUart10_IsBusy() == 0U)
            {
                ArmEchoUart10_StartSend_IT();
            }
        }
    }
}



void Mecanum_task_USB(ChassisVel_t *chassis_user, MecanumParam_t *param_user, WheelSpeed_t *speed_user)    // USB 调用的麦轮逆运动学接口，输入采用统一机器人坐标系。
{
    Mecanum_Calc(chassis_user, param_user, speed_user);
}

void Arm_HoldCurrentPosition(uint8_t source)
{
    float hold_j1;
    float hold_j2;
    float hold_j3;
    float *ctrl_target;

    hold_j1 = -(float)motor_fdcan3[0].total_angle * ARM_HOLD_TNUM1;
    hold_j2 =  (float)motor_fdcan3[1].total_angle * ARM_HOLD_TNUM23;
    hold_j3 =  (float)motor_fdcan3[2].total_angle * ARM_HOLD_TNUM23;

    if (hold_j1 > ARM_IK_J1_LIMIT_DEG) {
        hold_j1 = ARM_IK_J1_LIMIT_DEG;
    } else if (hold_j1 < -ARM_IK_J1_LIMIT_DEG) {
        hold_j1 = -ARM_IK_J1_LIMIT_DEG;
    }

    ctrl_target = (source == TOOL_USB_SOURCE) ? ctrl_J_USB : ctrl_J_USART;

    taskENTER_CRITICAL();
    ctrl_target[0] = hold_j1;
    ctrl_target[1] = hold_j2;
    ctrl_target[2] = hold_j3;
    taskEXIT_CRITICAL();
}

void Arm_task_USB(float x,float y,float z)
{
    const ArmIK_AppState_t *app;

    if(start_X_USB == x && start_Y_USB == y && start_Z_USB == z)
    {
        Arm_HoldCurrentPosition(TOOL_USB_SOURCE);
    }
    else
    {
        ArmIK_ComponentStep(x, y, z);

        /* 读取最近一次通过安全检查的目标。 */
        app = ArmIK_GetAppState();

        if (app->has_last_valid != 0U)
        {

            taskENTER_CRITICAL();
            model_J_USB[0] = app->active_model.theta1;
            model_J_USB[1] = app->active_model.theta2;
            model_J_USB[2] = app->active_model.theta3;

            ctrl_J_USB[0] = app->active_motor_deg.j1_deg;
            ctrl_J_USB[1] = app->active_motor_deg.j2_deg;
            ctrl_J_USB[2] = app->active_motor_deg.j3_deg;
            taskEXIT_CRITICAL();

        }
    }
}

/* Control-task step (nominal 1 ms): feedback, odometry, motion, climb and
 * watchdogs. The TIM3 ISR below only gives a task notification.
 */
static void R2_Control_1msStep(void)
{
    float now_sec;
    float w_delta[CHASSIS_MOTOR_COUNT];
    float robot_dx, robot_dy, robot_dyaw;
    int32_t cur_enc[CHASSIS_MOTOR_COUNT];
    int32_t delta;
    uint8_t i;
    float imu_yaw_rad;
    float robot_vx_mps = 0.0f;
    float robot_vy_mps = 0.0f;
    float odom_vx_mps = 0.0f;
    float odom_vy_mps = 0.0f;
    float odom_wz_radps = 0.0f;
    R2_Move_Ctrl_t *active_ctrl;
    INS_NavState_t ins_state;
    uint8_t imu_yaw_valid;
    uint8_t climb_debug_source = R2_CLIMB_DEBUG_SOURCE_NONE;

    /* Logical control-step time; HAL ticks remain the watchdog time base. */
    now_sec = (float)g_r2_tick_ms * 0.001f;
    g_r2_debug_odom.tick_ms = g_r2_tick_ms;
    g_r2_tick_ms++;

    /*
     * 先用 IMU 绝对 yaw 修正朝向，再用该 yaw 做 robot->world 里程计积分。
     * R2_Move_UpdateOdom() 的接口约定要求调用方先更新 yaw。
     */
    INS_GetState(&ins_state);
    imu_yaw_rad = ins_state.yaw_total_rad;
    imu_yaw_valid = ins_state.imu_online;
    g_r2_debug_odom.imu_yaw_rad = imu_yaw_rad;
    g_r2_debug_odom.imu_yaw_valid = imu_yaw_valid;
    if (imu_yaw_valid != 0U) {
        R2_Move_UpdateYaw(&g_r2_ctrl_usart, imu_yaw_rad);
        R2_Move_UpdateYaw(&g_r2_ctrl_usb,   imu_yaw_rad);
    }

    /* 读取 4 路编码器当前累积值 */
    for (i = 0U; i < CHASSIS_MOTOR_COUNT; i++) {
        cur_enc[i] = motor_fdcan1[i].total_angle;
        g_r2_debug_odom.current_enc[i] = cur_enc[i];
        g_r2_debug_odom.last_enc[i] = g_r2_last_enc[i];
    }

    /* ── 里程计更新 ── */
    if (g_r2_enc_inited != 0U) {

        /* 编码器增量 → 线位移 (m) */
        for (i = 0U; i < CHASSIS_MOTOR_COUNT; i++) {
            delta = cur_enc[i] - g_r2_last_enc[i];
            w_delta[i] = EncoderDeltaToWheelMeter(delta);
            g_r2_debug_odom.enc_delta[i] = delta;
            g_r2_debug_odom.wheel_delta_m[i] = w_delta[i];
        }

        /* Encoder deltas are motor-shaft signed. Correct FR/BR once, then
         * use the same forward kinematics as the wheel-velocity API.
         */
        {
            WheelSpeed_t wheel_delta;
            ChassisVel_t body_delta;
            wheel_delta.fr = -w_delta[CHASSIS_MOTOR_FR];
            wheel_delta.br = -w_delta[CHASSIS_MOTOR_BR];
            wheel_delta.bl =  w_delta[CHASSIS_MOTOR_BL];
            wheel_delta.fl =  w_delta[CHASSIS_MOTOR_FL];
            ChassisForwardKinematics(&wheel_delta, &mecParam, &body_delta);
            robot_dx = body_delta.vx;
            robot_dy = body_delta.vy;
            robot_dyaw = body_delta.vw;
        }

        robot_vx_mps = robot_dx * 1000.0f;
        robot_vy_mps = robot_dy * 1000.0f;
        odom_wz_radps = robot_dyaw * 1000.0f;
        g_r2_debug_odom.robot_dx_m = robot_dx;
        g_r2_debug_odom.robot_dy_m = robot_dy;
        g_r2_debug_odom.robot_dyaw_rad = robot_dyaw;
        g_r2_debug_odom.odom_wz_radps = odom_wz_radps;

        /* 累加到世界系里程计（内部做 robot→world 旋转） */
        R2_Move_UpdateOdom(&g_r2_ctrl_usart, robot_dx, robot_dy, robot_dyaw);
        R2_Move_UpdateOdom(&g_r2_ctrl_usb,   robot_dx, robot_dy, robot_dyaw);

        /*
         * R2_Move_UpdateOdom() keeps encoder-yaw integration for IMU-offline
         * fallback. When IMU is online, restore the absolute heading before
         * yaw-hold and POS feedback read ctrl->odom_yaw.
         */
        if (imu_yaw_valid != 0U) {
            R2_Move_UpdateYaw(&g_r2_ctrl_usart, imu_yaw_rad);
            R2_Move_UpdateYaw(&g_r2_ctrl_usb,   imu_yaw_rad);
        }

    } else {
        /* 首次调用：仅快照编码器基准值 */
        g_r2_enc_inited = 1U;
        for (i = 0U; i < CHASSIS_MOTOR_COUNT; i++) {
            g_r2_debug_odom.enc_delta[i] = 0;
            g_r2_debug_odom.wheel_delta_m[i] = 0.0f;
        }
        g_r2_debug_odom.robot_dx_m = 0.0f;
        g_r2_debug_odom.robot_dy_m = 0.0f;
        g_r2_debug_odom.robot_dyaw_rad = 0.0f;
        g_r2_debug_odom.odom_vx_mps = 0.0f;
        g_r2_debug_odom.odom_vy_mps = 0.0f;
        g_r2_debug_odom.odom_wz_radps = 0.0f;
    }
    g_r2_debug_odom.enc_inited = g_r2_enc_inited;

    /* 保存本轮编码器值，供下一周期算增量 */
    for (i = 0U; i < CHASSIS_MOTOR_COUNT; i++) {
        g_r2_last_enc[i] = cur_enc[i];
    }

    /* ── 运动控制更新（浮点运算，H7 FPU 可胜任） ── */
    USB_ControlWatchdog_Check();
    USART_ControlWatchdog_Check();

    R2_YawAutoTune_Step(&g_r2_ctrl_usb, g_r2_tick_ms);

    R2_Move_Update(&g_r2_ctrl_usart, now_sec);
    R2_Move_Update(&g_r2_ctrl_usb,   now_sec);
    R2_Climb_Update(&g_r2_climb_usart, &g_r2_ctrl_usart, g_r2_tick_ms);
    R2_Climb_Update(&g_r2_climb_usb,   &g_r2_ctrl_usb,   g_r2_tick_ms);

    /* 同步到全局变量，方便调试观测 */
    if (USART_Task_flag == 1U) {
        total_speed = g_r2_ctrl_usart.wheel_speed;
        total_vel   = g_r2_ctrl_usart.robot_vel;
        active_ctrl = &g_r2_ctrl_usart;
        g_r2_debug_odom.active_source = TOOL_USART_SOURCE;
        climb_debug_source = R2_CLIMB_DEBUG_SOURCE_USART;
    } else if (USB_Task_flag == 1U) {
        total_speed = g_r2_ctrl_usb.wheel_speed;
        total_vel   = g_r2_ctrl_usb.robot_vel;
        active_ctrl = &g_r2_ctrl_usb;
        g_r2_debug_odom.active_source = TOOL_USB_SOURCE;
        climb_debug_source = R2_CLIMB_DEBUG_SOURCE_USB;
    } else {
        active_ctrl = &g_r2_ctrl_usart;
        g_r2_debug_odom.active_source = TOOL_USART_SOURCE;
        climb_debug_source = R2_CLIMB_DEBUG_SOURCE_NONE;
    }

    R2_Climb_UpdateDebugViews(&g_r2_climb_usart,
                              &g_r2_climb_usb,
                              climb_debug_source);

    g_r2_debug_odom.active_odom_x = active_ctrl->odom_x;
    g_r2_debug_odom.active_odom_y = active_ctrl->odom_y;
    g_r2_debug_odom.active_odom_yaw = active_ctrl->odom_yaw;
    {
        float c = cosf(active_ctrl->odom_yaw);
        float s = sinf(active_ctrl->odom_yaw);
        odom_vx_mps = robot_vx_mps * c - robot_vy_mps * s;
        odom_vy_mps = robot_vx_mps * s + robot_vy_mps * c;
    }
    g_r2_debug_odom.odom_vx_mps = odom_vx_mps;
    g_r2_debug_odom.odom_vy_mps = odom_vy_mps;

    INS_SetOdometry(active_ctrl->odom_x,
                    active_ctrl->odom_y,
                    odom_vx_mps,
                    odom_vy_mps,
                    odom_wz_radps);

}

void control_tim1mscallback(void)
{
    BaseType_t higher_priority_task_woken = pdFALSE;

    if (s_control_task_handle != NULL) {
        vTaskNotifyGiveFromISR(s_control_task_handle, &higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
}
