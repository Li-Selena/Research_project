#include "Data_Analysis.h"
#include "mecanum_classic.h"
#include "R2_move.h"
#include "R2_yaw_autotune.h"
#include "arm_tools.h"
#include "arm_user.h"
#include "fdcan_receive.h"
#include "PC_TX_Task.h"
#include "CRC.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>

extern R2_Move_Ctrl_t g_r2_ctrl_usb;

static void   USB_Read4Floats(const uint8_t *d, float *f);
static uint8_t USB_Read4FloatsChecked(const uint8_t *d, uint8_t len, float *f);
static uint8_t USB_AllowEmptyOrFloatPayload(uint8_t len);
static void USB_ChassisWatchdog_Feed(void);
static void USB_ChassisWatchdog_Disarm(void);
static void USB_ArmWatchdog_Feed(void);
static void USB_ArmWatchdog_Disarm(void);
static void USB_ToolWatchdog_Feed(void);
static void USB_ToolWatchdog_Disarm(void);
static void USB_ArmWatchdog_Check(uint32_t now_tick);
static void USB_ToolWatchdog_Check(uint32_t now_tick);
static void USB_CommandRx_Record(uint8_t cmd, const uint8_t *d, uint8_t len);
static void USB_ToolSetActuator(uint8_t dev, uint8_t act);
static float  Clamp(float x, float lo, float hi);

extern void    Arm_task_USB(float x, float y, float z);
extern volatile uint8_t USB_Task_flag;
extern volatile uint8_t USART_Task_flag;
extern float   ctrl_J_USB[4];

/* ── 各模块使能标志 ── */
uint8_t Mecanum_control_flag = 0U;
uint8_t Arm_control_flag     = 0U;
uint8_t Tool_control_flag    = 0U;

ChassisVel_t total_vel_USB   = {0};
WheelSpeed_t total_speed_USB = {0};

static volatile uint32_t s_usb_chassis_last_tick = 0U;
static volatile uint8_t  s_usb_chassis_watchdog_armed = 0U;
static volatile uint8_t  s_usb_chassis_timeout = 0U;
static volatile uint32_t s_usb_arm_last_tick = 0U;
static volatile uint8_t  s_usb_arm_watchdog_armed = 0U;
static volatile uint8_t  s_usb_arm_timeout = 0U;
static volatile uint32_t s_usb_tool_last_tick = 0U;
static volatile uint8_t  s_usb_tool_watchdog_armed = 0U;
static volatile uint8_t  s_usb_tool_timeout = 0U;
static volatile uint32_t s_usb_cmd_last_tick = 0U;
static volatile uint32_t s_usb_cmd_count = 0U;
static volatile uint8_t  s_usb_cmd_last_cmd = 0U;
static volatile uint8_t  s_usb_cmd_last_len = 0U;
static volatile uint8_t  s_usb_cmd_last_payload_valid = 0U;
static volatile uint8_t  s_usb_cmd_last_data[16] = {0U};
static volatile float    s_usb_cmd_last_f[4] = {0.0f};

#define USB_ARM_TARGET_TOL_DEG  2.0f
#define USB_ARM_TNUM1           0.0002464f
#define USB_ARM_TNUM23          0.0004577f


uint8_t tool_dev = TOOL_DEV_CLAMP;  /* 0=夹爪 1=吸盘 */


/* ═══════════════════════════════════════════════════════
 *  命令路由器 — 统一解析 4 个 float，各命令按需取用
 * ═══════════════════════════════════════════════════════ */

/*
 * Data_Analysis — USB 命令解析
 *
 * 约定：上位机每次发送 16 字节数据区（4 个 float，小端），
 * 不同命令根据自身需求选择性取用 f[0]~f[3]，忽略不需要的。
 *
 *   f[0]  f[1]  f[2]                            f[3]
 *   vx    vy    vw/active-frame target_yaw_deg  (reserved) ← CHS_SET_VEL
 *   dx    dy    dyaw/active-frame target_yaw_deg (reserved) ← CHS_SET_POS
 *   x     y     z     (reserved) ← ARM_SET_TARGET
 *   dev   act   —     —          ← TOOL_SET_STATE
 *   dev   —     —     —          ← TOOL_SET_MODE
 *   act   —     —     —          ← TOOL_ACTION (legacy current-position action)
 *   src   —     —     —          ← SYS_SWITCH_SOURCE
 *   mode  —     —     —          ← CHS_SET_MODE
 *
 * 底盘和机械臂均采用 FLU：X 前、Y 左、Z 上、yaw 逆时针为正。
 */
void Data_Analysis(uint8_t cmd, const uint8_t* d, uint8_t len)
{
    float f[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    uint8_t climb_auto_level = 0U;
    uint8_t climb_auto_allowed = 1U;

    USB_CommandRx_Record(cmd, d, len);

    switch (cmd) {

    /* ── System ── */
    case USB_CMD_SYS_DISABLE:
        if (!USB_AllowEmptyOrFloatPayload(len)) break;
        USB_ChassisWatchdog_Disarm();
        USB_ArmWatchdog_Disarm();
        USB_ToolWatchdog_Disarm();
        taskENTER_CRITICAL();
        R2_YawAutoTune_Stop();
        R2_Move_Stop(&g_r2_ctrl_usb);
        R2_Climb_Stop(&g_r2_climb_usb);
        taskEXIT_CRITICAL();
        Arm_HoldCurrentPosition(TOOL_USB_SOURCE);
        taskENTER_CRITICAL();
        Tool_HoldSource(TOOL_USB_SOURCE);
        taskEXIT_CRITICAL();
        Mecanum_control_flag = 0U;
        Arm_control_flag     = 0U;
        Tool_control_flag    = 0U;
        break;
    case USB_CMD_SYS_ENABLE:
        if (!USB_AllowEmptyOrFloatPayload(len)) break;
        Mecanum_control_flag = 1U;
        Arm_control_flag     = 1U;
        Tool_control_flag    = 1U;
        break;
    case USB_CMD_SYS_SWITCH_SOURCE:
        if (!USB_Read4FloatsChecked(d, len, f)) break;
        Control_SetSource(((uint8_t)f[0]) ? TOOL_USB_SOURCE : TOOL_USART_SOURCE);
        if (((uint8_t)f[0]) == 0U) {
            USB_ChassisWatchdog_Disarm();
            USB_ArmWatchdog_Disarm();
            USB_ToolWatchdog_Disarm();
            taskENTER_CRITICAL();
            R2_YawAutoTune_Stop();
            R2_Move_Stop(&g_r2_ctrl_usb);
            R2_Climb_Stop(&g_r2_climb_usb);
            taskEXIT_CRITICAL();
            Arm_HoldCurrentPosition(TOOL_USB_SOURCE);
        }
        break;
    case USB_CMD_SYS_GET_STATUS:
        if (!USB_AllowEmptyOrFloatPayload(len)) break;
        PC_TX_ReqSysStatus();
        break;
    case USB_CMD_SYS_STOP:
        if (!USB_AllowEmptyOrFloatPayload(len)) break;
        USB_ChassisWatchdog_Disarm();
        USB_ArmWatchdog_Disarm();
        USB_ToolWatchdog_Disarm();
        taskENTER_CRITICAL();
        R2_YawAutoTune_Stop();
        R2_Move_Stop(&g_r2_ctrl_usb);
        R2_Climb_Stop(&g_r2_climb_usb);
        Tool_StopSource(TOOL_USB_SOURCE);
        taskEXIT_CRITICAL();
        Arm_HoldCurrentPosition(TOOL_USB_SOURCE);
        break;

    /* ── Chassis ── */
    case USB_CMD_CHS_DISABLE:
        if (!USB_AllowEmptyOrFloatPayload(len)) break;
        USB_ChassisWatchdog_Disarm();
        taskENTER_CRITICAL();
        R2_YawAutoTune_Stop();
        R2_Move_Stop(&g_r2_ctrl_usb);
        taskEXIT_CRITICAL();
        Mecanum_control_flag = 0U;
        break;
    case USB_CMD_CHS_ENABLE:
        if (!USB_AllowEmptyOrFloatPayload(len)) break;
        Mecanum_control_flag = 1U;
        break;

    case USB_CMD_CHS_SET_MODE:
        if (!USB_Read4FloatsChecked(d, len, f)) break;
        {
            uint8_t mode = (uint8_t)f[0];
            if (mode <= 7U && USB_Task_flag && Mecanum_control_flag) {
                taskENTER_CRITICAL();
                R2_YawAutoTune_Stop();
                R2_Move_SetMode(&g_r2_ctrl_usb, (R2_MoveMode_t)mode);
                taskEXIT_CRITICAL();
            }
        }
        break;

    case USB_CMD_CHS_SET_VEL:
        if (!USB_Read4FloatsChecked(d, len, f)) break;
        {
            uint8_t no_yaw_mode = R2_Move_IsNoYawMode(g_r2_ctrl_usb.mode);
            float yaw_data = f[2];

            f[0] = Clamp(f[0], MEC_REMOTE_VX_MIN_MPS, MEC_REMOTE_VX_MAX_MPS);
            f[1] = Clamp(f[1], MEC_REMOTE_VY_MIN_MPS, MEC_REMOTE_VY_MAX_MPS);
            if (no_yaw_mode == 0U) {
                f[2] = Clamp(f[2], MEC_REMOTE_VW_MIN_RAD_S, MEC_REMOTE_VW_MAX_RAD_S);
            }
            total_vel_USB.vx = f[0];
            total_vel_USB.vy = f[1];
            total_vel_USB.vw = (no_yaw_mode != 0U) ? 0.0f : f[2];

            if (USB_Task_flag && Mecanum_control_flag) {
                taskENTER_CRITICAL();
                R2_YawAutoTune_Stop();
                if (no_yaw_mode != 0U) {
                    if (R2_Move_IsWorldMode(g_r2_ctrl_usb.mode)) {
                        R2_Move_SetWorldLockYaw(&g_r2_ctrl_usb,
                                                yaw_data * 0.0174533f);
                    } else {
                        R2_Move_SetRobotLockYaw(&g_r2_ctrl_usb,
                                                yaw_data * 0.0174533f);
                    }
                    R2_Move_SetVel(&g_r2_ctrl_usb, f[0], f[1], 0.0f);
                } else {
                    R2_Move_SetVel(&g_r2_ctrl_usb, f[0], f[1], f[2]);
                }
                taskEXIT_CRITICAL();
                USB_ChassisWatchdog_Feed();
            }
        }
        break;

    case USB_CMD_CHS_SET_POS:
        if (!USB_Read4FloatsChecked(d, len, f)) break;
        {
            uint8_t no_yaw_mode = R2_Move_IsNoYawMode(g_r2_ctrl_usb.mode);
            float yaw_data = f[2];

            total_vel_USB.vx = f[0];
            total_vel_USB.vy = f[1];
            total_vel_USB.vw = (no_yaw_mode != 0U) ? 0.0f : f[2];
            if (USB_Task_flag && Mecanum_control_flag) {
                int8_t set_result;

                taskENTER_CRITICAL();
                R2_YawAutoTune_Stop();
                if (no_yaw_mode != 0U) {
                    if (R2_Move_IsWorldMode(g_r2_ctrl_usb.mode)) {
                        R2_Move_SetWorldLockYaw(&g_r2_ctrl_usb,
                                                yaw_data * 0.0174533f);
                    } else {
                        R2_Move_SetRobotLockYaw(&g_r2_ctrl_usb,
                                                yaw_data * 0.0174533f);
                    }
                    set_result = R2_Move_SetDist(&g_r2_ctrl_usb, f[0], f[1], 0.0f);
                } else {
                    set_result = R2_Move_SetDist(&g_r2_ctrl_usb, f[0], f[1], f[2]);
                }
                taskEXIT_CRITICAL();
                if (set_result == 0) {
                    USB_ChassisWatchdog_Disarm();
                }
            }
        }
        break;

    case USB_CMD_CHS_STOP:
        if (!USB_AllowEmptyOrFloatPayload(len)) break;
        USB_ChassisWatchdog_Disarm();
        taskENTER_CRITICAL();
        R2_YawAutoTune_Stop();
        R2_Move_Stop(&g_r2_ctrl_usb);
        taskEXIT_CRITICAL();
        break;
    case USB_CMD_CHS_GET_STATUS:
        if (!USB_AllowEmptyOrFloatPayload(len)) break;
        PC_TX_ReqChsStatus();
        break;

    /* ── Arm ── */
    case USB_CMD_ARM_DISABLE:
        if (!USB_AllowEmptyOrFloatPayload(len)) break;
        USB_ArmWatchdog_Disarm();
        Arm_HoldCurrentPosition(TOOL_USB_SOURCE);
        Arm_control_flag = 0U;
        break;
    case USB_CMD_ARM_ENABLE:
        if (!USB_AllowEmptyOrFloatPayload(len)) break;
        Arm_control_flag = 1U;
        break;
    case USB_CMD_ARM_SET_TARGET:
        if (!USB_Read4FloatsChecked(d, len, f)) break;
        /* f[0]=x  f[1]=y  f[2]=z (mm) */
        if (USB_Task_flag && Arm_control_flag) {
            if (ArmIK_TargetInputAllowed(f[0], f[1], f[2], 0, 0) != 0U) {
                Arm_task_USB(f[0], f[1], f[2]);
                USB_ArmWatchdog_Feed();
            } else {
                ArmIK_ComponentStep(f[0], f[1], f[2]);
                Arm_HoldCurrentPosition(TOOL_USB_SOURCE);
                USB_ArmWatchdog_Disarm();
            }
        } else {
            USB_ArmWatchdog_Disarm();
            Arm_HoldCurrentPosition(TOOL_USB_SOURCE);
        }
        break;
    case USB_CMD_ARM_STOP:
        if (!USB_AllowEmptyOrFloatPayload(len)) break;
        USB_ArmWatchdog_Disarm();
        Arm_HoldCurrentPosition(TOOL_USB_SOURCE);
        break;
    case USB_CMD_ARM_GET_STATUS:
        if (!USB_AllowEmptyOrFloatPayload(len)) break;
        PC_TX_ReqArmStatus();
        break;

    /* ── Tool ── */
    case USB_CMD_TOOL_DISABLE:
        if (!USB_AllowEmptyOrFloatPayload(len)) break;
        USB_ToolWatchdog_Disarm();
        taskENTER_CRITICAL();
        Tool_HoldSource(TOOL_USB_SOURCE);
        taskEXIT_CRITICAL();
        Tool_control_flag = 0U;
        break;
    case USB_CMD_TOOL_ENABLE:
        if (!USB_AllowEmptyOrFloatPayload(len)) break;
        Tool_control_flag = 1U;
        break;
    case USB_CMD_TOOL_SET_MODE:
        if (!USB_Read4FloatsChecked(d, len, f)) break;
        if (!USB_Task_flag || !Tool_control_flag) break;
        taskENTER_CRITICAL();
        Tool_SetSelectedDev(TOOL_USB_SOURCE, (uint8_t)f[0]);  /* 0=夹爪 1=吸盘 */
        tool_dev = Tool_GetSelectedDev(TOOL_USB_SOURCE);
        taskEXIT_CRITICAL();
        USB_ToolWatchdog_Disarm();
        break;
    case USB_CMD_TOOL_ACTION:
        if (!USB_Read4FloatsChecked(d, len, f)) break;
        {
            uint8_t act = (uint8_t)f[0];  /* 0=闭合 1=张开 */
            if (!USB_Task_flag || !Tool_control_flag) break;
            taskENTER_CRITICAL();
            USB_ToolSetActuator(Tool_GetSelectedDev(TOOL_USB_SOURCE), act);
            taskEXIT_CRITICAL();
            USB_ToolWatchdog_Feed();
        }
        break;
    case USB_CMD_TOOL_SET_STATE:
        if (!USB_Read4FloatsChecked(d, len, f)) break;
        {
            uint8_t dev = (uint8_t)f[0];  /* 0=夹爪 1=吸盘 */
            uint8_t act = (uint8_t)f[1];  /* 0=闭合 1=张开 */
            if (!USB_Task_flag || !Tool_control_flag) break;
            taskENTER_CRITICAL();
            USB_ToolSetActuator(dev, act);
            taskEXIT_CRITICAL();
            USB_ToolWatchdog_Feed();
        }
        break;
    case USB_CMD_TOOL_STOP:
        if (!USB_AllowEmptyOrFloatPayload(len)) break;
        USB_ToolWatchdog_Disarm();
        taskENTER_CRITICAL();
        Tool_StopSource(TOOL_USB_SOURCE);
        taskEXIT_CRITICAL();
        break;
    case USB_CMD_TOOL_GET_STATUS:
        if (!USB_AllowEmptyOrFloatPayload(len)) break;
        PC_TX_ReqToolStatus();
        break;

    case USB_CMD_ROBOT_GET_STATUS:
        if (!USB_AllowEmptyOrFloatPayload(len)) break;
        PC_TX_ReqRobotStatus();
        break;

    /* ── Climb ── */
    case USB_CMD_YAW_TUNE_START:
        if (!USB_AllowEmptyOrFloatPayload(len)) break;
        if (len == 16U) {
            if (!USB_Read4FloatsChecked(d, len, f)) break;
        }
        if (USB_Task_flag && Mecanum_control_flag) {
            uint8_t pass_count = (uint8_t)f[0];
            uint8_t started;
            taskENTER_CRITICAL();
            started = R2_YawAutoTune_Start(&g_r2_ctrl_usb, pass_count);
            taskEXIT_CRITICAL();
            if (started != 0U) {
                USB_ChassisWatchdog_Disarm();
            }
        }
        break;

    case USB_CMD_YAW_TUNE_STOP:
        if (!USB_AllowEmptyOrFloatPayload(len)) break;
        USB_ChassisWatchdog_Disarm();
        taskENTER_CRITICAL();
        R2_YawAutoTune_Stop();
        R2_Move_Stop(&g_r2_ctrl_usb);
        taskEXIT_CRITICAL();
        break;

    case USB_CMD_YAW_TUNE_GET_STATUS:
        if (!USB_AllowEmptyOrFloatPayload(len)) break;
        PC_TX_ReqYawTuneStatus();
        break;

    case USB_CMD_CLIMB_DISABLE:
        if (!USB_AllowEmptyOrFloatPayload(len)) break;
        taskENTER_CRITICAL();
        if (g_r2_climb_usb.test_chassis_active != 0U) {
            R2_Move_Stop(&g_r2_ctrl_usb);
        }
        R2_Climb_Stop(&g_r2_climb_usb);
        taskEXIT_CRITICAL();
        break;

    case USB_CMD_CLIMB_ENABLE:
        if (!USB_AllowEmptyOrFloatPayload(len)) break;
        if (USB_Task_flag) {
            taskENTER_CRITICAL();
            R2_Climb_SetInput(&g_r2_climb_usb, 1U, 0U, 0U);
            taskEXIT_CRITICAL();
        }
        break;

    case USB_CMD_CLIMB_SET_CTRL:
        if (!USB_Read4FloatsChecked(d, len, f)) break;
        if (USB_Task_flag) {
            climb_auto_level = ((uint8_t)f[2]) ? 1U : 0U;
            climb_auto_allowed = 1U;

            taskENTER_CRITICAL();
            if ((((uint8_t)f[0]) == 0U) &&
                (g_r2_climb_usb.test_chassis_active != 0U)) {
                R2_Move_Stop(&g_r2_ctrl_usb);
            }
            /* Consume blocked auto edges so held-high input cannot start later. */
            if ((((uint8_t)f[0]) != 0U) &&
                (climb_auto_level != 0U) &&
                (g_r2_climb_usb.last_auto_level == 0U) &&
                (climb_auto_allowed == 0U)) {
                R2_Climb_SetInput(&g_r2_climb_usb,
                                  1U,
                                  ((uint8_t)f[1]) ? 1U : 0U,
                                  0U);
                g_r2_climb_usb.last_auto_level = 1U;
            } else {
                R2_Climb_SetInput(&g_r2_climb_usb,
                                  ((uint8_t)f[0]) ? 1U : 0U,
                                  ((uint8_t)f[1]) ? 1U : 0U,
                                  climb_auto_level);
                if (((uint8_t)f[0] != 0U) &&
                    (climb_auto_level != 0U) &&
                    (climb_auto_allowed != 0U)) {
                    USB_ChassisWatchdog_Disarm();
                }
            }
            taskEXIT_CRITICAL();
        }
        break;

    case USB_CMD_CLIMB_STEP:
        if (!USB_AllowEmptyOrFloatPayload(len)) break;
        if (USB_Task_flag) {
            taskENTER_CRITICAL();
            R2_Climb_RequestFlowStep(&g_r2_climb_usb,
                                     R2_CLIMB_FLOW_UPSTAIRS);
            taskEXIT_CRITICAL();
        }
        break;

    case USB_CMD_CLIMB_UP_AUTO:
        if (!USB_AllowEmptyOrFloatPayload(len)) break;
        if (USB_Task_flag) {
            taskENTER_CRITICAL();
            USB_ChassisWatchdog_Disarm();
            R2_Climb_RequestFlowAuto(&g_r2_climb_usb,
                                     R2_CLIMB_FLOW_UPSTAIRS);
            taskEXIT_CRITICAL();
        }
        break;

    case USB_CMD_CLIMB_DOWN_STEP:
        if (!USB_AllowEmptyOrFloatPayload(len)) break;
        if (USB_Task_flag) {
            taskENTER_CRITICAL();
            R2_Climb_RequestFlowStep(&g_r2_climb_usb,
                                     R2_CLIMB_FLOW_DOWNSTAIRS);
            taskEXIT_CRITICAL();
        }
        break;

    case USB_CMD_CLIMB_DOWN_AUTO:
        if (!USB_AllowEmptyOrFloatPayload(len)) break;
        if (USB_Task_flag) {
            taskENTER_CRITICAL();
            USB_ChassisWatchdog_Disarm();
            R2_Climb_RequestFlowAuto(&g_r2_climb_usb,
                                     R2_CLIMB_FLOW_DOWNSTAIRS);
            taskEXIT_CRITICAL();
        }
        break;

    case USB_CMD_CLIMB_UP_GATE:
        if (!USB_AllowEmptyOrFloatPayload(len)) break;
        if (USB_Task_flag) {
            taskENTER_CRITICAL();
            USB_ChassisWatchdog_Disarm();
            R2_Climb_RequestFlowGate(&g_r2_climb_usb,
                                     R2_CLIMB_FLOW_UPSTAIRS);
            taskEXIT_CRITICAL();
        }
        break;

    case USB_CMD_CLIMB_DOWN_GATE:
        if (!USB_AllowEmptyOrFloatPayload(len)) break;
        if (USB_Task_flag) {
            taskENTER_CRITICAL();
            USB_ChassisWatchdog_Disarm();
            R2_Climb_RequestFlowGate(&g_r2_climb_usb,
                                     R2_CLIMB_FLOW_DOWNSTAIRS);
            taskEXIT_CRITICAL();
        }
        break;

    case USB_CMD_CLIMB_UP_AUTO_PAUSE:
        if (!USB_AllowEmptyOrFloatPayload(len)) break;
        if (USB_Task_flag) {
            taskENTER_CRITICAL();
            USB_ChassisWatchdog_Disarm();
            R2_Climb_RequestFlowAutoPause(&g_r2_climb_usb,
                                          R2_CLIMB_FLOW_UPSTAIRS);
            taskEXIT_CRITICAL();
        }
        break;

    case USB_CMD_CLIMB_DOWN_AUTO_PAUSE:
        if (!USB_AllowEmptyOrFloatPayload(len)) break;
        if (USB_Task_flag) {
            taskENTER_CRITICAL();
            USB_ChassisWatchdog_Disarm();
            R2_Climb_RequestFlowAutoPause(&g_r2_climb_usb,
                                          R2_CLIMB_FLOW_DOWNSTAIRS);
            taskEXIT_CRITICAL();
        }
        break;

    case USB_CMD_CLIMB_AUTO_RESUME:
        if (!USB_AllowEmptyOrFloatPayload(len)) break;
        if (USB_Task_flag) {
            taskENTER_CRITICAL();
            USB_ChassisWatchdog_Disarm();
            R2_Climb_RequestAutoResume(&g_r2_climb_usb);
            taskEXIT_CRITICAL();
        }
        break;

    case USB_CMD_CLIMB_STOP:
        if (!USB_AllowEmptyOrFloatPayload(len)) break;
        taskENTER_CRITICAL();
        if (g_r2_climb_usb.test_chassis_active != 0U) {
            R2_Move_Stop(&g_r2_ctrl_usb);
        }
        R2_Climb_Stop(&g_r2_climb_usb);
        taskEXIT_CRITICAL();
        break;

    case USB_CMD_CLIMB_TEST_ACTION:
        if (!USB_Read4FloatsChecked(d, len, f)) break;
        if (USB_Task_flag) {
            taskENTER_CRITICAL();
            if (g_r2_climb_usb.test_chassis_active != 0U) {
                R2_Move_Stop(&g_r2_ctrl_usb);
            }
            R2_Climb_RequestTestAction(&g_r2_climb_usb, (uint8_t)f[0]);
            taskEXIT_CRITICAL();
        }
        break;

    case USB_CMD_CLIMB_GET_STATUS:
        if (!USB_AllowEmptyOrFloatPayload(len)) break;
        PC_TX_ReqClimbStatus();
        break;

    default: break;
    }
}


/* ═══════════════════════════════════════════════════════
 *  工具函数
 * ═══════════════════════════════════════════════════════ */

static void USB_CommandRx_Record(uint8_t cmd, const uint8_t *d, uint8_t len)
{
    uint8_t i;
    uint8_t payload_valid;
    uint8_t copy_len = 0U;
    float f[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    payload_valid = ((d != NULL) && (len == 16U)) ? 1U : 0U;
    if (d != NULL) {
        copy_len = (len <= 16U) ? len : 16U;
    }
    if (payload_valid != 0U) {
        USB_Read4Floats(d, f);
    }

    taskENTER_CRITICAL();
    s_usb_cmd_last_tick = HAL_GetTick();
    s_usb_cmd_count++;
    s_usb_cmd_last_cmd = cmd;
    s_usb_cmd_last_len = len;
    s_usb_cmd_last_payload_valid = payload_valid;
    for (i = 0U; i < 16U; i++) {
        s_usb_cmd_last_data[i] = (i < copy_len) ? d[i] : 0U;
    }
    for (i = 0U; i < 4U; i++) {
        s_usb_cmd_last_f[i] = (payload_valid != 0U) ? f[i] : 0.0f;
    }
    taskEXIT_CRITICAL();
}

static void USB_ToolSetActuator(uint8_t dev, uint8_t act)
{
    if ((act != 0U) && (act != 1U)) {
        return;
    }

    if (dev == TOOL_DEV_CHUCK) {
        chuck_Handle_t *usb_chuck = Tool_GetChuck(TOOL_USB_SOURCE);
        if (act == 0U) {
            Tool_SetChuckActuator(usb_chuck, CHUCK_CLOSE);
        } else {
            Tool_SetChuckActuator(usb_chuck, CHUCK_OPEN);
        }
    } else if (dev == TOOL_DEV_CLAMP) {
        clamp_Handle_t *usb_clamp = Tool_GetClamp(TOOL_USB_SOURCE);
        if (act == 0U) {
            Tool_SetClampActuator(usb_clamp, CLAMP_CLOSE);
        } else {
            Tool_SetClampActuator(usb_clamp, CLAMP_OPEN);
        }
    }
}

void USB_GetCommandRxState(USB_CommandRxState_t *out)
{
    uint8_t i;

    if (out == NULL) {
        return;
    }

    taskENTER_CRITICAL();
    out->last_tick = s_usb_cmd_last_tick;
    out->count = s_usb_cmd_count;
    out->last_cmd = s_usb_cmd_last_cmd;
    out->last_len = s_usb_cmd_last_len;
    out->last_payload_valid = s_usb_cmd_last_payload_valid;
    out->reserved[0] = 0U;
    out->reserved[1] = 0U;
    out->reserved[2] = 0U;
    for (i = 0U; i < 16U; i++) {
        out->last_data[i] = s_usb_cmd_last_data[i];
    }
    for (i = 0U; i < 4U; i++) {
        out->last_f[i] = s_usb_cmd_last_f[i];
    }
    taskEXIT_CRITICAL();
}

static void USB_ChassisWatchdog_Feed(void)
{
    taskENTER_CRITICAL();
    s_usb_chassis_last_tick = HAL_GetTick();
    s_usb_chassis_watchdog_armed = 1U;
    s_usb_chassis_timeout = 0U;
    taskEXIT_CRITICAL();
}

static void USB_ChassisWatchdog_Disarm(void)
{
    taskENTER_CRITICAL();
    s_usb_chassis_watchdog_armed = 0U;
    s_usb_chassis_timeout = 0U;
    s_usb_chassis_last_tick = 0U;
    taskEXIT_CRITICAL();
}

void USB_ChassisWatchdog_Check(void)
{
    uint32_t now_tick;

    now_tick = HAL_GetTick();

    taskENTER_CRITICAL();
    if ((USB_Task_flag != 0U) &&
        (Mecanum_control_flag != 0U) &&
        (s_usb_chassis_watchdog_armed != 0U) &&
        (g_r2_ctrl_usb.emergency_stop == 0U) &&
        (R2_Move_IsVelMode(g_r2_ctrl_usb.mode) != 0U) &&
        ((now_tick - s_usb_chassis_last_tick) > USB_CHASSIS_TIMEOUT_MS))
    {
        R2_Move_Stop(&g_r2_ctrl_usb);
        total_vel_USB.vx = 0.0f;
        total_vel_USB.vy = 0.0f;
        total_vel_USB.vw = 0.0f;
        total_speed_USB.fl = 0.0f;
        total_speed_USB.fr = 0.0f;
        total_speed_USB.bl = 0.0f;
        total_speed_USB.br = 0.0f;
        s_usb_chassis_watchdog_armed = 0U;
        s_usb_chassis_timeout = 1U;
    }
    taskEXIT_CRITICAL();
}

uint8_t USB_ChassisWatchdog_IsTimeout(void)
{
    uint8_t timeout;

    taskENTER_CRITICAL();
    timeout = s_usb_chassis_timeout;
    taskEXIT_CRITICAL();

    return timeout;
}

uint32_t USB_ChassisWatchdog_LastTick(void)
{
    uint32_t tick;

    taskENTER_CRITICAL();
    tick = s_usb_chassis_last_tick;
    taskEXIT_CRITICAL();

    return tick;
}

static void USB_ArmWatchdog_Feed(void)
{
    taskENTER_CRITICAL();
    s_usb_arm_last_tick = HAL_GetTick();
    s_usb_arm_watchdog_armed = 1U;
    s_usb_arm_timeout = 0U;
    taskEXIT_CRITICAL();
}

static void USB_ArmWatchdog_Disarm(void)
{
    taskENTER_CRITICAL();
    s_usb_arm_watchdog_armed = 0U;
    s_usb_arm_timeout = 0U;
    s_usb_arm_last_tick = 0U;
    taskEXIT_CRITICAL();
}

static void USB_ToolWatchdog_Feed(void)
{
    taskENTER_CRITICAL();
    s_usb_tool_last_tick = HAL_GetTick();
    s_usb_tool_watchdog_armed = 1U;
    s_usb_tool_timeout = 0U;
    taskEXIT_CRITICAL();
}

static void USB_ToolWatchdog_Disarm(void)
{
    taskENTER_CRITICAL();
    s_usb_tool_watchdog_armed = 0U;
    s_usb_tool_timeout = 0U;
    s_usb_tool_last_tick = 0U;
    taskEXIT_CRITICAL();
}

static uint8_t USB_ArmTargetReached(void)
{
    float target_j1;
    float target_j2;
    float target_j3;
    float actual_j1;
    float actual_j2;
    float actual_j3;
    float err_j1;
    float err_j2;
    float err_j3;

    taskENTER_CRITICAL();
    target_j1 = Clamp(ctrl_J_USB[0], -ARM_IK_J1_LIMIT_DEG, ARM_IK_J1_LIMIT_DEG);
    target_j2 = ctrl_J_USB[1];
    target_j3 = ctrl_J_USB[2];
    actual_j1 = -(float)motor_fdcan3[0].total_angle * USB_ARM_TNUM1;
    actual_j2 =  (float)motor_fdcan3[1].total_angle * USB_ARM_TNUM23;
    actual_j3 =  (float)motor_fdcan3[2].total_angle * USB_ARM_TNUM23;
    taskEXIT_CRITICAL();

    err_j1 = actual_j1 - target_j1;
    err_j2 = actual_j2 - target_j2;
    err_j3 = actual_j3 - target_j3;

    if (err_j1 < 0.0f) err_j1 = -err_j1;
    if (err_j2 < 0.0f) err_j2 = -err_j2;
    if (err_j3 < 0.0f) err_j3 = -err_j3;

    return ((err_j1 <= USB_ARM_TARGET_TOL_DEG) &&
            (err_j2 <= USB_ARM_TARGET_TOL_DEG) &&
            (err_j3 <= USB_ARM_TARGET_TOL_DEG)) ? 1U : 0U;
}

static void USB_ArmWatchdog_Check(uint32_t now_tick)
{
    uint8_t need_check = 0U;

    taskENTER_CRITICAL();
    if ((USB_Task_flag != 0U) &&
        (Arm_control_flag != 0U) &&
        (s_usb_arm_watchdog_armed != 0U) &&
        ((now_tick - s_usb_arm_last_tick) > USB_ARM_TIMEOUT_MS))
    {
        need_check = 1U;
    }
    taskEXIT_CRITICAL();

    if (need_check == 0U) {
        return;
    }

    if (USB_ArmTargetReached() != 0U) {
        taskENTER_CRITICAL();
        s_usb_arm_watchdog_armed = 0U;
        taskEXIT_CRITICAL();
        return;
    }

    Arm_HoldCurrentPosition(TOOL_USB_SOURCE);

    taskENTER_CRITICAL();
    s_usb_arm_watchdog_armed = 0U;
    s_usb_arm_timeout = 1U;
    taskEXIT_CRITICAL();
}

static void USB_ToolWatchdog_Check(uint32_t now_tick)
{
    uint8_t moving = 0U;
    uint8_t timeout = 0U;

    taskENTER_CRITICAL();
    if ((USB_Task_flag != 0U) &&
        (Tool_control_flag != 0U) &&
        (s_usb_tool_watchdog_armed != 0U))
    {
        if (Tool_GetSelectedDev(TOOL_USB_SOURCE) == TOOL_DEV_CHUCK) {
            moving = (Tool_GetChuck(TOOL_USB_SOURCE)->run_status == TOOL_STATUS_MOVING) ? 1U : 0U;
        } else {
            moving = (Tool_GetClamp(TOOL_USB_SOURCE)->run_status == TOOL_STATUS_MOVING) ? 1U : 0U;
        }

        if (moving == 0U) {
            s_usb_tool_watchdog_armed = 0U;
        } else if ((now_tick - s_usb_tool_last_tick) > USB_TOOL_TIMEOUT_MS) {
            s_usb_tool_watchdog_armed = 0U;
            s_usb_tool_timeout = 1U;
            timeout = 1U;
        }
    }
    taskEXIT_CRITICAL();

    if (timeout != 0U) {
        taskENTER_CRITICAL();
        Tool_HoldSource(TOOL_USB_SOURCE);
        if (Tool_GetSelectedDev(TOOL_USB_SOURCE) == TOOL_DEV_CHUCK) {
            Tool_GetChuck(TOOL_USB_SOURCE)->run_status = TOOL_STATUS_ERROR;
        } else {
            Tool_GetClamp(TOOL_USB_SOURCE)->run_status = TOOL_STATUS_ERROR;
        }
        taskEXIT_CRITICAL();
    }
}

void USB_ControlWatchdog_Check(void)
{
    uint32_t now_tick;

    now_tick = HAL_GetTick();

    USB_ChassisWatchdog_Check();
    USB_ArmWatchdog_Check(now_tick);
    USB_ToolWatchdog_Check(now_tick);
}

uint8_t USB_ArmWatchdog_IsTimeout(void)
{
    uint8_t timeout;

    taskENTER_CRITICAL();
    timeout = s_usb_arm_timeout;
    taskEXIT_CRITICAL();

    return timeout;
}

uint32_t USB_ArmWatchdog_LastTick(void)
{
    uint32_t tick;

    taskENTER_CRITICAL();
    tick = s_usb_arm_last_tick;
    taskEXIT_CRITICAL();

    return tick;
}

uint8_t USB_ToolWatchdog_IsTimeout(void)
{
    uint8_t timeout;

    taskENTER_CRITICAL();
    timeout = s_usb_tool_timeout;
    taskEXIT_CRITICAL();

    return timeout;
}

uint32_t USB_ToolWatchdog_LastTick(void)
{
    uint32_t tick;

    taskENTER_CRITICAL();
    tick = s_usb_tool_last_tick;
    taskEXIT_CRITICAL();

    return tick;
}

uint8_t USB_ControlWatchdog_TimeoutFlags(void)
{
    uint8_t flags = 0U;

    taskENTER_CRITICAL();
    if (s_usb_chassis_timeout != 0U) flags |= 0x01U;
    if (s_usb_arm_timeout != 0U)     flags |= 0x02U;
    if (s_usb_tool_timeout != 0U)    flags |= 0x04U;
    taskEXIT_CRITICAL();

    return flags;
}

static void USB_Read4Floats(const uint8_t *d, float *f)
{
    union { uint8_t b[4]; float v; } u;
    u.b[0]=d[0]; u.b[1]=d[1]; u.b[2]=d[2]; u.b[3]=d[3]; f[0]=u.v;
    u.b[0]=d[4]; u.b[1]=d[5]; u.b[2]=d[6]; u.b[3]=d[7]; f[1]=u.v;
    u.b[0]=d[8]; u.b[1]=d[9]; u.b[2]=d[10];u.b[3]=d[11];f[2]=u.v;
    u.b[0]=d[12];u.b[1]=d[13];u.b[2]=d[14];u.b[3]=d[15];f[3]=u.v;
}

static uint8_t USB_Read4FloatsChecked(const uint8_t *d, uint8_t len, float *f)
{
    if ((d == NULL) || (f == NULL) || (len != 16U)) {
        return 0U;
    }

    USB_Read4Floats(d, f);
    /* Reject non-finite control values before limits, enum casts or PID use. */
    if (!isfinite(f[0]) || !isfinite(f[1]) ||
        !isfinite(f[2]) || !isfinite(f[3])) {
        return 0U;
    }
    return 1U;
}

static uint8_t USB_AllowEmptyOrFloatPayload(uint8_t len)
{
    return ((len == 0U) || (len == 16U)) ? 1U : 0U;
}

static float Clamp(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}
