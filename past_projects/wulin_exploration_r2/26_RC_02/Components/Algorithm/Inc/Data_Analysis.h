#ifndef __DATA_ANALYSIS_H__
#define __DATA_ANALYSIS_H__

#include "CAN_Task.h"
#include "Control_Task.h"
#include "INS_Task.h"
#include "bsp_usb.h"

/*
 * USB 命令表；参数命令通常为 16 字节，无参数命令允许空 payload
 *
 * 带参数命令: 0xA5 0x5A 0x10 CMD DATA[16] CRC16 0xFF
 * 简单命令（enable/disable/stop/status）允许 LEN=0，也兼容旧的 LEN=0x10。
 * 需要 float payload 的命令必须 LEN=0x10，data 不足不会被解析。
 *
 *   Module:  0x0=System   0x1=Chassis   0x2=Arm   0x3=Tool
 *            0x4=Robot/YawTune，0x5=Climb，0x90=异步 IK 结果
 *
 *   Action:  0x0=Disable  0x1=Enable    0x2=SetMode/Config
 *            0x3=SetVEL   0x4=SetPOS    0x5=Stop     0x6=Status
 */

/* ── System (0x0_) ── */
#define USB_CMD_SYS_DISABLE         0x00U
#define USB_CMD_SYS_ENABLE          0x01U
#define USB_CMD_SYS_SWITCH_SOURCE   0x02U  /* datas[0]: 0=USART 1=USB */
#define USB_CMD_SYS_STOP            0x05U
#define USB_CMD_SYS_GET_STATUS      0x06U

/* ── Chassis (0x1_) — R2_move 八种模式 ── */
#define USB_CMD_CHS_DISABLE         0x10U
#define USB_CMD_CHS_ENABLE          0x11U
#define USB_CMD_CHS_SET_MODE        0x12U  /* datas[0]: mode 0-7 */
#define USB_CMD_CHS_SET_VEL         0x13U  /* FLU: vx forward, vy left, yaw_data CCW, reserved */
#define USB_CMD_CHS_SET_POS         0x14U  /* FLU: dx forward, dy left, yaw_data CCW, reserved */
#define USB_CMD_CHS_STOP            0x15U
#define USB_CMD_CHS_GET_STATUS      0x16U

/* ── Arm (0x2_) — 3-DOF 机械臂 ── */
#define USB_CMD_ARM_DISABLE         0x20U
#define USB_CMD_ARM_ENABLE          0x21U
#define USB_CMD_ARM_SET_TARGET      0x23U  /* 4 float LE: x,y,z (mm), reserved; arm-base FLU frame */
#define USB_CMD_ARM_STOP            0x25U
#define USB_CMD_ARM_GET_STATUS      0x26U

/* ── Tool (0x3_) — 吸盘/夹爪 ── */
#define USB_CMD_TOOL_DISABLE        0x30U
#define USB_CMD_TOOL_ENABLE         0x31U
#define USB_CMD_TOOL_SET_MODE       0x32U  /* datas[0]: 0=夹爪 1=吸盘 */
#define USB_CMD_TOOL_ACTION         0x33U  /* datas[0]: 0=闭合 1=张开 */
#define USB_CMD_TOOL_SET_STATE      0x34U  /* datas[0]: dev, datas[1]: 0=闭合 1=张开 */
#define USB_CMD_TOOL_STOP           0x35U  /* 停止 USB 工具旋转并关闭执行器，详见 Tool_StopSource */
#define USB_CMD_TOOL_GET_STATUS     0x36U

/* Robot total status (0x4_) */
#define USB_CMD_ROBOT_GET_STATUS    0x46U
#define USB_CMD_YAW_TUNE_START      0x47U  /* optional f0=pass_count, default 1 */
#define USB_CMD_YAW_TUNE_STOP       0x48U
#define USB_CMD_YAW_TUNE_GET_STATUS 0x49U

/* Climb step state machine (0x5_) */
#define USB_CMD_CLIMB_DISABLE       0x50U
#define USB_CMD_CLIMB_ENABLE        0x51U
#define USB_CMD_CLIMB_SET_CTRL      0x52U  /* f0=enable, f1=step, f2=auto */
#define USB_CMD_CLIMB_STEP          0x53U
#define USB_CMD_CLIMB_UP_STEP       USB_CMD_CLIMB_STEP
#define USB_CMD_CLIMB_UP_AUTO       0x54U
#define USB_CMD_CLIMB_AUTO          USB_CMD_CLIMB_UP_AUTO
#define USB_CMD_CLIMB_RUN           USB_CMD_CLIMB_UP_AUTO
#define USB_CMD_CLIMB_UP_RUN        USB_CMD_CLIMB_UP_AUTO
#define USB_CMD_CLIMB_STOP          0x55U
#define USB_CMD_CLIMB_GET_STATUS    0x56U
#define USB_CMD_CLIMB_TEST_ACTION   0x57U  /* f0=R2_ClimbTestAction_t */
#define USB_CMD_CLIMB_DOWN_STEP     0x58U
#define USB_CMD_CLIMB_DOWN_AUTO     0x59U
#define USB_CMD_CLIMB_DOWN_RUN      USB_CMD_CLIMB_DOWN_AUTO
#define USB_CMD_CLIMB_UP_GATE       0x5AU
#define USB_CMD_CLIMB_DOWN_GATE     0x5BU
#define USB_CMD_CLIMB_UP_AUTO_PAUSE 0x5CU
#define USB_CMD_CLIMB_DOWN_AUTO_PAUSE 0x5DU
#define USB_CMD_CLIMB_AUTO_RESUME   0x5EU

#define USB_CHASSIS_TIMEOUT_MS      100U
#define USB_ARM_TIMEOUT_MS          300U
#define USB_TOOL_TIMEOUT_MS         2000U

typedef struct
{
    uint32_t last_tick;
    uint32_t count;
    uint8_t last_cmd;
    uint8_t last_len;
    uint8_t last_payload_valid;
    uint8_t reserved[3];
    uint8_t last_data[16];
    float last_f[4];
} USB_CommandRxState_t;

/* ── 各模块使能标志 ── */
extern uint8_t Mecanum_control_flag;
extern uint8_t Arm_control_flag;
extern uint8_t Tool_control_flag;

extern ChassisVel_t  total_vel_USB;
extern WheelSpeed_t  total_speed_USB;

/* 始终接收 4 个 float（16 字节 LE），各命令按需取用 */
void Data_Analysis(uint8_t cmd, const uint8_t* datas, uint8_t len);
void USB_GetCommandRxState(USB_CommandRxState_t *out);
void USB_ControlWatchdog_Check(void);
void USB_ChassisWatchdog_Check(void);
uint8_t USB_ChassisWatchdog_IsTimeout(void);
uint32_t USB_ChassisWatchdog_LastTick(void);
uint8_t USB_ArmWatchdog_IsTimeout(void);
uint32_t USB_ArmWatchdog_LastTick(void);
uint8_t USB_ToolWatchdog_IsTimeout(void);
uint32_t USB_ToolWatchdog_LastTick(void);
uint8_t USB_ControlWatchdog_TimeoutFlags(void);

#endif
