#include "CRC.h"
#include "usart.h"
#include "struct_typedef.h"
#include "math.h"
#include <string.h>
#include "arm_tools.h"
#include "arm_user.h"
#include "R2_move.h"


/* 串口句柄 */
extern UART_HandleTypeDef huart10;

/* 全局接收状态 */
ParseState BT_Uart10 = STATE_WAIT_HEADER;
uint8_t bt_data[BT_FRAME_DATA_LEN];
uint8_t data_index = 0;
uint8_t checksum = 0;
volatile uint8_t bt_parse_ok = 0;
uint8_t btReceiveData = 0;

/* 调试观测：最新一帧的解析结果 */
CRC_Debug_t crc_dbg = {0};

volatile uint8_t USB_Task_flag   = 0U;
volatile uint8_t USART_Task_flag = 1U;  /* 上电默认 USART 源激活 */
uint8_t UU_flag = 0;

/* 机械臂控制量 */
int8_t arm_flag = 0;
float arm_X = 0.0f;
float arm_Y = 0.0f;
float arm_Z = 0.0f;
uint8_t arm_input_valid = 0U;

uint8_t tool_flag = TOOL_DEV_CLAMP;
uint8_t clampuse_flag = CLAMP_CLOSE;
uint8_t chuckuse_flag = CHUCK_CLOSE;
uint8_t climb_enable_flag = 0U;
uint8_t climb_step_flag = 0U;
uint8_t climb_auto_flag = 0U;


extern void Arm_HoldCurrentPosition(uint8_t source);

static volatile uint8_t s_usart_control_timeout = 0U;
static volatile uint32_t s_usart_control_timeout_tick = 0U;

void Control_SetSource(uint8_t source)
{
    if (source == TOOL_USB_SOURCE) {
        USB_Task_flag = 1U;
        USART_Task_flag = 0U;
        Tool_SetActiveSource(TOOL_USB_SOURCE);
    } else {
        USART_Task_flag = 1U;
        USB_Task_flag = 0U;
        Tool_SetActiveSource(TOOL_USART_SOURCE);
    }
}

/*
 * UART10 单字节接收状态机
 *
 * 帧格式：0xA5 + 40 字节数据区 + 1 字节校验和 + 0x5A
 * 校验和 = data[0] + ... + data[39] 取低 8 位
 */
void UART10_Receive(uint8_t receiveData)
{
    switch (BT_Uart10)
    {
        case STATE_WAIT_HEADER:
        {
            if (receiveData == 0xA5U)
            {
                BT_Uart10 = STATE_RECV_DATA;
                data_index = 0;
                memset(bt_data, 0, sizeof(bt_data));
            }
        }
        break;

        case STATE_RECV_DATA:
        {
            bt_data[data_index++] = receiveData;

            if (data_index >= BT_FRAME_DATA_LEN)
            {
                BT_Uart10 = STATE_RECV_CHECKSUM;
            }
        }
        break;

        case STATE_RECV_CHECKSUM:
        {
            checksum = receiveData;
            BT_Uart10 = STATE_RECV_TAIL;
        }
        break;

        case STATE_RECV_TAIL:
        {
            if (receiveData == 0x5AU)
            {
                uint8_t calc_checksum = 0U;
                uint8_t i;

                for (i = 0; i < BT_FRAME_DATA_LEN; i++)
                {
                    calc_checksum += bt_data[i];
                }

                crc_dbg.checksum_calc = calc_checksum;
                crc_dbg.checksum_recv = checksum;

                if (calc_checksum == checksum)
                {
                    crc_dbg.last_frame_tick = HAL_GetTick();
                    crc_dbg.checksum_ok = 1U;
                    bt_parse_ok = 1U;
                }
                else
                {
                    crc_dbg.checksum_fail_count++;
                    crc_dbg.last_checksum_fail_tick = HAL_GetTick();
                    crc_dbg.checksum_ok = 0U;
                }
            }

            BT_Uart10 = STATE_WAIT_HEADER;
            data_index = 0;
        }
        break;

        default:
        {
            BT_Uart10 = STATE_WAIT_HEADER;
            data_index = 0;
        }
        break;
    }
}

/*
 * 从帧中读取一个 float（little-endian）。
 */
static float read_float_le(const uint8_t *buf)
{
    union { uint8_t b[4]; float f; } u;
    u.b[0] = buf[0];
    u.b[1] = buf[1];
    u.b[2] = buf[2];
    u.b[3] = buf[3];
    return u.f;
}

/*
 * R2 底盘数据解析（直接传 float，无需 int16 编解码）。
 *
 * @param ctrl  目标控制器
 * @param mode  0-7 运动模式
 * @param p1    VEL:vx(m/s) / POS:dx(m)
 * @param p2    VEL:vy(m/s) / POS:dy(m)
 * @param p3    yaw_data:
 *               ROBOT_NO_YAW: target_yaw_robot_deg
 *               WORLD_NO_YAW: target_yaw_world_deg
 *               other VEL: vw(rad/s)
 *               other POS: dyaw(rad)
 */
void R2_Chassis_Process(R2_Move_Ctrl_t *ctrl, uint8_t mode,
                        float p1, float p2, float p3)
{
    R2_MoveMode_t m;

    if (ctrl == NULL) {
        return;
    }

    /* 全 0 = 静止；速度模式走斜坡减速，其他模式直接停止 */
    if (mode > 7U) {
        if (R2_Move_IsVelMode(ctrl->mode)) {
            R2_Move_SetVel(ctrl, 0.0f, 0.0f, 0.0f);
        } else {
            R2_Move_Stop(ctrl);
        }
        return;
    }

    m = (R2_MoveMode_t)mode;

    if (ctrl->mode != m) {
        R2_Move_SetMode(ctrl, m);
    }

    if (R2_Move_IsNoYawMode(m)) {
        if (R2_Move_IsWorldMode(m)) {
            R2_Move_SetWorldLockYaw(ctrl, p3 * 0.0174533f);
        } else {
            R2_Move_SetRobotLockYaw(ctrl, p3 * 0.0174533f);
        }

        if (R2_Move_IsVelMode(m)) {
            R2_Move_SetVel(ctrl, p1, p2, 0.0f);
        } else {
            R2_Move_SetDist(ctrl, p1, p2, 0.0f);
        }
        return;
    }

    if (R2_Move_IsVelMode(m)) {
        R2_Move_SetVel(ctrl, p1, p2, p3);
    } else {
        R2_Move_SetDist(ctrl, p1, p2, p3);
    }
}

/*
 * 处理完整一帧 UART10 遥控数据。
 * CRC 仅服务 USART 遥控器 → 固定写入 g_r2_ctrl_usart。
 *
 * 帧布局：
 *   byte 0~12  : 控制字段（mode, arm, UU, tool position, clamp, chuck）
 *   byte 13~15 : climb_enable, climb_step, climb_auto
 *   byte 16~39 : 6 个 float（chassis×3 + arm×3）
 *                 chassis param1=X forward, param2=Y left;
 *                 chassis param3 follows yaw_data:
 *                 ROBOT_NO_YAW=target_yaw_robot_deg,
 *                 WORLD_NO_YAW=target_yaw_world_deg,
 *                 other VEL=vw(rad/s), other POS=dyaw(rad)
 */
void BT_Data_MAC_Process(float *V_x, float *V_y, float *V_w, int8_t *cmd)
{
    uint8_t frame[BT_FRAME_DATA_LEN];
    uint8_t mode;
    float   chs_p1, chs_p2, chs_p3;
    float   ax, ay, az;

    (void)cmd;
    (void)V_x; (void)V_y; (void)V_w;

    if (bt_parse_ok == 0U) return;

    __disable_irq();
    memcpy(frame, bt_data, sizeof(frame));
    bt_parse_ok = 0U;
    __enable_irq();

    /* ── byte 0~7: 8 个模式标志位，同时只有一个为 1，全 0 = 静止 ── */
    {
        uint8_t i;
        mode = 0xFFU;
        for (i = 0U; i < 8U; i++) {
            if (frame[i] == 1U) { mode = i; break; }
        }
    }

    /* ── byte 8~12: 5 控制字节 ── */
    arm_flag     = (int8_t)frame[8];  /* ARM */
    UU_flag      = frame[9];          /* UU  */
    tool_flag    = frame[10];         /* tool position */
    clampuse_flag = frame[11];        /* clamp state */
    chuckuse_flag = frame[12];        /* chuck state */
    climb_enable_flag = frame[13];
    climb_step_flag   = frame[14];
    climb_auto_flag   = frame[15];

    /* Current float payload starts after the 3 climb bytes: byte 16~39. */

    /* ── 6 float 数据区（byte 16~39） ── */
    chs_p1 = read_float_le(&frame[BT_FRAME_FLOAT_OFFSET + 0U]);   /* chassis X: forward */
    chs_p2 = read_float_le(&frame[BT_FRAME_FLOAT_OFFSET + 4U]);   /* chassis Y: left */
    chs_p3 = read_float_le(&frame[BT_FRAME_FLOAT_OFFSET + 8U]);   /* chassis param3 */
    ax     = read_float_le(&frame[BT_FRAME_FLOAT_OFFSET + 12U]);  /* arm_x (mm) */
    ay     = read_float_le(&frame[BT_FRAME_FLOAT_OFFSET + 16U]);  /* arm_y (mm) */
    az     = read_float_le(&frame[BT_FRAME_FLOAT_OFFSET + 20U]);  /* arm_z (mm) */

    /* ── 填充调试观测变量 ── */
    crc_dbg.mode         = mode;
    crc_dbg.chs_p1       = chs_p1;
    crc_dbg.chs_p2       = chs_p2;
    crc_dbg.chs_p3       = chs_p3;
    crc_dbg.arm_x        = ax;
    crc_dbg.arm_y        = ay;
    crc_dbg.arm_z        = az;
    crc_dbg.arm_flag     = (uint8_t)arm_flag;
    crc_dbg.uu_flag      = UU_flag;
    crc_dbg.tool_flag    = tool_flag;
    crc_dbg.clampuse_flag = clampuse_flag;
    crc_dbg.chuckuse_flag = chuckuse_flag;
    crc_dbg.climb_enable = climb_enable_flag;
    crc_dbg.climb_step   = climb_step_flag;
    crc_dbg.climb_auto   = climb_auto_flag;
    crc_dbg.frame_count++;
    crc_dbg.last_frame_tick = HAL_GetTick();
    crc_dbg.checksum_ok  = 1U;
    crc_dbg.control_timeout = 0U;
    s_usart_control_timeout = 0U;

    /* ── 机械臂控制 ── */
    arm_input_valid = 0U;
    if ((arm_flag == 1) && (UU_flag == 0U)) {
        if (ArmIK_TargetInputAllowed(ax, ay, az, 0, 0) != 0U) {
            arm_X = ax;
            arm_Y = ay;
            arm_Z = az;
            arm_input_valid = 1U;
        } else {
            ArmIK_ComponentStep(ax, ay, az);
            Arm_HoldCurrentPosition(TOOL_USART_SOURCE);
        }
    } else if (arm_flag != 1) {
        /* Keep the last target values for debug; Control_Task holds the arm. */
    }

    /* ── USART/USB 控制源切换 ── */
    Control_SetSource((UU_flag == 1U) ? TOOL_USB_SOURCE : TOOL_USART_SOURCE);

    /* ── R2 底盘/上台阶控制（仅 USART 源写入 USART 控制器） ── */
    if (USART_Task_flag == 1U) {
        R2_Chassis_Process(&g_r2_ctrl_usart, mode, chs_p1, chs_p2, chs_p3);
        R2_Climb_SetInput(&g_r2_climb_usart,
                          climb_enable_flag,
                          climb_step_flag,
                          climb_auto_flag);
    } else {
        R2_Move_Stop(&g_r2_ctrl_usart);
        R2_Climb_Stop(&g_r2_climb_usart);
    }

    /* ── 工具控制 ── */
    if (USART_Task_flag == 1U) {
        Tool_SetSelectedDev(TOOL_USART_SOURCE, tool_flag);

        Tool_SetClampActuator(Tool_GetClamp(TOOL_USART_SOURCE),
                              (clampuse_flag != 0U) ? CLAMP_OPEN : CLAMP_CLOSE);
        Tool_SetChuckActuator(Tool_GetChuck(TOOL_USART_SOURCE),
                              (chuckuse_flag != 0U) ? CHUCK_OPEN : CHUCK_CLOSE);
    }
}


void USART_ControlWatchdog_Check(void)
{
    uint32_t now_tick;
    uint32_t last_tick;
    uint8_t need_hold = 0U;

    now_tick = HAL_GetTick();

    __disable_irq();
    last_tick = crc_dbg.last_frame_tick;
    if ((USART_Task_flag != 0U) &&
        (last_tick != 0U) &&
        (s_usart_control_timeout == 0U) &&
        ((now_tick - last_tick) > USART_CONTROL_TIMEOUT_MS))
    {
        s_usart_control_timeout = 1U;
        s_usart_control_timeout_tick = now_tick;
        crc_dbg.control_timeout = 1U;
        need_hold = 1U;
    }
    __enable_irq();

    if (need_hold != 0U) {
        R2_Move_Stop(&g_r2_ctrl_usart);
        R2_Climb_Stop(&g_r2_climb_usart);
        Arm_HoldCurrentPosition(TOOL_USART_SOURCE);
        Tool_HoldSource(TOOL_USART_SOURCE);
    }
}

uint8_t USART_ControlWatchdog_IsTimeout(void)
{
    uint8_t timeout;

    __disable_irq();
    timeout = s_usart_control_timeout;
    __enable_irq();

    return timeout;
}
