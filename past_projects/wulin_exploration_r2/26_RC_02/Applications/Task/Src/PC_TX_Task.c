#include "PC_TX_Task.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bsp_usb.h"
#include "R2_move.h"
#include "R2_laser_user.h"
#include "R2_yaw_autotune.h"
#include "arm_user.h"
#include "arm_tools.h"
#include "fdcan_receive.h"
#include "INS_Task.h"
#include "CRC.h"
#include "robot_frame.h"

extern motor_measure_t motor_fdcan2[8];

volatile RobotStatusView_t g_robot_status_view;

/* ── 外部引用 ── */

/* 底盘控制器（USB 通道） */
extern R2_Move_Ctrl_t g_r2_ctrl_usb;
extern R2_Move_Ctrl_t g_r2_ctrl_usart;

/* 任务/模块标志 */
extern volatile uint8_t USB_Task_flag;
extern volatile uint8_t USART_Task_flag;
extern uint8_t Mecanum_control_flag;
extern uint8_t Arm_control_flag;
extern uint8_t Tool_control_flag;

/* 电机数据 */
extern motor_measure_t motor_fdcan1[8];  /* 底盘，在线检测用 */
extern motor_measure_t motor_fdcan3[8];  /* 机械臂，实际编码器回传 */


/* ═══════════════════════════════════════════════════════
 *  状态请求 FIFO（Data_Analysis 入队，本任务每轮最多消费一个请求）
 * ═══════════════════════════════════════════════════════ */

#define PC_TX_STATUS_QUEUE_LEN  16U

static volatile uint8_t  s_status_queue[PC_TX_STATUS_QUEUE_LEN];
static volatile uint8_t  s_status_q_head = 0U;
static volatile uint8_t  s_status_q_tail = 0U;
static volatile uint8_t  s_status_q_count = 0U;
static volatile uint32_t s_status_q_drop_count = 0U;

static void PC_TX_EnqueueStatus(uint8_t cmd)
{
    taskENTER_CRITICAL();
    if (s_status_q_count < PC_TX_STATUS_QUEUE_LEN) {
        s_status_queue[s_status_q_tail] = cmd;
        s_status_q_tail++;
        if (s_status_q_tail >= PC_TX_STATUS_QUEUE_LEN) {
            s_status_q_tail = 0U;
        }
        s_status_q_count++;
    } else {
        s_status_q_drop_count++;
    }
    taskEXIT_CRITICAL();
}

static uint8_t PC_TX_DequeueStatus(uint8_t *cmd)
{
    uint8_t ok = 0U;

    taskENTER_CRITICAL();
    if (s_status_q_count != 0U) {
        *cmd = s_status_queue[s_status_q_head];
        s_status_q_head++;
        if (s_status_q_head >= PC_TX_STATUS_QUEUE_LEN) {
            s_status_q_head = 0U;
        }
        s_status_q_count--;
        ok = 1U;
    }
    taskEXIT_CRITICAL();

    return ok;
}


/* ── Data_Analysis 调用的请求接口 ── */

void PC_TX_ReqSysStatus(void)   { PC_TX_EnqueueStatus(USB_CMD_SYS_GET_STATUS); }
void PC_TX_ReqChsStatus(void)   { PC_TX_EnqueueStatus(USB_CMD_CHS_GET_STATUS); }
void PC_TX_ReqArmStatus(void)   { PC_TX_EnqueueStatus(USB_CMD_ARM_GET_STATUS); }
void PC_TX_ReqToolStatus(void)  { PC_TX_EnqueueStatus(USB_CMD_TOOL_GET_STATUS); }
void PC_TX_ReqRobotStatus(void) { PC_TX_EnqueueStatus(USB_CMD_ROBOT_GET_STATUS); }
void PC_TX_ReqClimbStatus(void) { PC_TX_EnqueueStatus(USB_CMD_CLIMB_GET_STATUS); }
void PC_TX_ReqYawTuneStatus(void) { PC_TX_EnqueueStatus(USB_CMD_YAW_TUNE_GET_STATUS); }


/* ═══════════════════════════════════════════════════════
 *  工具函数
 * ═══════════════════════════════════════════════════════ */

/* float → 4 字节小端，写入 buf[offset..offset+3] */
static void PackFloatLE(float v, uint8_t *buf, uint8_t offset)
{
    union { float f; uint8_t b[4]; } u;
    u.f = v;
    buf[offset    ] = u.b[0];
    buf[offset + 1] = u.b[1];
    buf[offset + 2] = u.b[2];
    buf[offset + 3] = u.b[3];
}

static void PackU32LE(uint32_t v, uint8_t *buf, uint8_t offset)
{
    buf[offset    ] = (uint8_t)( v        & 0xFFU);
    buf[offset + 1] = (uint8_t)((v >>  8) & 0xFFU);
    buf[offset + 2] = (uint8_t)((v >> 16) & 0xFFU);
    buf[offset + 3] = (uint8_t)((v >> 24) & 0xFFU);
}

static void PackI32LE(int32_t v, uint8_t *buf, uint8_t offset)
{
    PackU32LE((uint32_t)v, buf, offset);
}

static float AbsF(float x)
{
    return (x < 0.0f) ? -x : x;
}

/* 电机在线计数（msg_cnt > 50 视为在线） */
static uint8_t MotorOnline(const motor_measure_t *m, uint8_t n)
{
    uint8_t cnt = 0U;
    uint8_t i;
    for (i = 0U; i < n; i++) {
        if (m[i].msg_cnt > 50U) cnt++;
    }
    return cnt;
}

#define PC_TX_CHASSIS_VEL_MOVE_TOL  0.01f
#define PC_TX_CHASSIS_WZ_MOVE_TOL   0.01f
#define PC_TX_ARM_MOVE_TOL_DEG      2.0f


/* ═══════════════════════════════════════════════════════
 *  System 状态回传 (cmd=0x06) - LEN=8
 * ═══════════════════════════════════════════════════════
 *
 *  [0] USB_Task_flag
 *  [1] USART_Task_flag
 *  [2] Mecanum_control_flag
 *  [3] Arm_control_flag
 *  [4] Tool_control_flag
 *  [5] 底盘电机在线数 (0~4)
 *  [6..7] reserved
 */
static void SendSysStatus(void)
{
    uint8_t buf[8];
    buf[0] = USB_Task_flag;
    buf[1] = USART_Task_flag;
    buf[2] = Mecanum_control_flag;
    buf[3] = Arm_control_flag;
    buf[4] = Tool_control_flag;
    buf[5] = MotorOnline(motor_fdcan1, CHASSIS_MOTOR_COUNT);
    buf[6] = USB_ControlWatchdog_TimeoutFlags();
    buf[7] = 0U;
    Send_Cmd_Data(USB_CMD_SYS_GET_STATUS, buf, 8U);
}


/* ═══════════════════════════════════════════════════════
 *  Chassis 状态回传 (cmd=0x16) - LEN=112
 * ═══════════════════════════════════════════════════════
 *
 *  当前模式 / 运动状态 / 实时速度 / 实时里程计 / 运动限制 / 世界系朝向
 *  / POS 诊断（进度 + 位置误差，供上位机判断真实到位）
 *  / 目标速度与目标位移
 *  所有 X/vx/dx 沿前向，Y/vy/dy 沿左向，yaw/wz 逆时针为正。
 *
 *  [0]     mode            (uint8, 0~7)
 *  [1]     pos_state       (uint8, 0=IDLE 1=RUNNING 2=DONE)
 *  [2]     emergency_stop  (uint8, 0/1)
 *  [3]     reserved
 *  [4..7]  robot_vel.vx    (float LE, m/s)
 *  [8..11] robot_vel.vy    (float LE, m/s)
 *  [12..15]robot_vel.vw    (float LE, rad/s)
 *  [16..19]odom_x          (float LE, m)
 *  [20..23]odom_y          (float LE, m)
 *  [24..27]odom_yaw        (float LE, rad)
 *  [28..31]v_max           (float LE, m/s)
 *  [32..35]a_max           (float LE, m/s²)
 *  [36..39]j_max           (float LE, m/s³)
 *  [40..43]pos_progress    (float LE, 0~1,  当前运动进度)
 *  [44..47]pos_err_x       (float LE, m,    位置误差)
 *  [48..51]pos_err_y       (float LE, m)
 *  [52..55]pos_err_yaw     (float LE, rad,  yaw 误差)
 *  [56..83]INS nav         (x/y/yaw/yaw_total/vx/vy/wz)
 *  [84]    imu_online      (uint8)
 *  [85]    timeout_flags   (uint8, bit0=USB chassis, bit1=USB arm, bit2=USB tool)
 *  [86]    status_flags    (uint8)
 *  [87]    error_flags     (uint8)
 *  [88..111]target_vx/vy/vw, target_dx/dy/dyaw (float LE)
 */
#define CHS_STATUS_LEN  112U

static void SendChsStatus(void)
{
    uint8_t buf[CHS_STATUS_LEN];
    R2_Move_Ctrl_t snapshot;
    const R2_Move_Ctrl_t *c = &snapshot;
    INS_NavState_t nav;
    uint8_t chassis_motor_online;
    uint8_t pos_running;
    uint8_t moving;
    uint8_t timeout_flags;
    uint8_t status_flags;
    uint8_t error_flags;

    taskENTER_CRITICAL();
    snapshot = g_r2_ctrl_usb;
    taskEXIT_CRITICAL();
    INS_GetState(&nav);

    chassis_motor_online = MotorOnline(motor_fdcan1, CHASSIS_MOTOR_COUNT);
    pos_running = (c->pos_state == R2_POS_RUNNING) ? 1U : 0U;
    moving = ((pos_running != 0U) ||
              (AbsF(c->robot_vel.vx) > PC_TX_CHASSIS_VEL_MOVE_TOL) ||
              (AbsF(c->robot_vel.vy) > PC_TX_CHASSIS_VEL_MOVE_TOL) ||
              (AbsF(c->robot_vel.vw) > PC_TX_CHASSIS_WZ_MOVE_TOL)) ? 1U : 0U;
    timeout_flags = USB_ControlWatchdog_TimeoutFlags();
    status_flags =
        ((Mecanum_control_flag != 0U) ? 0x01U : 0U) |
        ((R2_Move_IsVelMode(c->mode) != 0U) ? 0x02U : 0U) |
        ((R2_Move_IsPosMode(c->mode) != 0U) ? 0x04U : 0U) |
        ((pos_running != 0U) ? 0x08U : 0U) |
        ((moving != 0U) ? 0x10U : 0U) |
        ((c->pos_state == R2_POS_DONE) ? 0x20U : 0U) |
        ((nav.imu_online != 0U) ? 0x40U : 0U) |
        ((chassis_motor_online >= CHASSIS_MOTOR_COUNT) ? 0x80U : 0U);
    error_flags =
        (((timeout_flags & 0x01U) != 0U) ? 0x01U : 0U) |
        ((nav.imu_online == 0U) ? 0x02U : 0U) |
        ((c->emergency_stop != 0U) ? 0x04U : 0U) |
        ((chassis_motor_online == 0U) ? 0x08U : 0U) |
        (((chassis_motor_online > 0U) &&
          (chassis_motor_online < CHASSIS_MOTOR_COUNT)) ? 0x10U : 0U);

    buf[0] = (uint8_t)c->mode;
    buf[1] = (uint8_t)c->pos_state;
    buf[2] = c->emergency_stop;
    buf[3] = 0U;   /* reserved */

    PackFloatLE(c->robot_vel.vx, buf,  4);
    PackFloatLE(c->robot_vel.vy, buf,  8);
    PackFloatLE(c->robot_vel.vw, buf, 12);
    PackFloatLE(c->odom_x,       buf, 16);
    PackFloatLE(c->odom_y,       buf, 20);
    PackFloatLE(c->odom_yaw,     buf, 24);
    PackFloatLE(c->v_max,        buf, 28);
    PackFloatLE(c->a_max,        buf, 32);
    PackFloatLE(c->j_max,        buf, 36);
    PackFloatLE(c->pos_progress, buf, 40);
    PackFloatLE(c->pos_err_x,    buf, 44);
    PackFloatLE(c->pos_err_y,    buf, 48);
    PackFloatLE(c->pos_err_yaw,  buf, 52);
    PackFloatLE(nav.x_m,         buf, 56);
    PackFloatLE(nav.y_m,         buf, 60);
    PackFloatLE(nav.yaw_rad,     buf, 64);
    PackFloatLE(nav.yaw_total_rad, buf, 68);
    PackFloatLE(nav.vx_mps,      buf, 72);
    PackFloatLE(nav.vy_mps,      buf, 76);
    PackFloatLE(nav.wz_radps,    buf, 80);
    buf[84] = nav.imu_online;
    buf[85] = timeout_flags;
    buf[86] = status_flags;
    buf[87] = error_flags;
    PackFloatLE(c->target_vx,    buf,  88);
    PackFloatLE(c->target_vy,    buf,  92);
    PackFloatLE(c->target_vw,    buf,  96);
    PackFloatLE(c->target_dx,    buf, 100);
    PackFloatLE(c->target_dy,    buf, 104);
    PackFloatLE(c->target_dyaw,  buf, 108);

    Send_Cmd_Data(USB_CMD_CHS_GET_STATUS, buf, CHS_STATUS_LEN);
}


/* ═══════════════════════════════════════════════════════
 *  Arm 状态回传 (cmd=0x26) - LEN=64
 * ═══════════════════════════════════════════════════════
 *
 *  逆解结果 / 命令采纳 / 电机目标值 / 电机实际编码器（物理闭环）
 *  requested_x/y/z 使用机械臂基座 FLU：X 前、Y 左、Z 上。
 *
 *  [0]     has_last_valid   (uint8)
 *  [1]     last_status_code (uint8, 0=OK 1=UNREACHABLE 2=UNSAFE 3=PARAM_ERR)
 *  [2]     last_action_code (uint8, 0=APPLY_NEW 1=HOLD_LAST 2=KEEP_CURRENT)
 *  [3]     reserved
 *  [4..7]  model_theta1     (float LE, rad) — 逆解结果
 *  [8..11] model_theta2     (float LE, rad)
 *  [12..15]model_theta3     (float LE, rad)
 *  [16..19]motor_j1_target  (float LE, deg) — 电机目标值
 *  [20..23]motor_j2_target  (float LE, deg)
 *  [24..27]motor_j3_target  (float LE, deg)
 *  [28..31]actual_j1_deg    (float LE, deg) — 编码器实际值（物理闭环）
 *  [32..35]actual_j2_deg    (float LE, deg)
 *  [36..39]actual_j3_deg    (float LE, deg)
 *  [40..51]requested_x/y/z  (float LE, mm)
 *  [52]    reachable        (uint8)
 *  [53]    safe             (uint8)
 *  [54]    unsafe_reason    (uint8)
 *  [55]    actual_valid     (uint8)
 *  [56]    motor_online     (uint8, FDCAN3 ID1..3)
 *  [57]    status_flags     (uint8)
 *  [58]    error_flags      (uint8)
 *  [59]    timeout_flags    (uint8)
 *  [60..63]max_abs_err_deg  (float LE)
 */
#define ARM_STATUS_LEN  64U

/*
 * 编码器 → 关节角换算系数（与 CAN_Task 一致）。
 * 符号方向：pid_call_3 中 J1 传入负值，此处回传同样取反。
 */
#define ARM_TNUM1   0.0002464f   /* 360/8192/36/94*19, J1 编码器→deg */
#define ARM_TNUM23  0.0004577f   /* 360/8192/3591*187/5, J2/J3 编码器→deg */

static void SendArmStatus(void)
{
    uint8_t buf[ARM_STATUS_LEN];
    const ArmIK_FullState_t *arm_state;
    float actual_j1, actual_j2, actual_j3;
    float err_j1, err_j2, err_j3, max_err;
    uint8_t arm_motor_online;
    uint8_t timeout_flags;
    uint8_t status_flags;
    uint8_t error_flags;

    actual_j1 = -(float)motor_fdcan3[0].total_angle * ARM_TNUM1;
    actual_j2 =  (float)motor_fdcan3[1].total_angle * ARM_TNUM23;
    actual_j3 =  (float)motor_fdcan3[2].total_angle * ARM_TNUM23;
    ArmIK_SetActualMotorDeg(actual_j1,
                            actual_j2,
                            actual_j3,
                            (MotorOnline(motor_fdcan3, 3U) >= 3U) ? 1U : 0U);

    arm_state = ArmIK_GetFullState();

    if (arm_state == NULL) {
        for (uint8_t i = 0U; i < ARM_STATUS_LEN; i++) buf[i] = 0U;
        Send_Cmd_Data(USB_CMD_ARM_GET_STATUS, buf, ARM_STATUS_LEN);
        return;
    }

    arm_motor_online = MotorOnline(motor_fdcan3, 3U);
    timeout_flags = USB_ControlWatchdog_TimeoutFlags();
    err_j1 = AbsF(arm_state->actual_motor_deg.j1_deg - arm_state->active_motor_deg.j1_deg);
    err_j2 = AbsF(arm_state->actual_motor_deg.j2_deg - arm_state->active_motor_deg.j2_deg);
    err_j3 = AbsF(arm_state->actual_motor_deg.j3_deg - arm_state->active_motor_deg.j3_deg);
    max_err = err_j1;
    if (err_j2 > max_err) max_err = err_j2;
    if (err_j3 > max_err) max_err = err_j3;
    status_flags =
        ((Arm_control_flag != 0U) ? 0x01U : 0U) |
        (((max_err > PC_TX_ARM_MOVE_TOL_DEG) &&
          (Arm_control_flag != 0U)) ? 0x02U : 0U) |
        ((arm_motor_online >= 3U) ? 0x04U : 0U) |
        ((arm_state->actual_motor_valid != 0U) ? 0x08U : 0U) |
        ((arm_state->reachable != 0U) ? 0x10U : 0U) |
        ((arm_state->safe != 0U) ? 0x20U : 0U) |
        ((arm_state->has_last_valid != 0U) ? 0x40U : 0U) |
        ((arm_state->last_status_code == ARM_IK_RESULT_OK) ? 0x80U : 0U);
    error_flags =
        ((arm_state->last_status_code != ARM_IK_RESULT_OK) ? 0x01U : 0U) |
        ((arm_state->last_status_code == ARM_IK_RESULT_UNREACHABLE) ? 0x02U : 0U) |
        ((arm_state->last_status_code == ARM_IK_RESULT_UNSAFE) ? 0x04U : 0U) |
        ((arm_state->last_status_code == ARM_IK_RESULT_PARAM_ERR) ? 0x08U : 0U) |
        (((timeout_flags & 0x02U) != 0U) ? 0x10U : 0U) |
        ((arm_motor_online < 3U) ? 0x20U : 0U) |
        ((arm_state->actual_motor_valid == 0U) ? 0x40U : 0U);

    buf[0] = arm_state->has_last_valid;
    buf[1] = arm_state->last_status_code;
    buf[2] = arm_state->last_action_code;
    buf[3] = 0U;

    /* 逆解结果 — 模型控制角 (rad) */
    PackFloatLE(arm_state->active_model_rad.theta1, buf,  4);
    PackFloatLE(arm_state->active_model_rad.theta2, buf,  8);
    PackFloatLE(arm_state->active_model_rad.theta3, buf, 12);

    /* 电机目标值 (deg) */
    PackFloatLE(arm_state->active_motor_deg.j1_deg, buf, 16);
    PackFloatLE(arm_state->active_motor_deg.j2_deg, buf, 20);
    PackFloatLE(arm_state->active_motor_deg.j3_deg, buf, 24);

    /* 编码器实际值 → 关节角 (deg)，符号与 CAN_Task pid_call_3 一致 */
    PackFloatLE(arm_state->actual_motor_deg.j1_deg, buf, 28);
    PackFloatLE(arm_state->actual_motor_deg.j2_deg, buf, 32);
    PackFloatLE(arm_state->actual_motor_deg.j3_deg, buf, 36);
    PackFloatLE(arm_state->requested_pt.x, buf, 40);
    PackFloatLE(arm_state->requested_pt.y, buf, 44);
    PackFloatLE(arm_state->requested_pt.z, buf, 48);
    buf[52] = arm_state->reachable;
    buf[53] = arm_state->safe;
    buf[54] = (uint8_t)arm_state->unsafe_reason;
    buf[55] = arm_state->actual_motor_valid;
    buf[56] = arm_motor_online;
    buf[57] = status_flags;
    buf[58] = error_flags;
    buf[59] = timeout_flags;
    PackFloatLE(max_err, buf, 60);

    Send_Cmd_Data(USB_CMD_ARM_GET_STATUS, buf, ARM_STATUS_LEN);
}


/* ═══════════════════════════════════════════════════════
 *  Tool 状态回传 (cmd=0x36) - LEN=32
 * ═══════════════════════════════════════════════════════
 *
 *  当前状态 / 实体状态 / 真实角度 / 安全标志
 *
 *  [0]     tool_dev        (uint8, 0=夹爪 1=吸盘)
 *  [1]     clamp.state     (uint8, 0=CLOSE 1=OPEN)
 *  [2]     clamp.run_status(uint8, 0=IDLE 1=MOVING 2=ERROR)
 *  [3]     clamp.safe_flag (uint8, 0=UNSAFE 1=SAFE)
 *  [4..7]  clamp.real_angle(float LE)
 *  [8]     chuck.state     (uint8)
 *  [9]     chuck.run_status(uint8)
 *  [10]    chuck.safe_flag (uint8)
 *  [11]    active_source   (uint8, 0=USART 1=USB)
 *  [12..15]chuck.real_angle(float LE)
 *  [16..19]clamp.target_angle(float LE)
 *  [20..23]chuck.target_angle(float LE)
 *  [24]    status_flags    (uint8)
 *  [25]    error_flags     (uint8)
 *  [26]    clamp.pending_state(uint8)
 *  [27]    chuck.pending_state(uint8)
 *  [28]    timeout_flags   (uint8)
 *  [29..31]reserved
 */
#define TOOL_STATUS_LEN  32U

static void SendToolStatus(void)
{
    uint8_t buf[TOOL_STATUS_LEN];
    uint8_t source;
    uint8_t selected_dev;
    uint8_t selected_moving;
    uint8_t timeout_flags;
    uint8_t status_flags;
    uint8_t error_flags;
    clamp_Handle_t clamp_snapshot;
    chuck_Handle_t chuck_snapshot;

    taskENTER_CRITICAL();
    source = Tool_GetActiveSource();
    selected_dev = Tool_GetSelectedDev(source);
    clamp_snapshot = *Tool_GetClamp(source);
    chuck_snapshot = *Tool_GetChuck(source);
    taskEXIT_CRITICAL();

    buf[0] = selected_dev;
    buf[1] = clamp_snapshot.state;
    buf[2] = (uint8_t)clamp_snapshot.run_status;
    buf[3] = clamp_snapshot.safe_flag;
    PackFloatLE(clamp_snapshot.real_angle, buf, 4);

    buf[8]  = chuck_snapshot.state;
    buf[9]  = (uint8_t)chuck_snapshot.run_status;
    buf[10] = chuck_snapshot.safe_flag;
    buf[11] = source;
    PackFloatLE(chuck_snapshot.real_angle, buf, 12);
    PackFloatLE(clamp_snapshot.target_angle, buf, 16);
    PackFloatLE(chuck_snapshot.target_angle, buf, 20);

    timeout_flags = USB_ControlWatchdog_TimeoutFlags();
    selected_moving =
        (((selected_dev == TOOL_DEV_CHUCK) &&
          (chuck_snapshot.run_status == TOOL_STATUS_MOVING)) ||
         ((selected_dev == TOOL_DEV_CLAMP) &&
          (clamp_snapshot.run_status == TOOL_STATUS_MOVING))) ? 1U : 0U;
    status_flags =
        ((Tool_control_flag != 0U) ? 0x01U : 0U) |
        ((clamp_snapshot.run_status == TOOL_STATUS_MOVING) ? 0x02U : 0U) |
        ((chuck_snapshot.run_status == TOOL_STATUS_MOVING) ? 0x04U : 0U) |
        ((selected_moving != 0U) ? 0x08U : 0U) |
        ((clamp_snapshot.safe_flag != 0U) ? 0x10U : 0U) |
        ((chuck_snapshot.safe_flag != 0U) ? 0x20U : 0U) |
        ((source == TOOL_USB_SOURCE) ? 0x40U : 0U);
    error_flags =
        ((clamp_snapshot.run_status == TOOL_STATUS_ERROR) ? 0x01U : 0U) |
        ((chuck_snapshot.run_status == TOOL_STATUS_ERROR) ? 0x02U : 0U) |
        (((timeout_flags & 0x04U) != 0U) ? 0x04U : 0U) |
        (((selected_dev == TOOL_DEV_CHUCK) &&
          (chuck_snapshot.safe_flag == 0U) &&
          (chuck_snapshot.run_status != TOOL_STATUS_MOVING)) ? 0x08U : 0U) |
        (((selected_dev == TOOL_DEV_CLAMP) &&
          (clamp_snapshot.safe_flag == 0U) &&
          (clamp_snapshot.run_status != TOOL_STATUS_MOVING)) ? 0x08U : 0U) |
        ((source != TOOL_USB_SOURCE) ? 0x10U : 0U);
    buf[24] = status_flags;
    buf[25] = error_flags;
    buf[26] = clamp_snapshot.pending_state;
    buf[27] = chuck_snapshot.pending_state;
    buf[28] = timeout_flags;
    buf[29] = 0U;
    buf[30] = 0U;
    buf[31] = 0U;

    Send_Cmd_Data(USB_CMD_TOOL_GET_STATUS, buf, TOOL_STATUS_LEN);
}

#define CLIMB_STATUS_LEN  68U

static uint8_t ClimbValueReached(float pos, float target, float tol)
{
    float err = pos - target;

    if (err < 0.0f) {
        err = -err;
    }

    return (err <= tol) ? 1U : 0U;
}

static uint8_t ClimbLegReachedMask(const R2_Climb_Ctrl_t *climb)
{
    uint8_t i;
    uint8_t mask = 0U;

    if (climb == 0) {
        return 0U;
    }

    for (i = 0U; i < 4U; i++) {
        if (ClimbValueReached(climb->leg_pos_mm[i],
                              climb->leg_target_mm[i],
                              R2_CLIMB_LEG_TOL_MM) != 0U) {
            mask |= (uint8_t)(1U << i);
        }
    }

    return mask;
}

static uint8_t ClimbDriveReachedMask(const R2_Climb_Ctrl_t *climb)
{
    uint8_t i;
    uint8_t mask = 0U;

    if (climb == 0) {
        return 0U;
    }

    for (i = 0U; i < 2U; i++) {
        if (ClimbValueReached(climb->drive_pos_mm[i],
                              climb->drive_target_mm[i],
                              R2_CLIMB_DRIVE_TOL_MM) != 0U) {
            mask |= (uint8_t)(1U << i);
        }
    }

    return mask;
}

static void SendClimbStatus(void)
{
    uint8_t buf[CLIMB_STATUS_LEN];
    uint8_t i;
    uint8_t active_source;
    uint8_t leg_reached_mask;
    uint8_t drive_reached_mask;
    uint8_t status_flags;
    uint8_t leg_busy;
    uint8_t drive_busy;
    uint8_t ready_for_next;
    R2_Climb_Ctrl_t climb;
    uint32_t elapsed_ms = 0U;

    for (i = 0U; i < CLIMB_STATUS_LEN; i++) {
        buf[i] = 0U;
    }

    taskENTER_CRITICAL();
    active_source = (USB_Task_flag != 0U) ? TOOL_USB_SOURCE :
                    ((USART_Task_flag != 0U) ? TOOL_USART_SOURCE : 2U);
    climb = (active_source == TOOL_USB_SOURCE) ?
            g_r2_climb_usb : g_r2_climb_usart;
    taskEXIT_CRITICAL();

    if ((climb.state_start_ms != 0U) &&
        (climb.last_update_ms >= climb.state_start_ms)) {
        elapsed_ms = climb.last_update_ms - climb.state_start_ms;
    }

    leg_reached_mask = ClimbLegReachedMask(&climb);
    drive_reached_mask = ClimbDriveReachedMask(&climb);
    leg_busy = ((leg_reached_mask & 0x0FU) != 0x0FU) ? 1U : 0U;
    drive_busy = ((drive_reached_mask & 0x03U) != 0x03U) ? 1U : 0U;
    ready_for_next =
        ((((climb.state_done != 0U) ||
           (climb.state == R2_CLIMB_STATE_IDLE) ||
           (climb.state == R2_CLIMB_STATE_DONE)) &&
          (climb.pending_step == 0U) &&
          (climb.pending_auto == 0U) &&
          (climb.pending_test_action == 0U) &&
          (climb.test_chassis_active == 0U) &&
          (leg_busy == 0U) &&
          (drive_busy == 0U)) ? 1U : 0U);
    status_flags =
        ((R2_Climb_IsMotorActive(&climb) != 0U) ? 0x01U : 0U) |
        ((leg_busy != 0U) ? 0x02U : 0U) |
        ((drive_busy != 0U) ? 0x04U : 0U) |
        ((climb.test_chassis_active != 0U) ? 0x08U : 0U) |
        ((climb.pending_step != 0U) ? 0x10U : 0U) |
        ((climb.pending_auto != 0U) ? 0x20U : 0U) |
        ((climb.pending_test_action != 0U) ? 0x40U : 0U) |
        ((ready_for_next != 0U) ? 0x80U : 0U);

    buf[0] = (uint8_t)climb.state;
    buf[1] = climb.enabled;
    buf[2] = climb.auto_run;
    buf[3] = climb.state_done;
    buf[4] = climb.error_flags;
    buf[5] = active_source;
    buf[6] = MotorOnline(motor_fdcan2, 6U);
    buf[7] = climb.test_action;

    PackU32LE(elapsed_ms,           buf,  8);
    PackU32LE(climb.last_update_ms, buf, 12);

    PackFloatLE(climb.leg_pos_mm[0], buf, 16);
    PackFloatLE(climb.leg_pos_mm[1], buf, 20);
    PackFloatLE(climb.leg_pos_mm[2], buf, 24);
    PackFloatLE(climb.leg_pos_mm[3], buf, 28);

    PackFloatLE(climb.leg_target_mm[0], buf, 32);
    PackFloatLE(climb.leg_target_mm[1], buf, 36);
    PackFloatLE(climb.leg_target_mm[2], buf, 40);
    PackFloatLE(climb.leg_target_mm[3], buf, 44);

    PackFloatLE(climb.drive_pos_mm[0], buf, 48);
    PackFloatLE(climb.drive_pos_mm[1], buf, 52);
    PackFloatLE(climb.drive_target_mm[0], buf, 56);
    PackFloatLE(climb.drive_target_mm[1], buf, 60);
    buf[64] = climb.flow;
    buf[65] = status_flags;
    buf[66] = leg_reached_mask;
    buf[67] = drive_reached_mask;

    Send_Cmd_Data(USB_CMD_CLIMB_GET_STATUS, buf, CLIMB_STATUS_LEN);
}


/* ═══════════════════════════════════════════════════════
 *  PC_TX 主任务 — 5ms 轮询，消费状态请求标志
 * ═══════════════════════════════════════════════════════ */

#define YAW_TUNE_STATUS_LEN  64U

static void SendYawTuneStatus(void)
{
    uint8_t buf[YAW_TUNE_STATUS_LEN];
    R2_YawAutoTuneStatus_t s;
    uint8_t i;

    for (i = 0U; i < YAW_TUNE_STATUS_LEN; i++) {
        buf[i] = 0U;
    }

    R2_YawAutoTune_GetStatus(&s);

    buf[0] = s.state;
    buf[1] = s.segment_index;
    buf[2] = s.segment_count;
    buf[3] = s.fail_reason;

    PackU32LE(s.tick_ms, buf, 4);
    PackU32LE(s.segment_elapsed_ms, buf, 8);
    PackFloatLE(s.yaw_error_deg, buf, 12);
    PackFloatLE(s.yaw_error_abs_max_deg, buf, 16);
    PackFloatLE(s.yaw_rate_error_rms_dps, buf, 20);
    PackFloatLE(s.gyro_z_abs_max_dps, buf, 24);
    PackFloatLE(s.score, buf, 28);
    PackFloatLE(s.angle_kp, buf, 32);
    PackFloatLE(s.angle_kd, buf, 36);
    PackFloatLE(s.rate_kp, buf, 40);
    PackFloatLE(s.rate_ki, buf, 44);
    PackFloatLE(s.rate_kd, buf, 48);
    PackFloatLE(s.pos_kp_yaw, buf, 52);
    PackFloatLE(s.last_adjust, buf, 56);
    buf[60] = s.pass_index;
    buf[61] = s.pass_count;
    buf[62] = s.active_mode;
    buf[63] = s.phase;

    Send_Cmd_Data(USB_CMD_YAW_TUNE_GET_STATUS, buf, YAW_TUNE_STATUS_LEN);
}

/* Protocol v4 declares the unified X-forward/Y-left coordinate semantics. */
#define ROBOT_STATUS_LEN        240U
#define ROBOT_CMD_RECENT_MS     500U
#define ROBOT_ARM_MOVE_TOL_DEG  2.0f
#define ROBOT_VEL_MOVE_TOL      0.01f
#define ROBOT_WZ_MOVE_TOL       0.01f

void RobotStatusView_Update(void)
{
    uint8_t i;
    uint8_t active_source;
    uint8_t tool_source;
    uint8_t selected_dev;
    uint8_t timeout_flags;
    uint8_t usb_recent;
    uint8_t usart_recent;
    uint8_t active_source_stale;
    uint8_t chassis_moving;
    uint8_t chassis_pos_running;
    uint8_t arm_moving;
    uint8_t tool_moving;
    uint8_t tool_error;
    uint8_t climb_motor_active;
    uint8_t chassis_motor_online;
    uint8_t arm_motor_online;
    uint8_t arm_enabled;
    uint8_t climb_motor_online;
    uint8_t yaw_tune_running;
    uint8_t enable_flags = 0U;
    uint8_t executing_flags = 0U;
    uint8_t error_flags = 0U;
    uint8_t online_flags = 0U;
    uint32_t now_tick;
    uint32_t climb_elapsed_ms = 0U;
    USB_CommandRxState_t usb_rx;
    CRC_Debug_t usart_rx;
    R2_Move_Ctrl_t chs;
    R2_Climb_Ctrl_t climb;
    R2_LaserMeasure_t laser;
    R2_YawAutoTuneStatus_t yaw_tune;
    INS_NavState_t nav;
    const ArmIK_AppState_t *app;
    clamp_Handle_t clamp_snapshot;
    chuck_Handle_t chuck_snapshot;
    float actual_j[ROBOT_STATUS_VIEW_ARM_JOINTS];
    float arm_err[ROBOT_STATUS_VIEW_ARM_JOINTS] = {0.0f, 0.0f, 0.0f};
    volatile RobotStatusView_t *view = &g_robot_status_view;

    now_tick = HAL_GetTick();
    USB_GetCommandRxState(&usb_rx);
    INS_GetState(&nav);
    R2_LaserUser_GetMeasure(&laser);
    R2_YawAutoTune_GetStatus(&yaw_tune);
    app = ArmIK_GetAppState();

    taskENTER_CRITICAL();
    active_source = (USB_Task_flag != 0U) ? TOOL_USB_SOURCE :
                    ((USART_Task_flag != 0U) ? TOOL_USART_SOURCE : ROBOT_STATUS_VIEW_SOURCE_NONE);
    tool_source = (active_source == TOOL_USB_SOURCE) ? TOOL_USB_SOURCE : TOOL_USART_SOURCE;
    chs = (active_source == TOOL_USB_SOURCE) ? g_r2_ctrl_usb : g_r2_ctrl_usart;
    climb = (active_source == TOOL_USB_SOURCE) ? g_r2_climb_usb : g_r2_climb_usart;
    usart_rx = crc_dbg;
    selected_dev = Tool_GetSelectedDev(tool_source);
    clamp_snapshot = *Tool_GetClamp(tool_source);
    chuck_snapshot = *Tool_GetChuck(tool_source);
    taskEXIT_CRITICAL();

    usb_recent = ((usb_rx.last_tick != 0U) &&
                  ((now_tick - usb_rx.last_tick) <= ROBOT_CMD_RECENT_MS)) ? 1U : 0U;
    usart_recent = ((usart_rx.last_frame_tick != 0U) &&
                    ((now_tick - usart_rx.last_frame_tick) <= ROBOT_CMD_RECENT_MS)) ? 1U : 0U;

    active_source_stale = 0U;
    if ((active_source == TOOL_USB_SOURCE) && (usb_recent == 0U)) {
        active_source_stale = 1U;
    } else if ((active_source == TOOL_USART_SOURCE) && (usart_recent == 0U)) {
        active_source_stale = 1U;
    } else if (active_source == ROBOT_STATUS_VIEW_SOURCE_NONE) {
        active_source_stale = 1U;
    }

    if (active_source == TOOL_USB_SOURCE) {
        arm_enabled = (Arm_control_flag != 0U) ? 1U : 0U;
    } else if (active_source == TOOL_USART_SOURCE) {
        arm_enabled = (usart_rx.arm_flag == 1U) ? 1U : 0U;
    } else {
        arm_enabled = 0U;
    }

    chassis_pos_running = (chs.pos_state == R2_POS_RUNNING) ? 1U : 0U;
    chassis_moving = ((chassis_pos_running != 0U) ||
                      (AbsF(chs.robot_vel.vx) > ROBOT_VEL_MOVE_TOL) ||
                      (AbsF(chs.robot_vel.vy) > ROBOT_VEL_MOVE_TOL) ||
                      (AbsF(chs.robot_vel.vw) > ROBOT_WZ_MOVE_TOL)) ? 1U : 0U;

    actual_j[0] = -(float)motor_fdcan3[0].total_angle * ARM_TNUM1;
    actual_j[1] =  (float)motor_fdcan3[1].total_angle * ARM_TNUM23;
    actual_j[2] =  (float)motor_fdcan3[2].total_angle * ARM_TNUM23;

    if ((app != NULL) && (app->has_last_valid != 0U)) {
        arm_err[0] = AbsF(actual_j[0] - app->active_motor_deg.j1_deg);
        arm_err[1] = AbsF(actual_j[1] - app->active_motor_deg.j2_deg);
        arm_err[2] = AbsF(actual_j[2] - app->active_motor_deg.j3_deg);
    }
    arm_moving = (((arm_err[0] > ROBOT_ARM_MOVE_TOL_DEG) ||
                   (arm_err[1] > ROBOT_ARM_MOVE_TOL_DEG) ||
                   (arm_err[2] > ROBOT_ARM_MOVE_TOL_DEG)) &&
                  (arm_enabled != 0U)) ? 1U : 0U;

    tool_moving = ((clamp_snapshot.run_status == TOOL_STATUS_MOVING) ||
                   (chuck_snapshot.run_status == TOOL_STATUS_MOVING)) ? 1U : 0U;
    tool_error = ((clamp_snapshot.run_status == TOOL_STATUS_ERROR) ||
                  (chuck_snapshot.run_status == TOOL_STATUS_ERROR)) ? 1U : 0U;

    climb_motor_active = R2_Climb_IsMotorActive(&climb);
    if ((climb.state_start_ms != 0U) &&
        (climb.last_update_ms >= climb.state_start_ms)) {
        climb_elapsed_ms = climb.last_update_ms - climb.state_start_ms;
    }

    chassis_motor_online = MotorOnline(motor_fdcan1, CHASSIS_MOTOR_COUNT);
    arm_motor_online = MotorOnline(motor_fdcan3, 3U);
    climb_motor_online = MotorOnline(motor_fdcan2, 6U);
    yaw_tune_running = (yaw_tune.state == (uint8_t)R2_YAW_AUTOTUNE_RUNNING) ? 1U : 0U;
    ArmIK_SetActualMotorDeg(actual_j[0],
                            actual_j[1],
                            actual_j[2],
                            (arm_motor_online >= 3U) ? 1U : 0U);

    if (Mecanum_control_flag != 0U) enable_flags |= 0x01U;
    if (arm_enabled != 0U)          enable_flags |= 0x02U;
    if (Tool_control_flag != 0U)    enable_flags |= 0x04U;
    if (climb.enabled != 0U)        enable_flags |= 0x08U;

    if (chassis_moving != 0U)       executing_flags |= 0x01U;
    if (chassis_pos_running != 0U)  executing_flags |= 0x02U;
    if (arm_moving != 0U)           executing_flags |= 0x04U;
    if (tool_moving != 0U)          executing_flags |= 0x08U;
    if (climb_motor_active != 0U)   executing_flags |= 0x10U;
    if (yaw_tune_running != 0U)     executing_flags |= 0x20U;
    if (executing_flags != 0U)      executing_flags |= 0x80U;

    timeout_flags = USB_ControlWatchdog_TimeoutFlags();
    if (usart_rx.control_timeout != 0U) timeout_flags |= 0x08U;
    if ((timeout_flags & 0x01U) != 0U) error_flags |= 0x01U;
    if ((timeout_flags & 0x02U) != 0U) error_flags |= 0x02U;
    if ((timeout_flags & 0x04U) != 0U) error_flags |= 0x04U;
    if (nav.imu_online == 0U)          error_flags |= 0x08U;
    if (chs.emergency_stop != 0U)      error_flags |= 0x10U;
    if ((app == NULL) || (app->last_status_code != ARM_IK_RESULT_OK)) error_flags |= 0x20U;
    if (tool_error != 0U)              error_flags |= 0x40U;
    if (active_source_stale != 0U)     error_flags |= 0x80U;

    if (usb_recent != 0U)             online_flags |= 0x01U;
    if (usart_recent != 0U)           online_flags |= 0x02U;
    if (nav.imu_online != 0U)         online_flags |= 0x04U;
    if (chassis_motor_online >= CHASSIS_MOTOR_COUNT) online_flags |= 0x08U;
    if (arm_motor_online >= 3U)       online_flags |= 0x10U;
    if (climb_motor_online >= 6U)     online_flags |= 0x20U;
    if (laser.all_online != 0U)       online_flags |= 0x40U;

    view->summary.tick_ms = now_tick;
    view->summary.active_source = active_source;
    view->summary.enable_flags = enable_flags;
    view->summary.executing_flags = executing_flags;
    view->summary.error_flags = error_flags;
    view->summary.online_flags = online_flags;
    view->summary.timeout_flags = timeout_flags;
    view->summary.active_source_stale = active_source_stale;

    view->rx.usb_count = usb_rx.count;
    view->rx.usb_last_tick = usb_rx.last_tick;
    view->rx.usb_last_cmd = usb_rx.last_cmd;
    view->rx.usb_last_len = usb_rx.last_len;
    view->rx.usb_payload_valid = usb_rx.last_payload_valid;
    view->rx.usb_recent = usb_recent;
    for (i = 0U; i < ROBOT_STATUS_VIEW_USB_DATA_LEN; i++) {
        view->rx.usb_last_data[i] = usb_rx.last_data[i];
    }
    for (i = 0U; i < ROBOT_STATUS_VIEW_FLOAT_COUNT; i++) {
        view->rx.usb_last_f[i] = usb_rx.last_f[i];
    }

    view->rx.usart_frame_count = usart_rx.frame_count;
    view->rx.usart_last_tick = usart_rx.last_frame_tick;
    view->rx.usart_checksum_fail_count = usart_rx.checksum_fail_count;
    view->rx.usart_checksum_ok = usart_rx.checksum_ok;
    view->rx.usart_control_timeout = usart_rx.control_timeout;
    view->rx.usart_recent = usart_recent;
    view->rx.usart_mode = usart_rx.mode;
    view->rx.usart_chassis_param[0] = usart_rx.chs_p1;
    view->rx.usart_chassis_param[1] = usart_rx.chs_p2;
    view->rx.usart_chassis_param[2] = usart_rx.chs_p3;
    view->rx.usart_arm_xyz_mm[0] = usart_rx.arm_x;
    view->rx.usart_arm_xyz_mm[1] = usart_rx.arm_y;
    view->rx.usart_arm_xyz_mm[2] = usart_rx.arm_z;
    view->rx.usart_arm_flag = usart_rx.arm_flag;
    view->rx.usart_source_flag = usart_rx.uu_flag;
    view->rx.usart_tool_flag = usart_rx.tool_flag;
    view->rx.usart_clampuse_flag = usart_rx.clampuse_flag;
    view->rx.usart_chuckuse_flag = usart_rx.chuckuse_flag;
    view->rx.usart_climb_enable = usart_rx.climb_enable;
    view->rx.usart_climb_step = usart_rx.climb_step;
    view->rx.usart_climb_auto = usart_rx.climb_auto;

    view->chassis.enabled = Mecanum_control_flag;
    view->chassis.mode = (uint8_t)chs.mode;
    view->chassis.pos_state = (uint8_t)chs.pos_state;
    view->chassis.emergency_stop = chs.emergency_stop;
    view->chassis.moving = chassis_moving;
    view->chassis.motor_online = chassis_motor_online;
    view->chassis.target_vx = chs.target_vx;
    view->chassis.target_vy = chs.target_vy;
    view->chassis.target_vw = chs.target_vw;
    view->chassis.target_dx = chs.target_dx;
    view->chassis.target_dy = chs.target_dy;
    view->chassis.target_dyaw = chs.target_dyaw;
    view->chassis.vel_vx = chs.robot_vel.vx;
    view->chassis.vel_vy = chs.robot_vel.vy;
    view->chassis.vel_vw = chs.robot_vel.vw;
    view->chassis.odom_x = chs.odom_x;
    view->chassis.odom_y = chs.odom_y;
    view->chassis.odom_yaw = chs.odom_yaw;
    view->chassis.nav_x = nav.x_m;
    view->chassis.nav_y = nav.y_m;
    view->chassis.nav_yaw = nav.yaw_total_rad;
    view->chassis.nav_vx = nav.vx_mps;
    view->chassis.nav_vy = nav.vy_mps;
    view->chassis.nav_wz = nav.wz_radps;
    view->chassis.wheel_speed[0] = chs.wheel_speed.fl;
    view->chassis.wheel_speed[1] = chs.wheel_speed.fr;
    view->chassis.wheel_speed[2] = chs.wheel_speed.bl;
    view->chassis.wheel_speed[3] = chs.wheel_speed.br;
    view->chassis.pos_progress = chs.pos_progress;
    view->chassis.pos_err_x = chs.pos_err_x;
    view->chassis.pos_err_y = chs.pos_err_y;
    view->chassis.pos_err_yaw = chs.pos_err_yaw;

    view->arm.enabled = arm_enabled;
    view->arm.has_last_valid = (app != NULL) ? app->has_last_valid : 0U;
    view->arm.last_status_code = (app != NULL) ? app->last_status_code : ARM_IK_RESULT_PARAM_ERR;
    view->arm.last_action_code = (app != NULL) ? app->last_action_code : ARM_IK_ACTION_KEEP_CURRENT;
    view->arm.moving = arm_moving;
    view->arm.motor_online = arm_motor_online;
    if ((active_source == TOOL_USART_SOURCE) || (active_source == ROBOT_STATUS_VIEW_SOURCE_NONE)) {
        view->arm.target_xyz_mm[0] = usart_rx.arm_x;
        view->arm.target_xyz_mm[1] = usart_rx.arm_y;
        view->arm.target_xyz_mm[2] = usart_rx.arm_z;
    } else if ((usb_rx.last_cmd == USB_CMD_ARM_SET_TARGET) &&
               (usb_rx.last_payload_valid != 0U)) {
        view->arm.target_xyz_mm[0] = usb_rx.last_f[0];
        view->arm.target_xyz_mm[1] = usb_rx.last_f[1];
        view->arm.target_xyz_mm[2] = usb_rx.last_f[2];
    }
    if (app != NULL) {
        view->arm.model_rad[0] = app->active_model.theta1;
        view->arm.model_rad[1] = app->active_model.theta2;
        view->arm.model_rad[2] = app->active_model.theta3;
        view->arm.target_deg[0] = app->active_motor_deg.j1_deg;
        view->arm.target_deg[1] = app->active_motor_deg.j2_deg;
        view->arm.target_deg[2] = app->active_motor_deg.j3_deg;
    }
    for (i = 0U; i < ROBOT_STATUS_VIEW_ARM_JOINTS; i++) {
        view->arm.actual_deg[i] = actual_j[i];
        view->arm.err_deg[i] = arm_err[i];
    }

    view->tool.enabled = Tool_control_flag;
    view->tool.active_source = active_source;
    view->tool.selected_dev = selected_dev;
    view->tool.moving = tool_moving;
    view->tool.error = tool_error;
    view->tool.clamp_state = clamp_snapshot.state;
    view->tool.clamp_run_status = (uint8_t)clamp_snapshot.run_status;
    view->tool.clamp_safe_flag = clamp_snapshot.safe_flag;
    view->tool.clamp_real_angle = clamp_snapshot.real_angle;
    view->tool.clamp_target_angle = clamp_snapshot.target_angle;
    view->tool.chuck_state = chuck_snapshot.state;
    view->tool.chuck_run_status = (uint8_t)chuck_snapshot.run_status;
    view->tool.chuck_safe_flag = chuck_snapshot.safe_flag;
    view->tool.chuck_real_angle = chuck_snapshot.real_angle;
    view->tool.chuck_target_angle = chuck_snapshot.target_angle;

    view->climb.source = active_source;
    view->climb.enabled = climb.enabled;
    view->climb.state = (uint8_t)climb.state;
    view->climb.auto_run = climb.auto_run;
    view->climb.state_done = climb.state_done;
    view->climb.error_flags = climb.error_flags;
    view->climb.flow = climb.flow;
    view->climb.pending_step = climb.pending_step;
    view->climb.pending_auto = climb.pending_auto;
    view->climb.pending_test_action = climb.pending_test_action;
    view->climb.test_action = climb.test_action;
    view->climb.test_active = climb.test_active;
    view->climb.test_chassis_active = climb.test_chassis_active;
    view->climb.motor_active = climb_motor_active;
    view->climb.motor_online = climb_motor_online;
    view->climb.elapsed_ms = climb_elapsed_ms;
    view->climb.last_update_ms = climb.last_update_ms;
    for (i = 0U; i < ROBOT_STATUS_VIEW_CLIMB_LEGS; i++) {
        view->climb.leg_pos_mm[i] = climb.leg_pos_mm[i];
        view->climb.leg_target_mm[i] = climb.leg_target_mm[i];
    }
    for (i = 0U; i < ROBOT_STATUS_VIEW_CLIMB_DRIVES; i++) {
        view->climb.drive_pos_mm[i] = climb.drive_pos_mm[i];
        view->climb.drive_target_mm[i] = climb.drive_target_mm[i];
    }

    view->laser.valid_flags =
        ((laser.x_pos_valid != 0U) ? 0x01U : 0U) |
        ((laser.y_pos_valid != 0U) ? 0x02U : 0U) |
        ((laser.height_valid != 0U) ? 0x04U : 0U);
    view->laser.online_flags =
        ((laser.x_pos_online != 0U) ? 0x01U : 0U) |
        ((laser.y_pos_online != 0U) ? 0x02U : 0U) |
        ((laser.height_online != 0U) ? 0x04U : 0U);
    view->laser.waiting_flags =
        ((laser.channel[0].waiting_response != 0U) ? 0x01U : 0U) |
        ((laser.channel[1].waiting_response != 0U) ? 0x02U : 0U) |
        ((laser.channel[2].waiting_response != 0U) ? 0x04U : 0U);
    view->laser.all_valid = laser.all_valid;
    view->laser.all_online = laser.all_online;
    view->laser.reserved[0] = 0U;
    view->laser.reserved[1] = 0U;
    view->laser.reserved[2] = 0U;
    view->laser.update_tick = laser.update_tick;
    view->laser.distance_mm[0] = laser.x_pos_mm;
    view->laser.distance_mm[1] = laser.y_pos_mm;
    view->laser.distance_mm[2] = laser.height_mm;
    view->laser.raw_distance_mm[0] = laser.x_pos_raw_mm;
    view->laser.raw_distance_mm[1] = laser.y_pos_raw_mm;
    view->laser.raw_distance_mm[2] = laser.height_raw_mm;
    for (i = 0U; i < ROBOT_STATUS_VIEW_LASER_COUNT; i++) {
        view->laser.offset_mm[i] = laser.channel[i].offset_mm;
        view->laser.last_update_tick[i] = laser.channel[i].last_update_tick;
        view->laser.timeout_count[i] = laser.channel[i].timeout_count;
        view->laser.crc_error_count[i] = laser.channel[i].crc_error_count;
        view->laser.parse_error_count[i] = laser.channel[i].parse_error_count;
        view->laser.uart_error_count[i] = laser.channel[i].uart_error_count;
    }

    view->yaw_tune.state = yaw_tune.state;
    view->yaw_tune.segment_index = yaw_tune.segment_index;
    view->yaw_tune.segment_count = yaw_tune.segment_count;
    view->yaw_tune.pass_index = yaw_tune.pass_index;
    view->yaw_tune.pass_count = yaw_tune.pass_count;
    view->yaw_tune.fail_reason = yaw_tune.fail_reason;
    view->yaw_tune.active_mode = yaw_tune.active_mode;
    view->yaw_tune.phase = yaw_tune.phase;
    view->yaw_tune.tick_ms = yaw_tune.tick_ms;
    view->yaw_tune.segment_elapsed_ms = yaw_tune.segment_elapsed_ms;
    view->yaw_tune.yaw_error_deg = yaw_tune.yaw_error_deg;
    view->yaw_tune.yaw_error_abs_max_deg = yaw_tune.yaw_error_abs_max_deg;
    view->yaw_tune.yaw_rate_error_rms_dps = yaw_tune.yaw_rate_error_rms_dps;
    view->yaw_tune.gyro_z_abs_max_dps = yaw_tune.gyro_z_abs_max_dps;
    view->yaw_tune.wheel_rpm_abs_max = yaw_tune.wheel_rpm_abs_max;
    view->yaw_tune.score = yaw_tune.score;
    view->yaw_tune.last_adjust = yaw_tune.last_adjust;
    view->yaw_tune.angle_kp = yaw_tune.angle_kp;
    view->yaw_tune.angle_ki = yaw_tune.angle_ki;
    view->yaw_tune.angle_kd = yaw_tune.angle_kd;
    view->yaw_tune.rate_kp = yaw_tune.rate_kp;
    view->yaw_tune.rate_ki = yaw_tune.rate_ki;
    view->yaw_tune.rate_kd = yaw_tune.rate_kd;
    view->yaw_tune.pos_kp_yaw = yaw_tune.pos_kp_yaw;
}

static void SendRobotStatus(void)
{
    uint8_t buf[ROBOT_STATUS_LEN];
    uint8_t i;
    const volatile RobotStatusView_t *view = &g_robot_status_view;

    for (i = 0U; i < ROBOT_STATUS_LEN; i++) {
        buf[i] = 0U;
    }

    RobotStatusView_Update();

    buf[0] = ROBOT_STATUS_PROTOCOL_VERSION;
    buf[1] = view->summary.active_source;
    buf[2] = view->summary.enable_flags;
    buf[3] = view->summary.executing_flags;
    buf[4] = view->summary.error_flags;
    buf[5] = view->summary.online_flags;
    buf[6] = view->rx.usb_last_cmd;
    buf[7] = view->rx.usb_last_len;
    PackU32LE(view->rx.usb_count,                 buf,  8);
    PackU32LE(view->rx.usb_last_tick,             buf, 12);
    PackU32LE(view->rx.usart_frame_count,         buf, 16);
    PackU32LE(view->rx.usart_last_tick,           buf, 20);
    PackU32LE(view->rx.usart_checksum_fail_count, buf, 24);
    buf[28] = view->rx.usart_checksum_ok;
    buf[29] = view->rx.usart_mode;
    buf[30] = view->arm.last_status_code;
    buf[31] = view->arm.last_action_code;
    PackFloatLE(view->chassis.nav_x,     buf, 32);
    PackFloatLE(view->chassis.nav_y,     buf, 36);
    PackFloatLE(view->chassis.nav_yaw,   buf, 40);
    PackFloatLE(view->chassis.nav_vx,    buf, 44);
    PackFloatLE(view->chassis.nav_vy,    buf, 48);
    PackFloatLE(view->chassis.nav_wz,    buf, 52);
    PackFloatLE(view->chassis.odom_x,    buf, 56);
    PackFloatLE(view->chassis.odom_y,    buf, 60);
    PackFloatLE(view->chassis.odom_yaw,  buf, 64);
    PackFloatLE(view->chassis.vel_vx,    buf, 68);
    PackFloatLE(view->chassis.vel_vy,    buf, 72);
    PackFloatLE(view->chassis.vel_vw,    buf, 76);
    PackFloatLE(view->arm.err_deg[0],    buf, 80);
    PackFloatLE(view->arm.err_deg[1],    buf, 84);
    PackFloatLE(view->arm.err_deg[2],    buf, 88);
    buf[92] = view->tool.selected_dev;
    buf[93] = (view->tool.selected_dev == TOOL_DEV_CHUCK) ? view->tool.chuck_run_status
                                                          : view->tool.clamp_run_status;
    buf[94] = view->chassis.pos_state;
    buf[95] = view->summary.timeout_flags;

    buf[96] = view->climb.source;
    buf[97] = view->climb.state;
    buf[98] = view->climb.enabled;
    buf[99] = view->climb.auto_run;
    buf[100] = view->climb.state_done;
    buf[101] = view->climb.error_flags;
    buf[102] = view->climb.pending_step;
    buf[103] = view->climb.pending_auto;
    buf[104] = view->climb.motor_active;
    buf[105] = view->climb.motor_online;
    buf[106] = view->climb.test_action;
    buf[107] = ((view->climb.test_active != 0U) ? 0x01U : 0U) |
               ((view->climb.test_chassis_active != 0U) ? 0x02U : 0U) |
               ((view->climb.flow == (uint8_t)R2_CLIMB_FLOW_DOWNSTAIRS) ? 0x04U : 0U);
    PackU32LE(view->climb.elapsed_ms,     buf, 108);
    PackU32LE(view->climb.last_update_ms, buf, 112);

    buf[116] = view->laser.valid_flags;
    buf[117] = view->laser.online_flags;
    buf[118] = view->laser.waiting_flags;
    buf[119] = view->laser.all_valid;
    buf[120] = view->laser.all_online;
    PackU32LE(view->laser.update_tick,      buf, 124);
    PackI32LE(view->laser.distance_mm[0],   buf, 128);
    PackI32LE(view->laser.distance_mm[1],   buf, 132);
    PackI32LE(view->laser.distance_mm[2],   buf, 136);

    buf[140] = view->yaw_tune.state;
    buf[141] = view->yaw_tune.fail_reason;
    buf[142] = view->yaw_tune.phase;
    buf[143] = view->yaw_tune.active_mode;
    PackU32LE(view->yaw_tune.tick_ms,             buf, 144);
    PackU32LE(view->yaw_tune.segment_elapsed_ms,  buf, 148);
    PackFloatLE(view->yaw_tune.yaw_error_deg,     buf, 152);
    PackFloatLE(view->yaw_tune.score,             buf, 156);

    PackFloatLE(view->chassis.target_vx,           buf, 160);
    PackFloatLE(view->chassis.target_vy,           buf, 164);
    PackFloatLE(view->chassis.target_vw,           buf, 168);
    PackFloatLE(view->chassis.target_dx,           buf, 172);
    PackFloatLE(view->chassis.target_dy,           buf, 176);
    PackFloatLE(view->chassis.target_dyaw,         buf, 180);
    PackFloatLE(view->arm.target_xyz_mm[0],        buf, 184);
    PackFloatLE(view->arm.target_xyz_mm[1],        buf, 188);
    PackFloatLE(view->arm.target_xyz_mm[2],        buf, 192);
    PackFloatLE(view->arm.target_deg[0],           buf, 196);
    PackFloatLE(view->arm.target_deg[1],           buf, 200);
    PackFloatLE(view->arm.target_deg[2],           buf, 204);
    PackFloatLE(view->arm.actual_deg[0],           buf, 208);
    PackFloatLE(view->arm.actual_deg[1],           buf, 212);
    PackFloatLE(view->arm.actual_deg[2],           buf, 216);
    PackFloatLE(view->tool.clamp_target_angle,     buf, 220);
    PackFloatLE(view->tool.clamp_real_angle,       buf, 224);
    PackFloatLE(view->tool.chuck_target_angle,     buf, 228);
    PackFloatLE(view->tool.chuck_real_angle,       buf, 232);
    buf[236] = view->arm.motor_online;
    buf[237] = view->tool.error;
    buf[238] = view->climb.error_flags;
    buf[239] = view->summary.active_source_stale;

    Send_Cmd_Data(USB_CMD_ROBOT_GET_STATUS, buf, ROBOT_STATUS_LEN);
}

void PC_TX_Task(void const * argument)
{
    uint8_t status_cmd;
    uint32_t last_laser_poll_tick;

    (void)argument;
    R2_LaserUser_Init();
    last_laser_poll_tick = osKernelSysTick();

    for (;;)
    {
        uint32_t now_tick = osKernelSysTick();

        if ((now_tick - last_laser_poll_tick) >= R2_LASER_POLL_PERIOD_MS) {
            last_laser_poll_tick = now_tick;
            R2_LaserUser_Poll10ms();
        }

        RobotStatusView_Update();

        /* 每次只处理一个请求，避免 USB 突发拥堵 */
        if (PC_TX_DequeueStatus(&status_cmd) != 0U) {
            switch (status_cmd) {
            case USB_CMD_SYS_GET_STATUS:
                SendSysStatus();
                break;
            case USB_CMD_CHS_GET_STATUS:
                SendChsStatus();
                break;
            case USB_CMD_ARM_GET_STATUS:
                SendArmStatus();
                break;
            case USB_CMD_TOOL_GET_STATUS:
                SendToolStatus();
                break;
            case USB_CMD_ROBOT_GET_STATUS:
                SendRobotStatus();
                break;
            case USB_CMD_CLIMB_GET_STATUS:
                SendClimbStatus();
                break;
            case USB_CMD_YAW_TUNE_GET_STATUS:
                SendYawTuneStatus();
                break;
            default:
                break;
            }
        }

        osDelay(5);
    }
}
