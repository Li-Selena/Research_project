#include "chassis_move.h"
#include "fdcan_receive.h"
#include <string.h>
#include "ins_nav_math.h"


/*
 * ─── 轮子方向符号表 ─────────────────────────────────────
 * motor_fdcan1[0..3] = Motor1(FR), Motor2(BR), Motor3(BL), Motor4(FL).
 * Physical IDs start at the front-right corner and go clockwise.
 *
 * 因为 CAN_Task 中对 fr / br 取反（-total_speed.fr），
 * 所以 odometry 读回时也要对这两个通道取反，
 * 才能保持 “正向 = 轮子前进” 的统一约定。
 */
static const int8_t motor_to_wheel_sign[CHASSIS_MOTOR_COUNT] = { -1, -1, 1, 1 };

/* 电机测量值 extern 声明 */
extern motor_measure_t motor_fdcan1[8];

/* ─── 公开 API ───────────────────────────────────────── */

/* 编码器增量 → 轮子线位移 (m) */
float EncoderDeltaToWheelMeter(int32_t delta_enc)
{
    return (float)delta_enc * ENCODER_TO_WHEEL_M;
}

/* 电机 RPM → 轮子线速度 (m/s)，自动校正轮子方向符号 */
float MotorRPMToWheelMPS(float motor_rpm, uint8_t motor_idx)
{
    return motor_rpm * MOTOR_RPM_TO_WHEEL_MPS
           * (float)(int8_t)motor_to_wheel_sign[motor_idx % CHASSIS_MOTOR_COUNT];
}

/*
 * 麦克纳姆正向运动学：四轮线速度 → 底盘速度 (vx, vy, vw)
 * 公式（经典麦克纳姆解算矩阵的逆）：
 *   vx = (w_fl + w_fr + w_bl + w_br) / 4
 *   vy = (-w_fl + w_fr + w_bl - w_br) / 4
 *   vw = (-w_fl + w_fr - w_bl + w_br) / (4 * (L + W))
 */
void ChassisForwardKinematics(const WheelSpeed_t *w,
                              const MecanumParam_t *p,
                              ChassisVel_t *out)
{
    float sum_xy, sum_k;

    if ((w == NULL) || (p == NULL) || (out == NULL)) return;

    sum_xy = 0.25f;
    sum_k  = 0.25f / (p->L + p->W);

    out->vx = ( w->fl + w->fr + w->bl + w->br) * sum_xy;
    out->vy = (-w->fl + w->fr + w->bl - w->br) * sum_xy;
    out->vw = (-w->fl + w->fr - w->bl + w->br) * sum_k;
}

/* ─── 里程计更新 ────────────────────────────────────── */

/*
 * 由电机编码器累计值推算底盘当前位姿。
 * 与启动时的快照 start_enc / start_odom_* 比较，得到相对位移增量，
 * 再叠加到起点坐标上，得到世界坐标系下的当前位置。
 */
void ChassisOdometry_Update(ChassisMove_Ctrl_t *ctrl,
                            const MecanumParam_t *param)
{
    float wheel_delta[CHASSIS_MOTOR_COUNT];
    WheelSpeed_t wheel_deltas;
    ChassisVel_t delta;
    int32_t cur_enc;
    uint8_t i;

    if (ctrl == NULL || param == NULL) return;

    /* 读取 4 个底盘电机的累积角度，计算本轮增量 */
    for (i = 0U; i < CHASSIS_MOTOR_COUNT; i++) {
        cur_enc = motor_fdcan1[i].total_angle;
        /* 编码器增量 × 符号 × 转换系数 → 轮子线位移 */
        wheel_delta[i] = EncoderDeltaToWheelMeter(
            (int32_t)motor_to_wheel_sign[i] * (cur_enc - ctrl->start_enc[i]));
    }

    wheel_deltas.fr = wheel_delta[CHASSIS_MOTOR_FR];
    wheel_deltas.br = wheel_delta[CHASSIS_MOTOR_BR];
    wheel_deltas.bl = wheel_delta[CHASSIS_MOTOR_BL];
    wheel_deltas.fl = wheel_delta[CHASSIS_MOTOR_FL];

    /* 正向运动学：轮子位移增量 → 底盘位移增量 */
    ChassisForwardKinematics(&wheel_deltas, param, &delta);

    /* This optional controller assumes straight travel at the start heading.
     * Rotate start-body displacement to world axes before adding the origin.
     * The main R2 controller integrates per-tick deltas for changing headings.
     */
    float world_dx, world_dy;
    INS_NavMath_RobotToWorld(delta.vx, delta.vy, ctrl->start_odom_yaw,
                             &world_dx, &world_dy);
    ctrl->odom_x   = ctrl->start_odom_x   + world_dx;
    ctrl->odom_y   = ctrl->start_odom_y   + world_dy;
    ctrl->odom_yaw = ctrl->start_odom_yaw + delta.vw;
}

/* ─── 控制器生命周期 ────────────────────────────────── */

/*
 * 上电初始化，清零所有状态并设置默认运动参数。
 * 默认值：v_max=1m/s, a_max=2m/s², j_max=10m/s³, pos_kp=3.0
 */
void ChassisMove_Init(ChassisMove_Ctrl_t *ctrl)
{
    if (ctrl == NULL) return;
    memset(ctrl, 0, sizeof(*ctrl));
    ctrl->state   = CHASSIS_MOVE_IDLE;
    ctrl->pos_kp  = 3.0f;
    ctrl->v_max   = 1.0f;
    ctrl->a_max   = 2.0f;
    ctrl->j_max   = 10.0f;
}

/* 运行时动态修改运动参数上限 */
void ChassisMove_SetLimits(ChassisMove_Ctrl_t *ctrl,
                           float v_max, float a_max, float j_max)
{
    if (ctrl == NULL) return;
    ctrl->v_max = v_max;
    ctrl->a_max = a_max;
    ctrl->j_max = j_max;
}

/*
 * 启动一次固定距离移动。
 * 对三个轴 (dx, dy, dyaw) 分别做 S 曲线规划，取最长时间轴为统一时长，
 * 覆写较短轴的 duration 以保证三轴同步到达终点。
 */
void ChassisMove_Start(ChassisMove_Ctrl_t *ctrl,
                       float dx, float dy, float dyaw)
{
    float max_dur;
    uint8_t i;

    if (ctrl == NULL) return;

    /* ── 快照当前里程计位置作为运动起点 ── */
    ctrl->start_odom_x   = ctrl->odom_x;
    ctrl->start_odom_y   = ctrl->odom_y;
    ctrl->start_odom_yaw = ctrl->odom_yaw;

    for (i = 0U; i < CHASSIS_MOTOR_COUNT; i++) {
        ctrl->start_enc[i] = motor_fdcan1[i].total_angle;
    }

    /* ── 保存目标 ── */
    ctrl->target_dx   = dx;
    ctrl->target_dy   = dy;
    ctrl->target_dyaw = dyaw;

    /* ── 三个轴分别做 S 曲线规划 ── */
    SCurve_Plan(&ctrl->s_x,   dx,   ctrl->v_max, ctrl->a_max, ctrl->j_max);
    SCurve_Plan(&ctrl->s_y,   dy,   ctrl->v_max, ctrl->a_max, ctrl->j_max);
    SCurve_Plan(&ctrl->s_yaw, dyaw, ctrl->v_max, ctrl->a_max, ctrl->j_max);

    /* 取最长的 duration 作为统一时长 */
    max_dur = ctrl->s_x.duration;
    if (ctrl->s_y.duration   > max_dur) max_dur = ctrl->s_y.duration;
    if (ctrl->s_yaw.duration > max_dur) max_dur = ctrl->s_yaw.duration;

    ctrl->total_duration = max_dur;

    /* ── 若三个轴都几乎不动，直接结束 ── */
    if (max_dur < 0.001f) {
        ctrl->state = CHASSIS_MOVE_DONE;
        ctrl->cmd_vel.vx = 0.0f;
        ctrl->cmd_vel.vy = 0.0f;
        ctrl->cmd_vel.vw = 0.0f;
        return;
    }

    /* ── 把三个轴的 duration 统一为最长值，保证同步 ──
     *    直接覆写 duration 字段即可，S 曲线按归一化时间取值。
     */
    ctrl->s_x.duration   = max_dur;
    ctrl->s_y.duration   = max_dur;
    ctrl->s_yaw.duration = max_dur;

    /* ── 记录启动时间，由外部传入 ── */
    ctrl->start_time = 0.0f;
    ctrl->state      = CHASSIS_MOVE_RUNNING;

    ctrl->cmd_vel.vx = 0.0f;
    ctrl->cmd_vel.vy = 0.0f;
    ctrl->cmd_vel.vw = 0.0f;
}

/*
 * 每控制周期调用一次。
 * 根据当前时间计算 S 曲线归一化进度 t_norm，
 * 查表得到参考位置/速度作为前馈，叠加里程计位置误差 × P 增益作为修正，
 * 最终输出期望底盘速度 cmd_vel。
 */
ChassisMove_State_t ChassisMove_Update(ChassisMove_Ctrl_t *ctrl,
                                       float dt, float now_sec)
{
    float elapsed, t_norm;
    float ref_x, ref_y, ref_yaw;     /* S 曲线参考位置 */
    float vff_x, vff_y, vff_yaw;     /* S 曲线前馈速度 */
    float err_x, err_y, err_yaw;

    if (ctrl == NULL) return CHASSIS_MOVE_IDLE;
    if (ctrl->state != CHASSIS_MOVE_RUNNING) return ctrl->state;

    /* 第一次进入记录启动时刻 */
    if (ctrl->start_time == 0.0f) {
        ctrl->start_time = now_sec;
    }

    elapsed = now_sec - ctrl->start_time;
    if (elapsed < 0.0f) elapsed = 0.0f;

    t_norm = (ctrl->total_duration > 0.001f)
             ? (elapsed / ctrl->total_duration) : 1.0f;

    /* ── 轨迹结束 ── */
    if (t_norm >= 1.0f) {
        ctrl->state = CHASSIS_MOVE_DONE;
        ctrl->cmd_vel.vx = 0.0f;
        ctrl->cmd_vel.vy = 0.0f;
        ctrl->cmd_vel.vw = 0.0f;
        return CHASSIS_MOVE_DONE;
    }

    /* ── S 曲线参考值 ── */
    ref_x   = SCurve_EvalPos(&ctrl->s_x,   t_norm);
    ref_y   = SCurve_EvalPos(&ctrl->s_y,   t_norm);
    ref_yaw = SCurve_EvalPos(&ctrl->s_yaw, t_norm);

    vff_x   = SCurve_EvalVel(&ctrl->s_x,   t_norm);
    vff_y   = SCurve_EvalVel(&ctrl->s_y,   t_norm);
    vff_yaw = SCurve_EvalVel(&ctrl->s_yaw, t_norm);

    /* Compare against the trajectory in the start body frame. */
    float body_dx, body_dy;
    INS_NavMath_WorldToRobot(ctrl->odom_x - ctrl->start_odom_x,
                             ctrl->odom_y - ctrl->start_odom_y,
                             ctrl->start_odom_yaw, &body_dx, &body_dy);
    err_x   = ref_x - body_dx;
    err_y   = ref_y - body_dy;
    err_yaw = ref_yaw - (ctrl->odom_yaw - ctrl->start_odom_yaw);

    /* ── 前馈 + P 修正 = 期望底盘速度 ── */
    ctrl->cmd_vel.vx = vff_x   + ctrl->pos_kp * err_x;
    ctrl->cmd_vel.vy = vff_y   + ctrl->pos_kp * err_y;
    ctrl->cmd_vel.vw = vff_yaw + ctrl->pos_kp * err_yaw;

    INS_NavMath_WorldToRobot(ctrl->cmd_vel.vx, ctrl->cmd_vel.vy,
                             ctrl->odom_yaw - ctrl->start_odom_yaw,
                             &ctrl->cmd_vel.vx, &ctrl->cmd_vel.vy);
    (void)dt;
    return CHASSIS_MOVE_RUNNING;
}

/* 急停：立即清零输出并回到 IDLE，放弃当前运动 */
void ChassisMove_Stop(ChassisMove_Ctrl_t *ctrl)
{
    if (ctrl == NULL) return;
    ctrl->state = CHASSIS_MOVE_IDLE;
    ctrl->cmd_vel.vx = 0.0f;
    ctrl->cmd_vel.vy = 0.0f;
    ctrl->cmd_vel.vw = 0.0f;
}
