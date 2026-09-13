#include "R2_move.h"
#include "pid_user.h"
#include "ins_nav_math.h"

/*
 * ─── 八种麦克纳姆底盘运动模式 ─────────────────────────
 *
 * VEL 模式（不给定距离）：
 *   上位机持续发送 vx/vy/vw，内部 speedPlanner 做加减速平滑，
 *   可随时修改目标速度。
 *
 *   R2_MODE_ROBOT_NO_YAW_VEL  机器人系速度，朝向锁定（目标角速度为零，PID 可输出纠偏）
 *   R2_MODE_ROBOT_VEL         机器人系速度，朝向可变
 *   R2_MODE_WORLD_NO_YAW_VEL  世界系速度，  朝向锁定（目标角速度为零，PID 可输出纠偏）
 *   R2_MODE_WORLD_VEL         世界系速度，  朝向可变
 *
 * POS 模式（给定距离）：
 *   上位机发送目标位移 dx/dy/dyaw，内部 S 曲线规划轨迹，
 *   位置 P 环闭环。运动完成自动停转，状态变为 DONE。
 *
 *   R2_MODE_ROBOT_NO_YAW_POS  机器人系位移，朝向锁定（dyaw 输入忽略）
 *   R2_MODE_ROBOT_POS         机器人系位移，朝向可变
 *   R2_MODE_WORLD_NO_YAW_POS  世界系位移，  朝向锁定（dyaw 输入忽略）
 *   R2_MODE_WORLD_POS         世界系位移，  朝向可变
 *
 * ─── 坐标系转换 ──────────────────────────────────────
 *
 * 世界系 → 机器人系（旋转 -yaw）：
 *   [ robot_x ]   [  cos(θ)  sin(θ) ] [ world_x ]
 *   [ robot_y ] = [ -sin(θ)  cos(θ) ] [ world_y ]
 *
 * POS 模式流程：
 *   1. ROBOT 模式在启动时机器人系规划；WORLD 模式在世界系规划
 *   2. 各轴五次多项式轨迹同步时长
 *   3. 反馈和目标在相同参考系内计算误差
 *   4. 输出转到当前机器人系，再调用 Mecanum_Calc
 */

/* ─── 内部常量 ──────────────────────────────────────── */

#define R2_POS_MIN_DURATION       0.001f   /* 最短运动时长 1ms，防除零 */
#define R2_POS_EPSILON            1e-6f    /* 零位移判定阈值 */
#define R2_POS_ERR_XY_THRESHOLD   0.02f    /* XY 到位阈值 (m)，误差小于此值认为到达 */
#define R2_POS_ERR_YAW_THRESHOLD  0.05f    /* yaw 到位阈值 (rad ≈ 3°)，误差小于此值认为到达 */
#define R2_POS_TIMEOUT_MULT       2.0f     /* 超时倍数：2×规划时长仍未到达 → 强制结束 */
#define RAD_TO_DEG                57.29578f /* rad → ° */
#define DEG_TO_RAD                0.0174533f/* ° → rad */
#define R2_YAW_LOCK_FRAME_ROBOT   0U
#define R2_YAW_LOCK_FRAME_WORLD   1U

/*
 * 角度归一化到 [-π, +π]，用于 world_lock_yaw 存放前规整。
 * 避免里程计多圈累积传入 Chassis_Yaw_World_Frame_Ctrl
 * 后触发 Format_Yaw_Angle 大量 while 循环。
 */
static float wrap_pi(float rad)
{
    while (rad >  3.1415926f) rad -= 6.2831853f;
    while (rad < -3.1415926f) rad += 6.2831853f;
    return rad;
}

/* ─── 内部前向声明 ──────────────────────────────────── */

static void R2_Move_Update_Vel(R2_Move_Ctrl_t *ctrl);
static void R2_Move_Update_Pos(R2_Move_Ctrl_t *ctrl, float now_sec);

/*
 * 世界系 → 机器人系旋转。
 * 输入世界系 (wx, wy)，当前朝向 yaw，输出机器人系 (rx, ry)。
 * yaw 是世界系中机器人 +X 的朝向角；向量分量变换使用 -yaw，
 * 等价于将世界系向量逆时针旋转 -yaw 到机器人系。
 */
static void world_to_robot(float wx, float wy, float yaw,
                           float *rx, float *ry)
{
    INS_NavMath_WorldToRobot(wx, wy, yaw, rx, ry);
}

static void R2_Move_EnsureYawLockForMode(R2_Move_Ctrl_t *ctrl,
                                         R2_MoveMode_t mode)
{
    if ((ctrl == NULL) || !R2_Move_IsNoYawMode(mode)) {
        return;
    }

    if (R2_Move_IsWorldMode(mode)) {
        ctrl->world_lock_yaw = wrap_pi(ctrl->world_lock_yaw);
        ctrl->yaw_lock_frame = R2_YAW_LOCK_FRAME_WORLD;
        ctrl->yaw_lock_valid = 1U;
    } else {
        if ((ctrl->yaw_lock_valid == 0U) ||
            (ctrl->yaw_lock_frame != R2_YAW_LOCK_FRAME_ROBOT)) {
            ctrl->robot_lock_origin_yaw = wrap_pi(ctrl->odom_yaw);
        }
        ctrl->robot_lock_yaw = wrap_pi(ctrl->robot_lock_yaw);
        ctrl->yaw_lock_frame = R2_YAW_LOCK_FRAME_ROBOT;
        ctrl->yaw_lock_valid = 1U;
    }
}

static void R2_Move_EnsureYawLock(R2_Move_Ctrl_t *ctrl)
{
    if (ctrl == NULL) return;
    R2_Move_EnsureYawLockForMode(ctrl, ctrl->mode);
}

static float R2_Move_GetNoYawTargetWorldYaw(R2_Move_Ctrl_t *ctrl)
{
    if (ctrl == NULL) return 0.0f;

    R2_Move_EnsureYawLock(ctrl);
    if (!R2_Move_IsWorldMode(ctrl->mode)) {
        return wrap_pi(ctrl->robot_lock_origin_yaw + ctrl->robot_lock_yaw);
    }
    return wrap_pi(ctrl->world_lock_yaw);
}

static float R2_Move_ClampFloat(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static void R2_Move_LimitXYVector(float *vx, float *vy, float v_max)
{
    float mag;

    if ((vx == NULL) || (vy == NULL) || (v_max <= 0.0f)) {
        return;
    }

    mag = sqrtf((*vx * *vx) + (*vy * *vy));
    if (mag > v_max) {
        float scale = v_max / mag;
        *vx *= scale;
        *vy *= scale;
    }
}

/* ─── 生命周期 ──────────────────────────────────────── */

void R2_Move_Init(R2_Move_Ctrl_t *ctrl, const MecanumParam_t *param, float dt_s)
{
    if (ctrl == NULL) return;

    memset(ctrl, 0, sizeof(*ctrl));

    ctrl->mode           = R2_MODE_ROBOT_NO_YAW_VEL;
    ctrl->pos_state      = R2_POS_IDLE;
    ctrl->param          = param;
    ctrl->emergency_stop = 1U;   /* 上电急停，收到第一帧遥控/上位机命令后自动解除 */

    /* 速度平滑器：横移轴更软，降低麦轮侧滑启停时的 yaw 冲击 */
    speedPlanner_Init(&ctrl->sp_vx, R2_MOVE_X_A_MAX_MPS2, R2_MOVE_X_A_MAX_MPS2, dt_s);
    speedPlanner_Init(&ctrl->sp_vy, R2_MOVE_Y_A_MAX_MPS2, R2_MOVE_Y_A_MAX_MPS2, dt_s);
    speedPlanner_Init(&ctrl->sp_vw, 6.0f, 20.0f, dt_s);

    /* 默认运动参数 */
    ctrl->v_max = R2_MOVE_XY_V_MAX_MPS;
    ctrl->a_max = R2_MOVE_XY_A_MAX_MPS2;
    ctrl->j_max = R2_MOVE_XY_J_MAX_MPS3;
    ctrl->pos_kp     = 3.0f;
    ctrl->pos_kp_yaw = 1.8f;   /* yaw position P from auto tune */
}

/* ─── 参数设定 ──────────────────────────────────────── */

void R2_Move_SetLimits(R2_Move_Ctrl_t *ctrl, float v_max, float a_max, float j_max)
{
    if (ctrl == NULL) return;

    if (v_max <= 0.0f) v_max = R2_MOVE_XY_V_MAX_MPS;
    if (a_max <= 0.0f) a_max = R2_MOVE_XY_A_MAX_MPS2;
    if (j_max <= 0.0f) j_max = R2_MOVE_XY_J_MAX_MPS3;

    ctrl->v_max = v_max;
    ctrl->a_max = a_max;
    ctrl->j_max = j_max;

    ctrl->sp_vx.accel = a_max;
    ctrl->sp_vx.decel = a_max;
    ctrl->sp_vy.accel = a_max;
    ctrl->sp_vy.decel = a_max;
}

void R2_Move_SetMode(R2_Move_Ctrl_t *ctrl, R2_MoveMode_t mode)
{
    R2_MoveMode_t old_mode;
    uint8_t old_no_yaw;
    uint8_t new_no_yaw;
    uint8_t frame_changed;

    if (ctrl == NULL) return;

    old_mode = ctrl->mode;
    old_no_yaw = R2_Move_IsNoYawMode(old_mode);
    new_no_yaw = R2_Move_IsNoYawMode(mode);
    frame_changed = (R2_Move_IsWorldMode(old_mode) != R2_Move_IsWorldMode(mode))
                    ? 1U : 0U;

    /* 从 POS 切换到 VEL 时中止未完成的位置运动 */
    if (R2_Move_IsPosMode(ctrl->mode) && R2_Move_IsVelMode(mode)) {
        ctrl->pos_state = R2_POS_IDLE;
    }

    ctrl->mode = mode;

    /* NO_YAW modes keep the last commanded heading target in their own frame. */
    if (new_no_yaw != 0U) {
        if ((old_no_yaw == 0U) || (frame_changed != 0U)) {
            ctrl->yaw_lock_valid = 0U;
            if (!R2_Move_IsWorldMode(mode)) {
                ctrl->robot_lock_yaw = 0.0f;
            }
        }
        R2_Move_EnsureYawLockForMode(ctrl, mode);
    } else {
        ctrl->yaw_lock_valid = 0U;
    }

    /* 切换模式时清除 IMU yaw PID 历史，防止旧积分干扰 */
    Chassis_Yaw_PID_Clear();
}

/* ─── VEL 模式：速度设定 ────────────────────────────── */

void R2_Move_SetVel(R2_Move_Ctrl_t *ctrl, float vx, float vy, float vw)
{
    if (ctrl == NULL) return;

    R2_Move_LimitXYVector(&vx, &vy, ctrl->v_max);
    vw = R2_Move_ClampFloat(vw, MEC_REMOTE_VW_MIN_RAD_S, MEC_REMOTE_VW_MAX_RAD_S);

    /* 保存速度目标；调用此接口会中止未完成位置任务，实际运行需 VEL 模式。 */
    ctrl->target_vx = vx;
    ctrl->target_vy = vy;
    ctrl->target_vw = vw;

    /* 将目标值写入 speedPlanner，由它做斜坡平滑 */
    speedPlanner_SetTarget(&ctrl->sp_vx, vx);
    speedPlanner_SetTarget(&ctrl->sp_vy, vy);

    if (R2_Move_IsNoYawMode(ctrl->mode)) {
        R2_Move_EnsureYawLock(ctrl);
        speedPlanner_SetTarget(&ctrl->sp_vw, 0.0f);
    } else {
        ctrl->yaw_lock_valid = 0U;
        speedPlanner_SetTarget(&ctrl->sp_vw, vw);
    }

    /* 有新速度写入就自动解除急停，并中止未完成的位置运动 */
    ctrl->emergency_stop = 0U;
    if (R2_Move_IsPosMode(ctrl->mode)) {
        ctrl->pos_state = R2_POS_IDLE;
    }
}

/* ─── POS 模式：位移设定 ────────────────────────────── */

/*
 * 启动一次位置控制运动。
 *
 * 坐标系策略：
 *   ROBOT 模式：dx/dy 是机器人系位移，S 曲线在机器人系规划。
 *               位置反馈用世界系里程计增量旋转到 start-robot-frame。
 *   WORLD 模式：dx/dy 是世界系位移，S 曲线直接在世界系规划。
 *               位置反馈直接用世界系里程计增量（无需旋转）。
 *
 * 内部流程：
 *   1. ROBOT: NOP;  WORLD: 不转换，S 曲线直接用世界系距离
 *   2. 对三个轴分别做 S 曲线规划
 *   3. 取最长 duration 统一三轴时长，保证同步到达
 *   4. 快照当前里程计作为起点
 */
int8_t R2_Move_SetDist(R2_Move_Ctrl_t *ctrl, float dx, float dy, float dyaw)
{
    float max_dur;

    if (ctrl == NULL) return -1;
    if (ctrl->pos_state == R2_POS_RUNNING) return -1;  /* 忙，不可重入 */

    /* 必须在 POS 模式下调此接口 */
    if (!R2_Move_IsPosMode(ctrl->mode)) return -1;

    /*
     * 规划坐标系选择：
     *   ROBOT 模式：S 曲线在机器人系规划，世界系里程计反馈时需旋转
     *   WORLD 模式：S 曲线在世界系规划，里程计反馈直接使用
     */
    if (R2_Move_IsWorldMode(ctrl->mode)) {
        ctrl->pos_world_plan = 1U;
        /* WORLD: dx/dy 直接作为世界系位移规划 */
    } else {
        ctrl->pos_world_plan = 0U;
        /* ROBOT: dx/dy 即为机器人系位移 */
    }

    /* ── S 曲线规划 ── */
    SCurve_Plan(&ctrl->plan_x,   dx,   ctrl->v_max, ctrl->a_max, ctrl->j_max);
    SCurve_Plan(&ctrl->plan_y,   dy,   ctrl->v_max, ctrl->a_max, ctrl->j_max);

    if (R2_Move_IsNoYawMode(ctrl->mode)) {
        R2_Move_EnsureYawLock(ctrl);
        SCurve_Plan(&ctrl->plan_yaw, 0.0f, ctrl->v_max, ctrl->a_max, ctrl->j_max);
    } else {
        ctrl->yaw_lock_valid = 0U;
        SCurve_Plan(&ctrl->plan_yaw, dyaw, ctrl->v_max, ctrl->a_max, ctrl->j_max);
    }

    /* ── 三轴同步：取最长 duration ── */
    max_dur = ctrl->plan_x.duration;
    if (ctrl->plan_y.duration   > max_dur) max_dur = ctrl->plan_y.duration;
    if (ctrl->plan_yaw.duration > max_dur) max_dur = ctrl->plan_yaw.duration;

    /* 三个轴几乎不动 → 直接标记完成 */
    if (max_dur < R2_POS_MIN_DURATION) {
        ctrl->pos_state          = R2_POS_DONE;
        ctrl->pos_total_duration = 0.0f;
        ctrl->robot_vel.vx = 0.0f;
        ctrl->robot_vel.vy = 0.0f;
        ctrl->robot_vel.vw = 0.0f;
        ctrl->wheel_speed.fl = 0.0f;
        ctrl->wheel_speed.fr = 0.0f;
        ctrl->wheel_speed.bl = 0.0f;
        ctrl->wheel_speed.br = 0.0f;
        return 0;
    }

    /* 覆写较短轴 duration，确保同时到达 */
    ctrl->plan_x.duration   = max_dur;
    ctrl->plan_y.duration   = max_dur;
    ctrl->plan_yaw.duration = max_dur;

    ctrl->pos_total_duration = max_dur;
    ctrl->pos_start_time     = 0.0f;       /* 首次 Update 时记录 */

    /* ── 快照里程计起点（世界系） ── */
    ctrl->pos_start_x   = ctrl->odom_x;
    ctrl->pos_start_y   = ctrl->odom_y;
    ctrl->pos_start_yaw = ctrl->odom_yaw;

    /* 保存目标位移，供外部查询 */
    ctrl->target_dx   = dx;
    ctrl->target_dy   = dy;
    ctrl->target_dyaw = dyaw;

    /* 清零输出，等待 Update 驱动 */
    ctrl->robot_vel.vx = 0.0f;
    ctrl->robot_vel.vy = 0.0f;
    ctrl->robot_vel.vw = 0.0f;

    ctrl->pos_state      = R2_POS_RUNNING;
    ctrl->emergency_stop = 0U;

    return 0;
}

R2_PosState_t R2_Move_GetPosState(const R2_Move_Ctrl_t *ctrl)
{
    if (ctrl == NULL) return R2_POS_IDLE;
    return ctrl->pos_state;
}

/* ─── NO_YAW 锁定朝向 ───────────────────────────────── */

void R2_Move_SetRobotLockYaw(R2_Move_Ctrl_t *ctrl, float yaw_rad)
{
    if (ctrl == NULL) return;
    if ((ctrl->yaw_lock_valid == 0U) ||
        (ctrl->yaw_lock_frame != R2_YAW_LOCK_FRAME_ROBOT)) {
        ctrl->robot_lock_origin_yaw = wrap_pi(ctrl->odom_yaw);
    }
    ctrl->robot_lock_yaw = wrap_pi(yaw_rad);
    ctrl->yaw_lock_frame = R2_YAW_LOCK_FRAME_ROBOT;
    ctrl->yaw_lock_valid = 1U;
}

void R2_Move_SetWorldLockYaw(R2_Move_Ctrl_t *ctrl, float yaw_rad)
{
    if (ctrl == NULL) return;
    ctrl->world_lock_yaw = wrap_pi(yaw_rad);
    ctrl->yaw_lock_frame = R2_YAW_LOCK_FRAME_WORLD;
    ctrl->yaw_lock_valid = 1U;
}

/* ─── 位姿更新 ──────────────────────────────────────── */

void R2_Move_UpdateYaw(R2_Move_Ctrl_t *ctrl, float yaw)
{
    if (ctrl == NULL) return;
    ctrl->odom_yaw = yaw;
}

/*
 * 更新里程计。
 *
 * robot_dx / robot_dy 是机器人系下的本轮位移增量（由轮速编码器积分得到），
 * 内部旋转到世界系后累加。
 *
 * delta_yaw 是 yaw 角增量 (rad)，两种坐标系下数值相同。
 *
 * 注意：调用方应当先调用 R2_Move_UpdateYaw() 更新当前朝向，
 * 再调用本函数，确保 robot→world 旋转用最新 yaw。
 */
void R2_Move_UpdateOdom(R2_Move_Ctrl_t *ctrl, float robot_dx,
                        float robot_dy, float delta_yaw)
{
    float world_dx, world_dy;

    if (ctrl == NULL) return;
    INS_NavMath_RobotToWorld(robot_dx, robot_dy, ctrl->odom_yaw,
                             &world_dx, &world_dy);
    ctrl->odom_x += world_dx;
    ctrl->odom_y += world_dy;
    ctrl->odom_yaw += delta_yaw;
}

/* ─── 核心更新 ──────────────────────────────────────── */

void R2_Move_Update(R2_Move_Ctrl_t *ctrl, float now_sec)
{
    if (ctrl == NULL) return;

    /* ── 急停 ── */
    if (ctrl->emergency_stop != 0U) {
        goto zero_output;
    }

    /* ── 分支：VEL 模式 ── */
    if (R2_Move_IsVelMode(ctrl->mode)) {
        R2_Move_Update_Vel(ctrl);
        return;
    }

    /* ── 分支：POS 模式 ── */
    if (R2_Move_IsPosMode(ctrl->mode)) {
        R2_Move_Update_Pos(ctrl, now_sec);
        return;
    }

zero_output:
    ctrl->robot_vel.vx = 0.0f;
    ctrl->robot_vel.vy = 0.0f;
    ctrl->robot_vel.vw = 0.0f;
    ctrl->wheel_speed.fl = 0.0f;
    ctrl->wheel_speed.fr = 0.0f;
    ctrl->wheel_speed.bl = 0.0f;
    ctrl->wheel_speed.br = 0.0f;
}

/* ─── VEL 模式更新 ──────────────────────────────────── */

/*
 * VEL 模式流程：
 *   1. speedPlanner 做加减速平滑，得到当前瞬时速度
 *   2. WORLD 模式：世界系速度 → 机器人系速度
 *   3. NO_YAW / YAW → Chassis_Yaw_*_Ctrl IMU 闭环
 *   4. Mecanum_Calc 解算轮速
 */
static void R2_Move_Update_Vel(R2_Move_Ctrl_t *ctrl)
{
    float vx_smooth, vy_smooth, vw_smooth;
    float vx_robot, vy_robot, vw_robot;

    /* Step 1: speedPlanner 平滑 */
    vx_smooth = speedPlanner_Update(&ctrl->sp_vx);
    vy_smooth = speedPlanner_Update(&ctrl->sp_vy);
    vw_smooth = speedPlanner_Update(&ctrl->sp_vw);

    /* Step 2: 坐标系转换 */
    if (R2_Move_IsWorldMode(ctrl->mode)) {
        world_to_robot(vx_smooth, vy_smooth, ctrl->odom_yaw,
                       &vx_robot, &vy_robot);
    } else {
        vx_robot = vx_smooth;
        vy_robot = vy_smooth;
    }

    /*
     * Step 3: yaw 控制（IMU 闭环）。
     *
     * PID 返回值是 °/s，乘 DEG_TO_RAD 转为 rad/s。
     *
     * ROBOT_NO_YAW:     机器人系锁定角 → World_Frame_Ctrl(换算后目标角)
     * ROBOT:            跟踪目标角速度 → Robot_Frame_Ctrl(vw °/s)
     * WORLD_NO_YAW:     世界系锁定角    → World_Frame_Ctrl(目标角)
     * WORLD:            跟踪目标角速度 → Robot_Frame_Ctrl(vw °/s)
     */
    switch (ctrl->mode) {

    case R2_MODE_ROBOT_NO_YAW_VEL:
        vw_robot = Chassis_Yaw_World_Frame_Ctrl(
                       R2_Move_GetNoYawTargetWorldYaw(ctrl) * RAD_TO_DEG)
                   * DEG_TO_RAD;
        break;

    case R2_MODE_ROBOT_VEL:
        vw_robot = Chassis_Yaw_Robot_Frame_Ctrl(vw_smooth * RAD_TO_DEG)
                   * DEG_TO_RAD;
        break;

    case R2_MODE_WORLD_NO_YAW_VEL:
        vw_robot = Chassis_Yaw_World_Frame_Ctrl(
                       R2_Move_GetNoYawTargetWorldYaw(ctrl) * RAD_TO_DEG)
                   * DEG_TO_RAD;
        break;

    case R2_MODE_WORLD_VEL:
        vw_robot = Chassis_Yaw_Robot_Frame_Ctrl(vw_smooth * RAD_TO_DEG)
                   * DEG_TO_RAD;
        break;

    default:
        vw_robot = 0.0f;
        break;
    }

    /* Step 4: 写入 + 逆运动学 */
    ctrl->robot_vel.vx = vx_robot;
    ctrl->robot_vel.vy = vy_robot;
    ctrl->robot_vel.vw = vw_robot;

    if (ctrl->param != NULL) {
        Mecanum_Calc(&ctrl->robot_vel, ctrl->param, &ctrl->wheel_speed);
    }
}

/* ─── POS 模式更新 ──────────────────────────────────── */

/*
 * POS 模式流程。
 *
 * 框架选择：
 *   pos_world_plan == 0（ROBOT 模式）：
 *     1. 非 RUNNING → 零输出
 *     2. 首次进入记录 pos_start_time
 *     3. 计算 t_norm，轨迹结束后等待误差收敛或超时才 DONE
 *     4. S 曲线查表得 ref（start-robot-frame 下的参考位置/速度）
 *     5. 世界系里程计增量 → 旋转到 start-robot-frame → 位置误差
 *     6. P 环：cmd = vff + Kp * err（结果在 start-robot-frame）
 *     7. cmd 旋转到 current-robot-frame，包括锁 yaw 时的实际偏差
 *     8. NO_YAW 用目标朝向做闭环，vw 可以非零
 *     9. Mecanum_Calc
 *
 *   pos_world_plan == 1（WORLD 模式）：
 *     1~3 同上
 *     4. S 曲线查表得 ref（world-frame 下的参考位置/速度）
 *     5. 世界系里程计增量直接做位置误差（无需旋转）
 *     6. P 环：cmd = vff + Kp * err（结果在 world-frame）
 *     7. cmd 旋转到 current-robot-frame
 *     8. NO_YAW 用目标朝向做闭环，vw 可以非零
 *     9. Mecanum_Calc
 *
 * 第 7 步是解决"边平移边旋转导致轨迹弯曲"的关键：
 *   P 环输出的速度在参考系（start-robot 或 world），
 *   必须旋转到机器人当前朝向的坐标系，Mecanum_Calc 才能正确解算。
 */
static void R2_Move_Update_Pos(R2_Move_Ctrl_t *ctrl, float now_sec)
{
    float elapsed, t_norm;
    float ref_x,  ref_y,  ref_yaw;
    float vff_x,  vff_y,  vff_yaw;
    float delta_x, delta_y, delta_yaw;
    float err_x, err_y, err_yaw;
    float cmd_vx, cmd_vy, cmd_vw;

    if (ctrl->pos_state != R2_POS_RUNNING) {
        goto zero_output;
    }

    /* 首次进入记录启动时刻 */
    if (ctrl->pos_start_time == 0.0f) {
        ctrl->pos_start_time = now_sec;
    }

    /* 计算归一化时间 */
    elapsed = now_sec - ctrl->pos_start_time;
    if (elapsed < 0.0f) elapsed = 0.0f;

    t_norm = (ctrl->pos_total_duration > R2_POS_MIN_DURATION)
             ? (elapsed / ctrl->pos_total_duration) : 1.0f;

    /*
     * S 曲线求值用 clamped 时间，t_norm 本身可能 >1.0（超时后仍保持 RUNNING）。
     * 超时后 VFF = 0，参考位置锁定终点，P 环继续纠偏。
     */
    {
        float t_eval = (t_norm > 1.0f) ? 1.0f : t_norm;
        uint8_t past_end = (t_norm >= 1.0f) ? 1U : 0U;

        ref_x   = SCurve_EvalPos(&ctrl->plan_x,   t_eval);
        ref_y   = SCurve_EvalPos(&ctrl->plan_y,   t_eval);
        ref_yaw = SCurve_EvalPos(&ctrl->plan_yaw, t_eval);

        vff_x   = past_end ? 0.0f : SCurve_EvalVel(&ctrl->plan_x,   t_eval);
        vff_y   = past_end ? 0.0f : SCurve_EvalVel(&ctrl->plan_y,   t_eval);
        vff_yaw = past_end ? 0.0f : SCurve_EvalVel(&ctrl->plan_yaw, t_eval);
    }

    /* 世界系里程计增量 */
    delta_x   = ctrl->odom_x   - ctrl->pos_start_x;
    delta_y   = ctrl->odom_y   - ctrl->pos_start_y;
    delta_yaw = ctrl->odom_yaw - ctrl->pos_start_yaw;

    if (ctrl->pos_world_plan == 0U) {
        /*
         * ROBOT 模式：
         * ref / vff 在 start-robot-frame 内。
         * 里程计是世界系 → 旋转到 start-robot-frame 做误差。
         * cmd 在 start-robot-frame。
         */
        float robot_dx, robot_dy;

        world_to_robot(delta_x, delta_y, ctrl->pos_start_yaw,
                       &robot_dx, &robot_dy);

        err_x = ref_x - robot_dx;
        err_y = ref_y - robot_dy;

        cmd_vx = vff_x + ctrl->pos_kp * err_x;
        cmd_vy = vff_y + ctrl->pos_kp * err_y;

        /*
         * 将 cmd 从 start-robot-frame
         * 旋转到 current-robot-frame，否则轨迹弯曲。
         *
         * 帧关系：current frame = start frame 逆时针转 Δθ。
         * 向量坐标做被动变换（顺时针转 Δθ）：
         *   x' =  x·cos(Δθ) + y·sin(Δθ)
         *   y' = -x·sin(Δθ) + y·cos(Δθ)
         * 等价于 world_to_robot(cx, cy, delta_yaw, &rx, &ry)。
         */
        { /* Also rotate during yaw hold: measured heading can differ from start. */
            float rx, ry;
            world_to_robot(cmd_vx, cmd_vy, delta_yaw, &rx, &ry);
            cmd_vx = rx;
            cmd_vy = ry;
        }
    } else {
        /*
         * WORLD 模式：
         * ref / vff 在世界系内，里程计也在世界系。
         * 直接在世界系做误差，最后再旋转到机器人系。
         */
        err_x = ref_x - delta_x;
        err_y = ref_y - delta_y;

        cmd_vx = vff_x + ctrl->pos_kp * err_x;
        cmd_vy = vff_y + ctrl->pos_kp * err_y;

        /* 世界系 → 机器人系（用实时朝向） */
        {
            float rx, ry;
            world_to_robot(cmd_vx, cmd_vy, ctrl->odom_yaw, &rx, &ry);
            cmd_vx = rx;
            cmd_vy = ry;
        }
    }

    /*
     * yaw 控制：用 S 曲线 vff + odometry P 环计算目标角速度，
     * 再经 Chassis_Yaw_*_Ctrl 做 IMU 陀螺闭环。
     *
     * NO_YAW 模式：IMU 主动抑制 yaw 漂移 / 锁定绝对朝向。
     */
    if (R2_Move_IsNoYawMode(ctrl->mode)) {
        err_yaw = wrap_pi(R2_Move_GetNoYawTargetWorldYaw(ctrl) -
                          ctrl->odom_yaw);
    } else {
        err_yaw = wrap_pi(ref_yaw - delta_yaw);
    }
    cmd_vw  = vff_yaw + ctrl->pos_kp_yaw * err_yaw;

    switch (ctrl->mode) {

    case R2_MODE_ROBOT_NO_YAW_POS:
        cmd_vw = Chassis_Yaw_World_Frame_Ctrl(
                     R2_Move_GetNoYawTargetWorldYaw(ctrl) * RAD_TO_DEG)
                 * DEG_TO_RAD;
        break;

    case R2_MODE_ROBOT_POS:
        cmd_vw = Chassis_Yaw_Robot_Frame_Ctrl(cmd_vw * RAD_TO_DEG)
                 * DEG_TO_RAD;
        break;

    case R2_MODE_WORLD_NO_YAW_POS:
        cmd_vw = Chassis_Yaw_World_Frame_Ctrl(
                     R2_Move_GetNoYawTargetWorldYaw(ctrl) * RAD_TO_DEG)
                 * DEG_TO_RAD;
        break;

    case R2_MODE_WORLD_POS:
        cmd_vw = Chassis_Yaw_Robot_Frame_Ctrl(cmd_vw * RAD_TO_DEG)
                 * DEG_TO_RAD;
        break;

    default:
        cmd_vw = 0.0f;
        break;
    }

    /* ── 存储诊断值（供上位机判断真实到位） ── */
    ctrl->pos_progress = (t_norm > 1.0f) ? 1.0f : t_norm;
    ctrl->pos_err_x    = err_x;
    ctrl->pos_err_y    = err_y;
    ctrl->pos_err_yaw  = err_yaw;

    /*
     * ── DONE 判断：S 曲线结束 且 位置误差收敛 ──
     *
     * 策略：
     *   1. t_norm < 1.0  → 正常 RUNNING，S 曲线前馈 + P 环纠偏
     *   2. t_norm ≥ 1.0  → VFF 已归零，P 环继续用 ref 终点做位置闭环
     *      a) 误差已收敛 → 真正到达，DONE
     *      b) t_norm < TIMEOUT_MULT → 继续纠偏（被绊一下/稍微落后继续追）
     *      c) t_norm ≥ TIMEOUT_MULT → 超时保护，强制 DONE（上位机看大误差报警）
     */
    if (t_norm >= 1.0f) {
        float err_xy = sqrtf(err_x * err_x + err_y * err_y);

        if (err_xy < R2_POS_ERR_XY_THRESHOLD
            && fabsf(err_yaw) < R2_POS_ERR_YAW_THRESHOLD) {
            /* 误差收敛 → 真正到达 */
            ctrl->pos_state = R2_POS_DONE;
            goto zero_output;
        }

        if (t_norm >= R2_POS_TIMEOUT_MULT) {
            /* 超时仍未到达 → 强制结束（上位机诊断字段带大误差） */
            ctrl->pos_state = R2_POS_DONE;
            goto zero_output;
        }

        /* 否则：t_norm ∈ [1.0, TIMEOUT_MULT)，P 环继续纠偏 */
    }

    /* 写入 + 逆运动学 */
    R2_Move_LimitXYVector(&cmd_vx, &cmd_vy, ctrl->v_max);
    ctrl->robot_vel.vx = cmd_vx;
    ctrl->robot_vel.vy = cmd_vy;
    ctrl->robot_vel.vw = cmd_vw;

    if (ctrl->param != NULL) {
        Mecanum_Calc(&ctrl->robot_vel, ctrl->param, &ctrl->wheel_speed);
    }
    return;

zero_output:
    ctrl->robot_vel.vx   = 0.0f;
    ctrl->robot_vel.vy   = 0.0f;
    ctrl->robot_vel.vw   = 0.0f;
    ctrl->wheel_speed.fl = 0.0f;
    ctrl->wheel_speed.fr = 0.0f;
    ctrl->wheel_speed.bl = 0.0f;
    ctrl->wheel_speed.br = 0.0f;
}

/* ─── 急停与恢复 ────────────────────────────────────── */

void R2_Move_Stop(R2_Move_Ctrl_t *ctrl)
{
    if (ctrl == NULL) return;

    ctrl->emergency_stop  = 1U;
    ctrl->pos_state       = R2_POS_IDLE;
    ctrl->target_vx       = 0.0f;
    ctrl->target_vy       = 0.0f;
    ctrl->target_vw       = 0.0f;
    ctrl->yaw_lock_valid  = 0U;

    /* 清零 speedPlanner 内部状态 */
    speedPlanner_SetTarget(&ctrl->sp_vx, 0.0f);
    speedPlanner_SetTarget(&ctrl->sp_vy, 0.0f);
    speedPlanner_SetTarget(&ctrl->sp_vw, 0.0f);
    ctrl->sp_vx.current_v = 0.0f;
    ctrl->sp_vy.current_v = 0.0f;
    ctrl->sp_vw.current_v = 0.0f;

    ctrl->robot_vel.vx = 0.0f;
    ctrl->robot_vel.vy = 0.0f;
    ctrl->robot_vel.vw = 0.0f;
    ctrl->wheel_speed.fl = 0.0f;
    ctrl->wheel_speed.fr = 0.0f;
    ctrl->wheel_speed.bl = 0.0f;
    ctrl->wheel_speed.br = 0.0f;
}

void R2_Move_Resume(R2_Move_Ctrl_t *ctrl)
{
    if (ctrl == NULL) return;
    ctrl->emergency_stop = 0U;
}
