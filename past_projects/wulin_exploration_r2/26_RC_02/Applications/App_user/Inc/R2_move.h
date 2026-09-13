#ifndef __R2_MOVE_H
#define __R2_MOVE_H

#include "chassis_move.h"
#include "mecanum_classic.h"
#include "speedPlanner.h"
#include "include.h"
#include <string.h>
#include <math.h>

/* ─── 运动模式枚举（8 种） ──────────────────────────── */

/*
 * 两种参考系 × 是否锁 yaw × 两种控制方式 = 8 种模式。
 *
 * 坐标系：
 *   ROBOT — 机器人系（x=前, y=左, yaw=CCW）
 *   WORLD — 世界系（固定参考系，x/y 由初始朝向定义）
 *
 * 控制方式：
 *   VEL — 不指定距离，速度控制。上位机持续发送 vx/vy/vw，
 *         内部用 speedPlanner 做加减速平滑。
 *   POS — 指定距离，位置控制。上位机发送目标位移 dx/dy/dyaw，
 *         内部用 S 曲线规划轨迹 + 位置 P 环闭环。
 *
 * 模式切换无突变，上位机可随时无缝切换。
 */
typedef enum
{
    /* ── 速度控制（不给定距离） ── */
    R2_MODE_ROBOT_NO_YAW_VEL  = 0,  /* 机器人系平移，朝向锁定，速度模式 */
    R2_MODE_ROBOT_VEL         = 1,  /* 机器人系全向，朝向可变，速度模式 */
    R2_MODE_WORLD_NO_YAW_VEL  = 2,  /* 世界系平移，  朝向锁定，速度模式 */
    R2_MODE_WORLD_VEL         = 3,  /* 世界系全向，  朝向可变，速度模式 */

    /* ── 位置控制（给定距离） ── */
    R2_MODE_ROBOT_NO_YAW_POS  = 4,  /* 机器人系平移，朝向锁定，位置模式 */
    R2_MODE_ROBOT_POS         = 5,  /* 机器人系全向，朝向可变，位置模式 */
    R2_MODE_WORLD_NO_YAW_POS  = 6,  /* 世界系平移，  朝向锁定，位置模式 */
    R2_MODE_WORLD_POS         = 7,  /* 世界系全向，  朝向可变，位置模式 */
} R2_MoveMode_t;

#define R2_MOVE_XY_V_MAX_MPS        MEC_REMOTE_XY_MAX_MPS
#define R2_MOVE_XY_A_MAX_MPS2       2.0f
#define R2_MOVE_X_A_MAX_MPS2        R2_MOVE_XY_A_MAX_MPS2 /* forward/backward */
#define R2_MOVE_Y_A_MAX_MPS2        1.2f /* lateral ramp: reduce start/stop yaw kick */
#define R2_MOVE_XY_J_MAX_MPS3       10.0f

/* ─── 位置控制子状态 ────────────────────────────────── */

typedef enum
{
    R2_POS_IDLE    = 0,  /* 无位置运动 */
    R2_POS_RUNNING = 1,  /* 位置运动执行中 */
    R2_POS_DONE    = 2,  /* 位置运动已完成 */
} R2_PosState_t;

/* ─── 控制器结构体 ──────────────────────────────────── */

typedef struct
{
    R2_MoveMode_t mode;             /* 当前运动模式 */
    R2_PosState_t pos_state;        /* 位置控制子状态 */

    /* ── 上位机设定的目标值 ── */

    /* 速度目标（VEL 模式使用，可被 speedPlanner 平滑） */
    float target_vx;                /* m/s */
    float target_vy;                /* m/s */
    float target_vw;                /* rad/s, CCW 为正 */

    /* 位移目标（POS 模式使用） */
    float target_dx;                /* m,  机器人系：前为正；世界系：X 轴 */
    float target_dy;                /* m,  机器人系：左为正；世界系：Y 轴 */
    float target_dyaw;              /* rad, CCW 为正 */

    /* ── 世界系位姿估计 ── */

    /*
     * 由外部 IMU / 里程计定期写入。
     * POS 模式下自动累加里程计增量用于位置环反馈。
     */
    float odom_x;                   /* m,   世界系 X */
    float odom_y;                   /* m,   世界系 Y */
    float odom_yaw;                 /* rad, 世界系朝向 */

    /* ── 速度平滑器（VEL 模式用） ── */

    speedPlanner_t sp_vx;           /* vx 轴速度斜坡 */
    speedPlanner_t sp_vy;           /* vy 轴速度斜坡 */
    speedPlanner_t sp_vw;           /* vw 轴速度斜坡 */

    /* ── 位置控制（POS 模式用） ── */

    SCurve_Plan_t plan_x;           /* dx 轴 S 曲线规划 */
    SCurve_Plan_t plan_y;           /* dy 轴 S 曲线规划 */
    SCurve_Plan_t plan_yaw;         /* dyaw 轴 S 曲线规划 */
    float pos_total_duration;       /* 三轴中最长规划时长 (s)，用于同步 */
    float pos_start_time;           /* 运动开始时刻 (s) */
    float pos_start_x;              /* 启动时里程计 X 快照 */
    float pos_start_y;              /* 启动时里程计 Y 快照 */
    float pos_start_yaw;            /* 启动时里程计 yaw 快照 */
    float pos_kp;                   /* 平移位置环 P 增益 (1/s) */
    float pos_kp_yaw;               /* yaw 位置环 P 增益 (1/s)，量纲不同需独立整定 */
    uint8_t pos_world_plan;         /* 1=S 曲线在世界系规划（WORLD 模式），0=机器人系 */

    /* ── POS 模式诊断（供上位机判断真实到位） ── */
    float pos_progress;             /* t_norm ∈ [0,1]，当前运动进度 */
    float pos_err_x;                /* 最近一次 X 误差 (m) */
    float pos_err_y;                /* 最近一次 Y 误差 (m) */
    float pos_err_yaw;              /* 最近一次 yaw 误差 (rad) */

    /* ── IMU yaw 锁定 ── */
    float world_lock_yaw;           /* WORLD_NO_YAW 模式锁定的世界系目标角 (rad) */
    float robot_lock_yaw;           /* ROBOT_NO_YAW 模式锁定的机器人系目标角 (rad) */
    float robot_lock_origin_yaw;    /* 机器人系锁定参考朝向，对应世界系 yaw (rad) */
    uint8_t yaw_lock_valid;         /* 1=NO_YAW heading target is valid */
    uint8_t yaw_lock_frame;         /* 0=robot-frame lock, 1=world-frame lock */

    /* ── 运动参数上限 ── */

    float v_max;                    /* 最大线速度 (m/s) */
    float a_max;                    /* 最大加速度 (m/s²) */
    float j_max;                    /* 最大 jerk     (m/s³) */

    /* ── 底盘参数 ── */

    const MecanumParam_t *param;    /* 指向全局 mecParam */

    /* ── 输出 ── */

    ChassisVel_t robot_vel;         /* 机器人系期望底盘速度 */
    WheelSpeed_t wheel_speed;       /* 四轮期望线速度 (m/s) */

    /* ── 安全 ── */

    uint8_t emergency_stop;         /* 急停标志 */

} R2_Move_Ctrl_t;

/* ─── 公开 API ──────────────────────────────────────── */

/**
 * @brief 初始化控制器
 * @param ctrl   控制器句柄
 * @param param  底盘几何参数（指向全局 mecParam）
 * @param dt_s   控制周期 (s)，用于 speedPlanner 初始化
 */
void R2_Move_Init(R2_Move_Ctrl_t *ctrl, const MecanumParam_t *param, float dt_s);

/**
 * @brief 设定运动参数上限
 */
void R2_Move_SetLimits(R2_Move_Ctrl_t *ctrl, float v_max, float a_max, float j_max);

/**
 * @brief 切换运动模式，可随时调用
 */
void R2_Move_SetMode(R2_Move_Ctrl_t *ctrl, R2_MoveMode_t mode);

/* ── VEL 模式接口 ── */

/**
 * @brief 设定目标速度（VEL 模式使用）
 *
 * 速度含义取决于当前坐标系：
 *   ROBOT 系：vx=前, vy=左, vw=CCW
 *   WORLD 系：vx=世界X, vy=世界Y, vw=CCW
 *
 * NO_YAW 模式下 vw 被忽略，锁定角由 R2_Move_SetRobotLockYaw()
 * 或 R2_Move_SetWorldLockYaw() 指定。
 */
void R2_Move_SetVel(R2_Move_Ctrl_t *ctrl, float vx, float vy, float vw);

/* ── POS 模式接口 ── */

/**
 * @brief 启动一次指定距离的位置运动（POS 模式使用）
 *
 * 位移含义取决于当前坐标系：
 *   ROBOT 系：dx=前移量, dy=左移量, dyaw=CCW旋转量
 *   WORLD 系：dx=世界X位移, dy=世界Y位移, dyaw=CCW旋转量
 *
 * ROBOT 模式在启动机器人系内规划，WORLD 模式直接在世界系规划；
 * 两者执行期间都由位置 P 环闭环。
 *
 * NO_YAW 模式下 dyaw 被忽略，锁定角由 R2_Move_SetRobotLockYaw()
 * 或 R2_Move_SetWorldLockYaw() 指定。
 *
 * @return 0=成功, -1=已在运行中（需等待完成或手动Stop）
 */
int8_t R2_Move_SetDist(R2_Move_Ctrl_t *ctrl, float dx, float dy, float dyaw);

/**
 * @brief 查询位置运动状态
 * @return IDLE / RUNNING / DONE
 */
R2_PosState_t R2_Move_GetPosState(const R2_Move_Ctrl_t *ctrl);

/* ── 通用接口 ── */

/**
 * @brief 更新当前朝向（IMU 读数后用）
 */
void R2_Move_UpdateYaw(R2_Move_Ctrl_t *ctrl, float yaw);

/**
 * @brief 更新里程计（编码器读数后用）
 *
 * robot_dx / robot_dy：本轮机器人系位移增量（由轮速编码器积分得到），
 * 内部旋转到世界系后累加到 odom_x/y。
 *
 * delta_yaw：yaw 角增量 (rad)，两种坐标系下数值相同。
 *
 * 注意：调用方应先调 UpdateYaw() 更新朝向，再调此函数，
 * 确保 robot→world 旋转用最新的 yaw 值。
 */
void R2_Move_UpdateOdom(R2_Move_Ctrl_t *ctrl, float robot_dx,
                        float robot_dy, float delta_yaw);

/**
 * @brief 每控制周期调用一次，计算最终轮速
 * @param ctrl  控制器句柄
 * @param now_sec  当前系统时间 (s)，POS 模式需要
 *
 * VEL 模式：speedPlanner 平滑 → 坐标转换 → Mecanum_Calc
 * POS 模式：S 曲线规划 → 位置 P 环 → 坐标转换 → Mecanum_Calc
 */
void R2_Move_Update(R2_Move_Ctrl_t *ctrl, float now_sec);

/**
 * @brief 急停：立即清零输出，中止所有运动
 */
void R2_Move_Stop(R2_Move_Ctrl_t *ctrl);

/**
 * @brief 设定 ROBOT_NO_YAW 模式下的锁定朝向
 * @param ctrl 控制器句柄
 * @param yaw_rad 机器人系目标 yaw 角 (rad)
 *
 * yaw_rad 按机器人系解释；内部用进入 ROBOT_NO_YAW 时的参考朝向
 * 换算为 IMU 角度闭环目标。
 */
void R2_Move_SetRobotLockYaw(R2_Move_Ctrl_t *ctrl, float yaw_rad);

/**
 * @brief 设定 WORLD_NO_YAW 模式下的锁定朝向
 * @param ctrl 控制器句柄
 * @param yaw_rad 要锁定的绝对 yaw 角 (rad)
 *
 * 调用后 WORLD_NO_YAW_* 模式将用 Chassis_Yaw_World_Frame_Ctrl
 * 闭环维持此角度。
 * 若不显式调用，默认沿用上一次有效目标角；上电初始值为 0 rad。
 */
void R2_Move_SetWorldLockYaw(R2_Move_Ctrl_t *ctrl, float yaw_rad);

/**
 * @brief 解除急停
 */
void R2_Move_Resume(R2_Move_Ctrl_t *ctrl);

/* ── 内联辅助 ───────────────────────────────────────── */

static inline uint8_t R2_Move_IsVelMode(R2_MoveMode_t mode)
{
    return (mode <= R2_MODE_WORLD_VEL) ? 1U : 0U;
}

static inline uint8_t R2_Move_IsPosMode(R2_MoveMode_t mode)
{
    return (mode >= R2_MODE_ROBOT_NO_YAW_POS) ? 1U : 0U;
}

static inline uint8_t R2_Move_IsWorldMode(R2_MoveMode_t mode)
{
    return ((mode == R2_MODE_WORLD_NO_YAW_VEL) ||
            (mode == R2_MODE_WORLD_VEL)        ||
            (mode == R2_MODE_WORLD_NO_YAW_POS) ||
            (mode == R2_MODE_WORLD_POS)) ? 1U : 0U;
}

static inline uint8_t R2_Move_IsNoYawMode(R2_MoveMode_t mode)
{
    return ((mode & 0x01U) == 0U) ? 1U : 0U;  /* 偶数模式 = NO_YAW */
}

#endif /* __R2_MOVE_H */
