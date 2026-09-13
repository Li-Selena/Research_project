#ifndef __SPEED_PLANNER_H
#define __SPEED_PLANNER_H

#include "stdint.h"

typedef struct{
    float target_v; // m/s for translation, rad/s for yaw
    float current_v;
    float accel;    // m/s^2 or rad/s^2
    float decel;
    float dt;       // seconds
} speedPlanner_t;



extern speedPlanner_t SP_user;
extern float dt_flag;

void speedPlanner_Init(speedPlanner_t *planner, float accel, float decel, float cycle_time);
void speedPlanner_SetTarget(speedPlanner_t *planner, float target_v);
float speedPlanner_Update(speedPlanner_t *planner);




/*
 * S 形速度规划结果。
 * 一次规划产生一条轨迹，可反复调用 EvalPos / EvalVel / EvalAcc
 * 查询归一化时刻的参考值。
 */
typedef struct
{
    float distance;     /* 总位移 (m 或 rad)，含符号 */
    float duration;     /* 规划总时长 (s)，由速度/加速度/jerk 上限三者共同决定 */
    float sign;         /* 位移方向符号，+1 或 -1。零位移时为 0 */
    float v_peak;       /* 峰值速度 (m/s 或 rad/s)，在 t=0.5 处出现 */
    float a_peak;       /* 峰值加速度绝对值 (m/s² 或 rad/s²)，在 t≈0.21 / 0.79 处出现 */
} SCurve_Plan_t;

/*
 * 根据 5 次多项式做 S 形速度规划。
 * 内部按 v_max / a_max / j_max 约束计算最短可行的运动时长，
 * 确保峰值不会超出驱动器或机构的能力。
 *
 * @param plan     输出规划结果
 * @param distance 目标位移（绝对值=距离，符号=方向）
 * @param v_max    最大允许速度 (m/s 或 rad/s)
 * @param a_max    最大允许加速度 (m/s² 或 rad/s²)
 * @param j_max    最大允许 jerk (m/s³ 或 rad/s³)
 */
void SCurve_Plan(SCurve_Plan_t *plan, float distance,
                 float v_max, float a_max, float j_max);

/*
 * 查询归一化时刻 t_norm ∈ [0, 1] 处的参考速度。
 * 内部用因式分解后的公式，比直接算多项式快且数值更稳。
 *
 * @return 参考速度 (m/s 或 rad/s)，已含方向符号
 */
float SCurve_EvalVel(const SCurve_Plan_t *plan, float t_norm);

/*
 * 查询归一化时刻 t_norm ∈ [0, 1] 处的参考位置。
 * t=0 → 0, t=1 → distance。
 *
 * @return 当前应到达的位置 (m 或 rad)，已含方向符号
 */
float SCurve_EvalPos(const SCurve_Plan_t *plan, float t_norm);

/*
 * 查询归一化时刻 t_norm ∈ [0, 1] 处的参考加速度。
 *
 * @return 参考加速度 (m/s² 或 rad/s²)，已含方向符号
 */
float SCurve_EvalAcc(const SCurve_Plan_t *plan, float t_norm);

/* 检查轨迹是否已完成（t_norm >= 1.0 即视为到达终点） */
static inline uint8_t SCurve_IsDone(const SCurve_Plan_t *plan, float t_norm)
{
    (void)plan;
    return (t_norm >= 1.0f) ? 1U : 0U;
}


#endif
