#include "speedPlanner.h"
#include <math.h>
#include <stddef.h>



speedPlanner_t SP_user;
float dt_flag = 0;


void speedPlanner_Init(speedPlanner_t *planner, float accel, float decel, float cycle_time)
{
    planner->accel = accel;
    planner->decel = decel;
    planner->dt = cycle_time;
    planner->target_v = 0.0f;
    planner->current_v = 0.0f;
}
void speedPlanner_SetTarget(speedPlanner_t *planner, float target_v)
{
    planner->target_v = target_v;
}

float speedPlanner_Update(speedPlanner_t *planner)
{
    float step;

    // 计算目标差
    float error = planner->target_v - planner->current_v;

    // 误差极小 → 直接等于目标
    if (fabsf(error) < 0.001f) {
        planner->current_v = planner->target_v;
        return planner->current_v;
    }

    // 正确加减速判断
    if (error > 0) {
        step = planner->accel * planner->dt;
    } else {
        step = planner->decel * planner->dt;
    }

    // 梯度逼近
    if (error > 0) {
        planner->current_v += step;
        if (planner->current_v > planner->target_v)
            planner->current_v = planner->target_v;
    } else {
        planner->current_v -= step;
        if (planner->current_v < planner->target_v)
            planner->current_v = planner->target_v;
    }

    return planner->current_v;
}


/*
 * ─── 5 次多项式 S 形速度规划 ─────────────────────────
 *
 * 选用 quintic polynomial 作为轨迹基函数，
 * 边界条件满足零速零加速度启停，位移从 0 平滑过渡到 S：
 *
 *   位移 p(t) = S * ( 6t⁵ - 15t⁴ + 10t³ )
 *   速度 v(t) = S * ( 30t⁴ - 60t³ + 30t² )
 *   加速度 a(t) = S * ( 120t³ - 180t² + 60t )
 *
 * 其中 S = |distance|，t ∈ [0, 1] 为归一化时间。
 *
 * ─── 峰值约束（反推最短可行时长 T） ─────────────────
 *
 * 多项式各阶导数的极值均可用解析式表达，
 * 给定物理上限即可解出满足约束的最短 T：
 *
 *   v_peak = 1.875  × S / T      出现在 t = 0.5
 *   a_peak = 5.7735 × S / T²    出现在 t ≈ 0.2113, 0.7887
 *   j_peak = 60     × S / T³    出现在 t = 0, 1 端点
 *
 *   → T = max( 1.875·S / v_max,
 *              √(5.7735·S / a_max),
 *              ∛(60·S / j_max) )
 *
 * 取 max 保证三个约束同时满足，
 * 即最终轨迹不会超过任意一个物理上限。
 */

/*
 * 一次规划调用，计算出满足 v/a/j 约束的最短运动时长 T，
 * 并回填峰值速度/加速度供监控用。
 */
void SCurve_Plan(SCurve_Plan_t *plan, float distance,
                 float v_max, float a_max, float j_max)
{
    float abs_dist;
    float T_v, T_a, T_j;
    float duration;

    if (plan == NULL) return;

    abs_dist = fabsf(distance);

    /* 位移量级太小（亚微米 / 亚微弧度），直接置零避免数值问题 */
    if (abs_dist < 1e-6f) {
        plan->distance = 0.0f;
        plan->sign     = 0.0f;
        plan->duration = 0.0f;
        plan->v_peak   = 0.0f;
        plan->a_peak   = 0.0f;
        return;
    }

    /* 分别按速度、加速度、jerk 约束计算各自需要的时长 */
    T_v = (v_max > 0.0f) ? (1.875f * abs_dist / v_max) : 0.0f;
    T_a = (a_max > 0.0f) ? sqrtf(5.7735f * abs_dist / a_max) : 0.0f;
    T_j = (j_max > 0.0f) ? powf(60.0f * abs_dist / j_max, 1.0f / 3.0f) : 0.0f;

    /* 取最严格的约束（最长时长） */
    duration = T_v;
    if (T_a > duration) duration = T_a;
    if (T_j > duration) duration = T_j;

    /* 下限钳位 1 ms，防止后续除法出现除零或极大值 */
    if (duration < 0.001f) duration = 0.001f;

    plan->distance = distance;
    plan->sign     = (distance >= 0.0f) ? 1.0f : -1.0f;
    plan->duration = duration;
    /* 反算实际峰值，供外部监控（不会超出给定上限） */
    plan->v_peak   = 1.875f * abs_dist / duration;
    plan->a_peak   = 5.7735f * abs_dist / (duration * duration);
}

/*
 * 参考速度查询。
 * 使用因式分解形式 v = 30·S·t²·(1-t)² / T，比直接算四次多项式更稳定。
 */
float SCurve_EvalVel(const SCurve_Plan_t *plan, float t_norm)
{
    float t, abs_dist;

    if (plan == NULL) return 0.0f;
    /* 端点速度均为零 */
    if (t_norm <= 0.0f) return 0.0f;
    if (t_norm >= 1.0f) return 0.0f;

    t = t_norm;
    abs_dist = (plan->distance >= 0.0f) ? plan->distance : -plan->distance;

    /* v(t) = 30 * S * t² * (1-t)² / T */
    return plan->sign * (30.0f * abs_dist * t * t * (1.0f - t) * (1.0f - t) / plan->duration);
}

/*
 * 参考位置查询。
 * 使用 Horner 化简 p = S·t³·(6t² - 15t + 10)，省一次乘法。
 */
float SCurve_EvalPos(const SCurve_Plan_t *plan, float t_norm)
{
    float t, abs_dist;

    if (plan == NULL) return 0.0f;
    /* 起点为零，终点为总位移 */
    if (t_norm <= 0.0f) return 0.0f;
    if (t_norm >= 1.0f) return plan->distance;

    t = t_norm;
    abs_dist = (plan->distance >= 0.0f) ? plan->distance : -plan->distance;

    /* p(t) = S * t³ * (6t² - 15t + 10) */
    return plan->sign * (abs_dist * t * t * t * (6.0f * t * t - 15.0f * t + 10.0f));
}

/*
 * 参考加速度查询。
 * 使用因式分解形式 a = 60·S·t·(2t-1)·(t-1) / T²。
 * 加速度在 t=0.5 处过零，将轨迹分为加速段和减速段。
 */
float SCurve_EvalAcc(const SCurve_Plan_t *plan, float t_norm)
{
    float t, abs_dist;

    if (plan == NULL) return 0.0f;
    /* 端点加速度均为零 */
    if (t_norm <= 0.0f) return 0.0f;
    if (t_norm >= 1.0f) return 0.0f;

    t = t_norm;
    abs_dist = (plan->distance >= 0.0f) ? plan->distance : -plan->distance;

    /* a(t) = 60 * S * t * (2t-1) * (t-1) / T² */
    return plan->sign * (60.0f * abs_dist * t * (2.0f * t - 1.0f) * (t - 1.0f)
                         / (plan->duration * plan->duration));
}


// void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
// {
//   if (htim->Instance == TIM3) {

//     speedPlanner_Update(&SP_user);
//     dt_flag ++;

//   }
// }

