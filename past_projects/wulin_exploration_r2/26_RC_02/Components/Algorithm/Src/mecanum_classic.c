#include "mecanum_classic.h"
//#include "pid_user.h"

ChassisVel_t total_vel = {0,0,0};
WheelSpeed_t total_speed = {0,0,0,0};

MecanumParam_t mecParam = {
    .wheel_radius = MEC_R,       // Shared radius used for RPM and encoder conversion (m).

    .L = 0.338f,                  // Chassis center to front/rear axle (m).
    .W = 0.375f,                  // Half of left/right wheel spacing (m).

    .max_wheel_speed = MEC_DEBUG_WHEEL_LIMIT_MPS  /* debug limit; physical max is MEC_WHEEL_PHYSICAL_MAX_MPS */
};
static float abs_f(float x)
{
    return (x >= 0.0f) ? x : -x;
}

static float max_f(float a, float b)
{
    return (a > b) ? a : b;
}

/**
 * @brief 麦克纳姆轮底盘运动学解算（长方形）
 */
void Mecanum_Calc(
    const ChassisVel_t *chassis,
    const MecanumParam_t *param,
    WheelSpeed_t *wheel)
{
    /* Body frame: X forward, Y left, yaw CCW; wheel speeds positive forward.
     * FR/BR motor installation reversal is applied only in CAN_Task.
     */
    float k = (param->L + param->W) * chassis->vw;
    float v_fl = chassis->vx - chassis->vy - k;
    float v_fr = chassis->vx + chassis->vy + k;
    float v_bl = chassis->vx + chassis->vy - k;
    float v_br = chassis->vx - chassis->vy + k;

    // 3. 找出当前计算出的最大轮线速度绝对值
    float max_val = 0.0f;
    max_val = max_f(max_val, abs_f(v_fl));
    max_val = max_f(max_val, abs_f(v_fr));
    max_val = max_f(max_val, abs_f(v_bl));
    max_val = max_f(max_val, abs_f(v_br));

    // 4. 速度归一化限速保护
    // 如果最大的轮子线速度超出了最大限速，则等比例缩小所有轮子的速度，确保行驶轨迹不走歪
    if (max_val > param->max_wheel_speed)
    {
        float scale = param->max_wheel_speed / max_val;
        v_fl *= scale;
        v_fr *= scale;
        v_bl *= scale;
        v_br *= scale;
    }

    /* Return wheel linear speeds (m/s); downstream PID converts to RPM. */
    wheel->fl = v_fl;
    wheel->fr = v_fr;
    wheel->bl = v_bl;
    wheel->br = v_br;
}
