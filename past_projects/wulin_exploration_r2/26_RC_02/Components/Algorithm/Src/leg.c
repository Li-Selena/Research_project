#include "leg.h"
#include "math.h"
#include "struct_typedef.h"
#define PI 3.1415926

//腿电机角度 rad
leg lf_leg = {0,0};
leg rf_leg = {0,0};

float  lf_last_theta1 = LEGINIT_OFFSET;
float  lf_last_theta2 = LEGINIT_OFFSET;
float  rf_last_theta1 = LEGINIT_OFFSET;
float  rf_last_theta2 = LEGINIT_OFFSET;

float lb_leg = 0;
float rb_leg = 0;

// 旧二连杆平面模型参数，单位 mm；不参与当前四立杆爬阶控制。
float L1 = 94.5f;
float L2 = 112.5f;


// 旧平面腿部控制状态。


// 旧高度命令标志。
int8_t LEG_Cmd = 0;
float bleg_theta = 0;
float fleg_high = 0;

// 角度归一化与连续分支辅助函数。
static float WrapToPi(float angle);
static float UnwrapNear(float angle, float ref);
int InverseKinematics_Continuous(float x, float y,
                                 float last_theta1, float last_theta2,
                                 float *theta1, float *theta2);

// int leg_control_UpdateTarget(float legx,float legy,float legh)
// {
//     	int ret_lf, ret_rf;

//         /* 左前腿：逆解输出模型角到 lf_leg */
//         ret_lf = InverseKinematics_Continuous(legx, legy,
//                                       lf_last_theta1, lf_last_theta2,
//                                       &lf_leg.theta1, &lf_leg.theta2);

//         /* 右前腿：逆解输出模型角到 rf_leg */
//         ret_rf = InverseKinematics_Continuous(legx, legy,
//                                       rf_last_theta1, rf_last_theta2,
//                                       &rf_leg.theta1, &rf_leg.theta2);

//         /* 左前腿 */
//         if (ret_lf)
//         {
//             /* 保存本次模型角，供下一次最小角度连续化使用 */
//             lf_last_theta1 = lf_leg.theta1;
//             lf_last_theta2 = lf_leg.theta2;

//             /* 模型角 -> 控制角 */
//             lf_leg.theta1 = -lf_last_theta1 + LEGINIT_OFFSET;
//             lf_leg.theta2 =  lf_last_theta2 - LEGINIT_OFFSET;
//         }

//         /* 右前腿 */
//         if (ret_rf)
//         {
//             /* 保存本次模型角，供下一次最小角度连续化使用 */
//             rf_last_theta1 = rf_leg.theta1;
//             rf_last_theta2 = rf_leg.theta2;

//             /* 模型角 -> 控制角 */
//             rf_leg.theta1 =  rf_last_theta1 - LEGINIT_OFFSET;
//             rf_leg.theta2 = -rf_last_theta2 + LEGINIT_OFFSET;
//         }
//         lb_leg = -4.0f * legh * 3.14159f/180.0f;
// 	    rb_leg =  4.0f * legh * 3.14159f/180.0f;
// }




// 旧二连杆平面 IK；局部 x/y 是连杆运动平面的坐标，不是整车坐标。
// 逆运动学解算
int InverseKinematics_Continuous(float x, float y,
                                 float last_theta1, float last_theta2,
                                 float *theta1, float *theta2)
{
    float L_sq = x * x + y * y;
    float L;
    float numerator, denominator;
    float cos_beta, beta;
    float beta_limit;
    float phi;

    /* 0. 空指针保护 */
    if (theta1 == 0 || theta2 == 0)
        return 0;

    /* 1. 防止原点奇点 (x=0, y=0 时无法确定中心对称轴) */
    if (L_sq < 1e-6f)
        return 0;

    L = sqrtf(L_sq);

    /* ================= 2. 核心限制：中心禁区 (防碰撞/奇异区) ================= */
    const float LIMIT_RADIUS = 100.0f;
    if (L < LIMIT_RADIUS)
    {
        // 若目标闯入禁区，利用相似三角形原理，将其强制推到 R=100 的圆弧边界上
        float scale = LIMIT_RADIUS / L;
        x = x * scale;
        y = y * scale;
        L = LIMIT_RADIUS;
        L_sq = LIMIT_RADIUS * LIMIT_RADIUS;
    }
    /* ========================================================================= */

    /* 3. 最大臂展越界保护 (防止 L > L1+L2 导致结构拉断) */
    if (L > (L1 + L2) || L < fabsf(L1 - L2))
        return 0;

    /* 4. 建立并联三角形几何模型 */
    phi = atan2f(y, x); // 目标点中心连线与 X 轴的夹角

    // 利用余弦定理，计算驱动臂(L1)与中心连线(L)的夹角 beta
    numerator   = L_sq + L1 * L1 - L2 * L2;
    denominator = 2.0f * L1 * L;
    cos_beta = numerator / denominator;

    // 浮点精度保护，防止 acosf 出现 NaN
    if (cos_beta > 1.0f) cos_beta = 1.0f;
    if (cos_beta < -1.0f) cos_beta = -1.0f;
    beta = acosf(cos_beta);

    /* ================= 5. 核心限制：机械死区 (防自干涉卡死) ================= */
    // beta 越小，两个驱动臂靠得越近。限制 beta 的最小值，防止双臂“剪刀式”碰撞
    beta_limit = 65.0f * 0.5f * PI / 180.0f; // 即 32.5 度 (0.567 rad)
    if (beta < beta_limit)
    {
        beta = beta_limit;
    }
    /* ========================================================================= */

    /* 6. 计算两个独立驱动臂的模型角度
       前臂：中心角向下/上偏 beta
       后臂：中心角向上/下偏 beta (映射到 PI-x 的硬件坐标系) */
    float t1 = phi - beta;
    float t2 = PI - (phi + beta);

    /* ================= 7. 核心限制：连续化处理 (防飞车/防断点跳变) ================= */
    // 并联机构绝对不能在 -PI 和 PI 之间来回跳变，否则会撕裂连杆
    t1 = UnwrapNear(t1, last_theta1);
    t2 = UnwrapNear(t2, last_theta2);
    /* =============================================================================== */

    /* 8. 输出最终安全角度 */
    *theta1 = t1;
    *theta2 = t2;

    return 1;
}

static float WrapToPi(float angle)
{
    while (angle > PI)
        angle -= 2.0f * PI;

    while (angle <= -PI)
        angle += 2.0f * PI;

    return angle;
}

static float UnwrapNear(float angle, float ref)
{
    angle = WrapToPi(angle);

    while ((angle - ref) > PI)
        angle -= 2.0f * PI;

    while ((angle - ref) <= -PI)
        angle += 2.0f * PI;

    return angle;
}

//快速求根（可选）
float Q_rsqrt(float number) {
    long i;
    float x2, y;
    const float threehalfs = 1.5f;
    x2 = number * 0.5f;
    y = number;
    i = *(long*)&y;
    i = 0x5F3759DF - (i >> 1);
    y = *(float*)&i;
    y = y * (threehalfs - (x2 * y * y)); // 牛顿迭代修正倒数平方根估计。
    return y;
}

void backLeg_Left(float h,float*theta)// 旧后左腿高度接口，当前无动作实现。
{
	// 占位接口：尚未使用高度参数生成目标。

	// 高度到角度转换尚未实现。

	// 尚未写入电机输出。
}

void backLeg_Right(float h,float*theta)// 旧后右腿高度接口，当前无动作实现。
{
	// 占位接口：尚未使用高度参数生成目标。

	// 高度到角度转换尚未实现。

	// 尚未写入电机输出。
}