#include "arm_ik_3r_safe_stm32h7.h"
#include <math.h>
#include <string.h>

#define ARM3R_WORKSPACE_MARGIN_MM        50.0f
#define ARM3R_WORKSPACE_EPS_MM           1.0e-3f
#define ARM3R_J1_LOCK_PLANE_TOL_MM       5.0f
#define ARM3R_J1_ROTATE_MIN_Z_MM         250.0f

/* 50 mm inward Cartesian envelope, J1 +/-180 deg output clamp ignored. */
#define ARM3R_WS_NEG_RHO_MIN_MM          218.485805f
#define ARM3R_WS_NEG_RHO_MAX_MM          552.543230f
#define ARM3R_WS_NEG_Z_MIN_MM            (-71.821546f)
#define ARM3R_WS_NEG_Z_MAX_MM            352.822094f

#define ARM3R_WS_POS_RHO_MIN_MM          50.0f
#define ARM3R_WS_POS_RHO_MAX_MM          268.782303f
#define ARM3R_WS_POS_Z_MIN_MM            133.457255f
#define ARM3R_WS_POS_Z_MAX_MM            535.889697f

static float arm3r_clamp(float x, float min_val, float max_val)
{
    if (x < min_val)
    {
        return min_val;
    }

    if (x > max_val)
    {
        return max_val;
    }

    return x;
}

static uint8_t arm3r_in_range_mm(float value, float min_val, float max_val)
{
    return ((value >= (min_val - ARM3R_WORKSPACE_EPS_MM)) &&
            (value <= (max_val + ARM3R_WORKSPACE_EPS_MM))) ? 1U : 0U;
}

static uint8_t arm3r_in_workspace_box(float rho,
                                      float z,
                                      float min_rho,
                                      float max_rho,
                                      float min_z,
                                      float max_z)
{
    return (arm3r_in_range_mm(rho, min_rho, max_rho) &&
            arm3r_in_range_mm(z, min_z, max_z)) ? 1U : 0U;
}

static uint8_t Arm3R_CheckWorkspaceMargin(float x,
                                          float y,
                                          float z,
                                          float theta2_model,
                                          Arm3R_UnsafeReason_t *reason)
{
    float rho;
    float t2_deg;
    uint8_t ok;
    uint8_t pos_ok;
    uint8_t neg_ok;

    if (reason != 0)
    {
        *reason = ARM3R_UNSAFE_NONE;
    }

    if (z <= ARM3R_J1_ROTATE_MIN_Z_MM)
    {
        rho = fabsf(x);
    }
    else
    {
        rho = sqrtf((x * x) + (y * y));
    }
    t2_deg = Arm3R_RadToDeg(theta2_model);

    if (t2_deg > 0.0f)
    {
        ok = arm3r_in_workspace_box(rho,
                                    z,
                                    ARM3R_WS_POS_RHO_MIN_MM,
                                    ARM3R_WS_POS_RHO_MAX_MM,
                                    ARM3R_WS_POS_Z_MIN_MM,
                                    ARM3R_WS_POS_Z_MAX_MM);
    }
    else if (t2_deg < 0.0f)
    {
        ok = arm3r_in_workspace_box(rho,
                                    z,
                                    ARM3R_WS_NEG_RHO_MIN_MM,
                                    ARM3R_WS_NEG_RHO_MAX_MM,
                                    ARM3R_WS_NEG_Z_MIN_MM,
                                    ARM3R_WS_NEG_Z_MAX_MM);
    }
    else
    {
        pos_ok = arm3r_in_workspace_box(rho,
                                        z,
                                        ARM3R_WS_POS_RHO_MIN_MM,
                                        ARM3R_WS_POS_RHO_MAX_MM,
                                        ARM3R_WS_POS_Z_MIN_MM,
                                        ARM3R_WS_POS_Z_MAX_MM);
        neg_ok = arm3r_in_workspace_box(rho,
                                        z,
                                        ARM3R_WS_NEG_RHO_MIN_MM,
                                        ARM3R_WS_NEG_RHO_MAX_MM,
                                        ARM3R_WS_NEG_Z_MIN_MM,
                                        ARM3R_WS_NEG_Z_MAX_MM);
        ok = ((pos_ok != 0U) && (neg_ok != 0U)) ? 1U : 0U;
    }

    if (ok == 0U)
    {
        if (reason != 0)
        {
            *reason = ARM3R_UNSAFE_WORKSPACE_MARGIN;
        }
        return 0U;
    }

    return 1U;
}

float Arm3R_DegToRad(float deg)
{
    return deg * ARM3R_PI / 180.0f;
}

float Arm3R_RadToDeg(float rad)
{
    return rad * 180.0f / ARM3R_PI;
}

float Arm3R_NormalizeAngle(float angle_rad)
{
    while (angle_rad >= ARM3R_PI)
    {
        angle_rad -= 2.0f * ARM3R_PI;
    }

    while (angle_rad < -ARM3R_PI)
    {
        angle_rad += 2.0f * ARM3R_PI;
    }

    return angle_rad;
}

void Arm3R_ResetResult(Arm3R_Result_t *result)
{
    if (result == 0)
    {
        return;
    }

    memset(result, 0, sizeof(Arm3R_Result_t));
    result->status = ARM3R_ERR_PARAM;
    result->unsafe_reason = ARM3R_UNSAFE_NONE;
}

void Arm3R_Init(Arm3R_Handle_t *arm, const Arm3R_Config_t *cfg)
{
    if ((arm == 0) || (cfg == 0))
    {
        return;
    }

    memset(arm, 0, sizeof(Arm3R_Handle_t));
    arm->cfg = *cfg;
    Arm3R_ResetResult(&arm->result);
    arm->inited = 1;
}

const Arm3R_Result_t *Arm3R_GetResult(const Arm3R_Handle_t *arm)
{
    if (arm == 0)
    {
        return 0;
    }

    return &arm->result;
}

void Arm3R_ModelToCtrl(float theta1,
                       float theta2,
                       float theta3,
                       const Arm3R_JointRef_t *j1_ref,
                       const Arm3R_JointRef_t *j2_ref,
                       const Arm3R_JointRef_t *j3_ref,
                       float *j1,
                       float *j2,
                       float *j3)
{
    if ((j1_ref == 0) || (j2_ref == 0) || (j3_ref == 0) ||
        (j1 == 0) || (j2 == 0) || (j3 == 0))
    {
        return;
    }

    /*
     * 解耦控制定义：
     * j1 = theta1 - theta1_ref
     * j2 = theta2 - theta2_ref
     * j3 = (theta2 + theta3) - (theta2_ref + theta3_ref)
     *
     * 注意：
     * 这里输出的是“几何意义上的控制角”，
     * 不乘 dir，便于安全判定和人工理解。
     */
    *j1 = Arm3R_NormalizeAngle(theta1 - j1_ref->model_ref);
    *j2 = Arm3R_NormalizeAngle(theta2 - j2_ref->model_ref);
    *j3 = Arm3R_NormalizeAngle((theta2 + theta3) -
                               (j2_ref->model_ref + j3_ref->model_ref));
}

void Arm3R_CtrlToModel(float j1,
                       float j2,
                       float j3,
                       const Arm3R_JointRef_t *j1_ref,
                       const Arm3R_JointRef_t *j2_ref,
                       const Arm3R_JointRef_t *j3_ref,
                       float *theta1,
                       float *theta2,
                       float *theta3)
{
    if ((j1_ref == 0) || (j2_ref == 0) || (j3_ref == 0) ||
        (theta1 == 0) || (theta2 == 0) || (theta3 == 0))
    {
        return;
    }

    *theta1 = Arm3R_NormalizeAngle(j1_ref->model_ref + j1);
    *theta2 = Arm3R_NormalizeAngle(j2_ref->model_ref + j2);
    *theta3 = Arm3R_NormalizeAngle(j3_ref->model_ref + j3 - j2);
}

void Arm3R_FK_Model(float theta1,
                    float theta2,
                    float theta3,
                    float d1,
                    float a2,
                    float a3,
                    float *x,
                    float *y,
                    float *z)
{
    float r;

    if ((x == 0) || (y == 0) || (z == 0))
    {
        return;
    }

    r = -a2 * sinf(theta2) - a3 * sinf(theta2 + theta3);

    *x = cosf(theta1) * r;
    *y = sinf(theta1) * r;
    *z = d1 + a2 * cosf(theta2) + a3 * cosf(theta2 + theta3);
}

uint8_t Arm3R_GetJ3SafeRange(float theta2_model,
                             float *min_j3,
                             float *max_j3)
{
    float t2_deg;

    if ((min_j3 == 0) || (max_j3 == 0))
    {
        return 0;
    }

    t2_deg = Arm3R_RadToDeg(theta2_model);

    /*
     * joint2 模型角总安全范围：
     *   -62 deg <= theta2 <= 80 deg
     */
    if ((t2_deg < -62.0f) || (t2_deg > 80.0f))
    {
        return 0;
    }

    /*
     * 分段联动保护：
     *
     * 1) 0 < theta2 <= 80:
     *    0 <= j3 <= 51.191847
     *
     * 2) -62 <= theta2 < 0:
     *    -63.22950 <= j3 <= 10
     *
     * 3) theta2 == 0:
     *    当前按“特殊分水岭”处理，允许正负两侧整个范围。
     *
     * 注意：
     * 新参数保持原物理规则不变，因此 theta2 分段位置仍为 0。
     * theta2 上界包含新的 80 deg 上电姿态；该上电姿态对应 j3_ctrl = 0 deg。
     * 因此正半区下界保持 0 deg，其余 j3_ctrl 边界按新参考角换算。
     * theta2 == 0 时允许 [-63.22950, 51.191847]。
     */
    if (t2_deg > 0.0f)
    {
        *min_j3 = Arm3R_DegToRad(0.0f);
        *max_j3 = Arm3R_DegToRad(51.191847f);
    }
    else if (t2_deg < 0.0f)
    {
        *min_j3 = Arm3R_DegToRad(-63.22950f);
        *max_j3 = Arm3R_DegToRad(10.0f);
    }
    else
    {
        *min_j3 = Arm3R_DegToRad(-63.22950f);
        *max_j3 = Arm3R_DegToRad(51.191847f);
    }

    return 1;
}

uint8_t Arm3R_CheckSafety(float theta2_model,
                          float j3_ctrl,
                          Arm3R_UnsafeReason_t *reason)
{
    float min_j3;
    float max_j3;

    if (reason != 0)
    {
        *reason = ARM3R_UNSAFE_NONE;
    }

    /* 先检查 theta2 总范围 */
    if (!Arm3R_GetJ3SafeRange(theta2_model, &min_j3, &max_j3))
    {
        if (reason != 0)
        {
            *reason = ARM3R_UNSAFE_THETA2_RANGE;
        }
        return 0;
    }

    /* 再检查 j3 联动范围 */
    if ((j3_ctrl < (min_j3 - ARM3R_EPS)) || (j3_ctrl > (max_j3 + ARM3R_EPS)))
    {
        if (reason != 0)
        {
            *reason = ARM3R_UNSAFE_J3_RANGE;
        }
        return 0;
    }

    return 1;
}

Arm3R_Status_t Arm3R_Solve(Arm3R_Handle_t *arm,
                           float x,
                           float y,
                           float z,
                           float theta1_hint)
{
    float theta1;
    float theta2;
    float theta3;
    float plane_err;

    float r;
    float u;
    float h;
    float D;
    float s3_abs;
    float k1;
    float k2;

    if (arm == 0)
    {
        return ARM3R_ERR_NULL;
    }

    if (arm->inited == 0U)
    {
        Arm3R_ResetResult(&arm->result);
        arm->result.status = ARM3R_ERR_NOT_INIT;
        return ARM3R_ERR_NOT_INIT;
    }

    Arm3R_ResetResult(&arm->result);

    arm->result.req_pt.x = x;
    arm->result.req_pt.y = y;
    arm->result.req_pt.z = z;

    if ((arm->cfg.link.a2 <= ARM3R_EPS) || (arm->cfg.link.a3 <= ARM3R_EPS))
    {
        arm->result.status = ARM3R_ERR_PARAM;
        return ARM3R_ERR_PARAM;
    }

    /* 1. 求 theta1 */
    /*
     * The large arm is mechanically offset 20 deg from the model vertical.
     * At z <= 250 mm, keep motion in the model XZ plane instead of rotating J1.
     */
    if (z <= ARM3R_J1_ROTATE_MIN_Z_MM)
    {
        theta1 = 0.0f;
        plane_err = y;

        if (fabsf(plane_err) > ARM3R_J1_LOCK_PLANE_TOL_MM)
        {
            arm->result.status = ARM3R_ERR_UNSAFE;
            arm->result.reachable = 1U;
            arm->result.safe = 0U;
            arm->result.unsafe_reason = ARM3R_UNSAFE_J1_LOCK_PLANE;
            return ARM3R_ERR_UNSAFE;
        }

        if ((fabsf(x) < ARM3R_EPS) && (fabsf(y) < ARM3R_EPS))
        {
            arm->result.base_singular = 1U;
        }
    }
    else if ((fabsf(x) < ARM3R_EPS) && (fabsf(y) < ARM3R_EPS))
    {
        theta1 = theta1_hint;
        arm->result.base_singular = 1U;
    }
    else
    {
        /*
         * Coordinate convention:
         * +X is robot/front, +Y is left, +Z is up.
         * Rotating from +X toward +Y is positive; rotating from +X toward -Y
         * is negative. atan2f(y, x) gives the required [-180, +180] J1 angle.
         */
        theta1 = atan2f(y, x);
    }
    theta1 = Arm3R_NormalizeAngle(theta1);

    /* 2. 转到 joint2/joint3 工作平面 */
    r = cosf(theta1) * x + sinf(theta1) * y;
    u = -r;
    h = z - arm->cfg.link.d1;

    /* 3. 余弦定理解 theta3 */
    D = (u * u + h * h - arm->cfg.link.a2 * arm->cfg.link.a2 -
         arm->cfg.link.a3 * arm->cfg.link.a3) /
        (2.0f * arm->cfg.link.a2 * arm->cfg.link.a3);

    if ((D < (-1.0f - 1e-5f)) || (D > (1.0f + 1e-5f)))
    {
        arm->result.status = ARM3R_ERR_UNREACHABLE;
        arm->result.reachable = 0U;
        return ARM3R_ERR_UNREACHABLE;
    }

    D = arm3r_clamp(D, -1.0f, 1.0f);
    s3_abs = sqrtf(fmaxf(0.0f, 1.0f - D * D));

    /*
     * 4. 只保留凸型单解
     *
     * 约定：
     *   凸型 <=> theta3 < 0
     */
    theta3 = atan2f(-s3_abs, D);

    /* 5. 求 theta2 */
    k1 = arm->cfg.link.a2 + arm->cfg.link.a3 * cosf(theta3);
    k2 = arm->cfg.link.a3 * sinf(theta3);
    theta2 = atan2f(u, h) - atan2f(k2, k1);

    theta1 = Arm3R_NormalizeAngle(theta1);
    theta2 = Arm3R_NormalizeAngle(theta2);
    theta3 = Arm3R_NormalizeAngle(theta3);

    arm->result.reachable = 1U;

    /* 6. 保存模型角 */
    arm->result.model.theta1 = theta1;
    arm->result.model.theta2 = theta2;
    arm->result.model.theta3 = theta3;
    arm->result.model.valid = 1U;

    /* 7. 计算控制角 */
    Arm3R_ModelToCtrl(theta1,
                      theta2,
                      theta3,
                      &arm->cfg.j1_ref,
                      &arm->cfg.j2_ref,
                      &arm->cfg.j3_ref,
                      &arm->result.ctrl.j1,
                      &arm->result.ctrl.j2,
                      &arm->result.ctrl.j3);
    arm->result.ctrl.valid = 1U;

    /* 8. 安全检查 */
    if (!Arm3R_CheckSafety(theta2, arm->result.ctrl.j3, &arm->result.unsafe_reason))
    {
        arm->result.status = ARM3R_ERR_UNSAFE;
        arm->result.safe = 0U;
        return ARM3R_ERR_UNSAFE;
    }

    if (!Arm3R_CheckWorkspaceMargin(x, y, z, theta2, &arm->result.unsafe_reason))
    {
        arm->result.status = ARM3R_ERR_UNSAFE;
        arm->result.safe = 0U;
        return ARM3R_ERR_UNSAFE;
    }

    arm->result.status = ARM3R_OK;
    arm->result.safe = 1U;
    return ARM3R_OK;
}

void Arm3R_GeomCtrlToMotorCtrl(const Arm3R_CtrlAngles_t *geom_ctrl,
                               const Arm3R_Config_t *cfg,
                               Arm3R_CtrlAngles_t *motor_ctrl)
{
    if ((geom_ctrl == 0) || (cfg == 0) || (motor_ctrl == 0))
    {
        return;
    }

    motor_ctrl->j1 = (float)cfg->j1_ref.dir * geom_ctrl->j1;
    motor_ctrl->j2 = (float)cfg->j2_ref.dir * geom_ctrl->j2;
    motor_ctrl->j3 = (float)cfg->j3_ref.dir * geom_ctrl->j3;
    motor_ctrl->valid = geom_ctrl->valid;
}
