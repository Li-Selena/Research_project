#include <string.h>
#include <math.h>
#include "arm_user.h"
#include "bsp_usb.h"
#include "arm_ik_3r_safe_stm32h7.h"

/* 机械臂应用层状态。 */
/* 三关节运动学求解器句柄。 */
Arm3R_Handle_t g_arm_ik;

/* 目标接受状态与完整诊断快照。 */
ArmIK_AppState_t g_arm_ik_app;
ArmIK_FullState_t g_arm_ik_full_state;

static ArmIK_MotorDeg_t s_arm_actual_motor_deg;




/* 内部辅助接口。 */
/* 私有状态管理、角度转换与结果回传函数。 */
static void ArmIK_ResetAppState(void);
static void ArmIK_SendResultToUSB(uint8_t cmd, const uint8_t *data, uint16_t len);
static void ArmIK_SendResultToUART10(uint8_t cmd, const uint8_t *data, uint16_t len);
static void ArmIK_SendResultToAll(uint8_t status_code,
                                  uint8_t unsafe_reason,
                                  uint8_t action_code,
                                  const Arm3R_Result_t *res);
static void ArmIK_MotorCtrlRadToDeg(const Arm3R_CtrlAngles_t *motor_ctrl_rad,
                                    ArmIK_MotorDeg_t *motor_ctrl_deg);
static void ArmIK_ModelRadToDeg(const Arm3R_ModelAngles_t *model_rad,
                                ArmIK_ModelDeg_t *model_deg);
static float ArmIK_ClampJ1Deg(float deg);
static void ArmIK_ApplyJ1CoordinateLimit(ArmIK_MotorDeg_t *motor_ctrl_deg);
static void ArmIK_BuildFullState(ArmIK_FullState_t *state);
static void ArmIK_UpdateFullState(void);
static void ArmIK_SaveAsLastValidTarget(float x,
                                        float y,
                                        float z,
                                        const Arm3R_Result_t *res,
                                        const Arm3R_CtrlAngles_t *motor_ctrl_rad,
                                        const ArmIK_MotorDeg_t *motor_ctrl_deg);
static uint8_t ArmIK_HandleInvalidTarget(void);

/* 内部辅助函数实现。 */
/* 初始化时清空应用状态、实测关节角和诊断快照。 */
static void ArmIK_ResetAppState(void)
{
    memset(&g_arm_ik_app, 0, sizeof(g_arm_ik_app));
    memset(&s_arm_actual_motor_deg, 0, sizeof(s_arm_actual_motor_deg));
    ArmIK_UpdateFullState();
}

/* 使用统一 USB 帧发送接口回传 IK 结果。 */
static void ArmIK_SendResultToUSB(uint8_t cmd, const uint8_t *data, uint16_t len)
{
    Send_Cmd_Data(cmd, (uint8_t *)data, len);
}

/* UART10 IK 结果发送扩展点；当前为空实现，未发送该 6 字节结果帧。 */
static void ArmIK_SendResultToUART10(uint8_t cmd, const uint8_t *data, uint16_t len)
{
    (void)cmd;
    (void)data;
    (void)len;

    /* 尚未连接 UART10 发送实现；不能据此认定双通道结果已回传。 */
    /* Uart10_Send_Cmd_Data(cmd, (uint8_t *)data, len); */
}

/* 构造 6 字节 IK 状态；USB 已接通，UART10 扩展点当前不发送。 */
static void ArmIK_SendResultToAll(uint8_t status_code,
                                  uint8_t unsafe_reason,
                                  uint8_t action_code,
                                  const Arm3R_Result_t *res)
{
    uint8_t tx_data[6];

    tx_data[0] = status_code;
    tx_data[1] = unsafe_reason;
    tx_data[2] = action_code;

    if (res != 0)
    {
        tx_data[3] = (uint8_t)res->reachable;
        tx_data[4] = (uint8_t)res->safe;
    }
    else
    {
        tx_data[3] = 0U;
        tx_data[4] = 0U;
    }

    tx_data[5] = g_arm_ik_app.has_last_valid;

    /* 使用 USB_CMD_ARM_IK_RESULT 命令字。 */
    ArmIK_SendResultToUSB(USB_CMD_ARM_IK_RESULT, tx_data, sizeof(tx_data));
    ArmIK_SendResultToUART10(USB_CMD_ARM_IK_RESULT, tx_data, sizeof(tx_data));
}

/* 电机控制角由弧度换为度；方向映射已经由 GeomCtrlToMotorCtrl 完成。 */
static void ArmIK_MotorCtrlRadToDeg(const Arm3R_CtrlAngles_t *motor_ctrl_rad,
                                    ArmIK_MotorDeg_t *motor_ctrl_deg)
{
    if ((motor_ctrl_rad == 0) || (motor_ctrl_deg == 0))
    {
        return;
    }

    motor_ctrl_deg->j1_deg = Arm3R_RadToDeg(motor_ctrl_rad->j1);
    motor_ctrl_deg->j2_deg = Arm3R_RadToDeg(motor_ctrl_rad->j2);
    motor_ctrl_deg->j3_deg = Arm3R_RadToDeg(motor_ctrl_rad->j3);
    motor_ctrl_deg->valid  = motor_ctrl_rad->valid;
}

static void ArmIK_ModelRadToDeg(const Arm3R_ModelAngles_t *model_rad,
                                ArmIK_ModelDeg_t *model_deg)
{
    if ((model_rad == 0) || (model_deg == 0))
    {
        return;
    }

    model_deg->theta1_deg = Arm3R_RadToDeg(model_rad->theta1);
    model_deg->theta2_deg = Arm3R_RadToDeg(model_rad->theta2);
    model_deg->theta3_deg = Arm3R_RadToDeg(model_rad->theta3);
    model_deg->valid = model_rad->valid;
}

static void ArmIK_BuildFullState(ArmIK_FullState_t *state)
{
    const Arm3R_Result_t *res;

    if (state == 0)
    {
        return;
    }

    memset(state, 0, sizeof(*state));

    res = &g_arm_ik.result;

    state->cfg = g_arm_ik.cfg;
    state->inited = g_arm_ik.inited;
    state->has_last_valid = g_arm_ik_app.has_last_valid;
    state->reachable = res->reachable;
    state->safe = res->safe;
    state->base_singular = res->base_singular;
    state->last_status_code = g_arm_ik_app.last_status_code;
    state->last_action_code = g_arm_ik_app.last_action_code;
    state->actual_motor_valid = s_arm_actual_motor_deg.valid;
    state->solve_status = res->status;
    state->unsafe_reason = res->unsafe_reason;

    state->requested_pt = g_arm_ik_app.last_req_pt;
    state->solved_req_pt = res->req_pt;
    state->last_valid_pt = g_arm_ik_app.last_valid_pt;

    state->solved_model_rad = res->model;
    ArmIK_ModelRadToDeg(&state->solved_model_rad, &state->solved_model_deg);

    state->solved_geom_rad = res->ctrl;
    ArmIK_MotorCtrlRadToDeg(&state->solved_geom_rad, &state->solved_geom_deg);

    state->active_model_rad = g_arm_ik_app.active_model;
    ArmIK_ModelRadToDeg(&state->active_model_rad, &state->active_model_deg);

    if (g_arm_ik_app.has_last_valid != 0U)
    {
        state->active_geom_rad = g_arm_ik_app.last_valid_geom;
        state->active_motor_rad = g_arm_ik_app.last_valid_motor;
    }

    ArmIK_MotorCtrlRadToDeg(&state->active_geom_rad, &state->active_geom_deg);

    state->active_motor_deg = g_arm_ik_app.active_motor_deg;
    state->last_valid_model_rad = g_arm_ik_app.last_valid_model;
    ArmIK_ModelRadToDeg(&state->last_valid_model_rad, &state->last_valid_model_deg);

    state->last_valid_geom_rad = g_arm_ik_app.last_valid_geom;
    ArmIK_MotorCtrlRadToDeg(&state->last_valid_geom_rad, &state->last_valid_geom_deg);

    state->last_valid_motor_rad = g_arm_ik_app.last_valid_motor;
    state->last_valid_motor_deg = g_arm_ik_app.last_valid_motor_deg;
    state->actual_motor_deg = s_arm_actual_motor_deg;
}

static void ArmIK_UpdateFullState(void)
{
    ArmIK_BuildFullState(&g_arm_ik_full_state);
}

static float ArmIK_ClampJ1Deg(float deg)
{
    if (deg > ARM_IK_J1_LIMIT_DEG)
    {
        return ARM_IK_J1_LIMIT_DEG;
    }

    if (deg < -ARM_IK_J1_LIMIT_DEG)
    {
        return -ARM_IK_J1_LIMIT_DEG;
    }

    return deg;
}

static void ArmIK_ApplyJ1CoordinateLimit(ArmIK_MotorDeg_t *motor_ctrl_deg)
{
    float ref_j1;

    if (motor_ctrl_deg == 0)
    {
        return;
    }

    ref_j1 = (g_arm_ik_app.has_last_valid != 0U) ?
             g_arm_ik_app.active_motor_deg.j1_deg :
             0.0f;

    /*
     * XY -> J1 uses atan2(y, x):
     * +Y is positive, -Y is negative. Only the exact rear axis is ambiguous.
     * Keep that +/-180 boundary on the previous side to avoid a sign flip.
     */
    if ((motor_ctrl_deg->j1_deg >= (ARM_IK_J1_LIMIT_DEG - 1.0e-3f)) &&
        (ref_j1 < 0.0f))
    {
        motor_ctrl_deg->j1_deg = -ARM_IK_J1_LIMIT_DEG;
    }
    else if ((motor_ctrl_deg->j1_deg <= (-ARM_IK_J1_LIMIT_DEG + 1.0e-3f)) &&
             (ref_j1 > 0.0f))
    {
        motor_ctrl_deg->j1_deg = ARM_IK_J1_LIMIT_DEG;
    }
    else
    {
        motor_ctrl_deg->j1_deg = ArmIK_ClampJ1Deg(motor_ctrl_deg->j1_deg);
    }
}

/* 保存通过可达性、安全区与电机角转换检查的目标。 */
static void ArmIK_SaveAsLastValidTarget(float x,
                                        float y,
                                        float z,
                                        const Arm3R_Result_t *res,
                                        const Arm3R_CtrlAngles_t *motor_ctrl_rad,
                                        const ArmIK_MotorDeg_t *motor_ctrl_deg)
{
    if ((res == 0) || (motor_ctrl_rad == 0) || (motor_ctrl_deg == 0))
    {
        return;
    }

    /* 保存最近有效的末端坐标，单位 mm。 */
    g_arm_ik_app.last_valid_pt.x = x;
    g_arm_ik_app.last_valid_pt.y = y;
    g_arm_ik_app.last_valid_pt.z = z;

    g_arm_ik_app.last_valid_model = res->model;   /* 模型关节角，单位 rad。 */

    /* 保存几何控制角与经过方向映射的电机控制角。 */
    g_arm_ik_app.last_valid_geom = res->ctrl;
    g_arm_ik_app.last_valid_motor = *motor_ctrl_rad;
    g_arm_ik_app.last_valid_motor_deg = *motor_ctrl_deg;

    g_arm_ik_app.active_model = res->model;       /* 当前保持的模型关节角，单位 rad。 */

    /* 同步当前电机目标，单位 deg。 */
    g_arm_ik_app.active_motor_deg = *motor_ctrl_deg;

    g_arm_ik_app.has_last_valid = 1U;
}

/* 无效目标有历史安全解则保持历史；无历史解则保留当前输出。 */
static uint8_t ArmIK_HandleInvalidTarget(void)
{
    if (g_arm_ik_app.has_last_valid != 0U)
    {
        /* 恢复上一组安全目标。 */
        g_arm_ik_app.active_model = g_arm_ik_app.last_valid_model;        // 模型关节角，rad。
        g_arm_ik_app.active_motor_deg = g_arm_ik_app.last_valid_motor_deg;// 电机控制角，deg。
        return ARM_IK_ACTION_HOLD_LAST;
    }

    /* 没有历史安全目标时不构造新的运动目标。 */
    return ARM_IK_ACTION_KEEP_CURRENT;
}

/* 公共应用接口。 */
/* 初始化三关节模型：d1=0、a2=a3=320 mm，参考角 0/80/-165 deg。 */
void ArmIK_ComponentInit(void)
{
    Arm3R_Config_t cfg;

    /* 连杆参数，单位 mm。 */
    cfg.link.d1 = 0.0f;
    cfg.link.a2 = 320.0f;
    cfg.link.a3 = 320.0f;

    /* 编码器上电零位对应的模型角，转换为 rad 保存。 */
    cfg.j1_ref.model_ref = Arm3R_DegToRad(0.0f);
    cfg.j2_ref.model_ref = Arm3R_DegToRad(80.0f);
    cfg.j3_ref.model_ref = Arm3R_DegToRad(-165.0f);

    /* 几何控制角到电机控制角的方向映射。 */
    cfg.j1_ref.dir = +1;
    cfg.j2_ref.dir = -1;
    cfg.j3_ref.dir = +1;

    /* 初始化运动学与安全检查器。 */
    Arm3R_Init(&g_arm_ik, &cfg);

    /* 重置应用状态。 */
    ArmIK_ResetAppState();
}

uint8_t ArmIK_TargetInputAllowed(float x,
                                 float y,
                                 float z,
                                 Arm3R_Status_t *status,
                                 Arm3R_UnsafeReason_t *unsafe_reason)
{
    Arm3R_Handle_t check_arm;
    Arm3R_Status_t check_status;

    if (status != 0)
    {
        *status = ARM3R_ERR_PARAM;
    }

    if (unsafe_reason != 0)
    {
        *unsafe_reason = ARM3R_UNSAFE_NONE;
    }

    if ((!isfinite(x)) || (!isfinite(y)) || (!isfinite(z)))
    {
        return 0U;
    }

    if (g_arm_ik.inited == 0U)
    {
        if (status != 0)
        {
            *status = ARM3R_ERR_NOT_INIT;
        }
        return 0U;
    }

    check_arm = g_arm_ik;
    check_status = Arm3R_Solve(&check_arm, x, y, z, 0.0f);

    if (status != 0)
    {
        *status = check_status;
    }

    if (unsafe_reason != 0)
    {
        *unsafe_reason = check_arm.result.unsafe_reason;
    }

    return (check_status == ARM3R_OK) ? 1U : 0U;
}

/* 接受统一机械臂基座坐标系目标（X 前、Y 左、Z 上，mm），求解并报告结果。 */
void ArmIK_ComponentStep(float x, float y, float z)
{
    Arm3R_Status_t ret;
    const Arm3R_Result_t *res;

    /* 经过电机方向映射的控制角，单位 rad。 */
    Arm3R_CtrlAngles_t motor_ctrl_rad;

    /* 最终电机角目标，单位 deg。 */
    ArmIK_MotorDeg_t motor_ctrl_deg;

    uint8_t action_code;

    /* 记录本次请求，失败时仍可从诊断状态读取。 */
    g_arm_ik_app.last_req_pt.x = x;
    g_arm_ik_app.last_req_pt.y = y;
    g_arm_ik_app.last_req_pt.z = z;

    if ((!isfinite(x)) || (!isfinite(y)) || (!isfinite(z)))
    {
        action_code = ArmIK_HandleInvalidTarget();
        g_arm_ik_app.last_status_code = ARM_IK_RESULT_PARAM_ERR;
        g_arm_ik_app.last_action_code = action_code;
        ArmIK_UpdateFullState();
        ArmIK_SendResultToAll(ARM_IK_RESULT_PARAM_ERR, 0U, action_code, 0);
        return;
    }

    /* 求解目标点的逆运动学并进行安全检查。 */
    ret = Arm3R_Solve(&g_arm_ik, x, y, z, 0.0f);

    /* 读取此次求解结果。 */
    res = Arm3R_GetResult(&g_arm_ik);

    if (ret == ARM3R_OK)
    {
        /* 安全可达：转换为电机角、检查 J1 边界并更新目标。 */
        Arm3R_GeomCtrlToMotorCtrl(&res->ctrl, &g_arm_ik.cfg, &motor_ctrl_rad);
        ArmIK_MotorCtrlRadToDeg(&motor_ctrl_rad, &motor_ctrl_deg);
        ArmIK_ApplyJ1CoordinateLimit(&motor_ctrl_deg);
        motor_ctrl_rad.j1 = Arm3R_DegToRad(motor_ctrl_deg.j1_deg);

        ArmIK_SaveAsLastValidTarget(x, y, z, res, &motor_ctrl_rad, &motor_ctrl_deg);

        action_code = ARM_IK_ACTION_APPLY_NEW;
        g_arm_ik_app.last_status_code = ARM_IK_RESULT_OK;
        g_arm_ik_app.last_action_code = action_code;
        ArmIK_UpdateFullState();
        ArmIK_SendResultToAll(ARM_IK_RESULT_OK, 0U, action_code, res);
    }
    else if (ret == ARM3R_ERR_UNREACHABLE)
    {
        /* 几何不可达：保留已有安全目标并回传错误。 */
        action_code = ArmIK_HandleInvalidTarget();
        g_arm_ik_app.last_status_code = ARM_IK_RESULT_UNREACHABLE;
        g_arm_ik_app.last_action_code = action_code;
        ArmIK_UpdateFullState();
        ArmIK_SendResultToAll(ARM_IK_RESULT_UNREACHABLE, 0U, action_code, res);
    }
    else if (ret == ARM3R_ERR_UNSAFE)
    {
        /* 目标违反安全限制：回传具体 unsafe_reason。 */
        uint8_t unsafe_reason = 0U;

        if (res != 0)
        {
            unsafe_reason = (uint8_t)res->unsafe_reason;
        }

        action_code = ArmIK_HandleInvalidTarget();
        g_arm_ik_app.last_status_code = ARM_IK_RESULT_UNSAFE;
        g_arm_ik_app.last_action_code = action_code;
        ArmIK_UpdateFullState();
        ArmIK_SendResultToAll(ARM_IK_RESULT_UNSAFE, unsafe_reason, action_code, res);
    }
    else
    {
        /* 其它求解错误：保持输出并回传参数错误。 */
        action_code = ArmIK_HandleInvalidTarget();
        g_arm_ik_app.last_status_code = ARM_IK_RESULT_PARAM_ERR;
        g_arm_ik_app.last_action_code = action_code;
        ArmIK_UpdateFullState();
        ArmIK_SendResultToAll(ARM_IK_RESULT_PARAM_ERR, 0U, action_code, res);
    }
}

/* 内部紧凑 XYZ 接口仅接收 12 字节；USB ARM_SET_TARGET 使用 16 字节命令布局。 */
void ArmIK_ComponentHandleXYZPayload(const uint8_t *payload, uint16_t len)
{
    float x;
    float y;
    float z;
    uint8_t action_code;

    /* 严格校验紧凑 XYZ payload 长度。 */
    if ((payload == 0) || (len != ARM_IK_XYZ_PAYLOAD_LEN))
    {
        action_code = ArmIK_HandleInvalidTarget();
        g_arm_ik_app.last_status_code = ARM_IK_RESULT_PARAM_ERR;
        g_arm_ik_app.last_action_code = action_code;
        ArmIK_UpdateFullState();
        ArmIK_SendResultToAll(ARM_IK_RESULT_PARAM_ERR, 0U, action_code, 0);
        return;
    }

    /* 按 STM32 little-endian float32 读取 x/y/z。 */
    memcpy(&x, &payload[0], 4);
    memcpy(&y, &payload[4], 4);
    memcpy(&z, &payload[8], 4);

    /* 交给统一的目标求解与安全检查入口。 */
    ArmIK_ComponentStep(x, y, z);
}

/* 读取当前采用的电机控制角（deg）。 */
const ArmIK_MotorDeg_t *ArmIK_GetActiveMotorDeg(void)
{
    return &g_arm_ik_app.active_motor_deg;
}

void ArmIK_SetActualMotorDeg(float j1_deg,
                             float j2_deg,
                             float j3_deg,
                             uint8_t valid)
{
    s_arm_actual_motor_deg.j1_deg = j1_deg;
    s_arm_actual_motor_deg.j2_deg = j2_deg;
    s_arm_actual_motor_deg.j3_deg = j3_deg;
    s_arm_actual_motor_deg.valid = (valid != 0U) ? 1U : 0U;
    ArmIK_UpdateFullState();
}

const ArmIK_FullState_t *ArmIK_GetFullState(void)
{
    ArmIK_UpdateFullState();
    return &g_arm_ik_full_state;
}

/* 读取应用状态（请求、历史有效目标、错误与动作码）。 */
const ArmIK_AppState_t *ArmIK_GetAppState(void)
{
    return &g_arm_ik_app;
}
