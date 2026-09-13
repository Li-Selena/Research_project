#ifndef __ARM_USER_H__
#define __ARM_USER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "bsp_usb.h"
#include "arm_ik_3r_safe_stm32h7.h"

/* ========================= 结果状态码 ========================= */
/*
 * 发给上位机 / 遥控器的主状态码
 */
#define ARM_IK_RESULT_OK            0U  /* 可达且安全，已采用新目标 */
#define ARM_IK_RESULT_UNREACHABLE   1U  /* 几何上不可达 */
#define ARM_IK_RESULT_UNSAFE        2U  /* 几何上可达，但不满足安全保护 */
#define ARM_IK_RESULT_PARAM_ERR     3U  /* 参数错误、未初始化、数据异常等 */

/* ========================= 动作状态码 ========================= */
/*
 * 当输入目标无效时，应用层到底做了什么处理
 */
#define ARM_IK_ACTION_APPLY_NEW     0U  /* 本次新目标有效，已更新输出 */
#define ARM_IK_ACTION_HOLD_LAST     1U  /* 本次目标无效，保持上一组安全目标 */
#define ARM_IK_ACTION_KEEP_CURRENT  2U  /* 本次目标无效，且无历史安全目标，保持当前不动 */

#define ARM_IK_J1_LIMIT_DEG         180.0f

/* ========================= 输入负载长度 ========================= */
/*
 * 目标点数据格式：
 * x(float) + y(float) + z(float)
 * 共 12 字节
 */
#define ARM_IK_XYZ_PAYLOAD_LEN      12U

/* ========================= 状态回包格式 ========================= */
/*
 * 回包 payload 一共 6 字节：
 *
 * tx_data[0] = status_code
 *   0: 可达且安全
 *   1: 不可达
 *   2: 不安全
 *   3: 参数错误 / 未初始化 / 数据异常
 *
 * tx_data[1] = unsafe_reason
 *   0: 无
 *   1: theta2 超总范围
 *   2: j3 联动保护超范围
 *   3: 目标点超出内缩后的工作空间
 *   4: 低位目标不在模型 XZ 平面内
 *
 * tx_data[2] = action_code
 *   0: 已采用新目标
 *   1: 保持上一组安全目标
 *   2: 保持当前不动
 *
 * tx_data[3] = reachable
 *   0: 不可达
 *   1: 可达
 *
 * tx_data[4] = safe
 *   0: 不安全
 *   1: 安全
 *
 * tx_data[5] = has_last_valid
 *   0: 当前还没有历史安全目标
 *   1: 当前存在历史安全目标
 */

/* ========================= 电机角度制目标 ========================= */
/*
 * 这个结构体就是最终给你电机层使用的目标。
 * 单位全部是“角度制 deg”。
 *
 * 注意：
 * 这不是模型角，也不是几何控制角（rad），
 * 而是经过：
 * 1) 几何控制角求解
 * 2) dir 方向映射
 * 3) rad -> deg
 * 后得到的“最终电机目标角”
 */
typedef struct
{
    float j1_deg;
    float j2_deg;
    float j3_deg;
    uint8_t valid;
} ArmIK_MotorDeg_t;

typedef struct
{
    float theta1_deg;
    float theta2_deg;
    float theta3_deg;
    uint8_t valid;
} ArmIK_ModelDeg_t;

/* ========================= 应用层运行状态 ========================= */
/*
 * 这个结构体用于保存：
 * 1. 最近一次收到的目标点
 * 2. 最近一次安全有效目标点
 * 3. 最近一次有效控制角
 * 4. 当前实际维持的角度制目标
 *
 * 这样当目标点不安全或不可达时，
 * 应用层就可以继续保持上一组安全目标，不会乱跳。
 */
typedef struct
{
    Arm3R_Point_t last_req_pt;            /* 最近一次收到的目标点 */
    Arm3R_Point_t last_valid_pt;          /* 最近一次有效安全目标点 */

    Arm3R_ModelAngles_t last_valid_model; /* 最近一次有效模型控制角（rad） */
    Arm3R_ModelAngles_t active_model;     /* 当前实际维持的模型控制角（rad） */

    Arm3R_CtrlAngles_t last_valid_geom;   /* 最近一次有效几何控制角（rad） */
    Arm3R_CtrlAngles_t last_valid_motor;  /* 最近一次有效电机方向控制角（rad） */

    ArmIK_MotorDeg_t last_valid_motor_deg;/* 最近一次有效电机角度制目标（deg） */
    ArmIK_MotorDeg_t active_motor_deg;    /* 当前实际维持输出的目标（deg） */

    uint8_t has_last_valid;               /* 是否已经存在历史安全目标 */

    /* ── 最近一次 IK 求解结果（防假阳性） ── */
    uint8_t last_status_code;             /* 0=OK, 1=UNREACHABLE, 2=UNSAFE, 3=PARAM_ERR */
    uint8_t last_action_code;             /* 0=APPLY_NEW, 1=HOLD_LAST, 2=KEEP_CURRENT */

} ArmIK_AppState_t;

/*
 * Complete arm runtime snapshot.
 * rad fields keep solver units; deg fields are for debug and motor views.
 */
typedef struct
{
    Arm3R_Config_t cfg;

    uint8_t inited;
    uint8_t has_last_valid;
    uint8_t reachable;
    uint8_t safe;
    uint8_t base_singular;
    uint8_t last_status_code;
    uint8_t last_action_code;
    uint8_t actual_motor_valid;

    Arm3R_Status_t solve_status;
    Arm3R_UnsafeReason_t unsafe_reason;

    Arm3R_Point_t requested_pt;
    Arm3R_Point_t solved_req_pt;
    Arm3R_Point_t last_valid_pt;

    Arm3R_ModelAngles_t solved_model_rad;
    ArmIK_ModelDeg_t solved_model_deg;
    Arm3R_CtrlAngles_t solved_geom_rad;
    ArmIK_MotorDeg_t solved_geom_deg;

    Arm3R_ModelAngles_t active_model_rad;
    ArmIK_ModelDeg_t active_model_deg;
    Arm3R_CtrlAngles_t active_geom_rad;
    ArmIK_MotorDeg_t active_geom_deg;
    Arm3R_CtrlAngles_t active_motor_rad;
    ArmIK_MotorDeg_t active_motor_deg;

    Arm3R_ModelAngles_t last_valid_model_rad;
    ArmIK_ModelDeg_t last_valid_model_deg;
    Arm3R_CtrlAngles_t last_valid_geom_rad;
    ArmIK_MotorDeg_t last_valid_geom_deg;
    Arm3R_CtrlAngles_t last_valid_motor_rad;
    ArmIK_MotorDeg_t last_valid_motor_deg;

    ArmIK_MotorDeg_t actual_motor_deg;
} ArmIK_FullState_t;

/* ========================= 全局句柄 ========================= */
extern Arm3R_Handle_t g_arm_ik;
extern ArmIK_AppState_t g_arm_ik_app;
extern ArmIK_FullState_t g_arm_ik_full_state;

/* ========================= 对外接口 ========================= */
/*
 * 初始化组件
 * 一般在系统初始化完成后调用一次
 */
void ArmIK_ComponentInit(void);

/*
 * 输入目标点进行：
 * 1. 逆解
 * 2. 安全判断
 * 3. 状态双路回传（USB + UART10）
 * 4. 更新当前角度制目标
 */
void ArmIK_ComponentStep(float x, float y, float z);

uint8_t ArmIK_TargetInputAllowed(float x,
                                 float y,
                                 float z,
                                 Arm3R_Status_t *status,
                                 Arm3R_UnsafeReason_t *unsafe_reason);

/*
 * 处理外部收到的 XYZ 负载数据
 * payload 格式固定为 3 个 float：x, y, z
 */
void ArmIK_ComponentHandleXYZPayload(const uint8_t *payload, uint16_t len);

/*
 * 获取当前实际维持的电机角度制目标
 * 你自己的电机控制层可以直接读取这个结果
 */
const ArmIK_MotorDeg_t *ArmIK_GetActiveMotorDeg(void);

void ArmIK_SetActualMotorDeg(float j1_deg,
                             float j2_deg,
                             float j3_deg,
                             uint8_t valid);

const ArmIK_FullState_t *ArmIK_GetFullState(void);

/*
 * 获取整个应用层状态
 * 调试时很有用，例如查看 last_valid_pt / has_last_valid
 */
const ArmIK_AppState_t *ArmIK_GetAppState(void);

#ifdef __cplusplus
}
#endif

#endif /* __ARM_USER_H__ */
