#ifndef __ARM_IK_3R_SAFE_STM32H7_H__
#define __ARM_IK_3R_SAFE_STM32H7_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#ifndef ARM3R_PI
#define ARM3R_PI 3.14159265358979323846f
#endif

#ifndef ARM3R_EPS
#define ARM3R_EPS 1e-6f
#endif

/* ==================== 状态定义 ==================== */
typedef enum
{
    ARM3R_OK = 0,               /* 可达且安全，可执行 */
    ARM3R_ERR_NULL = -1,        /* 空指针 */
    ARM3R_ERR_PARAM = -2,       /* 参数非法 */
    ARM3R_ERR_UNREACHABLE = -3, /* 目标点不可达 */
    ARM3R_ERR_UNSAFE = -4,      /* 目标点可达，但不满足安全保护 */
    ARM3R_ERR_NOT_INIT = -5     /* 句柄未初始化 */
} Arm3R_Status_t;

typedef enum
{
    ARM3R_UNSAFE_NONE = 0,
    ARM3R_UNSAFE_THETA2_RANGE = 1, /* theta2 超出总范围 */
    ARM3R_UNSAFE_J3_RANGE = 2,     /* j3 超出联动保护范围 */
    ARM3R_UNSAFE_WORKSPACE_MARGIN = 3, /* 目标点超出内缩后的工作空间 */
    ARM3R_UNSAFE_J1_LOCK_PLANE = 4     /* 低位目标不在模型 XZ 平面内 */
} Arm3R_UnsafeReason_t;

/* ==================== 基本数据结构 ==================== */
typedef struct
{
    float x;
    float y;
    float z;
} Arm3R_Point_t;

typedef struct
{
    float theta1; /* rad */
    float theta2; /* rad */
    float theta3; /* rad */
    uint8_t valid;
} Arm3R_ModelAngles_t;

/*
 * 解耦控制角（几何意义）
 *
 * j1: 底座相对上电参考姿态的变化量
 * j2: 第一段活动连杆绝对方向相对上电参考姿态的变化量
 * j3: 第二段活动连杆绝对方向相对上电参考姿态的变化量
 */
typedef struct
{
    float j1; /* rad */
    float j2; /* rad */
    float j3; /* rad */
    uint8_t valid;
} Arm3R_CtrlAngles_t;

/*
 * 固定上电参考姿态
 *
 * model_ref:
 *   上电参考姿态在模型中的角
 *
 * dir:
 *   如果后面需要换成“实际电机指令角”，可以再乘 dir。
 *   当前安全判定和控制角定义默认按几何正方向，不直接使用 dir 做安全判断。
 */
typedef struct
{
    float model_ref; /* rad */
    int8_t dir;      /* +1 / -1 */
} Arm3R_JointRef_t;

typedef struct
{
    float d1;
    float a2;
    float a3;
} Arm3R_LinkParam_t;

typedef struct
{
    Arm3R_LinkParam_t link;
    Arm3R_JointRef_t j1_ref;
    Arm3R_JointRef_t j2_ref;
    Arm3R_JointRef_t j3_ref;
} Arm3R_Config_t;

typedef struct
{
    Arm3R_Status_t status;
    Arm3R_UnsafeReason_t unsafe_reason;

    uint8_t reachable;
    uint8_t safe;
    uint8_t base_singular;

    Arm3R_Point_t req_pt;      /* 输入目标点 */
    Arm3R_ModelAngles_t model; /* 输出模型角 */
    Arm3R_CtrlAngles_t ctrl;   /* 输出控制角（几何意义） */
} Arm3R_Result_t;

typedef struct
{
    Arm3R_Config_t cfg;
    Arm3R_Result_t result;
    uint8_t inited;
} Arm3R_Handle_t;

/* ==================== 工具函数 ==================== */
float Arm3R_DegToRad(float deg);
float Arm3R_RadToDeg(float rad);
float Arm3R_NormalizeAngle(float angle_rad);

/* ==================== 组件接口 ==================== */
void Arm3R_Init(Arm3R_Handle_t *arm, const Arm3R_Config_t *cfg);
void Arm3R_ResetResult(Arm3R_Result_t *result);
const Arm3R_Result_t *Arm3R_GetResult(const Arm3R_Handle_t *arm);

/* ==================== 正/逆解辅助函数 ==================== */
/*
 * 模型角 -> 解耦控制角
 *
 * j1 = theta1 - theta1_ref
 * j2 = theta2 - theta2_ref
 * j3 = (theta2+theta3) - (theta2_ref+theta3_ref)
 */
void Arm3R_ModelToCtrl(float theta1,
                       float theta2,
                       float theta3,
                       const Arm3R_JointRef_t *j1_ref,
                       const Arm3R_JointRef_t *j2_ref,
                       const Arm3R_JointRef_t *j3_ref,
                       float *j1,
                       float *j2,
                       float *j3);

/*
 * 解耦控制角 -> 模型角
 *
 * theta1 = theta1_ref + j1
 * theta2 = theta2_ref + j2
 * theta3 = theta3_ref + j3 - j2
 */
void Arm3R_CtrlToModel(float j1,
                       float j2,
                       float j3,
                       const Arm3R_JointRef_t *j1_ref,
                       const Arm3R_JointRef_t *j2_ref,
                       const Arm3R_JointRef_t *j3_ref,
                       float *theta1,
                       float *theta2,
                       float *theta3);

/*
 * 正运动学（模型角 -> 连杆末端坐标）
 *
 * px = cos(theta1) * (-a2*sin(theta2) - a3*sin(theta2 + theta3))
 * py = sin(theta1) * (-a2*sin(theta2) - a3*sin(theta2 + theta3))
 * pz = d1 + a2*cos(theta2) + a3*cos(theta2 + theta3)
 */
void Arm3R_FK_Model(float theta1,
                    float theta2,
                    float theta3,
                    float d1,
                    float a2,
                    float a3,
                    float *x,
                    float *y,
                    float *z);

/* ==================== 安全保护函数 ==================== */
/*
 * 获取在当前 theta2 模型角下，j3 的安全范围
 *
 * 返回：
 *   1 = theta2 在总安全区间内，min_j3/max_j3 有效
 *   0 = theta2 超出总安全区间
 */
uint8_t Arm3R_GetJ3SafeRange(float theta2_model,
                             float *min_j3,
                             float *max_j3);

/*
 * 检查模型角 + 控制角是否满足安全保护
 *
 * 返回：
 *   1 = 安全
 *   0 = 不安全
 */
uint8_t Arm3R_CheckSafety(float theta2_model,
                          float j3_ctrl,
                          Arm3R_UnsafeReason_t *reason);

/* ==================== 主求解函数 ==================== */
/*
 * 逆运动学 + 安全保护判定
 *
 * 功能：
 * 1. 根据目标点求逆解（只保留凸型单解）
 * 2. 得到模型角
 * 3. 换算得到控制角
 * 4. 根据安全保护判断工作点是否合理
 */
Arm3R_Status_t Arm3R_Solve(Arm3R_Handle_t *arm,
                           float x,
                           float y,
                           float z,
                           float theta1_hint);

/*
 * 可选辅助：将“几何控制角”映射成“电机方向控制角”。
 *
 * 注意：
 * 1. 这里只做 dir 方向映射，不做零点补偿、不做编码器单位换算。
 * 2. 如果你已经有自己的电机控制层，也可以完全不用这个函数。
 */
void Arm3R_GeomCtrlToMotorCtrl(const Arm3R_CtrlAngles_t *geom_ctrl,
                               const Arm3R_Config_t *cfg,
                               Arm3R_CtrlAngles_t *motor_ctrl);

#ifdef __cplusplus
}
#endif

#endif /* __ARM_IK_3R_SAFE_STM32H7_H__ */
