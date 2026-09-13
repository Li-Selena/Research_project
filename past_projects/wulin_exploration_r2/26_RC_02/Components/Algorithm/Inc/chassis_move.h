#ifndef __CHASSIS_MOVE_H
#define __CHASSIS_MOVE_H

#include "speedPlanner.h"
#include "s_curve.h"
#include "mecanum_classic.h"


/*
 * ─── 电机 → 轮子 转换常量 ───────────────────────────────
 *
 * 复用 mecanum_classic.h 中的：
 *   MOTOR_IN2OUT = 187 / 3591       电机轴 → 轮轴 减速比
 *   RPM_TO_MS    = 2π * 0.075 / 60  rpm 轮 → m/s 线速
 *   MEC_R        = 0.075            轮子半径 (m)
 *
 * MOTOR_RPM_TO_WHEEL_MPS = MOTOR_IN2OUT * RPM_TO_MS
 *   ≈ 0.052074 * 0.007854 ≈ 0.000409 m/s per motor RPM
 *
 * ENCODER_TO_WHEEL_M = MOTOR_IN2OUT * (2π * MEC_R) / 8192
 *   ≈ 0.052074 * 0.00005752 ≈ 2.996e-6 m per encoder count
 *   等价：1 m ≈ 333778 编码器计数
 */
#define ENCODER_RESOLUTION  8192.0f

/* motor RPM → wheel m/s (mecanum_classic.h 已有等效值 MOTOR_IN2OUT*RPM_TO_MS) */
#define MOTOR_RPM_TO_WHEEL_MPS  (MOTOR_IN2OUT * RPM_TO_MS)

/* encoder delta count → wheel linear meter */
#define ENCODER_TO_WHEEL_M  (MOTOR_IN2OUT * (2.0f * 3.1415926f * MEC_R / ENCODER_RESOLUTION))

/*
 * Physical chassis motor order.
 * Motor IDs start at the front-right corner and go clockwise:
 *   1 / CAN 0x201 / index 0 = FR
 *   2 / CAN 0x202 / index 1 = BR
 *   3 / CAN 0x203 / index 2 = BL
 *   4 / CAN 0x204 / index 3 = FL
 */
typedef enum
{
    CHASSIS_MOTOR_FR = 0U,
    CHASSIS_MOTOR_BR = 1U,
    CHASSIS_MOTOR_BL = 2U,
    CHASSIS_MOTOR_FL = 3U,
    CHASSIS_MOTOR_COUNT = 4U
} ChassisMotorIndex_t;


/* 底盘运动控制器状态机 */
typedef enum
{
    CHASSIS_MOVE_IDLE    = 0,   /* 空闲，无运动 */
    CHASSIS_MOVE_RUNNING = 1,   /* 运动中 */
    CHASSIS_MOVE_DONE    = 2    /* 运动完成，等待外部读取结果 */
} ChassisMove_State_t;

typedef struct
{
    ChassisMove_State_t state;

    /* 目标位移（底盘坐标系：dx 前、dy 左、dyaw CCW） */
    float target_dx;        /* m */
    float target_dy;        /* m */
    float target_dyaw;      /* rad */

    /* 三个轴各自的 S 曲线规划 */
    SCurve_Plan_t s_x;
    SCurve_Plan_t s_y;
    SCurve_Plan_t s_yaw;

    float total_duration;   /* 三轴协调后的统一时长 (s)，取最长轴 */
    float start_time;       /* 运动开始时刻的系统时间 (s)，首次 Update 时记录 */

    /* 里程计起点（编码器原始累积值，上电清零） */
    int32_t start_enc[CHASSIS_MOTOR_COUNT]; /* [0..3] = FR, BR, BL, FL physical motors 1..4 */

    /* 里程计世界坐标起点 (m, rad)，启动瞬间快照 */
    float start_odom_x;
    float start_odom_y;
    float start_odom_yaw;

    /* 当前里程计位置 (m, rad)，由 ChassisOdometry_Update() 实时更新 */
    float odom_x;
    float odom_y;
    float odom_yaw;

    /* 位置环 P 增益（前馈 + P 修正结构） */
    float pos_kp;

    /* 输出：期望底盘速度 (m/s, rad/s)，供下游逆运动学解算 */
    ChassisVel_t cmd_vel;

    /* 运动参数上限，通过 ChassisMove_SetLimits() 或 Init() 设定 */
    float v_max;    /* 最大线速度 (m/s) */
    float a_max;    /* 最大加速度 (m/s²) */
    float j_max;    /* 最大 jerk (m/s³) */
} ChassisMove_Ctrl_t;

/**
 * @brief 初始化控制器（上电调用一次）
 */
void ChassisMove_Init(ChassisMove_Ctrl_t *ctrl);

/**
 * @brief 设定运动参数上限
 */
void ChassisMove_SetLimits(ChassisMove_Ctrl_t *ctrl,
                           float v_max, float a_max, float j_max);

/**
 * @brief 启动一次固定距离移动
 * @param dx    前移距离 (m)，正=前
 * @param dy    左移距离 (m)，正=左
 * @param dyaw  CCW 旋转角度 (rad)，正=CCW
 */
void ChassisMove_Start(ChassisMove_Ctrl_t *ctrl,
                       float dx, float dy, float dyaw);

/**
 * @brief 每控制周期调用一次
 * @param dt      距上次调用的时间 (s)
 * @param now_sec 当前系统时间 (s)，用于计算 S 曲线进度
 * @return 当前状态
 */
ChassisMove_State_t ChassisMove_Update(ChassisMove_Ctrl_t *ctrl,
                                       float dt, float now_sec);

/**
 * @brief 由电机反馈数据更新里程计（每次读取电机数据后调用）
 * @param ctrl  控制器句柄
 * @param param 底盘几何参数
 */
void ChassisOdometry_Update(ChassisMove_Ctrl_t *ctrl,
                            const MecanumParam_t *param);

/**
 * @brief 急停，立即清零输出并回到 IDLE
 */
void ChassisMove_Stop(ChassisMove_Ctrl_t *ctrl);

/**
 * @brief 正向运动学：四轮线速度 → 底盘速度
 * @param wheels  四轮线速度 [fl, fr, bl, br] (m/s)
 * @param param   底盘参数
 * @param out     底盘速度输出
 */
void ChassisForwardKinematics(const WheelSpeed_t *wheels,
                              const MecanumParam_t *param,
                              ChassisVel_t *out);

/**
 * @brief 编码器差值 → 单轮线位移 (m)
 * @param delta_enc 编码器增量（含符号，已做转向校正）
 * @return 轮子线位移 (m)
 */
float EncoderDeltaToWheelMeter(int32_t delta_enc);

/**
 * @brief 电机 RPM → 单轮线速度 (m/s)，已做转向校正
 * @param motor_rpm 电机转速 (RPM)
 * @param motor_idx Physical motor index: 0=FR, 1=BR, 2=BL, 3=FL
 * @return 轮子线速度 (m/s，正向=前进)
 */
float MotorRPMToWheelMPS(float motor_rpm, uint8_t motor_idx);

#endif
