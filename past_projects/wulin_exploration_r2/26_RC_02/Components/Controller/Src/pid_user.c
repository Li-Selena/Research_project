#include "pid_user.h"
#include "INS_Task.h"

extern motor_measure_t motor_fdcan1[8];  /* 底盘电机 */
extern motor_measure_t motor_fdcan2[8];  /* 抬升电机 */
extern motor_measure_t motor_fdcan3[8];  /* 机械臂电机 */

pid_type_def pid_v_1[8],pid_pos_1[8];   /* FDCAN1: 底盘 */
pid_type_def pid_v_2[8],pid_pos_2[8];   /* FDCAN2: 抬升 */
pid_type_def pid_v_3[8],pid_pos_3[8];   /* FDCAN3: 机械臂 */

float motor_speed_3508_pid_1[8]    = {5, 0.02, 0.1, 0.2};//3508 底盘速度环
float motor_position_3508_pid_1[8] = {0.2, 0, 1, 0};
float motor_speed_2006_pid_1[8]    = {9, 0.1, 0, 0.3};//2006
float motor_position_2006_pid_1[8] = {0.2, 0, 0, 0};
float motor_speed_2006_climb_drive_pid_1[8]    = {9, 0.1, 0, 0.3};//FDCAN1 M2006 climb drive 5..6
float motor_position_2006_climb_drive_pid_1[8] = {0.2, 0, 0, 0};

float motor_speed_3508_pid_2[8]    = {5, 0.02, 0.1, 0.2};//3508 抬升
float motor_position_3508_pid_2[8] = {0.2, 0, 1, 0};
float motor_speed_2006_pid_2[8]    = {9, 0.1, 0, 0.3};//2006
float motor_position_2006_pid_2[8] = {0.2, 0, 0, 0};

float motor_speed_3508_pid_3[8]    = {5, 0.02, 0.1, 0.2};//3508 机械臂
float motor_position_3508_pid_3[8] = {0.2, 0, 1, 0};
float motor_speed_2006_pid_3[8]    = {9, 0.1, 0, 0.3};//2006
float motor_position_2006_pid_3[8] = {0.2, 0, 0, 0};

// 实例化 Yaw 轴控制的 PID 结构体
pid_type_def pid_yaw_angle; // 世界坐标系：角度外环
pid_type_def pid_yaw_rate;  // 机器人/世界坐标系：角速度内环

// PID 参数数组: {Kp, Ki, Kd, Kf(前馈)}
// 注意：以下参数需根据底盘实际重量和动力情况进行整定
const float PID_YAW_ANGLE_PARAM[4] = {2.03f, 0.0f, 0.05f, 0.0f};  // yaw angle loop Kp/Ki/Kd/Kf
const float PID_YAW_RATE_PARAM[4]  = {0.8f, 0.0f, 0.0f, 0.0f};  // yaw rate loop Kp/Ki/Kd/Kf

#define FDCAN2_CLIMB_LEG_POS_MAX_RPM       3000
#define FDCAN2_CLIMB_LEG_POS_MAX_IOUT       300
#define FDCAN2_CLIMB_DRIVE_POS_MAX_RPM     4200
#define FDCAN2_CLIMB_DRIVE_POS_MAX_IOUT     300
#define FDCAN1_CLIMB_DRIVE_POS_MAX_RPM     4200
#define FDCAN1_CLIMB_DRIVE_POS_MAX_IOUT     300


#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }


// 初始化各电机及底盘 yaw PID 参数。
void PID_devices_Init(void)
{
	for(int i=0;i<4;i++)
	{
		/* FDCAN1: 底盘电机 */
		PID_init(&pid_v_1[i], PID_POSITION, motor_speed_3508_pid_1, 10000, 6000);
		PID_init(&pid_pos_1[i], PID_POSITION, motor_position_3508_pid_1, 1000, 300);

		/* FDCAN2: 抬升电机 */
		PID_init(&pid_v_2[i], PID_POSITION, motor_speed_3508_pid_2, 10000, 6000);
		PID_init(&pid_pos_2[i], PID_POSITION, motor_position_3508_pid_2,
		         FDCAN2_CLIMB_LEG_POS_MAX_RPM,
		         FDCAN2_CLIMB_LEG_POS_MAX_IOUT);

		/* FDCAN3: 机械臂电机 */
		if (i == 3) {
			PID_init(&pid_v_3[i], PID_POSITION, motor_speed_2006_pid_3, 10000, 6000);
			PID_init(&pid_pos_3[i], PID_POSITION, motor_position_2006_pid_3, 1000, 300);
		} else {
			PID_init(&pid_v_3[i], PID_POSITION, motor_speed_3508_pid_3, 10000, 6000);
			PID_init(&pid_pos_3[i], PID_POSITION, motor_position_3508_pid_3, 1000, 300);
		}
	}
	for(int i=4;i<8;i++)
	{
		/* FDCAN1: 底盘辅助电机 */
		PID_init(&pid_v_1[i], PID_POSITION, motor_speed_3508_pid_1, 10000, 6000);
		PID_init(&pid_pos_1[i], PID_POSITION, motor_position_3508_pid_1, 1000, 300);

		if ((i == 4) || (i == 5)) {
			PID_init(&pid_v_1[i], PID_POSITION, motor_speed_2006_climb_drive_pid_1, 10000, 6000);
			PID_init(&pid_pos_1[i], PID_POSITION, motor_position_2006_climb_drive_pid_1,
			         FDCAN1_CLIMB_DRIVE_POS_MAX_RPM,
			         FDCAN1_CLIMB_DRIVE_POS_MAX_IOUT);
		}

		/* FDCAN2: motors 5..6 are rear M2006 drive wheels. */
		if ((i == 4) || (i == 5)) {
			PID_init(&pid_v_2[i], PID_POSITION, motor_speed_2006_pid_2, 10000, 6000);
			PID_init(&pid_pos_2[i], PID_POSITION, motor_position_2006_pid_2,
			         FDCAN2_CLIMB_DRIVE_POS_MAX_RPM,
			         FDCAN2_CLIMB_DRIVE_POS_MAX_IOUT);
		} else {
			PID_init(&pid_v_2[i], PID_POSITION, motor_speed_3508_pid_2, 10000, 6000);
			PID_init(&pid_pos_2[i], PID_POSITION, motor_position_3508_pid_2,
			         FDCAN2_CLIMB_LEG_POS_MAX_RPM,
			         FDCAN2_CLIMB_LEG_POS_MAX_IOUT);
		}

		/* FDCAN3: 机械臂辅助 */
		PID_init(&pid_v_3[i], PID_POSITION, motor_speed_3508_pid_3, 10000, 6000);
		PID_init(&pid_pos_3[i], PID_POSITION, motor_position_3508_pid_3, 1000, 300);
	}

    /* IMU yaw 轴 PID 初始化 */
    PID_init(&pid_yaw_angle, PID_POSITION, PID_YAW_ANGLE_PARAM, 180, 80);
    PID_init(&pid_yaw_rate,  PID_POSITION, PID_YAW_RATE_PARAM,  360, 60);
}


/* ── FDCAN1: 底盘电机 PID ── */
float PID_velocity_realize_1(float set_speed,int i)
{
		PID_calc(&pid_v_1[i-1],motor_fdcan1[i-1].speed_rpm , set_speed);
		return pid_v_1[i-1].out;
}

float PID_position_realize_1(float set_pos,int i)
{
		PID_calc(&pid_pos_1[i-1],motor_fdcan1[i-1].total_angle , set_pos);
		return pid_pos_1[i-1].out;
}

float pid_call_1(float position,int i)
{
		return PID_velocity_realize_1(PID_position_realize_1(position,i),i);
}

/* ── FDCAN2: 抬升电机 PID ── */
void PID_FDCAN1_Clear(uint8_t motor_id)
{
	if ((motor_id == 0U) || (motor_id > 8U)) {
		return;
	}

	PID_clear(&pid_v_1[motor_id - 1U]);
	PID_clear(&pid_pos_1[motor_id - 1U]);
}

float PID_velocity_realize_2(float set_speed,int i)
{
		PID_calc(&pid_v_2[i-1],motor_fdcan2[i-1].speed_rpm , set_speed);
		return pid_v_2[i-1].out;
}

float PID_position_realize_2(float set_pos,int i)
{

		PID_calc(&pid_pos_2[i-1],motor_fdcan2[i-1].total_angle , set_pos);
		return pid_pos_2[i-1].out;

}

float pid_call_2(float position,int i)
{
		return PID_velocity_realize_2(PID_position_realize_2(position,i),i);
}

void PID_FDCAN2_Clear(uint8_t motor_id)
{
	if ((motor_id == 0U) || (motor_id > 8U)) {
		return;
	}

	PID_clear(&pid_v_2[motor_id - 1U]);
	PID_clear(&pid_pos_2[motor_id - 1U]);
}



float PID_velocity_realize_3(float set_speed,int i)
{
		PID_calc(&pid_v_3[i-1],motor_fdcan3[i-1].speed_rpm , set_speed);
		return pid_v_3[i-1].out;
}

float PID_position_realize_3(float set_pos,int i)
{

		PID_calc(&pid_pos_3[i-1],motor_fdcan3[i-1].total_angle , set_pos);
		return pid_pos_3[i-1].out;

}

float pid_call_3(float position,int i)
{
		return PID_velocity_realize_3(PID_position_realize_3(position,i),i);
}

/**
 * @brief  角度归一化处理（极其重要）
 * @param  angle 原始角度误差
 * @return 处理后的最短路径误差 (-180 到 180)
 */
static float Format_Yaw_Angle(float angle)
{
    while (angle > 180.0f)  angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

/**
 * @brief  模式1：机器人坐标系 (相对防跑偏)
 * @param  target_yaw_rate 目标旋转速度 (单位: °/s)。直线平移时传 0.0f。
 * @return 旋转补偿输出量 (vw, °/s)
 *
 * yaw 符号统一约定：+target_yaw_rate / +gyro_z / +vw 都表示逆时针。
 * 这里不再对 gyro_z 取反，符号转换只应出现在实际电机安装映射层。
 */
float Chassis_Yaw_Robot_Frame_Ctrl(float target_yaw_rate)
{
    INS_NavState_t ins_state;

    INS_GetState(&ins_state);
    return PID_calc(&pid_yaw_rate, ins_state.gyro_z_dps, target_yaw_rate);
}

/**
 * @brief  模式2：世界坐标系 (绝对方向锁定)
 * @param  target_yaw_angle 目标绝对角度 (单位: °)
 * @return 旋转补偿输出量 (vw, °/s)
 *
 * 内环 gyro_z 与外环 yaw_total_deg 使用同一正方向：逆时针为正。
 */
float Chassis_Yaw_World_Frame_Ctrl(float target_yaw_angle)
{
    INS_NavState_t ins_state;
    float error_angle;
    float target_yaw_rate;

    INS_GetState(&ins_state);
    error_angle = target_yaw_angle - ins_state.yaw_total_deg;

    // 1. Calculate shortest-path yaw angle error.
    error_angle = Format_Yaw_Angle(error_angle);

    // 2. 角度环计算 (外环)
    target_yaw_rate = PID_calc(&pid_yaw_angle, 0.0f, error_angle);

    // 3. Angle-rate loop, gyro_z uses the same sign as target_yaw_rate.
    return PID_calc(&pid_yaw_rate, ins_state.gyro_z_dps, target_yaw_rate);
}

/**
 * @brief  清除 Yaw 轴 PID 历史积分与状态 (切换模式时调用)
 */
void Chassis_Yaw_PID_Clear(void)
{
    PID_clear(&pid_yaw_angle);
    PID_clear(&pid_yaw_rate);
}

void Chassis_Yaw_GetPIDParam(ChassisYawPIDParam_t *out)
{
    if (out == NULL) {
        return;
    }

    out->angle_kp = pid_yaw_angle.Kp;
    out->angle_ki = pid_yaw_angle.Ki;
    out->angle_kd = pid_yaw_angle.Kd;
    out->angle_kf = pid_yaw_angle.Kf;
    out->rate_kp  = pid_yaw_rate.Kp;
    out->rate_ki  = pid_yaw_rate.Ki;
    out->rate_kd  = pid_yaw_rate.Kd;
    out->rate_kf  = pid_yaw_rate.Kf;
}

void Chassis_Yaw_SetPIDParam(const ChassisYawPIDParam_t *param)
{
    if (param == NULL) {
        return;
    }

    pid_yaw_angle.Kp = param->angle_kp;
    pid_yaw_angle.Ki = param->angle_ki;
    pid_yaw_angle.Kd = param->angle_kd;
    pid_yaw_angle.Kf = param->angle_kf;
    pid_yaw_rate.Kp  = param->rate_kp;
    pid_yaw_rate.Ki  = param->rate_ki;
    pid_yaw_rate.Kd  = param->rate_kd;
    pid_yaw_rate.Kf  = param->rate_kf;

    Chassis_Yaw_PID_Clear();
}




