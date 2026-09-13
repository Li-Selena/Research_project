#ifndef __PID_USER_H
#define __PID_USER_H
#include "pid.h"
#include "include.h"

typedef struct
{
    float angle_kp;
    float angle_ki;
    float angle_kd;
    float angle_kf;
    float rate_kp;
    float rate_ki;
    float rate_kd;
    float rate_kf;
} ChassisYawPIDParam_t;

void PID_devices_Init(void);

float PID_velocity_realize_1(float set_speed,int i);
float PID_position_realize_1(float set_pos,int i);
float pid_call_1(float position,int i);
void PID_FDCAN1_Clear(uint8_t motor_id);

float PID_velocity_realize_2(float set_speed,int i);
float PID_position_realize_2(float set_pos,int i);
float pid_call_2(float position,int i);
void PID_FDCAN2_Clear(uint8_t motor_id);

float PID_velocity_realize_3(float set_speed,int i);
float PID_position_realize_3(float set_pos,int i);
float pid_call_3(float position,int i);

float Chassis_Yaw_Robot_Frame_Ctrl(float target_yaw_rate);
float Chassis_Yaw_World_Frame_Ctrl(float target_yaw_angle);
void Chassis_Yaw_PID_Clear(void);
void Chassis_Yaw_GetPIDParam(ChassisYawPIDParam_t *out);
void Chassis_Yaw_SetPIDParam(const ChassisYawPIDParam_t *param);


#endif























