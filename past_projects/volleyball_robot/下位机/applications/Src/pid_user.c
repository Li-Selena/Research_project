#include "pid_user.h"

extern motor_measure_t motor_can1[8];
extern motor_measure_t motor_can2[8];

pid_type_def pid_v_1[8],pid_pos_1[8];
pid_type_def pid_v_2[8],pid_pos_2[8];

float motor_position_classic0_pid[4] = {0.2, 0.1, 0,0};
float motor_speed_classic12_pid[4] = {20, 0.1, 0,2.5};// 3508电机 PID 参数设置为 10, 0.05, 0
float motor_position_classic12_pid[4] = {0.2, 0.1, 0};

float motor_speed_2006_pid[4] = {20, 0.3, 0.8}; // 2006 电机 PID 参数
float motor_position_2006_pid[4] = {0.9, 0, 0.05};


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


// PID 初始化
void PID_devices_Init(void)
{
    for(int i=0;i<4;i++)
    {
		PID_init(&pid_v_1[i], PID_POSITION, motor_speed_classic12_pid, 10000, 6000,500);
		PID_init(&pid_pos_1[i], PID_POSITION, motor_position_classic12_pid, 10000, 5000,500);
        
		PID_init(&pid_v_2[i], PID_POSITION, motor_speed_2006_pid, 15000, 8000,500);
		PID_init(&pid_pos_2[i], PID_POSITION, motor_position_2006_pid, 12000, 3000,500);
    }
    
    for(int i=4;i<8;i++)
    {        
        
        PID_init(&pid_v_2[i], PID_POSITION, motor_speed_2006_pid, 10000, 6000,0);
        PID_init(&pid_pos_2[i], PID_POSITION, motor_position_2006_pid, 400, 300,0);
    }
}

void chassis_pid_clear(void)
{
    for (uint8_t i = 0; i < 3; ++i) PID_clear(&pid_v_1[i]);
}

float PID_velocity_realize_1(float set_speed,int i)//speed前馈
{
        vel_PID_calc(&pid_v_1[i-1],body_accs[i-1],motor_can1[i-1].speed_rpm , set_speed);
        return pid_v_1[i-1].out;
}

float PID_velocity_realize_1_nonfilter(float set_speed,int i)//speed
{
        PID_calc(&pid_v_1[i-1],motor_can1[i-1].speed_rpm , set_speed);
        return pid_v_1[i-1].out;
}

float PID_position_realize_1(float set_pos,int i)
{

        PID_calc(&pid_pos_1[i-1],motor_can1[i-1].total_angle , set_pos);
        return pid_pos_1[i-1].out;

}

float pid_call_1(float position,int i)
{
        return PID_velocity_realize_1_nonfilter(PID_position_realize_1(position,i),i);
}






float PID_velocity_realize_2(float set_speed,int i)
{
        PID_calc(&pid_v_2[i-1],motor_can2[i-1].speed_rpm , set_speed);
        return pid_v_2[i-1].out;
}

float PID_velocity_realize_2_nonfilter(float set_speed,int i)//spped
{
        PID_calc(&pid_v_2[i-1],motor_can2[i-1].speed_rpm , set_speed);
        return pid_v_2[i-1].out;
}

float PID_position_realize_2(float set_pos,int i)
{

        PID_calc(&pid_pos_2[i-1],motor_can2[i-1].total_angle , set_pos);
        return pid_pos_2[i-1].out;

}

float pid_call_2(float position,int i)
{
        return PID_velocity_realize_2_nonfilter(PID_position_realize_2(position,i),i);
}
