#ifndef FILTER_AND_OBSERVER_H
#define FILTER_AND_OBSERVER_H
#include "control_config.h"

#define DT CONTROL_DT_S

typedef struct{
	
	int tmep;
	
}state_observer;

typedef enum{
	False=0,
	True
}my_bool;

typedef struct{
	
	float total_spatium;
	
	float v_max;
	float a_max;
	
	float expected_acc;
	float expected_vel;
	
	float spatium_acc;
	float spatium_const;
	
	float current_v;
	float current_s;
	
	float total_t;
	float t[2];
	float current_t;
	
	int8_t towards;
	float v_out;
	my_bool finished;
	
}spatium_designer;

typedef struct{
	
	float expected_vel;
	float current_vel;
	float max_acc;
	float current_acc;
	
}speed_designer;

extern speed_designer body_acc_design[3];
extern float body_feedforword[3];
extern float body_accs[3];
extern spatium_designer xyw_spatium_designer[3];
extern spatium_designer delta_designer;

float low_pass_filter(float input, float last_output,float alpha);
void vel_designer_init(speed_designer*this,float max_acc);
void xy_spatium_designer_init(spatium_designer*this,float v_m,float a_m);
void designer_update(speed_designer* this);
void feedforword_control(speed_designer*this,float *exp_vel);
void motion_profiles_reset(void);
void spatium_designer_init(spatium_designer*this,float v_m,float a_m);
void spatium_designer_set_target(spatium_designer*this,float total_spatium);
void spatium_designer_update(spatium_designer*this);

// S型速度规划器结构体
typedef struct {
    float v_target;   // 目标速度
    float v_curr;     // 当前速度
    float a_curr;     // 当前加速度
    float j_max;      // 最大加加速度
    float a_max;      // 最大加速度
    float dt;         // 控制周期（单位：秒）
} S_CurvePlanner;

// 初始化规划器
void SPlanner_Init(S_CurvePlanner* planner, float j_max, float a_max, float dt);

// 设置目标速度
void SPlanner_SetTarget(S_CurvePlanner* planner, float target);

// 更新速度（每周期调用）
float SPlanner_Update(S_CurvePlanner* planner);


#endif
