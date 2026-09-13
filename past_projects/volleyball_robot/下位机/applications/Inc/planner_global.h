#ifndef PLANNER_GLOBAL_H
#define PLANNER_GLOBAL_H

#include "automatic_control_promote.h"

// 三轴全局速度规划器
extern S_CurvePlanner vx_planner;
extern S_CurvePlanner vy_planner;
extern S_CurvePlanner w_planner;

// 初始化函数
void Planner_Global_Init(float dt);

#endif
