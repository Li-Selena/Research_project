#include "include.h"
#include "planner_global.h"

// 三个方向的 S 速度规划器
S_CurvePlanner vx_planner;
S_CurvePlanner vy_planner;
S_CurvePlanner w_planner;

// 全局初始化函数
void Planner_Global_Init(float dt) {
    SPlanner_Init(&vx_planner, 2500.0f, 500.0f, dt);  // j_max = 5, a_max = 2
    SPlanner_Init(&vy_planner, 2500.0f, 500.0f, dt);
    SPlanner_Init(&w_planner, 600.0f, 200.0f, dt);  // 通常角速度响应快
}
