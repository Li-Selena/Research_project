#include "include.h"

velocity tool_v={0,0};
velocity *ptr = &tool_v;

float vl_wheel_ave[4] = {0,0,0,0};//电机速度数组
float offset_angle = 0;//角度偏移量
float *of_ang_ptr = &offset_angle;


void set_velocity(velocity *tool_v, float x, float y) {
	tool_v->v_x = x;
	tool_v->v_y = y;
}
void set_vl_omega(float vl_omega, float val) {
	vl_omega = val;
}
void set_vl_r(float vl_r, float val) {
	vl_r = val;
}
void set_offset_angle(float angle, float val) {
	angle = val;
}

void set_vl_wheel(float *w_arr,velocity v,float omega,float r){
    static float angle = PI/6.0f;
    w_arr[0] = -0.5 * v.v_x + 0.866 * v.v_y - omega*r;
    w_arr[1] = v.v_x - omega*r;
    w_arr[2] = -0.5 * v.v_x - 0.866 * v.v_y - omega*r;
}




