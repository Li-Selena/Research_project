#define PI 3.1415926f

typedef struct {
	float v_x;
	float v_y;
}velocity;

extern float vl_wheel_ave[4] ;//电机速度数组
extern float offset_angle;//角度偏移量

extern void set_velocity(velocity *tool_v, float x, float y);
extern void set_vl_omega(float vl_omega, float val);
extern void set_vl_r(float vl_r, float val);
extern void set_offset_angle(float angle, float val);

extern void set_vl_wheel(float *w_arr,velocity v,float omega,float r);

extern velocity tool_v;
extern velocity *ptr;



