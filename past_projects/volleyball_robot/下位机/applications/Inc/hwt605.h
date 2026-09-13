#ifndef HWT605_H
#define HWT605_H

void hwt605_init(void);

typedef struct{

	float kp;
	float ki;
	float kd;
	
	float max_iout;
	float max_out;

}angle_PID_config;

typedef struct{
	
	float error[2];
	float de;
	float se;
	
	float pout;
	float iout;
	float dout;
	float out;
	
}angle_PID_struct;

extern angle_PID_struct angular_displacement_PID_struct;
extern angle_PID_struct angular_velocity_PID_struct;
extern angle_measure my_angle_measure;
extern uint8_t measure_flag;
extern uint8_t hwt_what_type;

float angular_displacement_PID(float angular_displacement);
float angular_velocity_PID(float angular_velocity);
float hwt_angle_to_radian(float angle);
void hwt605_request_yaw(void);
void hwt605_request_omega(void);

#endif 
