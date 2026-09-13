#ifndef DISPLACEMENT_PID
#define DISPLACEMENT_PID

typedef struct{
	
	float x_error[2];
	float y_error[2];
	float omega_error[2];
	float s_x_error;
	float s_y_error;
	float s_omega_error;

	float xy_kp;
	float xy_kd;
	float xy_ki;
	
	float z_kp;
	float z_kd;
	float z_ki;
	
	float xy_max_iout;
	float xy_max_out;
	
	float z_max_iout;
	float z_max_out;
	
	float iout[3];
	float pout[3];
	float dout[3];
	
	float out[3];

}displacement_PID;

extern displacement_PID my_displacement_pid;
extern float displace_buffer[3];
extern float motors_rnd[3];

float angle_error(float target, float current);
void get_rnd_count_and_diaplacement(void);
void rnd_count_and_diaplacement_reset(void);
void displacement_pid_clear(void);
void xy_displacement_pid_config(float kp,float kd,float ki,float max_iout,float max_out);
void z_displacement_pid_config(float kp,float kd,float ki,float max_iout,float max_out);
void x_displacement_control(float x);
void y_displacement_control(float y);
void theta_displacement_control(float theta);

#endif
