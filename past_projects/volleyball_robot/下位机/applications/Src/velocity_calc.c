#include "include.h"
#include "velocity_calc.h"

#define RADIUS 1.0f

float theta;

float calculated_velocity[3];

float calc_buffer[3];               //缓存
float calc_buffer_2nd[3];

float rotate_matrix[3][3]=
{

	{  0 , 0 , 0},
	{  0 , 0 , 0},
	{  0 , 0 , 1}

};                                       //旋转变换矩阵

float inverse_transform_matrix[3][3]=
{

	{  0 , 0 , 0},
	{  0 , 0 , 0},
	{  0 , 0 , 1}

};                                       //逆变矩阵

float solution_matrix[3][3] = {
	{-2.0f/3.0f, 1.0f/3.0f, 1.0f/3.0f},
	{0.0f, 1.0f/1.7320508075688772f, -1.0f/1.7320508075688772f},
	{-1.0f/(3.0f*RADIUS), -1.0f/(3.0f*RADIUS), -1.0f/(3.0f*RADIUS)}

};                                        //运动学正解矩阵

float inverse_solution_matrix[3][3] = {

	{-1 , 0 , -RADIUS},
	{0.5 , 0 , -RADIUS},
	{0.5 , 0 , -RADIUS}

};                                       //运动学逆解算矩阵


void inverse_solution_matrix_init(void)
{
	inverse_solution_matrix[1][1]=sqrt(3)/2;
	inverse_solution_matrix[2][1]=-sqrt(3)/2;
}

void solution_matrix_init(void)
{
	/* Matrix is fully initialized statically and is the inverse of the wheel matrix. */
}

void matrix_multiply(float matrix[3][3],float input_vector[3],float output_vector[3])
{
	float temp[3]={0};
	for (uint8_t i = 0;i< 3; ++i)
	{
			for (uint8_t k = 0; k < 3; ++k)
			{
				temp[i] += matrix[i][k] * input_vector[k];
			}
			output_vector[i]=temp[i];
	}

}

void rotate_matrix_calc(float theta)
{
	rotate_matrix[0][0] = cosf(theta);
	rotate_matrix[0][1] = -sinf(theta);
	rotate_matrix[1][0] = sinf(theta);
	rotate_matrix[1][1] = cosf(theta);
//	
//	inverse_transform_matrix[0][0] = transform_matrix[0][0];
//	inverse_transform_matrix[0][1] = -transform_matrix[0][1];
//	inverse_transform_matrix[1][0] = -transform_matrix[1][0];
//	inverse_transform_matrix[1][1] = transform_matrix[1][1];
}

void get_theta(float omega,float dt)
{
	
	theta+=omega * dt * 2 * PI ;
	
	while(theta>2 * PI)
	{
		theta-=2 * PI;
	}

}

float ftabs_maxfff(float num[3])
{
	float max = fabsf(num[0]);
    float b = fabsf(num[1]);
    float c = fabsf(num[2]);
    if (b > max) max = b;
    if (c > max) max = c;
    return max;
}

void vel_control(float exp_vel[3])//更安全的底盘控制
{
	float command[3] = {exp_vel[0], exp_vel[1], exp_vel[2]};
	float max_abs_vel=0;
	uint32_t now = HAL_GetTick();
	for (uint8_t i = 0; i < 3; ++i) {
		if (!motor_feedback_online(&motor_can1[i], now)) {
			CAN1_SetMotorCurrent(1, 0); CAN1_SetMotorCurrent(2, 0); CAN1_SetMotorCurrent(3, 0);
			chassis_pid_clear();
			return;
		}
	}
	matrix_multiply(inverse_solution_matrix,command,calculated_velocity);
	max_abs_vel=ftabs_maxfff(calculated_velocity);
	if(max_abs_vel>CHASSIS_MAX_RPM)
	{
		float scale=CHASSIS_MAX_RPM/max_abs_vel;
		for(uint8_t i=0;i<3;++i)
		{
			command[i]*=scale;
		}
		matrix_multiply(inverse_solution_matrix,command,calculated_velocity);
	}
	CAN1_SetMotorCurrent(1, PID_velocity_realize_1(calculated_velocity[0],1)*1.1f);
	CAN1_SetMotorCurrent(2, PID_velocity_realize_1(calculated_velocity[1],2));
	CAN1_SetMotorCurrent(3, PID_velocity_realize_1(calculated_velocity[2],3));
}
