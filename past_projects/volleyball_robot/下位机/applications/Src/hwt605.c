#include "include.h"
#include "hwt605.h"
#include "usart.h"

uint8_t hwt_tx_buffer[8]={0};
uint8_t measure_flag=0;
uint8_t hwt_what_type=0;
static uint32_t hwt_request_ms;
angle_PID_config angular_displacement_PID_config={30,0,0,200,2800};//kp,ki,kd,max_iout,max_out//184输出对应1rad/s的角速度
angle_PID_config angular_velocity_PID_config={40,0,0,400,1200};
angle_PID_struct angular_displacement_PID_struct={0};
angle_PID_struct angular_velocity_PID_struct={0};

void hwt605_request_yaw(void)
{
	if (hwt_what_type != 0U && (uint32_t)(HAL_GetTick()-hwt_request_ms) > IMU_TIMEOUT_MS)
		hwt_what_type = 0;
	if (hwt_what_type != 0U || huart2.gState != HAL_UART_STATE_READY) return;
	hwt_tx_buffer[0] = 0x50;
	hwt_tx_buffer[1] = 0x03;
	hwt_tx_buffer[2] = 0x00;
	hwt_tx_buffer[3] = 0x3D;
	hwt_tx_buffer[4] = 0x00;
	hwt_tx_buffer[5] = 0x03;
	hwt_tx_buffer[6] = 0x99;
	hwt_tx_buffer[7] = 0x86;
	if (HAL_UART_Transmit_IT(&huart2,hwt_tx_buffer,8) == HAL_OK) {
		hwt_what_type=1;
		hwt_request_ms=HAL_GetTick();
	}
}//50 03 00 3D 00 03 99 86 
void hwt605_request_omega(void)
{
	if (hwt_what_type != 0U && (uint32_t)(HAL_GetTick()-hwt_request_ms) > IMU_TIMEOUT_MS)
		hwt_what_type = 0;
	if (hwt_what_type != 0U || huart2.gState != HAL_UART_STATE_READY) return;
	hwt_tx_buffer[0] = 0x50;
	hwt_tx_buffer[1] = 0x03;
	hwt_tx_buffer[2] = 0x00;
	hwt_tx_buffer[3] = 0x37;
	hwt_tx_buffer[4] = 0x00;
	hwt_tx_buffer[5] = 0x03;
	hwt_tx_buffer[6] = 0xb9;
	hwt_tx_buffer[7] = 0x84;
	if (HAL_UART_Transmit_IT(&huart2,hwt_tx_buffer,8) == HAL_OK) {
		hwt_what_type=2;
		hwt_request_ms=HAL_GetTick();
	}
}//50 03 00 37 00 03 B9 84

float angular_displacement_PID(float angular_displacement)
{
	angular_displacement_PID_struct.error[1]=angular_displacement_PID_struct.error[0];
	angular_displacement_PID_struct.error[0]=angle_error(angular_displacement,my_angle_measure.yaw);
	
	angular_displacement_PID_struct.se+=angular_displacement_PID_struct.error[0];
	angular_displacement_PID_struct.de=angular_displacement_PID_struct.error[0]-angular_displacement_PID_struct.error[1];
	
	angular_displacement_PID_struct.dout=angular_displacement_PID_config.kd*angular_displacement_PID_struct.de;
	angular_displacement_PID_struct.iout=angular_displacement_PID_config.ki*angular_displacement_PID_struct.se;
	angular_displacement_PID_struct.pout=angular_displacement_PID_config.kp*angular_displacement_PID_struct.error[0];
	
	
	if(angular_displacement_PID_struct.iout>angular_displacement_PID_config.max_iout)
	{
		
		angular_displacement_PID_struct.iout=angular_displacement_PID_config.max_iout;
		
	}
	
	else if(angular_displacement_PID_struct.iout<-angular_displacement_PID_config.max_iout)
	{
		
		angular_displacement_PID_struct.iout=-angular_displacement_PID_config.max_iout;
		
	}
	
	angular_displacement_PID_struct.out=(angular_displacement_PID_struct.pout+angular_displacement_PID_struct.dout+angular_displacement_PID_struct.iout);
	
	if(angular_displacement_PID_struct.out>angular_displacement_PID_config.max_out)
	{
		
		angular_displacement_PID_struct.out=angular_displacement_PID_config.max_out;
		
	}
	
	else if(angular_displacement_PID_struct.out<-angular_displacement_PID_config.max_out)
	{
		
		angular_displacement_PID_struct.out=-angular_displacement_PID_config.max_out;
		
	}
	
	return angular_displacement_PID_struct.out;
	
}

float angular_velocity_PID(float angular_velocity)
{
	
	angular_velocity_PID_struct.error[1]=angular_velocity_PID_struct.error[0];
	angular_velocity_PID_struct.error[0]=angular_velocity-my_angle_measure.yaw_omega;
	
	angular_velocity_PID_struct.se+=angular_velocity_PID_struct.error[0];
	angular_velocity_PID_struct.de=angular_velocity_PID_struct.error[0]-angular_velocity_PID_struct.error[1];
	
	angular_velocity_PID_struct.dout=angular_velocity_PID_config.kd*angular_velocity_PID_struct.de;
	angular_velocity_PID_struct.iout=angular_velocity_PID_config.ki*angular_velocity_PID_struct.se;
	angular_velocity_PID_struct.pout=angular_velocity_PID_config.kp*angular_velocity_PID_struct.error[0];
	
	if(angular_velocity_PID_struct.iout>angular_velocity_PID_config.max_iout)
	{
		
		angular_velocity_PID_struct.iout=angular_velocity_PID_config.max_iout;
		
	}
	
	else if(angular_velocity_PID_struct.iout<-angular_velocity_PID_config.max_iout)
	{
		
		angular_velocity_PID_struct.iout=-angular_velocity_PID_config.max_iout;
		
	}
	
	angular_velocity_PID_struct.out=(angular_velocity_PID_struct.pout+angular_velocity_PID_struct.dout+angular_velocity_PID_struct.iout);
	
	if(angular_velocity_PID_struct.out>angular_velocity_PID_config.max_out)
	{
		
		angular_velocity_PID_struct.out=angular_velocity_PID_config.max_out;
		
	}
	
	else if(angular_velocity_PID_struct.out<-angular_velocity_PID_config.max_out)
	{
		
		angular_velocity_PID_struct.out=-angular_velocity_PID_config.max_out;
		
	}
	
	return angular_velocity_PID_struct.out;
	
}

float hwt_angle_to_radian(float angle)
{
	
	return angle * CONTROL_PI / 180.0f;
	
}
