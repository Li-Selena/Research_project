#ifndef __FDCAN_RECEIVE_H
#define __FDCAN_RECEIVE_H

#include "include.h"

/*
 * CAN send and receive ID.
 * On FDCAN1 chassis motors, M1..M4 are physical motor IDs:
 * M1=FR, M2=BR, M3=BL, M4=FL, starting at the front-right corner clockwise.
 */
typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,
    CAN_3508_M5_ID = 0x205,
    CAN_3508_M6_ID = 0x206,
    CAN_3508_M7_ID = 0x207,
    CAN_3508_M8_ID = 0x208,
    CAN_3508_ALL_ID = 0x1FF,
} can_msg_id_e;



typedef struct
{
 uint16_t angle;
 int16_t speed_rpm;
 int16_t given_current;
 uint8_t temperature;
 int16_t last_angle;
		int32_t total_angle;
		int32_t	round_cnt;
		uint16_t offset_angle;
		uint32_t			msg_cnt;
} motor_measure_t;

extern motor_measure_t motor_fdcan1[8];  /* FDCAN1 chassis: [0..3] = FR, BR, BL, FL */
extern motor_measure_t motor_fdcan2[8];  /* 抬升四电机 */
extern motor_measure_t motor_fdcan3[8];  /* 机械臂电机 */


void get_motor_measure(motor_measure_t *ptr,uint8_t data[]);
void get_motor_offset(motor_measure_t *ptr, uint8_t data[]);
void get_total_angle(motor_measure_t *p);


uint8_t FDCAN_Send(FDCAN_HandleTypeDef *hfdcan, uint16_t std_id,
                       int16_t m1, int16_t m2, int16_t m3, int16_t m4);

void FDCAN1_CMD_1(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void FDCAN1_CMD_2(int16_t motor5, int16_t motor6, int16_t motor7, int16_t motor8);
void FDCAN2_CMD_1(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void FDCAN2_CMD_2(int16_t motor5, int16_t motor6, int16_t motor7, int16_t motor8);
void FDCAN3_CMD_1(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void FDCAN3_CMD_2(int16_t motor5, int16_t motor6, int16_t motor7, int16_t motor8);








#endif



