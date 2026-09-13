#ifndef __BSP_FDCAN_H__
#define __BSP_FDCAN_H__
#include "main.h"
#include "fdcan.h"



//dji
void FDCAN_Start(FDCAN_HandleTypeDef *hfdcan);

void FDCAN1_Filter_Init(void);
void FDCAN2_Filter_Init(void);
void FDCAN3_Filter_Init(void);
void FDCAN_Motor_Start_All(void);


#endif
