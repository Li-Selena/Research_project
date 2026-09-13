#include "bsp_fdcan.h"
#include "fdcan_receive.h"
#include "stm32h7xx_hal_fdcan.h"

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

extern motor_measure_t motor_fdcan1[8];
extern motor_measure_t motor_fdcan2[8];
extern motor_measure_t motor_fdcan3[8];

static void FDCAN_Motor_Filter_Init(FDCAN_HandleTypeDef *hfdcan, uint32_t fifo)
{
    FDCAN_FilterTypeDef sFilterConfig = {0};

    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0U;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = fifo;
    sFilterConfig.FilterID1 = CAN_CHASSIS_ALL_ID;
    sFilterConfig.FilterID2 = 0x7F0U;

    if (HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

static void FDCAN_Process_Motor_Rx(FDCAN_HandleTypeDef *hfdcan, uint32_t fifo, motor_measure_t *motor)
{
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    uint32_t motor_index;

    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, fifo) > 0U)
    {
        if (HAL_FDCAN_GetRxMessage(hfdcan, fifo, &rx_header, rx_data) != HAL_OK)
        {
            break;
        }

        if ((rx_header.IdType != FDCAN_STANDARD_ID) ||
            (rx_header.Identifier < CAN_3508_M1_ID) ||
            (rx_header.Identifier > CAN_3508_M8_ID))
        {
            continue;
        }

        motor_index = rx_header.Identifier - CAN_3508_M1_ID;
        motor[motor_index].msg_cnt++;

        if (motor[motor_index].msg_cnt <= 50U)
        {
            get_motor_offset(&motor[motor_index], rx_data);
        }
        else
        {
            get_motor_measure(&motor[motor_index], rx_data);
        }
    }
}

void FDCAN_Start(FDCAN_HandleTypeDef *hfdcan)
{
    uint32_t notification = 0U;

    if (hfdcan == NULL)
    {
        return;
    }

    if (HAL_FDCAN_Start(hfdcan) != HAL_OK)
    {
        Error_Handler();
    }

    if ((hfdcan == &hfdcan1) || (hfdcan == &hfdcan2))
    {
        notification = FDCAN_IT_RX_FIFO0_NEW_MESSAGE;
    }
    else if (hfdcan == &hfdcan3)
    {
        notification = FDCAN_IT_RX_FIFO1_NEW_MESSAGE;
    }
    else
    {
        return;
    }

    if (HAL_FDCAN_ActivateNotification(hfdcan, notification, 0U) != HAL_OK)
    {
        Error_Handler();
    }
}

void FDCAN1_Filter_Init(void)
{
    FDCAN_Motor_Filter_Init(&hfdcan1, FDCAN_FILTER_TO_RXFIFO0);
}

void FDCAN2_Filter_Init(void)
{
    FDCAN_Motor_Filter_Init(&hfdcan2, FDCAN_FILTER_TO_RXFIFO0);
}

void FDCAN3_Filter_Init(void)
{
    FDCAN_Motor_Filter_Init(&hfdcan3, FDCAN_FILTER_TO_RXFIFO1);
}

void FDCAN_Motor_Start_All(void)
{
    FDCAN_Start(&hfdcan1);
    FDCAN_Start(&hfdcan2);
    FDCAN_Start(&hfdcan3);
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == RESET)
    {
        return;
    }

    if (hfdcan == &hfdcan1)
    {
        FDCAN_Process_Motor_Rx(hfdcan, FDCAN_RX_FIFO0, motor_fdcan1);
    }
    else if (hfdcan == &hfdcan2)
    {
        FDCAN_Process_Motor_Rx(hfdcan, FDCAN_RX_FIFO0, motor_fdcan2);
    }
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
    if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) == RESET)
    {
        return;
    }

    if (hfdcan == &hfdcan3)
    {
        FDCAN_Process_Motor_Rx(hfdcan, FDCAN_RX_FIFO1, motor_fdcan3);
    }
}
