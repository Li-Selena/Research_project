#include "bsp_can.h"
#include "can.h"

void CAN_Start(CAN_HandleTypeDef *hcan)
{
    if (HAL_CAN_Start(hcan) != HAL_OK ||
        HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
        Error_Handler();
}
static void filter_init(CAN_HandleTypeDef *hcan, uint32_t bank)
{
    CAN_FilterTypeDef filter = {0};
    filter.FilterActivation = ENABLE;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterBank = bank;
    filter.SlaveStartFilterBank = 14;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    if (HAL_CAN_ConfigFilter(hcan, &filter) != HAL_OK) Error_Handler();
}
void CAN1_Filter_Init(void) { filter_init(&hcan1, 0); }
void CAN2_Filter_Init(void) { filter_init(&hcan2, 14); }
