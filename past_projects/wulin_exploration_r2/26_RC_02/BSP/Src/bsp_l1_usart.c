#include "bsp_l1_usart.h"
#include "usart.h"
#include <string.h>

static uint8_t s_l1_rx_buf[BSP_L1_USART_COUNT][BSP_L1_USART_RX_BUF_LEN];
static UART_HandleTypeDef *const s_l1_uart[BSP_L1_USART_COUNT] = {
    &huart1,
    &huart8,
    &huart9
};

static uint8_t BSP_L1Usart_IsValidId(BSP_L1UsartId_t id)
{
    return ((uint32_t)id < BSP_L1_USART_COUNT) ? 1U : 0U;
}

HAL_StatusTypeDef BSP_L1Usart_StartReceive(BSP_L1UsartId_t id)
{
    UART_HandleTypeDef *huart;

    if (BSP_L1Usart_IsValidId(id) == 0U)
    {
        return HAL_ERROR;
    }

    huart = s_l1_uart[id];
    memset(s_l1_rx_buf[id], 0, BSP_L1_USART_RX_BUF_LEN);

    return HAL_UARTEx_ReceiveToIdle_IT(huart, s_l1_rx_buf[id], BSP_L1_USART_RX_BUF_LEN);
}

HAL_StatusTypeDef BSP_L1Usart_Transmit(BSP_L1UsartId_t id, const uint8_t *data, uint16_t len, uint32_t timeout_ms)
{
    if ((BSP_L1Usart_IsValidId(id) == 0U) || (data == NULL) || (len == 0U))
    {
        return HAL_ERROR;
    }

    return HAL_UART_Transmit(s_l1_uart[id], (uint8_t *)data, len, timeout_ms);
}

uint8_t *BSP_L1Usart_GetRxBuffer(BSP_L1UsartId_t id)
{
    if (BSP_L1Usart_IsValidId(id) == 0U)
    {
        return NULL;
    }

    return s_l1_rx_buf[id];
}

BSP_L1UsartId_t BSP_L1Usart_GetIdByHandle(UART_HandleTypeDef *huart)
{
    if (huart == NULL)
    {
        return BSP_L1_USART_INVALID;
    }

    if (huart->Instance == USART1)
    {
        return BSP_L1_USART_X_POS;
    }
    if (huart->Instance == UART8)
    {
        return BSP_L1_USART_Y_POS;
    }
    if (huart->Instance == UART9)
    {
        return BSP_L1_USART_HEIGHT;
    }

    return BSP_L1_USART_INVALID;
}

uint8_t BSP_L1Usart_IsL1Uart(UART_HandleTypeDef *huart)
{
    return (BSP_L1Usart_GetIdByHandle(huart) != BSP_L1_USART_INVALID) ? 1U : 0U;
}
