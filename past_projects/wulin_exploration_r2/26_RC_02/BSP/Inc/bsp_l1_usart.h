#ifndef __BSP_L1_USART_H
#define __BSP_L1_USART_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BSP_L1_USART_COUNT      3U
#define BSP_L1_USART_RX_BUF_LEN 32U

typedef enum {
    BSP_L1_USART_X_POS = 0,
    BSP_L1_USART_Y_POS,
    BSP_L1_USART_HEIGHT,
    BSP_L1_USART_INVALID = 0xFF
} BSP_L1UsartId_t;

HAL_StatusTypeDef BSP_L1Usart_StartReceive(BSP_L1UsartId_t id);
HAL_StatusTypeDef BSP_L1Usart_Transmit(BSP_L1UsartId_t id, const uint8_t *data, uint16_t len, uint32_t timeout_ms);
uint8_t *BSP_L1Usart_GetRxBuffer(BSP_L1UsartId_t id);
BSP_L1UsartId_t BSP_L1Usart_GetIdByHandle(UART_HandleTypeDef *huart);
uint8_t BSP_L1Usart_IsL1Uart(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_L1_USART_H */
