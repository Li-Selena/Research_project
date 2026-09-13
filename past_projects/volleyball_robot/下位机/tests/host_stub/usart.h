#ifndef MODBUS_TEST_USART_H
#define MODBUS_TEST_USART_H

#include <stdint.h>

typedef struct {
    uint32_t gState;
} UART_HandleTypeDef;

typedef int HAL_StatusTypeDef;
#define HAL_UART_STATE_READY 0U

extern UART_HandleTypeDef huart1;
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *uart, uint8_t *data, uint16_t length);

#endif
