#include "include.h"
#include "usart.h"

volatile uint32_t imu_last_rx_ms;
angle_measure my_angle_measure;

static uint8_t host_rx_byte;
static uint8_t imu_rx_byte;

void uart_receivers_start(void)
{
    modbus_rtu_init();
    HAL_UART_Receive_IT(&huart1, &host_rx_byte, 1);
    HAL_UART_Receive_IT(&huart2, &imu_rx_byte, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        modbus_rtu_feed(host_rx_byte);
        HAL_UART_Receive_IT(&huart1, &host_rx_byte, 1);
    } else if (huart->Instance == USART2) {
        hwt_parser_feed(imu_rx_byte);
        HAL_UART_Receive_IT(&huart2, &imu_rx_byte, 1);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    HAL_UART_AbortReceive(huart);
    if (huart->Instance == USART1) HAL_UART_Receive_IT(&huart1, &host_rx_byte, 1);
    else if (huart->Instance == USART2) HAL_UART_Receive_IT(&huart2, &imu_rx_byte, 1);
}

uint8_t host_input_fresh(void) { return robot_host_online(); }

uint8_t imu_input_fresh(void)
{
    return imu_last_rx_ms != 0U &&
           (uint32_t)(HAL_GetTick() - imu_last_rx_ms) <= IMU_TIMEOUT_MS;
}

void uart_service_tx(void) { modbus_rtu_service(); }
