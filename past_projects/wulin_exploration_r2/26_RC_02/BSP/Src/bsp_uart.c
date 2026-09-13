#include "bsp_uart.h"
#include "imu.h"
#include "l1_laser.h"
#include "CRC.h"
#include "arm_echo_uart10.h"
#include "usart.h"

extern uint8_t btReceiveData;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART10)
    {
        UART10_Receive(btReceiveData);

        if (HAL_UART_Receive_IT(&huart10, &btReceiveData, 1) != HAL_OK)
        {
            Error_Handler();
        }
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == UART7)
    {
        IMU_RxDmaEventCallback(Size);
    }
    else if ((huart->Instance == USART1) ||
             (huart->Instance == UART8) ||
             (huart->Instance == UART9))
    {
        L1_Laser_RxEventCallback(huart, Size);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART7)
    {
        IMU_RestartDmaReceive();
    }

    if (huart->Instance == USART10)
    {
        HAL_UART_Receive_IT(&huart10, &btReceiveData, 1);
        ArmEchoUart10_ErrorHandler();
    }

    if ((huart->Instance == USART1) ||
        (huart->Instance == UART8) ||
        (huart->Instance == UART9))
    {
        L1_Laser_ErrorCallback(huart);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART10)
    {
        ArmEchoUart10_TxCpltHandler();
    }
}
