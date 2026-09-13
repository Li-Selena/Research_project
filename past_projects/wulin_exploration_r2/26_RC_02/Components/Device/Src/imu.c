#include "imu.h"
#include <string.h>

extern UART_HandleTypeDef huart7;

#define IMU_RX_DMA_BUF_LEN 256U
#define IMU_OUTPUT_RATE    RRATE_200HZ
#define IMU_DMA_AXI_BASE   0x2401FF00U

IMU_Data_t imu_data = {0};
IMU_Debug_t g_imu_debug = {0};
uint8_t imu_rx_byte = 0;  /* Kept for old external references. */

static uint8_t imu_rx_dma_reserve[IMU_RX_DMA_BUF_LEN] __attribute__((section(".imu_dma"), aligned(32), used));
static uint8_t *const imu_rx_dma_buf = (uint8_t *)IMU_DMA_AXI_BASE;
static volatile uint16_t imu_rx_dma_read_pos = 0;
static volatile uint8_t imu_rx_dma_started = 0;
static volatile uint8_t imu_initialized = 0;

#ifndef AX
#define AX          0x00
#define AY          0x01
#define AZ          0x02
#define GX          0x03
#define GY          0x04
#define GZ          0x05
#define Roll        0x14
#define Pitch       0x15
#define Yaw         0x16
#endif

static void IMU_SerialWrite(uint8_t *pData, uint32_t len);
static void IMU_DelayMs(uint32_t ms);
static void IMU_RegUpdateCallback(uint32_t uiReg, uint32_t uiLen);
static HAL_StatusTypeDef IMU_StartDmaReceive(void);
static void IMU_ProcessDmaRange(uint16_t begin, uint16_t end);
static uint8_t IMU_RegRangeContains(uint32_t first_reg, uint32_t reg_num, uint32_t wanted_reg);

void IMU_Init(void)
{
    if (imu_initialized != 0U)
    {
        return;
    }

    WitSerialWriteRegister(IMU_SerialWrite);
    WitRegisterCallBack(IMU_RegUpdateCallback);
    WitDelayMsRegister((DelaymsCb)IMU_DelayMs);

    if (WitInit(WIT_PROTOCOL_NORMAL, 0xFF) != WIT_HAL_OK)
    {
        Error_Handler();
    }

    if (IMU_StartDmaReceive() != HAL_OK)
    {
        Error_Handler();
    }

    if (WitSetOutputRate(IMU_OUTPUT_RATE) != WIT_HAL_OK)
    {
        Error_Handler();
    }

    if (WitSetContent(RSW_ACC | RSW_GYRO | RSW_ANGLE) != WIT_HAL_OK)
    {
        Error_Handler();
    }

    imu_initialized = 1U;
    g_imu_debug.initialized = 1U;
}

void IMU_RxDmaEventCallback(uint16_t size)
{
    uint16_t write_pos = size;

    if ((imu_rx_dma_started == 0U) || (write_pos > IMU_RX_DMA_BUF_LEN))
    {
        g_imu_debug.dma_rx_overrun_count++;
        return;
    }

    g_imu_debug.dma_rx_event_count++;
    g_imu_debug.last_dma_size = size;
    g_imu_debug.last_dma_read_pos = imu_rx_dma_read_pos;
    g_imu_debug.dma_event_type = HAL_UARTEx_GetRxEventType(&huart7);
    if (huart7.hdmarx != NULL)
    {
        g_imu_debug.dma_ndtr = __HAL_DMA_GET_COUNTER(huart7.hdmarx);
    }

    SCB_InvalidateDCache_by_Addr((uint32_t *)imu_rx_dma_buf, IMU_RX_DMA_BUF_LEN);

    if (write_pos == imu_rx_dma_read_pos)
    {
        return;
    }

    if (write_pos > imu_rx_dma_read_pos)
    {
        g_imu_debug.dma_rx_byte_count += (uint32_t)(write_pos - imu_rx_dma_read_pos);
        IMU_ProcessDmaRange(imu_rx_dma_read_pos, write_pos);
    }
    else
    {
        g_imu_debug.dma_rx_byte_count += (uint32_t)(IMU_RX_DMA_BUF_LEN - imu_rx_dma_read_pos + write_pos);
        IMU_ProcessDmaRange(imu_rx_dma_read_pos, IMU_RX_DMA_BUF_LEN);
        if (write_pos > 0U)
        {
            IMU_ProcessDmaRange(0U, write_pos);
        }
    }

    imu_rx_dma_read_pos = write_pos;
    if (imu_rx_dma_read_pos >= IMU_RX_DMA_BUF_LEN)
    {
        imu_rx_dma_read_pos = 0U;
    }
}

void IMU_RestartDmaReceive(void)
{
    g_imu_debug.dma_restart_count++;
    g_imu_debug.last_error_code = (uint8_t)huart7.ErrorCode;
    imu_rx_dma_started = 0U;
    (void)HAL_UART_DMAStop(&huart7);
    (void)HAL_UART_AbortReceive(&huart7);
    if (IMU_StartDmaReceive() != HAL_OK)
    {
        g_imu_debug.dma_restart_fail_count++;
    }
}

void IMU_ParseData(void)
{
    /* Data is parsed byte-by-byte by WitSerialDataIn() from DMA RX events. */
}

static HAL_StatusTypeDef IMU_StartDmaReceive(void)
{
    HAL_StatusTypeDef status;

    imu_rx_dma_read_pos = 0U;
    g_imu_debug.dma_buf_addr = (uint32_t)imu_rx_dma_buf;
    g_imu_debug.rx_mode = 2U;
    memset(imu_rx_dma_buf, 0, IMU_RX_DMA_BUF_LEN);
    SCB_CleanInvalidateDCache_by_Addr((uint32_t *)imu_rx_dma_buf, IMU_RX_DMA_BUF_LEN);

    status = HAL_UARTEx_ReceiveToIdle_DMA(&huart7, imu_rx_dma_buf, IMU_RX_DMA_BUF_LEN);
    if (status == HAL_OK)
    {
        imu_rx_dma_started = 1U;
        g_imu_debug.dma_started = 1U;
        g_imu_debug.dma_start_count++;
        if (huart7.hdmarx != NULL)
        {
            __HAL_DMA_DISABLE_IT(huart7.hdmarx, DMA_IT_HT);
            g_imu_debug.dma_ndtr = __HAL_DMA_GET_COUNTER(huart7.hdmarx);
        }
    }
    else
    {
        imu_rx_dma_started = 0U;
        g_imu_debug.dma_started = 0U;
    }

    return status;
}

static void IMU_ProcessDmaRange(uint16_t begin, uint16_t end)
{
    while (begin < end)
    {
        imu_rx_byte = imu_rx_dma_buf[begin];
        g_imu_debug.last_rx_byte = imu_rx_byte;
        WitSerialDataIn(imu_rx_byte);
        begin++;
    }
}

static void IMU_SerialWrite(uint8_t *pData, uint32_t len)
{
    (void)HAL_UART_Transmit(&huart7, pData, (uint16_t)len, 20U);
}

static void IMU_DelayMs(uint32_t ms)
{
    Delay_ms(ms);
}

static void IMU_RegUpdateCallback(uint32_t uiReg, uint32_t uiLen)
{
    uint32_t primask;
    uint32_t now_tick;

    now_tick = HAL_GetTick();
    primask = __get_PRIMASK();
    __disable_irq();

    g_imu_debug.reg_update_count++;
    g_imu_debug.last_update_tick = now_tick;
    g_imu_debug.last_reg = uiReg;
    g_imu_debug.last_reg_num = uiLen;

    if (IMU_RegRangeContains(uiReg, uiLen, AX) != 0U)
    {
        g_imu_debug.raw_acc[0] = sReg[AX];
        g_imu_debug.raw_acc[1] = sReg[AY];
        g_imu_debug.raw_acc[2] = sReg[AZ];
        imu_data.acc_x = (float)sReg[AX] / 32768.0f * 16.0f;
        imu_data.acc_y = (float)sReg[AY] / 32768.0f * 16.0f;
        imu_data.acc_z = (float)sReg[AZ] / 32768.0f * 16.0f;
        imu_data.last_update_tick = now_tick;
        imu_data.online = 1U;
        imu_data.update_flag = 1U;
        g_imu_debug.acc_update_count++;
    }

    if (IMU_RegRangeContains(uiReg, uiLen, GX) != 0U)
    {
        g_imu_debug.raw_gyro[0] = sReg[GX];
        g_imu_debug.raw_gyro[1] = sReg[GY];
        g_imu_debug.raw_gyro[2] = sReg[GZ];
        imu_data.gyro_x = (float)sReg[GX] / 32768.0f * 2000.0f;
        imu_data.gyro_y = (float)sReg[GY] / 32768.0f * 2000.0f;
        imu_data.gyro_z = (float)sReg[GZ] / 32768.0f * 2000.0f;
        imu_data.last_update_tick = now_tick;
        imu_data.online = 1U;
        imu_data.update_flag = 1U;
        g_imu_debug.gyro_update_count++;
    }

    if (IMU_RegRangeContains(uiReg, uiLen, Roll) != 0U)
    {
        g_imu_debug.raw_angle[0] = sReg[Roll];
        g_imu_debug.raw_angle[1] = sReg[Pitch];
        g_imu_debug.raw_angle[2] = sReg[Yaw];
        imu_data.roll = (float)sReg[Roll] / 32768.0f * 180.0f;
        imu_data.pitch = (float)sReg[Pitch] / 32768.0f * 180.0f;
        imu_data.yaw = (float)sReg[Yaw] / 32768.0f * 180.0f;
        imu_data.last_update_tick = now_tick;
        imu_data.online = 1U;
        imu_data.update_flag = 1U;
        g_imu_debug.angle_update_count++;
    }

    if (primask == 0U)
    {
        __enable_irq();
    }
}

static uint8_t IMU_RegRangeContains(uint32_t first_reg, uint32_t reg_num, uint32_t wanted_reg)
{
    return ((wanted_reg >= first_reg) && (wanted_reg < (first_reg + reg_num))) ? 1U : 0U;
}
