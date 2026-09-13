#include "l1_laser.h"
#include "bsp_l1_usart.h"
#include <string.h>

#ifndef L1_LASER_RESPONSE_TIMEOUT_MS
#define L1_LASER_RESPONSE_TIMEOUT_MS 500U
#endif

#ifndef L1_LASER_TX_TIMEOUT_MS
#define L1_LASER_TX_TIMEOUT_MS 5U
#endif

#ifndef L1_LASER_CHANNEL_MIN_INTERVAL_MS
#define L1_LASER_CHANNEL_MIN_INTERVAL_MS 300U
#endif

#ifndef L1_LASER_OFFLINE_TIMEOUT_MS
#define L1_LASER_OFFLINE_TIMEOUT_MS 1000U
#endif

#ifndef L1_LASER_RAW_REG_TO_MM_DIV
#define L1_LASER_RAW_REG_TO_MM_DIV 1U
#endif

#if (L1_LASER_RAW_REG_TO_MM_DIV == 0U)
#error "L1_LASER_RAW_REG_TO_MM_DIV must not be 0"
#endif

#define L1_MODBUS_DEFAULT_ADDR    1U
#define L1_MODBUS_READ_REG_START  0x000FU
#define L1_MODBUS_READ_REG_COUNT  0x0002U
#define L1_MODBUS_READ_REQ_LEN    8U
#define L1_MODBUS_READ_RSP_LEN    9U
#define L1_MODBUS_EXCEPTION_RSP_LEN 5U
#define L1_MODBUS_FUNC_READ       0x03U
#define L1_MODBUS_FUNC_EXCEPTION  0x80U
#define L1_MODBUS_RSP_BYTE_COUNT  0x04U
#define L1_MODBUS_MEASURE_FAULT_MASK 0x80000000UL

L1_LaserState_t g_l1_laser_state[L1_LASER_COUNT];

static uint8_t s_l1_initialized = 0U;

static uint8_t L1_Laser_IsValidId(L1_LaserId_t id);
static uint8_t L1_Laser_IsOfflineExpired(L1_LaserId_t id, uint32_t now);
static uint16_t L1_Laser_Crc16(const uint8_t *data, uint16_t len);
static void L1_Laser_BuildReadCommand(uint8_t addr, uint8_t cmd[L1_MODBUS_READ_REQ_LEN]);
static void L1_Laser_SendReadCommand(L1_LaserId_t id);
static void L1_Laser_ParseResponse(L1_LaserId_t id, const uint8_t *buf, uint16_t len);
static void L1_Laser_MarkInvalid(L1_LaserId_t id, uint8_t force_invalid);
static int32_t L1_Laser_RawRegToMm(uint32_t raw_reg);
static L1_LaserId_t L1_Laser_IdFromBspId(BSP_L1UsartId_t bsp_id);

void L1_Laser_Init(void)
{
    uint32_t i;

    if (s_l1_initialized != 0U)
    {
        return;
    }

    memset(g_l1_laser_state, 0, sizeof(g_l1_laser_state));
    for (i = 0U; i < L1_LASER_COUNT; i++)
    {
        g_l1_laser_state[i].distance_mm = L1_LASER_INVALID_MM;
        g_l1_laser_state[i].raw_distance_mm = L1_LASER_INVALID_MM;
        g_l1_laser_state[i].modbus_addr = L1_MODBUS_DEFAULT_ADDR;
        (void)BSP_L1Usart_StartReceive((BSP_L1UsartId_t)i);
    }

    s_l1_initialized = 1U;
}

void L1_Laser_SetOffsetMm(L1_LaserId_t id, int32_t offset_mm)
{
    uint32_t primask;

    if (L1_Laser_IsValidId(id) == 0U)
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    g_l1_laser_state[id].offset_mm = offset_mm;
    if (g_l1_laser_state[id].raw_distance_mm >= 0)
    {
        g_l1_laser_state[id].distance_mm = g_l1_laser_state[id].raw_distance_mm - offset_mm;
    }
    if (primask == 0U)
    {
        __enable_irq();
    }
}

void L1_Laser_SetModbusAddr(L1_LaserId_t id, uint8_t addr)
{
    if ((L1_Laser_IsValidId(id) == 0U) || (addr == 0U))
    {
        return;
    }

    g_l1_laser_state[id].modbus_addr = addr;
}

void L1_Laser_Poll10ms(void)
{
    uint32_t i;

    if (s_l1_initialized == 0U)
    {
        L1_Laser_Init();
    }

    L1_Laser_CheckTimeouts();
    for (i = 0U; i < L1_LASER_COUNT; i++)
    {
        L1_Laser_SendReadCommand((L1_LaserId_t)i);
    }
}

void L1_Laser_CheckTimeouts(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t i;

    for (i = 0U; i < L1_LASER_COUNT; i++)
    {
        if ((g_l1_laser_state[i].waiting_response != 0U) &&
            ((now - g_l1_laser_state[i].last_request_tick) > L1_LASER_RESPONSE_TIMEOUT_MS))
        {
            g_l1_laser_state[i].timeout_count++;
            g_l1_laser_state[i].last_error_tick = now;
            g_l1_laser_state[i].waiting_response = 0U;
            L1_Laser_MarkInvalid((L1_LaserId_t)i, 0U);
        }
    }
}

void L1_Laser_GetState(L1_LaserId_t id, L1_LaserState_t *out)
{
    uint32_t primask;

    if ((L1_Laser_IsValidId(id) == 0U) || (out == NULL))
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    *out = g_l1_laser_state[id];
    if (primask == 0U)
    {
        __enable_irq();
    }
}

void L1_Laser_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    BSP_L1UsartId_t bsp_id = BSP_L1Usart_GetIdByHandle(huart);
    L1_LaserId_t id = L1_Laser_IdFromBspId(bsp_id);
    uint8_t *buf;

    if (L1_Laser_IsValidId(id) == 0U)
    {
        return;
    }

    buf = BSP_L1Usart_GetRxBuffer(bsp_id);
    if ((buf == NULL) || (size == 0U) || (size > BSP_L1_USART_RX_BUF_LEN))
    {
        g_l1_laser_state[id].parse_error_count++;
        L1_Laser_MarkInvalid(id, 0U);
    }
    else
    {
        L1_Laser_ParseResponse(id, buf, size);
    }

    (void)BSP_L1Usart_StartReceive(bsp_id);
}

void L1_Laser_ErrorCallback(UART_HandleTypeDef *huart)
{
    BSP_L1UsartId_t bsp_id = BSP_L1Usart_GetIdByHandle(huart);
    L1_LaserId_t id = L1_Laser_IdFromBspId(bsp_id);

    if (L1_Laser_IsValidId(id) == 0U)
    {
        return;
    }

    g_l1_laser_state[id].uart_error_count++;
    g_l1_laser_state[id].last_error_code = huart->ErrorCode;
    g_l1_laser_state[id].last_error_tick = HAL_GetTick();
    g_l1_laser_state[id].waiting_response = 0U;
    L1_Laser_MarkInvalid(id, 0U);

    (void)HAL_UART_AbortReceive(huart);
    (void)BSP_L1Usart_StartReceive(bsp_id);
}

static uint8_t L1_Laser_IsValidId(L1_LaserId_t id)
{
    return ((uint32_t)id < L1_LASER_COUNT) ? 1U : 0U;
}

static uint8_t L1_Laser_IsOfflineExpired(L1_LaserId_t id, uint32_t now)
{
    if (g_l1_laser_state[id].response_count == 0U)
    {
        return 1U;
    }

    return ((now - g_l1_laser_state[id].last_update_tick) > L1_LASER_OFFLINE_TIMEOUT_MS) ? 1U : 0U;
}

static uint16_t L1_Laser_Crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFU;
    uint16_t i;
    uint8_t bit;

    for (i = 0U; i < len; i++)
    {
        crc ^= data[i];
        for (bit = 0U; bit < 8U; bit++)
        {
            if ((crc & 0x0001U) != 0U)
            {
                crc = (uint16_t)((crc >> 1U) ^ 0xA001U);
            }
            else
            {
                crc = (uint16_t)(crc >> 1U);
            }
        }
    }

    return crc;
}

static void L1_Laser_BuildReadCommand(uint8_t addr, uint8_t cmd[L1_MODBUS_READ_REQ_LEN])
{
    uint16_t crc;

    cmd[0] = addr;
    cmd[1] = L1_MODBUS_FUNC_READ;
    cmd[2] = (uint8_t)(L1_MODBUS_READ_REG_START >> 8U);
    cmd[3] = (uint8_t)(L1_MODBUS_READ_REG_START & 0xFFU);
    cmd[4] = (uint8_t)(L1_MODBUS_READ_REG_COUNT >> 8U);
    cmd[5] = (uint8_t)(L1_MODBUS_READ_REG_COUNT & 0xFFU);

    crc = L1_Laser_Crc16(cmd, 6U);
    cmd[6] = (uint8_t)(crc & 0xFFU);
    cmd[7] = (uint8_t)(crc >> 8U);
}

static void L1_Laser_SendReadCommand(L1_LaserId_t id)
{
    uint8_t cmd[L1_MODBUS_READ_REQ_LEN];
    uint32_t now = HAL_GetTick();
    HAL_StatusTypeDef status;

    if (L1_Laser_IsValidId(id) == 0U)
    {
        return;
    }

    if (g_l1_laser_state[id].waiting_response != 0U)
    {
        return;
    }
    if ((g_l1_laser_state[id].request_count != 0U) &&
        ((now - g_l1_laser_state[id].last_request_tick) < L1_LASER_CHANNEL_MIN_INTERVAL_MS))
    {
        return;
    }

    L1_Laser_BuildReadCommand(g_l1_laser_state[id].modbus_addr, cmd);

    g_l1_laser_state[id].waiting_response = 1U;
    g_l1_laser_state[id].last_request_tick = now;
    g_l1_laser_state[id].request_count++;

    status = BSP_L1Usart_Transmit((BSP_L1UsartId_t)id, cmd, L1_MODBUS_READ_REQ_LEN, L1_LASER_TX_TIMEOUT_MS);
    if (status != HAL_OK)
    {
        g_l1_laser_state[id].waiting_response = 0U;
        g_l1_laser_state[id].uart_error_count++;
        g_l1_laser_state[id].last_error_tick = HAL_GetTick();
        L1_Laser_MarkInvalid(id, 0U);
    }
}

static void L1_Laser_ParseResponse(L1_LaserId_t id, const uint8_t *buf, uint16_t len)
{
    uint16_t pos;
    uint16_t crc_calc;
    uint16_t crc_recv;
    uint32_t raw_reg;
    int32_t raw_mm;
    int32_t corrected_mm;
    uint8_t addr = g_l1_laser_state[id].modbus_addr;

    g_l1_laser_state[id].last_rx_len = len;

    for (pos = 0U; (uint16_t)(pos + L1_MODBUS_EXCEPTION_RSP_LEN) <= len; pos++)
    {
        if ((buf[pos] != addr) ||
            (buf[pos + 1U] != (uint8_t)(L1_MODBUS_FUNC_READ | L1_MODBUS_FUNC_EXCEPTION)))
        {
            continue;
        }

        crc_calc = L1_Laser_Crc16(&buf[pos], 3U);
        crc_recv = (uint16_t)buf[pos + 3U] | ((uint16_t)buf[pos + 4U] << 8U);
        if (crc_calc != crc_recv)
        {
            g_l1_laser_state[id].crc_error_count++;
            g_l1_laser_state[id].last_error_tick = HAL_GetTick();
            g_l1_laser_state[id].waiting_response = 0U;
            L1_Laser_MarkInvalid(id, 0U);
            return;
        }

        g_l1_laser_state[id].parse_error_count++;
        g_l1_laser_state[id].last_error_code = buf[pos + 2U];
        g_l1_laser_state[id].last_error_tick = HAL_GetTick();
        g_l1_laser_state[id].waiting_response = 0U;
        L1_Laser_MarkInvalid(id, 1U);
        return;
    }

    for (pos = 0U; (uint16_t)(pos + L1_MODBUS_READ_RSP_LEN) <= len; pos++)
    {
        if ((buf[pos] != addr) ||
            (buf[pos + 1U] != L1_MODBUS_FUNC_READ) ||
            (buf[pos + 2U] != L1_MODBUS_RSP_BYTE_COUNT))
        {
            continue;
        }

        crc_calc = L1_Laser_Crc16(&buf[pos], 7U);
        crc_recv = (uint16_t)buf[pos + 7U] | ((uint16_t)buf[pos + 8U] << 8U);
        if (crc_calc != crc_recv)
        {
            g_l1_laser_state[id].crc_error_count++;
            g_l1_laser_state[id].last_error_tick = HAL_GetTick();
            g_l1_laser_state[id].waiting_response = 0U;
            L1_Laser_MarkInvalid(id, 0U);
            return;
        }

        raw_reg = ((uint32_t)buf[pos + 3U] << 24U) |
                  ((uint32_t)buf[pos + 4U] << 16U) |
                  ((uint32_t)buf[pos + 5U] << 8U) |
                  ((uint32_t)buf[pos + 6U]);
        if ((raw_reg & L1_MODBUS_MEASURE_FAULT_MASK) != 0U)
        {
            g_l1_laser_state[id].parse_error_count++;
            g_l1_laser_state[id].last_error_code = raw_reg & (~L1_MODBUS_MEASURE_FAULT_MASK);
            g_l1_laser_state[id].last_error_tick = HAL_GetTick();
            g_l1_laser_state[id].waiting_response = 0U;
            L1_Laser_MarkInvalid(id, 1U);
            return;
        }

        raw_mm = L1_Laser_RawRegToMm(raw_reg);
        corrected_mm = raw_mm - g_l1_laser_state[id].offset_mm;

        g_l1_laser_state[id].raw_distance_mm = raw_mm;
        g_l1_laser_state[id].distance_mm = corrected_mm;
        g_l1_laser_state[id].valid = 1U;
        g_l1_laser_state[id].online = 1U;
        g_l1_laser_state[id].waiting_response = 0U;
        g_l1_laser_state[id].last_update_tick = HAL_GetTick();
        g_l1_laser_state[id].response_count++;
        return;
    }

    g_l1_laser_state[id].parse_error_count++;
    g_l1_laser_state[id].last_error_tick = HAL_GetTick();
    g_l1_laser_state[id].waiting_response = 0U;
    L1_Laser_MarkInvalid(id, 0U);
}

static void L1_Laser_MarkInvalid(L1_LaserId_t id, uint8_t force_invalid)
{
    uint32_t now = HAL_GetTick();
    uint8_t offline_expired;

    if (L1_Laser_IsValidId(id) == 0U)
    {
        return;
    }

    offline_expired = L1_Laser_IsOfflineExpired(id, now);
    if ((force_invalid != 0U) || (offline_expired != 0U))
    {
        g_l1_laser_state[id].valid = 0U;
    }
    if (offline_expired != 0U)
    {
        g_l1_laser_state[id].distance_mm = L1_LASER_INVALID_MM;
        g_l1_laser_state[id].raw_distance_mm = L1_LASER_INVALID_MM;
        g_l1_laser_state[id].online = 0U;
    }
}

static int32_t L1_Laser_RawRegToMm(uint32_t raw_reg)
{
#if (L1_LASER_RAW_REG_TO_MM_DIV == 1U)
    return (int32_t)raw_reg;
#else
    return (int32_t)((raw_reg + (L1_LASER_RAW_REG_TO_MM_DIV / 2U)) / L1_LASER_RAW_REG_TO_MM_DIV);
#endif
}

static L1_LaserId_t L1_Laser_IdFromBspId(BSP_L1UsartId_t bsp_id)
{
    if ((uint32_t)bsp_id >= L1_LASER_COUNT)
    {
        return (L1_LaserId_t)L1_LASER_COUNT;
    }

    return (L1_LaserId_t)bsp_id;
}
