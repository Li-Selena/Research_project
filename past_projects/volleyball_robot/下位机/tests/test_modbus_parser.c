#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "modbus_rtu.h"
#include "usart.h"

UART_HandleTypeDef huart1 = {HAL_UART_STATE_READY};
static uint32_t tick_ms;
static uint16_t holding[64];
static uint8_t response[256];
static uint16_t response_length;

uint32_t HAL_GetTick(void) { return tick_ms; }

HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *uart, uint8_t *data, uint16_t length)
{
    (void)uart;
    memcpy(response, data, length);
    response_length = length;
    return 0;
}

uint8_t robot_holding_read(uint16_t address, uint16_t count, uint16_t *values)
{
    if ((uint32_t)address + count > 64U) return 0;
    memcpy(values, &holding[address], count * sizeof(uint16_t));
    return 1;
}

uint8_t robot_holding_write(uint16_t address, uint16_t count, const uint16_t *values)
{
    if ((uint32_t)address + count > 64U) return 0;
    memcpy(&holding[address], values, count * sizeof(uint16_t));
    return 1;
}

uint8_t robot_input_read(uint16_t address, uint16_t count, uint16_t *values)
{
    if ((uint32_t)address + count > 64U) return 0;
    for (uint16_t i = 0; i < count; ++i) values[i] = (uint16_t)(0x2000U + address + i);
    return 1;
}

static uint16_t append_crc(uint8_t *frame, uint16_t payload_length)
{
    uint16_t crc = modbus_crc16(frame, payload_length);
    frame[payload_length] = (uint8_t)crc;
    frame[payload_length + 1U] = (uint8_t)(crc >> 8);
    return (uint16_t)(payload_length + 2U);
}

static void transact(const uint8_t *frame, uint16_t length)
{
    response_length = 0;
    for (uint16_t i = 0; i < length; ++i) {
        modbus_rtu_feed(frame[i]);
        ++tick_ms;
    }
    modbus_rtu_service();
}

static void verify_crc(void)
{
    uint16_t actual;
    assert(response_length >= 4U);
    actual = (uint16_t)(response[response_length - 2U] |
                        ((uint16_t)response[response_length - 1U] << 8));
    assert(actual == modbus_crc16(response, (uint16_t)(response_length - 2U)));
}

int main(void)
{
    uint8_t frame[32];
    uint16_t length;

    modbus_rtu_init();
    holding[2] = 0x1234U;
    holding[3] = 0xabcdU;

    memcpy(frame, (uint8_t[]){1, 3, 0, 2, 0, 2}, 6);
    length = append_crc(frame, 6);
    transact(frame, length);
    verify_crc();
    assert(response_length == 9U);
    assert(memcmp(response, (uint8_t[]){1, 3, 4, 0x12, 0x34, 0xab, 0xcd}, 7) == 0);

    memcpy(frame, (uint8_t[]){1, 4, 0, 2, 0, 2}, 6);
    length = append_crc(frame, 6);
    transact(frame, length);
    verify_crc();
    assert(memcmp(response, (uint8_t[]){1, 4, 4, 0x20, 0x02, 0x20, 0x03}, 7) == 0);

    memcpy(frame, (uint8_t[]){1, 6, 0, 5, 0x55, 0xaa}, 6);
    length = append_crc(frame, 6);
    transact(frame, length);
    verify_crc();
    assert(holding[5] == 0x55aaU);
    assert(memcmp(response, frame, length) == 0);

    memcpy(frame, (uint8_t[]){1, 0x10, 0, 6, 0, 2, 4, 0x11, 0x11, 0x22, 0x22}, 11);
    length = append_crc(frame, 11);
    transact(frame, length);
    verify_crc();
    assert(holding[6] == 0x1111U && holding[7] == 0x2222U);
    assert(memcmp(response, (uint8_t[]){1, 0x10, 0, 6, 0, 2}, 6) == 0);

    memcpy(frame, (uint8_t[]){1, 3, 0, 63, 0, 2}, 6);
    length = append_crc(frame, 6);
    transact(frame, length);
    verify_crc();
    assert(response[1] == 0x83U && response[2] == 0x02U);

    memcpy(frame, (uint8_t[]){1, 5, 0, 0, 0, 0}, 6);
    length = append_crc(frame, 6);
    transact(frame, length);
    verify_crc();
    assert(response[1] == 0x85U && response[2] == 0x01U);

    frame[length - 1U] ^= 0xffU;
    transact(frame, length);
    assert(response_length == 0U);

    puts("modbus parser tests passed");
    return 0;
}
