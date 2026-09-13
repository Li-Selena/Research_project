#include "include.h"
#include "usart.h"

#define MODBUS_MAX_FRAME 256U
#define MODBUS_MAX_READ_REGISTERS 125U
#define MODBUS_MAX_WRITE_REGISTERS 123U

static uint8_t rx_frame[MODBUS_MAX_FRAME];
static uint8_t pending_frame[MODBUS_MAX_FRAME];
static uint8_t tx_frame[MODBUS_MAX_FRAME];
static volatile uint16_t rx_length;
static volatile uint16_t expected_length;
static volatile uint16_t pending_length;
static volatile uint8_t request_pending;
static uint32_t last_rx_ms;

static uint16_t be16(const uint8_t *data)
{
    return (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
}

static void put_be16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value >> 8);
    data[1] = (uint8_t)value;
}

void modbus_rtu_init(void)
{
    rx_length = 0;
    expected_length = 0;
    pending_length = 0;
    request_pending = 0;
    last_rx_ms = 0;
}

void modbus_rtu_feed(uint8_t byte)
{
    uint32_t now = HAL_GetTick();
    uint16_t crc;
    if (rx_length && (uint32_t)(now - last_rx_ms) > MODBUS_FRAME_TIMEOUT_MS) {
        rx_length = 0;
        expected_length = 0;
    }
    last_rx_ms = now;
    if (request_pending || rx_length >= MODBUS_MAX_FRAME) {
        rx_length = 0;
        expected_length = 0;
        return;
    }
    rx_frame[rx_length++] = byte;
    if (rx_length == 2U) {
        uint8_t function = rx_frame[1];
        expected_length = function == MODBUS_FC_WRITE_MULTIPLE ? 0U : 8U;
    }
    if (rx_length == 7U && rx_frame[1] == MODBUS_FC_WRITE_MULTIPLE) {
        expected_length = (uint16_t)(9U + rx_frame[6]);
        if (expected_length > MODBUS_MAX_FRAME) {
            rx_length = 0;
            expected_length = 0;
            return;
        }
    }
    if (expected_length && rx_length == expected_length) {
        crc = modbus_crc16(rx_frame, (uint16_t)(rx_length - 2U));
        if (rx_frame[rx_length - 2U] == (uint8_t)crc &&
            rx_frame[rx_length - 1U] == (uint8_t)(crc >> 8) &&
            (rx_frame[0] == MODBUS_SLAVE_ADDRESS || rx_frame[0] == 0U)) {
            memcpy(pending_frame, rx_frame, rx_length);
            pending_length = rx_length;
            request_pending = 1;
        }
        rx_length = 0;
        expected_length = 0;
    }
}

static void send_response(uint16_t length)
{
    uint16_t crc = modbus_crc16(tx_frame, length);
    tx_frame[length] = (uint8_t)crc;
    tx_frame[length + 1U] = (uint8_t)(crc >> 8);
    (void)HAL_UART_Transmit_IT(&huart1, tx_frame, (uint16_t)(length + 2U));
}

static void send_exception(uint8_t function, uint8_t exception)
{
    tx_frame[0] = MODBUS_SLAVE_ADDRESS;
    tx_frame[1] = (uint8_t)(function | 0x80U);
    tx_frame[2] = exception;
    send_response(3U);
}

static void process_read(uint8_t function, uint16_t address, uint16_t count)
{
    uint16_t registers[MODBUS_MAX_READ_REGISTERS];
    uint8_t valid;
    if (count == 0U || count > MODBUS_MAX_READ_REGISTERS) {
        send_exception(function, 0x03U);
        return;
    }
    valid = function == MODBUS_FC_READ_HOLDING ?
        robot_holding_read(address, count, registers) :
        robot_input_read(address, count, registers);
    if (!valid) {
        send_exception(function, 0x02U);
        return;
    }
    tx_frame[0] = MODBUS_SLAVE_ADDRESS;
    tx_frame[1] = function;
    tx_frame[2] = (uint8_t)(count * 2U);
    for (uint16_t i = 0; i < count; ++i) put_be16(&tx_frame[3U + 2U * i], registers[i]);
    send_response((uint16_t)(3U + 2U * count));
}

static void process_request(const uint8_t *frame, uint16_t length)
{
    uint8_t address_byte = frame[0];
    uint8_t function = frame[1];
    uint16_t address = be16(&frame[2]);
    uint16_t count_or_value = be16(&frame[4]);
    uint16_t registers[MODBUS_MAX_WRITE_REGISTERS];
    uint8_t valid;

    if (address_byte == 0U && function != MODBUS_FC_WRITE_SINGLE && function != MODBUS_FC_WRITE_MULTIPLE)
        return;
    if (function == MODBUS_FC_READ_HOLDING || function == MODBUS_FC_READ_INPUT) {
        process_read(function, address, count_or_value);
    } else if (function == MODBUS_FC_WRITE_SINGLE) {
        valid = robot_holding_write(address, 1U, &count_or_value);
        if (address_byte == 0U) return;
        if (!valid) send_exception(function, 0x03U);
        else {
            memcpy(tx_frame, frame, 6U);
            send_response(6U);
        }
    } else if (function == MODBUS_FC_WRITE_MULTIPLE) {
        uint16_t count = count_or_value;
        if (length < 9U || count == 0U || count > MODBUS_MAX_WRITE_REGISTERS ||
            frame[6] != count * 2U || length != (uint16_t)(9U + frame[6])) {
            if (address_byte != 0U) send_exception(function, 0x03U);
            return;
        }
        for (uint16_t i = 0; i < count; ++i) registers[i] = be16(&frame[7U + 2U * i]);
        valid = robot_holding_write(address, count, registers);
        if (address_byte == 0U) return;
        if (!valid) send_exception(function, 0x03U);
        else {
            tx_frame[0] = MODBUS_SLAVE_ADDRESS;
            tx_frame[1] = function;
            put_be16(&tx_frame[2], address);
            put_be16(&tx_frame[4], count);
            send_response(6U);
        }
    } else if (address_byte != 0U) {
        send_exception(function, 0x01U);
    }
}

void modbus_rtu_service(void)
{
    uint8_t local[MODBUS_MAX_FRAME];
    uint16_t length;
    uint32_t irq;
    if (!request_pending || huart1.gState != HAL_UART_STATE_READY) return;
    irq = __get_PRIMASK();
    __disable_irq();
    length = pending_length;
    memcpy(local, pending_frame, length);
    request_pending = 0;
    pending_length = 0;
    __set_PRIMASK(irq);
    process_request(local, length);
}
