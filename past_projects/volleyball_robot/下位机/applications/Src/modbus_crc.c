#include "modbus_rtu.h"

uint16_t modbus_crc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xffffU;
    for (uint16_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8U; ++bit)
            crc = (crc & 1U) ? (uint16_t)((crc >> 1) ^ 0xa001U) : (uint16_t)(crc >> 1);
    }
    return crc;
}
