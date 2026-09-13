#ifndef MODBUS_RTU_H
#define MODBUS_RTU_H

#include <stdint.h>

#define MODBUS_FC_READ_HOLDING 0x03U
#define MODBUS_FC_READ_INPUT 0x04U
#define MODBUS_FC_WRITE_SINGLE 0x06U
#define MODBUS_FC_WRITE_MULTIPLE 0x10U

void modbus_rtu_init(void);
void modbus_rtu_feed(uint8_t byte);
void modbus_rtu_service(void);
uint16_t modbus_crc16(const uint8_t *data, uint16_t length);

#endif
