#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "modbus_rtu.h"

int main(void)
{
    const uint8_t request[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x0a};
    assert(modbus_crc16(request, sizeof(request)) == 0xcdc5U);
    puts("modbus CRC test passed");
    return 0;
}
