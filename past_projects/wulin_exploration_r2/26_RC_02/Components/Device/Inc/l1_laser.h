#ifndef __L1_LASER_H
#define __L1_LASER_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

#define L1_LASER_COUNT      3U
#define L1_LASER_INVALID_MM (-1)

typedef enum {
    L1_LASER_X_POS = 0,
    L1_LASER_Y_POS,
    L1_LASER_HEIGHT
} L1_LaserId_t;

typedef struct {
    int32_t distance_mm;
    int32_t raw_distance_mm;
    int32_t offset_mm;
    uint32_t last_request_tick;
    uint32_t last_update_tick;
    uint32_t last_error_tick;
    uint32_t request_count;
    uint32_t response_count;
    uint32_t timeout_count;
    uint32_t crc_error_count;
    uint32_t parse_error_count;
    uint32_t uart_error_count;
    uint16_t last_rx_len;
    uint8_t modbus_addr;
    uint8_t valid;
    uint8_t online;
    uint8_t waiting_response;
    uint32_t last_error_code;
} L1_LaserState_t;

extern L1_LaserState_t g_l1_laser_state[L1_LASER_COUNT];

void L1_Laser_Init(void);
void L1_Laser_SetOffsetMm(L1_LaserId_t id, int32_t offset_mm);
void L1_Laser_SetModbusAddr(L1_LaserId_t id, uint8_t addr);
void L1_Laser_Poll10ms(void);
void L1_Laser_CheckTimeouts(void);
void L1_Laser_GetState(L1_LaserId_t id, L1_LaserState_t *out);
void L1_Laser_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size);
void L1_Laser_ErrorCallback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* __L1_LASER_H */
