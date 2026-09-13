#ifndef __R2_LASER_USER_H
#define __R2_LASER_USER_H

#include "l1_laser.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef R2_LASER_X_POS_MODBUS_ADDR
#define R2_LASER_X_POS_MODBUS_ADDR 1U
#endif

#ifndef R2_LASER_Y_POS_MODBUS_ADDR
#define R2_LASER_Y_POS_MODBUS_ADDR 2U
#endif

#ifndef R2_LASER_HEIGHT_MODBUS_ADDR
#define R2_LASER_HEIGHT_MODBUS_ADDR 3U
#endif

#ifndef R2_LASER_X_POS_OFFSET_MM
#define R2_LASER_X_POS_OFFSET_MM 0
#endif

#ifndef R2_LASER_Y_POS_OFFSET_MM
#define R2_LASER_Y_POS_OFFSET_MM 0
#endif

#ifndef R2_LASER_HEIGHT_OFFSET_MM
#define R2_LASER_HEIGHT_OFFSET_MM 0
#endif

#define R2_LASER_POLL_PERIOD_MS 10U

/* x_pos/y_pos are named range channels, NOT signed odometry coordinates.
 * X is the climb approach range; distances and height remain positive mm.
 * Sensor channels retain wiring IDs; coordinate migration does not swap them. */
typedef struct {
    int32_t x_pos_mm;
    int32_t y_pos_mm;
    int32_t height_mm;
    int32_t x_pos_raw_mm;
    int32_t y_pos_raw_mm;
    int32_t height_raw_mm;
    uint8_t x_pos_valid;
    uint8_t y_pos_valid;
    uint8_t height_valid;
    uint8_t x_pos_online;
    uint8_t y_pos_online;
    uint8_t height_online;
    uint8_t all_valid;
    uint8_t all_online;
    uint32_t update_tick;
    L1_LaserState_t channel[L1_LASER_COUNT];
} R2_LaserMeasure_t;

extern R2_LaserMeasure_t g_r2_laser_measure;

void R2_LaserUser_Init(void);
void R2_LaserUser_Poll10ms(void);
void R2_LaserUser_GetMeasure(R2_LaserMeasure_t *out);
void R2_LaserUser_SetOffsetsMm(int32_t x_pos_offset_mm, int32_t y_pos_offset_mm, int32_t height_offset_mm);

#ifdef __cplusplus
}
#endif

#endif /* __R2_LASER_USER_H */
