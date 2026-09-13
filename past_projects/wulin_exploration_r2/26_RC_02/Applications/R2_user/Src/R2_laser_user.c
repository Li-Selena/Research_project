#include "R2_laser_user.h"
#include <string.h>

R2_LaserMeasure_t g_r2_laser_measure = {
    .x_pos_mm = L1_LASER_INVALID_MM,
    .y_pos_mm = L1_LASER_INVALID_MM,
    .height_mm = L1_LASER_INVALID_MM,
    .x_pos_raw_mm = L1_LASER_INVALID_MM,
    .y_pos_raw_mm = L1_LASER_INVALID_MM,
    .height_raw_mm = L1_LASER_INVALID_MM
};

static uint8_t s_r2_laser_initialized = 0U;

static void R2_LaserUser_SyncSnapshot(void);

void R2_LaserUser_Init(void)
{
    if (s_r2_laser_initialized != 0U)
    {
        return;
    }

    L1_Laser_Init();

    L1_Laser_SetModbusAddr(L1_LASER_X_POS, R2_LASER_X_POS_MODBUS_ADDR);
    L1_Laser_SetModbusAddr(L1_LASER_Y_POS, R2_LASER_Y_POS_MODBUS_ADDR);
    L1_Laser_SetModbusAddr(L1_LASER_HEIGHT, R2_LASER_HEIGHT_MODBUS_ADDR);

    R2_LaserUser_SetOffsetsMm(R2_LASER_X_POS_OFFSET_MM,
                              R2_LASER_Y_POS_OFFSET_MM,
                              R2_LASER_HEIGHT_OFFSET_MM);

    s_r2_laser_initialized = 1U;
    R2_LaserUser_SyncSnapshot();
}

void R2_LaserUser_Poll10ms(void)
{
    if (s_r2_laser_initialized == 0U)
    {
        R2_LaserUser_Init();
    }

    L1_Laser_Poll10ms();
    R2_LaserUser_SyncSnapshot();
}

void R2_LaserUser_GetMeasure(R2_LaserMeasure_t *out)
{
    uint32_t primask;

    if (out == NULL)
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    *out = g_r2_laser_measure;
    if (primask == 0U)
    {
        __enable_irq();
    }
}

void R2_LaserUser_SetOffsetsMm(int32_t x_pos_offset_mm, int32_t y_pos_offset_mm, int32_t height_offset_mm)
{
    L1_Laser_SetOffsetMm(L1_LASER_X_POS, x_pos_offset_mm);
    L1_Laser_SetOffsetMm(L1_LASER_Y_POS, y_pos_offset_mm);
    L1_Laser_SetOffsetMm(L1_LASER_HEIGHT, height_offset_mm);
    R2_LaserUser_SyncSnapshot();
}

static void R2_LaserUser_SyncSnapshot(void)
{
    R2_LaserMeasure_t next;
    uint32_t primask;

    memset(&next, 0, sizeof(next));

    L1_Laser_GetState(L1_LASER_X_POS, &next.channel[L1_LASER_X_POS]);
    L1_Laser_GetState(L1_LASER_Y_POS, &next.channel[L1_LASER_Y_POS]);
    L1_Laser_GetState(L1_LASER_HEIGHT, &next.channel[L1_LASER_HEIGHT]);

    next.x_pos_mm = next.channel[L1_LASER_X_POS].distance_mm;
    next.y_pos_mm = next.channel[L1_LASER_Y_POS].distance_mm;
    next.height_mm = next.channel[L1_LASER_HEIGHT].distance_mm;

    next.x_pos_raw_mm = next.channel[L1_LASER_X_POS].raw_distance_mm;
    next.y_pos_raw_mm = next.channel[L1_LASER_Y_POS].raw_distance_mm;
    next.height_raw_mm = next.channel[L1_LASER_HEIGHT].raw_distance_mm;

    next.x_pos_valid = next.channel[L1_LASER_X_POS].valid;
    next.y_pos_valid = next.channel[L1_LASER_Y_POS].valid;
    next.height_valid = next.channel[L1_LASER_HEIGHT].valid;

    next.x_pos_online = next.channel[L1_LASER_X_POS].online;
    next.y_pos_online = next.channel[L1_LASER_Y_POS].online;
    next.height_online = next.channel[L1_LASER_HEIGHT].online;

    next.all_valid = (uint8_t)(next.x_pos_valid && next.y_pos_valid && next.height_valid);
    next.all_online = (uint8_t)(next.x_pos_online && next.y_pos_online && next.height_online);
    next.update_tick = HAL_GetTick();

    primask = __get_PRIMASK();
    __disable_irq();
    g_r2_laser_measure = next;
    if (primask == 0U)
    {
        __enable_irq();
    }
}
