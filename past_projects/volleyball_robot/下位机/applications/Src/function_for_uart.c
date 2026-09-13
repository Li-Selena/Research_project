#include "include.h"

static uint8_t imu_frame[11];
static uint8_t imu_index;

void hwt_parser_feed(uint8_t byte)
{
    if (imu_index == 0U && byte != 0x50U) return;
    imu_frame[imu_index++] = byte;
    if (imu_index < sizeof(imu_frame)) return;
    imu_index = 0;
    uint16_t expected = modbus_crc16(imu_frame, 9U);
    uint16_t received = (uint16_t)(imu_frame[9] | ((uint16_t)imu_frame[10] << 8));
    if (imu_frame[0] != 0x50U || imu_frame[1] != 0x03U ||
        imu_frame[2] != 0x06U || expected != received) {
        hwt_what_type = 0;
        return;
    }
    int16_t raw = (int16_t)(((uint16_t)imu_frame[7] << 8) | imu_frame[8]);
    if (hwt_what_type == 1U) my_angle_measure.yaw = raw / 32768.0f * 180.0f;
    else if (hwt_what_type == 2U) {
        my_angle_measure.yaw_omega = raw / 32768.0f * 2000.0f;
        my_angle_measure.filtered_yaw_omega =
            low_pass_filter(my_angle_measure.yaw_omega,
                            my_angle_measure.filtered_yaw_omega, 0.3f);
    }
    imu_last_rx_ms = HAL_GetTick();
    hwt_what_type = 0;
}
