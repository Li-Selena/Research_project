#ifndef UART_APP_H
#define UART_APP_H

#include <stdint.h>

typedef struct {
    float roll, pitch, yaw;
    float filtered_roll, filtered_pitch, filtered_yaw;
    float roll_omega, pitch_omega, yaw_omega;
    float filtered_roll_omega, filtered_pitch_omega, filtered_yaw_omega;
    float x_acc, y_acc, z_acc;
    float filtered_x_acc, filtered_y_acc, filtered_z_acc;
} angle_measure;

extern volatile uint32_t imu_last_rx_ms;
extern angle_measure my_angle_measure;

void uart_receivers_start(void);
void hwt_parser_feed(uint8_t byte);
void uart_service_tx(void);
uint8_t host_input_fresh(void);
uint8_t imu_input_fresh(void);

#endif
