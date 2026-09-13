#ifndef __IMU_H
#define __IMU_H

#include "main.h"
#include "wit_c_sdk.h"
#include "bsp_tick.h"

/* Raw decoded sensor-frame data: X right, Y front, Z up.
 * INS_Update_From_IMU converts to the unified application frame. */
typedef struct {
    float acc_x;   /* g */
    float acc_y;   /* g */
    float acc_z;   /* g */
    float gyro_x;  /* deg/s */
    float gyro_y;  /* deg/s */
    float gyro_z;  /* deg/s */
    float roll;    /* deg */
    float pitch;   /* deg */
    float yaw;     /* deg */

    uint32_t last_update_tick;
    uint8_t online;
    uint8_t update_flag;
} IMU_Data_t;

typedef struct {
    uint32_t dma_start_count;
    uint32_t dma_rx_event_count;
    uint32_t dma_rx_byte_count;
    uint32_t dma_restart_count;
    uint32_t reg_update_count;
    uint32_t acc_update_count;
    uint32_t gyro_update_count;
    uint32_t angle_update_count;
    uint32_t last_update_tick;
    uint32_t last_reg;
    uint32_t last_reg_num;
    uint16_t last_dma_size;
    uint16_t last_dma_read_pos;
    uint8_t dma_started;
    uint8_t initialized;
    uint8_t last_rx_byte;
    uint8_t last_error_code;
    int16_t raw_acc[3];
    int16_t raw_gyro[3];
    int16_t raw_angle[3];
    uint32_t dma_buf_addr;
    uint32_t dma_ndtr;
    uint32_t dma_event_type;
    uint32_t dma_restart_fail_count;
    uint32_t dma_rx_overrun_count;
    uint8_t rx_mode;
} IMU_Debug_t;

extern IMU_Data_t imu_data;
extern IMU_Debug_t g_imu_debug;
extern uint8_t imu_rx_byte;

void IMU_Init(void);
void IMU_ParseData(void);
void IMU_RxDmaEventCallback(uint16_t size);
void IMU_RestartDmaReceive(void);

#endif /* __IMU_H */
