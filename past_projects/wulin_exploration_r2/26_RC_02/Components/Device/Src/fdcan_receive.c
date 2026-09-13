#include "fdcan_receive.h"
#include "bsp_fdcan.h"

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

motor_measure_t motor_fdcan1[8];
motor_measure_t motor_fdcan2[8];
motor_measure_t motor_fdcan3[8];

void get_motor_measure(motor_measure_t *ptr, uint8_t data[])
{
    int32_t angle_delta;

    if ((ptr == NULL) || (data == NULL))
    {
        return;
    }

    ptr->last_angle    = ptr->angle;
    ptr->angle         = (uint16_t)(data[0] << 8 | data[1]);
    ptr->speed_rpm     = (int16_t)(data[2] << 8 | data[3]);
    ptr->given_current = (int16_t)(data[4] << 8 | data[5]);
    ptr->temperature   = data[6];

    angle_delta = (int32_t)ptr->angle - (int32_t)ptr->last_angle;

    if (angle_delta > 4096)
    {
        ptr->round_cnt--;
    }
    else if (angle_delta < -4096)
    {
        ptr->round_cnt++;
    }

    ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle - ptr->offset_angle;
}

void get_motor_offset(motor_measure_t *ptr, uint8_t data[])
{
    if ((ptr == NULL) || (data == NULL))
    {
        return;
    }

    ptr->angle = (uint16_t)(data[0] << 8 | data[1]);
    ptr->offset_angle = ptr->angle;
    ptr->last_angle = ptr->angle;
    ptr->round_cnt = 0;
    ptr->total_angle = 0;
}

#define ABS(x) ((x > 0) ? (x) : (-(x)))

void get_total_angle(motor_measure_t *p)
{
    int res1, res2, delta;

    if (p == NULL)
    {
        return;
    }

    if (p->angle < p->last_angle)
    {
        res1 = p->angle + 8192 - p->last_angle;
        res2 = p->angle - p->last_angle;
    }
    else
    {
        res1 = p->angle - 8192 - p->last_angle;
        res2 = p->angle - p->last_angle;
    }

    delta = (ABS(res1) < ABS(res2)) ? res1 : res2;
    p->total_angle += delta;
    p->last_angle = p->angle;
}

uint8_t FDCAN_Send(FDCAN_HandleTypeDef *hfdcan, uint16_t std_id,
                   int16_t m1, int16_t m2, int16_t m3, int16_t m4)
{
    FDCAN_TxHeaderTypeDef fdcan_tx_header;
    uint8_t fdcan_send_data[8];
    uint16_t cmd1;
    uint16_t cmd2;
    uint16_t cmd3;
    uint16_t cmd4;

    if (hfdcan == NULL)
    {
        return 0U;
    }

    if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) == 0U)
    {
        return 0U;
    }

    fdcan_tx_header.Identifier = std_id;
    fdcan_tx_header.IdType = FDCAN_STANDARD_ID;
    fdcan_tx_header.TxFrameType = FDCAN_DATA_FRAME;
    fdcan_tx_header.DataLength = FDCAN_DLC_BYTES_8;
    fdcan_tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    fdcan_tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    fdcan_tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    fdcan_tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    fdcan_tx_header.MessageMarker = 0U;

    cmd1 = (uint16_t)m1;
    cmd2 = (uint16_t)m2;
    cmd3 = (uint16_t)m3;
    cmd4 = (uint16_t)m4;

    fdcan_send_data[0] = (uint8_t)(cmd1 >> 8);
    fdcan_send_data[1] = (uint8_t)(cmd1 & 0xFFU);
    fdcan_send_data[2] = (uint8_t)(cmd2 >> 8);
    fdcan_send_data[3] = (uint8_t)(cmd2 & 0xFFU);
    fdcan_send_data[4] = (uint8_t)(cmd3 >> 8);
    fdcan_send_data[5] = (uint8_t)(cmd3 & 0xFFU);
    fdcan_send_data[6] = (uint8_t)(cmd4 >> 8);
    fdcan_send_data[7] = (uint8_t)(cmd4 & 0xFFU);

    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &fdcan_tx_header, fdcan_send_data) != HAL_OK)
    {
        return 0U;
    }

    return 1U;
}

void FDCAN1_CMD_1(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    (void)FDCAN_Send(&hfdcan1, 0x200, motor1, motor2, motor3, motor4);
}

void FDCAN1_CMD_2(int16_t motor5, int16_t motor6, int16_t motor7, int16_t motor8)
{
    (void)FDCAN_Send(&hfdcan1, 0x1FF, motor5, motor6, motor7, motor8);
}

void FDCAN2_CMD_1(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    (void)FDCAN_Send(&hfdcan2, 0x200, motor1, motor2, motor3, motor4);
}

void FDCAN2_CMD_2(int16_t motor5, int16_t motor6, int16_t motor7, int16_t motor8)
{
    (void)FDCAN_Send(&hfdcan2, 0x1FF, motor5, motor6, motor7, motor8);
}

void FDCAN3_CMD_1(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    (void)FDCAN_Send(&hfdcan3, 0x200, motor1, motor2, motor3, motor4);
}

void FDCAN3_CMD_2(int16_t motor5, int16_t motor6, int16_t motor7, int16_t motor8)
{
    (void)FDCAN_Send(&hfdcan3, 0x1FF, motor5, motor6, motor7, motor8);
}
