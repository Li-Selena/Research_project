#include "include.h"
#include "can.h"

motor_measure_t motor_can1[8];
motor_measure_t motor_can2[8];
float motor_out1;
CAN_TxHeaderTypeDef can_tx_message;
uint8_t can_send_data[8];
uint32_t can_tx_dropped;
static int16_t can1_currents[8];

void get_motor_offset(motor_measure_t *motor, uint8_t data[])
{
    motor->angle = (uint16_t)((data[0] << 8) | data[1]);
    motor->last_angle = (int16_t)motor->angle;
    motor->offset_angle = motor->angle;
    motor->total_angle = 0;
    motor->round_cnt = 0;
    motor->initialized = 1;
}

void get_motor_measure(motor_measure_t *motor, uint8_t data[])
{
    int32_t delta;
    if (!motor->initialized) get_motor_offset(motor, data);
    motor->last_angle = (int16_t)motor->angle;
    motor->angle = (uint16_t)((data[0] << 8) | data[1]);
    delta = (int32_t)motor->angle - motor->last_angle;
    if (delta > 4096) { delta -= 8192; --motor->round_cnt; }
    if (delta < -4096) { delta += 8192; ++motor->round_cnt; }
    motor->total_angle += delta;
    motor->speed_rpm = (int16_t)((data[2] << 8) | data[3]);
    motor->given_current = (int16_t)((data[4] << 8) | data[5]);
    motor->temperature = data[6];
    motor->last_rx_ms = HAL_GetTick();
    ++motor->msg_cnt;
}

void get_total_angle(motor_measure_t *motor)
{
    int32_t delta = (int32_t)motor->angle - motor->last_angle;
    if (delta > 4096) delta -= 8192;
    if (delta < -4096) delta += 8192;
    motor->total_angle += delta;
    motor->last_angle = (int16_t)motor->angle;
}

void reset_motor_position(motor_measure_t *motors, uint8_t count)
{
    uint32_t irq = __get_PRIMASK();
    __disable_irq();
    for (uint8_t i = 0; i < count; ++i) {
        /* Keep live feedback/initialization: no false wrap on the next frame. */
        motors[i].offset_angle = motors[i].angle;
        motors[i].last_angle = (int16_t)motors[i].angle;
        motors[i].round_cnt = 0;
        motors[i].total_angle = 0;
    }
    __set_PRIMASK(irq);
}

uint8_t motor_feedback_snapshot(const motor_measure_t *motor, motor_measure_t *out)
{
    uint32_t irq = __get_PRIMASK();
    __disable_irq();
    *out = *motor;
    __set_PRIMASK(irq);
    return out->initialized;
}

uint8_t motor_feedback_online(const motor_measure_t *motor, uint32_t now)
{
    motor_measure_t copy;
    return motor_feedback_snapshot(motor, &copy) &&
           (uint32_t)(now - copy.last_rx_ms) <= MOTOR_TIMEOUT_MS;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];
    /* Read each FIFO element exactly once; bound work per interrupt. */
    for (uint8_t n = 0; n < 3; ++n) {
        if (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) == 0) break;
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK) break;
        if (header.RTR != CAN_RTR_DATA || header.DLC != 8) continue;
        if (header.IDE == CAN_ID_STD) {
            if (header.StdId < 0x201 || header.StdId > 0x208) continue;
            motor_measure_t *motors = hcan->Instance == CAN1 ? motor_can1 : motor_can2;
            get_motor_measure(&motors[header.StdId - 0x201], data);
        } else if (hcan->Instance == CAN2 && header.IDE == CAN_ID_EXT) {
            uint8_t type = (uint8_t)((header.ExtId >> 24) & 0x1f);
            uint8_t id = (uint8_t)(header.ExtId >> 8);
            if (type == 2 && (uint8_t)header.ExtId == MI_MASTERID && id > 0 && id < 9) {
                RxCAN_info_type_2_s info = {0};
                memcpy(&info, &header.ExtId, 4);
                MI_motor_RxDecode(&info, data);
                MI_Motor[id].RxCAN_info = info;
                MI_Motor[id].motor_mode_state = (motor_mode_state_e)info.mode_state;
                MI_Motor[id].last_rx_ms = HAL_GetTick();
                MI_Motor[id].feedback_valid = 1;
            } else if (type == 0) {
                OutputData.data_3 = (header.ExtId >> 8) & 0xffff;
            } else if (type == 17) {
                uint16_t index;
                float value;
                memcpy(&index, data, 2);
                memcpy(&value, data + 4, 4);
                OutputData.data_3 = header.ExtId & 0xff;
                OutputData.data_5 = index;
                OutputData.data_6 = value;
            }
        }
    }
}

static HAL_StatusTypeDef send_currents(CAN_HandleTypeDef *bus, uint32_t id,
                                      int16_t m1, int16_t m2, int16_t m3, int16_t m4)
{
    CAN_TxHeaderTypeDef header = {0};
    uint8_t data[8];
    uint32_t mailbox;
    const int16_t motors[4] = {m1, m2, m3, m4};
    header.StdId = id;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 8;
    for (uint8_t i = 0; i < 4; ++i) {
        data[2*i] = (uint8_t)((uint16_t)motors[i] >> 8);
        data[2*i+1] = (uint8_t)motors[i];
    }
    if (HAL_CAN_GetTxMailboxesFreeLevel(bus) == 0) return HAL_BUSY;
    return HAL_CAN_AddTxMessage(bus, &header, data, &mailbox);
}

void CAN1_BeginCycle(void) { memset(can1_currents, 0, sizeof(can1_currents)); }

void CAN1_SetMotorCurrent(uint8_t id, float value)
{
    float limit = id <= 3 ? CHASSIS_CURRENT_LIMIT : DELTA_CURRENT_LIMIT;
    if (id < 1 || id > 8) return;
    if (!isfinite(value)) value = 0;
    if (value > limit) value = limit;
    if (value < -limit) value = -limit;
    can1_currents[id-1] = (int16_t)value;
}

/* Compatibility setters; the sole hardware send occurs in CAN1_Flush(). */
void CAN1_CMD_1(int16_t a, int16_t b, int16_t c, int16_t d)
{
    CAN1_SetMotorCurrent(1,a); CAN1_SetMotorCurrent(2,b);
    CAN1_SetMotorCurrent(3,c); CAN1_SetMotorCurrent(4,d);
}
void CAN1_CMD_2(int16_t a, int16_t b, int16_t c, int16_t d)
{
    CAN1_SetMotorCurrent(5,a); CAN1_SetMotorCurrent(6,b);
    CAN1_SetMotorCurrent(7,c); CAN1_SetMotorCurrent(8,d);
}
HAL_StatusTypeDef CAN1_Flush(void)
{
    HAL_StatusTypeDef status;
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) < 2) { ++can_tx_dropped; return HAL_BUSY; }
    status = send_currents(&hcan1,0x200,can1_currents[0],can1_currents[1],can1_currents[2],can1_currents[3]);
    if (status == HAL_OK)
        status = send_currents(&hcan1,0x1ff,can1_currents[4],can1_currents[5],0,0);
    if (status != HAL_OK) ++can_tx_dropped;
    return status;
}
void CAN2_CMD_1(int16_t a,int16_t b,int16_t c,int16_t d) { (void)send_currents(&hcan2,0x200,a,b,c,d); }
void CAN2_CMD_2(int16_t a,int16_t b,int16_t c,int16_t d) { (void)send_currents(&hcan2,0x1ff,a,b,c,d); }
