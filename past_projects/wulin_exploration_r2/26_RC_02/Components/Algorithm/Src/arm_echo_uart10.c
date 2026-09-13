#include <string.h>
#include "arm_echo_uart10.h"
#include "arm_user.h"
#include "arm_ik_3r_safe_stm32h7.h"
#include "usart.h"

/*
 * 如果你工程里的串口句柄不是 huart10，
 * 可以在本文件前面改这个宏。
 */
#ifndef ARM_ECHO_UART_HANDLE
#define ARM_ECHO_UART_HANDLE    huart10
#endif

/* ========================= 内部状态缓存 ========================= */
typedef struct
{
    uint8_t status_code;
    uint8_t unsafe_reason;
    uint8_t action_code;
} ArmEchoUart10_ResultCache_t;

static ArmEchoUart10_ResultCache_t s_arm_echo_result_cache;

/*
 * 中断发送使用的静态发送缓冲区。
 *
 * 注意：
 * 不能在 StartSend_IT() 里定义局部数组然后交给 HAL_UART_Transmit_IT()，
 * 因为中断发送尚未完成时函数已经退出，局部数组会失效。
 * 所以这里必须使用静态全局缓冲区。
 */
static uint8_t s_arm_echo_tx_buf[ARM_ECHO_UART10_FRAME_LEN];

/*
 * 发送忙标志：
 * 0 = 空闲
 * 1 = 正在发送
 */
static volatile uint8_t s_arm_echo_tx_busy = 0U;

/* ========================= 内部函数声明 ========================= */
static uint8_t ArmEchoUart10_ChecksumLow8(const uint8_t *data, uint16_t len);
static void ArmEchoUart10_PutFloatLE(uint8_t *buf, float value);

/* ========================= 内部函数实现 ========================= */
/*
 * 数据区逐字节累加，取低八位。
 */
static uint8_t ArmEchoUart10_ChecksumLow8(const uint8_t *data, uint16_t len)
{
    uint32_t sum = 0U;
    uint16_t i;

    if (data == 0)
    {
        return 0U;
    }

    for (i = 0U; i < len; i++)
    {
        sum += data[i];
    }

    return (uint8_t)(sum & 0xFFU);
}

/*
 * 以 little-endian 方式写入 float。
 * STM32H7 默认就是 little-endian。
 */
static void ArmEchoUart10_PutFloatLE(uint8_t *buf, float value)
{
    if (buf == 0)
    {
        return;
    }

    memcpy(buf, &value, 4U);
}

/* ========================= 对外接口实现 ========================= */
void ArmEchoUart10_Init(void)
{
    memset(&s_arm_echo_result_cache, 0, sizeof(s_arm_echo_result_cache));
    memset(s_arm_echo_tx_buf, 0, sizeof(s_arm_echo_tx_buf));
    s_arm_echo_tx_busy = 0U;
}

void ArmEchoUart10_UpdateResult(uint8_t status_code,
                                uint8_t unsafe_reason,
                                uint8_t action_code)
{
    s_arm_echo_result_cache.status_code   = status_code;
    s_arm_echo_result_cache.unsafe_reason = unsafe_reason;
    s_arm_echo_result_cache.action_code   = action_code;
}

uint16_t ArmEchoUart10_BuildPacket(uint8_t *out_buf, uint16_t out_size)
{
    uint16_t idx;
    uint16_t payload_start;
    uint8_t checksum;

    ArmIK_AppState_t app_snapshot;
    Arm3R_ModelAngles_t model_snapshot;
    ArmEchoUart10_ResultCache_t result_snapshot;
    uint32_t primask;

    float model_theta1_deg;
    float model_theta2_deg;
    float model_theta3_deg;

    if ((out_buf == 0) || (out_size < ARM_ECHO_UART10_FRAME_LEN))
    {
        return 0U;
    }

    /*
     * 为了尽量保证同一帧数据的一致性，
     * 这里短时间关中断，复制一份快照再打包。
     */
    primask = __get_PRIMASK();
    __disable_irq();

    app_snapshot    = *ArmIK_GetAppState();
    model_snapshot  = g_arm_ik.result.model;
    result_snapshot = s_arm_echo_result_cache;

    if (!primask)
    {
        __enable_irq();
    }

    model_theta1_deg = Arm3R_RadToDeg(model_snapshot.theta1);
    model_theta2_deg = Arm3R_RadToDeg(model_snapshot.theta2);
    model_theta3_deg = Arm3R_RadToDeg(model_snapshot.theta3);

    idx = 0U;

    /* 帧头 */
    out_buf[idx++] = ARM_ECHO_UART10_FRAME_HEAD;

    /* payload 起始位置 */
    payload_start = idx;

    /* ========================= payload 开始 ========================= */
    out_buf[idx++] = ARM_ECHO_UART10_MSG_ID;
    out_buf[idx++] = result_snapshot.status_code;
    out_buf[idx++] = result_snapshot.unsafe_reason;
    out_buf[idx++] = result_snapshot.action_code;
    out_buf[idx++] = app_snapshot.has_last_valid;
    out_buf[idx++] = app_snapshot.active_motor_deg.valid;

    /* 当前设定位置：最近一次收到的目标点 */
    ArmEchoUart10_PutFloatLE(&out_buf[idx], app_snapshot.last_req_pt.x); idx += 4U;
    ArmEchoUart10_PutFloatLE(&out_buf[idx], app_snapshot.last_req_pt.y); idx += 4U;
    ArmEchoUart10_PutFloatLE(&out_buf[idx], app_snapshot.last_req_pt.z); idx += 4U;

    /* 当前有效位置：最近一次安全有效目标点 */
    ArmEchoUart10_PutFloatLE(&out_buf[idx], app_snapshot.last_valid_pt.x); idx += 4U;
    ArmEchoUart10_PutFloatLE(&out_buf[idx], app_snapshot.last_valid_pt.y); idx += 4U;
    ArmEchoUart10_PutFloatLE(&out_buf[idx], app_snapshot.last_valid_pt.z); idx += 4U;

    /* 当前实际执行的电机角度制目标 */
    ArmEchoUart10_PutFloatLE(&out_buf[idx], app_snapshot.active_motor_deg.j1_deg); idx += 4U;
    ArmEchoUart10_PutFloatLE(&out_buf[idx], app_snapshot.active_motor_deg.j2_deg); idx += 4U;
    ArmEchoUart10_PutFloatLE(&out_buf[idx], app_snapshot.active_motor_deg.j3_deg); idx += 4U;

    /* 当前求解得到的模型角，转为角度制 */
    ArmEchoUart10_PutFloatLE(&out_buf[idx], model_theta1_deg); idx += 4U;
    ArmEchoUart10_PutFloatLE(&out_buf[idx], model_theta2_deg); idx += 4U;
    ArmEchoUart10_PutFloatLE(&out_buf[idx], model_theta3_deg); idx += 4U;
    /* ========================= payload 结束 ========================= */

    checksum = ArmEchoUart10_ChecksumLow8(&out_buf[payload_start], ARM_ECHO_UART10_PAYLOAD_LEN);
    out_buf[idx++] = checksum;
    out_buf[idx++] = ARM_ECHO_UART10_FRAME_TAIL;

    return idx;
}

uint8_t ArmEchoUart10_IsBusy(void)
{
    return s_arm_echo_tx_busy;
}

uint8_t ArmEchoUart10_StartSend_IT(void)
{
    uint16_t tx_len;
    HAL_StatusTypeDef hal_ret;

    if (s_arm_echo_tx_busy != 0U)
    {
        return 0U;
    }

    tx_len = ArmEchoUart10_BuildPacket(s_arm_echo_tx_buf, sizeof(s_arm_echo_tx_buf));
    if (tx_len == 0U)
    {
        return 0U;
    }

    s_arm_echo_tx_busy = 1U;

    hal_ret = HAL_UART_Transmit_IT(&ARM_ECHO_UART_HANDLE, s_arm_echo_tx_buf, tx_len);
    if (hal_ret != HAL_OK)
    {
        s_arm_echo_tx_busy = 0U;
        return 0U;
    }

    return 1U;
}

void ArmEchoUart10_TxCpltHandler(void)
{
    s_arm_echo_tx_busy = 0U;
}

void ArmEchoUart10_ErrorHandler(void)
{
    s_arm_echo_tx_busy = 0U;
}