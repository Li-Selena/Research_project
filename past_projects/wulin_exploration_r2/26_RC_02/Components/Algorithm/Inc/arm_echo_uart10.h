#ifndef __ARM_ECHO_UART10_H__
#define __ARM_ECHO_UART10_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*
 * UART10 机械臂状态回显模块（中断发送版）
 *
 * 帧格式固定：
 *   [0]   = 0xA5                         帧头
 *   [1..N]= payload                      数据区（固定长度）
 *   [N+1] = checksum_low8                数据区逐字节累加低八位
 *   [N+2] = 0x5A                         帧尾
 *
 * 由于本协议没有长度字段，因此接收端必须按固定帧长解析。
 */

/* ========================= 协议常量 ========================= */
#define ARM_ECHO_UART10_FRAME_HEAD      0xA5U
#define ARM_ECHO_UART10_FRAME_TAIL      0x5AU
#define ARM_ECHO_UART10_MSG_ID          0xE1U

/*
 * payload 固定 54 字节：
 *
 * byte[0]   : msg_id
 * byte[1]   : status_code
 * byte[2]   : unsafe_reason
 * byte[3]   : action_code
 * byte[4]   : has_last_valid
 * byte[5]   : active_motor_valid
 *
 * byte[6..9]    : last_req_pt.x      float
 * byte[10..13]  : last_req_pt.y      float
 * byte[14..17]  : last_req_pt.z      float
 *
 * byte[18..21]  : last_valid_pt.x    float
 * byte[22..25]  : last_valid_pt.y    float
 * byte[26..29]  : last_valid_pt.z    float
 *
 * byte[30..33]  : active j1 deg      float
 * byte[34..37]  : active j2 deg      float
 * byte[38..41]  : active j3 deg      float
 *
 * byte[42..45]  : model theta1 deg   float
 * byte[46..49]  : model theta2 deg   float
 * byte[50..53]  : model theta3 deg   float
 */
#define ARM_ECHO_UART10_PAYLOAD_LEN     54U
#define ARM_ECHO_UART10_FRAME_LEN       (1U + ARM_ECHO_UART10_PAYLOAD_LEN + 1U + 1U)

/* ========================= 对外接口 ========================= */

/*
 * 初始化内部缓存和发送状态。
 * 建议系统初始化时调用一次。
 */
void ArmEchoUart10_Init(void);

/*
 * 更新最近一次求解结果状态。
 *
 * 说明：
 * 当前 arm_user 的 AppState 中没有保存：
 * 1. status_code
 * 2. unsafe_reason
 * 3. action_code
 *
 * 因此这里单独做一份缓存。
 */
void ArmEchoUart10_UpdateResult(uint8_t status_code,
                                uint8_t unsafe_reason,
                                uint8_t action_code);

/*
 * 将当前状态打包到 out_buf。
 *
 * 参数：
 *   out_buf   : 外部缓存
 *   out_size  : 缓存大小，必须 >= ARM_ECHO_UART10_FRAME_LEN
 *
 * 返回值：
 *   0         : 失败
 *   >0        : 实际帧长（固定为 ARM_ECHO_UART10_FRAME_LEN）
 */
uint16_t ArmEchoUart10_BuildPacket(uint8_t *out_buf, uint16_t out_size);

/*
 * 启动一次 UART10 中断发送。
 *
 * 返回值：
 *   1 : 成功启动发送
 *   0 : 当前忙 / 打包失败 / 串口启动失败
 */
uint8_t ArmEchoUart10_StartSend_IT(void);

/*
 * 查询当前是否正在发送。
 *
 * 返回值：
 *   0 : 空闲
 *   1 : 忙
 */
uint8_t ArmEchoUart10_IsBusy(void);

/*
 * UART 发送完成时调用。
 * 一般放在 HAL_UART_TxCpltCallback() 中。
 */
void ArmEchoUart10_TxCpltHandler(void);

/*
 * UART 发送异常时调用。
 * 建议放在 HAL_UART_ErrorCallback() 中，
 * 防止异常后 busy 标志一直不清。
 */
void ArmEchoUart10_ErrorHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* __ARM_ECHO_UART10_H__ */