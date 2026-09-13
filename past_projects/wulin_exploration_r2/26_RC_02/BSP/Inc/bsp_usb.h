
#ifndef __BSP_USB_H__
#define __BSP_USB_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "usbd_cdc.h"




/* 下位机异步机械臂 IK 结果命令，payload 为 6 字节。 */
#define USB_CMD_ARM_IK_RESULT      0x90U

/* USB 帧常量；参数布局见 ROBOT_USB_CONTROL_PROTOCOL.md。 */
#define USB_FRAME_HEAD1            0xA5U
#define USB_FRAME_HEAD2            0x5AU
#define USB_FRAME_TAIL             0xFFU
#define USB_FRAME_OVERHEAD         7U      /* 2 字节头 + LEN + CMD + 2 字节 CRC + 1 字节尾。 */
#define USB_FRAME_MAX_DATA_LEN     255U
#define USB_FRAME_BUF_SIZE         300U

extern uint8_t usb_Buf[USB_FRAME_BUF_SIZE];


/* USB 帧发送接口。 */
void     SendByte(uint8_t data);
uint8_t  Send(const uint8_t *data, uint16_t len);
uint16_t CRC16_Check(const uint8_t *data, uint16_t len);
uint8_t  Send_Cmd_Data(uint8_t cmd, const uint8_t *datas, uint8_t len);

/* 发送队列空间不足时累计丢帧数。 */
uint32_t USB_GetSendDropFrames(void);

/* 逐字节接收并把完整、CRC 正确的帧交给命令分发器。 */
void Data_Analysis(uint8_t cmd, const uint8_t* datas, uint8_t len);
void Receive(uint8_t bytedata);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_USB_H__ */
