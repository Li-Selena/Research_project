#include "bsp_usb.h"
#include "usbd_cdc_if.h"
#include <string.h>
#include "arm_user.h"
#include "Control_Task.h"

extern volatile uint8_t USB_Task_flag;
extern volatile uint8_t USART_Task_flag ;

void ArmIK_ComponentStep(float x, float y, float z);


/* USB 接收任务使用的字节缓冲。 */
uint8_t usb_Buf[USB_FRAME_BUF_SIZE];

/* 发送失败帧计数。 */
static volatile uint32_t s_usbSendDropFrames = 0U;

/* USB 流式解帧状态。 */
static uint8_t  s_usbRxStep    = 0U;
static uint16_t s_usbRxCnt     = 0U;
static uint8_t  s_usbRxLen     = 0U;
static uint8_t  s_usbRxCmd     = 0U;
static uint8_t *s_usbRxDataPtr = 0;
static uint16_t s_usbRxCrc16   = 0U;


/* 解析器内部辅助函数。 */

/* 清空当前半帧及解析状态。 */
static void USB_ParserReset(void)
{
    s_usbRxStep    = 0U;
    s_usbRxCnt     = 0U;
    s_usbRxLen     = 0U;
    s_usbRxCmd     = 0U;
    s_usbRxDataPtr = 0;
    s_usbRxCrc16   = 0U;
}

/* 当前字节为 A5 时，作为下一帧的第一个帧头。 */
static void USB_ParserRestartFromHead1(void)
{
    USB_ParserReset();
    s_usbRxStep = 1U;
    usb_Buf[s_usbRxCnt++] = USB_FRAME_HEAD1;
}

/* 追加一字节，缓冲区满则复位并返回失败。 */
static uint8_t USB_ParserPushByte(uint8_t byte)
{
    if (s_usbRxCnt >= USB_FRAME_BUF_SIZE)
    {
        USB_ParserReset();
        return 0U;
    }

    usb_Buf[s_usbRxCnt++] = byte;
    return 1U;
}



/* 兼容接口；此单字节发送函数当前不发送数据，请使用 Send/Send_Cmd_Data。 */

void SendByte(uint8_t data)
{
    (void)data;
    /* 保留空实现；不在这里拆分应用层帧。 */
}

uint8_t Send(const uint8_t *data, uint16_t len)
{
    if ((data == 0) || (len == 0U))
    {
        return 1U;
    }

    /* 先检查整帧空间，避免部分入队破坏接收端帧边界。 */
    if (CDC_App_TxFree() < len)
    {
        s_usbSendDropFrames++;
        return 0U;
    }

    if (CDC_App_Write(data, len) == 0U)
    {
        s_usbSendDropFrames++;
        return 0U;
    }

    return 1U;
}

uint32_t USB_GetSendDropFrames(void)
{
    return s_usbSendDropFrames;
}

/* CRC16/Modbus：初值 FFFF、多项式 A001。 */
uint16_t CRC16_Check(const uint8_t *data, uint16_t len)
{
    uint16_t crc16 = 0xFFFFU;
    uint16_t i;
    uint8_t  j;

    if ((data == 0) || (len == 0U))
    {
        return crc16;
    }

    for (i = 0U; i < len; i++)
    {
        crc16 ^= data[i];

        for (j = 0U; j < 8U; j++)
        {
            if ((crc16 & 0x0001U) != 0U)
            {
                crc16 >>= 1U;
                crc16 ^= 0xA001U;
            }
            else
            {
                crc16 >>= 1U;
            }
        }
    }

    return crc16;
}

/* 封装 A5 5A LEN CMD DATA CRC_H CRC_L FF 后一次入队。 */
uint8_t Send_Cmd_Data(uint8_t cmd, const uint8_t *datas, uint8_t len)
{
    uint8_t  buf[USB_FRAME_BUF_SIZE];
    uint16_t cnt = 0U;
    uint16_t i;
    uint16_t crc16;

    if ((datas == 0) && (len != 0U))
    {
        return 0U;
    }

    /* 检查 payload 加固定开销是否超过本地帧缓冲。 */
    if (((uint16_t)len + USB_FRAME_OVERHEAD) > USB_FRAME_BUF_SIZE)
    {
        s_usbSendDropFrames++;
        return 0U;
    }

    buf[cnt++] = USB_FRAME_HEAD1;
    buf[cnt++] = USB_FRAME_HEAD2;
    buf[cnt++] = len;
    buf[cnt++] = cmd;

    for (i = 0U; i < (uint16_t)len; i++)
    {
        buf[cnt++] = datas[i];
    }

    /* CRC 覆盖帧头、LEN、CMD 和 DATA，不含 CRC 本身与帧尾。 */
    crc16 = CRC16_Check(buf, cnt);

    /* 本项目线上顺序为 CRC 高字节在前、低字节在后。 */
    buf[cnt++] = (uint8_t)(crc16 >> 8);
    buf[cnt++] = (uint8_t)(crc16 & 0xFFU);
    buf[cnt++] = USB_FRAME_TAIL;

    return Send(buf, cnt);
}


/* USB 命令流解析。 */



/* 接收状态机。 */

/* 由 PC_RX_Task 逐字节调用。 */
void Receive(uint8_t bytedata)
{
    uint16_t calc_crc;

    switch (s_usbRxStep)
    {
    case 0: /* 等待第一个帧头 A5。 */
        if (bytedata == USB_FRAME_HEAD1)
        {
            USB_ParserRestartFromHead1();
        }
        break;

    case 1: /* 等待第二个帧头 5A。 */
        if (bytedata == USB_FRAME_HEAD2)
        {
            if (USB_ParserPushByte(bytedata) == 0U)
            {
                return;
            }
            s_usbRxStep = 2U;
        }
        else if (bytedata == USB_FRAME_HEAD1)
        {
            /* 连续 A5：保留最新 A5 作为新帧起点。 */
            USB_ParserRestartFromHead1();
        }
        else
        {
            USB_ParserReset();
        }
        break;

    case 2: /* 读取 payload 长度。 */
        s_usbRxLen = bytedata;

        /* 确认整个帧可放入解析缓冲。 */
        if (((uint16_t)s_usbRxLen + USB_FRAME_OVERHEAD) > USB_FRAME_BUF_SIZE)
        {
            USB_ParserReset();
            break;
        }

        if (USB_ParserPushByte(bytedata) == 0U)
        {
            return;
        }

        s_usbRxStep = 3U;
        break;

    case 3: /* 读取命令字。 */
        if (USB_ParserPushByte(bytedata) == 0U)
        {
            return;
        }

        s_usbRxCmd = bytedata;
        s_usbRxDataPtr = &usb_Buf[s_usbRxCnt];

        if (s_usbRxLen == 0U)
        {
            s_usbRxStep = 5U; /* 无 payload 时直接接收 CRC。 */
        }
        else
        {
            s_usbRxStep = 4U;
        }
        break;

    case 4: /* 接收 payload 字节。 */
        if (USB_ParserPushByte(bytedata) == 0U)
        {
            return;
        }

        /* 前四字节为 HEAD1、HEAD2、LEN、CMD。 */
        if ((s_usbRxCnt - 4U) >= (uint16_t)s_usbRxLen)
        {
            s_usbRxStep = 5U;
        }
        break;

    case 5: /* 接收 CRC 高字节。 */
        s_usbRxCrc16 = ((uint16_t)bytedata) << 8;
        s_usbRxStep = 6U;
        break;

    case 6: /* 接收 CRC 低字节并校验已接收内容。 */
        s_usbRxCrc16 |= bytedata;

        calc_crc = CRC16_Check(usb_Buf, s_usbRxCnt);

        if (s_usbRxCrc16 == calc_crc)
        {
            s_usbRxStep = 7U;
        }
        else if (bytedata == USB_FRAME_HEAD1)
        {
            /* CRC 失败且当前字节为 A5 时尝试同步下一帧。 */
            USB_ParserRestartFromHead1();
        }
        else
        {
            USB_ParserReset();
        }
        break;

    case 7: /* 等待帧尾 FF；完整帧才分发命令。 */
        if (bytedata == USB_FRAME_TAIL)
        {
            Data_Analysis(s_usbRxCmd, s_usbRxDataPtr, s_usbRxLen);
            USB_ParserReset();
        }
        else if (bytedata == USB_FRAME_HEAD1)
        {
            /* 帧尾错误且当前字节为 A5 时尝试同步下一帧。 */
            USB_ParserRestartFromHead1();
        }
        else
        {
            USB_ParserReset();
        }
        break;

    default:
        USB_ParserReset();
        break;
    }
}




