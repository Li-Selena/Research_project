#include "PC_RX_Task.h"
#include "cmsis_os.h"

/* CDC chunks must not overwrite the parser's partially assembled frame. */
static uint8_t cdc_read_buf[USB_FRAME_BUF_SIZE];
extern uint8_t bt_data[BT_FRAME_DATA_LEN];     // USART 遥控数据区，40 字节。


static void USB_RX_task(void);


void PC_RX_Task(void const * argument)
{
  /* USER CODE BEGIN PC_RX_Task */
  /* Infinite loop */
  for(;;)
  {

    USB_RX_task();



    osDelay(1);
  }
  /* USER CODE END PC_RX_Task */
}


static void USB_RX_task(void)
{
    uint32_t i;
    uint32_t read_len;

    read_len = CDC_App_Read(cdc_read_buf, sizeof(cdc_read_buf));
    for (i = 0; i < read_len; i++)
    {
        Receive(cdc_read_buf[i]);
    }
}
