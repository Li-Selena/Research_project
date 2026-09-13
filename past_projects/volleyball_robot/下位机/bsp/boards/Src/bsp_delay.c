#include "bsp_delay.h"
#include "main.h"

static uint8_t fac_us = 0;
static uint32_t fac_ms = 0;

void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;     // 使能 DWT 寄存器
    DWT->CYCCNT = 0;                                     // 清零计数器
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;                // 使能周期计数器
}

void delay_init(void)
{
    fac_us = SystemCoreClock / 1000000;
    fac_ms = SystemCoreClock / 1000;

}

uint32_t DWT_GetUs(void)
{
    return DWT->CYCCNT / (HAL_RCC_GetHCLKFreq() / 1000000);
}

void delay_us(uint16_t nus)
{
    uint32_t ticks = 0;
    uint32_t told = 0;
    uint32_t tnow = 0;
    uint32_t tcnt = 0;
    uint32_t reload = 0;
    reload = SysTick->LOAD;
    ticks = nus * fac_us;
    told = SysTick->VAL;
    while (1)
    {
        tnow = SysTick->VAL;
        if (tnow != told)
        {
            if (tnow < told)
            {
                tcnt += told - tnow;
            }
            else
            {
                tcnt += reload - tnow + told;
            }
            told = tnow;
            if (tcnt >= ticks)
            {
                break;
            }
        }
    }
}

void delay_ms(uint16_t nms)
{
    uint32_t ticks = 0;
    uint32_t told = 0;
    uint32_t tnow = 0;
    uint32_t tcnt = 0;
    uint32_t reload = 0;
    reload = SysTick->LOAD;
    ticks = nms * fac_ms;
    told = SysTick->VAL;
    while (1)
    {
        tnow = SysTick->VAL;
        if (tnow != told)
        {
            if (tnow < told)
            {
                tcnt += told - tnow;
            }
            else
            {
                tcnt += reload - tnow + told;
            }
            told = tnow;
            if (tcnt >= ticks)
            {
                break;
            }
        }
    }
}


