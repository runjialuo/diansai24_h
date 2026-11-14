#include "Delay.h"

/* 1 us 对应的 SysTick 计数值 */
#define SYSTICK_TICKS_PER_US    (SystemCoreClock / 1000000U)

/* 初始化：SysTick 已由 HAL_Init() 配好，这里留空接口，
   方便以后换时基（例如改成 TIM1 自由运行计数器）时不用改调用处 */
void Delay_Init(void)
{
    /* 若以后需要，可在这里读一次 SysTick->LOAD 做自检 */
}

/*-------------------------------------------------------------
 * 微秒延时：查询 SysTick->VAL 的递减量
 *   VAL 是向下计数的，且可能在一轮延时中重装多次，
 *   所以用"累加 + 回绕判断"来算已经过去的计数。
 *------------------------------------------------------------*/
void Delay_us(uint32_t us)
{
    uint32_t load = SysTick->LOAD + 1U;          /* 一次重装的计数值（72000） */
    uint32_t want = us * SYSTICK_TICKS_PER_US;   /* 目标计数值                */
    uint32_t acc  = 0;
    uint32_t last = SysTick->VAL;

    while (acc < want)
    {
        uint32_t now = SysTick->VAL;

        if (now <= last)
        {
            /* 正常递减：last -> now */
            acc += (last - now);
        }
        else
        {
            /* 期间发生重装：last -> 0 -> LOAD -> now */
            acc += (last + load - now);
        }

        last = now;
    }
}

/*-------------------------------------------------------------
 * 毫秒延时：直接交给 HAL 的 tick 计数（同样是 SysTick 驱动）
 *   等价于 FreeRTOS 的 osDelay(ms) —— 阻塞期间中断照常响应
 *------------------------------------------------------------*/
void Delay_ms(uint32_t ms)
{
    while(ms--)
    {
        Delay_us(1000);
    }
}