#include "line_sensor.h"
#include "main.h"

/*==============================================================
 *  四路灰度传感器读取（Hardware 层）
 *
 *  引脚在 MX_GPIO_Init() 里已经配成上拉输入，
 *  所以这里只做纯粹的读电平——不做消抖、不做滤波，
 *  消抖交给 Software 层的 LineTrack 用"连续 N 次确认"处理。
 *==============================================================*/

void Line_Init(void)
{
    /* 引脚配置由 MX_GPIO_Init() 完成，这里只做一次空读，
       让第一次采样就是有效值（避免上电时读到悬空电平） */
    (void)Line_ReadRaw();
}

uint8_t Line_ReadRaw(void)
{
    uint8_t state = 0;

    if (HAL_GPIO_ReadPin(IR_X1_GPIO_Port, IR_X1_Pin) == LINE_ACTIVE_LEVEL)
    {
        state |= LINE_BIT_X1;
    }
    if (HAL_GPIO_ReadPin(IR_X2_GPIO_Port, IR_X2_Pin) == LINE_ACTIVE_LEVEL)
    {
        state |= LINE_BIT_X2;
    }
    if (HAL_GPIO_ReadPin(IR_X3_GPIO_Port, IR_X3_Pin) == LINE_ACTIVE_LEVEL)
    {
        state |= LINE_BIT_X3;
    }
    if (HAL_GPIO_ReadPin(IR_X4_GPIO_Port, IR_X4_Pin) == LINE_ACTIVE_LEVEL)
    {
        state |= LINE_BIT_X4;
    }

    return state;
}

uint8_t Line_CountBits(uint8_t state)
{
    uint8_t i;
    uint8_t cnt = 0;

    for (i = 0; i < 4U; i++)
    {
        if (state & (uint8_t)(1U << i))
        {
            cnt++;
        }
    }
    return cnt;
}
