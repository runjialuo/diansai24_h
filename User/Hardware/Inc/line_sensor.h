#ifndef __LINE_SENSOR_H
#define __LINE_SENSOR_H

#include <stdint.h>

/*==============================================================
 *  四路灰度循迹传感器（Hardware 层）
 *
 *  只负责"把 4 个引脚读成一个字节"，不做任何控制运算。
 *
 *  引脚（CubeMX 生成，见 Core/Inc/main.h）：
 *      IR_X1 -> PB11   最左
 *      IR_X2 -> PB10
 *      IR_X3 -> PB1
 *      IR_X4 -> PB0    最右
 *
 *  物理布局（实测间距）：
 *      X1 --2.8cm-- X2 -0.7cm- X3 --2.8cm-- X4      总宽约 6.3cm
 *
 *  电平约定：模块输出为比较器开漏，**低电平 = 压到黑线**
 *      如果你的模块是"高电平压线"，把 LINE_ACTIVE_LEVEL 改成 GPIO_PIN_SET
 *==============================================================*/

#define LINE_ACTIVE_LEVEL   GPIO_PIN_RESET   /* 低电平 = 压线 */

/* 位定义：bit0 = X1（最左） ... bit3 = X4（最右） */
#define LINE_BIT_X1         0x01U
#define LINE_BIT_X2         0x02U
#define LINE_BIT_X3         0x04U
#define LINE_BIT_X4         0x08U
#define LINE_ALL            0x0FU

void Line_Init(void);

/* 读一次 4 路状态，返回位掩码（1 = 压黑线） */
uint8_t Line_ReadRaw(void);

/* 统计掩码里有几路压线（0~4） */
uint8_t Line_CountBits(uint8_t state);

#endif /* __LINE_SENSOR_H */
