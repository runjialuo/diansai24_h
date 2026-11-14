#ifndef OLED_H
#define OLED_H

#include "main.h"

/*==============================================================
 *  SSD1306 OLED 显示屏（128x64，8x16 ASCII 字库）
 *
 *  底层走 Bsp_Config.c 里的软件 I2C（PB8 = SCL，PB9 = SDA），
 *  本文件只关心"显示什么"，不关心"怎么打时序"。
 *
 *  坐标约定（与模板工程一致，方便直接搬运 ECar33 的显示代码）：
 *      Line   ：1 ~ 4    （从上到下 4 行，每行 16 列）
 *      Column ：1 ~ 16   （从左到右）
 *==============================================================*/

#define OLED_ADDRESS    0x78    /* 8 位写地址（7 位 0x3C） */

void OLED_Init(void);

void OLED_Clear(void);                              /* 全屏清空        */
void OLED_ClearLine(uint8_t Line);                  /* 只清某一行      */

void OLED_SetCursor(uint8_t Y, uint8_t X);          /* 页 0~7，列 0~127 */

void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char);
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String);

void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length);
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);

/* 浮点显示：整数部分固定 IntLen 位（不足补 0，避免位数变化导致闪烁），
   小数部分 DecLen 位，四舍五入。例：OLED_ShowFloatNum(3, 1, -12.35f, 3, 1) -> "-012.4" */
void OLED_ShowFloatNum(uint8_t Line, uint8_t Column, float Num, uint8_t IntLen, uint8_t DecLen);

#endif /* OLED_H */
