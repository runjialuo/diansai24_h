#include "OLED.h"
#include "IIC.h"
#include "OLED_Font.h"

/*==============================================================
 *  SSD1306 驱动（软件 I2C 版）
 *
 *  帧格式：
 *      写命令：START + 0x78 + 0x00 + cmd  + STOP
 *      写数据：START + 0x78 + 0x40 + data + STOP
 *  0x00 / 0x40 是 SSD1306 的"控制字节"：Co=0, D/C#=0 表示命令，=1 表示数据
 *==============================================================*/

/* 写一条命令 */
static void OLED_WriteCommand(uint8_t Command)
{
    IIC_Start();
    IIC_SendByte(OLED_ADDRESS);
    IIC_WaitAck();
    IIC_SendByte(0x00);             /* 控制字节：写命令 */
    IIC_WaitAck();
    IIC_SendByte(Command);
    IIC_WaitAck();
    IIC_Stop();
}

/* 写一个字节显示数据 */
static void OLED_WriteData(uint8_t Data)
{
    IIC_Start();
    IIC_SendByte(OLED_ADDRESS);
    IIC_WaitAck();
    IIC_SendByte(0x40);             /* 控制字节：写数据 */
    IIC_WaitAck();
    IIC_SendByte(Data);
    IIC_WaitAck();
    IIC_Stop();
}

/* 设置光标：Y = 页地址 0~7，X = 列地址 0~127 */
void OLED_SetCursor(uint8_t Y, uint8_t X)
{
    OLED_WriteCommand((uint8_t)(0xB0 | Y));                    /* 页地址     */
    OLED_WriteCommand((uint8_t)(0x10 | ((X & 0xF0) >> 4)));    /* 列高 4 位  */
    OLED_WriteCommand((uint8_t)(0x00 | (X & 0x0F)));           /* 列低 4 位  */
}

/* 清屏：8 页 × 128 列全写 0（注意：约 40~50 ms，不要放进控制周期） */
void OLED_Clear(void)
{
    uint8_t i, j;

    for (j = 0; j < 8; j++)
    {
        OLED_SetCursor(j, 0);
        for (i = 0; i < 128; i++)
        {
            OLED_WriteData(0x00);
        }
    }
}

/* 只清一行（2 页） */
void OLED_ClearLine(uint8_t Line)
{
    uint8_t i;

    OLED_SetCursor((uint8_t)((Line - 1) * 2), 0);
    for (i = 0; i < 128; i++)
    {
        OLED_WriteData(0x00);
    }

    OLED_SetCursor((uint8_t)((Line - 1) * 2 + 1), 0);
    for (i = 0; i < 128; i++)
    {
        OLED_WriteData(0x00);
    }
}

/* 显示一个字符（8×16，占 2 页） */
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char)
{
    uint8_t i;

    if ((Char < ' ') || (Char > '~'))
    {
        Char = ' ';
    }

    /* 上半页 */
    OLED_SetCursor((uint8_t)((Line - 1) * 2), (uint8_t)((Column - 1) * 8));
    for (i = 0; i < 8; i++)
    {
        OLED_WriteData(OLED_F8x16[Char - ' '][i]);
    }

    /* 下半页 */
    OLED_SetCursor((uint8_t)((Line - 1) * 2 + 1), (uint8_t)((Column - 1) * 8));
    for (i = 0; i < 8; i++)
    {
        OLED_WriteData(OLED_F8x16[Char - ' '][i + 8]);
    }
}

/* 显示字符串 */
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String)
{
    uint8_t i;

    for (i = 0; String[i] != '\0'; i++)
    {
        OLED_ShowChar(Line, (uint8_t)(Column + i), String[i]);
    }
}

/* 求 X 的 Y 次方 */
static uint32_t OLED_Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1;

    while (Y--)
    {
        Result *= X;
    }
    return Result;
}

/* 十进制无符号 */
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    uint8_t i;

    for (i = 0; i < Length; i++)
    {
        OLED_ShowChar(Line, (uint8_t)(Column + i),
                      (char)(Number / OLED_Pow(10, Length - i - 1) % 10 + '0'));
    }
}

/* 十进制带符号（会多占 1 位放 +/-） */
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length)
{
    uint8_t i;
    uint32_t Number1;

    if (Number >= 0)
    {
        OLED_ShowChar(Line, Column, '+');
        Number1 = (uint32_t)Number;
    }
    else
    {
        OLED_ShowChar(Line, Column, '-');
        Number1 = (uint32_t)(-Number);
    }

    for (i = 0; i < Length; i++)
    {
        OLED_ShowChar(Line, (uint8_t)(Column + i + 1),
                      (char)(Number1 / OLED_Pow(10, Length - i - 1) % 10 + '0'));
    }
}

/* 十六进制 */
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    uint8_t i, SingleNumber;

    for (i = 0; i < Length; i++)
    {
        SingleNumber = (uint8_t)(Number / OLED_Pow(16, Length - i - 1) % 16);
        if (SingleNumber < 10)
        {
            OLED_ShowChar(Line, (uint8_t)(Column + i), (char)(SingleNumber + '0'));
        }
        else
        {
            OLED_ShowChar(Line, (uint8_t)(Column + i), (char)(SingleNumber - 10 + 'A'));
        }
    }
}

/* 二进制 */
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    uint8_t i;

    for (i = 0; i < Length; i++)
    {
        OLED_ShowChar(Line, (uint8_t)(Column + i),
                      (char)(Number / OLED_Pow(2, Length - i - 1) % 2 + '0'));
    }
}

/* 浮点：整数部分定宽 IntLen（补 0），小数部分 DecLen 位，四舍五入 */
void OLED_ShowFloatNum(uint8_t Line, uint8_t Column, float Num, uint8_t IntLen, uint8_t DecLen)
{
    uint8_t i;
    uint32_t pow10 = 1;
    int32_t scaled, intPart;
    uint32_t frac;

    if (DecLen > 6U)
    {
        DecLen = 6U;
    }
    for (i = 0; i < DecLen; i++)
    {
        pow10 *= 10U;
    }

    /* 四舍五入到 DecLen 位小数 */
    if (Num >= 0.0f)
    {
        scaled = (int32_t)(Num * (float)pow10 + 0.5f);
    }
    else
    {
        scaled = (int32_t)(Num * (float)pow10 - 0.5f);
    }

    intPart = scaled / (int32_t)pow10;
    frac    = (uint32_t)((scaled < 0) ? -(scaled % (int32_t)pow10)
                                      :  (scaled % (int32_t)pow10));

    OLED_ShowChar(Line, Column, (scaled < 0) ? '-' : '+');
    OLED_ShowNum(Line, (uint8_t)(Column + 1U),
                 (uint32_t)((intPart < 0) ? -intPart : intPart), IntLen);
    OLED_ShowChar(Line, (uint8_t)(Column + 1U + IntLen), '.');
    OLED_ShowNum(Line, (uint8_t)(Column + 2U + IntLen), frac, DecLen);
}

/*==============================================================
 *  初始化（引脚已由 MX_GPIO_Init 配成开漏输出，这里只发命令）
 *==============================================================*/
void OLED_Init(void)
{
    HAL_Delay(100);         /* 等屏幕电源稳定 */

    IIC_Init();             /* SCL / SDA 释放为高，总线空闲 */

    OLED_WriteCommand(0xAE);    /* 关闭显示 */

    OLED_WriteCommand(0xD5);    /* 时钟分频 / 振荡频率 */
    OLED_WriteCommand(0x80);

    OLED_WriteCommand(0xA8);    /* 多路复用率 = 64 */
    OLED_WriteCommand(0x3F);

    OLED_WriteCommand(0xD3);    /* 显示偏移 = 0 */
    OLED_WriteCommand(0x00);

    OLED_WriteCommand(0x40);    /* 显示起始行 = 0 */

    OLED_WriteCommand(0xA1);    /* 左右方向：0xA1 正常 / 0xA0 反置 */

    OLED_WriteCommand(0xC8);    /* 上下方向：0xC8 正常 / 0xC0 反置 */

    OLED_WriteCommand(0xDA);    /* COM 引脚硬件配置 */
    OLED_WriteCommand(0x12);

    OLED_WriteCommand(0x81);    /* 对比度 */
    OLED_WriteCommand(0xCF);

    OLED_WriteCommand(0xD9);    /* 预充电周期 */
    OLED_WriteCommand(0xF1);

    OLED_WriteCommand(0xDB);    /* VCOMH 取消选择级别 */
    OLED_WriteCommand(0x30);

    OLED_WriteCommand(0xA4);    /* 输出跟随 RAM 内容 */

    OLED_WriteCommand(0xA6);    /* 正常显示（0xA7 为反色） */

    OLED_WriteCommand(0x8D);    /* 电荷泵 */
    OLED_WriteCommand(0x14);

    OLED_WriteCommand(0xAF);    /* 开启显示 */

    OLED_Clear();
}
