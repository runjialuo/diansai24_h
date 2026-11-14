#include "ui.h"

#if UI_ENABLE

#include "OLED.h"
#include "Software_Config.h"      /* Question_Choose / cur_Question   */
#include "attitude.h"             /* Attitude_GetYaw/GetGz/IsOk     */
#include "line_sensor.h"          /* Line_ReadRaw                   */
#include "line_track.h"           /* LineTrack_GetErr/GetDelta      */
#include "yaw_ctrl.h"             /* YawCtrl_GetDelta               */

/* ============================================================
 *  显示布局（每行 16 字符，固定宽度避免残影）
 *
 *  第 1 行：Q1  IMU:OK
 *  第 2 行：IR:1010 ON
 *  第 3 行：Y:+0123 G:-0005     (航向 ×10，角速度 ×10)
 *  第 4 行：D:+0300 E:+0123     (差速 ×10，偏差 ×100)
 * ============================================================ */

static uint8_t m_line = 0;          /* 当前刷到第几行（0~3） */

/* 限幅到 4 位十进制能显示的范围，防止数值溢出显示成 0000 */
static int32_t Ui_Clamp(int32_t v)
{
    if (v >  9999) return  9999;
    if (v < -9999) return -9999;
    return v;
}

void Ui_Init(void)
{
    OLED_Clear();
    OLED_ShowString(1, 1, "Q:1 STP IMU:OK");
    OLED_ShowString(2, 1, "IR:    ---- ");
    OLED_ShowString(3, 1, "Y:+00000 G:+00000");
    OLED_ShowString(4, 1, "D:+00000 E:+00000");
    m_line = 0;
}

/* 第 1 行：题目号 + 运行状态 + IMU
     Q:1 RUN IMU:OK
     1234567890123456 */
static void Ui_Line1(void)
{
    OLED_ShowString(1, 1, "Q:");
    OLED_ShowNum(1, 3, Question_Choose, 1);
    OLED_ShowString(1, 5, (cur_Question != 0U) ? "RUN" : "STP");
    OLED_ShowString(1, 9, "IMU:");
    OLED_ShowString(1, 13, Attitude_IsOk() ? "OK" : "NG");
}

/* 第 2 行：四路灰度 + 是否压线 */
static void Ui_Line2(void)
{
    uint8_t ir = Line_ReadRaw();

    OLED_ShowString(2, 1, "IR:");
    OLED_ShowBinNum(2, 4, ir, 4);
    if (ir != 0U)
    {
        OLED_ShowString(2, 9, "ON  ");
    }
    else
    {
        OLED_ShowString(2, 9, "----");
    }
}

/* 第 3 行：航向 / 角速度（×10） */
static void Ui_Line3(void)
{
    OLED_ShowString(3, 1, "Y:");
    OLED_ShowSignedNum(3, 3, Ui_Clamp((int32_t)(Attitude_GetYaw() * 10.0f)), 4);

    OLED_ShowString(3, 9, "G:");
    OLED_ShowSignedNum(3, 11, Ui_Clamp((int32_t)(Attitude_GetGz() * 10.0f)), 4);
}

/* 第 4 行：差速 / 偏差 */
static void Ui_Line4(void)
{
    OLED_ShowString(4, 1, "D:");
    OLED_ShowSignedNum(4, 3, Ui_Clamp((int32_t)(YawCtrl_GetDelta() * 10.0f)), 4);

    OLED_ShowString(4, 9, "E:");
    OLED_ShowSignedNum(4, 11, Ui_Clamp((int32_t)(LineTrack_GetErr() * 100.0f)), 4);
}

static uint8_t m_skip = 0;          /* 跑车时的降频计数 */

void Ui_Refresh(void)
{
    /* 停车：每 100ms 刷一行（4 行轮转，0.4 秒刷完一屏）
       跑车：每 500ms 刷一行（一行约 40ms I2C，占空比约 8%） */
    if (cur_Question != 0U)
    {
        m_skip++;
        if (m_skip < 5U)
        {
            return;
        }
    }
    m_skip = 0;

    switch (m_line)
    {
    case 0: Ui_Line1(); break;
    case 1: Ui_Line2(); break;
    case 2: Ui_Line3(); break;
    default: Ui_Line4(); break;
    }

    m_line = (uint8_t)((m_line + 1U) & 0x03U);   /* 4 行轮转 */
}

/* 立即重画全部 4 行：按键后调用，让屏幕马上反映新状态
   ⚠️ 约 150ms 的 I2C 时间，只在按键时用 */
void Ui_RefreshNow(void)
{
    m_line = 0;
    m_skip = 0;

    Ui_Line1();
    Ui_Line2();
    Ui_Line3();
    Ui_Line4();
}

#else   /* UI_ENABLE == 0 */

void Ui_Init(void)       { }
void Ui_Refresh(void)    { }
void Ui_RefreshNow(void) { }

#endif  /* UI_ENABLE */
