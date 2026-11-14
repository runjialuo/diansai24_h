#include "report.h"
#include "Serial.h"
#include "Car.h"
#include "attitude.h"
#include "main.h"

/*==============================================================
 *  VOFA+ FireWater 上报
 *
 *  帧格式：pwm_l,pwm_r,yaw\r\n
 *  例：45,-45,12.3
 *
 *      pwm_l / pwm_r : 左右轮实际 PWM（%），来自 Car_SetSpeed 的记录
 *      yaw           : 航向（度，保留 1 位小数）
 *
 *  发送走 Serial_SendArray_IT（中断），不阻塞主循环。
 *==============================================================*/

static uint32_t m_last_ms = 0;

/* 无符号整数转字符串，返回写入长度 */
static uint8_t U32_ToStr(char *dst, uint32_t v)
{
    char    tmp[10];
    uint8_t m = 0U;
    uint8_t i;

    if (v == 0U)
    {
        dst[0] = '0';
        return 1U;
    }

    while ((v > 0U) && (m < sizeof(tmp)))
    {
        tmp[m++] = (char)('0' + (v % 10U));
        v /= 10U;
    }

    for (i = 0U; i < m; i++) dst[i] = tmp[m - 1U - i];
    return m;
}

/* 有符号整数转字符串：-25 -> "-25" */
static uint8_t I16_ToStr(char *dst, int16_t v)
{
    char     tmp[8];
    uint8_t  m = 0U;
    uint8_t  n = 0U;
    uint8_t  i;
    uint16_t u;

    if (v < 0)
    {
        dst[n++] = '-';
        u = (uint16_t)(-v);
    }
    else
    {
        u = (uint16_t)v;
    }

    if (u == 0U)
    {
        dst[n++] = '0';
        return n;
    }

    while ((u > 0U) && (m < sizeof(tmp)))
    {
        tmp[m++] = (char)('0' + (u % 10U));
        u /= 10U;
    }

    /* 数字部分从符号之后开始写，避免把 '-' 覆盖掉 */
    for (i = 0U; i < m; i++) dst[n + i] = tmp[m - 1U - i];

    return (uint8_t)(n + m);
}

/* 浮点转 1 位小数：12.34 -> "12.3"，-5.6 -> "-5.6" */
static uint8_t F1_ToStr(char *dst, float v)
{
    int32_t  scaled;
    uint8_t  n = 0U;
    uint8_t  m;

    if (v >  3200.0f) v =  3200.0f;
    if (v < -3200.0f) v = -3200.0f;

    /* 四舍五入到 0.1 */
    scaled = (int32_t)((v >= 0.0f) ? (v * 10.0f + 0.5f) : (v * 10.0f - 0.5f));

    if (scaled < 0)
    {
        dst[n++] = '-';
        scaled   = -scaled;
    }

    m = U32_ToStr(&dst[n], (uint32_t)(scaled / 10));
    n = (uint8_t)(n + m);

    dst[n++] = '.';
    dst[n++] = (char)('0' + (scaled % 10));

    return n;
}

void Report_Init(void)
{
    m_last_ms = HAL_GetTick();
}

void Report_Task(void)
{
    char    buf[48];
    uint8_t n = 0U;

    if ((HAL_GetTick() - m_last_ms) < REPORT_PERIOD_MS)
    {
        return;
    }
    m_last_ms = HAL_GetTick();

    /* 通道 1：左轮 PWM（%） */
    n += I16_ToStr(&buf[n], e_pwm_l);
    buf[n++] = ',';

    /* 通道 2：右轮 PWM（%） */
    n += I16_ToStr(&buf[n], e_pwm_r);
    buf[n++] = ',';

    /* 通道 3：航向（度） */
    n += F1_ToStr(&buf[n], Attitude_GetYaw());

    /* FireWater 帧尾 */
    buf[n++] = '\r';
    buf[n++] = '\n';

    Serial_SendArray_IT((uint8_t *)buf, n);
}
