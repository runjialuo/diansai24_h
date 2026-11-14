#include "line_track.h"
#include "line_sensor.h"
#include "Software_Config.h"

/*==============================================================
 *  循迹控制实现（Software 层）
 *
 *  控制律：
 *      err   = Σ(w_i × 压线_i) / 压线数        ← 线相对车体中心的位置(cm)
 *      delta = KP×err + KD×(err − err_prev) + KFF×gz
 *
 *  只做两件事：算偏差、出差速。
 *  段结束（看到线 / 丢线）由上层状态机用 OnLine / IsLost 判断。
 *==============================================================*/

/* 每次 Update 的周期（ms），用于把时间参数换算成"调用次数" */
#define LINE_LOST_CALLS     (LINE_LOST_MS  / LINE_PERIOD_MS)

static const float m_weight[4] =
{
    LINE_WEIGHT_1, LINE_WEIGHT_2, LINE_WEIGHT_3, LINE_WEIGHT_4
};

static float    m_err           = 0.0f;  /* 本周期偏差（cm）       */
static float    m_err_prev      = 0.0f;  /* 上周期偏差（D 项用）   */
static float    m_cur_delta     = 0.0f;  /* 本周期差速输出（%）    */
static float    m_last_delta    = 0.0f;  /* 上一周期差速输出（%）  */
static uint8_t  m_online        = 0;     /* 本周期是否压线         */
static uint8_t  m_lost          = 0;     /* 丢线超时标志           */
static uint16_t m_lost_cnt      = 0;     /* 连续丢线计数           */

void LineTrack_Init(void)
{
    m_err        = 0.0f;
    m_err_prev   = 0.0f;
    m_cur_delta  = 0.0f;
    m_last_delta = 0.0f;
    m_online     = 0;
    m_lost       = 0;
    m_lost_cnt   = 0;
}

/*--------------------------------------------------------------
 *  每 10ms 调用
 *------------------------------------------------------------*/
void LineTrack_Update(uint8_t state, float gz_dps)
{
    uint8_t i;
    uint8_t cnt = 0;
    float   sum = 0.0f;
    float   err;

    /* ---- 1. 统计压线数与加权位置 ---- */
    for (i = 0; i < 4U; i++)
    {
        if (state & (uint8_t)(1U << i))
        {
            sum += m_weight[i];
            cnt++;
        }
    }

    /* ---- 2. 偏差计算 / 丢线处理 ---- */
    if (cnt == 0U)
    {
        /* 全白：这一拍没有有效偏差，沿用上一次
           （不能清零，否则一丢线车就会突然回正） */
        m_online = 0;
        if (m_lost_cnt < LINE_LOST_CALLS)
        {
            m_lost_cnt++;
        }
        else
        {
            m_lost = 1;
        }
        err = m_err_prev;
    }
    else
    {
        m_online   = 1;
        m_lost_cnt = 0;
        m_lost     = 0;
        err        = sum / (float)cnt;

        /* 限幅，防止极端情况打出过大差速 */
        if (err >  LINE_ERR_MAX) err =  LINE_ERR_MAX;
        if (err < -LINE_ERR_MAX) err = -LINE_ERR_MAX;
    }

    /* ---- 3. PD + 角速度前馈 ---- */
    m_cur_delta = (LINE_KP * err)+ (LINE_KD * (err - m_err_prev))+ (LINE_KFF * gz_dps);
    /*添加滤波*/
    m_cur_delta = 0.7f*m_cur_delta + 0.3f*m_last_delta;

    if (m_cur_delta >  LINE_DELTA_MAX) m_cur_delta =  LINE_DELTA_MAX;
    if (m_cur_delta < -LINE_DELTA_MAX) m_cur_delta = -LINE_DELTA_MAX;
    
    m_last_delta = m_cur_delta;

    /* ---- 4. 保存状态 ---- */
    m_err      = err;
    m_err_prev = err;
}

/*--------------------------------------------------------------
 *  访问接口
 *------------------------------------------------------------*/
float LineTrack_GetErr(void)
{
    return m_err;
}

float LineTrack_GetDelta(void)
{
    return m_cur_delta;
}

uint8_t LineTrack_OnLine(void)
{
    return m_online;
}

uint8_t LineTrack_IsLost(void)
{
    return m_lost;
}
