#include "attitude.h"
#include "mpu6050.h"
#include "main.h"

/*==============================================================
 *  姿态解算实现
 *
 *  纯陀螺积分：yaw += gz * dt
 *      - 不用加速度计做倾角修正（小车在水平面跑，倾角信息没用）
 *      - 不做卡尔曼/互补滤波（没有磁力计，加速度计也纠正不了 yaw）
 *      - 漂移靠「每段开始时 ResetYaw」消除，而不是靠滤波
 *==============================================================*/

#define ATTITUDE_MAX_DT_S   0.050f   /* dt 上限 50ms：防止长时间阻塞后积分跳变 */
#define ATTITUDE_MIN_DT_S   0.001f

/* ---- 抑制陀螺零偏残留 ----
 * MPU6050 就算标定过，仍会有 0.1~0.5 °/s 的温漂残留，
 * 直接积分就是一分钟几十度。两层处理：
 *   1) 慢速残差跟踪：读数很接近静止时，把残差慢慢"吃掉"
 *      （真正的转向角速度都远大于 0.5 °/s，不会被误吃）
 *   2) 死区：小于 0.15 °/s 直接当 0，彻底掐掉蠕动 */
#define ATTITUDE_TRACK_WINDOW_DPS   1.50f    /* 进入跟踪的窗口        */
#define ATTITUDE_TRACK_GAIN         0.005f   /* 每个更新周期吸收比例  */
#define ATTITUDE_DEADBAND_DPS       0.15f    /* 死区                  */

static float    m_yaw      = 0.0f;
static float    m_gz       = 0.0f;
static float    m_bias_res = 0.0f;   /* 运行时残差补偿（° /s） */
static uint8_t  m_ok       = 0;
static uint8_t  m_enabled  = 0;      /* 0 = 停止解算（不读 I2C、不积分） */
static uint32_t m_last_ms  = 0;

/*--------------------------------------------------------------
 *  初始化：IMU + 零偏标定 + yaw 清零
 *  返回 1 = 成功，0 = 失败（IMU 无应答）
 *  ⚠️ 标定约 0.6 秒（预热 300ms + 采样 300 次），期间必须静止
 *------------------------------------------------------------*/
uint8_t Attitude_Init(void)
{
    m_yaw      = 0.0f;
    m_gz       = 0.0f;
    m_bias_res = 0.0f;
    m_last_ms  = HAL_GetTick();

    m_enabled  = 0U;             /* 上电不积分，等按 START 再开 */

    m_ok = (MPU6050_Init() == 0) ? 1U : 0U;

    /* 零偏标定移到按 START 时做：那时车已放好、静止，标定才准 */

    return m_ok;
}

/*--------------------------------------------------------------
 *  主循环调用：按实际经过的时间积分
 *------------------------------------------------------------*/
void Attitude_Update(void)
{
    uint32_t now;
    float    dt;

    if (!m_ok || !m_enabled)
    {
        return;
    }

    now = HAL_GetTick();
    dt  = (float)(now - m_last_ms) * 0.001f;

    if (dt < ATTITUDE_MIN_DT_S)
    {
        return;                      /* 主循环太快，这一轮不积分 */
    }
    if (dt > ATTITUDE_MAX_DT_S)
    {
        dt = ATTITUDE_MAX_DT_S;      /* 防止长时间阻塞后积分跳变 */
    }
    m_last_ms = now;

    m_gz = MPU6050_GetGyroZDps() - m_bias_res;   /* 已扣标定零偏 + 运行时残差 */

    /* 1) 慢速残差跟踪：读数接近静止时，把残留零偏慢慢吸收掉
          （真实转向的角速度远大于 0.5 °/s，不会被误吃） */
    if ((m_gz > -ATTITUDE_TRACK_WINDOW_DPS) && (m_gz < ATTITUDE_TRACK_WINDOW_DPS))
    {
        m_bias_res += m_gz * ATTITUDE_TRACK_GAIN;
    }

    /* 2) 死区：小于 0.15 °/s 视为完全静止 */
    if ((m_gz > -ATTITUDE_DEADBAND_DPS) && (m_gz < ATTITUDE_DEADBAND_DPS))
    {
        m_gz = 0.0f;
    }

    m_yaw += m_gz * dt;

    /* 归一化到 -180 ~ +180 */
    while (m_yaw > 180.0f)
    {
        m_yaw -= 360.0f;
    }
    while (m_yaw < -180.0f)
    {
        m_yaw += 360.0f;
    }
}

float Attitude_GetYaw(void)
{
    return m_yaw;
}

float Attitude_GetGz(void)
{
    return m_gz;
}

void Attitude_ResetYaw(void)
{
    m_yaw = 0.0f;
}

uint8_t Attitude_IsOk(void)
{
    return m_ok;
}

/*--------------------------------------------------------------
 *  开始 / 停止解算
 *------------------------------------------------------------*/
void Attitude_Enable(void)
{
    m_last_ms = HAL_GetTick();       /* 重新对齐时间，避免 dt 跳变 */
    m_enabled = 1U;
}

void Attitude_Disable(void)
{
    m_enabled  = 0U;
    m_gz       = 0.0f;
    m_bias_res = 0.0f;
}

uint8_t Attitude_IsEnabled(void)
{
    return m_enabled;
}

/*--------------------------------------------------------------
 *  清空全部姿态数据（RESET 键用）
 *------------------------------------------------------------*/
void Attitude_Reset(void)
{
    m_yaw      = 0.0f;
    m_gz       = 0.0f;
    m_bias_res = 0.0f;
    m_last_ms  = HAL_GetTick();
}