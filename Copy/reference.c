#include "reference.h"
#include <string.h>

/*==============================================================
 *  参考实现汇总（不参与编译）
 *
 *  这份文件里没有一行硬件代码：
 *      读陀螺  → 由调用者把 gz_dps 传进来
 *      读灰度  → 由调用者把 4 位掩码传进来
 *      发串口  → 由调用者拿到缓冲区自己发
 *  所以它可以单独用 gcc 编译、在 PC 上跑单元测试。
 *==============================================================*/

/*==============================================================
 *  0. 可调参数（对应工程的 config.h）
 *==============================================================*/

/* 循迹 */
#define LINE_PERIOD_MS          10U
#define LINE_WEIGHT_1           (-3.15f)
#define LINE_WEIGHT_2           (-0.35f)
#define LINE_WEIGHT_3           ( 0.35f)
#define LINE_WEIGHT_4           ( 3.15f)
#define LINE_KP                 6.0f
#define LINE_KD                 2.0f
#define LINE_KFF                0.35f
#define LINE_DELTA_MAX          60.0f
#define LINE_ERR_MAX            4.0f
#define LINE_LOST_MS            100U

/* 姿态 */
#define ATTITUDE_MAX_DT_S       0.050f
#define ATTITUDE_MIN_DT_S       0.001f
#define ATTITUDE_TRACK_WINDOW   0.50f
#define ATTITUDE_TRACK_GAIN     0.001f
#define ATTITUDE_DEADBAND       0.15f

/*==============================================================
 *  1. 通用 PID
 *==============================================================*/

/* 对称限幅；lim <= 0 表示不限 */
static float ClampF(float v, float lim)
{
    if (lim <= 0.0f) return v;
    if (v >  lim)    return  lim;
    if (v < -lim)    return -lim;
    return v;
}

void Ref_PID_Init(pid_t *pid, uint8_t mode, const pid_param_t *param)
{
    if (pid == 0) return;

    if (param != 0)
    {
        pid->p = *param;
    }
    else
    {
        pid->p.kp = 0.0f;      pid->p.ki = 0.0f;      pid->p.kd = 0.0f;
        pid->p.out_max = 0.0f; pid->p.i_max = 0.0f;   pid->p.deadband = 0.0f;
    }
    pid->mode = mode;
    Ref_PID_Reset(pid);
}

void Ref_PID_Reset(pid_t *pid)
{
    if (pid == 0) return;
    pid->e1    = 0.0f;
    pid->e2    = 0.0f;
    pid->integ = 0.0f;
    pid->out   = 0.0f;
}

float Ref_PID_Calc(pid_t *pid, float target, float actual)
{
    float e, p_term, d_term;

    if (pid == 0) return 0.0f;

    /* 误差 + 死区 */
    e = target - actual;
    if ((pid->p.deadband > 0.0f) &&
        (e < pid->p.deadband) && (e > -pid->p.deadband))
    {
        e = 0.0f;
    }

    if (pid->mode == PID_MODE_INCREMENTAL)
    {
        /* 增量式：Δu = Kp(e−e₁) + Ki·e + Kd(e−2e₁+e₂)
           天然抗积分饱和，输出在上一拍基础上累加 */
        pid->out += (pid->p.kp * (e - pid->e1))
                  + (pid->p.ki * e)
                  + (pid->p.kd * (e - 2.0f * pid->e1 + pid->e2));
        pid->out  = ClampF(pid->out, pid->p.out_max);
    }
    else
    {
        /* 位置式：积分累加 + 抗饱和，再求和 */
        pid->integ += pid->p.ki * e;
        pid->integ  = ClampF(pid->integ, pid->p.i_max);

        p_term = pid->p.kp * e;
        d_term = pid->p.kd * (e - pid->e1);

        pid->out = ClampF(p_term + pid->integ + d_term, pid->p.out_max);
    }

    /* 误差历史：先 e2 后 e1，顺序别反 */
    pid->e2 = pid->e1;
    pid->e1 = e;

    return pid->out;
}

void Ref_PID_SetGains(pid_t *pid, float kp, float ki, float kd)
{
    if (pid == 0) return;
    pid->p.kp = kp;
    pid->p.ki = ki;
    pid->p.kd = kd;
}

/*==============================================================
 *  2. 姿态：陀螺积分
 *==============================================================*/

void Ref_Attitude_Init(attitude_t *a)
{
    if (a == 0) return;
    a->yaw      = 0.0f;
    a->gz       = 0.0f;
    a->bias_res = 0.0f;
}

void Ref_Attitude_Step(attitude_t *a, float gz_dps, float dt_s)
{
    if (a == 0) return;

    if (dt_s < ATTITUDE_MIN_DT_S) return;
    if (dt_s > ATTITUDE_MAX_DT_S) dt_s = ATTITUDE_MAX_DT_S;

    a->gz = gz_dps - a->bias_res;

    /* 慢速残差跟踪：读数接近静止时把残留零偏吃掉
       （真实转向远大于 0.5 °/s，不会被误吃） */
    if ((a->gz > -ATTITUDE_TRACK_WINDOW) && (a->gz < ATTITUDE_TRACK_WINDOW))
    {
        a->bias_res += a->gz * ATTITUDE_TRACK_GAIN;
    }

    /* 死区：小于 0.15 °/s 当静止 */
    if ((a->gz > -ATTITUDE_DEADBAND) && (a->gz < ATTITUDE_DEADBAND))
    {
        a->gz = 0.0f;
    }

    a->yaw += a->gz * dt_s;

    /* 归一化到 −180 ~ +180 */
    while (a->yaw >  180.0f) a->yaw -= 360.0f;
    while (a->yaw < -180.0f) a->yaw += 360.0f;
}

void Ref_Attitude_ResetYaw(attitude_t *a)
{
    if (a == 0) return;
    a->yaw = 0.0f;
}

/*==============================================================
 *  3. 循迹：加权偏差 + PD + 前馈
 *==============================================================*/

static const float s_weight[4] =
{
    LINE_WEIGHT_1, LINE_WEIGHT_2, LINE_WEIGHT_3, LINE_WEIGHT_4
};

#define LINE_LOST_CALLS     (LINE_LOST_MS  / LINE_PERIOD_MS)

void Ref_Line_Init(line_track_t *lt)
{
    if (lt == 0) return;
    lt->err         = 0.0f;
    lt->err_prev    = 0.0f;
    lt->delta       = 0.0f;
    lt->online      = 0;
    lt->lost_cnt    = 0;
}

void Ref_Line_Update(line_track_t *lt, uint8_t state, float gz_dps)
{
    uint8_t i, cnt = 0;
    float   sum = 0.0f;
    float   err;

    if (lt == 0) return;

    /* 1. 加权位置 */
    for (i = 0; i < 4U; i++)
    {
        if (state & (uint8_t)(1U << i))
        {
            sum += s_weight[i];
            cnt++;
        }
    }

    /* 2. 偏差 / 丢线 */
    if (cnt == 0U)
    {
        lt->online = 0;
        if (lt->lost_cnt < LINE_LOST_CALLS) lt->lost_cnt++;
        err = lt->err_prev;          /* 丢线时沿用上一次，不能清零 */
    }
    else
    {
        lt->online   = 1;
        lt->lost_cnt = 0;
        err = sum / (float)cnt;

        if (err >  LINE_ERR_MAX) err =  LINE_ERR_MAX;
        if (err < -LINE_ERR_MAX) err = -LINE_ERR_MAX;
    }

    /* 3. PD + 角速度前馈 */
    lt->delta = (LINE_KP  * err)
              + (LINE_KD  * (err - lt->err_prev))
              + (LINE_KFF * gz_dps);

    if (lt->delta >  LINE_DELTA_MAX) lt->delta =  LINE_DELTA_MAX;
    if (lt->delta < -LINE_DELTA_MAX) lt->delta = -LINE_DELTA_MAX;

    /* 4. 保存 */
    lt->err      = err;
    lt->err_prev = err;
}

/*==============================================================
 *  4. 蓝牙上报：定点小数转字符串 + 组帧
 *
 *  ⚠️ 为什么不用 sprintf("%f")
 *     本工程链接 newlib-nano（--specs=nano.specs），
 *     默认不支持浮点格式化，%f 会输出空/乱码（除非加 -u _printf_float）。
 *     自己转成整数部分 + '.' + 两位小数，零依赖、体积小、速度快。
 *==============================================================*/

/* 无符号整数 → 十进制字符串，返回写入长度 */
static uint8_t U32_ToStr(char *dst, uint32_t v)
{
    char    tmp[10];
    uint8_t n = 0, i;

    if (v == 0U)
    {
        dst[0] = '0';
        return 1;
    }

    while ((v > 0U) && (n < sizeof(tmp)))
    {
        tmp[n++] = (char)('0' + (v % 10U));
        v /= 10U;
    }

    for (i = 0; i < n; i++)
    {
        dst[i] = tmp[n - 1U - i];
    }
    return n;
}

/* 定点小数（保留 2 位），返回写入长度：-12.345 → "-12.35" */
static uint8_t F2_ToStr(char *dst, float v)
{
    int32_t scaled;
    int32_t ip, fp;
    uint8_t n = 0;

    /* 四舍五入到 0.01 */
    scaled = (int32_t)((v >= 0.0f) ? (v * 100.0f + 0.5f) : (v * 100.0f - 0.5f));

    ip = scaled / 100;
    fp = scaled % 100;
    if (fp < 0) fp = -fp;
    if (ip < 0) ip = -ip;

    if (scaled < 0) dst[n++] = '-';

    n += U32_ToStr(dst + n, (uint32_t)ip);
    dst[n++] = '.';
    dst[n++] = (char)('0' + (fp / 10));
    dst[n++] = (char)('0' + (fp % 10));

    return n;
}

int Ref_Report_BuildFrame(char *buf, int size,
                          float c0, float c1, float c2, float c3)
{
    char    tmp[64];
    uint8_t n = 0;
    int     i;

    if ((buf == 0) || (size <= 0)) return 0;

    n += F2_ToStr(&tmp[n], c0);  tmp[n++] = ',';
    n += F2_ToStr(&tmp[n], c1);  tmp[n++] = ',';
    n += F2_ToStr(&tmp[n], c2);  tmp[n++] = ',';
    n += F2_ToStr(&tmp[n], c3);
    tmp[n++] = '\r';
    tmp[n++] = '\n';

    if ((int)n >= size) return 0;      /* 缓冲区不够，宁可不发 */

    for (i = 0; i < (int)n; i++)
    {
        buf[i] = tmp[i];
    }
    return (int)n;
}
