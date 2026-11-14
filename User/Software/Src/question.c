#include "question.h"
#include "Software_Config.h"
#include "Car.h"
#include "attitude.h"
#include "line_sensor.h"
#include "line_track.h"
#include "stm32f1xx_hal.h"
#include "yaw_ctrl.h"
#include "mpu6050.h"
#include "main.h"

#define SEG_LINE    0U
#define SEG_ARC     1U

#define PATH_HIT_N      2U
#define ARC_LOST_END_MS 500U     /* 弧线连续丢线超过这个时间才算走完 */
#define ARC_ENTRY_MS    500U     /* 刚进弧线的前 500ms 慢速，等 PD 咬住线 */
#define ARC_ENTRY_PCT   22       /* 入场慢速 PWM */
#define TURN_TOL_DEG    2.0f     /*旋转角度误差*/
#define TURN_BASE       55      /* 转向时两轮的平均 PWM（都向前） */
#define TURN_DIFF       25      /* 差速幅度：55±25 -> 80/30（慢轮在死区之上） */    
#define TURN_TIMEOUT_MS 1500U

typedef struct{
    uint8_t type;
    int16_t turn_angle;
    uint16_t timeout_ms;
}s_seg;

static const s_seg* m_seg_tbl=0;    //当前路线
static uint8_t m_seg_cnt=0;         //路线段数
static uint8_t m_turnIsOk;          //是否完成了旋转
static int16_t m_seg_turnAngle;     //需要旋转的角度
static uint32_t m_turn_t0;
static uint8_t m_seg_idx=0;         //当前跑到第几段
static uint8_t m_seg_hit=0;         //连续几拍压到线
static uint8_t m_seg_armed=0;       //本段是否已见过白底
static uint16_t m_arcLostMs=0;      //弧线段连续丢线时间(ms)
static uint32_t m_seg_t0=0;         //本段路线开始的时间戳

static const s_seg m_tbl_q1[]={
    {SEG_LINE, 0, 12000U},
};

static const s_seg m_tbl_q2[] = {
    {SEG_LINE, 0, 10000U},
    {SEG_ARC,  0, 9000U},
    {SEG_LINE, 0, 10000U},
    {SEG_ARC,  0, 9000U},
};

/* Q3：+ 右转，- 左转（与 yaw 一致，顺时针为正）
   A 右转 38.66° → 到 C 左转 38.66° 对正弧线
   B 左转 38.66° → 到 D 右转 38.66° 对正弧线 */
#define Q3_SEG_LIST                  \
    {SEG_LINE,  -386, 12000U},        \
    {SEG_ARC,  386,  9000U},        \
    {SEG_LINE, 386, 12000U},        \
    {SEG_ARC,   -386,  9000U}

static const s_seg m_tbl_q3[]={
    Q3_SEG_LIST,
};

static const s_seg m_tbl_q4[]={
    Q3_SEG_LIST,Q3_SEG_LIST,Q3_SEG_LIST,Q3_SEG_LIST,
};

/*==============================================================
 *  四个问题统一放这里
 *
 *  每个问题两个接口：
 *      QuesN_Start()    进入时调用一次（按键里触发）
 *      QuesN_Task10ms() 每 10ms 调用，0=进行中 1=完成 2=超时
 *
 *  方向约定（表里的 turn_angle，与 yaw 一致，顺时针为正）：
 *      +  = 右转（顺时针）
 *      -  = 左转（逆时针）
 *  所以 turn_angle 和 yaw 直接用，不需要换算。
 *==============================================================*/

/*==============================================================
 *  共用驱动
 *==============================================================*/

/* 直线：航向环保持方向，base ± delta */
static void DriveStraight(void)
{
    float   d = YawCtrl_Updata10ms();
    int16_t l = (int16_t)(CAR_BASE_STRAIGHT_PCT + d);
    int16_t r = (int16_t)(CAR_BASE_STRAIGHT_PCT - d);

    if (l >  100) l =  100;
    if (l < -100) l = -100;
    if (r >  100) r =  100;
    if (r < -100) r = -100;

    Car_SetSpeed((int8_t)l, (int8_t)r);
}

/* 弧线：循迹环输出差速，base ± delta（base 可指定，用于入弧降速） */
static void DriveLineTrackBase(float base)
{
    float   d = LineTrack_GetDelta();
    int16_t l = (int16_t)(base + d);
    int16_t r = (int16_t)(base - d);
    /*死区限幅*/
    if((l > 0) && (l < 14)) l = 0;
    if((r > 0) && (r < 14)) r = 0;
    /*限幅*/
    if (l >  100) l =  100;
    if (l < -100) l = -100;
    if (r >  100) r =  100;
    if (r < -100) r = -100;

    Car_SetSpeed((int8_t)l, (int8_t)r);
}

static void DriveLineTrack(void)
{
    DriveLineTrackBase((float)CAR_BASE_ARC_PCT);
}

static void Path_Start(const s_seg *tbl, uint8_t cnt){
    Car_Stop();
    m_seg_tbl=tbl;
    m_seg_cnt=cnt;
    m_seg_idx=0U;
    m_seg_hit=0U;
    m_seg_armed=0U;
    m_arcLostMs=0U;

    if(Attitude_IsOk()==0U){
        m_seg_cnt=0;
        return;
    }
    Attitude_Enable();
    HAL_Delay(500);
    MPU6050_CalibGyro(300);
    Attitude_Reset();
    YawCtrl_Reset();
    LineTrack_Init();

    m_seg_turnAngle = tbl[0].turn_angle;
    m_turnIsOk = (m_seg_turnAngle==0)? 1U:0U;
    m_turn_t0 = HAL_GetTick();
    YawCtrl_SetTargetDeg((float)m_seg_turnAngle/10.0f);

    m_seg_t0=HAL_GetTick();
}

/*
0->进行中，1->完成，2->超时/失败
*/
static uint8_t Path_Run10ms(void){
    uint32_t now=HAL_GetTick();
    uint8_t ir=Line_ReadRaw();
    const s_seg *s;

    if((m_seg_tbl==0)||(m_seg_idx>=m_seg_cnt)){
        Car_Stop();
        return 2U;
    }
    s = &m_seg_tbl[m_seg_idx];

    if(!m_turnIsOk){
        /* 表里 + = 右转，- = 左转；yaw 顺时针为正，直接相减 */
        float e = ((float)m_seg_turnAngle/10.0f) - Attitude_GetYaw();

        if((e>-TURN_TOL_DEG)&&e<(TURN_TOL_DEG)){    
            Car_Stop();                      
            m_turnIsOk=1;
            YawCtrl_Reset();
            m_seg_t0=now;
        }
        else{
            /* 边走边转：两轮都向前，靠差速拐弯
               原地转四轮横向滑擦，摩擦太大带不动 */
            if(e > 0.0f){
                Car_SetSpeed(TURN_BASE+TURN_DIFF, TURN_BASE-TURN_DIFF);   /* 右转 */
            }else{
                Car_SetSpeed(TURN_BASE-TURN_DIFF, TURN_BASE+TURN_DIFF);   /* 左转 */
            }

            if((now-m_turn_t0)>TURN_TIMEOUT_MS){
                /* 转不动也别把整道题停掉：强制认为转完，继续往下跑 */
                m_turnIsOk = 1;
                YawCtrl_Reset();
                m_seg_t0 = now;
            }
            return 0U;
        }      
    }

    if(s->type == SEG_LINE){
        if(m_seg_armed==0U){
            if(ir==0U) m_seg_armed=1U;      /* 先见到白底，才允许判线 */
            DriveStraight();
            return 0U;
        }

        if(ir!=0U){if(m_seg_hit<PATH_HIT_N) m_seg_hit++;}
        else {m_seg_hit=0U;}

        if(m_seg_hit<PATH_HIT_N){
            DriveStraight();
            return 0U;
        }
    }
    else{
        if(ir!=0U){
            m_arcLostMs=0U;                        /* 有线 */
            LineTrack_Update(ir, Attitude_GetGz());

            if((now-m_seg_t0) < ARC_ENTRY_MS){
                /* 刚进弧线：低速 + 正常转向，慢慢把车拉上线 */
                DriveLineTrackBase((float)ARC_ENTRY_PCT);
            }else{
                DriveLineTrack();
            }
        }else{
            if(m_arcLostMs<60000U) m_arcLostMs+=10U;

            /* 丢线：直行找线，不继续按旧偏差打方向 */
            Car_SetSpeed((int8_t)CAR_BASE_ARC_PCT,(int8_t)CAR_BASE_ARC_PCT);
        }

        if(m_arcLostMs<ARC_LOST_END_MS) return 0U;  /* 还没到 500ms，不算走完 */
    }

    Reminder_Start(120);

    m_seg_idx++;
    if(m_seg_idx>=m_seg_cnt){
        Car_Stop();
        return 1U;
    }
    m_seg_hit=0U;
    m_seg_armed=0U;
    m_arcLostMs=0U;
    m_seg_t0=now;

    m_seg_turnAngle = m_seg_tbl[m_seg_idx].turn_angle;
    m_turnIsOk = (m_seg_turnAngle == 0)? 1U:0U;
    m_turn_t0  = now;

    YawCtrl_Reset();
    YawCtrl_SetTargetDeg((float)m_seg_turnAngle / 10.0f);
    LineTrack_Init();
    return 0U;
}


void Ques1_Start(void){
    Path_Start(m_tbl_q1, 1U);
}

uint8_t Ques1_Task10ms(void){
    return Path_Run10ms();
}

void Ques2_Start(void)
{
    Path_Start(m_tbl_q2, (uint8_t)(sizeof(m_tbl_q2)/sizeof(m_tbl_q2[0])));
}

uint8_t Ques2_Task10ms(void)
{
    return Path_Run10ms();
}

void Ques3_Start(void)
{
    Path_Start(m_tbl_q3, (uint8_t)(sizeof(m_tbl_q3)/sizeof(m_tbl_q3[0])));
}

uint8_t Ques3_Task10ms(void)
{
    return Path_Run10ms();
}

void Ques4_Start(void)
{
    Path_Start(m_tbl_q4, (uint8_t)(sizeof(m_tbl_q4)/sizeof(m_tbl_q4[0])));
}

uint8_t Ques4_Task10ms(void)
{
    return Path_Run10ms();
}


