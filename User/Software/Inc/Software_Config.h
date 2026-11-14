#ifndef __SOFTWARE_CONFIG_H
#define __SOFTWARE_CONFIG_H

#include <stdint.h>
#include "Car.h"

/*==============================================================
 *  全局参数集中表
 *
 *  原则：业务代码里不出现裸数字，所有可调参数都放在这里。
 *  现场调车时只改这个文件，不翻业务代码。
 *==============================================================*/

/*==============================================================
 *  【循迹】四路灰度
 *  位置单位：cm（以车体中线为 0，右为正）
 *  输出单位：% 占空比差速（叠加到速度环目标上）
 *==============================================================*/

/* 采样周期（与 10ms 控制节拍一致） */
#define LINE_PERIOD_MS          10U

/* 四路探头的等效位置：X1 --2.8-- X2 -0.7- X3 --2.8-- X4 */
#define LINE_WEIGHT_1           ( 3.15f)     /* X1 最右 */
#define LINE_WEIGHT_2           ( 0.35f)     /* X2      */
#define LINE_WEIGHT_3           ( -0.35f)    /* X3      */
#define LINE_WEIGHT_4           ( -3.15f)    /* X4 最左 */

/* PID 参数（在 line_track.c 里组装成 pid_param_t 交给 pid.c）
 *   增益按"每 10ms 一拍"标定，不是按秒——改控制周期要按比例重算
 *   KP  ：每偏 1cm 给多少 % 差速。起步 6.0，摆得太凶就减小
 *   KD  ：每拍的偏差变化量对应的阻尼，抑制蛇形
 *   KFF ：角速度前馈。弧线 ω = v/R = 30/40 = 0.75rad/s ≈ 43°/s，
 *         需要约 15% 差速，所以 KFF ≈ 15/43 ≈ 0.35 */
#define LINE_KP                 6.0f
#define LINE_KD                 2.0f
#define LINE_KFF                0.35f

/* 输出限幅防止一侧反转 */
#define LINE_DELTA_MAX          32.0f

/* 偏差限幅：探头最外侧约 3.15cm，留一点余量 */
#define LINE_ERR_MAX            4.0f

/* 丢线判定：连续 0 路压线超过这个时间 → IsLost = 1 */
#define LINE_LOST_MS            100U

/*==============================================================
 *  【底盘】开环基础速度（当前没有速度闭环）
 *==============================================================*/
#define CAR_BASE_STRAIGHT_PCT   60      /* 直线段 PWM % */
#define CAR_BASE_ARC_PCT        40      /* 弧线段 PWM %（慢一点更贴线） */

/*==============================================================
 *  【航向环】保持车头朝向
 *  输入：误差（度）  输出：差速（%）
 *==============================================================*/
#define YAW_KP                  1.2f    /* 比例：%/度              */
#define YAW_KD                  0.12f   /* 微分：%/(°/s)，陀螺阻尼 */
#define YAW_KI                  0.05f   /* 积分：每拍累加          */
#define YAW_IMAX                15.0f   /* 积分限幅（抗饱和）      */
#define YAW_OUTMAX              40.0f   /* 输出限幅 ±%             */
#define YAW_DEADBAND_DEG        0.1f    /* 死区 ±度                */ 

extern  volatile uint8_t e_flag_10ms;

/* 三个题目状态变量（定义在 main.c）：
     Question_Choose —— CNT 选中的题目 1~4，屏幕显示它
     cur_Question    —— 正在执行的题目，0 = 未启动；START 切换
     last_Question   —— 上一拍的 cur_Question，用来发现变化并 Start/Reset */
extern uint8_t Question_Choose;
extern uint8_t cur_Question;

void Sofaware_Init(void);
void Reminder_Start(uint16_t ms);
void Reminder_Tick(void);

#endif