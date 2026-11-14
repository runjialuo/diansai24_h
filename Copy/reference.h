#ifndef __REFERENCE_H
#define __REFERENCE_H

#include <stdint.h>

/*==============================================================
 *  参考实现汇总（不参与编译）
 *
 *  本文件把工程里 4 个 Software 模块（姿态、PID、循迹、上报）
 *  的算法部分抽出来，做成**零硬件依赖**的独立版本，方便：
 *      - 通读一遍理解控制链路
 *      - 自己重写时对照
 *      - 在 PC 上单独编译测试（gcc 即可，不需要 HAL）
 *
 *  与工程代码的关系：
 *      工程里硬件访问在 Hardware 层（mpu6050.c / line_sensor.c / Serial.c），
 *      这里把这些调用改成"参数传入"，所以本文件只依赖 <stdint.h> 和 <string.h>。
 *
 *  单位约定：
 *      角度：度（°），航向范围 −180 ~ +180，顺时针为正
 *      角速度：°/s
 *      线偏差：cm，线在车体右侧为正
 *      输出：百分比差速（%），左轮 +delta、右轮 −delta
 *      周期：固定 10 ms（增益按"每拍"标定，不含 dt）
 *==============================================================*/

/*==============================================================
 *  1. 通用 PID
 *==============================================================*/

/* 可调参数（可以整体存 Flash / 从蓝牙下发） */
typedef struct
{
    float kp;          /* 比例                        */
    float ki;          /* 积分（每拍累加一次）        */
    float kd;          /* 微分                        */
    float out_max;     /* 输出对称限幅，<= 0 不限     */
    float i_max;       /* 积分限幅（抗饱和），<= 0 不限 */
    float deadband;    /* 误差死区，0 关闭            */
} pid_param_t;

/* 运行状态（不参与保存） */
typedef struct
{
    pid_param_t p;
    float e1;          /* 上一拍误差 e₁                 */
    float e2;          /* 上上拍误差 e₂（增量式用）     */
    float integ;       /* 积分累加（位置式用）          */
    float out;         /* 上一拍输出                    */
    uint8_t mode;      /* PID_MODE_POSITION / INCREMENTAL */
} pid_t;

#define PID_MODE_POSITION       0U
#define PID_MODE_INCREMENTAL    1U

void  Ref_PID_Init(pid_t *pid, uint8_t mode, const pid_param_t *param);
void  Ref_PID_Reset(pid_t *pid);
float Ref_PID_Calc(pid_t *pid, float target, float actual);
void  Ref_PID_SetGains(pid_t *pid, float kp, float ki, float kd);

/*==============================================================
 *  2. 姿态（陀螺积分成航向）
 *==============================================================*/

typedef struct
{
    float yaw;          /* 当前航向，°                */
    float gz;           /* 当前角速度，°/s（已扣零偏）*/
    float bias_res;     /* 运行时残差补偿，°/s        */
} attitude_t;

void  Ref_Attitude_Init(attitude_t *a);
/* gz_dps：本拍角速度（已扣标定零偏）；dt_s：本拍经过时间（秒） */
void  Ref_Attitude_Step(attitude_t *a, float gz_dps, float dt_s);
void  Ref_Attitude_ResetYaw(attitude_t *a);

/*==============================================================
 *  3. 循迹（四路灰度 → 差速）
 *==============================================================*/

typedef struct
{
    float    err;          /* 本拍线偏差，cm            */
    float    err_prev;     /* 上拍偏差（D 项用）        */
    float    delta;        /* 本拍差速输出，%           */
    uint8_t  online;       /* 1 = 至少一路压线          */
    uint16_t lost_cnt;     /* 连续丢线计数              */
} line_track_t;

void  Ref_Line_Init(line_track_t *lt);
/* state：4 路位掩码（bit0=X1 最左 … bit3=X4 最右，1=压线）
   gz_dps：当前角速度，用于前馈 */
void  Ref_Line_Update(line_track_t *lt, uint8_t state, float gz_dps);

/*==============================================================
 *  4. 蓝牙上报（组帧，不负责发送）
 *==============================================================*/

/* 把 4 个通道拼成 VOFA+ FireWater 帧："a,b,c,d\r\n"
   返回写入长度（含 '\r' '\n'），失败返回 0
   ⚠️ 不要用 sprintf("%f")：newlib-nano 默认不支持浮点格式化 */
int Ref_Report_BuildFrame(char *buf, int size,
                          float c0, float c1, float c2, float c3);

#endif /* __REFERENCE_H */
