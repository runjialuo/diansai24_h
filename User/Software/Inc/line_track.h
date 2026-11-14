#ifndef __LINE_TRACK_H
#define __LINE_TRACK_H

#include <stdint.h>

/*==============================================================
 *  循迹控制（Software 层）
 *
 *  输入：四路灰度状态 + 当前 Z 轴角速度
 *  输出：差速 delta（%），交给驱动层叠加到左右轮目标上
 *
 *  算法：加权平均偏差 + PD + 角速度前馈
 *
 *      err   = Σ(w_i × 压线_i) / 压线数          ← 线相对车体中心的位置
 *      delta = KP×err + KD×(err−err_prev) + KFF×gz
 *
 *  段切换（看到线 / 丢线）用 OnLine / IsLost 判断，由上层状态机处理。
 *
 *  用法（10ms 一次）：
 *      LineTrack_Update(Line_ReadRaw(), Attitude_GetGz());
 *      delta = LineTrack_GetDelta();
 *==============================================================*/

void LineTrack_Init(void);

/* 每 10ms 调用一次
   state  = Line_ReadRaw() 的位掩码
   gz_dps = 当前 Z 轴角速度（°/s），用于前馈 */
void LineTrack_Update(uint8_t state, float gz_dps);

/* 加权偏差，单位 cm，线在车体右侧为正 */
float LineTrack_GetErr(void);

/* 差速输出，单位 %（正 = 车向右偏，需要左轮快右轮慢） */
float LineTrack_GetDelta(void);

/* 1 = 至少一路压线；0 = 全白（可能丢线） */
uint8_t LineTrack_OnLine(void);

/* 1 = 连续丢线超过 LINE_LOST_MS，需要上层处理（降速/停车） */
uint8_t LineTrack_IsLost(void);

#endif /* __LINE_TRACK_H */
