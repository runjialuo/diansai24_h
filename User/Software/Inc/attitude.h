#ifndef __ATTITUDE_H
#define __ATTITUDE_H

#include <stdint.h>

/*==============================================================
 *  姿态解算（Software 层）
 *
 *  MPU6050 只有陀螺 + 加速度，没有磁力计，所以只能给「相对航向」：
 *      上电时为 0，靠陀螺 Z 轴角速度积分得到 yaw
 *
 *  用法（主循环）：
 *      Attitude_Init();            // 上电一次：初始化 IMU + 零偏标定 + yaw 清零
 *      Attitude_Update();          // 每次主循环调用，内部按实际经过时间积分
 *
 *  积分用「实际 dt」（HAL_GetTick 差值）而不是固定 10ms，
 *  这样即使主循环被串口发送拖慢，航向也不会算错。
 *==============================================================*/

/* 失败标志：IMU 没接上时为 0，上层应据此禁用航向环 */
uint8_t Attitude_Init(void);

/* 主循环调用（建议 ≤10ms 一次），内部自行判断是否需要更新 */
void Attitude_Update(void);

/* 航向，单位：度，范围 -180 ~ +180 */
float Attitude_GetYaw(void);

/* 当前 Z 轴角速度，单位：°/s（已扣零偏），顺时针为正 */
float Attitude_GetGz(void);

/* 把当前航向清零（进入新直线段时用，抵消累积漂移） */
void Attitude_ResetYaw(void);

/* 清空全部姿态数据：yaw / gz / 残差补偿（RESET 键用） */
void Attitude_Reset(void);

/* 开始 / 停止解算：停止后 Update 直接返回，不再读 I2C、不再积分 */
void Attitude_Enable(void);
void Attitude_Disable(void);

/* 当前是否在解算：1 = 开，0 = 停 */
uint8_t Attitude_IsEnabled(void);

/* IMU 是否可用：1 = 正常，0 = 初始化失败 */
uint8_t Attitude_IsOk(void);

#endif /* __ATTITUDE_H */
