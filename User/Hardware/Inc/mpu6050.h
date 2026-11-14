#ifndef __MPU_6050_H
#define __MPU_6050_H

#include <stdint.h>

/*==============================================================
 *  MPU6050 六轴传感器驱动（软件 I2C 版）
 *
 *  接线（AD0 接 GND）：
 *      VCC  3.3V     GND  GND
 *      SCL  PB8      SDA  PB9        INT 悬空
 *      8 位写地址 0xD0，7 位 0x68
 *
 *  与 IIC.c 的关系：
 *      总线时序在 IIC.c，本文件只负责"读哪个寄存器、怎么换算"。
 *      IIC_ReadRegs() 支持连续读，所以 14 字节一次突发读完（≈1ms），
 *      比逐字节读 12 次快一个数量级。
 *
 *  坐标约定（模块平放、芯片朝上、X 轴朝车头）：
 *      gyro[2] = 绕竖直轴（Z）的角速度 → 航向积分用这个
 *================================================================*/

#define MPU6050_ADDR_W          0xD0    /* 写地址（AD0 = 0） */
#define MPU6050_ADDR_R          0xD1    /* 读地址            */
#define MPU6050_WHO_AM_I_VAL    0x68    /* WHO_AM_I 期望值   */

/* 量程换算系数（必须与 mpu6050.c 里的初始化配置一致） */
#define MPU6050_GYRO_LSB_PER_DPS    16.4f    /* ±2000 °/s */
#define MPU6050_ACC_LSB_PER_G       8192.0f  /* ±4 g      */

/*--------------------------------------------------------------
 *  一帧原始数据（14 字节一次突发读出）
 *
 *  acc  加速度计三轴（未换算，LSB）  量程 ±4 g   → 8192   LSB/g
 *  temp 芯片温度        （未换算，LSB）  换算：temp/340 + 36.53 ℃
 *  gyro 陀螺仪三轴     （未换算，LSB）  量程 ±2000 °/s → 16.4 LSB/(°/s)
 *
 *  ⚠️ acc 和 gyro 是**两个不同的传感器**，不是重复数据：
 *     acc 测"加速度"（含重力），gyro 测"角速度"。
 *     本工程只用 gyro[2]（绕竖直轴），acc 留给以后做倾角检测。
 *------------------------------------------------------------*/
typedef struct
{
    int16_t acc[3];      /* 加速度 X/Y/Z，LSB，±4 g      */
    int16_t temp;        /* 温度，LSB                    */
    int16_t gyro[3];     /* 角速度 X/Y/Z，LSB，±2000 °/s */
} s_mpu6050_raw;

/*--------------------------------------------------------------
 *  接口
 *------------------------------------------------------------*/

/* 初始化：唤醒 + 设采样率/滤波/量程，并读 WHO_AM_I 校验
   返回 0 = 成功，1 = 芯片无应答（检查接线 / AD0） */
uint8_t MPU6050_Init(void);

/* 读 WHO_AM_I，正常返回 0x68 */
uint8_t MPU6050_GetID(void);

/* 单寄存器读 / 写（调试用，正常流程不直接用） */
uint8_t MPU6050_ReadReg(uint8_t RegAddress);
uint8_t MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data);

/* 一次突发读 14 字节（加速度 6 + 温度 2 + 陀螺 6）
   返回 0 = 成功，1 = I2C 无应答 */
uint8_t MPU6050_GetRaw(s_mpu6050_raw *out);

/* 原始角速度 → °/s（未扣零偏） */
float MPU6050_GyroToDps(int16_t raw);

/* 取 Z 轴角速度（已扣零偏，单位 °/s），航向积分直接用它 */
float MPU6050_GetGyroZDps(void);

/* 上电静止标定：采集 samples 次求平均作为零偏
   ⚠️ 调用期间小车必须完全静止，否则零偏被污染 */
void    MPU6050_CalibGyro(uint16_t samples);

/* 读回当前零偏（原始 LSB），调试用 */
int16_t MPU6050_GetGyroBias(void);

#endif /* __MPU_6050_H */
