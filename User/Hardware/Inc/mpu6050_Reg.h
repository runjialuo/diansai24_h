#ifndef __MPU6050_REG_H
#define __MPU6050_REG_H

/*==============================================================
 *  MPU6050 寄存器地址表
 *  参考 STM32_SPL 教程工程（10-1 软件I2C读写MPU6050）
 *==============================================================*/

/* 采样率 / 滤波 / 量程 */
#define MPU6050_SMPLRT_DIV      0x19
#define MPU6050_CONFIG          0x1A
#define MPU6050_GYRO_CONFIG     0x1B
#define MPU6050_ACCEL_CONFIG    0x1C

/* 中断 / 电源管理 */
#define MPU6050_INT_PIN_CFG     0x37
#define MPU6050_INT_ENABLE      0x38
#define MPU6050_INT_STATUS      0x3A
#define MPU6050_PWR_MGMT_1      0x6B
#define MPU6050_PWR_MGMT_2      0x6C

/* 数据寄存器（连续排列，0x3B ~ 0x48，共 14 字节） */
#define MPU6050_ACCEL_XOUT_H    0x3B
#define MPU6050_ACCEL_XOUT_L    0x3C
#define MPU6050_ACCEL_YOUT_H    0x3D
#define MPU6050_ACCEL_YOUT_L    0x3E
#define MPU6050_ACCEL_ZOUT_H    0x3F
#define MPU6050_ACCEL_ZOUT_L    0x40
#define MPU6050_TEMP_OUT_H      0x41
#define MPU6050_TEMP_OUT_L      0x42
#define MPU6050_GYRO_XOUT_H     0x43
#define MPU6050_GYRO_XOUT_L     0x44
#define MPU6050_GYRO_YOUT_H     0x45
#define MPU6050_GYRO_YOUT_L     0x46
#define MPU6050_GYRO_ZOUT_H     0x47
#define MPU6050_GYRO_ZOUT_L     0x48

/* 身份寄存器 */
#define MPU6050_WHO_AM_I        0x75

/* ---- GYRO_CONFIG 量程位（bit4:3）---- */
#define MPU6050_GYRO_FS_250     0x00    /* ±250  °/s →  131.0 LSB/(°/s) */
#define MPU6050_GYRO_FS_500     0x08    /* ±500  °/s →   65.5 LSB/(°/s) */
#define MPU6050_GYRO_FS_1000    0x10    /* ±1000 °/s →   32.8 LSB/(°/s) */
#define MPU6050_GYRO_FS_2000    0x18    /* ±2000 °/s →   16.4 LSB/(°/s) */

/* ---- ACCEL_CONFIG 量程位（bit4:3）---- */
#define MPU6050_ACCEL_FS_2G     0x00    /* ±2  g → 16384 LSB/g */
#define MPU6050_ACCEL_FS_4G     0x08    /* ±4  g →  8192 LSB/g */
#define MPU6050_ACCEL_FS_8G     0x10    /* ±8  g →  4096 LSB/g */
#define MPU6050_ACCEL_FS_16G    0x18    /* ±16 g →  2048 LSB/g */

/* ---- CONFIG 的 DLPF_CFG（数字低通）----
 * 0x03 → 加速度 44Hz / 陀螺 42Hz，延迟约 4.9ms（控制用推荐）
 * 0x06 → 加速度  5Hz / 陀螺  5Hz，延迟约 19ms（太慢，不适合小车） */
#define MPU6050_DLPF_44HZ       0x03
#define MPU6050_DLPF_5HZ        0x06

#endif /* __MPU6050_REG_H */
