#include "mpu6050.h"
#include "mpu6050_Reg.h"
#include "main.h"
#include "IIC.h"

/*==============================================================
 *  MPU6050 驱动实现
 *
 *  与参考工程（SPL 版）的差别：
 *      1. 参考工程用 MyI2C_xxx 裸时序，这里复用 IIC.c 的 IIC_WriteReg /
 *         IIC_ReadRegs，驱动层不再关心时序；
 *      2. 参考工程逐字节读 12 次（每次都要 START+STOP，≈12ms），
 *         这里 14 字节一次突发读完（≈1ms），快一个数量级；
 *      3. 参考工程 GYRO 量程 ±2000dps / ACCEL ±16g、DLPF 5Hz（太慢），
 *         这里 ACCEL 改 ±4g、DLPF 改 44Hz，适合闭环控制；
 *      4. 增加了 WHO_AM_I 校验、零偏标定、Z 轴角速度直取。
 *==============================================================*/

/* ---- 初始化参数（改这里即可换量程，注意同步改 mpu6050.h 的换算系数）---- */
#define MPU6050_SMPLRT_DIV_VAL      9                       /* 1kHz/(1+9) = 100 Hz */
#define MPU6050_DLPF_VAL            MPU6050_DLPF_44HZ       /* 44Hz 低通           */
#define MPU6050_GYRO_FS_VAL         MPU6050_GYRO_FS_2000    /* ±2000 °/s           */
#define MPU6050_ACCEL_FS_VAL        MPU6050_ACCEL_FS_4G     /* ±4 g                */

/* 上电静止标定得到 Z 轴零偏（原始 LSB） */
static int16_t m_gz_bias = 0;

/*==============================================================
 *  寄存器读写
 *==============================================================*/
uint8_t MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{
    return IIC_WriteReg(MPU6050_ADDR_W, RegAddress, Data);
}

uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
    uint8_t data = 0;

    if (IIC_ReadRegs(MPU6050_ADDR_W, RegAddress, &data, 1) != 0)
    {
        return 0;
    }
    return data;
}

uint8_t MPU6050_GetID(void)
{
    return MPU6050_ReadReg(MPU6050_WHO_AM_I);
}

/*==============================================================
 *  初始化
 *  返回 0 = 成功，1 = 芯片无应答
 *==============================================================*/
uint8_t MPU6050_Init(void)
{
    /* 1. 软复位（PWR_MGMT_1 bit7 = DEVICE_RESET），等内部寄存器恢复 */
    if (MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x80) != 0)
    {
        return 1;                       /* 写不进去 = 根本没接上 */
    }
    HAL_Delay(100);

    /* 2. 唤醒 + 时钟源选 X 轴陀螺 PLL（比内部 8MHz 稳，减少零偏漂移） */
    MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);
    MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);      /* 六轴全开 */

    /* 3. 采样率 100 Hz（与 10ms 控制周期对齐）+ 44Hz 低通 */
    MPU6050_WriteReg(MPU6050_SMPLRT_DIV, MPU6050_SMPLRT_DIV_VAL);
    MPU6050_WriteReg(MPU6050_CONFIG, MPU6050_DLPF_VAL);

    /* 4. 量程 */
    MPU6050_WriteReg(MPU6050_GYRO_CONFIG, MPU6050_GYRO_FS_VAL);
    MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, MPU6050_ACCEL_FS_VAL);

    HAL_Delay(10);

    /* 5. 校验身份：必须读到 0x68 */
    if (MPU6050_GetID() != MPU6050_WHO_AM_I_VAL)
    {
        return 1;
    }

    return 0;
}

/*==============================================================
 *  数据读取：一次突发读 14 字节
 *  0x3B ACCEL_XOUT_H
 *  0x3D ACCEL_YOUT_H
 *  0x3F ACCEL_ZOUT_H
 *  0x41 TEMP_OUT_H
 *  0x43 GYRO_XOUT_H
 *  0x45 GYRO_YOUT_H
 *  0x47 GYRO_ZOUT_H
 *==============================================================*/
uint8_t MPU6050_GetRaw(s_mpu6050_raw *out)
{
    uint8_t buf[14];

    if (out == 0)
    {
        return 1;
    }

    if (IIC_ReadRegs(MPU6050_ADDR_W, MPU6050_ACCEL_XOUT_H, buf, 14) != 0)
    {
        return 1;
    }

    out->acc[0]  = (int16_t)((buf[0]  << 8) | buf[1]);
    out->acc[1]  = (int16_t)((buf[2]  << 8) | buf[3]);
    out->acc[2]  = (int16_t)((buf[4]  << 8) | buf[5]);
    out->temp    = (int16_t)((buf[6]  << 8) | buf[7]);
    out->gyro[0] = (int16_t)((buf[8]  << 8) | buf[9]);
    out->gyro[1] = (int16_t)((buf[10] << 8) | buf[11]);
    out->gyro[2] = (int16_t)((buf[12] << 8) | buf[13]);

    return 0;
}

/*==============================================================
 *  单位换算
 *==============================================================*/
float MPU6050_GyroToDps(int16_t raw)
{
    return (float)raw / MPU6050_GYRO_LSB_PER_DPS;
}

/* Z 轴角速度（已扣零偏），航向积分用这个 */
float MPU6050_GetGyroZDps(void)
{
    s_mpu6050_raw raw;

    if (MPU6050_GetRaw(&raw) != 0)
    {
        return 0.0f;
    }
    return (float)(raw.gyro[2] - m_gz_bias) / MPU6050_GYRO_LSB_PER_DPS;
}

/*==============================================================
 *  上电零偏标定
 *
 *  为什么需要预热：MPU6050 上电后芯片自加热，零偏在前 1~2 分钟持续变化。
 *  复位后立刻标定，标到的是"冷态零偏"，几十秒后就不准（残留 0.3~1 °/s）。
 *
 *  但预热太久（2 秒）会让启动很慢，所以这里折中：
 *      - 预热 300 ms（够芯片内部振荡器稳定，不够等温度完全稳定）
 *      - 采样 300 次、不加额外延时（读一次 14 字节约 1ms）→ 约 0.3 秒
 *      - 剩下的温度漂移交给 attitude.c 的"运行时残差跟踪"补
 *==============================================================*/
#define MPU6050_CALIB_WARMUP_MS   300U    /* 预热时间          */
#define MPU6050_CALIB_DISCARD     20U     /* 丢弃的过渡样本数  */
#define MPU6050_CALIB_SAMPLES     300U    /* 默认采样次数      */
#define MPU6050_CALIB_INTERVAL_MS 0U      /* 每次采样后的间隔  */

void MPU6050_CalibGyro(uint16_t samples)
{
    s_mpu6050_raw raw;
    int32_t  sum   = 0;
    uint16_t count = 0;
    uint16_t i;

    if (samples == 0)
    {
        samples = MPU6050_CALIB_SAMPLES;
    }

    /* 1. 预热 */
    HAL_Delay(MPU6050_CALIB_WARMUP_MS);

    /* 2. 丢弃过渡样本 */
    for (i = 0; i < MPU6050_CALIB_DISCARD; i++)
    {
        (void)MPU6050_GetRaw(&raw);
        if (MPU6050_CALIB_INTERVAL_MS > 0U)
        {
            HAL_Delay(MPU6050_CALIB_INTERVAL_MS);
        }
    }

    /* 3. 正式采样求平均 */
    for (i = 0; i < samples; i++)
    {
        if (MPU6050_GetRaw(&raw) == 0)
        {
            sum += raw.gyro[2];
            count++;
        }
        if (MPU6050_CALIB_INTERVAL_MS > 0U)
        {
            HAL_Delay(MPU6050_CALIB_INTERVAL_MS);
        }
    }

    if (count > 0)
    {
        m_gz_bias = (int16_t)(sum / (int32_t)count);
    }
}

int16_t MPU6050_GetGyroBias(void)
{
    return m_gz_bias;
}
