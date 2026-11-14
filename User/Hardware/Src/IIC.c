#include "IIC.h"
#include "Delay.h"

/*==============================================================
 *  软件 I2C 时序（PB8 = SCL，PB9 = SDA）
 *  依赖 Bsp_Config.c 里的 Delay_us()（DWT 实现，不占定时器）
 *==============================================================*/

/* 总线恢复空闲：SCL、SDA 都释放为高 */
void IIC_Init(void)
{
    IIC_SDA_H();
    IIC_SCL_H();
    Delay_us(IIC_DELAY_US);
}

/* 起始条件：SCL 为高期间，SDA 由高变低 */
void IIC_Start(void)
{
    IIC_SDA_H();
    IIC_SCL_H();
    Delay_us(IIC_DELAY_US);

    IIC_SDA_L();
    Delay_us(IIC_DELAY_US);

    IIC_SCL_L();
    Delay_us(IIC_DELAY_US);
}

/* 停止条件：SCL 为高期间，SDA 由低变高 */
void IIC_Stop(void)
{
    IIC_SDA_L();
    Delay_us(IIC_DELAY_US);

    IIC_SCL_H();
    Delay_us(IIC_DELAY_US);

    IIC_SDA_H();
    Delay_us(IIC_DELAY_US);
}

/* 发送一个字节（高位先发）；每字节后由调用者处理应答 */
void IIC_SendByte(uint8_t byte)
{
    uint8_t i;

    for (i = 0; i < 8; i++)
    {
        if (byte & 0x80)
        {
            IIC_SDA_H();
        }
        else
        {
            IIC_SDA_L();
        }
        byte <<= 1;

        Delay_us(IIC_DELAY_US);

        IIC_SCL_H();          /* 上升沿，从机采样 */
        Delay_us(IIC_DELAY_US);
        IIC_SCL_L();
        Delay_us(IIC_DELAY_US);
    }
}

/* 接收一个字节（高位先收）；收完由调用者发 ACK / NACK */
uint8_t IIC_RecvByte(void)
{
    uint8_t i;
    uint8_t byte = 0;

    IIC_SDA_H();              /* 释放 SDA，交给从机驱动 */

    for (i = 0; i < 8; i++)
    {
        Delay_us(IIC_DELAY_US);

        IIC_SCL_H();
        Delay_us(IIC_DELAY_US);

        byte <<= 1;
        if (IIC_SDA_READ() == GPIO_PIN_SET)
        {
            byte |= 0x01;
        }

        IIC_SCL_L();
    }

    Delay_us(IIC_DELAY_US);
    return byte;
}

/* 等待从机应答：返回 0 = 应答（SDA 为低），1 = 无应答 */
uint8_t IIC_WaitAck(void)
{
    uint8_t ack;

    IIC_SDA_H();              /* 释放 SDA                          */
    Delay_us(IIC_DELAY_US);

    IIC_SCL_H();              /* 第 9 个时钟，从机拉低表示应答      */
    Delay_us(IIC_DELAY_US);

    ack = (IIC_SDA_READ() == GPIO_PIN_RESET) ? 0U : 1U;

    IIC_SCL_L();
    Delay_us(IIC_DELAY_US);

    return ack;
}

/* 主机应答（后面还要继续读） */
void IIC_SendAck(void)
{
    IIC_SDA_L();
    Delay_us(IIC_DELAY_US);

    IIC_SCL_H();
    Delay_us(IIC_DELAY_US);
    IIC_SCL_L();
    Delay_us(IIC_DELAY_US);
}

/* 主机非应答（最后一个字节） */
void IIC_SendNAck(void)
{
    IIC_SDA_H();
    Delay_us(IIC_DELAY_US);

    IIC_SCL_H();
    Delay_us(IIC_DELAY_US);
    IIC_SCL_L();
    Delay_us(IIC_DELAY_US);
}

/*==============================================================
 *  寄存器读写封装
 *==============================================================*/

/* START + addr + reg + data + STOP */
uint8_t IIC_WriteReg(uint8_t addr, uint8_t reg, uint8_t data)
{
    uint8_t err;

    IIC_Start();

    IIC_SendByte(addr);
    if (IIC_WaitAck()) { IIC_Stop(); return 1U; }

    IIC_SendByte(reg);
    if (IIC_WaitAck()) { IIC_Stop(); return 1U; }

    IIC_SendByte(data);
    err = IIC_WaitAck();

    IIC_Stop();
    return err;
}

/* 从 reg 开始连读 len 字节（MPU6050 一次读 14 字节用这个）
   START + addrW + reg + RESTART + addrR + data... + STOP */
uint8_t IIC_ReadRegs(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
    uint16_t i;

    if ((buf == 0) || (len == 0U))
    {
        return 1U;
    }

    IIC_Start();

    IIC_SendByte(addr);                      /* 写地址 */
    if (IIC_WaitAck()) { IIC_Stop(); return 1U; }

    IIC_SendByte(reg);                       /* 寄存器地址 */
    if (IIC_WaitAck()) { IIC_Stop(); return 1U; }

    IIC_Start();                             /* 重复起始 */
    IIC_SendByte((uint8_t)(addr | 0x01U));   /* 读地址 */
    if (IIC_WaitAck()) { IIC_Stop(); return 1U; }

    for (i = 0; i < len; i++)
    {
        buf[i] = IIC_RecvByte();
        if (i == (uint16_t)(len - 1U))
        {
            IIC_SendNAck();                  /* 最后一个字节回 NACK */
        }
        else
        {
            IIC_SendAck();
        }
    }

    IIC_Stop();
    return 0U;
}

/* 探测设备：返回 0 = 在线，1 = 不在线 */
uint8_t IIC_Probe(uint8_t addr)
{
    uint8_t ack;

    IIC_Start();
    IIC_SendByte(addr);
    ack = IIC_WaitAck();
    IIC_Stop();

    return ack;
}
