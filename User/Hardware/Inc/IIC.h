#ifndef __IIC_H
#define __IIC_H

#include "main.h"

/*==============================================================
 *  软件模拟 I2C（位操作）
 *
 *  引脚（CubeMX 生成，见 Core/Inc/main.h）：
 *      IIC_SCL -> PB8   开漏输出 + 内部上拉，初值高
 *      IIC_SDA -> PB9   开漏输出 + 内部上拉，初值高
 *
 *  为什么必须"开漏 + 上拉"：
 *      I2C 是"线与"总线，任何设备只能把线拉低，不能主动拉高。
 *      开漏输出正好符合：写 0 = 拉低；写 1 = 释放，靠上拉拉回高。
 *      如果配成推挽，双方一个拉高一个拉低就会对顶短路。
 *
 *  总线上的设备（地址为 8 位写地址）：
 *      OLED (SSD1306) = 0x78
 *      MPU6050        = 0xD0（AD0 接 GND 时）
 *==============================================================*/

/* ---- 引脚电平操作 ---- */
#define IIC_SCL_H()     HAL_GPIO_WritePin(IIC_SCL_GPIO_Port, IIC_SCL_Pin, GPIO_PIN_SET)
#define IIC_SCL_L()     HAL_GPIO_WritePin(IIC_SCL_GPIO_Port, IIC_SCL_Pin, GPIO_PIN_RESET)
#define IIC_SDA_H()     HAL_GPIO_WritePin(IIC_SDA_GPIO_Port, IIC_SDA_Pin, GPIO_PIN_SET)
#define IIC_SDA_L()     HAL_GPIO_WritePin(IIC_SDA_GPIO_Port, IIC_SDA_Pin, GPIO_PIN_RESET)

/* 读 SDA：返回 GPIO_PIN_SET / GPIO_PIN_RESET */
#define IIC_SDA_READ()  HAL_GPIO_ReadPin(IIC_SDA_GPIO_Port, IIC_SDA_Pin)

/* 每个电平保持时间（us）。2us 时实测约 150kHz，OLED / MPU6050 都够用；
   杜邦线较长或上拉较弱时可加大到 5。 */
#define IIC_DELAY_US    2

/* ---- 基本时序 ---- */
void    IIC_Init(void);        /* 总线恢复空闲态                       */
void    IIC_Start(void);       /* 起始条件：SCL 高时 SDA 由高变低       */
void    IIC_Stop(void);        /* 停止条件：SCL 高时 SDA 由低变高       */
void    IIC_SendByte(uint8_t byte);
uint8_t IIC_RecvByte(void);
uint8_t IIC_WaitAck(void);     /* 返回 0 = 收到应答，1 = 无应答        */
void    IIC_SendAck(void);
void    IIC_SendNAck(void);

/* ---- 寄存器读写封装（MPU6050 等可直接用）----
 * addr 为 8 位写地址（如 0xD0），返回 0 = 成功，1 = 无应答 */
uint8_t IIC_WriteReg(uint8_t addr, uint8_t reg, uint8_t data);
uint8_t IIC_ReadRegs(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len);
uint8_t IIC_Probe(uint8_t addr);   /* 探测设备：返回 0 = 在线 */

#endif /* __IIC_H */
