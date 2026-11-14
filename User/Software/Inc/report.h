#ifndef __REPORT_H
#define __REPORT_H

#include <stdint.h>

/*==============================================================
 *  串口数据上报（VOFA+ FireWater 协议）
 *
 *  帧格式（3 通道，逗号分隔，\r\n 结尾）：
 *      pwm_l,pwm_r,yaw\r\n
 *      例：45,-45,12.3
 *
 *  ⚠️ 两个前提：
 *      1) 工程链接 newlib-nano，printf("%f") 不能用 → 这里自己转定点
 *      2) 9600 波特率下 1 字节约 1.04ms；本帧约 14 字节 ≈ 15ms
 *         发送走中断（Serial_SendArray_IT），不阻塞控制
 *==============================================================*/

/* 上报周期（ms）。9600 下不建议小于 100；改到 115200 可以设 20 */
#define REPORT_PERIOD_MS    200U

void Report_Init(void);      /* 上电调用一次     */
void Report_Task(void);      /* 主循环调用，内部限速 */

#endif /* __REPORT_H */
