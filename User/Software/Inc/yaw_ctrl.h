#ifndef __YAW_CTRL_H
#define __YAW_CTRL_H

#include "yaw_ctrl.h"
#include <stdint.h>

void YawCtrl_Init(void);
void YawCtrl_Reset(void);
void YawCtrl_SetTargetDeg(float deg);   /*设置目标航向（偏移角度）*/
float YawCtrl_Updata10ms(void);         /*返回差速delta*/
float YawCtrl_GetTargetDeg(void);       /**/
float YawCtrl_GetDelta(void);           /* 上一次输出的 delta（调试显示用） */

#endif