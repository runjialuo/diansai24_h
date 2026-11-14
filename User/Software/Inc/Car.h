#ifndef CAR_H
#define CAR_H

#include "Motor.h"
#include "main.h"

/* 最近一次下发的左右轮 PWM（%），只在 Car_SetSpeed 里更新 */
extern int8_t e_pwm_l;
extern int8_t e_pwm_r;

void Car_SetSpeed(int8_t l_Speed, int8_t r_Speed);

void Car_Stop(void);

#endif
