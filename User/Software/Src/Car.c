#include "Car.h"
#include "Motor.h"

/* 最近一次下发的左右轮 PWM（%），供串口上报观察 */
int8_t e_pwm_l = 0;
int8_t e_pwm_r = 0;

void Car_SetSpeed(int8_t l_Speed, int8_t r_Speed)
{
    e_pwm_l = l_Speed;
    e_pwm_r = r_Speed;

    Motor_Left_SetSpeed(l_Speed);
    Motor_Right_SetSpeed(r_Speed);
}

void Car_Stop()
{
    Car_SetSpeed(0, 0);
}
