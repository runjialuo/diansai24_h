#include "Bsp_Config.h"
#include "Motor.h"
#include "Serial.h"
#include "Delay.h"
#include "LED.h"
#include "IIC.h"
#include "OLED.h"
#include "Beep.h"
#include "line_sensor.h"

void Bsp_Init(){
    Delay_Init();
    IIC_Init();
    OLED_Init();
    LED_Init();
    Beep_Init();
    Serial_Init();
    Line_Init();
    Motor_Init();
    /* 注意：MPU6050 不在这里初始化。
       Attitude_Init() 会先复位再预热标定，在这里再 init 一次等于多复位一遍 */
}

