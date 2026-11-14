#include "Key.h"
#include "main.h"
#include "stm32f1xx_hal_gpio.h"

static uint8_t m_last_state=0;
static uint8_t m_event=0;

static uint8_t Key_Read(void){
    uint8_t s=0;
    if(HAL_GPIO_ReadPin(Key_Cnt_GPIO_Port, Key_Cnt_Pin)==GPIO_PIN_RESET) s|=KEY_CNT;
    if(HAL_GPIO_ReadPin(Key_Start_GPIO_Port, Key_Start_Pin)==GPIO_PIN_RESET) s|=KEY_START;
    if(HAL_GPIO_ReadPin(Key_Reset_GPIO_Port, Key_Reset_Pin)==GPIO_PIN_RESET) s|=KEY_RESET;

    return s;
}

void Key_Init(){
    m_last_state=Key_Read();
    m_event=0;
}

void Key_Scan(){
    uint8_t now=Key_Read();
    m_event|=(uint8_t)(now&(uint8_t)~m_last_state);/*检测上升沿*/
    m_last_state=now;
}

uint8_t Key_GetEvent(void){
    uint8_t e=m_event;
    m_event=0;
    return e;   
}