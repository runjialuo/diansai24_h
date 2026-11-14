#include  "Software_Config.h"
#include "Beep.h"
#include "LED.h"
#include "line_track.h"
#include "attitude.h"
#include "stm32f1xx_hal.h"
#include "question.h"
#include "stm32f1xx_hal_tim.h"
#include "yaw_ctrl.h"

static uint32_t m_reminder_ms=0;
volatile uint8_t e_flag_10ms=0;

void Sofaware_Init(void){
    LineTrack_Init();
    Attitude_Init();
    YawCtrl_Init();
}

void Reminder_Start(uint16_t ms){
    m_reminder_ms=HAL_GetTick()+ms;
    Beep_On();
    LED_On();
}

void Reminder_Tick(void){
    if(m_reminder_ms==0) return;
    if((int32_t)(HAL_GetTick()-m_reminder_ms)>=0){
        Beep_Off();
        LED_Off();
        m_reminder_ms=0;
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
    if(htim->Instance==TIM1){
        e_flag_10ms=1;
    }
}
