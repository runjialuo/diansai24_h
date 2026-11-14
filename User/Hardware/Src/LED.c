#include "LED.h"
#include "main.h"

void LED_Init(){
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, RESET);
}

void LED_On(){
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, SET);    
}

void LED_Off(){
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, RESET);   
}

