#include "Beep.h"
#include "main.h"

void Beep_Init(){
    HAL_GPIO_WritePin(Beep_GPIO_Port, Beep_Pin, SET);
}

void Beep_On(){
    HAL_GPIO_WritePin(Beep_GPIO_Port, Beep_Pin, RESET);
}

void Beep_Off(){
    HAL_GPIO_WritePin(Beep_GPIO_Port, Beep_Pin, SET);
}
