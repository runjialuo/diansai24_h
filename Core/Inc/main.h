/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Test_LED_Pin GPIO_PIN_13
#define Test_LED_GPIO_Port GPIOC
#define LED_Pin GPIO_PIN_0
#define LED_GPIO_Port GPIOA
#define TB6612_PWMA_Pin GPIO_PIN_1
#define TB6612_PWMA_GPIO_Port GPIOA
#define TB6612_PWMB_Pin GPIO_PIN_2
#define TB6612_PWMB_GPIO_Port GPIOA
#define TB6612_AIN2_Pin GPIO_PIN_4
#define TB6612_AIN2_GPIO_Port GPIOA
#define TB6612_AIN1_Pin GPIO_PIN_5
#define TB6612_AIN1_GPIO_Port GPIOA
#define TB6612_BIN1_Pin GPIO_PIN_6
#define TB6612_BIN1_GPIO_Port GPIOA
#define TB6612_BIN2_Pin GPIO_PIN_7
#define TB6612_BIN2_GPIO_Port GPIOA
#define IR_X4_Pin GPIO_PIN_0
#define IR_X4_GPIO_Port GPIOB
#define IR_X3_Pin GPIO_PIN_1
#define IR_X3_GPIO_Port GPIOB
#define IR_X2_Pin GPIO_PIN_10
#define IR_X2_GPIO_Port GPIOB
#define IR_X1_Pin GPIO_PIN_11
#define IR_X1_GPIO_Port GPIOB
#define Beep_Pin GPIO_PIN_15
#define Beep_GPIO_Port GPIOB
#define BT_RXD_Pin GPIO_PIN_9
#define BT_RXD_GPIO_Port GPIOA
#define BT_TXD_Pin GPIO_PIN_10
#define BT_TXD_GPIO_Port GPIOA
#define Key_Cnt_Pin GPIO_PIN_3
#define Key_Cnt_GPIO_Port GPIOB
#define Key_Start_Pin GPIO_PIN_4
#define Key_Start_GPIO_Port GPIOB
#define Key_Reset_Pin GPIO_PIN_5
#define Key_Reset_GPIO_Port GPIOB
#define IIC_SCL_Pin GPIO_PIN_8
#define IIC_SCL_GPIO_Port GPIOB
#define IIC_SDA_Pin GPIO_PIN_9
#define IIC_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
