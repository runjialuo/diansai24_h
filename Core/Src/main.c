/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Bsp_Config.h"
#include "Software_Config.h"
#include "mpu6050.h"
#include "attitude.h"
#include "line_track.h"
#include "question.h"
#include "yaw_ctrl.h"
#include "Key.h"
#include "ui.h"
#include "report.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t Question_Choose = 1;                /* CNT 选中的题目：1~4          */
uint8_t cur_Question    = 0;                /* 正在执行的题目：0 = 未启动   */
uint32_t e_ui_tick;                         /* OLED 刷新节拍          */
uint32_t e_key_tick;
uint32_t e_ctrl_tick;                       /* 10ms 控制节拍          */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Control_10ms(void){
  Attitude_Update();

  /* 每 10ms 推进正在执行的题目；完成/超时 -> 停车并回到未启动 */
  switch (cur_Question){
    case 1:
      if (Ques1_Task10ms() != 0U) { cur_Question = 0U; Car_Stop(); }
      break;
    case 2:
      if (Ques2_Task10ms() != 0U) { cur_Question = 0U; Car_Stop(); }
      break;
    case 3:
      if (Ques3_Task10ms() != 0U) { cur_Question = 0U; Car_Stop(); }
      break;
    case 4:
      if (Ques4_Task10ms() != 0U) { cur_Question = 0U; Car_Stop(); }
      break;
    default: break;                        /* 0 = 未启动，什么都不做 */
  }
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */
  
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  Bsp_Init();                        /* 延时 / 软件 I2C / OLED / LED / 蜂鸣器 / 串口 */
  Sofaware_Init();                   /*姿态解算 / 小车形式 */
  Key_Init();                        /* 三个按键（记录上电瞬间状态） */
  Report_Init();                     /* 串口上报（VOFA+ FireWater） */
  //Ui_Init();                         /* OLED 调参页面（UI_ENABLE=0 时为空函数） */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_TIM_Base_Start_IT(&htim1);  /*启动10ms的全局节拍器*/
  e_ctrl_tick=HAL_GetTick();

  while (1)
  {
    Reminder_Tick();

    if(e_flag_10ms){
      e_flag_10ms=0;
      Control_10ms();
    }
    if((HAL_GetTick()-e_key_tick)>=20U){
      uint8_t ev;

      e_key_tick=HAL_GetTick();
      Key_Scan();                      /* 采样 + 边沿检测（20ms 本身就是消抖） */
      ev = Key_GetEvent();

      if(ev & KEY_CNT){                /* PB3：选题 1→2→3→4→1（只选，不启动） */
        cur_Question = 0U;
        Car_Stop();
        Question_Choose = (uint8_t)((Question_Choose % 4U) + 1U);
      }
      if(ev & KEY_START){              /* PB4：启动 / 停止 */
        if (cur_Question == 0U)
        {
          cur_Question = Question_Choose;
          switch (cur_Question){
            case 1: Ques1_Start();  break;
            case 2: Ques2_Start();  break;
            case 3: Ques3_Start();  break;
            case 4: Ques4_Start();  break;
            default: break;
          }
        }
        else
        {
          cur_Question = 0U;
          Car_Stop();
        }
      }
      if(ev & KEY_RESET){              /* PB5：复位 / 急停 */
        Question_Choose = 1U;
        cur_Question = 0U;
        Car_Stop();
        Attitude_Disable();            /* 停止陀螺解算（不再读 I2C） */
        Attitude_Reset();              /* 清空 yaw / gz / 残差 */
      }
      if(ev != 0U){
        Ui_RefreshNow();               /* 不再蜂鸣，改用屏幕立即反馈 */
      }
    }
    /*数据上报用于调整参数*/
    Report_Task();                   /* 内部按 REPORT_PERIOD_MS 限速 */
    if((HAL_GetTick()-e_ui_tick)>=100U){
      e_ui_tick=HAL_GetTick();
      // Ui_Refresh();                    /* 停车 100ms/行，跑车 500ms/行 */
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
   }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
