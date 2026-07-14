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
#include "stm32h7xx_hal.h"

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
#define LeftA_Pin GPIO_PIN_0
#define LeftA_GPIO_Port GPIOA
#define LeftB_Pin GPIO_PIN_1
#define LeftB_GPIO_Port GPIOA
#define status_ble_Pin GPIO_PIN_2
#define status_ble_GPIO_Port GPIOA
#define LCD_BLK_Pin GPIO_PIN_5
#define LCD_BLK_GPIO_Port GPIOA
#define LCD_RST_Pin GPIO_PIN_14
#define LCD_RST_GPIO_Port GPIOE
#define LCD_DC_Pin GPIO_PIN_15
#define LCD_DC_GPIO_Port GPIOE
#define LCD_CS_Pin GPIO_PIN_11
#define LCD_CS_GPIO_Port GPIOB
#define RightB_Pin GPIO_PIN_12
#define RightB_GPIO_Port GPIOD
#define RightA_Pin GPIO_PIN_13
#define RightA_GPIO_Port GPIOD
#define Rmotor_Pin GPIO_PIN_6
#define Rmotor_GPIO_Port GPIOC
#define Lmotor_Pin GPIO_PIN_7
#define Lmotor_GPIO_Port GPIOC
#define RIN2_Pin GPIO_PIN_4
#define RIN2_GPIO_Port GPIOD
#define RIN1_Pin GPIO_PIN_3
#define RIN1_GPIO_Port GPIOD
#define SDMMC1_CD_Pin GPIO_PIN_15
#define SDMMC1_CD_GPIO_Port GPIOA
#define LIN1_Pin GPIO_PIN_10
#define LIN1_GPIO_Port GPIOA
#define LIN2_Pin GPIO_PIN_9
#define LIN2_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
