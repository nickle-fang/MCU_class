/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void BSP_Delay_us(uint32_t nus);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define D1_Pin GPIO_PIN_4
#define D1_GPIO_Port GPIOC
#define D2_Pin GPIO_PIN_5
#define D2_GPIO_Port GPIOC
#define IRQ_Pin GPIO_PIN_1
#define IRQ_GPIO_Port GPIOB
#define CSN_Pin GPIO_PIN_10
#define CSN_GPIO_Port GPIOB
#define CE_Pin GPIO_PIN_11
#define CE_GPIO_Port GPIOB
#define C4_Pin GPIO_PIN_12
#define C4_GPIO_Port GPIOB
#define D7_Pin GPIO_PIN_13
#define D7_GPIO_Port GPIOB
#define C3_Pin GPIO_PIN_14
#define C3_GPIO_Port GPIOB
#define D6_Pin GPIO_PIN_15
#define D6_GPIO_Port GPIOB
#define C2_Pin GPIO_PIN_6
#define C2_GPIO_Port GPIOC
#define D5_Pin GPIO_PIN_7
#define D5_GPIO_Port GPIOC
#define C1_Pin GPIO_PIN_8
#define C1_GPIO_Port GPIOC
#define R1_Pin GPIO_PIN_9
#define R1_GPIO_Port GPIOC
#define LED_0_Pin GPIO_PIN_8
#define LED_0_GPIO_Port GPIOA
#define D4_Pin GPIO_PIN_9
#define D4_GPIO_Port GPIOA
#define R2_Pin GPIO_PIN_10
#define R2_GPIO_Port GPIOA
#define D3_Pin GPIO_PIN_11
#define D3_GPIO_Port GPIOA
#define R3_Pin GPIO_PIN_12
#define R3_GPIO_Port GPIOA
#define R4_Pin GPIO_PIN_10
#define R4_GPIO_Port GPIOC
#define D0_Pin GPIO_PIN_11
#define D0_GPIO_Port GPIOC
#define LED4_Pin GPIO_PIN_12
#define LED4_GPIO_Port GPIOC
#define LED_1_Pin GPIO_PIN_2
#define LED_1_GPIO_Port GPIOD
#define EN_Pin GPIO_PIN_4
#define EN_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_5
#define LED2_GPIO_Port GPIOB
#define RW_Pin GPIO_PIN_6
#define RW_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_7
#define LED1_GPIO_Port GPIOB
#define RS_Pin GPIO_PIN_8
#define RS_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_9
#define LED3_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define uint8_t unsigned char
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
