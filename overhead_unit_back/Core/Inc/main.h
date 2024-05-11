/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "stm32l4xx_hal.h"

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
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define stick_y_Pin GPIO_PIN_0
#define stick_y_GPIO_Port GPIOC
#define stick_x_Pin GPIO_PIN_1
#define stick_x_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LED_R1_Pin GPIO_PIN_5
#define LED_R1_GPIO_Port GPIOA
#define LED_R7_Pin GPIO_PIN_1
#define LED_R7_GPIO_Port GPIOB
#define LED_R6_Pin GPIO_PIN_2
#define LED_R6_GPIO_Port GPIOB
#define LED_R5_Pin GPIO_PIN_11
#define LED_R5_GPIO_Port GPIOB
#define LED_R4_Pin GPIO_PIN_12
#define LED_R4_GPIO_Port GPIOB
#define LED_R10_Pin GPIO_PIN_13
#define LED_R10_GPIO_Port GPIOB
#define LED_R9_Pin GPIO_PIN_14
#define LED_R9_GPIO_Port GPIOB
#define LED_R8_Pin GPIO_PIN_15
#define LED_R8_GPIO_Port GPIOB
#define led_line_2_Pin GPIO_PIN_8
#define led_line_2_GPIO_Port GPIOA
#define led_line_3_Pin GPIO_PIN_9
#define led_line_3_GPIO_Port GPIOA
#define led_line_1_Pin GPIO_PIN_10
#define led_line_1_GPIO_Port GPIOA
#define LED_R3_Pin GPIO_PIN_11
#define LED_R3_GPIO_Port GPIOA
#define LED_R2_Pin GPIO_PIN_12
#define LED_R2_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

void fill_new_matrix();

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
