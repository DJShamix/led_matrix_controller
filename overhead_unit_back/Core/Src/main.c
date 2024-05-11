/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
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


uint16_t led_pins_array[] = {
		LED_R1_Pin, LED_R2_Pin, LED_R3_Pin, LED_R4_Pin, LED_R5_Pin,
		LED_R6_Pin, LED_R7_Pin, LED_R8_Pin, LED_R9_Pin, LED_R10_Pin
};

uint32_t led_gpios_array[] = {
		LED_R1_GPIO_Port, LED_R2_GPIO_Port, LED_R3_GPIO_Port,
		LED_R4_GPIO_Port, LED_R5_GPIO_Port, LED_R6_GPIO_Port,
		LED_R7_GPIO_Port, LED_R8_GPIO_Port, LED_R9_GPIO_Port, LED_R10_GPIO_Port

};


uint16_t calculation_matrix[3][10] = {
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
};


uint16_t led_matrix[3][10] = {
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
};


uint16_t adc_data[3] = {};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

long map(long in_value, long in_min, long in_max, long out_min, long out_max);
void get_stick_position(uint16_t adc_x, uint16_t adc_y, uint8_t *pos_x, uint8_t *pos_y);
void get_led_spot_size(uint16_t adc_spot, float *led_spot_size);
void calculate_led_matrix(uint8_t pos_x, uint8_t pos_y, float spot_size);
void fill_new_matrix();

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
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim2);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);


  htim1.Instance->CCR1 = 0;
  htim1.Instance->CCR2 = 0;
  htim1.Instance->CCR3 = 0;


  HAL_ADC_Start_DMA(&hadc1, (uint32_t*) &adc_data, 3);

  HAL_Delay(200);


  for(int i = 0; i < 3; i++){
	  for(int j = 0; j < 10; j++){
		  led_matrix[i][j] = 9990;
		  HAL_Delay(10);
  		  led_matrix[i][j] = 0;
  		  HAL_Delay(10);
  	  }
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  uint8_t stick_x 	= 0;
	  uint8_t stick_y 	= 0;
	  float   spot_size = 0;

	  get_stick_position(adc_data[0], adc_data[1], &stick_x, &stick_y);

	  get_led_spot_size(adc_data[2], &spot_size);

	  calculate_led_matrix(stick_x, stick_y, spot_size);

	  HAL_Delay(2);

	  fill_new_matrix();

	  //htim1.Instance->CCR1 = 9990;
	  //htim1.Instance->CCR2 = 0;
	  //htim1.Instance->CCR3 = 9990;

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 8;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

long map(long in_value, long in_min, long in_max, long out_min, long out_max){

	return (in_value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

}
//-------------------------


float map_float(long in_value, long in_min, long in_max, float out_min, float out_max){

	return (in_value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

}
//-------------------------


void get_stick_position(uint16_t adc_x, uint16_t adc_y, uint8_t *pos_x, uint8_t *pos_y){

	*pos_x = map(adc_x, 0, 4095, 0, 10);
	//*pos_y = map(adc_y, 0, 4050, 0, 2);

	if(adc_y >= 2700) 	  				   *pos_y = 2;
	else if(adc_y < 2700 && adc_y >= 1350) *pos_y = 1;
	else 								   *pos_y = 0;
}
//-------------------------


void get_led_spot_size(uint16_t adc_spot, float *led_spot_size){

	*led_spot_size = map_float(adc_spot, 0, 4095, 0.1, 7);
}
//-------------------------


void calculate_led_matrix(uint8_t pos_x, uint8_t pos_y, float spot_size){

	#define led_diffusion 	1.8
	uint16_t led_max_bright = 9990; // (gets from TIM1 PWM value)
	uint8_t init_x = pos_x;
	uint8_t init_y = pos_y;

	uint16_t result = 0.0;
	uint8_t counter_x = 0;
	uint8_t counter_y = 0;

	// Left-to-right side (relatively to center)
	for(int i = init_x; i >= 0; i--){
		result = led_max_bright * exp(-counter_x*spot_size);
		calculation_matrix[0][i] = result;
		calculation_matrix[1][i] = result;
	    calculation_matrix[2][i] = result;
	    counter_x++;
	  }

	counter_x = 0;

	// Right-to-left side (relatively to center)
	for(uint8_t i = init_x; i <= 9; i++){
		result = led_max_bright * exp(-counter_x*spot_size);
		calculation_matrix[0][i] = result;
		calculation_matrix[1][i] = result;
		calculation_matrix[2][i] = result;
		counter_x++;
	}

	//return;

	switch(init_y){

		case 0:
		  for(int i = init_y; i <= 2; i++){
			for(int j = 0; j <= 9; j++){
				calculation_matrix[i][j] = calculation_matrix[i][j] * exp(-counter_y*spot_size);
			}
			counter_y++;
		  }
		  break;

		case 1:
		  for(int i = init_y; i >= 0; i--){
			for(int j = 0; j <= 9; j++){
				calculation_matrix[i][j] = calculation_matrix[i][j] * exp(-counter_y*spot_size);
			}
			counter_y++;
		  }

		  counter_y = 0;

		  for(int i = init_y; i <= 2; i++){
			for(int j = 0; j <= 9; j++){
				calculation_matrix[i][j] = calculation_matrix[i][j] * exp(-counter_y*spot_size);
			}
			counter_y++;
		  }
		  break;

		case 2:
		  for(int i = init_y; i >= 0; i--){
			for(int j = 0; j <= 9; j++){
				calculation_matrix[i][j] = calculation_matrix[i][j] * exp(-counter_y*spot_size);
			}
			counter_y++;
		  }
		  break;
	  }
}
//-------------------------


void fill_new_matrix(){

	for(int i = 0; i <= 2; i++){
		for(int j = 0; j <= 9; j++){
			led_matrix[i][j] = calculation_matrix[i][j];
		}
	}
}
//-------------------------


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
