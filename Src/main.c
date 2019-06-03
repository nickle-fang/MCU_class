/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "NRF24L01.h"
#include "key.h"
#include "1602.h"
#include "LED.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef uint8_t u8;
typedef uint32_t u32;

uint8_t percent = 5;   //占空比，范围是1-10
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
void delay_init(u8 SYSCLK);
void delay_us(u32 nus);
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
u8 fac_us;
uint8_t tx_buf[25] = {0x48, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; 
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t TX_frequency = 0x5a;	//24L01频率初始化为5a
uint8_t RX_frequency = 0x5a;	//24L01频率初始化为5a

uint8_t key_val = 0;
uint8_t flag = 0;            //指示键值是否改变
unsigned char display_1[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
unsigned char display_2[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int posi = 0;                  //指示显示位数
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
	delay_init(72);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	LCD_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	LCD_write_string(0, 0, "WELCOME!");
	//LCD_write_aword(1, 1, 'a');
	HAL_Delay(3000);
	
	LCD1602_ClearScreen();
	
	while(NRF24L01_TX_Check())
	{
		HAL_GPIO_TogglePin (LED1_GPIO_Port,LED1_Pin);
		HAL_Delay(1000);
		LCD_write_string(0, 0, "2401 ERR!");
	}
	
	LCD1602_ClearScreen();
	
	TX_Mode();
	
	
	/*初始化完成提示音*/
//	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
//	HAL_Delay(150);
//	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
//	HAL_Delay(70);
//	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
//	HAL_Delay(150);
//	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
	
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
		//flag 初始化为 0
		if(flag == 1)
		{
			switch(key_val)
			{
				case 1: key1(); flag = key_val = 0; break;
				case 2: key2(); flag = key_val = 0; break;
				case 3: key3(); flag = key_val = 0; break;
				case 4: key4(); flag = key_val = 0; break;
				case 5: key5(); flag = key_val = 0; break;
				case 6: key6(); flag = key_val = 0; break;
				case 7: key7(); flag = key_val = 0; break;
				case 8: key8(); flag = key_val = 0; break;
				case 9: key9(); flag = key_val = 0; break;
				case 10: key10(); flag = key_val = 0; break;
				case 11: key11(); flag = key_val = 0; break;
				case 12: key12(); flag = key_val = 0; break;
				case 13: key13(); flag = key_val = 0; break;
				case 14: key14(); flag = key_val = 0; break;
				case 15: key15(); flag = key_val = 0; break;
				case 16: key16(); flag = key_val = 0; break;
			}
		}
		
		else
		{
			key_scan();
		}
		
//		HAL_GPIO_TogglePin(LED_0_GPIO_Port, LED_0_Pin);
//		HAL_Delay(500);
		
//		if(NRF24L01_TxPacket(tx_buf) == TX_OK)
//		{
//			HAL_GPIO_TogglePin (LED1_GPIO_Port,LED1_Pin);
//		  HAL_Delay(10);
//		}
//		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
//		HAL_Delay(1000);
//		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
//		HAL_Delay(1000);





//		HAL_GPIO_TogglePin(LED_0_GPIO_Port, LED_0_Pin);
//		HAL_Delay(100);
//		HAL_GPIO_TogglePin(LED_0_GPIO_Port, LED_0_Pin);
//		HAL_Delay(100);
//		HAL_GPIO_TogglePin(LED_0_GPIO_Port, LED_0_Pin);
//		HAL_Delay(100);
//		
//		HAL_GPIO_TogglePin(LED_1_GPIO_Port, LED_0_Pin);
//		HAL_Delay(100);
//		HAL_GPIO_TogglePin(LED_1_GPIO_Port, LED_0_Pin);
//		HAL_Delay(100);
//		HAL_GPIO_TogglePin(LED_1_GPIO_Port, LED_0_Pin);
//		HAL_Delay(100);
		
		HAL_GPIO_TogglePin(LED_0_GPIO_Port, LED_0_Pin);
		HAL_GPIO_TogglePin(LED_1_GPIO_Port, LED_0_Pin);
		HAL_Delay(100);
		HAL_GPIO_TogglePin(LED_0_GPIO_Port, LED_0_Pin);
		HAL_GPIO_TogglePin(LED_1_GPIO_Port, LED_0_Pin);
		HAL_Delay(100);
		HAL_GPIO_TogglePin(LED_0_GPIO_Port, LED_0_Pin);
		HAL_GPIO_TogglePin(LED_1_GPIO_Port, LED_0_Pin);
		HAL_Delay(100);
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

  /** Initializes the CPU, AHB and APB busses clocks 
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
  /** Initializes the CPU, AHB and APB busses clocks 
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

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 36-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500-1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, D1_Pin|D2_Pin|D5_Pin|R1_Pin 
                          |R4_Pin|D0_Pin|LED4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CSN_Pin|CE_Pin|D7_Pin|D6_Pin 
                          |EN_Pin|LED2_Pin|RW_Pin|LED1_Pin 
                          |RS_Pin|LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_0_Pin|D4_Pin|R2_Pin|D3_Pin 
                          |R3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : D1_Pin D2_Pin D5_Pin R1_Pin 
                           R4_Pin D0_Pin LED4_Pin */
  GPIO_InitStruct.Pin = D1_Pin|D2_Pin|D5_Pin|R1_Pin 
                          |R4_Pin|D0_Pin|LED4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : IRQ_Pin */
  GPIO_InitStruct.Pin = IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CSN_Pin CE_Pin D7_Pin D6_Pin 
                           EN_Pin LED2_Pin RW_Pin LED1_Pin 
                           RS_Pin LED3_Pin */
  GPIO_InitStruct.Pin = CSN_Pin|CE_Pin|D7_Pin|D6_Pin 
                          |EN_Pin|LED2_Pin|RW_Pin|LED1_Pin 
                          |RS_Pin|LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : C4_Pin C3_Pin */
  GPIO_InitStruct.Pin = C4_Pin|C3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : C2_Pin C1_Pin */
  GPIO_InitStruct.Pin = C2_Pin|C1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_0_Pin D4_Pin R2_Pin D3_Pin 
                           R3_Pin */
  GPIO_InitStruct.Pin = LED_0_Pin|D4_Pin|R2_Pin|D3_Pin 
                          |R3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_1_Pin */
  GPIO_InitStruct.Pin = LED_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_1_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/**
 * @brief Delay on us level
 * @func  void BSP_Delay_us()
 * @param uint32_t nus
 * @return   None
 * @note  nus <= 798915us(2^24/fac_us @fac_us=21)
**/
//void BSP_Delay_us(uint32_t nus)
//{
//    uint32_t temp;
//    //Get System Clock Frequency(in MHz)
//    fac_us = HAL_RCC_GetSysClockFreq() / 1000000 / 8;
//    SysTick->LOAD = nus*fac_us;          
//    SysTick->VAL = 0x00;
//    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;      
//    do
//    {
//        temp = SysTick->CTRL;
//    }while((temp&0x01) && !(temp&(1<<16)));   
//    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
//    SysTick->VAL = 0X00;     
//}





void delay_init(u8 SYSCLK)
{
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
    //SysTick  HCLK
    fac_us=SYSCLK;              // OS,fac_us 
}


void delay_us(u32 nus)
{
    u32 ticks;
    u32 told,tnow,tcnt=0;
    u32 reload=SysTick->LOAD;                   //LOAD
    ticks=nus*fac_us;
    told=SysTick->VAL;
    while(1)
    {
        tnow=SysTick->VAL;
        if(tnow!=told)
        {
            if(tnow<told)tcnt+=told-tnow;       // SYSTICK.
            else tcnt+=reload-tnow+told;
            told=tnow;
            if(tcnt>=ticks)break;
        }
    }
}

//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//	switch(GPIO_Pin)
//	{
//		case GPIO_PIN_6: row = 2;  break;
//		case GPIO_PIN_8: row = 1;  break;
//		case GPIO_PIN_12: row = 4;  break;
//		case GPIO_PIN_14: row = 3;  break;
//	}
//}

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
