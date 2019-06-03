#include "main.h"
#include "LED.h"


void turn_up()
{
	HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	HAL_Delay(1000);
	HAL_GPIO_TogglePin(LED_0_GPIO_Port, LED_0_Pin);
	HAL_Delay(1000);
	HAL_GPIO_TogglePin(LED_0_GPIO_Port, LED_0_Pin);
	HAL_Delay(1000);
	HAL_GPIO_TogglePin(LED_0_GPIO_Port, LED_0_Pin);
	HAL_Delay(1000);
	HAL_GPIO_TogglePin(LED_0_GPIO_Port, LED_0_Pin);
	HAL_Delay(1000);
}

void led_send()
{
	
}

void led_receive()
{
	
}