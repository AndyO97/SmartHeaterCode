/**
  ******************************************************************************
  * @Name           : BlinkingLedsApplication.c
  * @Description    : Toggle active Leds
  ******************************************************************************
  */
	
	#include "BlinkingLedsApplication.h"
	
	void  BlinkingLedsApp(uint16_t ledFlags)
	{
		/*LED 1 Toggle*/
		if(ledFlags&LED1_ENABLE_FLAG)
		{
			HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		}
		else
		{
			HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_RESET);
		}
		
		/*LED 2 Toggle*/
		if(ledFlags&LED2_ENABLE_FLAG)
		{
			HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
		}
		else
		{
			HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_RESET);
		}
		
		/*LED 3 Toggle*/
		if(ledFlags&LED3_ENABLE_FLAG)
		{
			HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
		}
		else
		{
			HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,GPIO_PIN_RESET);
		}
		/*LED 4 Toggle*/
		if(ledFlags&LED4_ENABLE_FLAG)
		{
			HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);
		}
		else
		{
			HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin,GPIO_PIN_RESET);
		}
		
	}

