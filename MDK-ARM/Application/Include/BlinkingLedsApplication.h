/**
  ******************************************************************************
  * @Name           : BlinkingLedsApplication.h
  * @Description    : BlinkingLedsApplication header file
  ******************************************************************************
  */
	
#ifndef BLINKING_LEDS_APPLICATION_HEADER
#define BLINKING_LEDS_APPLICATION_HEADER

	// Include Files
	#include <stdint.h>
	#include "stm32f4xx_hal.h"
	
	// Defines
	#define LED1_ENABLE_FLAG			1
	#define LED2_ENABLE_FLAG			2
	#define LED3_ENABLE_FLAG			4
	#define LED4_ENABLE_FLAG			8
	#define LED1_Pin 							GPIO_PIN_13
	#define LED1_GPIO_Port 				GPIOD
	#define LED2_Pin 							GPIO_PIN_12
	#define LED2_GPIO_Port 				GPIOD
	#define LED3_Pin 							GPIO_PIN_15
	#define LED3_GPIO_Port 				GPIOD
	#define LED4_Pin 							GPIO_PIN_14
	#define LED4_GPIO_Port 				GPIOD

	
	//Function Declarations
	extern void  BlinkingLedsApp(uint16_t ledFlags);
	
#endif

