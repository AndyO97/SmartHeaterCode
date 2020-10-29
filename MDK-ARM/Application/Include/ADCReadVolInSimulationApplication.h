/**
  ******************************************************************************
  * @Name           : ADCReadVolInSimulationApplication.h
  * @Description    : Simulates the reading of an ADC input
  ******************************************************************************
  */
	
#ifndef ADC_READ_VOL_IN_SIMULATION_APPLICATION_HEADER
	#define ADC_READ_VOL_IN_SIMULATION_APPLICATION_HEADER
	
	// Include Files
	#include <stdint.h>
	#include "stm32f4xx_hal.h"
	#include <time.h>
	
	// Defines
	#define ANALOG_MAXIMUM_VALUE 							4096
	#define ANALOG_MINIMUM_VALUE 							0
	#define COUNTER_MAXIMUM_VALUE 						400
	#define COUNTER_MINIMNUM_VALUE						0
	#define SIGNAL_SLOPE 											10
	#define ZERO															0
	#define milli_seconds500											500
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
	#define ADC1_Pin 							ADC1_IN1
	
	
	//Function Declaration
	extern uint16_t  ADCReadVolInSimulationApp(void);
	
	
#endif


