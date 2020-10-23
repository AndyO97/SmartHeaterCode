/**
  ******************************************************************************
  * @Name           : SetActiveLedsApplication.h
  * @Description    : SetActiveLedsApplication header file
  ******************************************************************************
  */
	
#ifndef SET_ACTIVE_LEDS_APPLICATION_HEADER
	#define SET_ACTIVE_LEDS_APPLICATION_HEADER

	// Include Files
	#include <stdint.h>
	
	// Defines
	#define LED1_ENABLE_MASK 					1
	#define LED2_ENABLE_MASK 					2
	#define LED3_ENABLE_MASK 					4
	#define LED4_ENABLE_MASK 					8
	#define LED1_MAXIMUM_ENABLE_VALUE 1024
	#define LED2_MAXIMUM_ENABLE_VALUE	2048
	#define LED3_MAXIMUM_ENABLE_VALUE 3072
	#define LED4_MAXIMUM_ENABLE_VALUE 4096
	#define ZERO 											0
	
	//Function Declarations
	extern uint16_t SetActiveLedsApp(uint16_t inputValue);
	
#endif

