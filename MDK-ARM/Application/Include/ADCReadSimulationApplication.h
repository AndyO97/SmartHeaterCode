/**
  ******************************************************************************
  * @Name           : ADCReadSimulationApplication.h
  * @Description    : Simulates the reading of an ADC input
  ******************************************************************************
  */
	
#ifndef ADC_READ_SIMULATION_APPLICATION_HEADER
	#define ADC_READ_SIMULATION_APPLICATION_HEADER
	
	// Include Files
	#include <stdint.h>
	
	// Defines
	#define ANALOG_MAXIMUM_VALUE 							4096
	#define ANALOG_MINIMUM_VALUE 							0
	#define COUNTER_MAXIMUM_VALUE 						400
	#define COUNTER_MINIMNUM_VALUE						0
	#define SIGNAL_SLOPE 											10
	#define ZERO															0
	
	//Function Declaration
	extern uint16_t  ADCReadSimulationApp(void);
	
	
#endif


