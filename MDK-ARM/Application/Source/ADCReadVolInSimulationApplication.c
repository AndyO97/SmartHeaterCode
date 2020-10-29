/**
  ******************************************************************************
  * @Name           : ADCReadSimulationApplication.c
  * @Description    : Simulates the reading of an ADC input
  ******************************************************************************
  */
	
	#include "ADCReadVolInSimulationApplication.h"
	

	
	uint16_t ADCReadVoInSimulationApp()
{
	static uint16_t analogValue=ZERO;
	
	//HAL_ADC_Start(&g_AdcHandle);
	//HAL_ADC_PollForConversion(&g_AdcHandle,1);
	//analogValue = HAL_ADC_GetValue(&g_AdcHandle);
	
	/*Turn on LD1[Debug1]*/
	HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	/*0.5 seconds delay*/
	clock_t start_time = clock(); 
	while (clock() < start_time + milli_seconds500) 
        ; 
	/*Turn off LD1[Debug1]*/
	HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_RESET);
	return analogValue;
}

