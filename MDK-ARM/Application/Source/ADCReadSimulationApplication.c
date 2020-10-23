/**
  ******************************************************************************
  * @Name           : ADCReadSimulationApplication.c
  * @Description    : Simulates the reading of an ADC input
  ******************************************************************************
  */
	
	#include "ADCReadSimulationApplication.h"

	
	uint16_t ADCReadSimulationApp()
{
	static uint16_t analogValue=ZERO;
	static uint16_t stepCounter=ZERO;
	stepCounter++;
	if(stepCounter>=COUNTER_MAXIMUM_VALUE)
	{
		stepCounter=COUNTER_MINIMNUM_VALUE;
		analogValue=ANALOG_MINIMUM_VALUE;
	}
	analogValue=stepCounter*SIGNAL_SLOPE;
	return analogValue;
}

