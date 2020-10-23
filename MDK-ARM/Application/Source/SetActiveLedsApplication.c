/**
  ******************************************************************************
  * @Name           : SetActiveLedsApplication.c
  * @Description    : Determines which led will be active
  ******************************************************************************
  */
	
	#include "SetActiveLedsApplication.h"

	
uint16_t SetActiveLedsApp(uint16_t inputValue)
	{
		uint16_t ledMaskValue;
		if(inputValue<LED1_MAXIMUM_ENABLE_VALUE)
			{
				ledMaskValue=LED1_ENABLE_MASK;
			}
		else if(inputValue<LED2_MAXIMUM_ENABLE_VALUE)
			{
				ledMaskValue=LED2_ENABLE_MASK;
			}
		else if(inputValue<LED3_MAXIMUM_ENABLE_VALUE)
			{
				ledMaskValue=LED3_ENABLE_MASK;
			}
		else if(inputValue<LED4_MAXIMUM_ENABLE_VALUE)
			{
				ledMaskValue=LED4_ENABLE_MASK;
			}
		else
			{
				ledMaskValue=ZERO;
			}
			return ledMaskValue;
	}


