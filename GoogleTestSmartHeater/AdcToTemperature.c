/***************************************************************************
 * File: AdcToTemperature.c
 * 
 * Objective: Function to convert an ADC value into a temperature value in °C
 * 
 * ***********************************************************************/

/*Include to include */
#include <stdio.h>
#include <stdlib.h>
#include <conio.h>
#include <math.h>

/*Function Declaration */

float fullConversionMain(int ACDBits);
float bits_to_volt(int ADCBits);
float volt_to_C(float VoltsMeasured);

/*Definitions */
#define Vref 5000
#define A2DResolution (float)pow(2, 8) /* 256 */
#define SensorVariation 0.296F

/*Main Function */
float fullConversionMain(int ACDBits)
{
    float Vsensor = 0.0F;
    float CelDegrees = 0.0F;

    Vsensor = bits_to_volt(ACDBits);
    CelDegrees = volt_to_C(Vsensor);

    return CelDegrees;
}

/*Function to convert Bits in Voltage in mV */
float bits_to_volt(int ADCBits)
{
    float Vsensor = 0.0F;
    if(ADCBits >= A2DResolution)
    {
        ADCBits = A2DResolution;
    }
    else
    {   if(ADCBits <= 0)
        {
            ADCBits = 0;
        }
    }
    Vsensor = (ADCBits * Vref) / A2DResolution;
    return Vsensor;
}

/*Function to convert from mV to °C */
float volt_to_C(float VoltsMeasured)
{
    float CDegrees = 0.0F;
    CDegrees = ((VoltsMeasured / Vref)*250)-40;
    return CDegrees;
}




/*Code not used *****************************************************/




