/***************************************************************************
 * File: canMessages.c
 * 
 * Objective: Function to convert an ADC value and temperature value into the message that will be send through CAN
 * 
 * ***********************************************************************/

/*Include to include */
#include <stdio.h>
#include <stdlib.h>
#include <conio.h>
#include <math.h>

/*Function Declaration */

float canConversionMain(int ACDBits);
float bits_to_volt(int ADCBits);
float volt_to_F(float VoltsMeasured);
float F_to_C(float FDegrees);
float volt_to_C(float VoltsMeasured);

/*Definitions */
#define Vref 5000
#define A2DResolution (float)pow(2, 8) /* 256 */
#define SensorVariation 0.296F

/*Main Function */
float canConversionMain(int ACDBits, int temperature)
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
        Vsensor = Vref;
    }
    else
    {   if(ADCBits <= 0)
        {
            Vsensor = 0;
        }
        else
        {
            Vsensor = (ADCBits * Vref) / A2DResolution;
        }
    }
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

/*Function to convert from mV to °F */
float volt_to_F(float VoltsMeasured)
{
    float FDegrees = 0.0F;
    FDegrees = VoltsMeasured / SensorVariation;
    return FDegrees;
}

/*Function to convert from °F to °C */
float F_to_C(float FDegrees)
{
    float CDegrees = 0.0F;
    CDegrees = (5/9) * (FDegrees -32);
    return CDegrees;
}



