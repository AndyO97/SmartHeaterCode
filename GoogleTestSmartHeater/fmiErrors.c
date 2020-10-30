/***************************************************************************
 * File: fmiErros.c
 * 
 * Objective: Function to evaluate the ADC value, temperature obtained and current to detect error
 * and in case that there is an error, assign the proper value
 * 
 * ***********************************************************************/

/*Include to include */
#include <stdio.h>
#include <stdlib.h>
#include <conio.h>
#include <math.h>

/*Function Declaration */

int fmiErrorsMain(int ACDBits, float temperature, float currentmA);
float bits_to_volt2(int ADCBits);
float volt_to_C2(float VoltsMeasured);

/*Definitions */
#define Vref 5000
#define A2DResolution (float)pow(2, 8) /* 256 */
#define MaxTemperature 210
#define MinTemperature -40
#define MaxCurrent 25
#define MinCurrent 0

/*Main Function */
int fmiErrorsMain(int ADCBits, float temperature, float currentmA)
{
    int fmiNo=7;

    /*Data Valid but Above Normal Operational Range */
    if ((ADCBits > A2DResolution) && (temperature==210))
    {
        fmiNo=0;
    }
    else
    {
        /*Data Valid but Below Normal Operational Range */
        if ((ADCBits < 0) && (temperature==-40))
        {
            fmiNo=1;
        }
        else
        {
            /*Data Erratic, Intermittent or Incorrect */
            if ((temperature != volt_to_C(bits_to_volt(ADCBits))) && (temperature<=210) && (temperature>=-40))
            {
                fmiNo=2;
            }
            else
            {
                /*Voltage Above Normal, or Shorted to High Source */
                if (temperature>210)
                {
                    fmiNo=3;
                }
                else
                {
                    /*Voltage Below Normal, or Shorted to Low Source */
                    if (temperature<-40)
                    {
                        fmiNo=4;
                    }
                    else
                    {
                        /*Current Below Normal or Open Circuit */
                        if(currentmA<MinCurrent){
                            fmiNo=5;
                        }
                        else
                        {
                            /*Current Above Normal or Grounded Circuit */
                            if (currentmA>MaxCurrent)
                            {
                                fmiNo=6;
                            } 
                        }  
                    }           
                }
            }
        }   
    }
    return fmiNo;
}

/*Function to convert Bits in Voltage in mV */
float bits_to_volt2(int ADCBits)
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
float volt_to_C2(float VoltsMeasured)
{
    float CDegrees = 0.0F;
    CDegrees = ((VoltsMeasured / Vref)*250)-40;
    return CDegrees;
}







