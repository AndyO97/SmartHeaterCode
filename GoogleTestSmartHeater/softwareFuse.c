/***************************************************************************
 * File: softwareFuse.c
 * 
 * Objective: Function to detect the level of current compared to the max threshold, increase the 
 * damage on the fuse and open the fuse if the accumulated level os reached
 * This function should be called every 10ms
 * 
 * 
 * ***********************************************************************/

/*Include to include */
#include <stdio.h>
#include <stdlib.h>
#include <conio.h>
#include <math.h>

/*Function Declaration */

int softwareFuseMain(float currentmA, int *AccumulatedLapsedTimePercent);

/*Definitions */
#define timeout 2000000     /* In ms */
#define MaxCurrent 25       /* 25mA max per pin in the Stm32 */
/* int AccumulatedLapsedTimePercent = 0; */


/*Main Function */
int softwareFuseMain(float currentmA, int *AccumulatedLapsedTimePercent)
{

    float CurrentRating = 0.0F;

    if(currentmA<0)
    {
        currentmA = currentmA * -1;
    }
    CurrentRating = (currentmA / MaxCurrent) *100;

    if(CurrentRating < 100)
    {
        //AccumulatedLapsedTimePercent = 0;
        return 0;
    }
    else
    {
        if(CurrentRating < 110)
        {
            (*AccumulatedLapsedTimePercent)+=10;
            if(*AccumulatedLapsedTimePercent>=timeout){
                return 1;
            }
        }
        else
        {
            if(CurrentRating < 135)
            {
                (*AccumulatedLapsedTimePercent)+=100;
                if(*AccumulatedLapsedTimePercent>=timeout){
                    return 1;
                }   
            }
            else{
                if(CurrentRating < 200)
                {
                    (*AccumulatedLapsedTimePercent)+=1000;
                    if(*AccumulatedLapsedTimePercent>=timeout){
                        return 1;
                    }   
                }
                else
                {
                    if(CurrentRating < 350)
                    {
                        (*AccumulatedLapsedTimePercent)+=10000;
                        if(*AccumulatedLapsedTimePercent>=timeout)
                        {
                            return 1;
                        }
                    }    
                    else
                    {
                        if(CurrentRating < 351)
                        {
                            (*AccumulatedLapsedTimePercent)+=100000;
                            if(*AccumulatedLapsedTimePercent>=timeout)
                            {
                                return 1;
                            }
                        }
                        else
                        {
                            return 1;
                        }
                           
                    }    
                        
                }    
                    
            }
        }
    }
    return 0;
}







