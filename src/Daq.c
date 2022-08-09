/**
 * @file Daq.c
 * @author Nick Berezny
 * @date 13 Apr 2022
 * @brief Functions for interfacing with Labjack Daq https://labjack.com/
 *
 */


#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>
#include <sched.h>
#include <stdbool.h>
#include <limits.h>
#include <LabJackM.h>

#include "./include/Parameters.h"
#include "./include/Structures.h"
#include "./include/Daq.h"


void ReadWriteDAQ(struct States * s, struct DAQ * daq)
{
    /**
     * @brief simultaneously read and write to Daq
     * @param[in] *s : pointer to robot States to store results
     */
    
    LJM_eNames(daq->daqHandle, DAQ_NUM_OF_CH, daq->aNames, daq->aWrites, daq->aNumValues, daq->aValues, &(daq->errorAddress));
    s->dx = (1.0 - 2.0*(double)daq->aValues[4])*((double)daq->aValues[5])*ENC_TO_M; //in m/dt
    s->Fext = 0.001*(FT_GAIN_g*daq->aValues[1] + FT_OFFSET_g); //in kg

}

int initDaq(struct DAQ *daq)
{

    /**
     * @brief initialize Daq 
     * @param[in] *s : pointer to robot States to store results
     */

    
    int err, handle; 
    handle = 0;
    // Open first found LabJack
    err = LJM_Open(LJM_dtANY, LJM_ctANY, "LJM_idANY", &handle);
    //ErrorCheck(err, "LJM_Open");

    LJM_eStreamStop(handle); //stop any previous streams
    //LJM_eWriteName(handle, "DAC0", MOTOR_ZERO); //set motor to zero
    
    
    //start Quadrature counter on DIO2 and DIO3
    /*
    printf("LJM error: %d\n",LJM_eWriteName(handle, "DIO2_EF_ENABLE", 0));
    printf("LJM error: %d\n",LJM_eWriteName(handle, "DIO3_EF_ENABLE", 0));

    printf("LJM error: %d\n",LJM_eWriteName(handle, "DIO2_EF_INDEX", 10));
    printf("LJM error: %d\n",LJM_eWriteName(handle, "DIO3_EF_INDEX", 10));

    printf("LJM error: %d\n",LJM_eWriteName(handle, "DIO2_EF_ENABLE", 1));
    printf("LJM error: %d\n",LJM_eWriteName(handle, "DIO3_EF_ENABLE", 1));
*/

    printf("LJM error: %d\n",LJM_eWriteName(handle, "DIO3_EF_ENABLE", 0));
 
    printf("LJM error: %d\n",LJM_eWriteName(handle, "DIO3_EF_INDEX", 8));

    printf("LJM error: %d\n",LJM_eWriteName(handle, "DIO3_EF_ENABLE", 1));
   

    //set Analog in resolution
    LJM_eWriteName(handle, "AIN0_RESOLUTION_INDEX", 1);
    LJM_eWriteName(handle, "AIN0_SETTLING_US", 0);
    LJM_eWriteName(handle, "AIN_ALL_NEGATIVE_CH", 199);

    printf("DAQ Handle: %d\n", handle);


    double aValues[6] = {0};
    const char * aNames[6] = {"DAC0", "AIN0","FIO0", "FIO1","FIO2","DIO3_EF_READ_A_AND_RESET"}; //
    int aNumValues[6] = {1,1,1,1,1,1};
    int aWrites[6] = {1,0,0,0,0,0};


    for(int i = 0; i < BUFFER_SIZE; i++)
    {
        memcpy(&(daq->aNames),aNames, 600*sizeof(char));
        memcpy(&(daq->aValues),aValues, 6*sizeof(double));
        memcpy(&(daq->aNumValues),aNumValues, 6*sizeof(int));
        memcpy(&(daq->aWrites),aWrites, 6*sizeof(int));
        memcpy(&(daq->daqHandle),&handle,sizeof(int));
    }
    

    /*
    s->daq.aValues = aValues;
    s->daq.aNumValues = aNumValues;
    s->daq.aWrites = aWrites;
    s->daq.errorAddress = 0;
    */
    return handle;

}

bool closeAllDaqs()
{
     /**
     * @brief closes all connected Daqs

     */


	LJM_ERROR_RETURN err;
	err = LJM_CloseAll();

	if(err == 0) return true;
	else return false;
}