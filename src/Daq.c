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
    s->dx = (1.0 - 2.0*(double)daq->aValues[4])*((double)daq->aValues[5])*ENC_TO_M/(STEP_SIZE_MS/1000.0); //in m/dt
    s->Fext = 0.001*(FT_GAIN_g*daq->aValues[1] + FT_OFFSET_g)*9.81; //in N
    s->lsb = daq->aValues[2];
    s->lsf = daq->aValues[3];
}

int initDaq(struct DAQ *daq)
{

    /**
     * @brief initialize Daq 
     * @param[in] *s : pointer to robot States to store results
     */

    
    int err, handle, errAdress;
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

    LJM_eWriteName(handle, "AIN1_RESOLUTION_INDEX", 1);
    LJM_eWriteName(handle, "AIN2_RESOLUTION_INDEX", 1);
    LJM_eWriteName(handle, "AIN3_RESOLUTION_INDEX", 1);
    LJM_eWriteName(handle, "AIN4_RESOLUTION_INDEX", 1);



    double aValues[DAQ_NUM_OF_CH] = {0};
    const char * aNames[DAQ_NUM_OF_CH];
    int aNumValues[DAQ_NUM_OF_CH];
    int aWrites[DAQ_NUM_OF_CH];

    if(DAQ_NUM_OF_CH==6)
    {
        const char * aNames[DAQ_NUM_OF_CH] = {"DAC0", "AIN0","FIO0", "FIO1","FIO2","DIO3_EF_READ_A_AND_RESET"}; //
        //const char * aNames[6] = {"DAC0", "AIN0","AIN1", "FIO1","FIO2","DIO3_EF_READ_A_AND_RESET"}; //
        int aNumValues[DAQ_NUM_OF_CH] = {1,1,1,1,1,1};
        int aWrites[DAQ_NUM_OF_CH] = {1,0,0,0,0,0};
    }
    else if(DAQ_NUM_OF_CH==10)
    {
        const char * aNames[DAQ_NUM_OF_CH] = {"DAC0", "AIN0","FIO0", "FIO1","FIO2","DIO3_EF_READ_A_AND_RESET","AIN1","AIN2","AIN3","AIN4"}; //
        int aNumValues[DAQ_NUM_OF_CH] = {1,1,1,1,1,1,1,1,1,1};
        int aWrites[DAQ_NUM_OF_CH] = {1,0,0,0,0,0,0,0,0,0};
    }



    for(int i = 0; i < BUFFER_SIZE; i++)
    {
        memcpy(&(daq->aNames),aNames, 100*DAQ_NUM_OF_CH*sizeof(char));
        memcpy(&(daq->aValues),aValues, DAQ_NUM_OF_CH*sizeof(double));
        memcpy(&(daq->aNumValues),aNumValues, DAQ_NUM_OF_CH*sizeof(int));
        memcpy(&(daq->aWrites),aWrites, DAQ_NUM_OF_CH*sizeof(int));
        memcpy(&(daq->daqHandle),&handle,sizeof(int));
        memcpy(&(daq->errorAddress),&errAdress,sizeof(int));
    }

    printf("DAQ Handle: %d\n", daq->daqHandle);
    

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