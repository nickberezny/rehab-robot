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

    int err = LJM_eNames(daq->daqHandle, DAQ_NUM_OF_CH, daq->aNames, daq->aWrites, daq->aNumValues, daq->aValues, &(daq->errorAddress));

    //UART for position 
    LJM_eWriteName(daq->daqHandle, "ASYNCH_NUM_BYTES_TX", 1);
    LJM_eWriteNameArray(daq->daqHandle, "ASYNCH_DATA_TX", 1, daq->writeValues, &(daq->errorAddress));
    LJM_eWriteName(daq->daqHandle, "ASYNCH_TX_GO", 1);

    //s->dx = (1.0 - 2.0*(double)daq->aValues[4])*((double)daq->aValues[5])*ENC_TO_M/(STEP_SIZE_MS/1000.0); //in m/dt
    s->Fext = 0.001*(FT_GAIN_g*daq->aValues[1] + FT_OFFSET_g)*9.81; //in N
    s->lsb = daq->aValues[2];
    s->lsf = daq->aValues[3];
    
    if(err != 0) printf("daq err %d\n", err);

    LJM_eWriteName(daq->daqHandle, "ASYNCH_NUM_BYTES_RX", 4);
    LJM_eReadNameArray(daq->daqHandle, "ASYNCH_DATA_RX", 4, daq->dataRead, &(daq->errorAddress));

    //printf("%d,%d,%d,%d\n",(int)daq->dataRead[0],(int)daq->dataRead[1],(int)daq->dataRead[2],(int)daq->dataRead[3]);

    if((int)daq->dataRead[0] == 255) s->dx = (int)daq->dataRead[1] + 100*(int)daq->dataRead[2] + 10000*(int)daq->dataRead[3];
    else if((int)daq->dataRead[1] == 255) s->dx = (int)daq->dataRead[2] + 100*(int)daq->dataRead[3] + 10000*(int)daq->dataRead[0];
    else if((int)daq->dataRead[2] == 255) s->dx = (int)daq->dataRead[3] + 100*(int)daq->dataRead[0] + 10000*(int)daq->dataRead[1];
    else if((int)daq->dataRead[3] == 255) s->dx = (int)daq->dataRead[0] + 100*(int)daq->dataRead[1] + 10000*(int)daq->dataRead[2];
    else s->dx = 0.0;
}


int initDaq(struct DAQ *daq) 
{

    /**
     * @brief initialize Daq 
     * @param[in] *s : pointer to robot States to store results
     */
    printf("Daq size: %d\n", daq->numChannels);
    
    int err, handle, errAdress;
    handle = 0;
    // Open first found LabJack
    err = LJM_Open(LJM_dtANY, LJM_ctANY, "LJM_idANY", &handle);
    //ErrorCheck(err, "LJM_Open");

    LJM_eStreamStop(handle); //stop any previous streams
    //LJM_eWriteName(handle, "DAC0", MOTOR_ZERO); //set motor to zero

    /*
    printf("LJM error: %d\n",LJM_eWriteName(handle, "DIO3_EF_ENABLE", 0));
    printf("LJM error: %d\n",LJM_eWriteName(handle, "DIO3_EF_INDEX", 8));
    printf("LJM error: %d\n",LJM_eWriteName(handle, "DIO3_EF_ENABLE", 1));
    */
    printf("LJM error: %d\n",LJM_eWriteName(handle, "DIO6_EF_ENABLE", 0));
    printf("LJM error: %d\n",LJM_eWriteName(handle, "DIO6_EF_INDEX", 10));
    printf("LJM error: %d\n",LJM_eWriteName(handle, "DIO6_EF_ENABLE", 1));
    printf("LJM error: %d\n",LJM_eWriteName(handle, "DIO7_EF_ENABLE", 0));
    printf("LJM error: %d\n",LJM_eWriteName(handle, "DIO7_EF_INDEX", 10));
    printf("LJM error: %d\n",LJM_eWriteName(handle, "DIO7_EF_ENABLE", 1));
   

    //set Analog in resolution
    LJM_eWriteName(handle, "AIN0_RESOLUTION_INDEX", 1);
    LJM_eWriteName(handle, "AIN0_SETTLING_US", 0);
    LJM_eWriteName(handle, "AIN_ALL_NEGATIVE_CH", 199);

    LJM_eWriteName(handle, "AIN1_RESOLUTION_INDEX", 1);
    LJM_eWriteName(handle, "AIN2_RESOLUTION_INDEX", 1);
    LJM_eWriteName(handle, "AIN3_RESOLUTION_INDEX", 1);
    LJM_eWriteName(handle, "AIN4_RESOLUTION_INDEX", 1);

    LJM_eWriteName(handle, "ASYNCH_ENABLE", 0);
    LJM_eWriteName(handle, "ASYNCH_TX_DIONUM", 2);  
    LJM_eWriteName(handle, "ASYNCH_RX_DIONUM", 3);  
    LJM_eWriteName(handle, "ASYNCH_BAUD", 19200);
    LJM_eWriteName(handle, "ASYNCH_NUM_DATA_BITS", 8);
    LJM_eWriteName(handle, "ASYNCH_PARITY", 0);
    LJM_eWriteName(handle, "ASYNCH_NUM_STOP_BITS", 1);
    LJM_eWriteName(handle, "ASYNCH_ENABLE", 1);


    double aValues[DAQ_NUM_OF_CH];
    memset( aValues, 0, DAQ_NUM_OF_CH*sizeof(double) );
    int aNumValues[DAQ_NUM_OF_CH];
    memset( aNumValues, 0, DAQ_NUM_OF_CH*sizeof(int) );
    int aWrites[DAQ_NUM_OF_CH];
    memset( aWrites, 0, DAQ_NUM_OF_CH*sizeof(int) );

    aWrites[0] = 1;

    for(int i = 0; i <DAQ_NUM_OF_CH;i++)
        aNumValues[i] = 1;

    const char * aNamesTmp6[4] = {"DAC0", "AIN0","FIO0", "FIO1"}; 
    const char * aNamesTmp10[8] = {"DAC0", "AIN0","FIO0", "FIO1","AIN1","AIN2","AIN3","AIN4"}; 
    const char * aNamesTmp11[9] = {"DAC0", "AIN0","FIO0", "FIO1","AIN1","AIN2","AIN3","AIN4","DIO6_EF_READ_A_F"}; 

    printf("DAQ NUM %d\n", daq->numChannels);

    
    if(daq->numChannels == 9)
        memcpy(&(daq->aNames),aNamesTmp11, 100*daq->numChannels*sizeof(char));
    else if(daq->numChannels == 8)
        memcpy(&(daq->aNames),aNamesTmp10, 100*daq->numChannels*sizeof(char));
    else
        memcpy(&(daq->aNames),aNamesTmp6, 100*daq->numChannels*sizeof(char));
    
    printf("DAQ NUM %d\n", daq->numChannels);
    memcpy(&(daq->aValues), aValues, DAQ_NUM_OF_CH*sizeof(double));
    memcpy(&(daq->aNumValues),aNumValues, DAQ_NUM_OF_CH*sizeof(int));
    memcpy(&(daq->aWrites),aWrites, DAQ_NUM_OF_CH*sizeof(int));
    memcpy(&(daq->daqHandle),&handle,sizeof(int));
    memcpy(&(daq->errorAddress),&errAdress,sizeof(int));

    daq->dataRead[0] = 0;
    daq->dataRead[1] = 0;
    daq->dataRead[2] = 0;
    daq->dataRead[3] = 0;

    daq->writeValues[0] = 7;
    
    printf("DAQ Handle: %d\n", daq->daqHandle);

    LJM_eWriteName(daq->daqHandle, "ASYNCH_NUM_BYTES_TX", 1);
    LJM_eWriteNameArray(daq->daqHandle, "ASYNCH_DATA_TX", 1, daq->writeValues, &(daq->errorAddress));
    LJM_eWriteName(daq->daqHandle, "ASYNCH_TX_GO", 1);
    
    LJM_eWriteName(daq->daqHandle, "ASYNCH_NUM_BYTES_TX", 1);
    LJM_eWriteNameArray(daq->daqHandle, "ASYNCH_DATA_TX", 1, daq->writeValues, &(daq->errorAddress));
    LJM_eWriteName(daq->daqHandle, "ASYNCH_TX_GO", 1);
    LJM_eWriteName(daq->daqHandle, "ASYNCH_NUM_BYTES_TX", 1);
    LJM_eWriteNameArray(daq->daqHandle, "ASYNCH_DATA_TX", 1, daq->writeValues, &(daq->errorAddress));
    LJM_eWriteName(daq->daqHandle, "ASYNCH_TX_GO", 1);

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

void readSPI()
{
    
}