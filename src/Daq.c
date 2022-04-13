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


void ReadWriteDAQ(struct States * s)
{
    /**
     * @brief simultaneously read and write to Daq
     * @param[in] *s : pointer to robot States to store results
     */
    

    LJM_eNames(s->daq.daqHandle, 5, s->daq.aNames, s->daq.aWrites, s->daq.aNumValues, s->daq.aValues, &(s->daq.errorAddress));
}

int initDaq(struct States * s)
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
    LJM_eWriteName(handle, "DIO2_EF_ENABLE", 0);
    LJM_eWriteName(handle, "DIO3_EF_ENABLE", 0);

    LJM_eWriteName(handle, "DIO2_EF_INDEX", 10);
    LJM_eWriteName(handle, "DIO3_EF_INDEX", 10);

    LJM_eWriteName(handle, "DIO2_EF_ENABLE", 1);
    LJM_eWriteName(handle, "DIO3_EF_ENABLE", 1);

    //set Analog in resolution
    LJM_eWriteName(handle, "AIN0_RESOLUTION_INDEX", 1);
    LJM_eWriteName(handle, "AIN0_SETTLING_US", 0);

    printf("%d\n", handle);


    double aValues[5] = {0};
    const char * aNames[5] = {"DAC0", "AIN0","FIO0", "FIO1", "DIO2_EF_READ_A_F_AND_RESET"};
    int aNumValues[5] = {1,1,1,1,1};
    int aWrites[5] = {1,0,0,0,0};

    memcpy(s->daq.aNames,aNames, 5*sizeof(double));

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