#include <LabJackM.h>
#include <stdio.h>
#include <stdbool.h>

#include "./include/Daq.h"

int initDaq()
{

/*------------------------------------------------------------------------
    Connect to Labjack DAQ, and set up its quadrature encoder counter 
------------------------------------------------------------------------*/
    
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
    return handle;

}

bool closeAllDaqs()
{
	LJM_ERROR_RETURN err;
	err = LJM_CloseAll();

	if(err == 0) return true;
	else return false;
}