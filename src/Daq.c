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
#include "./include/ForceSensor.h"
#include "./include/Daq.h"

void clearEncoderCounter(struct DAQ * daq)
{
        //TODO CHANGE COMMAND TO GET ABS POS
    LJM_eWriteName(daq->daqHandle, "ASYNCH_NUM_BYTES_TX", 1);
    LJM_eWriteNameArray(daq->daqHandle, "ASYNCH_DATA_TX", 1, &daq->clearCnt, &(daq->errorAddress));
    LJM_eWriteName(daq->daqHandle, "ASYNCH_TX_GO", 1);
}

void readAbsolutePosition(struct States * s, struct DAQ * daq)
{
        //TODO CHANGE COMMAND TO GET ABS POS
    LJM_eWriteName(daq->daqHandle, "ASYNCH_NUM_BYTES_TX", 1);
    LJM_eWriteNameArray(daq->daqHandle, "ASYNCH_DATA_TX", 1, &daq->readAbs, &(daq->errorAddress));
    LJM_eWriteName(daq->daqHandle, "ASYNCH_TX_GO", 1);

    LJM_eWriteName(daq->daqHandle, "ASYNCH_NUM_BYTES_RX", 3);
    LJM_eReadNameArray(daq->daqHandle, "ASYNCH_DATA_RX", 3, daq->dataRead, &(daq->errorAddress));

    s->d1 = (int)daq->dataRead[0];
    s->d2 = (int)daq->dataRead[1];
    s->d3 = (int)daq->dataRead[2];

    printf("%d,%d,%d\n",s->d1,s->d2,s->d3);

    if(s->d1 == 254) s->x = s->d2 + 100*s->d3;
    else if(s->d2 == 254) s->x = s->d3 + 100*s->d1;
    else if(s->d3 == 254) s->x = s->d1 + 100*s->d2;
    else s->x = 0.0; 

    s->x = s->x*ENC_TO_M;
}

void readI2C(struct States * s, struct DAQ * daq, int index)
{

    //printf("Send %d\n", (int)daq->i2cSend[0]);

    LJM_eWriteName(daq->daqHandle, "I2C_SLAVE_ADDRESS", daq->i2cAddr[index]);

    LJM_eWriteNameByteArray(daq->daqHandle, I2C_WRITE_NAME, 1, daq->i2cSend, &(daq->errorAddress));
    LJM_eWriteName(daq->daqHandle, "I2C_GO", 1); // Do the I2C communications.


    // Read the RX bytes
    for (int i = 0; i < 14; i++) {
        s->i2cRead[i] = 0;
    }

    LJM_eReadNameByteArray(daq->daqHandle, I2C_READ_NAME, 14, s->i2cRead, &(daq->errorAddress));

    s->accel[0] = ((s->i2cRead[0] << 8) + s->i2cRead[1])/16384.0; //x
    s->accel[1] = ((s->i2cRead[2] << 8) + s->i2cRead[3])/16384.0; //y
    s->accel[2] = ((s->i2cRead[4] << 8) + s->i2cRead[5])/16384.0; //z

    s->xAccel[index] = atan2(s->accel[1], sqrt( s->accel[0]*s->accel[0] + s->accel[2]*s->accel[2]));
    s->dxGyro[index] = ((s->i2cRead[8] << 8) + s->i2cRead[9])/(1.114*32.0*30023.0);


    //return x angle (acc) and gryo x vel 
}

void ReadWriteDAQ(struct States * s, struct DAQ * daq)
{
    /**
     * @brief simultaneously read and write to Daq
     * @param[in] *s : pointer to robot States to store results
     */
    //readI2C(s, daq, 0);
    //readI2C(s, daq, 1);

    //UART for position 

    LJM_eWriteName(daq->daqHandle, "ASYNCH_NUM_BYTES_TX", 1);
    LJM_eWriteNameArray(daq->daqHandle, "ASYNCH_DATA_TX", 1, &daq->readDiff, &(daq->errorAddress));
    LJM_eWriteName(daq->daqHandle, "ASYNCH_TX_GO", 1);

    

    int err = LJM_eNames(daq->daqHandle, DAQ_NUM_OF_CH, daq->aNames, daq->aWrites, daq->aNumValues, daq->aValues, &(daq->errorAddress));
    //readFroceSensor(daq->fdata);
    /*
    s->Fext = daq->fdata->F[2];//0.001*(FT_GAIN_g*daq->aValues[1] + FT_OFFSET_g)*9.81; //in N
    
    s->F[0] = daq->fdata->F[0];
    s->F[1] = daq->fdata->F[1];
    s->F[2] = daq->fdata->F[2];
    s->T[0] = daq->fdata->T[0];
    s->T[1] = daq->fdata->T[1];
    s->T[2] = daq->fdata->T[2];
*/
    s->lsb = daq->aValues[1];
    s->lsf = daq->aValues[2];
    
    if(err != 0) printf("daq err %d\n", err);

    //read UART
    LJM_eWriteName(daq->daqHandle, "ASYNCH_NUM_BYTES_RX", 3);
    LJM_eReadNameArray(daq->daqHandle, "ASYNCH_DATA_RX", 3, daq->dataRead, &(daq->errorAddress));

    //printf("%d,%d,%d,\n",(int)daq->dataRead[0],(int)daq->dataRead[1],(int)daq->dataRead[2]);

    s->d1 = (int)daq->dataRead[0];
    s->d2 = (int)daq->dataRead[1];
    s->d3 = (int)daq->dataRead[2];
   
    if(s->d1 == 254) s->dx = s->d2 + 100*s->d3;
    else if(s->d2 == 254) s->dx = s->d3 + 100*s->d1;
    else if(s->d3 == 254) s->dx = s->d1 + 100*s->d2;
    else if(s->d1 == 255) s->dx = -(s->d2 + 100*s->d3);
    else if(s->d2 == 255) s->dx = -(s->d3 + 100*s->d1);
    else if(s->d3 == 255) s->dx = -(s->d1 + 100*s->d2);
    else s->dx = 0.0; 

    s->dx = s->dx*ENC_TO_M/(STEP_SIZE_MS/1000.0);

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
   
    err = LJM_Open(LJM_dtANY, LJM_ctANY, "LJM_idANY", &handle);

    LJM_eStreamStop(handle); //stop any previous streams

    //set Analog in resolution
    LJM_eWriteName(handle, "AIN0_RESOLUTION_INDEX", 1);
    LJM_eWriteName(handle, "AIN0_SETTLING_US", 0);
    LJM_eWriteName(handle, "AIN_ALL_NEGATIVE_CH", 199);

    LJM_eWriteName(handle, "AIN1_RESOLUTION_INDEX", 1);
    LJM_eWriteName(handle, "AIN2_RESOLUTION_INDEX", 1);
    LJM_eWriteName(handle, "AIN3_RESOLUTION_INDEX", 1);
    LJM_eWriteName(handle, "AIN4_RESOLUTION_INDEX", 1);

    LJM_eWriteName(handle, "ASYNCH_ENABLE", 0);
    LJM_eWriteName(handle, "ASYNCH_TX_DIONUM", 4);  
    LJM_eWriteName(handle, "ASYNCH_RX_DIONUM", 5);  
    LJM_eWriteName(handle, "ASYNCH_BAUD", 38400);
    LJM_eWriteName(handle, "ASYNCH_NUM_DATA_BITS", 8);
    LJM_eWriteName(handle, "ASYNCH_PARITY", 0);
    LJM_eWriteName(handle, "ASYNCH_NUM_STOP_BITS", 1);
    LJM_eWriteName(handle, "ASYNCH_ENABLE", 1);

    
    //I2C
    LJM_eWriteName(handle, "I2C_SDA_DIONUM", 3);
    LJM_eWriteName(handle, "I2C_SCL_DIONUM", 2);

    LJM_eWriteName(handle, "I2C_SPEED_THROTTLE", 0);
    LJM_eWriteName(handle, "I2C_OPTIONS", 0);

    char aBytes[12] = {0x6B, 0x00};
    

    LJM_eWriteName(handle, "I2C_SLAVE_ADDRESS", 0x68);
    LJM_eWriteName(handle, "I2C_NUM_BYTES_TX", 2); // Set the number of bytes to transmit
    LJM_eWriteName(handle, "I2C_NUM_BYTES_RX", 14); // Set the number of bytes to receive
    LJM_eWriteNameByteArray(handle, I2C_WRITE_NAME, 2, aBytes, &errAdress);
    LJM_eWriteName(handle, "I2C_GO", 1); // Do the I2C communications.
    LJM_eWriteName(handle, "I2C_NUM_BYTES_TX", 1); // Set the number of bytes to transmit

    LJM_eWriteName(handle, "I2C_SLAVE_ADDRESS", 0x69);
    LJM_eWriteName(handle, "I2C_NUM_BYTES_TX", 2); // Set the number of bytes to transmit
    LJM_eWriteName(handle, "I2C_NUM_BYTES_RX", 14); // Set the number of bytes to receive
    LJM_eWriteNameByteArray(handle, I2C_WRITE_NAME, 2, aBytes, &errAdress);
    LJM_eWriteName(handle, "I2C_GO", 1); // Do the I2C communications.
    LJM_eWriteName(handle, "I2C_NUM_BYTES_TX", 1); // Set the number of bytes to transmit
    

    double aValues[DAQ_NUM_OF_CH];
    memset( aValues, 0, DAQ_NUM_OF_CH*sizeof(double) );
    int aNumValues[DAQ_NUM_OF_CH];
    memset( aNumValues, 0, DAQ_NUM_OF_CH*sizeof(int) );
    int aWrites[DAQ_NUM_OF_CH];
    memset( aWrites, 0, DAQ_NUM_OF_CH*sizeof(int) );

    aWrites[0] = 1;

    for(int i = 0; i <DAQ_NUM_OF_CH;i++)
        aNumValues[i] = 1;

    const char * aNamesTmp10[7] = {"DAC0","FIO0", "FIO1","AIN0","AIN1","AIN2","AIN3"};  

    memcpy(&(daq->aNames),aNamesTmp10, 100*daq->numChannels*sizeof(char));

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

    daq->readDiff = 7;
    daq->clearCnt = 4;
    daq->readAbs = 9;

    daq->i2cAddr[0] = 0x68;
    daq->i2cAddr[1] = 0x69;
    daq->i2cSend[0] = 0x3B;
    
    printf("DAQ Handle: %d\n", daq->daqHandle);

    LJM_eWriteName(daq->daqHandle, "ASYNCH_NUM_BYTES_TX", 1);
    LJM_eWriteNameArray(daq->daqHandle, "ASYNCH_DATA_TX", 1, &daq->readDiff, &(daq->errorAddress));
    LJM_eWriteName(daq->daqHandle, "ASYNCH_TX_GO", 1);
    
    LJM_eWriteName(daq->daqHandle, "ASYNCH_NUM_BYTES_TX", 1);
    LJM_eWriteNameArray(daq->daqHandle, "ASYNCH_DATA_TX", 1, &daq->readDiff, &(daq->errorAddress));
    LJM_eWriteName(daq->daqHandle, "ASYNCH_TX_GO", 1);
    LJM_eWriteName(daq->daqHandle, "ASYNCH_NUM_BYTES_TX", 1);
    LJM_eWriteNameArray(daq->daqHandle, "ASYNCH_DATA_TX", 1, &daq->readDiff, &(daq->errorAddress));
    LJM_eWriteName(daq->daqHandle, "ASYNCH_TX_GO", 1);

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