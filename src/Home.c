/**
 * @file Home.c
 * @author Nick Berezny
 * @date 13 Apr 2022
 * @brief Functions to home robot to front and back ends
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
#include "./include/Home.h"
#include "./include/Daq.h"

void HomeToBack(struct States * s, struct DAQ * daq, bool getXend)
{ 
    extern struct ControlParams *controlParams;
    extern struct CommData *commData;
    /**
     * @brief Slowly moves robot until contact with back limit switch
     * @param[in] *s : pointer to robot States
     */
    
    daq->aValues[0] = CMD_GAIN*(0.0) + CMD_OFFSET;
    
    ReadWriteDAQ(s, daq);
    
    char buffer[100];
    
    s->x = 0;
    s->dx = 0;
    daq->aValues[0] = CMD_GAIN*(-0.15) + CMD_OFFSET;

    //controlParams->x_for_q[0] = 0;
    //controlParams->q1[0] = s->xAccel[0];
    //controlParams->q2[0] = s->xAccel[1];
    //controlParams->qn = 1;

    while(s->lsb == 0)
    {
        s->x += s->dx*(STEP_SIZE_MS/1000.0);
        ReadWriteDAQ(s,daq);
        //readI2C(s, daq, 0);
        //readI2C(s, daq, 1);
        

        //sprintf(buffer, "%.4f::%.4f::%.4f::%.4f", s->x, s->xAccel[0], s->x, s->xAccel[1]);
        //sendMessage(commData->sockfd, buffer);
    }

    sprintf(buffer, "END");
    //sendMessage(commData->sockfd, buffer);

    daq->aValues[0] = CMD_GAIN*(0.0) + CMD_OFFSET;
    if(getXend) controlParams->xend = -s->x;
    printf("xend: %f\n",controlParams->xend); 


}

void HomeToFront(struct States * s, struct DAQ * daq)
{
    
    /**
     * @brief Slowly moves robot until contact with front limit switch
     * @param[in] *s : pointer to robot States
     */
    printf("daqhandle %d\n", daq->daqHandle);
    daq->aValues[0] = CMD_GAIN*(0.0) + CMD_OFFSET;
    LJM_eNames(daq->daqHandle, DAQ_NUM_OF_CH, daq->aNames, daq->aWrites, daq->aNumValues, daq->aValues, &(daq->errorAddress));

    
    daq->aValues[0] = CMD_GAIN*(0.28) + CMD_OFFSET;
    
    while(s->lsf == 0)
    {
        ReadWriteDAQ(s,daq);        

    }
    
    daq->aValues[0] = CMD_GAIN*(0.0) + CMD_OFFSET;

}
