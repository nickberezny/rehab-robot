/**
 * @file Communication.c
 * @author Nick Berezny
 * @date 17 Aug 2022
 * @brief Calibrate force sensor offset, etc
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
#include "./include/Calibration.h"
#include "./include/Daq.h"


void AddJointAngel(struct States * s, struct DAQ * daq)
{
    //read Daq
    //read I2C
    //Add to interp vectors ...

    double n = 100.0;

    extern struct ControlParams *controlParams;

    controlParams->x_for_q[controlParams->qn] = 0;
    controlParams->q1[controlParams->qn] = 0;
    controlParams->q2[controlParams->qn] = 0;

    for(int i = 0; i < n; i++)
    {
        daq->aValues[0] = CMD_GAIN*(0.0) + CMD_OFFSET;
        readAbsolutePosition(s, daq);
        readI2C(s, daq, 0);
        readI2C(s, daq, 1);

        controlParams->x_for_q[controlParams->qn] += s->x;
        controlParams->q1[controlParams->qn] += s->xAccel[0];
        controlParams->q2[controlParams->qn] += s->xAccel[1];

        
    }

    controlParams->x_for_q[controlParams->qn] = controlParams->x_for_q[controlParams->qn]/n;
    controlParams->q1[controlParams->qn] = controlParams->q1[controlParams->qn]/n;
    controlParams->q2[controlParams->qn] = controlParams->q2[controlParams->qn]/n;
    controlParams->qn += 1;

}

void CalibrateForceOffset(struct States * s, struct DAQ * daq)
{
    extern struct ControlParams *controlParams;

    double force = 0;
    double gyro1 = 0;
    double gyro2 = 0;
    int samples = 3000;

    for(int i = 0; i < samples; i++)
    {
        daq->aValues[0] = 2.5;
        ReadWriteDAQ(s,daq);
        force += s->Fext;
        gyro1 += s->dxGyro[0];
        gyro2 += s->dxGyro[1];
        usleep(1000);
    }   

    controlParams->gyro_offset[0] = gyro1/((double)samples);
    controlParams->gyro_offset[1] = gyro2/((double)samples);
    controlParams->Fext_offset = force/((double)samples);
    printf("Force offset: %f\n", controlParams->Fext_offset);
}

void CalibrateFSR(struct States * s, struct DAQ * daq)
{

    double fsr1 = 0;
    double fsr2 = 0;
    int samples = 3000;

    for(int i = 0; i < samples; i++)
    {
        daq->aValues[0] = 2.5;
        ReadWriteDAQ(s,daq);
        fsr1 += daq->aValues[3];
        fsr2 += daq->aValues[4];
        usleep(1000);
    }

    fsr1 = fsr1/3000.0;
    fsr2 = fsr2/3000.0;


}