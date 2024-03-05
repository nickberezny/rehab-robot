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