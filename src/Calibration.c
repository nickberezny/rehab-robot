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
    int samples = 3000;

    for(int i = 0; i < samples; i++)
    {
        daq->aValues[0] = 2.5;
        ReadWriteDAQ(s,daq);
        force += s->Fext;
        usleep(1000);
    }   

    controlParams->Fext_offset = force/((double)samples);
    printf("Force offset: %f\n", controlParams->Fext_offset);
}