/**
 * @file Home.c
 * @author Nick Berezny
 * @date 3 Mar 2022
 * @Homing the robot
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

void HomeToBack(struct States * s)
{
    s->daq.aValues[0] = MOTOR_ZERO; 
    ReadWriteDAQ(s);
    s->h.lsb = s->daq.aValues[3];

    s->daq.aValues[0] = MOTOR_SLOW_BWD; 

    while(s->h.lsb == 0)
    {
        ReadWriteDAQ(s);
        s->h.lsb = s->daq.aValues[3];
    }
}

void HomeToFront(struct States * s)
{
    s->daq.aValues[0] = MOTOR_ZERO; 
    ReadWriteDAQ(s);
    s->h.lsf = s->daq.aValues[2];
    s->x = 0;

    s->daq.aValues[0] = MOTOR_SLOW_FWD; 

    while(s->h.lsf == 0)
    {
        s->x += ENC_TO_MM * (double)s->daq.aValues[4];
        ReadWriteDAQ(s);
        s->h.lsf = s->daq.aValues[2];
    }

   s->xend = s->x;
}