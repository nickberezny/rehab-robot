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

void HomeToBack(struct States * s, struct DAQ * daq)
{
    extern struct ControlParams *controlParams;
    /**
     * @brief Slowly moves robot until contact with back limit switch
     * @param[in] *s : pointer to robot States
     */

    daq->aValues[0] = CMD_GAIN*(0.0) + CMD_OFFSET;
    ReadWriteDAQ(s, daq);
    s->lsb = daq->aValues[3];
    s->x = 0;
    s->dx = 0;
    daq->aValues[0] = CMD_GAIN*(0.25) + CMD_OFFSET;

    while(s->lsb == 0)
    {
        s->x -= s->dx;
        ReadWriteDAQ(s,daq);
        s->lsb = daq->aValues[3];
    }

    daq->aValues[0] = CMD_GAIN*(0.0) + CMD_OFFSET;
    controlParams->xend = s->x;
    printf("xend: %f\n",controlParams->xend);
}

void HomeToFront(struct States * s, struct DAQ * daq)
{
    /**
     * @brief Slowly moves robot until contact with front limit switch
     * @param[in] *s : pointer to robot States
     */

    daq->aValues[0] = CMD_GAIN*(0) + CMD_OFFSET;
    ReadWriteDAQ(s,daq);
    s->lsf = daq->aValues[2];
    

    daq->aValues[0] = CMD_GAIN*(-0.35) + CMD_OFFSET;

    while(s->lsf == 0)
    {
        ReadWriteDAQ(s,daq);
        s->lsf = daq->aValues[2];
    }

    daq->aValues[0] = CMD_GAIN*(0.0) + CMD_OFFSET;

}