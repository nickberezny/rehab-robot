/**
 * @file Controller.c
 * @author Nick Berezny
 * @date 13 Apr 2022
 * @brief Contains controller thread for real-time control of robot
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>
#include <sched.h>
#include <stdbool.h>
#include <limits.h>

#include "./include/Parameters.h"
#include "./include/Structures.h"
#include "./include/TimeUtilities.h"
#include "./include/Controller.h"
#include "./include/CUIC.h"
#include "./include/Daq.h"


void * controllerThread (void * d)
{
    /**
     * @brief ControllerThread function to be run in POSIX thread
     * @param[in] *d : pointer to robot States structure
     */
    
    printf("Starting control thread...\n");
    extern struct States *s_next; // = (struct CUICStruct*)d;
    s_next = &((struct States*)d)[0];

    pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
    printf("mutex cont %d\n",pthread_mutex_lock(&s_next->lock));

    extern int iter_cont;
    iter_cont = 0;

    extern struct States *s; // = (struct CUICStruct*)d;
    extern struct ControlParams *ctlParams;
    extern struct DAQ *daq;
    sleep(1);
   
    struct timespec t_last;
    clock_gettime(CLOCK_MONOTONIC, &t_last);  

    double pos = 0;
    double vel = 0;
    double cmd = 0.2;

    while(true)
    {

        s = &((struct States*)d)[iter_cont];
        iter_cont= iter_cont + 1;
        if(iter_cont== BUFFER_SIZE) iter_cont= 0;
        s_next = &((struct States*)d)[iter_cont];


        clock_gettime(CLOCK_MONOTONIC, &s->t_start);  

        //***************************************************************************************************************
        
        //printf("time: %d ; %d\n", s->t_start.tv_sec - t_last.tv_sec, s->t_start.tv_nsec - t_last.tv_nsec);
        if(daq->aValues[2] || daq->aValues[3]) cmd = 0;
        //read daq 
        daq->aValues[0]=CMD_GAIN*(-10.0*(0.21-pos) + 2.0*s->dx) + CMD_OFFSET;
        if(daq->aValues[0] > 3.5) daq->aValues[0] = 3.5;
        if(daq->aValues[0] < 1.5) daq->aValues[0] = 1.5;

        ReadWriteDAQ(s, daq);
        printf("CMD: %f, FT: %f, LS: %f, %f, Enc: %f, %f\n", daq->aValues[0], daq->aValues[1], daq->aValues[2], daq->aValues[3], daq->aValues[4], daq->aValues[5]);

        //vel = (double)s->daq.aValues[4]*s->daqENC_TO_MM;
        pos += s->dx;
        s->x = pos;

        printf("Pos: %f",pos);

        //***************************************************************************************************************
        
        printf("mutex cont lock %d\n",pthread_mutex_lock(&s_next->lock));
        printf("mutex cont unlock %d\n",pthread_mutex_unlock(&s->lock));
        t_last = s->t_start;
        
        getTimeToSleep(&s->t_start, &s->t_end);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &s->t_end, NULL);

        
    }



    pthread_mutex_unlock(&s->lock);
    pthread_mutex_unlock(&s_next->lock);
    printf("Done Controller...\n");
    return NULL;
}

