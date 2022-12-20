/**
 * @file Controller.c
 * @author Nick Berezny
 * @date 6 Oct 2022
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
#include "./include/GetModelData.h"


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
    extern int iter_reset;
    iter_cont = 0;


    extern struct States *s; // = (struct CUICStruct*)d;
    extern struct ControlParams *controlParams;
    extern struct DAQ *daq;
    extern quitThreads;
    sleep(1);
    
    clock_gettime(CLOCK_MONOTONIC, &controlParams->t_first);  

    iter_reset = 0;
    double FextArray[FEXT_FIR_SIZE + 1] = {0};
    int FextOrder = FEXT_FIR_SIZE;
    double VelArray[VEL_FIR_SIZE + 1] = {0};
    int VelOrder = VEL_FIR_SIZE;
    

    controlParams->firstRun = true;
    s_next->x = 0.0;
    s_next->dx = 0.0;
    s_next->xv = 0.0;
    s_next->xv_prev = 0.0;

    while(!quitThreads)
    {
        
        s = &((struct States*)d)[iter_cont];
        iter_cont= iter_cont + 1;
        if(iter_cont == BUFFER_SIZE)
        {
            controlParams->firstRun = false;
            iter_cont = 0;
        } 
        s_next = &((struct States*)d)[iter_cont];

        clock_gettime(CLOCK_MONOTONIC, &s->t_start);  

        //***************************************************************************************************************
        
        getElapsedTime(&controlParams->t_first, &s->t_start, &s->dt);

        s->x0 = controlParams->x0;
        
        if(iter_cont == iter_reset && controlParams->delta !=0 )
        {
            PeriodicReset(s);
            printf("iter_reset:%d\n",iter_reset);

            iter_reset += controlParams->delta;

            if(iter_reset>BUFFER_SIZE-1)
            {
                iter_reset = iter_reset-BUFFER_SIZE;
            }

        }

        VirtualTrajectory(s,controlParams);
        ComputedTorque(s,controlParams);

        daq->aValues[0] = s->cmd;
        
        //daq->aValues[0] = CMD_GAIN*s->cmd + CMD_OFFSET;
        
        if(daq->aValues[0] > 3.5) daq->aValues[0] = 3.5;
        if(daq->aValues[0] < 1.5) daq->aValues[0] = 1.5;
        
        //daq->aValues[0] = 2.5;
        s->cmd = daq->aValues[0];
        ReadWriteDAQ(s_next, daq);
        s_next->Fext -= controlParams->Fext_offset;
        checkVelocity(s,s_next);

        FIR_FILTER(FextArray, &s_next->Fext, &FextOrder);
        FIR_FILTER(VelArray, &s_next->dx, &VelOrder);

        s_next->x = s->x + s_next->dx*(STEP_SIZE_MS/1000.0);

        s_next->xv_prev = s->xv;
        s_next->dxv_prev = s->dxv;

        //***************************************************************************************************************

        if(controlParams->firstRun) printf("mutex next unlock %d\n",pthread_mutex_unlock(&s_next->lock));
        pthread_mutex_lock(&s_next->lock);
        pthread_mutex_unlock(&s->lock);

        getTimeToSleep(&s->t_start, &s->t_end);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &s->t_end, NULL);

        
    }

    pthread_mutex_unlock(&s->lock);
    pthread_mutex_unlock(&s_next->lock);
    printf("Done Controller...\n");
    return NULL;
}


void checkVelocity(struct States * s, struct States * s_next)
{
    extern struct ControlParams *controlParams;
    //printf("Vel: %f next: %f\n", s->dx, s_next->dx);
    if(abs(s_next->dx)>controlParams->dx_bound && abs(s->dx-s_next->dx)>abs(s->dx+s_next->dx))
    {
        s_next->dx = -s_next->dx;
        printf("Check Vel!\n");
    }

}