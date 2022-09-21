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
    iter_cont = 0;

    extern struct States *s; // = (struct CUICStruct*)d;
    extern struct ControlParams *controlParams;
    extern struct DAQ *daq;
    sleep(1);
   
    struct timespec t_first;
    clock_gettime(CLOCK_MONOTONIC, &t_first);  

    double pos = 0.0;
    double vel = 0;
    double xnext = 0;

    double dir = 1.0;
    double cmd = 0.1;

    double highBound = 0.3;
    double lowBound = 0.06;
    double offset = 0.1;
    double offset_iter = 0.01;


    double FextArray[FEXT_FIR_SIZE + 1] = {0};
    int FextOrder = FEXT_FIR_SIZE;
    double VelArray[VEL_FIR_SIZE + 1] = {0};
    int VelOrder = VEL_FIR_SIZE;

    //double CmdArray[10 + 1] = {2.5};
    //int CmdOrder = 10;


    s_next->xv = 0.0;
    s_next->xv_prev = 0.0;

    while(true)
    {

        s = &((struct States*)d)[iter_cont];
        iter_cont= iter_cont + 1;
        if(iter_cont== BUFFER_SIZE) iter_cont= 0;
        s_next = &((struct States*)d)[iter_cont];


        clock_gettime(CLOCK_MONOTONIC, &s->t_start);  

        //***************************************************************************************************************
        
        getElapsedTime(&t_first, &s->t_start, &s->dt);

        //RunCommandSteps(s, &highBound, &lowBound, &offset, &offset_iter, &dir);
        //daq->aValues[0] = 2.5 - offset*dir;
        
        s->x0 = 0.15;
        
        
        if(iter_cont == 0 || iter_cont == 5)
        {
            PeriodicReset(s);
        }
        
        VirtualTrajectory(s,controlParams);
        ComputedTorque(s,controlParams);
        //GetCommand(s,controlParams);

        //BasicPD(s,controlParams);
        
        //FIR_FILTER(CmdArray, &s->cmd, &CmdOrder);
        /*
        if(s->cmd > 2.5 && fabs(s->dx) < 0.05)
        {
            s->cmd += 0.08;
        }
        if(s->cmd < 2.5 && fabs(s->dx) < 0.05)
        {
            s->cmd -= 0.08;
        }
        */


        daq->aValues[0] = s->cmd;
        
        //daq->aValues[0] = CMD_GAIN*s->cmd + CMD_OFFSET;

        //if(iter_cont==5) printf("Fext: %f, cmd: %f, xv: %f, dxv: %f, ddxv: %f\n",s->Fext,daq->aValues[0], s->xv, s->dxv, s->ddxv);
        //if(iter_cont==5) printf("Pressure: %f, Fext: %f\n", s->Fext, s->xv);

        //if(iter_cont==0) printf("dx: %f\n", s->dx);

        //if(abs(s->x - s->xv)>0.1) daq->aValues[0] = 2.5;
        
        if(daq->aValues[0] > 3.5) daq->aValues[0] = 3.5;
        if(daq->aValues[0] < 1.5) daq->aValues[0] = 1.5;
        
        //daq->aValues[0] = 2.5;
        s->cmd = daq->aValues[0];

        ReadWriteDAQ(s_next, daq);
        s_next->Fext -= controlParams->Fext_offset;
        checkVelocity(s,s_next);

        FIR_FILTER(FextArray, &s_next->Fext, &FextOrder);
        FIR_FILTER(VelArray, &s_next->dx, &VelOrder);

        pos += s_next->dx*(STEP_SIZE_MS/1000.0);
        s_next->x = pos;

        s_next->xv_prev = s->xv;
        s_next->dxv_prev = s->dxv;

        //***************************************************************************************************************
        
        pthread_mutex_lock(&s_next->lock);
        pthread_mutex_unlock(&s->lock);
        //t_last = s->t_start;
        
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