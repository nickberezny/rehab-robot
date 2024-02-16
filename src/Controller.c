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
#include "./include/ControlModes.h"
#include "./include/Tensorflow.h"


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
    //what?
    
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
        
        getElapsedTime(&controlParams->t_first, &s->t_start, &s->t);
        if(controlParams->t_last <= s->t)
        {
            quitThreads = true;
        }

        
        //set trajectory
        Interpolation((controlParams->t), (controlParams->x), &(s->t), &(controlParams->x0), &(controlParams->x0_index), controlParams->trajSize);
        Interpolation((controlParams->tdist), (controlParams->xdist), &(s->t), &(controlParams->x0dist), &(controlParams->x0_index), controlParams->trajSize);

        
        s->x0 = controlParams->x0 + controlParams->x0dist;
        s->x0_to_send = controlParams->x0;
        s->x0_duration = controlParams->x0_duration[controlParams->x0_index];
        if(controlParams->x0_is_percent) s->x0 = s->x0*controlParams->xend;

        if(controlParams->x0dist != 0)
        {
            if(controlParams->F_index == 0) controlParams->X_init_for_Kest = s->x;
            controlParams->F_for_Kest[controlParams->F_index] = s->Fext;
            controlParams->F_index = controlParams->F_index + 1;
        }
        else if(controlParams->x0dist == 0 && controlParams->F_index != 0)
        {
            //calc Kest
            //avg force
            //controlParams->Kest = Fmean/(s->x-controlParam->X_init_for_Kest);
            AverageVector(controlParams->F_for_Kest, &(controlParams->Kest), 100);
            controlParams->Kest = controlParams->Kest/(s->x - controlParams->X_init_for_Kest + 0.000001);
            controlParams->F_index = 0;
            //printf("Kest: %f\n", controlParams->Kest);
        }

        //printf("t:%f, x:%f, x0d: %f\n",s->t,s->x0,s->x0_duration);
        
        //ctl*****************
        switch((int)controlParams->controlMode)
        {
            case PD_MODE:
                //Position control to x0
                PositionMode(s, controlParams);
                break; 
            case IMP_MODE:
                //Impedance Control with Computed Torque
                ImpedanceMode(s, controlParams);
                break; 
            case ADM_MODE:
                //Admittance Control
                AdmittanceMode(s, controlParams);
                break; 
            case UIC_MODE:
                //Unified Interaction Control
                if(iter_cont == iter_reset && controlParams->delta !=0 )
                {
                    PeriodicReset(s);
                    iter_reset += controlParams->delta;
                    printf("iter reset %d\n", iter_cont);

                    if(iter_reset>BUFFER_SIZE-1)
                    {

                        iter_reset = iter_reset-BUFFER_SIZE;
                    }

                }
                UICMode(s, controlParams);
                break; 
            case UIAC_MODE:
                //Unified Impedance and Admittance Control
                if(controlParams->cont_iteration < controlParams->delta*controlParams->alpha)
                {
                    ImpedanceMode(s,controlParams);
                    PeriodicReset(s);
                    controlParams->cont_iteration = controlParams->cont_iteration  + 1;
                    printf("imp %f\n", s->cmd);
                    printf("test test %d\n", controlParams->cont_iteration);
                }
                else
                {
                    AdmittanceMode(s,controlParams);
                    controlParams->cont_iteration = controlParams->cont_iteration + 1;
                    printf("adm %f\n", s->cmd);
                }

                if(controlParams->cont_iteration > controlParams->delta)
                {
                    controlParams->cont_iteration = 0;
                }
                //run imp for delta
                //run adm for delta (reset)
                break;
            case FORCE_NORM_MODE:
                s->x0 = 0.5 + (0.1/controlParams->xend);
                s->x0_to_send = 0.5*controlParams->xend;
                AdmittanceMode(s, controlParams);
                break;
                
            
        }

        //ctl***************
        
        s->cmd += 2.5; 
        //printf("cmd : %f\n",s->cmd);

        if(s->cmd > 4) s->cmd = 4;
        if(s->cmd < 1) s->cmd = 1;

        if(s->lsb & s->cmd < 2.5) s->cmd = 2.5;
        if(s->lsf & s->cmd > 2.5) s->cmd = 2.5;

        if(quitThreads) s->cmd = 2.5;

        daq->aValues[0] = s->cmd;
        
        ReadWriteDAQ(s_next, daq);
        s_next->Fext -= controlParams->Fext_offset;
        s->Fraw = s_next->Fext;
        
         if(controlParams->recordEMG)
        {
            s->emg1 = daq->aValues[4];
            s->emg2 = daq->aValues[5];
            s->emg3 = daq->aValues[6];
            s->emg4 = daq->aValues[7];
        }
    

        s_next->x = s->x + s_next->dx*(STEP_SIZE_MS/1000.0);
        //checkVelocity(s,s_next);

        //FIR_FILTER(FextArray, &s_next->Fext, &FextOrder);
        //FIR_FILTER(VelArray, &s_next->dx, &VelOrder);

        Butterworth10(&(s_next->dx),&(s->dx),controlParams->dx_filt_x,controlParams->dx_filt_y, controlParams->filter_a_100Hz, controlParams->filter_b_100Hz);
        Butterworth10(&(s_next->Fext),&(s->Fext),controlParams->F_filt_x,controlParams->F_filt_y, controlParams->filter_a_10Hz, controlParams->filter_b_10Hz);

        s_next->Fext = s->Fext;
        s_next->dx = s->dx;
        

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