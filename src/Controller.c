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
#include "./include/TrajectoryModes.h"


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
        //

        //set trajectory
        Interpolation(&(controlParams->t), &(controlParams->x), &(s->t), &(controlParams->x0), controlParams->trajSize);
        s->x0 = controlParams->x0;
        printf("t:%f, x:%f\n",s->t,s->x0);

        //ctl*****************
        switch(controlParams->controlMode)
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
            case STOCHASTIC_MODE:
                //Stochastic forces in static positions for Limb Imp Estimation
                switch(controlParams->stochasticState)
                {

                    case 0:
                        //PositionMode(s, controlParams);
                        s->cmd = 0.0;
                        controlParams->tf = s->t;
                        controlParams->t_last = s->t;
                        break;
                    case 1:
                        if(s->t >= controlParams->t_last)
                        {
                            StochasticForceMode(s, controlParams);
                            controlParams->t_last = s->t + controlParams->stochasticStepTime;
                        }

                        if(s->t <= controlParams->t_last - (3.0* controlParams->stochasticStepTime / 4.0))
                        {
                            s_next->cmd = s->cmd;
                        }
                        else
                        {
                            s_next->cmd = 0.0;
                        }

                        if((s->t-controlParams->tf)>controlParams->phaseTime) //TODO change
                        {
                            controlParams->stochasticState = 2;
                            printf("Next state : 2\n");
                        }

                        break;
                    case 2: 
                        if(controlParams->x0 == controlParams->xend)
                            quitThreads = 1;
                        else
                        {
                            //avoid 5% of length on either end
                            controlParams->x0 += 0.9*controlParams->xend/(double)controlParams->numPositions;
                            controlParams->stochasticState = 0;
                            printf("x0: %f, Next state : 0\n", controlParams->x0);
                        }
                        break;
                
                }

                break; 
            case STOCHASTIC_TRAJ_MODE:
                //t_last is next force time
                //phase time is duration of force
                //stochastic Step time is duration between for application 
                AdmittanceMode(s, controlParams);

                if(s->t > controlParams->t_last + controlParams->phaseTime && controlParams->stochasticState == 1)
                {
                    controlParams->t_last += controlParams->phaseTime + controlParams->stochasticStepTime*(((double)rand()/(double)RAND_MAX) + 1.0);
                    printf("new t_last %f\n", controlParams->t_last);
                    controlParams->stochasticState = 2;
                }

                if(s->t > controlParams->t_last) 
                {
                    if(controlParams->stochasticState == 2)
                    {
                        controlParams->F_stochastic = (((double)rand()/(double)RAND_MAX)-0.5)*2.0*controlParams->Fmax;
                        controlParams->stochasticState = 1;
                    }
                    
                    s->cmd += controlParams->F_stochastic;
                    printf("Add force %f, %f\n",s->t, controlParams->F_stochastic);
                }                  
                
                break;
            
        }

        //ctl***************

        s->cmd += 2.5; 
        //printf("cmd : %f\n",s->cmd);

        if(s->cmd > 4) s->cmd = 4;
        if(s->cmd < 1) s->cmd = 1;

        if(s->lsb & s->cmd < 2.5) s->cmd = 2.5;
        if(s->lsf & s->cmd > 2.5) s->cmd = 2.5;

        daq->aValues[0] = s->cmd;

        ReadWriteDAQ(s_next, daq);
        s_next->Fext -= controlParams->Fext_offset;
        s->Fraw = s_next->Fext;

        if(controlParams->recordEMG)
        {
            s->emg1 = daq->aValues[6];
            s->emg2 = daq->aValues[7];
            s->emg3 = daq->aValues[8];
            s->emg4 = daq->aValues[9];
        }

       
        s->gonio = (double)daq->aValues[10]*0.002618;
        
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