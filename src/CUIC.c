/**
 * @file CUIC.c
 * @author Nick Berezny
 * @date 13 Apr 2022
 * @brief Functions for CUIC [insert link to paper]
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
#include "./include/CUIC.h"
#include "./include/TimeUtilities.h"

void VirtualTrajectory(struct States * s, struct ControlParams * p)
{
    /**
     * @brief determines next step of virtual trajectory according to desired impedance
     * @param[in] *s : pointer to robot States to store results
     * @param[in] *p : pointer to robot Params containing impedance values
     */
    
    s->xv = p->Ad[0]*(s->xv_prev-s->x0) + p->Ad[1]*s->dxv_prev + p->Bd[0]*s->Fext + s->x0 ;
    s->dxv = p->Ad[2]*(s->xv_prev-s->x0) + p->Ad[3]*s->dxv_prev + p->Bd[1]*s->Fext;
    s->ddxv = (s->Fext/p->Md - (p->Dd)/(p->Md)*(s->dxv) - (p->Kd)/(p->Md)*(s->xv - s->x0));///(STEP_SIZE_MS*1000.0); 

    if(p->xend >= 0.0 && x->xv > p->xend)
    {
       x->xv = p->xend;
    }
    if(x->xv < 0.0)
    {
        x->xv = 0.0;
    }

}

void BasicPD(struct States * s, struct ControlParams * p)
{
    /*
    if(s->xv > p->xend) {                                                                         
        s->cmd = p->kv*(-s->dx) + p->kp*(p->xend - s->x);
    }
    else if(s->xv < 0.0) {
        s->cmd = p->kv*(-s->dx) + p->kp*(0.0 - s->x);
    } else {
        s->cmd = s->ddxv + p->kv*(s->dxv - s->dx) + p->kp*(s->xv - s->x);
    }
    */
    s->cmd = p->kv*(s->dxv - s->dx) + p->kp*(s->xv - s->x);
    //s->cmd = 2.5; //(p->kp*(s->x0-s->x) - p->kv*s->dx) + 2.5;
}

void ComputedTorque(struct States * s, struct ControlParams * p)
{
    s->cmd = p->m*(s->ddxv + (p->kv)*(s->dxv - s->dx) + (p->kp)*(s->xv - s->x)) + p->c*s->dx + s->Fext/431.0 + 2.5;
}

void ComputedTorqueImp(struct States * s, struct ControlParams * p)
{
    s->cmd = (p->m/p->Md)*(p->Md*s->ddx0 + p->Dd*(s->dx-s->dx0)+p->Kd*(s->x-s->x0)-s->Fext) + p->c*s->dx + s->Fext/431.0 + 2.5;
}

void PeriodicReset(struct States * s)
{
    s->xv_prev = s->x;
    s->dxv_prev = s->dx;
}

void GetCommand(struct States * s, struct ControlParams * p)
{
    /**
     * @brief determines CUIC command based on mixing parameter 
     * @param[in] *s : pointer to robot States to store results
     * @param[in] *p : pointer to robot Params containing impedance values
     */
    
    s->xstar = s->ddxv + ((p->alpha)*p->kv + (1.0-p->alpha)*(p->Dd/p->Md))*(s->dxv - s->dx) + ((p->alpha)*p->kp + (1.0-p->alpha)*(p->Kd/p->Md))*(s->xv - s->x);
    s->cmd = p->m*s->xstar + p->c*s->dx - s->Fext;
}