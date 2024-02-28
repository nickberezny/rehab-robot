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
#include "./include/Tensorflow.h"

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

}

void VirtualTrajectoryZeroStiffness(struct States * s, struct ControlParams * p)
{
    s->xv = p->Azero[0]*(s->xv_prev-s->x0) + p->Azero[1]*s->dxv_prev + p->Bd[0]*s->Fext + s->x0 ;
    s->dxv = p->Azero[2]*(s->xv_prev-s->x0) + p->Azero[3]*s->dxv_prev + p->Bd[1]*s->Fext;
    s->ddxv = (s->Fext/p->Md - (p->Dd)/(p->Md)*(s->dxv));///(STEP_SIZE_MS*1000.0); 

}

void PeriodicReset(struct States * s)
{
    s->xv_prev = s->x;
    s->dxv_prev = s->dx;
}

void BasicPD(struct States * s, struct ControlParams * p)
{

    s->cmd = (p->kp*(s->x0-s->x) - p->kv*s->dx);
}

void BasicPDxv(struct States * s, struct ControlParams * p)
{

    s->cmd = (p->kp*(s->xv-s->x) - p->kv*(s->dx-s->dxv));
}

void ComputedTorque(struct States * s, struct ControlParams * p)
{
    s->cmd = p->m*(s->ddxv + (p->kv)*(s->dxv - s->dx) + (p->kp)*(s->xv - s->x)) + p->c*s->dx + s->Fext/466.0;
    //printf("x0: %.2f\n", s->x0);
    //if(p->useFriction) GetFriction(s, p);
}

void ComputedTorqueImp(struct States * s, struct ControlParams * p)
{
    s->cmd = (p->m/p->Md)*(p->Md*s->ddx0 - p->Dd*(s->dx-s->dx0) - p->Kd*(s->x-s->x0) + s->Fext) + p->c*s->dx - s->Fext/431.0;
    //if(p->useFriction) GetFriction(s, p);
}

void GetFriction(struct States * s, struct ControlParams * p)
{
    p->tensorflow->inputVals[0] = s->x;
    p->tensorflow->inputVals[1] = s->dx;
    p->tensorflow->inputVals[2] = s->Fext;

    runModel(p->tensorflow);
    s->cmd = s->cmd + p->tensorflow->outputVals[0];

}