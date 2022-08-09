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
    
    s->xv = p->Ad[1]*s->xv_prev + p->Ad[2]*s->dxv_prev + p->Bd[1]*s->Fext;
    s->dxv = p->Ad[3]*s->xv_prev + p->Ad[4]*s->dxv_prev + p->Bd[2]*s->Fext;
    s->ddxv = s->Fext - (p->Dd)/(p->Md)*(s->dxv - s->dx0) - (p->Kd)/(p->Md)*(s->xv - s->x0);

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