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

void VirtualTrajectory(struct States * s, struct Params * p)
{
    /**
     * @brief determines next step of virtual trajectory according to desired impedance
     * @param[in] *s : pointer to robot States to store results
     * @param[in] *p : pointer to robot Params containing impedance values
     */
    

    s->ddxv = (1/(p->Md))*(s->Fext - p->Bd*(s->dxv - s->dx0) - p->Kd*(s->xv - s->x0)) + s->ddx0;
    s->dxv += s->ddxv*s->dt;
    s->xv += s->dxv*s->dt;
}

void GetCommand(struct States * s, struct Params * p)
{
    /**
     * @brief determines CUIC command based on mixing parameter 
     * @param[in] *s : pointer to robot States to store results
     * @param[in] *p : pointer to robot Params containing impedance values
     */
    
    s->xstar = s->ddxv + ((p->alpha)*p->kv + (1-p->alpha)*(p->Bd/p->Md))*(s->dxv - s->dx) + ((p->alpha)*p->kp + (1-p->alpha)*(p->Kd/p->Md))*(s->xv - s->x);
    s->cmd = p->m*s->xstar + p->c*s->dx - s->Fext;
}