/**
 * @file CUIC.c
 * @author Nick Berezny
 * @date 2 Mar 2022
 * @Continuous Unified Interaction Control 
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
    s->ddxv = (1/(p->Md))*(s->Fext - p->Bd*(s->dxv - s->dx0) - p->Kd*(s->xv - s->x0)) + s->ddx0;
    s->dxv += s->ddxv*s->dt;
    s->xv += s->dxv*s->dt;
}

void GetCommand(struct States * s, struct Params * p)
{
    s->xstar = s->ddxv + ((p->alpha)*p->kv + (1-p->alpha)*(p->Bd/p->Md))*(s->dxv - s->dx) + ((p->alpha)*p->kp + (1-p->alpha)*(p->Kd/p->Md))*(s->xv - s->x);
    s->cmd = p->m*s->xstar + p->c*s->dx - s->Fext;
}