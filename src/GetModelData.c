/**
 * @file GetModelData.c
 * @author Nick Berezny
 * @date 5 Aug 2022
 * @brief Functions for CUIC [insert link to paper]
 *
 */


#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>
#include <sched.h>
#include <stdbool.h>
#include <limits.h>
#include <math.h>

#include "./include/Parameters.h"
#include "./include/Structures.h"
#include "./include/CUIC.h"
#include "./include/TimeUtilities.h"

void GetSineTrajectory(double * time, double * freq, double * xstart, double * xend, double * nextPos)
{

	*nextPos = (*xend/2.0)*(-cos((*freq)*(*time))+1.0) + (*xstart);

}

void GetSineCommand(double *time, double * freq, double *amplitude, double * command)
{

    *command = (*amplitude)*sin((*freq)*(*time)) + 2.5;

}

void RunCommandSteps(struct States * s, double *highBound, double *lowBound, double *offset, double *offset_iter, double *dir)
{
    if(s->x > *highBound && *dir == 1.0) 
    {
        *dir = -1.0; 

    }
    if(s->x < *lowBound && *dir == -1.0)
    {
        *dir = 1.0; 
        *offset += *offset_iter;
    }
}

