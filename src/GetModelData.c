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

	*nextPos = (*xend)*(sin((*freq)*(*time))+1.0) + (*xstart);

}

