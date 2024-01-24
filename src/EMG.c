/**
 * @file EMG.c
 * @author Nick Berezny
 * @date 23 Jan 2024
 * @brief Processing EMG
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
#include "./include/EMG.h"



void FilterEMG(double * emg, double * emg_filtered, double * emgVec, int size)
{

	*emg_filtered = 0;

	//rectify 
	for(int i = 0; i < size-1; i++)
	{
		emgVec[i] = emgVec[i+1];
		*emg_filtered = *emg_filtered + emgVec[i];
	}

	emgVec[size-1] = fabs(*emg);
	*emg_filtered = (*emg_filtered + emgVec[size-1])/((double)size);

	return;
}


