/**
 * @file TestDaq.c
 * @author Nick Berezny
 * @date 17 Feb 2023
 * @brief Prints values from DAQ every 0.5 seconds
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
#include "./include/Tensorflow.h"

struct ControlParams *controlParams;
struct tensorFlowVars *tensorflow;
double x = 0;

int main(int aFextrgc, char* argv[]) 
{

	tensorflow = calloc(1, sizeof *tensorflow);
	long prev_t = 0;
	int ii = 0;

	controlParams = calloc(17, sizeof *controlParams);
	struct States s[1] = {0};


	printf("Time, Force, x, LSF, LSB\n");
	//
	clock_gettime(CLOCK_MONOTONIC, &controlParams->t_first);  

	printf("Time, Force, x, LSF, LSB\n");

    controlParams->tensorflow = tensorflow;

    controlParams->tensorflow->InputSize = 8;
    controlParams->tensorflow->OutputSize = 2;

    double inputVals[8] = {0.0};
    controlParams->tensorflow->inputVals = inputVals;

    double outPutVals[2] = {0.0};
    controlParams->tensorflow->outputVals = outPutVals;

    initModel(controlParams->tensorflow);

    for(int i = 0; i<controlParams->tensorflow->NumInputs; i++)
    {
        controlParams->tensorflow->inputVals[i] = 0.1;
    }
    
    runModel(controlParams->tensorflow); //run once to init


	while(1)
	{
        controlParams->tensorflow->inputVals[0] = (double)ii+1;
        controlParams->tensorflow->inputVals[1] = (double)ii+1;
        controlParams->tensorflow->inputVals[2] = (double)ii+1;
        controlParams->tensorflow->inputVals[3] = (double)ii+1;
        controlParams->tensorflow->inputVals[4] = (double)ii+1;
        controlParams->tensorflow->inputVals[5] = (double)ii+1;
        controlParams->tensorflow->inputVals[6] = (double)ii+1;
        controlParams->tensorflow->inputVals[7] = (double)ii+1;
        runModel(controlParams->tensorflow);
		printf("%f,%f\n", controlParams->tensorflow->outputVals[0],controlParams->tensorflow->outputVals[1] );
        ii = ii + 10;
		usleep(700);
	}
	
}
