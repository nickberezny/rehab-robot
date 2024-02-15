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
#include "./include/Daq.h"


struct DAQ *daq;
struct ControlParams *controlParams;
double x = 0;

int main(int aFextrgc, char* argv[]) 
{

	//connect to daq
	daq = calloc(6,sizeof *daq);
	controlParams = calloc(17, sizeof *controlParams);
	struct States s[BUFFER_SIZE] = {0};
	daq->numChannels = 9;
	initDaq(daq);
	printf("Time, Force, x, LSF, LSB\n");
	clock_gettime(CLOCK_MONOTONIC, &controlParams->t_first);  
	s->x = 0.0;

	while(1)
	{
		//read + print FT, ENC, LS
		clock_gettime(CLOCK_MONOTONIC, &s->t_start);
		getElapsedTime(&controlParams->t_first, &s->t_start, &s->dt);  
		//timeStep(struct timespec * ts, struct timespec * tf, int * dt);
		ReadWriteDAQ(s, daq);
		s->x += s->dx;

		printf("Encoder: %.5f\n", s->dx);

		getTimeToSleep(&s->t_start, &s->t_end);
        //clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &s->t_end, NULL);
		usleep(10000);
	}

}
