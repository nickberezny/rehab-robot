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
#include "./include/ForceSensor.h"


struct DAQ *daq;
struct ForceSensorData *fdata;
struct ControlParams *controlParams;
double x = 0;

int main(int aFextrgc, char* argv[]) 
{

	
	fdata = calloc(1,sizeof *fdata);
	initForceSensorUDP(fdata);

	long prev_t = 0;
/*
	
	for(int i=0;i<10000;i++)
	{
		readFroceSensor(fdata);
		printf("%f, %f, %f\n",fdata->F[0],fdata->F[1],fdata->F[2]);
		usleep(500);
	}
	*/
	

	//connect to daq
	daq = calloc(1,sizeof *daq);
	controlParams = calloc(17, sizeof *controlParams);
	struct States s[1] = {0};
	daq->numChannels = 9;
	initDaq(daq);
	daq->i2cAddr[0] = 0x68;
	printf("Time, Force, x, LSF, LSB\n");
	clock_gettime(CLOCK_MONOTONIC, &controlParams->t_first);  
	s->x = 0.0;

	while(1)
	{
		//read + print FT, ENC, LS
		clock_gettime(CLOCK_MONOTONIC, &s->t_start);
		getElapsedTime(&controlParams->t_first, &s->t_start, &s->dt);  
		//timeStep(struct timespec * ts, struct timespec * tf, int * dt);
		//ReadWriteDAQ(s, daq);
		//s->x += s->dx*(STEP_SIZE_MS/1000.0);
		//s->gonio = ((double)daq->aValues[8])*0.002618;
		//printf("Encoder: %.5f, Gonio: %.3f\n", s->x, s->gonio);
		//printf("%f, %f, %f\n",  s->accel[0],  s->accel[1],  s->accel[2]);
		//readFroceSensor(fdata);
		//printf("%d,%f, %f, %f\n",s->t_start.tv_nsec,fdata->F[0],fdata->F[1],fdata->F[2]);

		readI2C(s, daq, 0);

		if(s->t_start.tv_nsec-prev_t > 1200000)
		{
			printf("%d\n",s->t_start.tv_nsec-prev_t);
		}

		prev_t = s->t_start.tv_nsec;

		getTimeToSleep(&s->t_start, &s->t_end);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &s->t_end, NULL);
		//usleep(1000);
	}
	
}
