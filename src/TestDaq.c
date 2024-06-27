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
#include "./include/Home.h"
#include "./include/ForceSensor.h"


struct DAQ *daq;
struct ForceSensorData *fdata;
struct ControlParams *controlParams;
double x = 0;

int main(int aFextrgc, char* argv[]) 
{

	
	fdata = calloc(1,sizeof *fdata);

	long prev_t = 0;
	int ii = 0;
	
	daq = calloc(1,sizeof *daq);
	controlParams = calloc(17, sizeof *controlParams);
	struct States s[1] = {0};
	
	daq->numChannels = 7;
	initDaq(daq);
	daq->fdata = fdata; //need this after init daq????

	initForceSensorUDP(daq->fdata);

	printf("Time, Force, x, LSF, LSB\n");
	//
	clock_gettime(CLOCK_MONOTONIC, &controlParams->t_first);  
	s->x = 0.0;
	sleep(2);
	tareForceSensor(daq->fdata);
	sleep(2);
	//tareForceSensor(daq->fdata);
	//sleep(1);
	//tareForceSensor(daq->fdata);

	startForceSensorStream(daq->fdata);
	printf("Time, Force, x, LSF, LSB\n");


	while(1)
	{

		//read + print FT, ENC, LS
		s->cmd = 2.45;
		daq->aValues[1] = s->cmd;
		ReadWriteDAQ(s, daq);
		//readFroceSensor(daq->fdata);
		s->x += s->dx*(STEP_SIZE_MS/1000.0);
		s->Fext = daq->fdata->F[2];

	
		
		printf("%f, %f, %d, %d\n", s->x, s->Fext, s->lsb, s->lsf);
		//printf("%f, %f\n", s->x,s->Text);

        ii = ii + 1;
		usleep(1000);
	}
	
}
