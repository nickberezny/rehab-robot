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
#include "./include/PreRun.h"

void * preRunThread (void * d)
{

	extern struct preRunStates *ps; // = (struct CUICStruct*)d;
    ps = &((struct preRunStates*)d)[0];

    extern struct DAQ *daq;

	clock_gettime(CLOCK_MONOTONIC, &ps->t_first);  

	while(true)
	{
		clock_gettime(CLOCK_MONOTONIC, &ps->t_start);          
        getElapsedTime(&ps->t_first, &ps->t_start, &ps->t);
		//read daq

		//run cmd if goto
		daq->aValues[0] = 0.0;
		ReadWriteDAQ(ps, daq);
		ps->x = ps->x + ps->dx*(STEP_SIZE_MS/1000.0);

		printf("Record x\n");

		getTimeToSleep(&ps->t_start, &ps->t_end);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ps->t_end, NULL);
	}


}