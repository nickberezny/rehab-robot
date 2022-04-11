/**
 * @file Main.c
 * @author Nick Berezny
 * @date 18 Jan 2022
 * @Controller thread 
 *
 */
#include "./include/Parameters.h"

#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>
#include <sched.h>
#include <stdbool.h>
#include <limits.h>
#include <string.h>

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <signal.h>

#include "./include/Parameters.h"
#include "./include/Structures.h"
#include "./include/TimeUtilities.h"
#include "./include/Daq.h"
#include "./include/Log.h"
#include "./include/Home.h"


int main(int argc, char* argv[]) 
{

    printf("Starting force sensor...\n");
    ReadForceSensor(1000);

}

void ReadForceSensor(int numOfSamples)
{

    struct States s = {0};

    initDaq(&s);
    
    double aValues[1] = {0};
    const char * aNames[1];
    int aNumValues[1];
    int aWrites[1];
    int errorAddress;

    aNames[0] = "AIN0";
    aNumValues[0] = 1;
    aWrites[0] = 0;
    errorAddress = 0;

    time_t rawtime;
    struct tm * timeinfo;
    time ( &rawtime );
    timeinfo = localtime ( &rawtime );

    initLog("/ForceSensor.txt", &s, timeinfo);

    s.h.fp = fopen(s.h.filepath,"a");

    fprintf ((s.h.fp), "t(s),t(ns),f\n");

    int i = 0;

    struct timespec t_last;
    clock_gettime(CLOCK_MONOTONIC, &t_last);  

    while(i < numOfSamples)
    {


        clock_gettime(CLOCK_MONOTONIC, &s.t_start);  
        
        //if(i==0) printf("time: %d ; %d\n", s->t_start.tv_sec - t_last.tv_sec, s->t_start.tv_nsec - t_last.tv_nsec);
        LJM_eNames(s.daq.daqHandle, 1, aNames, aWrites, aNumValues, aValues, &(errorAddress));

        fprintf ((s.h.fp), "%d, %d, %.3f \n", s.t_start.tv_sec, s.t_start.tv_nsec, aValues[0]);

        t_last = s.t_start;
        
        getTimeToSleep(&s.t_start, &s.t_end);

        i = i + 1;

        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &s.t_end, NULL);

    }

    fclose(s.h.fp);

}

void Home()
{   
    struct States s = {0};
    initDaq(&s);
    HomeToBack(&s);
    HomeToFront(&s);
}