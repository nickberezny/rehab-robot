/**
 * @file Controller.c
 * @author Nick Berezny
 * @date 18 Jan 2022
 * @Controller thread 
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
#include "./include/Controller.h"
#include "./include/CUIC.h"


void * controllerThread (void * d)
{
    printf("Starting control thread...\n");

    struct States *s; // = (struct CUICStruct*)d;
    struct Params *p;
    int j = 0;

    struct timespec t_last;
    clock_gettime(CLOCK_MONOTONIC, &t_last);  

    while(j < BUFFER_SIZE)
    {

        s = &((struct States*)d)[j];
        p = &(s->p);


        clock_gettime(CLOCK_MONOTONIC, &s->t_start);  
        
        printf("time: %d ; %d\n", s->t_start.tv_sec - t_last.tv_sec, s->t_start.tv_nsec - t_last.tv_nsec);

        pthread_mutex_lock(&s->lock);

        //read daq 

        VirtualTrajectory(s,p);
        GetCommand(s,p);

        //write daq
        
        pthread_mutex_unlock(&s->lock);
        t_last = s->t_start;
        j = j + 1;

        getTimeToSleep(&s->t_start, &s->t_end);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &s->t_end, NULL);

        
    }

    
}