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
#include "./include/Controller.h"
#include "./include/TimeUtilities.h"

void * controllerThread (void * d)
{
    printf("Starting control thread...\n");

    struct CUICStruct *cont; // = (struct CUICStruct*)d;
    int j = 0;

    struct timespec t_last;
    clock_gettime(CLOCK_MONOTONIC, &t_last);  

    while(j < BUFFER_SIZE)
    {

        cont = &((struct CUICStruct*)d)[j];

        clock_gettime(CLOCK_MONOTONIC, &cont->t_start);  
        
        printf("time: %d ; %d\n", cont->t_start.tv_sec - t_last.tv_sec, cont->t_start.tv_nsec - t_last.tv_nsec);

        pthread_mutex_lock(&cont->lock);

        //** do control stuff ...
        pthread_mutex_unlock(&cont->lock);
        t_last = cont->t_start;
        j = j + 1;

        getTimeToSleep(&cont->t_start, &cont->t_end);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &cont->t_end, NULL);

        
    }

    
}