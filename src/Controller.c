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

#include "./include/Structures.h"
#include "./include/Controller.h"

void * controllerThread (void * d)
{

    struct CUICStruct *cont = (struct CUICStruct*)d;
    pthread_mutex_lock(&cont->lock);

    printf("Starting control thread...\n");

    clock_gettime(CLOCK_MONOTONIC, &cont->t_start); 

    //printf("time: %d ; %d\n", cont->t_start.tv_sec, cont->t_start.tv_nsec);
}