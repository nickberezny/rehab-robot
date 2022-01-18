/**
 * @file Threads.c
 * @author Nick Berezny
 * @date 17 Jan 2022
 * @brief File containing example of doxygen usage for quick reference.
 *
 * Test test
 */


#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>
#include <sched.h>
#include <stdbool.h>
#include <limits.h>

#include "./include/Threads.h"


bool initThread(pthread_attr_t * attr, struct sched_param * param, int priority)
{

    /*------------------------------------------------------------------------
        Create a thread with priority and scheduler (set in imp_variables)
    ------------------------------------------------------------------------*/

	// Initialize pthread attributes (default values) 
    if (pthread_attr_init(attr)) {
        printf("init pthread attributes failed \n");
        return false;
    }

    // Set a specific stack size  
    if (pthread_attr_setstacksize(attr, PTHREAD_STACK_MIN)) {
        printf("pthread setstacksize failed \n");
        return false;
    }


    // Set scheduler policy and priority of pthread (SCHED_FIFO, SCHED_RR, SCHED_DEADLINE)
    if (pthread_attr_setschedpolicy(attr, SCHED_FIFO)) {
        printf("pthread setschedpolicy failed \n");
        return false;
    }
    param->sched_priority = priority; //priority (0-99)

    if (pthread_attr_setschedparam(attr, param)) {
        printf("pthread setschedparam failed \n");
        return false;
    }

    // Use scheduling parameters of attr 
    if (pthread_attr_setinheritsched(attr, PTHREAD_EXPLICIT_SCHED)) {
        printf("pthread setinheritsched failed \n");
        return false;
    }

    return true;

}