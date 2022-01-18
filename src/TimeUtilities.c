/**
 * @file TimeUtilities.c
 * @author Nick Berezny
 * @date 18 Jan 2022
 * @Controller thread 
 *
 */

#include <stdio.h>
#include <stdbool.h>
#include <pthread.h>

#include "./include/Parameters.h"
#include "./include/TimeUtilities.h"

int timeStep(struct timespec * ts, struct timespec * tf)
{
    return (tf->tv_sec - ts->tv_sec)*NSEC_IN_SEC + (tf->tv_nsec - ts->tv_nsec);;
}


