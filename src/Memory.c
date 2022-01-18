/**
 * @file Memory.c
 * @author Nick Berezny
 * @date 18 Jan 2022
 * @Controller thread 
 *
 */

#include <stdio.h>
#include <stdbool.h>
#include <pthread.h>

#include "./include/Memory.h"

int initMutex(pthread_mutex_t * lock)
{
    return pthread_mutex_init(lock, NULL); 
}