/**
 * @file Memory.c
 * @author Nick Berezny
 * @date 13 Apr 2022
 * @brief functions for memory management
 *
 */

#include <stdio.h>
#include <stdbool.h>
#include <pthread.h>
#include <sys/mman.h>

#include "./include/Parameters.h"
#include "./include/Structures.h"
#include "./include/Memory.h"

int initMutex(struct States * s)
{
    pthread_mutexattr_t *attr;
    pthread_mutexattr_settype(attr, PTHREAD_MUTEX_RECURSIVE_NP);
    return pthread_mutex_init(&s->lock, attr); 

}

void lockMemory()
{
    if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
       printf("mlockall failed: %m\n");
       return;
    }
}

void unlockMemory()
{
    if(munlockall()) {
       printf("munlockall failed: %m\n");
       return;
    }
}