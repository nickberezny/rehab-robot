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

#include "./include/Parameters.h"
#include "./include/Structures.h"
#include "./include/Memory.h"

int initMutex(struct States * s)
{
    return pthread_mutex_init(&s->lock, NULL); 
}