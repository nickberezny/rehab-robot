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

#include "./include/Structures.h"

#include "./include/Controller.h"
#include "./include/Daq.h"
#include "./include/Memory.h"
#include "./include/Threads.h"




int main(int argc, char* argv[]) 
{
    printf("Starting controller...\n");

    struct CUICStruct data = {0};

    struct sched_param param[NUMBER_OF_THREADS];
    pthread_attr_t attr[NUMBER_OF_THREADS];
    pthread_t thread[NUMBER_OF_THREADS];

    //init Daq
    //init Threads
    //init Mutexes
    //Data log etc...

    //initDaq();

    for(int i = 0; i < NUMBER_OF_THREADS; i++)
    {
        initThread(&attr[i], &param[i], 98-i);
        initMutex(&data.lock);
    }

    //....

    initThread(&attr[0], &param[0], 98);

    pthread_create(&thread[0], &attr[0], controllerThread, (void *)&data);
    pthread_join(thread[0], NULL);



}