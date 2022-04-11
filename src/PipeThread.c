/**
 * @file Log.c
 * @author Nick Berezny
 * @date 20 Jan 2022
 * @Data Logging init and thread 
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>
#include <sched.h>
#include <stdbool.h>
#include <limits.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "./include/Parameters.h"
#include "./include/Structures.h"
#include "./include/Controller.h"
#include "./include/TimeUtilities.h"

void * pipeThread (void * d)
{
    //sleep(0.001f);
    struct States *s; // = (struct CUICStruct*)d;
    int i = 0;
    char sendData[1024];
    printf("Pipe starting...\n");
    s = &((struct States*)d)[0];

    while(i < BUFFER_SIZE-1)
    {
        if(i == 0)
        {
            
            //printf("%d mutex: %d\n", i, pthread_mutex_lock(&s->lock));
            printf("Pipe!\n");
            //sprintf(sendData, "%f\n", 4.7);
            //write(s->fd_data, sendData, sizeof(sendData));

            //printf("%d unlock mutex: %d\n", i, pthread_mutex_unlock(&s->lock));
            sleep(0.01);
        }
        
        //i = i + 1;
        
    }
    
    printf("Done Pipe...\n");
    //pthread_exit(NULL);
    return NULL;
}
