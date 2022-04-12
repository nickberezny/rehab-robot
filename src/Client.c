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
#include "./include/Communication.h"
#include "./include/Client.h"

void * clientThread (void * d)
{
    //sleep(0.001f);
    struct States *s; // = (struct CUICStruct*)d;
    int i = 0;
    char msg[2048];

    while(i < BUFFER_SIZE-1)
    {

        s = &((struct States*)d)[i];
        printf("%d mutex: %d\n", i, pthread_mutex_lock(&s->lock));
        sprintf(msg, "msg::%d\n", i);
        printf("%d\n", *(s->sockfd));
        //send(*(s->sockfd), msg, strlen(msg),0);
        sendMessage(s->sockfd, "hi there");
        printf("%d unlock mutex: %d\n", i, pthread_mutex_unlock(&s->lock));
        i = i + 1;
        
    }
    
    printf("Done Client...\n");
    //pthread_exit(NULL);
    return NULL;
}




