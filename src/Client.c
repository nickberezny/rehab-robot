/**
 * @file Client.c
 * @author Nick Berezny
 * @date 13 Apr 2022
 * @brief Contains client thread for sending data to TCP server during robot operation.
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
    /**
     * @brief ClientThread function to be run in POSIX thread
     * @param[in] *d : pointer to robot States structure
     */

    struct States *s; 
    int i = 0;
    int j = 0;
    char msg[2048];

    while(true)
    {

        s = &((struct States*)d)[i];
        printf("%d mutex: %d\n", i, pthread_mutex_lock(&s->lock));
        if(i == 0)
        {
            j = j +1;
            sprintf(msg, "UI::%d", j);
            printf("%d\n", *(s->sockfd));
            //send(*(s->sockfd), msg, strlen(msg),0);
            sendMessage(s->sockfd, msg);
        }
        
        printf("%d unlock mutex: %d\n", i, pthread_mutex_unlock(&s->lock));
        i = i + 1;
        if(i == BUFFER_SIZE) i = 0;
        
    }
    
    printf("Done Client...\n");
    //pthread_exit(NULL);
    return NULL;
}




