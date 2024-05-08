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

    extern struct States *s_client; 
    extern struct CommData *commData;
    extern struct ControlParams *controlParams;
    extern int iter_client;
    extern char buffer[2048];
    extern quitThreads;

    printf("Starting client thread...\n");

    iter_client = 0;

    while(!quitThreads)
    {
        s_client = &((struct States*)d)[iter_client];
        pthread_mutex_lock(&s_client->lock);
        if(iter_client == 3)
        {

            //sprintf(buffer, "PLOT::%.2f::%.2f", s_client->x, s_client->x0);
            if(controlParams->x0_is_percent)
            {
                sprintf(buffer, "PLOT::%.4f::%.4f::%.2f::%.2f::%.2f::%.2f::%.2f::%.2f::%.2f::", s_client->x/controlParams->xend, 
                    s_client->x0_to_send,s_client->x0_duration,s_client->Fext,
                    s_client->emg1,s_client->emg2,s_client->emg3,s_client->emg4, s_client->xGyro);
            }
            else
            {
                sprintf(buffer, "PLOT::%.4f::%.4f::%.2f::%.2f::%.2f::%.2f::%.2f::%.2f::%.2F::", s_client->x/controlParams->xend, s_client->x0_to_send/controlParams->xend,
                    s_client->x0_duration,s_client->Fext,
                    s_client->emg1,s_client->emg2,s_client->emg3,s_client->emg4, s_client->xGyro); 
                    
               // double xtest = 1.0;
                //sprintf(buffer, "PLOT::%.2f::%.2f::%.2f::%.2f::%.2f::%.2f::%.2f::%.2f::%.2f::",xtest,xtest + 1.0,xtest + 2.0,xtest+3.0,xtest+4.0,xtest,xtest,xtest,xtest);
            }
            sendMessage(commData->sockfd, buffer);
        }
        
        pthread_mutex_unlock(&s_client->lock);
        iter_client = iter_client + 1;
        if(iter_client == BUFFER_SIZE) iter_client = 0;
        
    }
    
    pthread_mutex_unlock(&s_client->lock);
    printf("Done Client...\n");
    //pthread_exit(NULL);
    return NULL;
}




