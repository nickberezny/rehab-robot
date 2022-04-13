/**
 * @file Controller.c
 * @author Nick Berezny
 * @date 13 Apr 2022
 * @brief Contains controller thread for real-time control of robot
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>
#include <sched.h>
#include <stdbool.h>
#include <limits.h>

#include "./include/Parameters.h"
#include "./include/Structures.h"
#include "./include/TimeUtilities.h"
#include "./include/Controller.h"
#include "./include/CUIC.h"
#include "./include/Daq.h"


void * controllerThread (void * d)
{
    /**
     * @brief ControllerThread function to be run in POSIX thread
     * @param[in] *d : pointer to robot States structure
     */
    
    printf("Starting control thread...\n");
    struct States *s_next; // = (struct CUICStruct*)d;
    s_next = &((struct States*)d)[0];

    pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);

    //pthread_mutex_init(&s_next->lock, NULL);
    printf("x val: %f\n",s_next->x);

    printf("mutex cont %d\n",pthread_mutex_lock(&s_next->lock));

    int i = 0;
    

    struct States *s; // = (struct CUICStruct*)d;
    struct Params *p;
   
    struct timespec t_last;
    clock_gettime(CLOCK_MONOTONIC, &t_last);  


    //while(i < BUFFER_SIZE-1)
    while(true)
    {
        s = &((struct States*)d)[i];
        i = i + 1;
        if(i == BUFFER_SIZE) i = 0;
        s_next = &((struct States*)d)[i+1];

        p = &(s->p);

        clock_gettime(CLOCK_MONOTONIC, &s->t_start);  
        
        printf("time: %d ; %d\n", s->t_start.tv_sec - t_last.tv_sec, s->t_start.tv_nsec - t_last.tv_nsec);

        //read daq 
        //ReadWriteDAQ(s);

        //VirtualTrajectory(s,p);
        //GetCommand(s,p);    

        s->x = i;
        
        printf("%d mutex cont %d\n",i,pthread_mutex_lock(&s_next->lock));
        printf("%d unlock mutex cont %d\n",i, pthread_mutex_unlock(&s->lock));
        t_last = s->t_start;
        
        getTimeToSleep(&s->t_start, &s->t_end);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &s->t_end, NULL);

        
    }



    pthread_mutex_unlock(&s->lock);
    pthread_mutex_unlock(&s_next->lock);
    printf("Done Controller...\n");
    return NULL;
}