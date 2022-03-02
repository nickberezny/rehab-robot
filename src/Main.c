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
#include <string.h>

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "./include/Structures.h"

#include "./include/Controller.h"
#include "./include/Daq.h"
#include "./include/Memory.h"
#include "./include/Threads.h"
#include "./include/Log.h"
#include "./include/UI.h"
#include "./include/Communication.h"

int mainController(int argc, char* argv[]);


int main(int argc, char* argv[]) 
{
    int mypid = fork();

    if( 0 != mypid )
    {
        printf( "lol child\n" );
        mainUI(argc, argv);
    }
    else
    {
        printf( "lol parent\n" );
        mainController(argc, argv);
    }
        

    return( 0 );
}



int mainController(int argc, char* argv[])
{
    printf("Starting controller...\n");
    int fd;
    char buf[100];
    openComm(&fd);

    /* open, read, and display the message from the FIFO */
    
    int temp = read(fd, buf, 100);

    printf("Received: %s\n", buf);
    close(&fd);
    /* remove the FIFO */
    unlink("/tmp/myfifo");


    struct States data[BUFFER_SIZE] = {0};

    struct sched_param param[NUMBER_OF_THREADS];
    pthread_attr_t attr[NUMBER_OF_THREADS];
    pthread_t thread[NUMBER_OF_THREADS];

    //init Daq
    //init Threads
    //init Mutexes
    //Data log etc...

    if(argc > 1 && !strcmp(argv[1], "NOLOG"))
    {
        printf("Logging Deactivated...\n");
        
    }
    else
    {
        time_t rawtime;
        struct tm * timeinfo;

        time ( &rawtime );
        timeinfo = localtime ( &rawtime );

        FILE * fp;

        initLog("/test.txt", fp, timeinfo);
    }

    
    printf("Check\n");
    //initDaq();

    for(int i = 0; i < NUMBER_OF_THREADS; i++)
    {
        printf("init: %d\n",initThread(&attr[i], &param[i], 98-i));
    }

    for(int i = 0; i < BUFFER_SIZE; i++)
    {
        initMutex(&data[i].lock);
    }
    printf("Check\n");

    //....

    printf("thread: %d\n",pthread_create(&thread[0], &attr[0], controllerThread, (void *)&data));
    pthread_join(thread[0], NULL);

}