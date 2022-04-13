/**
 * @file Main.c
 * @author Nick Berezny
 * @date 18 Jan 2022
 * @brief entry point for rehab-robot-controller
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
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <unistd.h>
#include <signal.h>
#include <sys/resource.h>

#include "./include/Parameters.h"
#include "./include/Structures.h"

#include "./include/Controller.h"
#include "./include/Daq.h"
#include "./include/Memory.h"
#include "./include/Threads.h"
#include "./include/Log.h"
#include "./include/Communication.h"
#include "./include/Client.h"



int mainController(int argc, char* argv[]);



int main(int argc, char* argv[]) 
{
    printf("Starting Robot...\n");

    struct States data[BUFFER_SIZE] = {0};
    struct States *d = &data[0];
    d->currentState = 0; 

    pthread_t thread[NUMBER_OF_THREADS];
    pthread_attr_t attr[NUMBER_OF_THREADS];

    bool homed = false;
    bool calibrated = false;
    bool set = false;

    char sendData[1024];

    int sockfd;
    char buffer[2048];
    bzero(buffer, sizeof(buffer));
    struct sockaddr_in servaddr;
    int port = 5000;
    int len;

    openClientSocket(&sockfd, &servaddr, &port);


    d->currentState = WAIT_STATE; //State = Set

    for(int i = 0; i < BUFFER_SIZE; i++)
    {
        data[i].sockfd = &sockfd;
    }

    sendMessage(d->sockfd, "ROBOT");
    read(sockfd, buffer, sizeof(buffer));

    printf("Server : %s\n", buffer);

    printf("Hello message sent.\n");
    sleep(5);


    sendMessage(&sockfd, "UI::hi from robot");

    /*
    bool isRunning = false;

    while(!isRunning)
    {
        bzero(buffer, sizeof(buffer));
        read(sockfd, buffer, sizeof(buffer));
        printf("Server : %s\n", buffer);

        if(strcmp(buffer, "STOP") == 0) isRunning = true;
        else if(strcmp(buffer, "HOME") == 0 || strcmp(buffer, "SET") == 0 || strcmp(buffer, "CALIBRATE") == 0)
        {
            sendMessage(&sockfd, "UI::STARTTASK", &servaddr);
            sleep(2);
            sprintf(sendData, "UI::");
            strcat(sendData, buffer);
            printf("%s, %s\n", buffer, sendData);
            sendMessage(&sockfd, sendData, &servaddr);
        }
    }
*/
    

    //RUN CAL .........


    //***********Main Loop******************//

    while(true)
    {
        if(d->currentState == WAIT_STATE && homed && set && calibrated) d->currentState = READY_STATE;
        printf("Current state: %d\n", d->currentState);

        switch (d->currentState)
        {
            case WAIT_STATE:
                WaitForMsg(&sockfd, &d->currentState);
                break;
            case HOME_STATE:
                sendMessage(&sockfd, "UI::STARTTASK");
                //run fn
                sleep(2);
                sprintf(sendData, "UI::HOME");
                sendMessage(&sockfd, sendData);
                homed = true;
                d->currentState = WAIT_STATE;
                break;
            case CALIBRATE_STATE:
                sendMessage(&sockfd, "UI::STARTTASK");
                //run fn
                sleep(2);
                sprintf(sendData, "UI::CALIBRATE");
                sendMessage(&sockfd, sendData);
                calibrated = true;
                d->currentState = WAIT_STATE;
                break;
            case SET_STATE:
                sendMessage(&sockfd, "UI::STARTTASK");
                //run fn
                sleep(2);
                sprintf(sendData, "UI::SET");
                sendMessage(&sockfd, sendData);
                set = true;
                d->currentState = WAIT_STATE;
                break;
            case READY_STATE:
                //ready to run
                WaitForMsg(&sockfd, &d->currentState);
                break;
            case RUN_STATE:
                d->currentState = STOP_STATE;
                sleep(2);
                ReadyController(data, attr, thread,  argc, argv);
                RunController(data, thread, attr, &sockfd);
                break;
            case STOP_STATE:
                //stop
                //reset states ...
                pthread_cancel(thread[0]);
                //pthread_cancel(thread[1]);
                pthread_cancel(thread[2]);
                sleep(2); 
                d->currentState = WAIT_STATE;
                break; 
            case SHUTDOWN_STATE:
                sleep(0.5);
                //stop
                //reset states ... 
                printf("Shutting down now...\n");
                goto EndWhile;
                break;
            default:
                goto EndWhile;
                break;
        }



    }
    
    EndWhile: ;
 
    printf("ENDING!\n");
    close(sockfd);
    
    return 0;

}



void WaitForMsg(int *fd, int *state)
{
    char buf[1024];
    int * len;

    while(true)
    {
        
        /* open, read, and display the message from the FIFO */
        //sprintf(buf,"");
        bzero(buf, sizeof(buf));
        read(*fd, buf, sizeof(buf));

        if(strcmp(buf, "") != 0) printf("Received (Main): %s\n", buf);

        if(strcmp(buf, "HOME") == 0)
        {
            if(*state == WAIT_STATE || *state == READY_STATE) *state = HOME_STATE;
            break;
        }
        else if(strcmp(buf, "CALIBRATE") == 0)
        {
            if(*state == WAIT_STATE || *state == READY_STATE) *state = CALIBRATE_STATE;
            break;
        }
        else if(strcmp(buf, "SET") == 0)
        {
            if(*state == WAIT_STATE || *state == READY_STATE) *state = SET_STATE;
            break;
        }
        else if(strcmp(buf, "RUN") == 0)
        {
            if(*state == READY_STATE) *state = RUN_STATE;
            break;
        }
        else if(strcmp(buf, "STOP") == 0)
        {
            if(*state == RUN_STATE) *state = STOP_STATE;
            break;
        }
        else if(strcmp(buf, "SHUTDOWN") == 0)
        {
            printf("Shutting down...\n");
            *state = SHUTDOWN_STATE;
            goto MsgRec;
            break;
        }
    }

    MsgRec: ;

    return;
}


void ReadyController(struct States * data, pthread_attr_t *attr, pthread_t *thread, int argc, char* argv[])
{
    struct sched_param param[NUMBER_OF_THREADS];
    
    if(argc > 1 && !strcmp(argv[1], "NOLOG"))
    {
        printf("Logging Deactivated...\n");
        
    }
    else
    {

        printf("Logging Activated...\n");

        time_t rawtime;
        struct tm * timeinfo;

        time ( &rawtime );
        timeinfo = localtime ( &rawtime );

        //FILE * fp;

        initLog("/test.txt", &data[0], timeinfo);
    }

    printf("path: %s\n", data[0].h.filepath);
    data[0].h.fp = fopen(data[0].h.filepath,"a");

    for(int i = 1; i < BUFFER_SIZE; i++)
    {
        data[i].h.fp = data[0].h.fp;
    }

    fclose(data[0].h.fp);

    
    for(int i = 0; i < NUMBER_OF_THREADS; i++)
    {
        printf("init: %d\n",initThread(&attr[i], &param[i], 98-i));
    }

    pthread_mutex_t temp_lock[BUFFER_SIZE];

    for(int i = 0; i < BUFFER_SIZE; i++)
    {
        printf("%d mutex: %d\n", i, pthread_mutex_init(&data[i].lock, NULL));
    }

    data[0].x = 4;
    data[1].x = 4;
    //....

    
}


void RunController(struct States *data, pthread_t *thread, pthread_attr_t *attr, int * fd)
{

    printf("x test: %f\n",data[0].x);

    printf("thread: %d\n",pthread_create(&thread[0], &attr[0], controllerThread, (void *)data));
    //printf("thread: %d\n",pthread_create(&thread[1], &attr[1], logThread, (void *)data));
    printf("thread: %d\n",pthread_create(&thread[2], &attr[2], clientThread, (void *)data));
    //pthread_join(thread[0], NULL);
    //pthread_join(thread[1], NULL);

    //pthread_cancel(thread[0]);
    //pthread_cancel(thread[1]);
    //pthread_cancel(thread[2]);

    WaitForMsg(data[0].sockfd, &(data[0].currentState));

    printf("Finished Threads...\n");

    return;
}