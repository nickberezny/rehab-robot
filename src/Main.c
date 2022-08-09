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
#include <regex.h>

#include "./include/Parameters.h"
#include "./include/Structures.h"

#include "./include/Controller.h"
#include "./include/Daq.h"
#include "./include/Memory.h"
#include "./include/Threads.h"
#include "./include/Log.h"
#include "./include/Communication.h"
#include "./include/Client.h"


//global variables 
char buffer[2048];

struct States *s_client; 
struct States *s_log;
struct States *s;
struct States *s_next;

struct ControlParams *controlParams;
struct LogData *logData;
struct CommData *commData;
struct DAQ *daq;

int iter_client;
int iter_log;
int iter_cont;

struct regexMatch regex =
{
    .Md = "Md([0-9]*.[0-9]*),",
    .Dd = "Bd([0-9]*.[0-9]*),",
    .Kd = "Kd([0-9]*.[0-9]*),",
    .xstart = "x_start([0-9]*.[0-9]*),",
    .xend = "x_end([0-9]*.[0-9]*),",
    .x0 = "x0([0-9]*.[0-9]*),",
    .dx0 = "dx0([0-9]*.[0-9]*),"
} ; //regex matches

regex_t compiled;
regmatch_t matches[2];
char matchBuffer[100];

int main(int argc, char* argv[]) 
{

    printf("Starting Robot...\n");
    struct States data[BUFFER_SIZE] = {0};
    struct States *d = &data[0];

    controlParams = malloc(sizeof *controlParams);
    logData = malloc(sizeof *logData);
    commData = malloc(sizeof *commData);
    daq = malloc(sizeof *daq);

    pthread_t thread[NUMBER_OF_THREADS];
    memset (thread, 0, NUMBER_OF_THREADS * sizeof (pthread_t));
    pthread_attr_t attr[NUMBER_OF_THREADS];

    bool homed = false;
    bool calibrated = false;
    bool set = false;

    char sendData[1024];

    int sockfd;
    
    bzero(buffer, sizeof(buffer));
    struct sockaddr_in servaddr;
    int port = 5000;
    int len;

    openClientSocket(&sockfd, &servaddr, &port);
    controlParams->currentState = WAIT_STATE; //State = Set
    printf("Starting Robot...\n");
    initDaq(daq);

    for(int i = 0; i < BUFFER_SIZE; i++)
    {
        commData->sockfd = &sockfd;
    }

    sendMessage(commData->sockfd, "ROBOT");
    read(sockfd, buffer, sizeof(buffer));
    sleep(5);
    sendMessage(&sockfd, "UI::hi from robot");



    //***********Main Loop******************//

    while(true)
    {
        if(controlParams->currentState == WAIT_STATE && homed && set && calibrated) controlParams->currentState = READY_STATE;
        printf("Current state: %d\n", controlParams->currentState);

        switch (controlParams->currentState)
        {
            case WAIT_STATE:
                WaitForMsg(&sockfd, &controlParams->currentState);
                break;

            case HOME_STATE:
                printf("Starting Robot...\n");
                sendMessage(&sockfd, "UI::STARTTASK");
                printf("Starting Robot...\n");
                HomeToFront(d,daq);
                HomeToBack(d,daq);
                printf("Starting Robot...\n");
                sleep(2);
                sprintf(sendData, "UI::HOME");
                sendMessage(&sockfd, sendData);
                
                homed = true;
                controlParams->currentState = WAIT_STATE;
                break;

            case CALIBRATE_STATE:
                sendMessage(&sockfd, "UI::STARTTASK");
                //run fn
                sleep(2);
                sprintf(sendData, "UI::CALIBRATE");
                sendMessage(&sockfd, sendData);
                calibrated = true;
                controlParams->currentState = WAIT_STATE;
                break;

            case SET_STATE:
                sendMessage(&sockfd, "UI::STARTTASK");
                //run fn
                WaitForParamMsg(&sockfd);
                double Atemp[2][2] = {{0.0, 1.0},{-controlParams->Kd/-controlParams->Md, --controlParams->Dd/-controlParams->Md}};
                double A[2][2];
                DiscretizeMatrix(Atemp,A);
                controlParams->Ad = A;

                double Btemp[2] = {0.0, 1.0};
                double B[2];
                DicretizeInput(A, Atemp, Btemp, B);

                controlParams->Bd = B;

                printf("Ad: %f, %f, %f, %f\n",A[0][0],A[0][1],A[1][0],A[1][1]);
                printf("Bd: %f, %f\n",B[0],B[1]);

                sleep(2);
                sprintf(sendData, "UI::SET");
                sendMessage(&sockfd, sendData);
                set = true;
                controlParams->currentState = WAIT_STATE;
                break;

            case READY_STATE:
                //ready to run
                WaitForMsg(&sockfd, &controlParams->currentState);
                break;
            case RUN_STATE:
                controlParams->currentState = STOP_STATE;
                sleep(2);
                ReadyController(data, attr, thread,  argc, argv);
                RunController(data, thread, attr, &sockfd);
                break;
            case STOP_STATE:
                //stop
                //reset states ...
                pthread_cancel(thread[0]);
                pthread_cancel(thread[1]);
                pthread_cancel(thread[2]);
                sleep(2); 
                controlParams->currentState = WAIT_STATE;
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


void WaitForParamMsg(int *fd)
{


    while(true)
    {
        
        /* open, read, and display the message from the FIFO */
        //sprintf(buffer,"");
        bzero(buffer, sizeof(buffer));
        read(*fd, buffer, sizeof(buffer));

        if(strcmp(buffer, "") != 0) printf("Received (Main): %s\n", buffer);

        regcomp(&compiled, regex.Md, REG_EXTENDED);
        if(regexec(&compiled, buffer, 2, matches, 0)==0){
            sprintf(matchBuffer, "%.*s\n", matches[1].rm_eo-matches[1].rm_so,  buffer+matches[1].rm_so );
            sscanf(matchBuffer, "%lf", &(controlParams->Md));
            printf("Md %f\n",controlParams->Md);
        }

        regcomp(&compiled, regex.Dd, REG_EXTENDED);
        if(regexec(&compiled, buffer, 2, matches, 0)==0){
            sprintf(matchBuffer, "%.*s\n", matches[1].rm_eo-matches[1].rm_so,  buffer+matches[1].rm_so );
            sscanf(matchBuffer, "%lf", &(controlParams->Dd));
            printf("Dd %f\n",controlParams->Dd);
        }

        regcomp(&compiled, regex.Kd, REG_EXTENDED);
        if(regexec(&compiled, buffer, 2, matches, 0)==0){
            sprintf(matchBuffer, "%.*s\n", matches[1].rm_eo-matches[1].rm_so,  buffer+matches[1].rm_so );
            sscanf(matchBuffer, "%lf", &(controlParams->Kd));
            printf("Kd %f\n",controlParams->Kd);
        }


        goto MsgRec;
        break;

    }

    MsgRec: ;
    return;
}


void WaitForMsg(int *fd, int *state)
{


    while(true)
    {
        
        /* open, read, and display the message from the FIFO */
        //sprintf(buffer,"");
        bzero(buffer, sizeof(buffer));
        read(*fd, buffer, sizeof(buffer));

        if(strcmp(buffer, "") != 0) printf("Received (Main): %s\n", buffer);

        if(strcmp(buffer, "HOME") == 0)
        {
            if(*state == WAIT_STATE || *state == READY_STATE) *state = HOME_STATE;
            break;
        }
        else if(strcmp(buffer, "CALIBRATE") == 0)
        {
            if(*state == WAIT_STATE || *state == READY_STATE) *state = CALIBRATE_STATE;
            break;
        }
        else if(strcmp(buffer, "SET") == 0)
        {
            if(*state == WAIT_STATE || *state == READY_STATE) *state = SET_STATE;
            break;
        }
        else if(strcmp(buffer, "RUN") == 0)
        {
            if(*state == READY_STATE) *state = RUN_STATE;
            break;
        }
        else if(strcmp(buffer, "STOP") == 0)
        {
            if(*state == RUN_STATE) *state = STOP_STATE;
            break;
        }
        else if(strcmp(buffer, "SHUTDOWN") == 0)
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

    //*************Initialize Data log*******************
    
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
        initLog("/test.txt", logData, timeinfo);
    }

    printf("path: %s\n", logData->filepath);
    logData->fp = fopen(logData->filepath,"a");



    //fclose(data[0].h.fp);

    //*************Initialize threads + mutexes*******************
    
    for(int i = 0; i < NUMBER_OF_THREADS; i++)
    {
        printf("init: %d\n",initThread(&attr[i], &param[i], 95-i));
    }

    pthread_mutex_t temp_lock[BUFFER_SIZE];
    /*pthread_mutexattr_t mutex_attr;
    pthread_mutexattr_init(&mutex_attr);
    pthread_mutexattr_settype(&mutex_attr, PTHREAD_MUTEX_RECURSIVE_NP);*/

    for(int i = 0; i < BUFFER_SIZE; i++)
    {
        printf("%d mutex: %d\n", i, pthread_mutex_init(&data[i].lock, NULL));
    }

    //*************Lock Memory*******************

    lockMemory();
}


void RunController(struct States *data, pthread_t *thread, pthread_attr_t *attr, int * fd)
{

    printf("thread: %d\n",pthread_create(&thread[0], &attr[0], controllerThread, (void *)data));
    printf("thread: %d\n",pthread_create(&thread[1], &attr[1], logThread, (void *)data));
    //printf("thread: %d\n",pthread_create(&thread[2], &attr[2], clientThread, (void *)data));
    //pthread_join(thread[0], NULL);
    //pthread_join(thread[1], NULL);

    //pthread_cancel(thread[0]);
    //pthread_cancel(thread[1]);
    //pthread_cancel(thread[2]);

    WaitForMsg(commData->sockfd, &(controlParams->currentState));

    printf("Finished Threads...\n");

    return;
}