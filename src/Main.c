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
#include "./include/Tensorflow.h"
#include "./include/ReadProcessFile.h"



//global variables 
char buffer[4096];
char buffer_small[10];
char sessionPath[200] = "../ControllerSetup/";

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
int iter_reset;

double homeToBack;

bool quitThreads = false;

bool loggingActivated = true; 

struct regexMatch regex =
{
    .Md = "Md([0-9]*.[0-9]*),",
    .Dd = "Bd([0-9]*.[0-9]*),",
    .Kd = "Kd([0-9]*.[0-9]*),",
    .xstart = "x_start([0-9]*.[0-9]*),",
    .xend = "x_end([0-9]*.[0-9]*),",
    .x0 = "x0_([0-9]*.[0-9]*),",
    .dx0 = "dx0_([0-9]*.[0-9]*),",
    .alpha = "alpha([0-9]*.[0-9]*)",
    .delta = "delta([0-9]*)",
    .kv = "kv([0-9]*.[0-9]*)",
    .kp = "kp([0-9]*.[0-9]*)",
    .kp = "kp([0-9]*.[0-9]*)",
    .filename = "filename(*.txt)",
    .Home = "Home([0-9]*)",
    .mass = "mass([0-9]*.[0-9]*),",
    .damp = "damp([0-9]*.[0-9]*),",
    .controlMode = "controlMode([0-9]),",
    .trajectoryMode = "trajMode([0-9]),",
    .Fmax = "Fmax([0-9]*.[0-9]*),",
    .recordEMG = "recordEMG([0-9])",
    .phaseTime = "PhaseTime([0-9]*.[0-9]*)",
    .numPositions = "NumPositions([0-9])",
    .stochasticStepTime = "StochasticStepTime([0-9]*.[0-9]*)",
    .randomRate = "Rate([0-9]*.[0-9]*)",
    .amplitude = "Amplitude([0-9]*.[0-9]*)",
    .frequency = "Frequency([0-9]*.[0-9]*)",
    .offset = "Offset([0-9]*.[0-9]*)",
    .useFriction = "useFriction([0-9]*)",
} ; //regex matches

regex_t compiled;
regmatch_t matches[2];
char matchBuffer[100];
char folder[1000] = "log/";
int fileIteration = 0;

void GetParameterFloat(char *regex, double *param);
void GetParameterInt(char *regex, int *param);

int main(int argc, char* argv[]) 
{

   time_t t;
   srand((unsigned) time(&t));
 
    printf("Booting Robot...\n");
    struct States data[BUFFER_SIZE] = {0};
    struct States *d = &data[0];

    
    
    controlParams = calloc(17, sizeof *controlParams);
    logData = calloc(2, sizeof *logData);
    commData = calloc(1, sizeof *commData);
    daq = calloc(6,sizeof *daq);

    //Butterworth filter params
    controlParams->filter_a_100Hz[0] = 0.0; 
    controlParams->filter_a_100Hz[1] = -1.76004188;
    controlParams->filter_a_100Hz[2] = 1.182893262;
    controlParams->filter_a_100Hz[3] = -0.27805991;

    controlParams->filter_b_100Hz[0] = 0.018098933;
    controlParams->filter_b_100Hz[1] = 0.054296799;
    controlParams->filter_b_100Hz[2] = 0.054296799;
    controlParams->filter_b_100Hz[3] = 0.018098933;

    controlParams->filter_a_10Hz[0] = 0.0; 
    controlParams->filter_a_10Hz[1] = -2.8743569;
    controlParams->filter_a_10Hz[2] = 2.7564832;
    controlParams->filter_a_10Hz[3] = -0.8818931;

    controlParams->filter_b_10Hz[0] = 0.000029146;
    controlParams->filter_b_10Hz[1] = 0.000087439;
    controlParams->filter_b_10Hz[2] = 0.000087439;
    controlParams->filter_b_10Hz[3] = 0.000029146;
    for(int i = 0; i < FILTER_ORDER+1; i++)
    {
        controlParams->dx_filt_x[i] = 0.0;
        controlParams->dx_filt_y[i] = 0.0;
        controlParams->F_filt_x[i] = 0.0;
        controlParams->F_filt_y[i] = 0.0;
    }

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


    char * filename_path = "../ControllerSetup/S1";
    controlParams->t  = calloc(20, sizeof(*(controlParams->t)));
    controlParams->x  = calloc(20, sizeof(*(controlParams->x)));
    ReadSessionFiles(filename_path, controlParams);
    /*
    ReadControlFile(filename_path, controlParams);
    
    ReadProcessFile(filename_path, controlParams);
    ReadTrajectoryFile(filename_path, controlParams);
*/
    //initialize folder 
    
    time_t rawtime;
    struct tm * timeinfo;
    time ( &rawtime );
    timeinfo = localtime ( &rawtime );
    
    initFolder(timeinfo,folder);

    openClientSocket(&sockfd, &servaddr, &port);
    controlParams->currentState = WAIT_STATE; //State = Set
    printf("Starting Robot...\n");

    
    
    //*************Initialize Tensorflow Neural Net*******************



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
                
                sendMessage(&sockfd, "UI::STARTTASK::");
                
                if(homeToBack == 0)
                {
                    HomeToFront(d,daq);
                    HomeToBack(d,daq);
                }
                else if(homeToBack == 1)
                {
                    HomeToBack(d,daq);
                    controlParams->xend = 0.4;
                }
                
                
                sleep(2);
                sprintf(sendData, "UI::HOME");
                sendMessage(&sockfd, sendData);
                
                homed = true;
                controlParams->currentState = WAIT_STATE;
                break;

            case CALIBRATE_STATE:
                sendMessage(&sockfd, "UI::STARTTASK::");
                //run fn
                CalibrateForceOffset(d,daq);

                sleep(1);
                sprintf(sendData, "UI::CALIBRATE");
                sendMessage(&sockfd, sendData);
                calibrated = true;
                controlParams->currentState = WAIT_STATE;
                break;

            case SET_STATE:
                sendMessage(&sockfd, "UI::STARTTASK::");
                //run fn
                WaitForParamMsg(&sockfd);
                double Atemp[2][2] = {{0.0, 1.0},{-controlParams->Kd/controlParams->Md, -controlParams->Dd/controlParams->Md}};
                double A[2][2];
                DiscretizeMatrix(Atemp,A);
                controlParams->Ad = A;

                double Btemp[2] = {0.0, 1.0/controlParams->Md};
                double B[2];
                DicretizeInput(A, Atemp, Btemp, B);

                controlParams->Bd = B;

                printf("Ad: %f, %f, %f, %f\n",A[0][0],A[0][1],A[1][0],A[1][1]);
                printf("Bd: %f, %f\n",B[0],B[1]);

                controlParams->dx_bound = 0.01;
                controlParams->m = 0.858;//1.0/0.8041;
                controlParams->c = 0.35;//1.096/0.8041;

                if(controlParams->controlMode == UIC_MODE)
                {
                    controlParams->kv = controlParams->alpha*controlParams->kv + (1.0-controlParams->alpha)*(controlParams->Dd/controlParams->Md);
                    controlParams->kp = controlParams->alpha*controlParams->kp + (1.0-controlParams->alpha)*(controlParams->Kd/controlParams->Md);
                }

                printf("Gains: %f, %f\n", controlParams->kv,controlParams->kp);

                //*************Initialize Daq*******************

                daq->numChannels = 11;
                initDaq(daq);



                if(controlParams->useFriction)
                {
                    controlParams->tensorflow->NumInputs = 3;
                    controlParams->tensorflow->NumOutputs = 1;
                    initModel(controlParams->tensorflow);
                }
                    

                sleep(2);
                sprintf(sendData, "UI::SET");
                sendMessage(&sockfd, sendData);

                set = true;
                controlParams->currentState = WAIT_STATE;

                break;

            case READY_STATE:
                //ready to run
                quitThreads = false;
                WaitForMsg(&sockfd, &controlParams->currentState);
                break;
            case RUN_STATE:
                controlParams->currentState = STOP_STATE;
                sleep(2);
                ReadyController(data, attr, thread,  argc, argv);
                RunController(data, thread, attr, &sockfd);
                break;
            case STOP_STATE:
                quitThreads = true;
                for(int i = 0; i < BUFFER_SIZE; i++)
                {
                    pthread_mutex_unlock(&data[i].lock);
                }
                ResetController();
                homed = false;
                calibrated = false;
                set = false;
                sendMessage(commData->sockfd, "UI::STOP");
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

        GetParameterFloat(regex.Md, &(controlParams->Md));
        GetParameterFloat(regex.Dd, &(controlParams->Dd));
        GetParameterFloat(regex.Kd, &(controlParams->Kd));

        GetParameterFloat(regex.xstart, &(controlParams->xstart));
        GetParameterFloat(regex.x0, &(controlParams->x0));

        GetParameterFloat(regex.alpha, &(controlParams->alpha));
        GetParameterFloat(regex.delta, &(controlParams->delta));

        GetParameterFloat(regex.kp, &(controlParams->kp));
        GetParameterFloat(regex.kv, &(controlParams->kv));

        GetParameterFloat(regex.mass, &(controlParams->m));
        GetParameterFloat(regex.damp, &(controlParams->c));

        GetParameterFloat(regex.Fmax, &(controlParams->Fmax));
        GetParameterFloat(regex.phaseTime, &(controlParams->phaseTime));
        GetParameterInt(regex.numPositions, &(controlParams->numPositions));
        GetParameterFloat(regex.stochasticStepTime, &(controlParams->stochasticStepTime));
        GetParameterFloat(regex.randomRate, &(controlParams->randomRate));

        GetParameterFloat(regex.Home, &(homeToBack));
        GetParameterInt(regex.controlMode, &(controlParams->controlMode));
        GetParameterInt(regex.trajectoryMode, &(controlParams->trajectoryMode));
        GetParameterInt(regex.recordEMG, &(controlParams->recordEMG));

        GetParameterFloat(regex.frequency, &(controlParams->frequency));
        GetParameterFloat(regex.amplitude, &(controlParams->amplitude));
        GetParameterFloat(regex.offset, &(controlParams->offset));

        GetParameterInt(regex.useFriction, &(controlParams->useFriction));
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

        strncpy(buffer_small, buffer, 4);

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
        else if(strcmp(buffer_small, "STOP") == 0)
        {
            printf("STOP!\n");
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
        else if(strcmp(buffer, "NEXT") == 0)
        {
            printf("Next stochastic force run\n");
            if(controlParams->stochasticState==0)
                controlParams->stochasticState = 1;
            else
                printf("Error, already running\n");
        }
        else if(strncmp(buffer, "S_", 2) == 0)
        {

            //open process file
            printf("Path: %s\n", buffer);
            printf("Path: %s\n", &(buffer[2]));
            strcat(sessionPath,&(buffer[2]));
            printf("Path: %s\n", sessionPath);


            //then, read controllers and trajectories (get all in session folder)
            ReadSessionFiles(sessionPath);
        }
        else if(strncmp(buffer, "P_", 2) == 0)
        {

            //open process file
            char * filename_path = "../path/";
            strcat(filename_path,&(buffer[2]));
            ReadProcessFile(filename_path, controlParams);

            //then, read controllers and trajectories (get all in session folder)

           
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
        loggingActivated = false;
    }
    else
    {
        printf("Logging Activated...\n");
        fileIteration += 1;
        char filename[20] = "";
        sprintf(filename, "/run%d.txt",fileIteration);
        char foldercpy[1000] = ""; 
        strcpy(foldercpy, folder);
        initLog(filename, logData, foldercpy);
    }

    printf("path: %s\n", logData->filepath);
    logData->fp = fopen(logData->filepath,"a");

    iter_client = 0;
    iter_log = 0;
    iter_cont = 0;

    //fclose(data[0].h.fp);
    //initModel(tensorflow);
    //runModel(tensorflow);
    


    //*************Initialize threads + mutexes*******************
    
    memset (thread, 0, NUMBER_OF_THREADS * sizeof (pthread_t));
    pthread_t empty_thread[NUMBER_OF_THREADS];
    thread = empty_thread;

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
        pthread_mutex_unlock(&data[i].lock);
    }

    //*************Lock Memory*******************

    lockMemory();
}


void RunController(struct States *data, pthread_t *thread, pthread_attr_t *attr, int * fd)
{

    printf("thread: %d\n",pthread_create(&thread[0], &attr[0], controllerThread, (void *)data));
    if(loggingActivated) printf("thread: %d\n",pthread_create(&thread[1], &attr[1], logThread, (void *)data));
    printf("thread: %d\n",pthread_create(&thread[2], &attr[2], clientThread, (void *)data));
    //pthread_join(thread[0], NULL);
    //pthread_join(thread[1], NULL);

    WaitForMsg(commData->sockfd, &(controlParams->currentState));

    printf("Finished Threads...\n");

    return;
}

void ResetController()
{

    munlockall();
    memset(controlParams, 0, sizeof(*controlParams));
    //memset(logData, 0, sizeof(*logData));
    //memset(daq, 0, sizeof(*daq));


}

void GetParameterFloat(char *regex, double *param)
{
    regcomp(&compiled, regex, REG_EXTENDED);
    if(regexec(&compiled, buffer, 2, matches, 0)==0){
        sprintf(matchBuffer, "%.*s\n", matches[1].rm_eo-matches[1].rm_so,  buffer+matches[1].rm_so );
        sscanf(matchBuffer, "%lf", param);
        printf("%s %f\n", regex, *param);
    }
}

void GetParameterInt(char *regex, int *param)
{
    regcomp(&compiled, regex, REG_EXTENDED);
    if(regexec(&compiled, buffer, 2, matches, 0)==0){
        sprintf(matchBuffer, "%.*s\n", matches[1].rm_eo-matches[1].rm_so,  buffer+matches[1].rm_so );
        sscanf(matchBuffer, "%d", param);
        printf("%s %d\n", regex, *param);
    }
}