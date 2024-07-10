#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <inttypes.h>
#include <stdlib.h>
#include <fcntl.h>

#include <sys/time.h>

#include "./include/Parameters.h"
#include "./include/Structures.h"
#include "./include/ForceSensor.h"


void reverseBits(struct ForceSensorData * forceSensorData, int32_t * num)
{
    
    forceSensorData->temp[0] = (*num &  0xff000000) >> 24; 
    forceSensorData->temp[1] = (*num &  0x00ff0000) >> 8; 
    forceSensorData->temp[2] = (*num &  0x0000ff00) << 8; 
    forceSensorData->temp[3] = (*num &  0x000000ff) << 24; 
    *num = forceSensorData->temp[0] | forceSensorData->temp[1] | forceSensorData->temp[2] | forceSensorData->temp[3];

}

void initForceSensorUDP(struct ForceSensorData * forceSensorData)
{

    //forceSensorData->server_addr;
    forceSensorData->server_struct_length = sizeof(forceSensorData->server_addr);
    forceSensorData->socket_desc = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

    int flags = fcntl(forceSensorData->socket_desc, F_GETFL, 0);
    //if (flags == -1) return false;
    flags = (flags | O_NONBLOCK);
    fcntl(forceSensorData->socket_desc, F_SETFL, flags) == 0;
    
    if(forceSensorData->socket_desc < 0){
        printf("Error while creating socket\n");
        return;
    }
    printf("Socket created successfully\n");
    
    forceSensorData->server_addr.sin_family = AF_INET;
    forceSensorData->server_addr.sin_port = htons(10547);
    forceSensorData->server_addr.sin_addr.s_addr = inet_addr("192.168.1.11");
    
    forceSensorData->server_struct_length = sizeof(forceSensorData->server_addr);

    forceSensorData->F[0]=0.0;
    forceSensorData->F[1]=0.0;
    forceSensorData->F[2]=0.0;
    forceSensorData->T[0]=0.0;
    forceSensorData->T[1]=0.0;
    forceSensorData->T[2]=0.0;

    /*
    uint64_t command_header = 0x1234;
    uint64_t command = 0x002; //start stream
    uint64_t argument = 1; //doesn't work??

    uint64_t data = argument << 32 | command << 16 | command_header;

    uint64_t b[8];

    //change endianess?
    b[0] = (data &  0x00000000000000ff) << 8;
    b[1] = (data &  0x000000000000ff00) >> 8;
    b[2] = (data &  0x0000000000ff0000) << 8;
    b[3] = (data &  0x00000000ff000000) >> 8;

    b[4] = (data &  0x000000ff) << 8;
    b[5] = (data &  0x0000ff00) >> 8;
    b[6] = (data &  0x00ff0000) << 8;
    b[7] = (data &  0xff000000) >> 8;


    uint64_t sendData = 72057595162014738; //bias set (0x0043)
    //sendData = 72057595178791954; //bias reset (0x0044)

    printf("data %" PRIu64 "\n", sendData);

    if(sendto(forceSensorData->socket_desc, &sendData, 8, 0,
         (struct sockaddr*)&(forceSensorData->server_addr), forceSensorData->server_struct_length) < 0){
        printf("Unable to send message\n");
    }

    sleep(1);
    sendData = 72057594071495698; //stream start (read one)
    sendData = 33567762; //stream start (infinity)

    if(sendto(forceSensorData->socket_desc, &sendData, 8, 0,
         (struct sockaddr*)&(forceSensorData->server_addr), forceSensorData->server_struct_length) < 0){
        printf("Unable to send message\n");
    }
    */
}

void readFroceSensor(struct ForceSensorData * forceSensorData)
{
	/*
    if(recvfrom(forceSensorData->socket_desc, forceSensorData->msg, 4*9, 0,
     (struct sockaddr*)&(forceSensorData->server_addr), &(forceSensorData->server_struct_length)) < 0){
    printf("Error while receiving server's msg\n");
    }
    */
     
    //uint64_t sendData = 72057594071495698;
    //sendto(forceSensorData->socket_desc, &sendData, 8, 0,(struct sockaddr*)&(forceSensorData->server_addr), forceSensorData->server_struct_length);

   
    int i = 0;

    //printf("%d, %d\n",forceSensorData->socket_desc, forceSensorData->msg);


    while(recvfrom(forceSensorData->socket_desc, forceSensorData->msg, 4*9, 0,
     (struct sockaddr*)&(forceSensorData->server_addr), &(forceSensorData->server_struct_length)) != -1)
    {
        i = i + 1;
        //printf("%d\n",i++);
    }

    //printf("%d\n",forceSensorData->msg[3]);
    if(i>0)
    {
        reverseBits(forceSensorData,&(forceSensorData->msg[3]));
        forceSensorData->F[0] = (double)forceSensorData->msg[3]/1000000.0;
        reverseBits(forceSensorData,&(forceSensorData->msg[4]));
        forceSensorData->F[1] = (double)forceSensorData->msg[4]/1000000.0;
        reverseBits(forceSensorData,&(forceSensorData->msg[5]));
        forceSensorData->F[2] = (double)forceSensorData->msg[5]/1000000.0;

        reverseBits(forceSensorData,&(forceSensorData->msg[6]));
        forceSensorData->T[0] = (double)forceSensorData->msg[6]/1000000.0;
        reverseBits(forceSensorData,&(forceSensorData->msg[7]));
        forceSensorData->T[1] = (double)forceSensorData->msg[7]/1000000.0;
        reverseBits(forceSensorData,&(forceSensorData->msg[8]));
        forceSensorData->T[2] = (double)forceSensorData->msg[8]/1000000.0;
    }

    if(forceSensorData->F[2]>100 || forceSensorData->F[2]<-100)
    {
        printf("%d, %f",i,forceSensorData->F[2]);
    }

}

void tareForceSensor(struct ForceSensorData * forceSensorData)
{
    forceSensorData->sendData = 1124086802;//72057595162014738; //bias set (0x0043) //

    if(sendto(forceSensorData->socket_desc, &forceSensorData->sendData, 8, 0,
     (struct sockaddr*)&(forceSensorData->server_addr), forceSensorData->server_struct_length) < 0){
    printf("Unable to send message\n");
    }

    return;
}

void startForceSensorStream(struct ForceSensorData * forceSensorData)
{
    forceSensorData->sendData = 33567762; // start stream (0x0002) 

    if(sendto(forceSensorData->socket_desc, &forceSensorData->sendData, 8, 0,
     (struct sockaddr*)&(forceSensorData->server_addr), forceSensorData->server_struct_length) < 0){
    printf("Unable to send message\n");
    }

    sleep(0.1);
    readFroceSensor(forceSensorData);
    sleep(0.1);

    while(forceSensorData->F[0] == 0.0 && forceSensorData->F[1] == 0.0 && forceSensorData->F[2] == 0.0)
    {

        printf("Failed to start force sensor...\n");

        if(sendto(forceSensorData->socket_desc, &forceSensorData->sendData, 8, 0,
        (struct sockaddr*)&(forceSensorData->server_addr), forceSensorData->server_struct_length) < 0){
        printf("Unable to send message\n");
        }

        sleep(0.5);
        readFroceSensor(forceSensorData);
        sleep(0.1);
    }

    return;
}

void stopForceSensorStream(struct ForceSensorData * forceSensorData)
{
    forceSensorData->sendData = 0; //stop(0x000)

    if(sendto(forceSensorData->socket_desc, &forceSensorData->sendData, 8, 0,
     (struct sockaddr*)&(forceSensorData->server_addr), forceSensorData->server_struct_length) < 0){
    printf("Unable to send message\n");
    }

    return;
}