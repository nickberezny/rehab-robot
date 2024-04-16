#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <inttypes.h>
#include <stdlib.h>

#include <sys/time.h>

#include "ForceSensor.h"

struct ForceSensorData
{
	int socket_desc;
	struct sockaddr_in server_addr;
	int server_struct_length = sizeof(server_addr);
	int32_t msg[9];
	int32_t temp[4];
	double F[3];
 	double T[3];
}

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

    forceSensorData->server_addr;
    iforceSensorData->server_struct_length = sizeof(forceSensorData->server_addr);
    forceSensorData->socket_desc = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    
    if(forceSensorData->socket_desc < 0){
        printf("Error while creating socket\n");
        return -1;
    }
    printf("Socket created successfully\n");
    
    forceSensorData->server_addr.sin_family = AF_INET;
    forceSensorData->server_addr.sin_port = htons(547);
    forceSensorData->server_addr.sin_addr.s_addr = inet_addr("192.168.1.11");
    
    uint64_t command_header = 0x1234;
    uint64_t command = 0x02; //firmwareVersionRead
    uint64_t argument = 100; //doesn't work??

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

    uint64_t sendData = b[0] | b[1] | b[2] | b[3] | b[4] | b[5] | b[6] | b[7];

    if(sendto(forceSensorData->socket_desc, &sendData, 8, 0,
         (struct sockaddr*)&(forceSensorData->server_addr), forceSensorData->server_struct_length) < 0){
        printf("Unable to send message\n");
        return -1;
    }
}

void readFroceSensor(struct ForceSensorData * forceSensorData)
{
	
    if(recvfrom(forceSensorData->socket_desc, msg, 4*9, 0,
     (struct sockaddr*)&(forceSensorData->server_addr), &(forceSensorData->server_struct_length)) < 0){
    printf("Error while receiving server's msg\n");
    return -1;
    }

    reverseBits(forceSensorData,&(msg[3]));
    forceSensorData->F[0] = (double)msg[3]/1000000.0;
    reverseBits(forceSensorData,&(msg[4]));
    forceSensorData->F[1] = (double)msg[4]/1000000.0;
    reverseBits(forceSensorData,&(msg[5]));
    forceSensorData->F[2] = (double)msg[5]/1000000.0;

    reverseBits(forceSensorData,&(msg[6]));
    forceSensorData->T[0] = (double)msg[6]/1000000.0;
    reverseBits(forceSensorData,&(msg[7]));
    forceSensorData->T[1] = (double)msg[7]/1000000.0;
    reverseBits(forceSensorData,&(msg[8]));
    forceSensorData->T[2] = (double)msg[8]/1000000.0;
    
    /*
    F[0] = (double)reverseBits(forceSensorData,msg[3])/1000000.0;
    F[1] = (double)reverseBits(forceSensorData,msg[4])/1000000.0;
    F[2] = (double)reverseBits(forceSensorData,msg[5])/1000000.0;

    T[0] = (double)reverseBits(forceSensorData,msg[6])/1000000.0;
    T[1] = (double)reverseBits(forceSensorData,msg[7])/1000000.0;
    T[2] = (double)reverseBits(forceSensorData,msg[8])/1000000.0;
    */

}
