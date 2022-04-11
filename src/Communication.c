/**
 * @file Communication.c
 * @author Nick Berezny
 * @date 21 Jan 2022
 * @Communications (FIFO,...) 
 *
 */

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <unistd.h>
#include <stdbool.h>
#include <stdlib.h>


void openClientSocket(int *fd, struct sockaddr_in *servaddr, int *port)
{
    // Creating socket file descriptor
    if ( (*fd = socket(AF_INET, SOCK_STREAM, 0)) < 0 ) {
        perror("socket creation failed");
        //exit(EXIT_FAILURE);
    }
   
    memset(servaddr, 0, sizeof(*servaddr));
       
    // Filling server information
    servaddr->sin_family = AF_INET;
    servaddr->sin_port = htons(*port);
    servaddr->sin_addr.s_addr = inet_addr("192.168.0.93");
    //inet_pton(AF_INET, "127.0.0.1",  servaddr->sin_addr.s_addr);

    if (connect(*fd, servaddr, sizeof(*servaddr)) != 0) {
        printf("connection with the server failed...\n");
        exit(0);
    }
    else
        printf("connected to the server..\n");
    
    
}

void sendMessage(int *fd, char msg[], struct sockaddr_in *servaddr)
{
    //sendto(*fd, msg, strlen(msg), MSG_CONFIRM, servaddr, sizeof(*servaddr));
    write(*fd, msg, sizeof(char)*strlen(msg));
}

void recvMessage(int *fd, char *buffer, int *len, struct sockaddr_in *servaddr)
{
    //int n = recvfrom(*fd, buffer, 1024, MSG_WAITALL, servaddr, len);
    //buffer[n] = '\0';
    //read(*fd, buffer, sizeof(buffer));
    recv(*fd, buffer, sizeof(buffer), MSG_WAITALL);
}




