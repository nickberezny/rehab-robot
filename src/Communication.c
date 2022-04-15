/**
 * @file Communication.c
 * @author Nick Berezny
 * @date 13 Apr 2022
 * @brief Provides methods for setting up and using TCP communication
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
#include "./include/Parameters.h"


void openClientSocket(int *fd, struct sockaddr_in *servaddr, int *port)
{

    /**
     * @brief sets and opens a TCP client 
     * @param[in] *fd pointer to socket fd
     * @param[in] *servaddr pointer to server address
     * @param[in] *port pointer to port integer (e.g. 5000)
     */

    if ( (*fd = socket(AF_INET, SOCK_STREAM, 0)) < 0 ) {
        perror("socket creation failed");
        //exit(EXIT_FAILURE);
    }
   
    memset(servaddr, 0, sizeof(*servaddr));
       
    // Filling server information
    servaddr->sin_family = AF_INET;
    servaddr->sin_port = htons(*port);
    servaddr->sin_addr.s_addr = inet_addr(ADDR);


    while(true)
    {
        if (connect(*fd, servaddr, sizeof(*servaddr)) != 0) 
        {
            printf("connection with the server failed...\n");
            sleep(5);
        }
        else
        {
            printf("connected to the server..\n");
            break;
        }
    }
    
    
    
}

void sendMessage(int *fd, char msg[])
{

    /**
     * @brief send message over TCP client
     * @param[in] *fd pointer to socket fd
     * @param[in] msg[] message to send (in format DEV::MSG, e.g. UI::HOME)
     */

    write(*fd, msg, sizeof(char)*strlen(msg));
}

void recvMessage(int *fd, char *buffer)
{
    /**
     * @brief recieve message from TCP server (blocking)
     * @param[in] *fd pointer to socket fd
     * @param[in] *buffer char buffer to contain results
     */

    recv(*fd, buffer, sizeof(buffer), MSG_WAITALL);
}




