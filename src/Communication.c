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
#include <unistd.h>

void openComm(int * fd)
{
    char * myfifo = "/tmp/myfifo";

    int temp = open(myfifo, O_RDONLY);
    *fd = temp;
}

void initFIFO(int * fd)
{
    printf("Init FIFO\n");
    // FIFO file path
    int fd1;
    char * myfifo = "/tmp/myfifo3";

    mkfifo(myfifo, 0666);

    printf("Init FIFO\n");

    // Open FIFO for write only
    fd1 = open(myfifo, O_WRONLY);

    printf("%d\n", fd1);

    write(fd1, "Hi", sizeof("Hi"));
    close(fd1);

    unlink(myfifo);

    printf("Done!\n", fd1);
    return;
}

void sendFIFO(int * fd, char * data[])
{
    write(*fd, "Hi There", strlen("Hi There"));
    close(*fd);

    //unlink(*fd);

    return;
}

void recvFIFO(int * fd, char * buffer[])
{
	*fd = open(*fd,O_RDONLY);
    char buf[100];
    read(*fd, buf, 100);
    printf("Received: %s\n", buf);
    close(*fd);

    return;

}