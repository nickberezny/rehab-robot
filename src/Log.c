/**
 * @file Log.c
 * @author Nick Berezny
 * @date 20 Jan 2022
 * @Data Logging init and thread 
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>
#include <sched.h>
#include <stdbool.h>
#include <limits.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "./include/Parameters.h"
#include "./include/Structures.h"
#include "./include/Controller.h"
#include "./include/TimeUtilities.h"



void initFolder(char * filename, struct tm * timeinfo, char * folder)
{

    char * data_file_name;
    data_file_name = asctime(timeinfo);
    int en = strlen(data_file_name);
    data_file_name[en-1] = '_';

    
    strcat(folder, data_file_name);

    for(int i; i < strlen(folder) - 1; i++)
    {
        if (folder[i] == ' ') 
            folder[i]='_';
        if (folder[i] == ':')
            folder[i]='-';
    }

    mkdir(folder,S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

    
    
    
}


void initLog(char * filename, FILE * fp, struct tm * timeinfo)
{
    char folder[1000] = "log/";

    initFolder(filename, timeinfo, folder);
    
    strcat(folder, filename);
    printf("%s\n", folder); 

    fp = fopen(folder,"w");

    //file header
    fprintf(fp, "Rehab Robot Log File\n");
    fprintf(fp, "%s\n", asctime(timeinfo));

    fclose(fp);

}


void * logThread (void * d)
{
    return;
}