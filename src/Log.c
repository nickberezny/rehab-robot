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

void * logThread (void * d)
{
    //sleep(0.001f);
    struct States *s; // = (struct CUICStruct*)d;
    int i = 0;

    while(i < BUFFER_SIZE-1)
    {

        s = &((struct States*)d)[i];
        printf("%d mutex: %d\n", i, pthread_mutex_lock(&s->lock));
        printf("Log Thread... %f \n", s->x);
        //fprintf ((s->h.fp), "data %f \n", s->x);
        printf("%d unlock mutex: %d\n", i, pthread_mutex_unlock(&s->lock));
        i = i + 1;
        
    }
    
    printf("Done Log...\n");
    //pthread_exit(NULL);
    return NULL;
}



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


void initLog(char * filename, struct States * s, struct tm * timeinfo)
{
    char folder[1000] = "log/";

    initFolder(filename, timeinfo, folder);
    
    strcat(folder, filename);
    

    strcpy(s->h.filepath, folder);
    s->h.fp = fopen(folder,"w");

    //file header
    fprintf(s->h.fp, "Rehab Robot Log File\n");
    fprintf(s->h.fp, "%s\n", asctime(timeinfo));
    fclose(s->h.fp);

    printf("%s\n", folder); 

   

}


