void initFolder(char * filename, struct tm * timeinfo, char * folder);
void initLog(char * filename, FILE * fp, struct tm * timeinfo);
void * logThread (void * d);
