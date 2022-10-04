void initFolder(struct tm * timeinfo, char * folder);
void initLog(char * filename, struct LogData *logData, struct tm * timeinfo);
void * logThread (void * d);
