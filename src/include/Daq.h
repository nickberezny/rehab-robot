void ReadWriteDAQ(struct States * s, struct DAQ * daq);
int initDaq(struct DAQ *daq);
bool closeAllDaqs();
void zeroDaq(struct DAQ * daq);