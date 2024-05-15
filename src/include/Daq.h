void ReadWriteDAQ(struct States * s, struct DAQ * daq);
int initDaq(struct DAQ *daq);
bool closeAllDaqs();
void clearEncoderCounter(struct DAQ * daq);
void readAbsolutePosition(struct States * s, struct DAQ * daq);