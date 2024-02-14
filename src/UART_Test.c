#include <string.h>
#include <stdio.h>
#include <LabJackM.h>

void UART(int handle);

int main()
{

	int err, handle, errAdress;

    handle = 0;
    // Open first found LabJack
    err = LJM_Open(LJM_dtANY, LJM_ctANY, "LJM_idANY", &handle);
    printf("Handle %d\n", handle);
	UART(handle);

	return 0;
}

void UART(int handle)
{


	int errAdress;
	double dataRead[2];
	const double writeValues[1] = {7};
	// Setting CS, CLK, MISO, and MOSI lines for the T7 and other devices.
	LJM_eWriteName(handle, "ASYNCH_ENABLE", 0);
	LJM_eWriteName(handle, "ASYNCH_TX_DIONUM", 1);  // CS is FIO1
	LJM_eWriteName(handle, "ASYNCH_RX_DIONUM", 0);  // CLK is FIO0
	LJM_eWriteName(handle, "ASYNCH_BAUD", 19200);
	LJM_eWriteName(handle, "ASYNCH_NUM_DATA_BITS", 8);
	LJM_eWriteName(handle, "ASYNCH_PARITY", 0);
	LJM_eWriteName(handle, "ASYNCH_NUM_STOP_BITS", 1);
	LJM_eWriteName(handle, "ASYNCH_ENABLE", 1);

	while(1)
	{
		
		LJM_eWriteName(handle, "ASYNCH_NUM_BYTES_TX", 1);
		LJM_eWriteNameArray(handle, "ASYNCH_DATA_TX", 1, writeValues, &errAdress);
		LJM_eWriteName(handle, "ASYNCH_TX_GO", 1);
		
		LJM_eWriteName(handle, "ASYNCH_NUM_BYTES_RX", 2);
		LJM_eReadNameArray(handle, "ASYNCH_DATA_RX", 2, dataRead, &errAdress);
		printf("d1: %d\n", (int)dataRead[0]);
		printf("d2: %d\n", (int)dataRead[1]);

		int num = ((int)dataRead[1]) | ((int)dataRead[0]) << 8;

		printf("Num: %d\n", num);
		
		usleep(20000);

	}

}
