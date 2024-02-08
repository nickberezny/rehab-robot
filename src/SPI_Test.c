/**
 * Name: spi.c
 * Desc: Demonstrates SPI communication.
 *
 *       You can short MOSI to MISO for testing.
 *
 *       T7:
 *           MOSI    FIO2
 *           MISO    FIO3
 *           CLK     FIO0
 *           CS      FIO1
 *
 *       T4:
 *           MOSI    FIO6
 *           MISO    FIO7
 *           CLK     FIO4
 *           CS      FIO5
 *
 * If you short MISO to MOSI, then you will read back the same bytes that you
 * write.  If you short MISO to GND, then you will read back zeros.  If you
 * short MISO to VS or leave it unconnected, you will read back 255s.
**/

#include <LabJackM.h>

void SPI(int handle);

int main()
{

	int err, handle, errAdress;

    handle = 0;
    // Open first found LabJack
    err = LJM_Open(LJM_dtANY, LJM_ctANY, "LJM_idANY", &handle);

	SPI(handle);

	return 0;
}

void SPI(int handle)
{
	int iter, errAdress, err;
	const int numBytes = 1;
	char dataRead[1] = {0};
	char dataWrite[1] = {0x00};

	// Setting CS, CLK, MISO, and MOSI lines for the T7 and other devices.
	LJM_eWriteName(handle, "SPI_CS_DIONUM", 1);  // CS is FIO1
	LJM_eWriteName(handle, "SPI_CLK_DIONUM", 0);  // CLK is FIO0
	LJM_eWriteName(handle, "SPI_MISO_DIONUM", 3);  // MISO is FIO3
	LJM_eWriteName(handle, "SPI_MOSI_DIONUM", 2);  // MOSI is FIO2


	// Selecting Mode CPHA=1 (bit 0), CPOL=1 (bit 1)
	LJM_eWriteName(handle, "SPI_MODE", 3);

	// Speed Throttle:
	// Valid speed throttle values are 1 to 65536 where 0 = 65536.
	// Configuring Max. Speed (~800 kHz) = 0
	LJM_eWriteName(handle, "SPI_SPEED_THROTTLE", 0);

	// SPI_OPTIONS:
	// bit 0:
	//     0 = Active low clock select enabled
	//     1 = Active low clock select disabled.
	// bit 1:
	//     0 = DIO directions are automatically changed
	//     1 = DIO directions are not automatically changed.
	// bits 2-3: Reserved
	// bits 4-7: Number of bits in the last byte. 0 = 8.
	// bits 8-15: Reserved

	// Enabling active low clock select pin
	LJM_eWriteName(handle, "SPI_OPTIONS", 0);

	// Write(TX)/Read(RX) 4 bytes
	LJM_eWriteName(handle, "SPI_NUM_BYTES", numBytes);

	// Write the bytes
	dataWrite[1] = {0x88};
	LJM_eWriteNameByteArray(handle, "SPI_DATA_TX", numBytes, dataWrite, &errAdress);
	LJM_eWriteName(handle, "SPI_GO", 1);  // Initiate the transfer
	dataWrite[1] = {0x01};
	LJM_eWriteNameByteArray(handle, "SPI_DATA_TX", numBytes, dataWrite, &errAdress);
	LJM_eWriteName(handle, "SPI_GO", 1);  // Initiate the transfer
	dataWrite[1] = {0x90};
	LJM_eWriteNameByteArray(handle, "SPI_DATA_TX", numBytes, dataWrite, &errAdress);
	LJM_eWriteName(handle, "SPI_GO", 1);  // Initiate the transfer
	dataWrite[1] = {0x00};
	LJM_eWriteNameByteArray(handle, "SPI_DATA_TX", numBytes, dataWrite, &errAdress);
	LJM_eWriteName(handle, "SPI_GO", 1);  // Initiate the transfer



	// Display the bytes written
	printf("\n");
	for (iter = 0; iter < numBytes; iter++) {
		printf("dataWrite[%d] = 0x%02x\n", iter, dataWrite[iter]);
	}

	// Read the bytes
	err = LJM_eReadNameByteArray(handle, "SPI_DATA_RX", numBytes, dataRead, &errAdress);
	
	// Display the bytes read
	printf("\n");
	for (iter = 0; iter < numBytes; iter++) {
		printf("dataRead[%d]  = 0x%02x\n", iter, dataRead[iter]);
	}
}
