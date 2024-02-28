#include <string.h>
#include <LabJackM.h>

void I2C(int handle);

int main()
{

	int err, handle, errAdress;

    handle = 0;
    // Open first found LabJack
    err = LJM_Open(LJM_dtANY, LJM_ctANY, "LJM_idANY", &handle);

	I2C(handle);

	return 0;
}

void I2C(int handle)
{
	const char * I2C_WRITE_NAME = "I2C_DATA_TX";
	const char * I2C_READ_NAME = "I2C_DATA_RX";

	int numBytes, errAdress;
	char aBytes[32] = {0x6B, 0x00}; // TX/RX bytes will go here

	LJM_eWriteName(handle, "I2C_SDA_DIONUM", 0);
	LJM_eWriteName(handle, "I2C_SCL_DIONUM", 1);

	LJM_eWriteName(handle, "I2C_SPEED_THROTTLE", 0);
	LJM_eWriteName(handle, "I2C_OPTIONS", 0);
	LJM_eWriteName(handle, "I2C_SLAVE_ADDRESS", 0x68);

	LJM_eWriteName(handle, "I2C_NUM_BYTES_TX", 2); // Set the number of bytes to transmit
	LJM_eWriteName(handle, "I2C_NUM_BYTES_RX", 6); // Set the number of bytes to receive

	// Set the TX bytes. We are sending 1 byte for the address.
	numBytes = 2;
	//aBytes[0] = {0x6B; // Byte 0: Memory pointer = 0
	LJM_eWriteNameByteArray(handle, I2C_WRITE_NAME, numBytes, aBytes, &errAdress);
	LJM_eWriteName(handle, "I2C_GO", 1); // Do the I2C communications.


	sleep(1);

	for(int i = 0; i < 1000; i++)
	{
		numBytes = 1;
		aBytes[0] = 0x3B; // Byte 0: Memory pointer = 0
		LJM_eWriteNameByteArray(handle, I2C_WRITE_NAME, numBytes, aBytes, &errAdress);
		LJM_eWriteName(handle, "I2C_GO", 1); // Do the I2C communications.

		// Read the RX bytes.
		numBytes = 6;
		for (int i = 0; i < 6; i++) {
			aBytes[i] = 0;
		}
		LJM_eReadNameByteArray(handle, I2C_READ_NAME, numBytes, aBytes, &errAdress);

		int testX = (aBytes[0] << 8) + aBytes[1];

		printf("Read  User Memory [0-3] = ");
		for (int i = 0; i < 6; i++) {
			printf("%d ", (unsigned char)aBytes[i]);
		}
		printf("\n");
		sleep(1);
	}

	
}