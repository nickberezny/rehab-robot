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

	double testAcc[3];
	double testGyro[3];

	int numBytes, errAdress;
	char aBytes[64] = {0x6B, 0x00}; // TX/RX bytes will go here

	LJM_eWriteName(handle, "I2C_SDA_DIONUM", 2);
	LJM_eWriteName(handle, "I2C_SCL_DIONUM", 3);

	LJM_eWriteName(handle, "I2C_SPEED_THROTTLE", 0);
	LJM_eWriteName(handle, "I2C_OPTIONS", 0);
	LJM_eWriteName(handle, "I2C_SLAVE_ADDRESS", 0x68);

	LJM_eWriteName(handle, "I2C_NUM_BYTES_TX", 2); // Set the number of bytes to transmit
	LJM_eWriteName(handle, "I2C_NUM_BYTES_RX", 14); // Set the number of bytes to receive

	// Set the TX bytes. We are sending 1 byte for the address.
	numBytes = 2;
	//aBytes[0] = {0x6B; // Byte 0: Memory pointer = 0
	LJM_eWriteNameByteArray(handle, I2C_WRITE_NAME, numBytes, aBytes, &errAdress);
	LJM_eWriteName(handle, "I2C_GO", 1); // Do the I2C communications.

	LJM_eWriteName(handle, "I2C_NUM_BYTES_TX", 1); // Set the number of bytes to transmit


	LJM_eWriteName(handle, "I2C_SLAVE_ADDRESS", 0x69);
	LJM_eWriteName(handle, "I2C_NUM_BYTES_TX", 2); // Set the number of bytes to transmit
	LJM_eWriteName(handle, "I2C_NUM_BYTES_RX", 14); // Set the number of bytes to receive

	// Set the TX bytes. We are sending 1 byte for the address.
	numBytes = 2;
	//aBytes[0] = {0x6B; // Byte 0: Memory pointer = 0
	LJM_eWriteNameByteArray(handle, I2C_WRITE_NAME, numBytes, aBytes, &errAdress);
	LJM_eWriteName(handle, "I2C_GO", 1); // Do the I2C communications.

	LJM_eWriteName(handle, "I2C_NUM_BYTES_TX", 1); // Set the number of bytes to transmit

	sleep(1);

	for(int i = 0; i < 1000; i++)
	{
		LJM_eWriteName(handle, "I2C_SLAVE_ADDRESS", 0x68);
		numBytes = 1;
		aBytes[0] = 0x3B; // Byte 0: Memory pointer = 0
		LJM_eWriteNameByteArray(handle, I2C_WRITE_NAME, numBytes, aBytes, &errAdress);
		LJM_eWriteName(handle, "I2C_GO", 1); // Do the I2C communications.

		// Read the RX bytes.
		numBytes = 14;
		for (int i = 0; i < numBytes; i++) {
			aBytes[i] = 0;
		}
		LJM_eReadNameByteArray(handle, I2C_READ_NAME, numBytes, aBytes, &errAdress);

		testAcc[0] = (aBytes[0] << 8) + aBytes[1];
		testAcc[1] = (aBytes[2] << 8) + aBytes[3];
		testAcc[2] = (aBytes[4] << 8) + aBytes[5];

		testGyro[0] = (aBytes[8] << 8) + aBytes[9];
		testGyro[1] = (aBytes[10] << 8) + aBytes[11];
		testGyro[2] = (aBytes[11] << 8) + aBytes[12];

		//printf("%d: %d\n", (unsigned char)aBytes[0], (unsigned char)aBytes[0] << 8 );
		
		for (int i = 0; i < 3; i++) {
			printf("%f \n", testAcc[i]/16384.0);
		}
		for (int i = 0; i < 3; i++) {
			printf("%f \n", testGyro[i]/(1.114*32.0*30023.0));
		}
		printf("-----\n");

		LJM_eWriteName(handle, "I2C_SLAVE_ADDRESS", 0x69);
		numBytes = 1;
		aBytes[0] = 0x3B; // Byte 0: Memory pointer = 0
		LJM_eWriteNameByteArray(handle, I2C_WRITE_NAME, numBytes, aBytes, &errAdress);
		LJM_eWriteName(handle, "I2C_GO", 1); // Do the I2C communications.

		// Read the RX bytes.
		numBytes = 14;
		for (int i = 0; i < numBytes; i++) {
			aBytes[i] = 0;
		}
		LJM_eReadNameByteArray(handle, I2C_READ_NAME, numBytes, aBytes, &errAdress);

		testAcc[0] = (aBytes[0] << 8) + aBytes[1];
		testAcc[1] = (aBytes[2] << 8) + aBytes[3];
		testAcc[2] = (aBytes[4] << 8) + aBytes[5];

		testGyro[0] = (aBytes[8] << 8) + aBytes[9];
		testGyro[1] = (aBytes[10] << 8) + aBytes[11];
		testGyro[2] = (aBytes[11] << 8) + aBytes[12];

		//printf("%d: %d\n", (unsigned char)aBytes[0], (unsigned char)aBytes[0] << 8 );
		
		for (int i = 0; i < 3; i++) {
			printf("%f \n", testAcc[i]/16384.0);
		}
		for (int i = 0; i < 3; i++) {
			printf("%f \n", testGyro[i]/(1.114*32.0*30023.0));
		}
		printf("-----\n");
		sleep(1);
	}

	
}