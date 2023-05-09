#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <robotcontrol.h>

#include "./include/Structures.h"

void initADC()
{
	if(rc_adc_init()){
        fprintf(stderr,"ERROR: failed to run rc_init_adc()\n");
        return;
    }
}

void initCAN(struct CANBUS * c)
{
	int nbytes;
	struct sockaddr_can addr;
	struct ifreq ifr;

	const char *ifname = "vcan0";

	if ((c->conn = socket(PF_CAN, SOCK_RAW, CAN_RAW)) == -1) {
		perror("Error while opening socket");
		return -1;
	}

	strcpy(ifr.ifr_name, ifname);
	ioctl(c->conn, SIOCGIFINDEX, &ifr);
	
	addr.can_family  = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	printf("%s at index %d\n", ifname, ifr.ifr_ifindex);

	if (bind(c->conn, (struct sockaddr *)&addr, sizeof(addr)) == -1) {
		perror("Error in socket bind");
		return -2;
	}
}

void ADC_Test()
{
	rc_adc_read_volt(0);
}

void ReadWrite()
{
	return;
}

void CANTest(struct CANBUS * c)
{


	c->frame.can_id  = 0x123;
	c->frame.can_dlc = 2;
	c->frame.data[0] = 0x11;
	c->frame.data[1] = 0x22;

	nbytes = write(c->conn, &(c->frame), sizeof(struct can_frame));

	printf("Wrote %d bytes\n", nbytes);
	
	return 0;
}