/**
 * @file Structures.h
 * @author Nick Berezny
 * @date 13 Apr 2022
 * @brief Robot state structure and sub-structures for storing data during operation
 *
 */

#include "./Parameters.h"

struct Params {

	double Md, Bd, Kd;
	double kp, kv;
	double m, c;
	double delta, alpha;
};

struct Hardware {

	int lsb, lsf; //limit switches
	FILE * fp;
	char filepath[1000];

};


struct DAQ {

	double aValues[DAQ_NUM_OF_CH];
	const char * aNames[DAQ_NUM_OF_CH];
	int aNumValues[DAQ_NUM_OF_CH];
	int aWrites[DAQ_NUM_OF_CH];
	int errorAddress;
	int daqHandle;
};

struct States {

	pthread_mutex_t lock; 
	struct timespec t_start;
	struct timespec t_end;
	double dt;

	double x, dx, ddx;
	double Fext;
	double xv,dxv,ddxv;
	double x0,dx0,ddx0;
	double *Ad, *Bd;

	double xi; //change in x from encoder

	double xstar, cmd;

	double xend; //length of actuator

	int *sockfd;

	int currentState; //0-Cal;1-Set;2-Ready,3-Cont;4-Pause

	struct Params p; 
	struct Hardware h;
	struct DAQ daq;
};




