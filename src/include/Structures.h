/**
 * @file Structures.h
 * @author Nick Berezny
 * @date 13 Apr 2022
 * @brief Robot state structure and sub-structures for storing data during operation
 *
 */

#include "./Parameters.h"
#include <stdbool.h>

struct ControlParams {

	double Md, Dd, Kd;
	double kp, kv;
	double m, c;
	double delta, alpha;
	double *Ad, *Bd;
	double xend; //length of actuator
	double x0, dx0, ddx0;
	double xstart;
	double Fext_offset;
	double dx_bound;
	double tf;
	double t_start_phase;
	double Fmax; //for stochastic force generation
	struct timespec t_first;
	bool firstRun;
	
	int recordEMG;
	int currentState; 
	int controlMode; //0 = PD, 1 = Adm, 2 = Imp, 3 = UIC, 4 = Stoch. Force
	int trajectoryMode; //0 = Static pos, 1 = back and forth, 2 = static range (for stoch.force)
	int stochasticState; //0 go to x0, 1 apply forces

};

struct LogData {

	
	FILE * fp;
	char filepath[1000];

};

struct CommData {

	int *sockfd;

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
	double t;

	double x, dx, ddx;
	double Fext;
	double xv,dxv,ddxv;
	double xv_prev, dxv_prev, ddxv_prev;
	double x0,dx0,ddx0;
	double xstar, cmd;
	double emg1,emg2,emg3,emg4;
	
	int lsb, lsf; //limit switches
};

struct regexMatch {
	char *Md;
	char *Dd;
	char *Kd;
    char *xstart;
    char *xend;
    char *x0;
    char *dx0;
    char *alpha;
    char *delta;
    char *kv;
    char *kp;
    char *filename;
    char *Home;
    char *mass;
    char *damp;
    char *Fmax;
    char *controlMode;
    char *trajectoryMode;
    char *recordEMG;
};


