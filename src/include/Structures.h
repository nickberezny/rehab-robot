/**
 * @file Structures.h
 * @author Nick Berezny
 * @date 13 Apr 2022
 * @brief Robot state structure and sub-structures for storing data during operation
 *
 */

#include "./Parameters.h"
#include <stdbool.h>
#include "tensorflow/c/c_api.h"



struct preRunStates {
	double kp,kv;
	double dx,x;
	double Fext;
	struct timespec t_start;
	struct timespec t_end;
	struct timespec t_first;
	double dt;
	double t;
	int lsb, lsf;
};

struct tensorFlowVars {
	TF_Session* Session;
	TF_Status* Status;
	TF_Output* Input;
	TF_Output* Output;
	TF_Tensor** InputValues;
	TF_Tensor** OutputValues;
	int NumInputs;
	int NumOutputs;
	int InputSize;
	int OutputSize;
	double * inputVals;
	double * outputVals;
};


struct ControlParams {

	double Md, Dd, Kd;
	double kp, kv;
	double m, c;
	double delta, alpha;
	double *Ad, *Bd;
	double xend; //length of actuator
	double x0, dx0, ddx0, x0dist;
	double *t;
	double *x;
	double *tdist;
	double *xdist;
	double *x0_duration;
	int x0_is_percent;
	int trajSize;
	double ** cmd;
	double xstart;
	double Fext_offset;
	double dx_bound;
	double t_last;

	struct timespec t_first;
	bool firstRun;

    int x0_index;

	int recordEMG;
	int currentState; 
	int cont_iteration;
	double controlMode; //0 = PD, 1 = Adm, 2 = Imp, 3 = UIC, 4 = Stoch. Force

	int useFriction;
	struct tensorFlowVars * tensorflow;

	double F_for_Kest[100];
	int F_index;
	double X_init_for_Kest;
	double Kest;

	double filter_a_10Hz[4];
	double filter_b_10Hz[4]; 
	double filter_a_100Hz[4];
	double filter_b_100Hz[4]; 
	double dx_filt_x[FILTER_ORDER+1];
	double dx_filt_y[FILTER_ORDER+1];
	double F_filt_x[FILTER_ORDER+1];
	double F_filt_y[FILTER_ORDER+1];
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
	int * aNumValues[DAQ_NUM_OF_CH];
	int * aWrites[DAQ_NUM_OF_CH];
	int errorAddress;
	int daqHandle;
	int numChannels;

	double dataRead[3];
    double writeValues[1];
	
};

struct States {

	pthread_mutex_t lock; 
	struct timespec t_start;
	struct timespec t_end;
	double dt;
	double t;

	double x, dx, ddx;
	double Fext, Fraw;
	double xv,dxv,ddxv;
	double xv_prev, dxv_prev, ddxv_prev;
	double x0,dx0,ddx0,x0_to_send;
	double x0_duration;
	double xstar, cmd;
	double emg1,emg2,emg3,emg4;
	double gonio;
	
	
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
    char *phaseTime;
    char *numPositions;
    char *controlMode;
    char *trajectoryMode;
    char *recordEMG;
    char *stochasticStepTime;
    char *randomRate;
    char *amplitude;
  	char *frequency;
  	char *offset;
  	char *useFriction;
};


