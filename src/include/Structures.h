struct Params {

	double Md, Bd, Kd;
	double kp, kv;
	double m, c;
	double delta, alpha;
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

	double xstar, cmd;

	struct Params p; 
};


