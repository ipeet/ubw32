#include <math.h>
#include "est_apogee.h"

//This function will make the call as to when the motor should be switched off
//mu[2] is (altitude, velocity), t is the current time after launch, 
double est_apogee(const double *mu, double t) {
	//target height in metres
	double h0, v0, m0;
	h0 = mu[0];
	v0 = mu[1];
	m0 = 35 - 0.7*t - 0.0167*t*t;
	double h, v, m, hfinal;
	h = h0;
	v = v0;
	m = m0;
	hfinal = h;
	//time step will be fixed and pre-determined
	double tstep;
	tstep = 0.1; 
	//this loop will execute until the rocket starts heading down
	while(v > 0){
		//guess density of air
  		double KRhcoef1, KRhcoef2, KRhcoef3, rho_h;
		KRhcoef1 = -3.118*pow(10,-13)*pow(h, 3);
		KRhcoef2 = 7.4278*pow(10, -9)*pow(h, 2);
		KRhcoef3 = -0.00012549*h;
		rho_h = KRhcoef1+KRhcoef2+KRhcoef3+1.2137;   
		//guess force of drag
		double C1, CxArea, DragCoeff;
		CxArea = pow(0.1, 2)*3.141592; 
		DragCoeff = 0.5;
	    C1 = 0.5*DragCoeff*CxArea*rho_h*v;   
	    //linearly predict next set of h and v values
	    double a_guess;
	    a_guess = -9.81 - C1/m*pow(v,2);
	    v = v + a_guess*tstep;
	    h = h + v*tstep + 0.5*a_guess*pow(tstep, 2);
	    if(h > hfinal){
	        hfinal = h;
	    }
    }

    return hfinal;
}
