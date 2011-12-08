#include <math.h>
#include <stdio.h>
#include "air_functions.h"
#include "rckt_lib.h"


int main(){
	//Simulation parameters
	double time_max = 100;			//seconds
	double dtime1 = 0.01;			//time step
	double dtime2 = 0.1;	
	int time1_length;
		time1_length = int(time_max/dtime1)+1;
	int time2_length;
		time2_length = int(time_max/dtime2)+1;
	double time1[time1_length];
	double time2[time2_length];		
	rckt_xyz rckt_flight_prior[time1_length];
	rckt_xyz rckt_flight_now[time2_length];	
	rckt_prmtrs rckt_model1;

	rckt_model1.mw = 35.3;					//mass of fuelled rocket (kg)
	rckt_model1.md = rckt_model.mw-(3.9+0.5);		//mass of unfuelled rocket (kg)
	rckt_model1.br = 0;					//burnrate of rocket motor (kg/s)
	rckt_model1.ve = 2550;          			//velocity of ejected fuel (m/s)
	rckt_model1.pn = 75;					//pressure at exit area of nozzle (kPa)
	rckt_model1.hi = 1621;					//initial altitude at launch (m above avg sea level)
	rckt_model1.vi = 0;					//initial velocity of rocket (m/s)
	rckt_model1.An = M_PI*(0.025)^2;			//area of rocket nozzel (m^2)
	rckt_model1.Ar = M_PI*(0.10)^2;   			//frontal area of rocket (m^2)
	rckt_model1.Cd = 0.8;					//drag coefficient of rocket (unitless)
	rckt_model1.Htarget = 10000*.3048+rckt_model1_hi;	//Target maximum altitude (apogee) (m)

	double k;		
	for( k=2; k<=time_max; k=k+dtime1;){
		rckt_flight_now = rckt_forces(k, rckt_model1, rckt_flight_prior);
	}
	return;
}
