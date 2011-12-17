#include <math.h>
#include <stdio.h>
#include <assert.h>
#include <meschach/matrix.h>
#include <meschach/sparse.h>
#include "rckt_lib.h"
#include "air_functions.h"

int main(){
	double time[1001];
	double dtime = 0.1;	
	rckt_xyz rckt_flight[1001];
	rckt_rpw rckt_twist[1001];
	double altitude[1001];
	rckt_prmtrs rckt_model;
	
	rckt_model.mw = 40;	//mass of fuelled rocket (kg)
	rckt_model.md = 35;	//mass of unfuelled rocket (kg)
	rckt_model.Cd = 0.5;	//drag coefficient of rocket (unitless)
	rckt_model.Ar = M_PI*.01;//frontal area of rocket (m^2)
	
	time[0] = 0;

	rckt_flight[0].X_pos = 0;
	rckt_flight[0].Y_pos = 0;
	rckt_flight[0].Z_pos = 0;
	rckt_flight[0].X_vel = 0;
	rckt_flight[0].Y_vel = 0;
	rckt_flight[0].Z_vel = 0;
	rckt_flight[0].X_acc = 0;
	rckt_flight[0].Y_acc = 0;
	rckt_flight[0].Z_acc = 0;

	altitude[0] = 0;

	rckt_twist[0].R_pos = 0;
	rckt_twist[0].P_pos = 0;
	rckt_twist[0].W_pos = 0;
	rckt_twist[0].R_vel = 0;
	rckt_twist[0].P_vel = 0;
	rckt_twist[0].W_vel = 0;
	rckt_twist[0].R_acc = 0;
	rckt_twist[0].P_acc = 0;
	rckt_twist[0].W_acc = 0;

	int n;
	for(n = 1; n<=1000; n = n+1){
		assert(n>=0);		
		time[n] = time[n-1] + dtime;
		rckt_flight[n] = rckt_flight[0];	
		rckt_twist[n] = rckt_moments (	time[n], 
						dtime, 
						rckt_model, 
						rckt_flight[n-1], 
						rckt_twist[n-1]);		
	}
}
