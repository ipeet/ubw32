#include <math.h>
#include <stdio.h>
#include <meschach/matrix.h>
#include <meschach/sparse.h>
#include "air_functions.h"
#include "rckt_lib.h"
#include "gnuplot.h"
//#include <iostream>

int main(){
	FILE *file;
	file = fopen("rckt_forces_test_f_out.txt","w+"); 
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
		time[n] = time[n-1] + dtime;
		rckt_twist[0];  //Just test rckt_flight.c here 		
		rckt_flight[n] = rckt_forces (	time[n], 
						dtime, 
						rckt_model, 
						rckt_flight[n-1], 
						rckt_twist[n-1]);
		altitude[n] = rckt_flight[n].Z_pos;		
		printf("%f, %f, %f, %f, %f, %f, %f, %f, %f, %f \n",
						time[n], 
						rckt_flight[n].X_acc,
						rckt_flight[n].X_vel,
						rckt_flight[n].X_pos,
						rckt_flight[n].Y_acc,
						rckt_flight[n].Y_vel,
						rckt_flight[n].Y_pos,
						rckt_flight[n].Z_acc,
						rckt_flight[n].Z_vel,
						rckt_flight[n].Z_pos);
		fprintf(file,"%f, %f, %f, %f, %f, %f, %f, %f, %f, %f \n",
						time[n], 
						rckt_flight[n].X_acc,
						rckt_flight[n].X_vel,
						rckt_flight[n].X_pos,
						rckt_flight[n].Y_acc,
						rckt_flight[n].Y_vel,
						rckt_flight[n].Y_pos,
						rckt_flight[n].Z_acc,
						rckt_flight[n].Z_vel,
						rckt_flight[n].Z_pos);
	}
	gnuplot_series_t rckt_forces_test_data;
	rckt_forces_test_data.yvals = altitude;
	rckt_forces_test_data.xvals = time;
	rckt_forces_test_data.len = 1001;
	gnuplot_show(&rckt_forces_test_data, 1);
	fclose(file);
	return 0;
}	
