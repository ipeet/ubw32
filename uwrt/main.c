#include <math.h>
#include <stdio.h>
#include "air_functions.h"
//#include <iostream>

int main(){
	double z[1001];
	double air_pressure[1001];
	double air_temperature[1001];
	double air_density[1001];
	double air_windspeed[1001]; 
	double air_winddirect[1001];
	int n;
	for(n = 0; n<=1000; n++){
		z[n] = (double) n;
		air_functions(	z,
						air_pressure[n],
						air_temperature[n],
						air_density[n],
						air_windspeed[n],
						air_winddirect[n]);
		printf("%d, %f, %f, %f, %f, %f \n",
						n, 
						air_pressure[n],
						air_temperature[n],
						air_density[n],
						air_windspeed[n],
						air_winddirect[n]);
		return 0;
	}
	
	
