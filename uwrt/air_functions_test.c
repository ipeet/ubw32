#include <math.h>
#include <stdio.h>
#include "air_functions.h"
#include "gnuplot.h"
//#include <iostream>

int main(){
	FILE *file;
	file = fopen("air_test_f_out.txt","w+"); 
	double z[1001];
	air_data air;
	double a_p[1001];	
	int n;
	for(n = 0; n<=1000; n = n+1){
		z[n] = (double) n;
		air = air_functions(z[n]);
		a_p[n] = air.pressure;
		printf("%d, %f, %f, %f, %f, %f \n",
						n, 
						air.pressure,
						air.temperature,
						air.density,
						air.windspeed,
						air.winddirect);
		fprintf(file,"%d, %f, %f, %f, %f, %f \n",
						n, 
						air.pressure,
						air.temperature,
						air.density,
						air.windspeed,
						air.winddirect);
	}
	gnuplot_series_t air_functions_test_data;
	air_functions_test_data.yvals = a_p;
	air_functions_test_data.xvals = z;
	air_functions_test_data.len = 1001;
	gnuplot_show(&air_functions_test_data, 1);
	fclose(file);
	return 0;
}
	
	
