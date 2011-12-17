#include <math.h>
#include <stdio.h>
#include "air_functions.h"

//This file contains the functions that define the atmospheric parameters

air_data air_functions (double z){
	//z is the altitude in metres (m)
	air_data air;	
	double press_exp;
	press_exp = pow((1-0.0000225577*z),5.25588);
	air.pressure = 101325.00*press_exp;
	//Assumes a linearly varying temperature with altitude
	double Tfivekm = 255;  //Temperature at 5km (above sea level)
	double Tground = 288;  //Temperature at sea level
	air.temperature = ((Tground-Tfivekm)/(-5000))*(z-5000)+Tfivekm;
	//density in kg/m^3
	air.density = 0.029*(air.pressure)/((air.temperature)*8.31447);    
	air.windspeed = (air.windspeed);//*((rand()/double(RAND_MAX)-0.5)+1);
	air.winddirect = (air.winddirect);//*((rand()/double(RAND_MAX)-0.5)+1);
	return (air);
}
