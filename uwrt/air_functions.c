#include <math.h>
#include <iostream.h>
#include "est_apogee.h"

//This file contains the functions that define the atmospheric
//parameters

void air_functions (double z, 
					double & air_pressure,
					double & air_temperature,
					double & air_density,
					double & air_windspeed, 
					double & air_winddirect){
	//z is the altitude in metres (m)
	//airdata.pressure = air pressure in Pascals (Pa)
	air_pressure = 101325*(1-0.0000225577*z)^5.25588;
	//Assumes a linearly varying temperature with altitude
	double Tfivekm = 255;  //Temperature at 5km (above sea level)
	double Tground = 288;  //Temperature at sea level
	air_temperature = ((Tground-Tfivekm)/(-5000))*(z-5000)+Tfivekm;
	air_density = 0.029*air_pressure/(T*8.31447);    //density in kg/m^3
	air_windspeed = air_windspeed*((rand()/double(RAND_MAX)-0.5)+1);
	air_winddirect = air_winddirect*((rand()/double(RAND_MAX)-0.5)+1);
	return ();
}
