#include <math.h>
#include <stdio.h>
#include "rckt_lib.h"
#include "air_functions.h"

//This file contains the functions that define the atmospheric parameters

rckt_xyz rckt_forces (double time, rckt_prmtrs rckt_model, rckt_xyz rckt_flight){
	air_data air;		
	double Fd;				//Force of Drag	[N]
	double Ft;				//Force of Thrust [N]
	double Fg;				//Forcg of Gravity [N]	
	double h = rckt_flight.Z_pos;		//altitude in [m]
	double velocity;			//velocity in [m/s]
	double a;				//acceleration of rocket [m/s^2]
	double rho; 				//density of air [kg/m^3]
	double Cd = rckt_model.Cd;		//drag coefficient 
	double Ar = rckt_model.Ar;		//frontal area of rocket [m^2]
	double m  = rckt_model.mw;
	air = air_functions(h);						
	rho = air.density;
	velocity = pow((pow(rckt.X_vel,2)+pow(rckt.Y_vel,2)+pow(rckt.Z_vel,2)), 0.5);
	
	//Determine Drag Force
	Fd = 0.5*velocity*Cd*Ar*rho*pow(v,2);	
		
	//Determine Thrust Force	
	if (time<=6){
		Ft = sw*(1307-31.18*time);
	}else{
		Ft = 0;
	}
	
	//Determine Gravity Force
	Fg = m*(-9.81);

	//sum up the forces, and divide by the mass of the rocket to return the
	//acceleration of the rocket
	a = (Ft+Fd+Fg)/m;
	
	return;
}
