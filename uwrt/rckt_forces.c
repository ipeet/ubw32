#include <math.h>
#include <stdio.h>
#include <meschach/matrix.h>
#include <meschach/sparse.h>
#include <assert.h>
#include "rckt_lib.h"
#include "air_functions.h"

//This file contains the functions that define the atmospheric parameters

rckt_xyz rckt_forces (	double time, 
			double dtime, 
			rckt_prmtrs rckt_model, 
			rckt_xyz rckt_xyz_prior, 
			rckt_rpw rckt_rpw_prior){

	air_data air;		
	rckt_xyz rckt_xyz_now;	
	double Fd;				//Force of Drag	[N]
	double Ft;				//Force of Thrust [N]
	double Fg;				//Forcg of Gravity [N]	
	double z = 0;				//altitude in [m]
	double vel;				//velocity in [m/s]
	double a;				//acceleration of rocket [m/s^2]
	double rho; 				//density of air [kg/m^3]
	double Cd = rckt_model.Cd;		//drag coefficient 
	double Ar = rckt_model.Ar;		//frontal area of rocket [m^2]
	double m  = rckt_model.mw;
	int sw = 1;	
	z = rckt_xyz_prior.Z_pos;
	air = air_functions(z);						
	rho = air.density;
	
	double XX;
	double YY;
	double ZZ;
	
	XX = pow(rckt_xyz_prior.X_vel,2);	
	YY = pow(rckt_xyz_prior.Y_vel,2);
	ZZ = pow(rckt_xyz_prior.Z_vel,2);
//	vel = pow((XX+YY+ZZ), 0.5);
	vel = rckt_xyz_prior.Z_vel;	assert(vel > -1000);	

	//Determine Drag Force
	Fd = 0.5*Cd*Ar*rho*pow(vel,2);
	if(rckt_xyz_prior.Z_vel > 0){	
		Fd = -1*Fd;	
	}		
	if(rckt_xyz_prior.Z_pos < 0){
		Fd = 0;
		vel = 0;
	}

	//Determine Thrust Force	
	if (time<=10){
		Ft = 1000;
	}else{
		Ft = 0;
	}
	
	//Determine Gravity Force
	Fg = m*(-9.81);

	//sum up the forces, and divide by the mass of the rocket to return the acceleration of the rocket
	a = (Ft+Fd+Fg)/m;

	rckt_xyz_now.X_vel = 0; 			//x velocity of rocket
	rckt_xyz_now.Y_vel = 0; 			//y velocity of rocket
	rckt_xyz_now.Z_vel = vel+a*dtime; 			//z velocity of rocket
	rckt_xyz_now.X_pos = 0; 			//x coordinate of rocket
	rckt_xyz_now.Y_pos = 0; 			//y coordinate of rocket
    	rckt_xyz_now.Z_pos = z+(rckt_xyz_now.Z_vel)*dtime;//z coordinate of rocket
	rckt_xyz_now.X_acc = 0; 			//x acceleration of rocket
	rckt_xyz_now.Y_acc = 0; 			//y acceleration of rocket
	rckt_xyz_now.Z_acc = a; 			//z acceleration of rocket  
	printf("%f, %f, %f, %f \n", time, Ft, Fd, Fg);	
	return rckt_xyz_now;
}
