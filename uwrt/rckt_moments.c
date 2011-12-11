#include <math.h>
#include <stdio.h>
#include "rckt_lib.h"
#include "air_functions.h"

rckt_rpw rckt_moments (	double time, 
			double dtime,
			rckt_prmtrs rckt_model, 
			rckt_xyz rckt_xyz_prior, 
			rckt_rpw rckt_rpw_prior){
	rckt_rpw_prior.R_pos = 0; //roll position of rocket [rad]
	rckt_rpw_prior.P_pos = 0; //pitch position of rocket [rad]
	rckt_rpw_prior.W_pos = 0; //yaw position of rocket [rad]
	rckt_rpw_prior.R_vel = 0; //roll velocity of rocket [rad/s]
	rckt_rpw_prior.P_vel = 0; //pitch velocity of rocket [rad/s]
	rckt_rpw_prior.W_vel = 0; //yaw velocity of rocket [rad/s]
	rckt_rpw_prior.R_acc = 0; //roll acceleration of rocket [rad/s^2]
	rckt_rpw_prior.P_acc = 0; //pitch acceleration of rocket [rad/s^2]
	rckt_rpw_prior.W_acc = 0; //yaw acceleration of rocket [rad/s^2] 	
	return rckt_rpw_prior;
}
