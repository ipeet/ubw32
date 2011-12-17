#include <math.h>
#include <stdio.h>
#include <meschach/matrix.h>
#include <meschach/sparse.h>
#include "rckt_lib.h"
#include "air_functions.h"

rckt_rpw rckt_moments (	double time, 
			double dtime,
			rckt_prmtrs rckt_model, 
			rckt_xyz rckt_xyz_prior, 
			rckt_rpw rckt_rpw_prior){

	//play with matrices
	MAT *A;
	A = m_get(2,2);
	A->me[0][0] = 1;
	A->me[0][1] = 0;
	A->me[1][0] = 0;
	A->me[1][1] = 1;
	printf("# A =\n");       m_output(A);
//	
//	printf("A11: %f, A12: %f, /n A21: %f, A22: %f/n", A->me[0][0], A->me[0][1], A->me[1][0], A->me[1][1]);

	//get moments of inertia in all axis from rckt_model
	
	//get centres of gravity and pressure from the rckt_model

	//get Cl from rckt_model

	//rotate wind speed and direction into rocket-centred co-ordinates

	//rotate rocket velocities relative to the ground into rocket-based velocities
	
	//determine the resultant velocity of the rocket relative to the air (velocity relative to ground + wind relative to ground)

	//determine the Force of Lift acting on the rocket, and use known distance between the Cg and the Cp to determine the arm

	//use moment balance to determine the angular accelerations of the rocket

	//use dtime to determine the new angular velocities of the rocket

	//use dtime to determine the new angular positions of the rocket

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
