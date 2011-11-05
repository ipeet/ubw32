#include <string.h>
#include "error.h"
#include "unistd.h"
#include <math.h>
#include "gps.h"
#include "timing.h"
#include <stdio.h>
enum states { WAITING_FOR_SENSORS, WAITING_FOR_IGNITION, IGNITING, 
		UNDER_THRUST, COASTING, DROGUE_DEPLOYED, MAIN_DEPLOYED } current_state;
gps_data_t* datafromgps;
sw_stuff* thrust_switcher;
kf_data_t* deployment_criterion;
kf_data_t* apogee_assessment;
int timer
void StateMachine() 
{
	//Outputs state of rocket to global variable
		switch (current_state) 
		{
			case WAITING_FOR_SENSORS
			//We transition to wait for ignition when the gps reports data
			datafromgps=gps_data();
			timer=sys_time();
			if(datafromgps->gps_mode==1 || timer>300000){
			current_state=WAITING_FOR_IGNITION;
			}
			break;
			case WAITING_FOR_IGNITION
			//When someone hits the igniter, a pin lights up and we execute:
			//Turn on igniter
			//Wait 0.5 seconds
			sleep(0.5);
			//turn on pyrovalve
			current_state=SCREAMING_LIKE_MANIAC;
			break;
			case SCREAMING_LIKE_MANIAC
			//get new altitude prediction
			thrust_switcher=RocketSw(mu[2], t, sw_safety)
			//Check to see if you have reached apogee
			apogee_assessment=RocketKF(mu_prior[2], S_prior[2][2], y[2], t, delta_t,sw);
			if(apogee_assessment->mu[2] < 0){
				current_state=ALTITUDE_10000;
			}
			//check for 10000 feet predicted
			if(thrust_switcher.safety->1 && thrust_switcher.mtroff->1){
				current_state=ALTITUDE_10000;
			}
			break;
			case ALTITUDE_10000
			//wait 2 seconds
			sleep(2);
			current_state=EXPERIMENT_DONE;
			break;
			case EXPERIMENT_DONE
			//get altimeter data from Kalman Filter
			//When altimeter reads less than 1000
			deployment_criterion=RocketKF(mu_prior[2], S_prior[2][2], y[2], t, delta_t,sw);
			if(deployment_criterion->mu[1] < 2307){
				//deploy_drogue;
				current_state=UNDER_CEILING;
			}
			break;
			case UNDER_CEILING
			break;	
		}		
}
