#ifndef RCKT_LIB_H_
#define RCKT_LIB_H_

#include <stddef.h>

//points and vectors for the centre of gravity of the rocket
typedef struct {
	double X_pos; //x coordinate of rocket
	double Y_pos; //y coordinate of rocket
    	double Z_pos; //z coordinate of rocket
	double X_vel; //x velocity of rocket
	double Y_vel; //y velocity of rocket
	double Z_vel; //z velocity of rocket
	double X_acc; //x acceleration of rocket
	double Y_acc; //y acceleration of rocket
	double Z_acc; //z acceleration of rocket  	
} rckt_xyz;

//roll (R), pitch (P), yaw (W), and rates of change for those parameters
typedef struct {
	double R_pos; //roll position of rocket
	double P_pos; //pitch position of rocket
    	double W_pos; //yaw position of rocket
	double R_vel; //roll velocity of rocket
	double P_vel; //pitch velocity of rocket
	double W_vel; //yaw velocity of rocket
	double R_acc; //roll acceleration of rocket
	double P_acc; //pitch acceleration of rocket
	double W_acc; //yaw acceleration of rocket  	
} rckt_rpw;

//various constant parameters of the rocket
typedef struct{
	double mw;	//mass of fuelled rocket (kg)
	double md;	//mass of unfuelled rocket (kg)
	double br;	//burnrate of rocket motor (kg/s)
	double ve;      //velocity of ejected fuel (m/s)
	double pn;	//pressure at exit area of nozzle (kPa)
	double hi;	//initial altitude at launch (m above avg sea level)
	double vi;	//initial velocity of rocket (m/s)
	double An;	//area of rocket nozzel (m^2)
	double Ar;   	//frontal area of rocket (m^2)
	double Cd;	//drag coefficient of rocket (unitless)
	double Htarget;	//Target maximum altitude (apogee) (m)
} rckt_prmtrs

rckt_xyz rckt_forces (double time, rckt_prmtrs rckt_model, rckt_xyz rckt_flight);



#endif // RCKT_LIB_H_
