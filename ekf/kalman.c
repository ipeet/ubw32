#include <math.h>
#include "kalman.h"

static double _mu[2] = {1317,0};
static double _S[2][2] = {{0,0}, {0,0}}; 

// When the engine started firing, in ms
static long _firing_start = 0;
// Last update time, in ms
static long _last_update_ms = 0;

// Pre-fire update from gps
void kalman_setup_gps(double alt_est, long msmt_time_ms) {
    _firing_start = msmt_time_ms;
    _last_update_ms = msmt_time_ms;
    _mu[0] = alt_est;
}

// Pre-fire update from press
void kalman_setup_press(double alt_est, long msmt_time_ms) {
    _firing_start = msmt_time_ms;
    _last_update_ms = msmt_time_ms;
    _mu[0] = alt_est;
}

// Update from gps data
void update_gps(double alt_est, long msmt_time_ms, int firing) {
    double mu_prior[2] = {_mu[0], _mu[1]};
    double S_prior[2][2] = { {_S[0][0], _S[0][0]},
                         {_S[1][0], _S[1][1]} };
    double y[2] = {alt_est, 0};
    double y_good[2] = {1, 0};

    double t = msmt_time_ms - _firing_start;
    t *= 0.001; // [ms] -> [s]
    double delta_t = msmt_time_ms - _last_update_ms;
    delta_t *= 0.001; // [ms] -> [s]
    if (delta_t == 0.0) {
        // Can't do this, it will divide by zero
        return;
    }

    update_kalman(mu_prior, S_prior, y, y_good, t, delta_t, firing);
    _last_update_ms = msmt_time_ms;
}

// Update from pressure
void update_press(double alt_est, long msmt_time_ms, int firing) {
    double mu_prior[2] = {_mu[0], _mu[1]};
    double S_prior[2][2] = { {_S[0][0], _S[0][0]},
                         {_S[1][0], _S[1][1]} };

    double y[2] = {0, alt_est};
    double y_good[2] = {0, 1};

    double t = msmt_time_ms - _firing_start;
    t *= 0.001; // [ms] -> [s]
    double delta_t = msmt_time_ms - _last_update_ms;
    delta_t *= 0.001; // [ms] -> [s]
    if (delta_t == 0.0) {
        // Can't do this, it will divide by zero
        return;
    }

    update_kalman(mu_prior, S_prior, y, y_good, t, delta_t, firing);
    _last_update_ms = msmt_time_ms;
}

// Get last position + velocity estimates
const double* get_mu_est() {
    return _mu;
}

// Get the time of last estimate [s]
double get_last_t() {
    double ret = _last_update_ms;
    return ret * 0.001; // [ms] -> [s]
}

// Take a look at kf state
kf_data_t kalman_state() {
    kf_data_t ret = {
        .mu = {_mu[0], _mu[1]},
        .S = { {_S[0][0], _S[0][1]},
               {_S[1][0], _S[1][1]} }
    };
    return ret;
}

void update_kalman(
        double mu_prior[2], double S_prior[2][2], double y[2], double y_good[2], 
        double t, double delta_t, int sw) 
{
	//Continuous Motion Model
	double h_prior, v_prior, rho_h, c1, c2, c3, a_guess;
    double S_prior00, S_prior01, S_prior10, S_prior11;
    double mass;
    double CxArea = pow(0.1, 2)*3.14159265;
    double DragCoeff = 0.5;
    h_prior = mu_prior[0];
    v_prior = mu_prior[1];
    S_prior00 = S_prior[0][0];
    S_prior01 = S_prior[0][1];
    S_prior10 = S_prior[1][0];
    S_prior11 = S_prior[1][1];
    //mass in kg, 35 is initial mass
	if ((t < 14) && (sw = 1)){
		mass = 35 - 0.7*t - 0.0167*t*t;
	}
	else{
		mass = 28;
	}
  	//guess density of air
  	double KRhcoef1, KRhcoef2, KRhcoef3;
	KRhcoef1 = -3.118*pow(10, -13)*h_prior*pow(h_prior, 2);
	KRhcoef2 = 7.4278*pow(10, -9)*pow(h_prior, 2);
	KRhcoef3 = -0.00012549*h_prior;
	rho_h = KRhcoef1+KRhcoef2+KRhcoef3+1.2137;
	//printf ("h_prior = %lf ", h_prior);	
    //guess force of drag...ish
    double sign_v, thrust_check;
    if(v_prior > 0){
    	sign_v = 1;
    }	
    else{
    	sign_v = -1;
   	}
    c1 = sign_v*0.5*DragCoeff*CxArea*rho_h*v_prior;	
    //guess force of thrust without noise
    if((t < 14)||(sw == 1)){
    	c2 = 1307-31.18*t;
    }
    else{
    	c2 = 0;
    }     							
    c3 = 1 - delta_t*c1/mass;
    a_guess = -9.81 - c1*v_prior/mass + c2/mass;
    //*_p develops a set of estimates
    double h_p, v_p, Sp00, Sp01, Sp10, Sp11;
    h_p = h_prior + v_prior*delta_t;				
    v_p = c3*v_prior - 9.81*delta_t + c2/mass*delta_t; 
    //printf("t = %lf, h_p = %lf, v_p = %lf, ",t, h_p, v_p);
    //Sp is an estimated covariance around these estimates
    Sp00 = S_prior00+S_prior10*delta_t+0.01;							
    Sp01 = S_prior00*delta_t+S_prior01*c3+delta_t*S_prior10*delta_t*delta_t+S_prior11*delta_t*c3;
	Sp10 = S_prior10*c3;
	Sp11 = S_prior10*c3*delta_t+S_prior11*c3*c3+0.01;
	//Measurement update
	//K is the 'Kalman Gain' or a blending factor that is applied to the model and sensor readings
    double Kden, K00, K01, K10, K11;
    Kden = Sp00*(y_good[0]*100+y_good[1]*0.3);
    K00 = Sp00*y_good[0]*(Sp00*(1+y_good[1])+100)/Kden;
    K01 = Sp00*y_good[1]*(Sp00*(1+y_good[0])+0.09)/Kden;
    K10 = Sp10*y_good[0]*(Sp00*(1+y_good[1])+100)/Kden;
    K11 = Sp10*y_good[1]*(Sp00*(1+y_good[0])+0.09)/Kden;						
   	//mu is the final estimate that gets returned to main program
   	_mu[0] = h_p + K00*(y[0]*y_good[0]-h_p) + K01*(y[1]*y_good[1]-h_p);
    _mu[1] = v_p + K10*(y[0]*y_good[0]-h_p) + K11*(y[1]*y_good[1]-h_p);
    //S is the final covariance matrix that gets returned to the main program
    _S[0][0] = (1-K00*y_good[0]-K01*y_good[1])*Sp00;
    _S[0][1] = (1-K00*y_good[0]-K01*y_good[1])*Sp01;
    _S[1][0] = (-K10*y_good[0]-K11*y_good[1])*Sp00+Sp10;
    _S[1][1] = (-K10*y_good[0]-K11*y_good[1])*Sp01+Sp11;	
}
