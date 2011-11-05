#ifndef kalman_h_
#define kalman_h_

typedef struct {
    double mu[2];
    double S[2][2];
} kf_data_t;

// Pre-fire phase update from gps
void kalman_setup_gps(double alt_est, long msmt_time_ms);

// Pre-fire phase update from pressure
void kalman_setup_press(double alt_est, long msmt_time_ms);

// Update the kalaman estimator from gps data
void update_gps(double alt_est, long msmt_time_ms, int firing);

// Update the kalman estimator from pressure sensor
void update_press(double alt_est, long msmt_time_ms, int firing);

// Get the most recent position + velocity estimates
const double* get_mu_est();

// Get the time of the last estimate.
double get_last_t();

// Update KF directly
void update_kalman(
        double mu_prior[2], double S_prior[2][2], double y[2], double y_good[2], 
        double t, double delta_t, int sw);



// Inspect kf state
kf_data_t kalman_state();

#endif
