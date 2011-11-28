#ifndef AIR_FUNCTIONS_H_
#define AIR_FUNCTIONS_H_

/* Determine the properties of the atmosphere at that altitude */
void air_functions (double z, double & air_pressure,
					double & air_temperature,
					double & air_density,
					double & air_windspeed, 
					double & air_winddirect);

#endif //AIR_FUNCTIONS_H_

