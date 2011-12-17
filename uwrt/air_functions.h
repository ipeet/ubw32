#ifndef AIR_FUNCTIONS_H_
#define AIR_FUNCTIONS_H_

typedef struct { 
	double pressure;
	double temperature;
	double density;
	double windspeed; 
	double winddirect;
	} air_data;

/* Determine the properties of the atmosphere at that altitude */
air_data air_functions (double z);

#endif //AIR_FUNCTIONS_H_

