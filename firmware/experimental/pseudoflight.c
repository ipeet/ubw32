//on
//waiting for sensors
//confirmed sensors
	//LED indicator for Sensor Fix
//Start taking altitude data
//Set initial conditions
	//indicator LED for good to go
	
//Get signal to launch
//Start ignighter
//wait half a second
//Open valve for nitrous - pyrovalve

//enter a while loop for main flight

	//take sensor reading
	//go through Kalman Filter
	//go through apogee estimator code to see if we will reach apogee
		//no - ok, keep going
		//yes - set safety
			//check next iteration and see if we reach apogee
			//yes - turn off motor
			//no, reset safety to off
	//check to see if we are actually going down
		//no - ok, keep going
		//yes - think about deploying parachutes
			//6 seconds after apogee - deploy drogue
			//1000ft above ground - deploy main	