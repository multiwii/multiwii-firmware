#if GPS
#if defined(GPS_SERIAL) || defined(GPS_FROM_OSD)

  // APM_PI.h & APM_PI.cpp
  class APM_PI {
  public:
  
  	/// Constructor for PI that saves its settings to EEPROM
  	///
  	/// @note	PI must be named to avoid either multiple parameters with the
  	///			same name, or an overly complex constructor.
  	///
  	/// @param  initial_p       Initial value for the P term.
      /// @param  initial_i       Initial value for the I term.
      /// @param  initial_imax    Initial value for the imax term.4
  	///
  	APM_PI(const float &initial_p = 0.0,
  		   const float &initial_i = 0.0,
  		   const int16_t &initial_imax = 0.0) :
  		_kp  (initial_p),
  		_ki  (initial_i),
  		_imax(initial_imax)
  	{
  		// no need for explicit load, assuming that the main code uses AP_Var::load_all.
  	}
  
  	/// Iterate the PI, return the new control value
  	///
  	/// Positive error produces positive output.
  	///
  	/// @param error	The measured error value
  	/// @param dt		The time delta in milliseconds (note
  	///					that update interval cannot be more
  	///					than 65.535 seconds due to limited range
  	///					of the data type).
  	/// @param scaler	An arbitrary scale factor
  	///
  	/// @returns		The updated control output.
  	///
  	//long 	get_pi(int32_t error, float	 dt);
  	int32_t get_pi(int32_t error, float dt);
  	int32_t get_p(int32_t error);
  	int32_t get_i(int32_t error, float dt);
  
  
  	/// Reset the PI integrator
  	///
  	void	reset_I();
  
  
  	/// @name	parameter accessors
  	//@{
  
  	/// Overload the function call operator to permit relatively easy initialisation
  	void operator() (const float p,
  	                 const float i,
  	                 const int16_t imaxval) {
  		_kp = p; _ki = i; _imax = imaxval;
  	}
  
  	float	kP() const				{ return _kp; }
  	float	kI() const 				{ return _ki; }
  	int16_t	imax() const			{ return _imax; }
  
  	void	kP(const float v)		{ _kp=v; }
  	void	kI(const float v)		{ _ki=v; }
  	void	imax(const int16_t v)	{ _imax=abs(v); }
  	float	get_integrator() const	{ return _integrator; }
  	void	set_integrator(float i)	{ _integrator = i; }
  
  
  private:
  	float			_kp;
  	float			_ki;
  	int16_t			_imax;
  
  	float				_integrator;		///< integrator value
  };
  int32_t APM_PI::get_p(int32_t error)
  {
  	return (float)error * _kp;
  }
  
  int32_t APM_PI::get_i(int32_t error, float dt)
  {
  	if(dt != 0){
  		_integrator += ((float)error * _ki) * dt;
  
  		if (_integrator < -_imax) {
  			_integrator = -_imax;
  		} else if (_integrator > _imax) {
  			_integrator = _imax;
  		}
  	}
  	return _integrator;
  }
  
  int32_t APM_PI::get_pi(int32_t error, float dt)
  {
  	return get_p(error) + get_i(error, dt);
  }
  
  void
  APM_PI::reset_I()
  {
  	_integrator = 0;
  }
  
  
  
  // AC_PID.h & AC_PID.cpp
  class AC_PID {
  public:
  
  	/// Constructor for PID that saves its settings to EEPROM
  	///
  	/// @note	PIDs must be named to avoid either multiple parameters with the
  	///			same name, or an overly complex constructor.
  	///
  	/// @param  initial_p       Initial value for the P term.
      /// @param  initial_i       Initial value for the I term.
      /// @param  initial_d       Initial value for the D term.
      /// @param  initial_imax    Initial value for the imax term.4
  	///
  	AC_PID(
  	    const float &initial_p = 0.0,
  	    const float &initial_i = 0.0,
  	    const float &initial_d = 0.0,
  	    const int16_t &initial_imax = 0.0) :
  
  		_kp  (initial_p),
  		_ki  (initial_i),
  		_kd  (initial_d),
  		_imax(initial_imax)
  	{
  		// no need for explicit load, assuming that the main code uses AP_Param::load_all.
  	}
  
  	/// Iterate the PID, return the new control value
  	///
  	/// Positive error produces positive output.
  	///
  	/// @param error	The measured error value
  	/// @param dt		The time delta in milliseconds (note
  	///					that update interval cannot be more
  	///					than 65.535 seconds due to limited range
  	///					of the data type).
  	/// @param scaler	An arbitrary scale factor
  	///
  	/// @returns		The updated control output.
  	///
  	int32_t get_pid(int32_t error, float dt);
  	int32_t get_pi(int32_t error, float dt);
  	int32_t get_p(int32_t error);
  	int32_t get_i(int32_t error, float dt);
  	int32_t get_d(int32_t error, float dt);
  
  
  	/// Reset the PID integrator
  	///
  	void	reset_I();
  
  	/// Overload the function call operator to permit relatively easy initialisation
  	void operator() (const float p,
  	                 const float i,
  	                 const float d,
  	                 const int16_t imaxval) {
  		_kp = p; _ki = i; _kd = d; _imax = abs(imaxval);
  	}
  
  	float	kP() const				{ return _kp; }
  	float	kI() const 				{ return _ki; }
  	float	kD() const 				{ return _kd; }
  	int16_t	imax() const			{ return _imax; }
  
  	void	kP(const float v)		{ _kp = v; }
  	void	kI(const float v)		{ _ki = v; }
  	void	kD(const float v)		{ _kd = v; }
  	void	imax(const int16_t v)	{ _imax = abs(v); }
  
  	float	get_integrator() const	{ return _integrator; }
  	void	set_integrator(float i)	{ _integrator = i; }
  
  
  private:
  	float			_kp;
  	float			_ki;
  	float			_kd;
  	int16_t  		_imax;
  
  	float				_integrator;		///< integrator value
  	int32_t				_last_input;		///< last input for derivative
  	float				_last_derivative; 	///< last derivative for low-pass filter
  	float				_output;
  	float				_derivative;
  
  	/// Low pass filter cut frequency for derivative calculation.
  	///
  	static const float _filter = 7.9577e-3; // Set to  "1 / ( 2 * PI * f_cut )";
  	// Examples for _filter:
  	// f_cut = 10 Hz -> _filter = 15.9155e-3
  	// f_cut = 15 Hz -> _filter = 10.6103e-3
  	// f_cut = 20 Hz -> _filter =  7.9577e-3
  	// f_cut = 25 Hz -> _filter =  6.3662e-3
  	// f_cut = 30 Hz -> _filter =  5.3052e-3
  };
  int32_t AC_PID::get_p(int32_t error)
  {
  	return (float)error * _kp;
  }
  
  int32_t AC_PID::get_i(int32_t error, float dt)
  {
  	if((_ki != 0) && (dt != 0)){
  		_integrator += ((float)error * _ki) * dt;
  		if (_integrator < -_imax) {
  			_integrator = -_imax;
  		} else if (_integrator > _imax) {
  			_integrator = _imax;
  		}
  		return _integrator;
  	}
  	return 0;
  }
  
  int32_t AC_PID::get_d(int32_t input, float dt)
  {
  	if ((_kd != 0) && (dt != 0)) {
  		_derivative = (input - _last_input) / dt;
  
  		// discrete low pass filter, cuts out the
  		// high frequency noise that can drive the controller crazy
  		_derivative = _last_derivative +
  		        (dt / ( _filter + dt)) * (_derivative - _last_derivative);
  
  		// update state
  		_last_input 		= input;
  		_last_derivative    = _derivative;
  
  		// add in derivative component
  		return _kd * _derivative;
  	}
  	return 0;
  }
  
  int32_t AC_PID::get_pi(int32_t error, float dt)
  {
  	return get_p(error) + get_i(error, dt);
  }
  
  
  int32_t AC_PID::get_pid(int32_t error, float dt)
  {
  	return get_p(error) + get_i(error, dt) + get_d(error, dt);
  }
  
  
  
  /*
  int32_t AC_PID::get_pid(int32_t error, float dt)
  {
  	// Compute proportional component
  	_output = error * _kp;
  
  	// Compute derivative component if time has elapsed
  	if ((fabs(_kd) > 0) && (dt > 0)) {
  		_derivative = (error - _last_error) / dt;
  
  		// discrete low pass filter, cuts out the
  		// high frequency noise that can drive the controller crazy
  		_derivative = _last_derivative +
  		        (dt / ( _filter + dt)) * (_derivative - _last_derivative);
  
  		// update state
  		_last_error 		= error;
  		_last_derivative    = _derivative;
  
  		// add in derivative component
  		_output 	+= _kd * _derivative;
  	}
  
  	// Compute integral component if time has elapsed
  	if ((fabs(_ki) > 0) && (dt > 0)) {
  		_integrator 		+= (error * _ki) * dt;
  		if (_integrator < -_imax) {
  			_integrator = -_imax;
  		} else if (_integrator > _imax) {
  			_integrator = _imax;
  		}
  		_output 	+= _integrator;
  	}
  
  	return _output;
  }
  */
  
  void
  AC_PID::reset_I()
  {
  	_integrator = 0;
  	_last_input = 0;
  	_last_derivative = 0;
  }

  /****************** PI and PID controllers for GPS ********************/
  APM_PI	pi_poshold_lat(POSHOLD_P, POSHOLD_I, POSHOLD_IMAX * 100);
  APM_PI	pi_poshold_lon(POSHOLD_P, POSHOLD_I, POSHOLD_IMAX * 100);
  AC_PID	pid_poshold_rate_lat(POSHOLD_RATE_P, POSHOLD_RATE_I, POSHOLD_RATE_D, POSHOLD_RATE_IMAX * 100);
  AC_PID	pid_poshold_rate_lon(POSHOLD_RATE_P, POSHOLD_RATE_I, POSHOLD_RATE_D, POSHOLD_RATE_IMAX * 100);
  AC_PID	pid_nav_lat(NAV_P,NAV_I,NAV_D,NAV_IMAX * 100);
  AC_PID	pid_nav_lon(NAV_P,NAV_I,NAV_D,NAV_IMAX * 100);


  #define RADX100                    0.000174532925  
  #define CROSSTRACK_GAIN            1
  #define NAV_SPEED_MIN              100    // cm/sec
  #define NAV_SPEED_MAX              300    // cm/sec
  #define NAV_SLOW_NAV               true
  #define NAV_BANK_MAX 3000        //30deg max banking when navigating (just for security and testing)



  // used to track the elapsed time between GPS reads
  static uint32_t         nav_loopTimer;
  // Delta Time in milliseconds for navigation computations, updated with every good GPS read
  static float 			dTnav;



  static int16_t GPS_wp_radius    = GPS_WP_RADIUS;

  static int16_t x_actual_speed = 0;
  static int16_t y_actual_speed = 0;
  static int32_t last_longitude = 0;
  static int32_t last_latitude  = 0;
  
  static int16_t x_rate_d;
  static int16_t y_rate_d;

  // this is used to offset the shrinking longitude as we go towards the poles
  static float	GPS_scaleLonDown;
  static float	GPS_scaleLonUp;

  // The difference between the desired rate of travel and the actual rate of travel
  // updated after GPS read - 5-10hz
  static int16_t x_rate_error;
  static int16_t y_rate_error;
  static int32_t	long_error, lat_error;

  //Currently used WP
  static int32_t  GPS_WP_latitude,GPS_WP_longitude;

  ////////////////////////////////////////////////////////////////////////////////
  // Location & Navigation
  ////////////////////////////////////////////////////////////////////////////////
  // This is the angle from the copter to the "next_WP" location in degrees * 100
  static int32_t	target_bearing;
  ////////////////////////////////////////////////////////////////////////////////
  // Crosstrack
  ////////////////////////////////////////////////////////////////////////////////
  // deg * 100, The original angle to the next_WP when the next_WP was set
  // Also used to check when we pass a WP
  static int32_t 	original_target_bearing;
  // The amount of angle correction applied to target_bearing to bring the copter back on its optimum path
  static int16_t	crosstrack_error;
  ////////////////////////////////////////////////////////////////////////////////
  // The location of the copter in relation to home, updated every GPS read (1deg - 100)
  static int32_t	home_to_copter_bearing;
  // distance between plane and home in cm
  static int32_t	home_distance;
  // distance between plane and next_WP in cm
  static int32_t	wp_distance;
  
  // used for slow speed wind up when start navigation;
  static int16_t waypoint_speed_gov;

  ////////////////////////////////////////////////////////////////////////////////////
  // moving average filter variables
  //
  
  #define GPS_FILTER_VECTOR_LENGTH 5
  #define LAT  0
  #define LON  1
  
  static uint8_t GPS_filter_index = 0;
  static int32_t GPS_filter[2][GPS_FILTER_VECTOR_LENGTH];
  static int32_t GPS_filter_sum[2];
  static int32_t GPS_read[2];
  static int32_t GPS_filtered[2];
  static int32_t GPS_degree[2];    //the lat lon degree without any decimals (lat/10 000 000)
  static uint8_t axis;
  static uint16_t fraction3[2];


#endif














#if defined(I2C_GPS)

/////////////////////////////////////////////////////////////////////////////////////////
// I2C GPS helper functions
//
// Send a command to the I2C GPS module, first parameter command, second parameter wypoint number
void GPS_I2C_command(uint8_t command, uint8_t wp)
{
    uint8_t _cmd;
    
    _cmd = (wp << 4) + command;
  
    i2c_rep_start(I2C_GPS_ADDRESS);
    i2c_write(I2C_GPS_COMMAND);
    i2c_write(_cmd);
}

#endif 



void GPS_NewData() {
  #if defined(I2C_GPS)
    static uint8_t _i2c_gps_status;
  
    //Do not use i2c_writereg, since writing a register does not work if an i2c_stop command is issued at the end
    //Still investigating, however with separated i2c_repstart and i2c_write commands works... and did not caused i2c errors on a long term test.
  
    GPS_numSat = (_i2c_gps_status & 0xf0) >> 4;
    _i2c_gps_status = i2c_readReg(I2C_GPS_ADDRESS,I2C_GPS_STATUS_00);                 //Get status register 
    if (_i2c_gps_status & I2C_GPS_STATUS_3DFIX) {                                     //Check is we have a good 3d fix (numsats>5)
       GPS_fix = 1;                                     //Num of sats is stored the upmost 4 bits of status
       
       if (armed == 0) { GPS_fix_home = 0; }          // Clear home position when disarmed
       
       if (!GPS_fix_home && armed) {        //if home is not set set home position to WP#0 and activate it
          GPS_I2C_command(I2C_GPS_COMMAND_SET_WP,0);      //Store current position to WP#0 (this is used for RTH)
          nav_takeoff_bearing = heading;                  //Store takeof heading
          GPS_fix_home = 1;                                                           //Now we have a home   
       }
       if (_i2c_gps_status & I2C_GPS_STATUS_NEW_DATA) {                               //Check about new data
          if (GPS_update) { GPS_update = 0;} else { GPS_update = 1;}                  //Fancy flash on GUI :D
          //Read GPS data for distance, heading and gps position 

          i2c_rep_start(I2C_GPS_ADDRESS);
          i2c_write(I2C_GPS_NAV_BEARING);                                                //Start read from here 2x2 bytes distance and direction
          i2c_rep_start(I2C_GPS_ADDRESS+1);

          uint8_t *varptr = (uint8_t *)&nav_bearing;
          *varptr++ = i2c_readAck();
          *varptr   = i2c_readAck();

          varptr = (uint8_t *)&GPS_directionToHome;
          *varptr++ = i2c_readAck();
          *varptr   = i2c_readAck();
          GPS_directionToHome = GPS_directionToHome / 100;  // 1deg =1000 in the reg, downsize
          if (GPS_directionToHome>180) GPS_directionToHome -= 360;

          varptr = (uint8_t *)&GPS_distanceToHome;
          *varptr++ = i2c_readAck();
          *varptr   = i2c_readNak();
          GPS_distanceToHome = GPS_distanceToHome / 100;      //register is in CM, we need in meter

          i2c_rep_start(I2C_GPS_ADDRESS);
          i2c_write(I2C_GPS_LOCATION);                                                //Start read from here 2x2 bytes distance and direction
          i2c_rep_start(I2C_GPS_ADDRESS+1);

          varptr = (uint8_t *)&GPS_latitude;		// for OSD latitude displaying
          *varptr++ = i2c_readAck();
          *varptr++ = i2c_readAck();
          *varptr++ = i2c_readAck();
          *varptr   = i2c_readAck();

          varptr = (uint8_t *)&GPS_longitude;		// for OSD longitude displaying
          *varptr++ = i2c_readAck();
          *varptr++ = i2c_readAck();
          *varptr++ = i2c_readAck();
          *varptr   = i2c_readAck();

          varptr = (uint8_t *)&nav_lat;		 
          *varptr++ = i2c_readAck();
          *varptr++ = i2c_readAck();
          
          varptr = (uint8_t *)&nav_lon;		 
          *varptr++ = i2c_readAck();
          *varptr++ = i2c_readNak();
          
          debug1=nav_lat;
          debug2=nav_lon;
          
          i2c_rep_start(I2C_GPS_ADDRESS);
          i2c_write(I2C_GPS_GROUND_SPEED);          
          i2c_rep_start(I2C_GPS_ADDRESS+1);

          varptr = (uint8_t *)&GPS_speed;			// speed in cm/s for OSD
          *varptr++ = i2c_readAck();
          *varptr   = i2c_readAck();

          varptr = (uint8_t *)&GPS_altitude;       // altitude in meters for OSD
          *varptr++ = i2c_readAck();
          *varptr   = i2c_readAck();

          //GPS_ground_course
          varptr = (uint8_t *)&GPS_ground_course;
          *varptr++ = i2c_readAck();
          *varptr   = i2c_readNak();

          //Adjust heading when navigating
          if (GPSModeHome == 1)
          {  if ( !(_i2c_gps_status & I2C_GPS_STATUS_WP_REACHED) )
              {
          	//Tail control	
                if (NAV_CONTROLS_HEADING) {
                  if (NAV_TAIL_FIRST) {
                      magHold = nav_bearing/100-180;
                      if (magHold > 180)	magHold -= 360;
	              if (magHold < -180)	magHold += 360;
                  } else {
                      magHold = nav_bearing/100;
                  }
                }
              } else {        //Home position reached
              if (NAV_SET_TAKEOFF_HEADING) { magHold = nav_takeoff_bearing; }
              }
          }
        }
    } else {                                                                          //We don't have a fix zero out distance and bearing (for safety reasons)
      GPS_distanceToHome = 0;
      GPS_directionToHome = 0;
      GPS_numSat = 0;
    }

  #endif     

  #if defined(GPS_SERIAL)
  
    while (SerialAvailable(GPS_SERIAL)) {
     if (GPS_newFrame(SerialRead(GPS_SERIAL))) {

       if (GPS_update == 1) GPS_update = 0; else GPS_update = 1;
        if (GPS_fix == 1 && GPS_numSat >= 5) {
          if (armed == 0) {GPS_fix_home = 0;}
          if (GPS_fix_home == 0 && armed) {
            GPS_fix_home = 1;
            GPS_latitude_home  = GPS_latitude;
            GPS_longitude_home = GPS_longitude;
            GPS_calc_longitude_scaling(GPS_latitude);  //need an initial value for distance and bearing calc
            nav_takeoff_bearing = heading;             //save takeoff heading
          }

          //Apply moving average filter to GPS data
    #if defined(GPS_FILTERING)
         //latest unfiltered data is in GPS_latitude and GPS_longitude
         GPS_read[LAT] = GPS_latitude;
         GPS_read[LON] = GPS_longitude;
         GPS_filter_index = ++GPS_filter_index % GPS_FILTER_VECTOR_LENGTH;
         
         for (axis = 0; axis< 2; axis++) {
         GPS_degree[axis] = GPS_read[axis] / 10000000;  // get the degree to assure the sum fits to the int32_t
  
         // How close we are to a degree line ? its the first three digits from the fractions of degree
         // later we use it to Check if we are close to a degree line, if yes, disable averaging,
         fraction3[axis] = (GPS_read[axis]- GPS_degree[axis]*10000000) / 10000;
  
         GPS_filter_sum[axis] -= GPS_filter[axis][GPS_filter_index];
         GPS_filter[axis][GPS_filter_index] = GPS_read[axis] - (GPS_degree[axis]*10000000); 
         GPS_filter_sum[axis] += GPS_filter[axis][GPS_filter_index];
         GPS_filtered[axis] = GPS_filter_sum[axis] / GPS_FILTER_VECTOR_LENGTH + (GPS_degree[axis]*10000000);
         }       
         
         if ( nav_mode == NAV_MODE_POSHOLD) {      //we use gps averaging only in poshold mode...
             if ( fraction3[LAT]>1 && fraction3[LAT]<999 ) GPS_latitude = GPS_filtered[LAT];
             if ( fraction3[LON]>1 && fraction3[LON]<999 ) GPS_longitude = GPS_filtered[LON];
         } 
    
    #endif
          //dTnav calculation
          //Time for calculating x,y speed and navigation pids
	  dTnav = (float)(millis() - nav_loopTimer)/ 1000.0;
	  nav_loopTimer = millis();
          // prevent runup from bad GPS
	  dTnav = min(dTnav, 1.0);  

          //calculate distance and bearings for gui and other stuff continously
          GPS_distanceToHome = GPS_distance_cm(GPS_latitude,GPS_longitude,GPS_latitude_home,GPS_longitude_home) / 100;
          GPS_directionToHome = GPS_bearing(GPS_latitude_home,GPS_longitude_home,GPS_latitude,GPS_longitude)/100;    //From home to copter
          
          //calculate the current velocity based on gps coordinates continously to get a valid speed at the moment when we start navigating
          GPS_calc_velocity(GPS_latitude,GPS_longitude);        
          
          if (GPSModeHold == 1 || GPSModeHome == 1){    //ok we are navigating 

             //do gps nav calculations here, these are common for nav and poshold  
             wp_distance = GPS_distance_cm(GPS_latitude,GPS_longitude,GPS_WP_latitude,GPS_WP_longitude);
             target_bearing = GPS_bearing(GPS_latitude,GPS_longitude,GPS_WP_latitude,GPS_WP_longitude);
             GPS_calc_location_error(GPS_WP_latitude,GPS_WP_longitude,GPS_latitude,GPS_longitude);

             switch (nav_mode) {
              case NAV_MODE_POSHOLD: 
                //Desired output is in nav_lat and nav_lon where 1deg inclination is 100 
		GPS_calc_poshold(long_error, lat_error);
                break;
              case NAV_MODE_WP:
		int16_t speed = GPS_calc_desired_speed(NAV_SPEED_MAX, NAV_SLOW_NAV);      //slow navigation 
		// use error as the desired rate towards the target
                //Desired output is in nav_lat and nav_lon where 1deg inclination is 100 
		GPS_calc_nav_rate(speed);
	
    	        //Tail control	
                if (NAV_CONTROLS_HEADING) {
                  if (NAV_TAIL_FIRST) {
                      magHold = wrap_18000(nav_bearing-18000)/100;
                  } else {
                      magHold = nav_bearing/100;
                  }
                }
                // Are we there yet ?(within 2 meters of the destination)
		if ((wp_distance <= GPS_wp_radius) || check_missed_wp()){         //if yes switch to poshold mode
                  nav_mode = NAV_MODE_POSHOLD;
                  if (NAV_SET_TAKEOFF_HEADING) { magHold = nav_takeoff_bearing; }
                } 
              break;               
             }
        } //end of gps calcs  
        }
      }
    }

  #endif

  #if defined(GPS_FROM_OSD)
    if(GPS_update) {
      if (GPS_fix  && GPS_numSat > 3) {
        if (GPS_fix_home == 0) {
          GPS_fix_home = 1;
          GPS_latitude_home  = GPS_latitude;
          GPS_longitude_home = GPS_longitude;
        }
        if (GPSModeHold == 1)
          GPS_distance(GPS_latitude_hold,GPS_longitude_hold,GPS_latitude,GPS_longitude, &GPS_distanceToHold, &GPS_directionToHold);
        else
          GPS_distance(GPS_latitude_home,GPS_longitude_home,GPS_latitude,GPS_longitude, &GPS_distanceToHome, &GPS_directionToHome);
        }
        GPS_update = 0;
    }
  #endif // OSD ???
  
}

void GPS_reset_home_position() {
  #if defined(I2C_GPS)
    //set current position as home
    GPS_I2C_command(I2C_GPS_COMMAND_SET_WP,0);  //WP0 is the home position
  #else
    GPS_latitude_home  = GPS_latitude;
    GPS_longitude_home = GPS_longitude;
  #endif
}

//reset navigation (stop the navigation processor, and clear nav_lat and nav_lot
void GPS_reset_nav() {
      GPS_angle[ROLL]  = 0;
      GPS_angle[PITCH] = 0;
         nav_lat = 0;
     nav_lon = 0;
  #if defined(I2C_GPS)
   nav_lat = 0;
   nav_lon = 0;
   //GPS_I2C_command(I2C_GPS_COMMAND_STOP_NAV,0);
  #else
      pi_poshold_lat.reset_I();
      pi_poshold_lon.reset_I();
      pid_poshold_rate_lon.reset_I();
      pid_poshold_rate_lat.reset_I();
      pid_nav_lon.reset_I();
      pid_nav_lat.reset_I();
      nav_lon = 0;
      nav_lat = 0;
  #endif
}

//Get the relevant P I D values and set the PID/PI controllers 
void GPS_set_pids() {
#if defined(GPS_SERIAL)  || defined(GPS_FROM_OSD)
          pi_poshold_lat.kP((float)conf.P8[PIDPOS]/100.0f);
          pi_poshold_lon.kP((float)conf.P8[PIDPOS]/100.0f);
          pi_poshold_lat.kI((float)conf.I8[PIDPOS]/100.0f);
          pi_poshold_lon.kI((float)conf.I8[PIDPOS]/100.0f);
      
          pid_poshold_rate_lat.kP((float)conf.P8[PIDPOSR]/10.0f);
          pid_poshold_rate_lon.kP((float)conf.P8[PIDPOSR]/10.0f);
          pid_poshold_rate_lat.kI((float)conf.I8[PIDPOSR]/100.0f);
          pid_poshold_rate_lon.kI((float)conf.I8[PIDPOSR]/100.0f);
          pid_poshold_rate_lat.kD((float)conf.D8[PIDPOSR]/1000.0f);
          pid_poshold_rate_lon.kD((float)conf.D8[PIDPOSR]/1000.0f);
    
          pid_nav_lat.kP((float)conf.P8[PIDNAVR]/10.0f);
          pid_nav_lon.kP((float)conf.P8[PIDNAVR]/10.0f);
          pid_nav_lat.kI((float)conf.I8[PIDNAVR]/100.0f);
          pid_nav_lon.kI((float)conf.I8[PIDNAVR]/100.0f);
          pid_nav_lat.kD((float)conf.D8[PIDNAVR]/1000.0f);
          pid_nav_lon.kD((float)conf.D8[PIDNAVR]/1000.0f);
#endif

#if defined(I2C_GPS)
          i2c_rep_start(I2C_GPS_ADDRESS);
            i2c_write(I2C_GPS_HOLD_P);
             i2c_write(conf.P8[PIDPOS]);
             i2c_write(conf.I8[PIDPOS]);
          
          i2c_rep_start(I2C_GPS_ADDRESS);
            i2c_write(I2C_GPS_HOLD_RATE_P);
             i2c_write(conf.P8[PIDPOSR]);
             i2c_write(conf.I8[PIDPOSR]);
             i2c_write(conf.D8[PIDPOSR]);
          
          i2c_rep_start(I2C_GPS_ADDRESS);
            i2c_write(I2C_GPS_NAV_P);
             i2c_write(conf.P8[PIDNAVR]);
             i2c_write(conf.I8[PIDNAVR]);
             i2c_write(conf.D8[PIDNAVR]);
          
          GPS_I2C_command(I2C_GPS_COMMAND_UPDATE_PIDS,0);
          
          uint8_t nav_flags = 0;
          if (GPS_FILTERING)          nav_flags += I2C_NAV_FLAG_GPS_FILTER;
          if (GPS_LOW_SPEED_D_FILTER) nav_flags += I2C_NAV_FLAG_LOW_SPEED_D_FILTER; 
          
          i2c_rep_start(I2C_GPS_ADDRESS);
            i2c_write(I2C_GPS_NAV_FLAGS);
            i2c_write(nav_flags);
            
          i2c_rep_start(I2C_GPS_ADDRESS);
            i2c_write(I2C_GPS_WP_RADIUS);
            i2c_write(GPS_WP_RADIUS & 0x00FF); // lower eight bit   
            i2c_write(GPS_WP_RADIUS >> 8); // upper eight bit
#endif
  
}

//OK here is the SERIAL GPS code 
#if defined(GPS_SERIAL) || defined(GPS_FROM_OSD)

////////////////////////////////////////////////////////////////////////////////////
//PID based GPS navigation functions
//Author : EOSBandi
//Based on code and ideas from the Arducopter team: Jason Short,Randy Mackay, Pat Hickey, Jose Julio, Jani Hirvinen
//Andrew Tridgell, Justin Beech, Adam Rivera, Jean-Louis Naudin, Roberto Navoni

////////////////////////////////////////////////////////////////////////////////////
// this is used to offset the shrinking longitude as we go towards the poles	
// It's ok to calculate this once per waypoint setting, since it changes a little within the reach of a multicopter
//
void GPS_calc_longitude_scaling(int32_t lat)
{
	float rads 			= (abs((float)lat) / 10000000.0) * 0.0174532925;
	GPS_scaleLonDown 		= cos(rads);
	GPS_scaleLonUp 		        = 1.0f / cos(rads);
}

////////////////////////////////////////////////////////////////////////////////////
// Sets the waypoint to navigate, reset neccessary variables and calculate initial values
//
void GPS_set_next_wp(int32_t lat, int32_t lon)
{
  
  GPS_WP_latitude = lat;
  GPS_WP_longitude = lon;
  
  GPS_calc_longitude_scaling(lat);
  
  wp_distance = GPS_distance_cm(GPS_latitude,GPS_longitude,GPS_WP_latitude,GPS_WP_longitude);
  target_bearing = GPS_bearing(GPS_latitude,GPS_longitude,GPS_WP_latitude,GPS_WP_longitude);
  nav_bearing = target_bearing;
  GPS_calc_location_error(GPS_WP_latitude,GPS_WP_longitude,GPS_latitude,GPS_longitude);
  original_target_bearing = target_bearing;
  waypoint_speed_gov = NAV_SPEED_MIN;
  
}

////////////////////////////////////////////////////////////////////////////////////
// Check if we missed the destination somehow
//
static bool check_missed_wp()
{
	int32_t temp;
	temp = target_bearing - original_target_bearing;
	temp = wrap_18000(temp);
	return (abs(temp) > 10000);	// we passed the waypoint by 100 degrees
}

////////////////////////////////////////////////////////////////////////////////////
// Get distance between two points in cm
//
uint32_t GPS_distance_cm(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2) {
  float dLat = (lat2 - lat1);                                    // difference of latitude in 1/10 000 000 degrees
  float dLon = (float)(lon2 - lon1) * GPS_scaleLonDown;
  float dist = sqrt(sq(dLat) + sq(dLon)) * 1.113195;
  return dist;
}

////////////////////////////////////////////////////////////////////////////////////
// get bearing from pos1 to pos2, returns an 1deg = 100 precision
//
int32_t GPS_bearing(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2)
{
        float off_x = (float)lon2 - lon1;
        float off_y = ((float)(lat2 - lat1)) * GPS_scaleLonUp;
	float bearing =	9000.0f + atan2(-off_y, off_x) * 5729.57795f;      //Convert the output redians to 100xdeg

        if (bearing < 0) bearing += 36000;
	return bearing;
}

////////////////////////////////////////////////////////////////////////////////////
// keep old calculation function for compatibility (could be removed later) distance in meters, bearing in degree 
//
void GPS_distance(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2, uint16_t* dist, int16_t* bearing) {
 uint32_t distance = GPS_distance_cm(lat1,lon1,lat2,lon2);
 *dist = distance / 100;          //convert to meters
 int32_t bear =  GPS_bearing(lat1,lon1,lat2,lon2);
 *bearing = bear /  100;          //convert to degrees
}

////////////////////////////////////////////////////////////////////////////////////
// Calculata our current speed vector from gps position data
//
static void GPS_calc_velocity( int32_t gps_latitude, int32_t gps_longitude){

	static int16_t x_speed_old = 0;
	static int16_t y_speed_old = 0;

	// y_GPS_speed positve = Up
	// x_GPS_speed positve = Right

	// initialise last_longitude and last_latitude
	if( last_longitude == 0 && last_latitude == 0 ) {
		last_longitude = gps_longitude;
		last_latitude = gps_latitude;
	}

	float tmp = 1.0/dTnav;
	x_actual_speed 	= (float)(gps_longitude - last_longitude) *  GPS_scaleLonDown * tmp;
	y_actual_speed	= (float)(gps_latitude  - last_latitude)  * tmp;

	x_actual_speed	= (x_actual_speed + x_speed_old) / 2;
	y_actual_speed	= (y_actual_speed + y_speed_old) / 2;


	x_speed_old 	= x_actual_speed;
	y_speed_old 	= y_actual_speed;

	last_longitude 	= gps_longitude;
	last_latitude 	= gps_latitude;
}

////////////////////////////////////////////////////////////////////////////////////
// Calculate a location error between two gps coordinates
// Becuase we are using lat and lon to do our distance errors here's a quick chart:
//	100 	= 1m
//	1000 	= 11m	 = 36 feet
//	1800 	= 19.80m = 60 feet
//	3000 	= 33m
//	10000 	= 111m
//
static void GPS_calc_location_error( int32_t target_lat, int32_t target_lng, int32_t gps_lat, int32_t gps_lng )
{
        // X Error
	long_error	= (float)(target_lng - gps_lng) * GPS_scaleLonDown; 
	// Y Error
	lat_error	= target_lat - gps_lat;						
}

////////////////////////////////////////////////////////////////////////////////////
// Calculate nav_lat and nav_lon from the x and y error and the speed
//
static void GPS_calc_poshold(int x_error, int y_error)
{
	int32_t p,i,d;						
	int32_t output;
	int32_t x_target_speed, y_target_speed;

	// East / West
	x_target_speed 	= pi_poshold_lon.get_p(x_error);			// calculate desired speed from lon error
	x_rate_error	= x_target_speed - x_actual_speed;	                // calc the speed error

	p				= pid_poshold_rate_lon.get_p(x_rate_error);
	i				= pid_poshold_rate_lon.get_i(x_rate_error + x_error, dTnav);
	d				= pid_poshold_rate_lon.get_d(x_error, dTnav);
        d                               = constrain(d, -2000, 2000);
        // get rid of noise
#if defined(GPS_LOW_SPEED_D_FILTER)
        if(abs(x_actual_speed) < 50){
          d = 0;
        }
#endif
	output			= p + i + d;
	
        nav_lon			= constrain(output, -NAV_BANK_MAX, NAV_BANK_MAX); 		

	// North / South
	y_target_speed 	= pi_poshold_lat.get_p(y_error);			// calculate desired speed from lat error
	y_rate_error	= y_target_speed - y_actual_speed;

	p				= pid_poshold_rate_lat.get_p(y_rate_error);
	i				= pid_poshold_rate_lat.get_i(y_rate_error + y_error, dTnav);
	d				= pid_poshold_rate_lat.get_d(y_error, dTnav);
        d                               = constrain(d, -2000, 2000);
        // get rid of noise
#if defined(GPS_LOW_SPEED_D_FILTER)
        if(abs(y_actual_speed) < 50){
          d = 0;
        }
#endif
	output			= p + i + d;
	nav_lat			= constrain(output, -NAV_BANK_MAX, NAV_BANK_MAX); 

	// copy over I term to Nav_Rate -- if we change from poshold to RTH this will keep the wind compensation
	pid_nav_lon.set_integrator(pid_poshold_rate_lon.get_integrator());
	pid_nav_lat.set_integrator(pid_poshold_rate_lat.get_integrator());

}
////////////////////////////////////////////////////////////////////////////////////
// Calculate the desired nav_lat and nav_lon for distance flying such as RTH
//
static void GPS_calc_nav_rate(int max_speed)
{
	// push us towards the original track
	GPS_update_crosstrack();

	// nav_bearing includes crosstrack
	float temp 		= (9000l - nav_bearing) * RADX100;

	// East / West
	x_rate_error 	        = (cos(temp) * max_speed) - x_actual_speed; 
	x_rate_error 	        = constrain(x_rate_error, -1000, 1000);
	nav_lon			= pid_nav_lon.get_pid(x_rate_error, dTnav);
	nav_lon			= constrain(nav_lon, -NAV_BANK_MAX, NAV_BANK_MAX);

	// North / South
	y_rate_error 	        = (sin(temp) * max_speed) - y_actual_speed; 
	y_rate_error 	         = constrain(y_rate_error, -1000, 1000);	// added a rate error limit to keep pitching down to a minimum
	nav_lat			= pid_nav_lat.get_pid(y_rate_error, dTnav);
	nav_lat			= constrain(nav_lat, -NAV_BANK_MAX, NAV_BANK_MAX);

	// copy over I term to poshold_rate - So when arriving and entering to poshold we will have the wind compensation
	pid_poshold_rate_lon.set_integrator(pid_nav_lon.get_integrator());
	pid_poshold_rate_lat.set_integrator(pid_nav_lat.get_integrator());

}

////////////////////////////////////////////////////////////////////////////////////
// Calculating cross track error, this tries to keep the copter on a direct line 
// when flying to a waypoint.
//
static void GPS_update_crosstrack(void)
{
	if (abs(wrap_18000(target_bearing - original_target_bearing)) < 4500) {	 // If we are too far off or too close we don't do track following
		float temp = (target_bearing - original_target_bearing) * RADX100;
		crosstrack_error = sin(temp) * (wp_distance * CROSSTRACK_GAIN);	 // Meters we are off track line
		nav_bearing = target_bearing + constrain(crosstrack_error, -3000, 3000);
		nav_bearing = wrap_36000(nav_bearing);
	}else{
		nav_bearing = target_bearing;
	}
}

////////////////////////////////////////////////////////////////////////////////////
// Determine desired speed when navigating towards a waypoint, also implement slow 
// speed rampup when starting a navigation
//
//	|< WP Radius
//	0  1   2   3   4   5   6   7   8m
//	...|...|...|...|...|...|...|...|
//		  100  |  200	  300	  400cm/s
//	           |  		 		            +|+
//	           |< we should slow to 1.5 m/s as we hit the target
//
static int16_t GPS_calc_desired_speed(int16_t max_speed, bool _slow)
{
	// max_speed is default 400 or 4m/s

	if(_slow){
		max_speed 		= min(max_speed, wp_distance / 2);
		max_speed 		= max(max_speed, 0);
	}else{
		max_speed 		= min(max_speed, wp_distance);
		max_speed 		= max(max_speed, NAV_SPEED_MIN);	// go at least 100cm/s
	}

	// limit the ramp up of the speed
	// waypoint_speed_gov is reset to 0 at each new WP command
	if(max_speed > waypoint_speed_gov){
		waypoint_speed_gov += (int)(100.0 * dTnav); // increase at .5/ms
		max_speed = waypoint_speed_gov;
	}

	return max_speed;
}

////////////////////////////////////////////////////////////////////////////////////
// Utilities
//
int32_t wrap_18000(int32_t error)
{
	if (error > 18000)	error -= 36000;
	if (error < -18000)	error += 36000;
	return error;
}

int32_t wrap_36000(int32_t angle)
{
	if (angle > 36000)	angle -= 36000;
	if (angle < 0)		angle += 36000;
	return angle;
}


/* The latitude or longitude is coded this way in NMEA frames
  dddmm.mmmm   coded as degrees + minutes + minute decimal
  This function converts this format in a unique unsigned long where 1 degree = 10 000 000
  I increased the precision here, even if we think that the gps is not precise enough, with 10e5 precision it has 76cm resolution
  with 10e7 it's around 1 cm now. Increasing it further is irrelevant, since even 1cm resolution is unrealistic, however increased 
  resolution also increased precision of nav calculations
*/

#define DIGIT_TO_VAL(_x)	(_x - '0')
uint32_t GPS_coord_to_degrees(char* s)
{
	char *p, *q;
	uint8_t deg = 0, min = 0;
	unsigned int frac_min = 0;

	// scan for decimal point or end of field
	for (p = s; isdigit(*p); p++)
		;
	q = s;

	// convert degrees
	while ((p - q) > 2) {
		if (deg)
			deg *= 10;
		deg += DIGIT_TO_VAL(*q++);
	}
	// convert minutes
	while (p > q) {
		if (min)
			min *= 10;
		min += DIGIT_TO_VAL(*q++);
	}
	// convert fractional minutes
	// expect up to four digits, result is in
	// ten-thousandths of a minute
	if (*p == '.') {
		q = p + 1;
		for (int i = 0; i < 4; i++) {
			frac_min *= 10;
			if (isdigit(*q))
				frac_min += *q++ - '0';
		}
	}
	return deg * 10000000UL + (min * 1000000UL + frac_min*100UL) / 6;
}

// helper functions 
uint16_t grab_fields(char* src, uint8_t mult) {  // convert string to uint16
  uint8_t i;
  uint16_t tmp = 0;
  for(i=0; src[i]!=0; i++) {
    if(src[i] == '.') {
      i++;
      if(mult==0)   break;
      else  src[i+mult] = 0;
    }
    tmp *= 10;
    if(src[i] >='0' && src[i] <='9')	tmp += src[i]-'0';
  }
  return tmp;
}

uint8_t hex_c(uint8_t n) {		// convert '0'..'9','A'..'F' to 0..15
  n -= '0';
  if(n>9)  n -= 7;
  n &= 0x0F;
  return n;
} 

/* This is a light implementation of a GPS frame decoding
   This should work with most of modern GPS devices configured to output NMEA frames.
   It assumes there are some NMEA GGA frames to decode on the serial bus
   Here we use only the following data :
     - latitude
     - longitude
     - GPS fix is/is not ok
     - GPS num sat (4 is enough to be +/- reliable)
     // added by Mis
     - GPS altitude (for OSD displaying)
     - GPS speed (for OSD displaying)
*/
#define FRAME_GGA  1
#define FRAME_RMC  2

bool GPS_newFrame(char c) {
  uint8_t frameOK = 0;
  static uint8_t param = 0, offset = 0, parity = 0;
  static char string[15];
  static uint8_t checksum_param, frame = 0;

  if (c == '$') {
    param = 0; offset = 0; parity = 0;
  } else if (c == ',' || c == '*') {
    string[offset] = 0;
    if (param == 0) { //frame identification
      frame = 0;
      if (string[0] == 'G' && string[1] == 'P' && string[2] == 'G' && string[3] == 'G' && string[4] == 'A') frame = FRAME_GGA;
      if (string[0] == 'G' && string[1] == 'P' && string[2] == 'R' && string[3] == 'M' && string[4] == 'C') frame = FRAME_RMC;
    } else if (frame == FRAME_GGA) {
      if      (param == 2)                     {GPS_latitude = GPS_coord_to_degrees(string);}
      else if (param == 3 && string[0] == 'S') GPS_latitude = -GPS_latitude;
      else if (param == 4)                     {GPS_longitude = GPS_coord_to_degrees(string);}
      else if (param == 5 && string[0] == 'W') GPS_longitude = -GPS_longitude;
      else if (param == 6)                     {GPS_fix = string[0]  > '0' ;}
      else if (param == 7)                     {GPS_numSat = grab_fields(string,0);}
      else if (param == 9)                     {GPS_altitude = grab_fields(string,0);}	// altitude in meters added by Mis
    } else if (frame == FRAME_RMC) {
      if      (param == 7)                     {GPS_speed = ((uint32_t)grab_fields(string,1)*514444L)/100000L;}	// speed in cm/s added by Mis
    }
    param++; offset = 0;
    if (c == '*') checksum_param=1;
    else parity ^= c;
  } else if (c == '\r' || c == '\n') {
    if (checksum_param) { //parity checksum
      uint8_t checksum = hex_c(string[0]);
      checksum <<= 4;
      checksum += hex_c(string[1]);
      if (checksum == parity) frameOK = 1;
    }
    checksum_param=0;
  } else {
     if (offset < 15) string[offset++] = c;
     if (!checksum_param) parity ^= c;
  }
  return frameOK && (frame==FRAME_GGA);
}
#endif //SERIAL GPS

#endif // GPS
