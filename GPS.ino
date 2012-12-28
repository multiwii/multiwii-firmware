#if GPS

#if defined(TINY_GPS)
  #include "tinygps.h"
#endif


#if defined(INIT_MTK_GPS)

#define MTK_SET_BINARY          PSTR("$PGCMD,16,0,0,0,0,0*6A\r\n")
 #define MTK_SET_NMEA            PSTR("$PGCMD,16,1,1,1,1,1*6B\r\n")
 #define MTK_SET_NMEA_SENTENCES  PSTR("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n")
 #define MTK_OUTPUT_4HZ          PSTR("$PMTK220,250*29\r\n")
 #define MTK_OUTPUT_5HZ          PSTR("$PMTK220,200*2C\r\n")
 #define MTK_OUTPUT_10HZ         PSTR("$PMTK220,100*2F\r\n")
 #define MTK_NAVTHRES_OFF        PSTR("$PMTK397,0*23\r\n") // Set Nav Threshold (the minimum speed the GPS must be moving to update the position) to 0 m/s  
 #define SBAS_ON                 PSTR("$PMTK313,1*2E\r\n")
 #define WAAS_ON                 PSTR("$PMTK301,2*2E\r\n")
 #define SBAS_TEST_MODE          PSTR("$PMTK319,0*25\r\n")  //Enable test use of sbas satelite in test mode (usually PRN124 is in test mode)

#endif

#if defined(GPS_SERIAL) || defined(GPS_FROM_OSD) || defined(TINY_GPS)

#if defined(GPS_LEAD_FILTER)
// Set up gps lag
#if defined(UBLOX)
  #define GPS_LAG 0.5f                          //UBLOX GPS has a smaller lag than MTK and other
#else 
  #define GPS_LAG 1.0f                          //We assumes that MTK GPS has a 1 sec lag
#endif  

static int32_t  GPS_coord_lead[2];              // Lead filtered gps coordinates

class LeadFilter {
public:
    LeadFilter() :
        _last_velocity(0) {
    }

    // setup min and max radio values in CLI
    int32_t         get_position(int32_t pos, int16_t vel, float lag_in_seconds = 1.0);
    void            clear() { _last_velocity = 0; }

private:
    int16_t         _last_velocity;

};

int32_t LeadFilter::get_position(int32_t pos, int16_t vel, float lag_in_seconds)
{
    int16_t accel_contribution = (vel - _last_velocity) * lag_in_seconds * lag_in_seconds;
    int16_t vel_contribution = vel * lag_in_seconds;

    // store velocity for next iteration
    _last_velocity = vel;

    return pos + vel_contribution + accel_contribution;
}


LeadFilter xLeadFilter;      // Long GPS lag filter 
LeadFilter yLeadFilter;      // Lat  GPS lag filter 

#endif

  typedef struct PID_PARAM_ {
    float kP;
    float kI;
    float kD;
    float Imax;
  } PID_PARAM;
  
  PID_PARAM posholdPID_PARAM;
  PID_PARAM poshold_ratePID_PARAM;
  PID_PARAM navPID_PARAM;

  typedef struct PID_ {
    float   integrator; // integrator value
    int32_t last_input; // last input for derivative
    float   lastderivative; // last derivative for low-pass filter
    float   output;
    float   derivative;
  } PID;
  PID posholdPID[2];
  PID poshold_ratePID[2];
  PID navPID[2];

  int32_t get_P(int32_t error, struct PID_PARAM_* pid) {
    return (float)error * pid->kP;
  }

  int32_t get_I(int32_t error, float* dt, struct PID_* pid, struct PID_PARAM_* pid_param) {
    pid->integrator += ((float)error * pid_param->kI) * *dt;
    pid->integrator = constrain(pid->integrator,-pid_param->Imax,pid_param->Imax);
    return pid->integrator;
  }
    
  int32_t get_D(int32_t input, float* dt, struct PID_* pid, struct PID_PARAM_* pid_param) { // dt in milliseconds
    pid->derivative = (input - pid->last_input) / *dt;

    /// Low pass filter cut frequency for derivative calculation.
    float filter = 7.9577e-3; // Set to  "1 / ( 2 * PI * f_cut )";
    // Examples for _filter:
    // f_cut = 10 Hz -> _filter = 15.9155e-3
    // f_cut = 15 Hz -> _filter = 10.6103e-3
    // f_cut = 20 Hz -> _filter =  7.9577e-3
    // f_cut = 25 Hz -> _filter =  6.3662e-3
    // f_cut = 30 Hz -> _filter =  5.3052e-3

    // discrete low pass filter, cuts out the
    // high frequency noise that can drive the controller crazy
    pid->derivative = pid->lastderivative + (*dt / ( filter + *dt)) * (pid->derivative - pid->lastderivative);
    // update state
    pid->last_input = input;
    pid->lastderivative    = pid->derivative;
    // add in derivative component
    return pid_param->kD * pid->derivative;
  }

  void reset_PID(struct PID_* pid) {
    pid->integrator = 0;
    pid->last_input = 0;
    pid->lastderivative = 0;
  }

  #define _X 1
  #define _Y 0

  #define RADX100                    0.000174532925  
  #define CROSSTRACK_GAIN            1
  #define NAV_SPEED_MIN              100    // cm/sec
  #define NAV_SPEED_MAX              300    // cm/sec
  #define NAV_SLOW_NAV               true
  #define NAV_BANK_MAX 3000        //30deg max banking when navigating (just for security and testing)

  static float  dTnav;            // Delta Time in milliseconds for navigation computations, updated with every good GPS read
  static uint16_t GPS_wp_radius    = GPS_WP_RADIUS;
  static int16_t actual_speed[2] = {0,0};
  static float GPS_scaleLonDown; // this is used to offset the shrinking longitude as we go towards the poles

  // The difference between the desired rate of travel and the actual rate of travel
  // updated after GPS read - 5-10hz
  static int16_t rate_error[2];
  static int32_t error[2];

  //Currently used WP
  static int32_t GPS_WP[2];

  ////////////////////////////////////////////////////////////////////////////////
  // Location & Navigation
  ////////////////////////////////////////////////////////////////////////////////
  // This is the angle from the copter to the "next_WP" location in degrees * 100
  static int32_t target_bearing;
  ////////////////////////////////////////////////////////////////////////////////
  // Crosstrack
  ////////////////////////////////////////////////////////////////////////////////
  // deg * 100, The original angle to the next_WP when the next_WP was set
  // Also used to check when we pass a WP
  static int32_t original_target_bearing;
  // The amount of angle correction applied to target_bearing to bring the copter back on its optimum path
  static int16_t crosstrack_error;
  ////////////////////////////////////////////////////////////////////////////////
  // The location of the copter in relation to home, updated every GPS read (1deg - 100)
  // static int32_t home_to_copter_bearing; /* unused */
  // distance between plane and home in cm
  // static int32_t home_distance; /* unused */
  // distance between plane and next_WP in cm
  static uint32_t wp_distance;
  
  // used for slow speed wind up when start navigation;
  static uint16_t waypoint_speed_gov;

  ////////////////////////////////////////////////////////////////////////////////////
  // moving average filter variables
  //
  
  #define GPS_FILTER_VECTOR_LENGTH 5
  
  static uint8_t GPS_filter_index = 0;
  static int32_t GPS_filter[2][GPS_FILTER_VECTOR_LENGTH];
  static int32_t GPS_filter_sum[2];
  static int32_t GPS_read[2];
  static int32_t GPS_filtered[2];
  static int32_t GPS_degree[2];    //the lat lon degree without any decimals (lat/10 000 000)
  static uint16_t fraction3[2];
#endif

// This is the angle from the copter to the "next_WP" location
// with the addition of Crosstrack error in degrees * 100
static int32_t nav_bearing;
// saves the bearing at takeof (1deg = 1) used to rotate to takeoff direction when arrives at home
static int16_t nav_takeoff_bearing;


#if defined(I2C_GPS)
  /////////////////////////////////////////////////////////////////////////////////////////
  // I2C GPS helper functions
  //
  // Send a command to the I2C GPS module, first parameter command, second parameter wypoint number
  void GPS_I2C_command(uint8_t command, uint8_t wp) {
    uint8_t _cmd;
      
    _cmd = (wp << 4) + command;
    
    i2c_rep_start(I2C_GPS_ADDRESS<<1);
    i2c_write(I2C_GPS_COMMAND);
    i2c_write(_cmd);
  }
#endif 

#if defined(GPS_SERIAL) 
 #if defined(INIT_MTK_GPS) || defined(UBLOX)
  uint32_t init_speed[5] = {9600,19200,38400,57600,115200};
  void SerialGpsPrint(prog_char* str) {
    char b;
    while(str && (b = pgm_read_byte(str++))) {
      SerialWrite(GPS_SERIAL, b); 
      #if defined(UBLOX)
        delay(5);
      #endif      
    }
  }
 #endif
 #if defined(UBLOX)
   prog_char UBLOX_INIT[] PROGMEM = {                          // PROGMEM array must be outside any function !!!
     0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x05,0x00,0xFF,0x19,                            //disable all default NMEA messages
     0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x03,0x00,0xFD,0x15,
     0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x01,0x00,0xFB,0x11,
     0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x00,0x00,0xFA,0x0F,
     0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x02,0x00,0xFC,0x13,
     0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x04,0x00,0xFE,0x17,
     0xB5,0x62,0x06,0x01,0x03,0x00,0x01,0x02,0x01,0x0E,0x47,                            //set POSLLH MSG rate
     0xB5,0x62,0x06,0x01,0x03,0x00,0x01,0x03,0x01,0x0F,0x49,                            //set STATUS MSG rate
     0xB5,0x62,0x06,0x01,0x03,0x00,0x01,0x06,0x01,0x12,0x4F,                            //set SOL MSG rate
     0xB5,0x62,0x06,0x01,0x03,0x00,0x01,0x12,0x01,0x1E,0x67,                            //set VELNED MSG rate
     0xB5,0x62,0x06,0x16,0x08,0x00,0x03,0x07,0x03,0x00,0x51,0x08,0x00,0x00,0x8A,0x41,   //set WAAS to EGNOS
     0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A //set rate to 5Hz
   };
 #endif

  void GPS_SerialInit() {
    SerialOpen(GPS_SERIAL,GPS_BAUD);  
    delay(1000);
    #if defined(UBLOX)
      for(uint8_t i=0;i<5;i++){
        SerialOpen(GPS_SERIAL,init_speed[i]);          // switch UART speed for sending SET BAUDRATE command (NMEA mode)
        #if (GPS_BAUD==19200)
          SerialGpsPrint(PSTR("$PUBX,41,1,0003,0001,19200,0*23\r\n"));     // 19200 baud - minimal speed for 5Hz update rate
        #endif  
        #if (GPS_BAUD==38400)
          SerialGpsPrint(PSTR("$PUBX,41,1,0003,0001,38400,0*26\r\n"));     // 38400 baud
        #endif  
        #if (GPS_BAUD==57600)
          SerialGpsPrint(PSTR("$PUBX,41,1,0003,0001,57600,0*2D\r\n"));     // 57600 baud
        #endif  
        #if (GPS_BAUD==115200)
          SerialGpsPrint(PSTR("$PUBX,41,1,0003,0001,115200,0*1E\r\n"));    // 115200 baud
        #endif  
        while(!SerialTXfree(GPS_SERIAL)) delay(10);
      }
      delay(200);
      SerialOpen(GPS_SERIAL,GPS_BAUD);  
      for(uint8_t i=0; i<sizeof(UBLOX_INIT); i++) {                        // send configuration data in UBX protocol
        SerialWrite(GPS_SERIAL, pgm_read_byte(UBLOX_INIT+i));
        delay(5); //simulating a 38400baud pace (or less), otherwise commands are not accepted by the device.
      }
    #elif defined(INIT_MTK_GPS)                              // MTK GPS setup
      for(uint8_t i=0;i<5;i++){
        SerialOpen(GPS_SERIAL,init_speed[i]);                // switch UART speed for sending SET BAUDRATE command
        #if (GPS_BAUD==19200)
          SerialGpsPrint(PSTR("$PMTK251,19200*22\r\n"));     // 19200 baud - minimal speed for 5Hz update rate
        #endif  
        #if (GPS_BAUD==38400)
          SerialGpsPrint(PSTR("$PMTK251,38400*27\r\n"));     // 38400 baud
        #endif  
        #if (GPS_BAUD==57600)
          SerialGpsPrint(PSTR("$PMTK251,57600*2C\r\n"));     // 57600 baud
        #endif  
        #if (GPS_BAUD==115200)
          SerialGpsPrint(PSTR("$PMTK251,115200*1F\r\n"));    // 115200 baud
        #endif  
        while(!SerialTXfree(GPS_SERIAL)) delay(80);
      }
      // at this point we have GPS working at selected (via #define GPS_BAUD) baudrate
      // So now we have to set the desired mode and update rate (which depends on the NMEA or MTK_BINARYxx settings)
      SerialOpen(GPS_SERIAL,GPS_BAUD);

      SerialGpsPrint(MTK_NAVTHRES_OFF);
        while(!SerialTXfree(GPS_SERIAL)) delay(80);
      SerialGpsPrint(SBAS_ON);
        while(!SerialTXfree(GPS_SERIAL)) delay(80);
      SerialGpsPrint(WAAS_ON);
        while(!SerialTXfree(GPS_SERIAL)) delay(80);
      SerialGpsPrint(SBAS_TEST_MODE);
        while(!SerialTXfree(GPS_SERIAL)) delay(80);
      SerialGpsPrint(MTK_OUTPUT_5HZ);           // 5 Hz update rate

      #if defined(NMEA)
        SerialGpsPrint(MTK_SET_NMEA_SENTENCES); // only GGA and RMC sentence
      #endif     
      #if defined(MTK_BINARY19) || defined(MTK_BINARY16)
        SerialGpsPrint(MTK_SET_BINARY);
      #endif


#endif  //elif init_mtk_gps
  }
#endif //gps_serial

void GPS_NewData() {
  uint8_t axis;
  #if defined(I2C_GPS)
    static uint8_t GPS_pids_initialized;
    static uint8_t _i2c_gps_status;

    //Do not use i2c_writereg, since writing a register does not work if an i2c_stop command is issued at the end
    //Still investigating, however with separated i2c_repstart and i2c_write commands works... and did not caused i2c errors on a long term test.

    GPS_numSat = (_i2c_gps_status & 0xf0) >> 4;
    _i2c_gps_status = i2c_readReg(I2C_GPS_ADDRESS,I2C_GPS_STATUS_00);                 //Get status register 
    if (_i2c_gps_status & I2C_GPS_STATUS_3DFIX) {                                     //Check is we have a good 3d fix (numsats>5)
      f.GPS_FIX = 1;

      #if !defined(DONT_RESET_HOME_AT_ARM)
        if (!f.ARMED) {f.GPS_FIX_HOME = 0;}                                           //Clear home position if disarmed
      #endif

      if (!f.GPS_FIX_HOME && f.ARMED) {        //if home is not set set home position to WP#0 and activate it
         GPS_reset_home_position();
      }
      if (_i2c_gps_status & I2C_GPS_STATUS_NEW_DATA) {                                //Check about new data
        if (GPS_update) { GPS_update = 0;} else { GPS_update = 1;}                    //Fancy flash on GUI :D
        if (!GPS_pids_initialized) {
          GPS_set_pids();
          GPS_pids_initialized = 1;
        } 
          
        //Read GPS data for distance, heading and gps position 

        i2c_rep_start(I2C_GPS_ADDRESS<<1);
        i2c_write(I2C_GPS_NAV_BEARING);                                               //Start read from here 2x2 bytes distance and direction
        i2c_rep_start((I2C_GPS_ADDRESS<<1)|1);

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

        i2c_rep_start(I2C_GPS_ADDRESS<<1);
        i2c_write(I2C_GPS_LOCATION);                //Start read from here 2x2 bytes distance and direction
        i2c_rep_start((I2C_GPS_ADDRESS<<1)|1);

        varptr = (uint8_t *)&GPS_coord[LAT];        // for latitude displaying
        *varptr++ = i2c_readAck();
        *varptr++ = i2c_readAck();
        *varptr++ = i2c_readAck();
        *varptr   = i2c_readAck();

        varptr = (uint8_t *)&GPS_coord[LON];        // for longitude displaying
        *varptr++ = i2c_readAck();
        *varptr++ = i2c_readAck();
        *varptr++ = i2c_readAck();
        *varptr   = i2c_readAck();

        varptr = (uint8_t *)&nav[LAT];
        *varptr++ = i2c_readAck();
        *varptr++ = i2c_readAck();

        varptr = (uint8_t *)&nav[LON];
        *varptr++ = i2c_readAck();
        *varptr++ = i2c_readNak();
          
        i2c_rep_start(I2C_GPS_ADDRESS<<1);
        i2c_write(I2C_GPS_GROUND_SPEED);          
        i2c_rep_start((I2C_GPS_ADDRESS<<1)|1);

        varptr = (uint8_t *)&GPS_speed;          // speed in cm/s for OSD
        *varptr++ = i2c_readAck();
        *varptr   = i2c_readAck();

        varptr = (uint8_t *)&GPS_altitude;       // altitude in meters for OSD
        *varptr++ = i2c_readAck();
        *varptr   = i2c_readAck();

        //GPS_ground_course
        varptr = (uint8_t *)&GPS_ground_course;
        *varptr++ = i2c_readAck();
        *varptr   = i2c_readNak();

        if (!f.GPS_FIX_HOME) {     //If we don't have home set, do not display anything
          GPS_distanceToHome = 0;
          GPS_directionToHome = 0;
        }

        //Adjust heading when navigating
        if (f.GPS_HOME_MODE) {  
          if ( !(_i2c_gps_status & I2C_GPS_STATUS_WP_REACHED) ) {
            //Tail control
            if (NAV_CONTROLS_HEADING) {
              if (NAV_TAIL_FIRST) {
                magHold = nav_bearing/100-180;
                if (magHold > 180)  magHold -= 360;
                if (magHold < -180) magHold += 360;
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
      f.GPS_FIX = 0;
    }
  #endif     

  #if defined(GPS_SERIAL) || defined(TINY_GPS) || defined(GPS_FROM_OSD)
    #if defined(GPS_SERIAL)
    uint8_t c = SerialAvailable(GPS_SERIAL);
    while (c--) {
    //while (SerialAvailable(GPS_SERIAL)) {
      if (GPS_newFrame(SerialRead(GPS_SERIAL))) {
    #elif defined(TINY_GPS)
    {
      {
      tinygps_query();
    #elif defined(GPS_FROM_OSD)
    {
      if(GPS_update & 2) {  // Once second bit of GPS_update is set, indicate new GPS datas is readed from OSD - all in right format.
        GPS_update &= 1;    // We have: GPS_fix(0-2), GPS_numSat(0-15), GPS_coord[LAT & LON](signed, in 1/10 000 000 degres), GPS_altitude(signed, in meters) and GPS_speed(in cm/s)                     
    #endif
       if (GPS_update == 1) GPS_update = 0; else GPS_update = 1;
        if (f.GPS_FIX && GPS_numSat >= 5) {

          #if !defined(DONT_RESET_HOME_AT_ARM)
            if (!f.ARMED) {f.GPS_FIX_HOME = 0;}
          #endif
          if (!f.GPS_FIX_HOME && f.ARMED) {
            GPS_reset_home_position();
          }

          //Apply moving average filter to GPS data
          #if defined(GPS_FILTERING)
            GPS_filter_index = (GPS_filter_index+1) % GPS_FILTER_VECTOR_LENGTH;
            for (axis = 0; axis< 2; axis++) {
              GPS_read[axis] = GPS_coord[axis]; //latest unfiltered data is in GPS_latitude and GPS_longitude
              GPS_degree[axis] = GPS_read[axis] / 10000000;  // get the degree to assure the sum fits to the int32_t
      
              // How close we are to a degree line ? its the first three digits from the fractions of degree
              // later we use it to Check if we are close to a degree line, if yes, disable averaging,
              fraction3[axis] = (GPS_read[axis]- GPS_degree[axis]*10000000) / 10000;
      
              GPS_filter_sum[axis] -= GPS_filter[axis][GPS_filter_index];
              GPS_filter[axis][GPS_filter_index] = GPS_read[axis] - (GPS_degree[axis]*10000000); 
              GPS_filter_sum[axis] += GPS_filter[axis][GPS_filter_index];
              GPS_filtered[axis] = GPS_filter_sum[axis] / GPS_FILTER_VECTOR_LENGTH + (GPS_degree[axis]*10000000);
              if ( nav_mode == NAV_MODE_POSHOLD) {      //we use gps averaging only in poshold mode...
                if ( fraction3[axis]>1 && fraction3[axis]<999 ) GPS_coord[axis] = GPS_filtered[axis];
              }
            }
          #endif

          //dTnav calculation
          //Time for calculating x,y speed and navigation pids
          static uint32_t nav_loopTimer;
          dTnav = (float)(millis() - nav_loopTimer)/ 1000.0;
          nav_loopTimer = millis();
          // prevent runup from bad GPS
          dTnav = min(dTnav, 1.0);  

          //calculate distance and bearings for gui and other stuff continously - From home to copter
          uint32_t dist;
          int32_t  dir;
          GPS_distance_cm_bearing(&GPS_coord[LAT],&GPS_coord[LON],&GPS_home[LAT],&GPS_home[LON],&dist,&dir);
          GPS_distanceToHome = dist/100;
          GPS_directionToHome = dir/100;

          if (!f.GPS_FIX_HOME) {     //If we don't have home set, do not display anything
             GPS_distanceToHome = 0;
             GPS_directionToHome = 0;
          }
          
          //calculate the current velocity based on gps coordinates continously to get a valid speed at the moment when we start navigating
          GPS_calc_velocity();        
          
          if (f.GPS_HOLD_MODE || f.GPS_HOME_MODE){    //ok we are navigating 
            //do gps nav calculations here, these are common for nav and poshold  
            #if defined(GPS_LEAD_FILTER)
              GPS_distance_cm_bearing(&GPS_coord_lead[LAT],&GPS_coord_lead[LON],&GPS_WP[LAT],&GPS_WP[LON],&wp_distance,&target_bearing);
              GPS_calc_location_error(&GPS_WP[LAT],&GPS_WP[LON],&GPS_coord_lead[LAT],&GPS_coord_lead[LON]);
            #else
              GPS_distance_cm_bearing(&GPS_coord[LAT],&GPS_coord[LON],&GPS_WP[LAT],&GPS_WP[LON],&wp_distance,&target_bearing);
              GPS_calc_location_error(&GPS_WP[LAT],&GPS_WP[LON],&GPS_coord[LAT],&GPS_coord[LON]);
            #endif
            switch (nav_mode) {
              case NAV_MODE_POSHOLD: 
                //Desired output is in nav_lat and nav_lon where 1deg inclination is 100 
                GPS_calc_poshold();
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
}

void GPS_reset_home_position() {
  if (f.GPS_FIX && GPS_numSat >= 5) {
    #if defined(I2C_GPS)
      //set current position as home
      GPS_I2C_command(I2C_GPS_COMMAND_SET_WP,0);  //WP0 is the home position
    #else
      GPS_home[LAT] = GPS_coord[LAT];
      GPS_home[LON] = GPS_coord[LON];
      GPS_calc_longitude_scaling(GPS_coord[LAT]);  //need an initial value for distance and bearing calc
    #endif
    nav_takeoff_bearing = heading;             //save takeoff heading
    //Set ground altitude
    f.GPS_FIX_HOME = 1;
  }
}

//reset navigation (stop the navigation processor, and clear nav)
void GPS_reset_nav() {
  uint8_t i;
  
  for(i=0;i<2;i++) {
    GPS_angle[i]  = 0;
    nav_rated[i] = 0;
    nav[i] = 0;
    #if defined(I2C_GPS)
      //GPS_I2C_command(I2C_GPS_COMMAND_STOP_NAV,0);
    #else
      reset_PID(&posholdPID[i]);
      reset_PID(&poshold_ratePID[i]);
      reset_PID(&navPID[i]);
    #endif
  }
}

//Get the relevant P I D values and set the PID controllers 
void GPS_set_pids() {
  #if defined(GPS_SERIAL)  || defined(GPS_FROM_OSD) || defined(TINY_GPS)
    posholdPID_PARAM.kP   = (float)conf.P8[PIDPOS]/100.0;
    posholdPID_PARAM.kI   = (float)conf.I8[PIDPOS]/100.0;
    posholdPID_PARAM.Imax = POSHOLD_RATE_IMAX * 100;
    
    poshold_ratePID_PARAM.kP   = (float)conf.P8[PIDPOSR]/10.0;
    poshold_ratePID_PARAM.kI   = (float)conf.I8[PIDPOSR]/100.0;
    poshold_ratePID_PARAM.kD   = (float)conf.D8[PIDPOSR]/1000.0;
    poshold_ratePID_PARAM.Imax = POSHOLD_RATE_IMAX * 100;
    
    navPID_PARAM.kP   = (float)conf.P8[PIDNAVR]/10.0;
    navPID_PARAM.kI   = (float)conf.I8[PIDNAVR]/100.0;
    navPID_PARAM.kD   = (float)conf.D8[PIDNAVR]/1000.0;
    navPID_PARAM.Imax = POSHOLD_RATE_IMAX * 100;
  #endif

  #if defined(I2C_GPS)
    i2c_rep_start(I2C_GPS_ADDRESS<<1);
      i2c_write(I2C_GPS_HOLD_P);
       i2c_write(conf.P8[PIDPOS]);
       i2c_write(conf.I8[PIDPOS]);
    
    i2c_rep_start(I2C_GPS_ADDRESS<<1);
      i2c_write(I2C_GPS_HOLD_RATE_P);
       i2c_write(conf.P8[PIDPOSR]);
       i2c_write(conf.I8[PIDPOSR]);
       i2c_write(conf.D8[PIDPOSR]);
    
    i2c_rep_start(I2C_GPS_ADDRESS<<1);
      i2c_write(I2C_GPS_NAV_P);
       i2c_write(conf.P8[PIDNAVR]);
       i2c_write(conf.I8[PIDNAVR]);
       i2c_write(conf.D8[PIDNAVR]);
    
    GPS_I2C_command(I2C_GPS_COMMAND_UPDATE_PIDS,0);
    
    uint8_t nav_flags = 0;
    #if defined(GPS_FILTERING)
      nav_flags += I2C_NAV_FLAG_GPS_FILTER;
    #endif 
    #if defined(GPS_LOW_SPEED_D_FILTER)
      nav_flags += I2C_NAV_FLAG_LOW_SPEED_D_FILTER; 
    #endif
    
    i2c_rep_start(I2C_GPS_ADDRESS<<1);
      i2c_write(I2C_GPS_NAV_FLAGS);
      i2c_write(nav_flags);
      
    i2c_rep_start(I2C_GPS_ADDRESS<<1);
      i2c_write(I2C_GPS_WP_RADIUS);
      i2c_write(GPS_WP_RADIUS & 0x00FF); // lower eight bit   
      i2c_write(GPS_WP_RADIUS >> 8); // upper eight bit
  #endif
}

#if defined (TINY_GPS)
  int32_t GPS_coord_to_decimal(struct coord *c) {
    #define GPS_SCALE_FACTOR 10000000L
    uint32_t deg = 0;
    uint8_t i;
    deg = (uint32_t)c->deg * GPS_SCALE_FACTOR;
  
    uint32_t min = 0;
    min = (uint32_t)c->min * GPS_SCALE_FACTOR;
    /* add up the BCD fractions */
    uint16_t divisor = (uint16_t)GPS_SCALE_FACTOR/10;
    for (i=0; i<NMEA_MINUTE_FRACTS; i++) {
      uint8_t b = c->frac[i/2];
      uint8_t n = (i%2 ? b&0x0F : b>>4);
      min += n*(divisor);
      divisor /= 10;
    }
    /* now sum up degrees and minutes */
    return deg + min/60;
  }
#endif

//It was mobed here since even i2cgps code needs it
int32_t wrap_18000(int32_t ang) {
  if (ang > 18000)  ang -= 36000;
  if (ang < -18000) ang += 36000;
  return ang;
}



//OK here is the onboard GPS code
#if defined(GPS_SERIAL) || defined(GPS_FROM_OSD) || defined(TINY_GPS)

////////////////////////////////////////////////////////////////////////////////////
//PID based GPS navigation functions
//Author : EOSBandi
//Based on code and ideas from the Arducopter team: Jason Short,Randy Mackay, Pat Hickey, Jose Julio, Jani Hirvinen
//Andrew Tridgell, Justin Beech, Adam Rivera, Jean-Louis Naudin, Roberto Navoni

////////////////////////////////////////////////////////////////////////////////////
// this is used to offset the shrinking longitude as we go towards the poles
// It's ok to calculate this once per waypoint setting, since it changes a little within the reach of a multicopter
//
void GPS_calc_longitude_scaling(int32_t lat) {
  float rads       = (abs((float)lat) / 10000000.0) * 0.0174532925;
  GPS_scaleLonDown = cos(rads);
}

////////////////////////////////////////////////////////////////////////////////////
// Sets the waypoint to navigate, reset neccessary variables and calculate initial values
//
void GPS_set_next_wp(int32_t* lat, int32_t* lon) {
  GPS_WP[LAT] = *lat;
  GPS_WP[LON] = *lon;
 
  GPS_calc_longitude_scaling(*lat);
  GPS_distance_cm_bearing(&GPS_coord[LAT],&GPS_coord[LON],&GPS_WP[LAT],&GPS_WP[LON],&wp_distance,&target_bearing);

  nav_bearing = target_bearing;
  GPS_calc_location_error(&GPS_WP[LAT],&GPS_WP[LON],&GPS_coord[LAT],&GPS_coord[LON]);
  original_target_bearing = target_bearing;
  waypoint_speed_gov = NAV_SPEED_MIN;
}

////////////////////////////////////////////////////////////////////////////////////
// Check if we missed the destination somehow
//
static bool check_missed_wp() {
  int32_t temp;
  temp = target_bearing - original_target_bearing;
  temp = wrap_18000(temp);
  return (abs(temp) > 10000);   // we passed the waypoint by 100 degrees
}

////////////////////////////////////////////////////////////////////////////////////
// Get distance between two points in cm
// Get bearing from pos1 to pos2, returns an 1deg = 100 precision
void GPS_distance_cm_bearing(int32_t* lat1, int32_t* lon1, int32_t* lat2, int32_t* lon2,uint32_t* dist, int32_t* bearing) {
  float dLat = *lat2 - *lat1;                                    // difference of latitude in 1/10 000 000 degrees
  float dLon = (float)(*lon2 - *lon1) * GPS_scaleLonDown;
  *dist = sqrt(sq(dLat) + sq(dLon)) * 1.113195;
  
  *bearing = 9000.0f + atan2(-dLat, dLon) * 5729.57795f;      //Convert the output redians to 100xdeg
  if (*bearing < 0) *bearing += 36000;
}

#if defined(OBSOLATED)
////////////////////////////////////////////////////////////////////////////////////
// keep old calculation function for compatibility (could be removed later) distance in meters, bearing in degree 
//
void GPS_distance(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2, uint16_t* dist, int16_t* bearing) {
  uint32_t d1;
  int32_t  d2;
  GPS_distance_cm_bearing(&lat1,&lon1,&lat2,&lon2,&d1,&d2);
  *dist = d1 / 100;          //convert to meters
  *bearing = d2 /  100;      //convert to degrees
}
#endif


//*******************************************************************************************************
// calc_velocity_and_filtered_position - velocity in lon and lat directions calculated from GPS position
//       and accelerometer data
// lon_speed expressed in cm/s.  positive numbers mean moving east
// lat_speed expressed in cm/s.  positive numbers when moving north
// Note: we use gps locations directly to calculate velocity instead of asking gps for velocity because
//       this is more accurate below 1.5m/s
// Note: even though the positions are projected using a lead filter, the velocities are calculated
//       from the unaltered gps locations.  We do not want noise from our lead filter affecting velocity
//*******************************************************************************************************
static void GPS_calc_velocity(){
  static int16_t speed_old[2] = {0,0};
  static int32_t last[2] = {0,0};
  static uint8_t init = 0;

  if (init) {
    float tmp = 1.0/dTnav;
    actual_speed[_X] = (float)(GPS_coord[LON] - last[LON]) *  GPS_scaleLonDown * tmp;
    actual_speed[_Y] = (float)(GPS_coord[LAT]  - last[LAT])  * tmp;

#if !defined(GPS_LEAD_FILTER) 
    actual_speed[_X] = (actual_speed[_X] + speed_old[_X]) / 2;
    actual_speed[_Y] = (actual_speed[_Y] + speed_old[_Y]) / 2;
  
    speed_old[_X] = actual_speed[_X];
    speed_old[_Y] = actual_speed[_Y];

#endif
  }
  init=1;

  last[LON] = GPS_coord[LON];
  last[LAT] = GPS_coord[LAT];

#if defined(GPS_LEAD_FILTER)
  GPS_coord_lead[LON] = xLeadFilter.get_position(GPS_coord[LON], actual_speed[_X], GPS_LAG);
  GPS_coord_lead[LAT] = yLeadFilter.get_position(GPS_coord[LAT], actual_speed[_Y], GPS_LAG);
#endif

}

////////////////////////////////////////////////////////////////////////////////////
// Calculate a location error between two gps coordinates
// Because we are using lat and lon to do our distance errors here's a quick chart:
//   100  = 1m
//  1000  = 11m    = 36 feet
//  1800  = 19.80m = 60 feet
//  3000  = 33m
// 10000  = 111m
//
static void GPS_calc_location_error( int32_t* target_lat, int32_t* target_lng, int32_t* gps_lat, int32_t* gps_lng ) {
  error[LON] = (float)(*target_lng - *gps_lng) * GPS_scaleLonDown;  // X Error
  error[LAT] = *target_lat - *gps_lat; // Y Error
}

////////////////////////////////////////////////////////////////////////////////////
// Calculate nav_lat and nav_lon from the x and y error and the speed
//
static void GPS_calc_poshold() {
  int32_t d;
  int32_t target_speed;
  uint8_t axis;
  
  for (axis=0;axis<2;axis++) {
    target_speed = get_P(error[axis], &posholdPID_PARAM); // calculate desired speed from lat/lon error
    target_speed = constrain(target_speed,-100,100);      // Constrain the target speed in poshold mode to 1m/s it helps avoid runaways..
    rate_error[axis] = target_speed - actual_speed[axis]; // calc the speed error

    nav[axis]      =
        get_P(rate_error[axis],                                               &poshold_ratePID_PARAM)
       +get_I(rate_error[axis] + error[axis], &dTnav, &poshold_ratePID[axis], &poshold_ratePID_PARAM);

    d = get_D(error[axis],                    &dTnav, &poshold_ratePID[axis], &poshold_ratePID_PARAM);

    d = constrain(d, -2000, 2000);
    // get rid of noise
    if(abs(actual_speed[axis]) < 50) d = 0;

    nav[axis] +=d;
    nav[axis]  = constrain(nav[axis], -NAV_BANK_MAX, NAV_BANK_MAX);
    navPID[axis].integrator = poshold_ratePID[axis].integrator;
  }
}
////////////////////////////////////////////////////////////////////////////////////
// Calculate the desired nav_lat and nav_lon for distance flying such as RTH
//
static void GPS_calc_nav_rate(uint16_t max_speed) {
  float trig[2];
  uint8_t axis;
  // push us towards the original track
  GPS_update_crosstrack();

  // nav_bearing includes crosstrack
  float temp = (9000l - nav_bearing) * RADX100;
  trig[_X] = cos(temp);
  trig[_Y] = sin(temp);

  for (axis=0;axis<2;axis++) {
    rate_error[axis] = (trig[axis] * max_speed) - actual_speed[axis]; 
    rate_error[axis] = constrain(rate_error[axis], -1000, 1000);
    // P + I + D
    nav[axis]      =
        get_P(rate_error[axis],                        &navPID_PARAM)
       +get_I(rate_error[axis], &dTnav, &navPID[axis], &navPID_PARAM)
       +get_D(rate_error[axis], &dTnav, &navPID[axis], &navPID_PARAM);

    nav[axis]      = constrain(nav[axis], -NAV_BANK_MAX, NAV_BANK_MAX);
    poshold_ratePID[axis].integrator = navPID[axis].integrator;
  }
}

////////////////////////////////////////////////////////////////////////////////////
// Calculating cross track error, this tries to keep the copter on a direct line 
// when flying to a waypoint.
//
static void GPS_update_crosstrack(void) {
  if (abs(wrap_18000(target_bearing - original_target_bearing)) < 4500) {  // If we are too far off or too close we don't do track following
    float temp = (target_bearing - original_target_bearing) * RADX100;
    crosstrack_error = sin(temp) * (wp_distance * CROSSTRACK_GAIN);  // Meters we are off track line
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
//      |< WP Radius
//      0  1   2   3   4   5   6   7   8m
//      ...|...|...|...|...|...|...|...|
//                100  |  200     300     400cm/s
//                 |                                        +|+
//                 |< we should slow to 1.5 m/s as we hit the target
//
static uint16_t GPS_calc_desired_speed(uint16_t max_speed, bool _slow) {
  // max_speed is default 400 or 4m/s
  if(_slow){
    max_speed = min(max_speed, wp_distance / 2);
    //max_speed = max(max_speed, 0);
  }else{
    max_speed = min(max_speed, wp_distance);
    max_speed = max(max_speed, NAV_SPEED_MIN);  // go at least 100cm/s
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

int32_t wrap_36000(int32_t ang) {
  if (ang > 36000) ang -= 36000;
  if (ang < 0)     ang += 36000;
  return ang;
}

// This code is used for parsing NMEA data
#if defined(GPS_SERIAL)

/* Alex optimization 
  The latitude or longitude is coded this way in NMEA frames
  dm.f   coded as degrees + minutes + minute decimal
  Where:
    - d can be 1 or more char long. generally: 2 char long for latitude, 3 char long for longitude
    - m is always 2 char long
    - f can be 1 or more char long
  This function converts this format in a unique unsigned long where 1 degree = 10 000 000

  EOS increased the precision here, even if we think that the gps is not precise enough, with 10e5 precision it has 76cm resolution
  with 10e7 it's around 1 cm now. Increasing it further is irrelevant, since even 1cm resolution is unrealistic, however increased 
  resolution also increased precision of nav calculations
*/
/* This calc adds approc 2km error to the gps coordinates reverted back to the original working one 
uint32_t GPS_coord_to_degrees(char* s) {
  char *p = s, *d = s;
  uint8_t min, deg = 0;
  uint16_t frac = 0, mult = 10000;
  
  while(*p) {                                   // parse the string until its end
    if (d != s) {frac+=(*p-'0')*mult;mult/=10;} // calculate only fractional part on up to 5 digits  (d != s condition is true when the . is located)
    if (*p == '.') d=p;                         // locate '.' char in the string
    p++;
  }
  if (p==s) return 0;
  while (s<d-2) {deg *= 10;deg += *(s++)-'0';}  // convert degrees : all chars before minutes ; for the first iteration, deg = 0
  min = *(d-1)-'0' + (*(d-2)-'0')*10;           // convert minutes : 2 previous char before '.'
  return deg * 10000000UL + (min * 100000UL + frac)*10UL / 6;
}
*/

#define DIGIT_TO_VAL(_x)        (_x - '0')
uint32_t GPS_coord_to_degrees(char* s) {
  char *p, *q;
  uint8_t deg = 0, min = 0;
  unsigned int frac_min = 0;
  uint8_t i;

  // scan for decimal point or end of field
  for (p = s; isdigit(*p); p++) ;
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
    for (i = 0; i < 4; i++) {
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
    if(src[i] >='0' && src[i] <='9') tmp += src[i]-'0';
  }
  return tmp;
}

uint8_t hex_c(uint8_t n) {    // convert '0'..'9','A'..'F' to 0..15
  n -= '0';
  if(n>9)  n -= 7;
  n &= 0x0F;
  return n;
} 

bool GPS_newFrame(char c) {
  #if defined(NMEA)
    return GPS_NMEA_newFrame(c);
  #endif
  #if defined(UBLOX)
    return GPS_UBLOX_newFrame(c);
  #endif
  #if defined(MTK_BINARY16) || defined(MTK_BINARY19)
    return GPS_MTK_newFrame(c);
  #endif
}

#if defined(NMEA)
  /* This is a light implementation of a GPS frame decoding
     This should work with most of modern GPS devices configured to output NMEA frames.
     It assumes there are some NMEA GGA frames to decode on the serial bus
     Here we use only the following data :
       - latitude
       - longitude
       - GPS fix is/is not ok
       - GPS num sat (4 is enough to be +/- reliable)
       - GPS altitude
       - GPS speed
  */
  #define FRAME_GGA  1
  #define FRAME_RMC  2
  
  bool GPS_NMEA_newFrame(char c) {
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
        if      (param == 2)                     {GPS_coord[LAT] = GPS_coord_to_degrees(string);}
        else if (param == 3 && string[0] == 'S') GPS_coord[LAT] = -GPS_coord[LAT];
        else if (param == 4)                     {GPS_coord[LON] = GPS_coord_to_degrees(string);}
        else if (param == 5 && string[0] == 'W') GPS_coord[LON] = -GPS_coord[LON];
        else if (param == 6)                     {f.GPS_FIX = (string[0]  > '0');}
        else if (param == 7)                     {GPS_numSat = grab_fields(string,0);}
        else if (param == 9)                     {GPS_altitude = grab_fields(string,0);}  // altitude in meters added by Mis
      } else if (frame == FRAME_RMC) {
        if      (param == 7)                     {GPS_speed = ((uint32_t)grab_fields(string,1)*5144L)/1000L;}  //gps speed in cm/s will be used for navigation
        else if (param == 8)                     {GPS_ground_course = grab_fields(string,1); }                 //ground course deg*10 
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
    if (frame) GPS_Present = 1;
    return frameOK && (frame==FRAME_GGA);
  }
#endif //NMEA

#if defined(UBLOX)
  struct ubx_header {
    uint8_t preamble1;
    uint8_t preamble2;
    uint8_t msg_class;
    uint8_t msg_id;
    uint16_t length;
  };
  struct ubx_nav_posllh {
    uint32_t time;  // GPS msToW
    int32_t longitude;
    int32_t latitude;
    int32_t altitude_ellipsoid;
    int32_t altitude_msl;
    uint32_t horizontal_accuracy;
    uint32_t vertical_accuracy;
  };
  struct ubx_nav_solution {
    uint32_t time;
    int32_t time_nsec;
    int16_t week;
    uint8_t fix_type;
    uint8_t fix_status;
    int32_t ecef_x;
    int32_t ecef_y;
    int32_t ecef_z;
    uint32_t position_accuracy_3d;
    int32_t ecef_x_velocity;
    int32_t ecef_y_velocity;
    int32_t ecef_z_velocity;
    uint32_t speed_accuracy;
    uint16_t position_DOP;
    uint8_t res;
    uint8_t satellites;
    uint32_t res2;
  };
  struct ubx_nav_velned {
    uint32_t time;  // GPS msToW
    int32_t ned_north;
    int32_t ned_east;
    int32_t ned_down;
    uint32_t speed_3d;
    uint32_t speed_2d;
    int32_t heading_2d;
    uint32_t speed_accuracy;
    uint32_t heading_accuracy;
  };
  
  enum ubs_protocol_bytes {
    PREAMBLE1 = 0xb5,
    PREAMBLE2 = 0x62,
    CLASS_NAV = 0x01,
    CLASS_ACK = 0x05,
    CLASS_CFG = 0x06,
    MSG_ACK_NACK = 0x00,
    MSG_ACK_ACK = 0x01,
    MSG_POSLLH = 0x2,
    MSG_STATUS = 0x3,
    MSG_SOL = 0x6,
    MSG_VELNED = 0x12,
    MSG_CFG_PRT = 0x00,
    MSG_CFG_RATE = 0x08,
    MSG_CFG_SET_RATE = 0x01,
    MSG_CFG_NAV_SETTINGS = 0x24
  };
  enum ubs_nav_fix_type {
    FIX_NONE = 0,
    FIX_DEAD_RECKONING = 1,
    FIX_2D = 2,
    FIX_3D = 3,
    FIX_GPS_DEAD_RECKONING = 4,
    FIX_TIME = 5
  };
  enum ubx_nav_status_bits {
    NAV_STATUS_FIX_VALID = 1
  };
  
  // Packet checksum accumulators
  static uint8_t _ck_a;
  static uint8_t _ck_b;
  
  // State machine state
  static uint8_t _step;
  static uint8_t _msg_id;
  static uint16_t _payload_length;
  static uint16_t _payload_counter;
  
//  static bool next_fix;
  static uint8_t _class;

  static uint8_t _disable_counter;
  static uint8_t _fix_ok;
  
  // Receive buffer
  static union {
    ubx_nav_posllh posllh;
//    ubx_nav_status status;
    ubx_nav_solution solution;
    ubx_nav_velned velned;
    uint8_t bytes[];
   } _buffer;
  
  void _update_checksum(uint8_t *data, uint8_t len, uint8_t &ck_a, uint8_t &ck_b) {
    while (len--) {
      ck_a += *data;
      ck_b += ck_a;
      data++;
    }
  }
  
  bool GPS_UBLOX_newFrame(uint8_t data){
    bool parsed = false;

    switch(_step) {
      case 1:
        if (PREAMBLE2 == data) {
          _step++;
          break;
        }
        _step = 0;
      case 0:
        if(PREAMBLE1 == data) _step++;
        break;
      case 2:
        _step++;
        _class = data;
        _ck_b = _ck_a = data;  // reset the checksum accumulators
        break;
      case 3:
        _step++;
        _ck_b += (_ck_a += data);  // checksum byte
        _msg_id = data;
        break;
      case 4:
        _step++;
        _ck_b += (_ck_a += data);  // checksum byte
        _payload_length = data;  // payload length low byte
        break;
      case 5:
        _step++;
        _ck_b += (_ck_a += data);  // checksum byte
        _payload_length += (uint16_t)(data<<8);
        if (_payload_length > 512) {
          _payload_length = 0;
          _step = 0;
        }
        _payload_counter = 0;  // prepare to receive payload
      break;
      case 6:
        _ck_b += (_ck_a += data);  // checksum byte
        if (_payload_counter < sizeof(_buffer)) {
          _buffer.bytes[_payload_counter] = data;
        }
        if (++_payload_counter == _payload_length)
          _step++;
        break;
      case 7:
        _step++;
        if (_ck_a != data) _step = 0;  // bad checksum
      break;
      case 8:
        _step = 0;
        if (_ck_b != data)  break;  // bad checksum
        GPS_Present = 1;
        if (UBLOX_parse_gps())  { parsed = true; }
    } //end switch
    return parsed;
  }

  bool UBLOX_parse_gps(void) {
    switch (_msg_id) {
    case MSG_POSLLH:
      //i2c_dataset.time                = _buffer.posllh.time;
      if(_fix_ok) {
        GPS_coord[LON] = _buffer.posllh.longitude;
        GPS_coord[LAT] = _buffer.posllh.latitude;
        GPS_altitude   = _buffer.posllh.altitude_msl / 1000;      //alt in m
      }
      f.GPS_FIX = _fix_ok;
      return true;        // POSLLH message received, allow blink GUI icon and LED
      break;
    case MSG_SOL:
      _fix_ok = 0;
      if((_buffer.solution.fix_status & NAV_STATUS_FIX_VALID) && (_buffer.solution.fix_type == FIX_3D || _buffer.solution.fix_type == FIX_2D)) _fix_ok = 1;
      GPS_numSat = _buffer.solution.satellites;
      break;
    case MSG_VELNED:
      GPS_speed         = _buffer.velned.speed_2d;  // cm/s
      GPS_ground_course = (uint16_t)(_buffer.velned.heading_2d / 10000);  // Heading 2D deg * 100000 rescaled to deg * 10
      break;
    default:
      break;
    }
    return false;
  }
#endif //UBLOX

#if defined(MTK_BINARY16) || defined(MTK_BINARY19)

  struct diyd_mtk_msg {
        int32_t  latitude;
        int32_t  longitude;
        int32_t  altitude;
        int32_t  ground_speed;
        int32_t  ground_course;
        uint8_t  satellites;
        uint8_t  fix_type;
        uint32_t utc_date;
        uint32_t utc_time;
        uint16_t hdop;
    };
// #pragma pack(pop)
    enum diyd_mtk_fix_type {
       FIX_NONE = 1,
       FIX_2D = 2,
       FIX_3D = 3,
       FIX_2D_SBAS = 6,
       FIX_3D_SBAS = 7 
    };

#if defined(MTK_BINARY16)
    enum diyd_mtk_protocol_bytes {
        PREAMBLE1 = 0xd0,
        PREAMBLE2 = 0xdd,
    };
#endif

#if defined(MTK_BINARY19)
    enum diyd_mtk_protocol_bytes {
        PREAMBLE1 = 0xd1,
        PREAMBLE2 = 0xdd,
    };
#endif

    // Packet checksum accumulators
    uint8_t  _ck_a;
    uint8_t  _ck_b;

    // State machine state
    uint8_t  _step;
    uint8_t  _payload_counter;

    // Time from UNIX Epoch offset
    long  _time_offset;
    bool  _offset_calculated;

    // Receive buffer
    union {
        diyd_mtk_msg  msg;
        uint8_t       bytes[];
    } _buffer;

inline long _swapl(const void *bytes)
{
    const uint8_t *b = (const uint8_t *)bytes;
    union {
        long    v;
        uint8_t b[4];
    } u;

    u.b[0] = b[3];
    u.b[1] = b[2];
    u.b[2] = b[1];
    u.b[3] = b[0];

    return(u.v);
}

bool GPS_MTK_newFrame(uint8_t data)
{
       bool parsed = false;

restart:
        switch(_step) {

            // Message preamble, class, ID detection
            //
            // If we fail to match any of the expected bytes, we
            // reset the state machine and re-consider the failed
            // byte as the first byte of the preamble.  This
            // improves our chances of recovering from a mismatch
            // and makes it less likely that we will be fooled by
            // the preamble appearing as data in some other message.
            //
        case 0:
            if(PREAMBLE1 == data)
                _step++;
            break;
        case 1:
            if (PREAMBLE2 == data) {
                _step++;
                break;
            }
            _step = 0;
            goto restart;
        case 2:
            if (sizeof(_buffer) == data) {
                _step++;
                _ck_b = _ck_a = data;                  // reset the checksum accumulators
                _payload_counter = 0;
            } else {
                _step = 0;                             // reset and wait for a message of the right class
                goto restart;
            }
            break;

            // Receive message data
            //
        case 3:
            _buffer.bytes[_payload_counter++] = data;
            _ck_b += (_ck_a += data);
            if (_payload_counter == sizeof(_buffer))
                _step++;
            break;

            // Checksum and message processing
            //
        case 4:
            _step++;
            if (_ck_a != data) {
                _step = 0;
            }
            break;
        case 5:
            _step = 0;
            if (_ck_b != data) {
                break;
            }

            f.GPS_FIX                   = ((_buffer.msg.fix_type == FIX_3D) || (_buffer.msg.fix_type == FIX_3D_SBAS));

#if defined(MTK_BINARY16)
            GPS_coord[LAT]              = _buffer.msg.latitude * 10;    // XXX doc says *10e7 but device says otherwise
            GPS_coord[LON]              = _buffer.msg.longitude * 10;   // XXX doc says *10e7 but device says otherwise
#endif
#if defined(MTK_BINARY19)
            GPS_coord[LAT]              = _buffer.msg.latitude;         // With 1.9 now we have real 10e7 precision
            GPS_coord[LON]              = _buffer.msg.longitude;
#endif
            GPS_altitude                = _buffer.msg.altitude /100;    // altitude in meter
            GPS_speed                   = _buffer.msg.ground_speed;     // in m/s * 100 == in cm/s
            GPS_ground_course           = _buffer.msg.ground_course/100;  //in degrees
            GPS_numSat                  = _buffer.msg.satellites;
            //GPS_hdop                  = _buffer.msg.hdop;
            parsed = true;
            GPS_Present = 1;
        }
    return parsed;
}
#endif //MTK



#endif //SERIAL GPS
#endif //ONBOARD GPS CALC

#endif // GPS
