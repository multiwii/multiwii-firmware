#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "GPS.h"
#include "Serial.h"
#include "Sensors.h"
#include "MultiWii.h"
#include "EEPROM.h"
#include <math.h>



#if GPS

//Function prototypes for GPS frame parsing
bool GPS_newFrame(char c);

#if defined(NMEA)
  bool GPS_NMEA_newFrame(char c);
#endif
#if defined(UBLOX)
  bool GPS_UBLOX_newFrame(uint8_t data);
  bool UBLOX_parse_gps(void);
#endif
#if defined(MTK_BINARY16) || defined(MTK_BINARY19)
  bool GPS_MTK_newFrame(uint8_t data);
#endif


//Function prototypes for other GPS functions
//These perhaps could go to the gps.h file, however these are local to the gps.cpp  
void GPS_bearing(int32_t* lat1, int32_t* lon1, int32_t* lat2, int32_t* lon2, int32_t* bearing);
void GPS_distance_cm(int32_t* lat1, int32_t* lon1, int32_t* lat2, int32_t* lon2,uint32_t* dist);
static void GPS_calc_velocity(void);
static void GPS_calc_location_error( int32_t* target_lat, int32_t* target_lng, int32_t* gps_lat, int32_t* gps_lng );
static void GPS_calc_poshold(void);
static uint16_t GPS_calc_desired_speed(uint16_t max_speed, bool _slow);
static void GPS_calc_nav_rate(uint16_t max_speed);
int32_t wrap_18000(int32_t ang);
static bool check_missed_wp(void);
void GPS_calc_longitude_scaling(int32_t lat);
static void GPS_update_crosstrack(void);
int32_t wrap_36000(int32_t ang);




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

// Leadig filter - TODO: rewrite to normal C instead of C++

// Set up gps lag
#if defined(UBLOX) || defined (MTK_BINARY19)
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

  uint8_t land_detect;							//Detect land (extern)
  static uint32_t land_settle_timer;
  static uint32_t GPS_last_frame_seen; //Last gps frame seen at this time, used to detect stalled gps communication

  static float  dTnav;            // Delta Time in milliseconds for navigation computations, updated with every good GPS read
  static int16_t actual_speed[2] = {0,0};
  static float GPS_scaleLonDown; // this is used to offset the shrinking longitude as we go towards the poles

  // The difference between the desired rate of travel and the actual rate of travel
  // updated after GPS read - 5-10hz
  static int16_t rate_error[2];
  static int32_t error[2];

  static int32_t GPS_WP[2];						  //Currently used WP
  static int32_t GPS_FROM[2];					  //the pervious waypoint for precise track following
  int32_t target_bearing;						  // This is the angle from the copter to the "next_WP" location in degrees * 100
  static int32_t original_target_bearing;		  // deg * 100, The original angle to the next_WP when the next_WP was set, Also used to check when we pass a WP
  static int16_t crosstrack_error;				  // The amount of angle correction applied to target_bearing to bring the copter back on its optimum path
  uint32_t wp_distance;							  // distance between plane and next_WP in cm
  static uint16_t waypoint_speed_gov;			  // used for slow speed wind up when start navigation;


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

  static int16_t nav_takeoff_bearing;			  // saves the bearing at takeof (1deg = 1) used to rotate to takeoff direction when arrives at home


//Serial GPS initialization
#if defined(GPS_SERIAL) 
 #if defined(INIT_MTK_GPS) || defined(UBLOX)
  uint32_t init_speed[5] = {9600,19200,38400,57600,115200};
  void SerialGpsPrint(const char PROGMEM * str) {
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
   const prog_char UBLOX_INIT[] PROGMEM = {                          // PROGMEM array must be outside any function !!!
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
  void GPS_SerialInit(void) {
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
#endif 

//Main GPS nav loop. Called by the task scheduler
uint8_t GPS_NewData(void) {
      uint8_t c = SerialAvailable(GPS_SERIAL);
	  if (c==0) return 0;							//if we don't have char's to read, so return with a zero
      	  while (c--) {
		   if (GPS_newFrame(SerialRead(GPS_SERIAL))) {
			  //We had a valid GPS data frame, so signal task scheduler to switch to compute
			  if (GPS_update == 1) GPS_update = 0; else GPS_update = 1;
			  GPS_last_frame_seen = millis();
			  GPS_Frame = 1;
		   }
	      }
	  // Check for stalled GPS, if no frames seen for 1.2sec then consider it LOST
	  if ((millis() - GPS_last_frame_seen) > 1200)
	  {
		  //No update since 1200ms clear fix...
		  f.GPS_FIX = 0;
		  GPS_numSat = 0;
	  }
	  return 0;
}


//Main navigation processor and state engine
// TODO: add proceesing states to ease processing burden 
uint8_t GPS_Compute(void) 
{
	unsigned char axis;
	uint32_t dist;        //temp variable to store dist to copter
	int32_t  dir;         //temp variable to store dir to copter
	static uint32_t nav_loopTimer;

	//check that we have a valid frame, if not then return immediatly
    if (GPS_Frame == 0) return 0; else GPS_Frame = 0;

    if (GPS_update == 1) GPS_update = 0; else GPS_update = 1;	//Blink GPS update
  
    //check home position and set it if it was not set
    if (f.GPS_FIX && GPS_numSat >= 5) {
      #if !defined(DONT_RESET_HOME_AT_ARM)
         if (!f.ARMED) {f.GPS_FIX_HOME = 0;}
      #endif
      if (!f.GPS_FIX_HOME && f.ARMED) {
        GPS_reset_home_position();
      }


    //Apply moving average filter to GPS data
	if (GPS_conf.filtering)
	{
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
			if ( NAV_state == NAV_STATE_HOLD_INFINIT || NAV_state == NAV_STATE_HOLD_TIMED) {      //we use gps averaging only in poshold mode...
				if ( fraction3[axis]>1 && fraction3[axis]<999 ) GPS_coord[axis] = GPS_filtered[axis];
			}
		}
	}

    //dTnav calculation
    //Time for calculating x,y speed and navigation pids
    dTnav = (float)(millis() - nav_loopTimer)/ 1000.0;
    nav_loopTimer = millis();

    // prevent runup from bad GPS
    dTnav = min(dTnav, 1.0);  

    //calculate distance and bearings for gui and other stuff continously - From home to copter
    GPS_bearing(&GPS_coord[LAT],&GPS_coord[LON],&GPS_home[LAT],&GPS_home[LON],&dir);
    GPS_distance_cm(&GPS_coord[LAT],&GPS_coord[LON],&GPS_home[LAT],&GPS_home[LON],&dist);
    GPS_distanceToHome = dist/100;
    GPS_directionToHome = dir/100;

    if (!f.GPS_FIX_HOME) {     //If we don't have home set, do not display anything
      GPS_distanceToHome = 0;
      GPS_directionToHome = 0;
    }

    //Check fence setting and execute RTH if neccessary
    //TODO: autolanding
    if ((GPS_conf.fence > 0) && (GPS_conf.fence < GPS_distanceToHome) && (f.GPS_mode != GPS_MODE_RTH) )
      {
      init_RTH();
      }

    //calculate the current velocity based on gps coordinates continously to get a valid speed at the moment when we start navigating
    GPS_calc_velocity();        

    //Navigation state engine
    if (f.GPS_mode != GPS_MODE_NONE)    //ok we are navigating ###0002 
      { 

      //do gps nav calculations here, these are common for nav and poshold  
      GPS_bearing(&GPS_coord[LAT],&GPS_coord[LON],&GPS_WP[LAT],&GPS_WP[LON],&target_bearing);
      if (GPS_conf.lead_filter)
        {
        GPS_distance_cm(&GPS_coord_lead[LAT],&GPS_coord_lead[LON],&GPS_WP[LAT],&GPS_WP[LON],&wp_distance);
        GPS_calc_location_error(&GPS_WP[LAT],&GPS_WP[LON],&GPS_coord_lead[LAT],&GPS_coord_lead[LON]);
        }
      else
        {
        GPS_distance_cm(&GPS_coord[LAT],&GPS_coord[LON],&GPS_WP[LAT],&GPS_WP[LON],&wp_distance);
        GPS_calc_location_error(&GPS_WP[LAT],&GPS_WP[LON],&GPS_coord[LAT],&GPS_coord[LON]);
        }

      // Adjust altitude 
      // if we are holding position and reached target altitude, then ignore altitude nav, and let the user trim alt
      if ( !((NAV_state == NAV_STATE_HOLD_INFINIT) && (alt_change_flag == REACHED_ALT)))
        {
        if (!f.LAND_IN_PROGRESS) {
          alt_to_hold = get_new_altitude();
          AltHold = alt_to_hold;
          }
        }

      int16_t speed = 0;										//Desired navigation speed

      switch(NAV_state)										//Navigation state machine
        {
        case NAV_STATE_NONE:									//Just for clarity, do nothing when nav_state is none
          break;

        case NAV_STATE_LAND_START:
          GPS_calc_poshold();									//Land in position hold 			
          land_settle_timer = millis();
          NAV_state = NAV_STATE_LAND_SETTLE;	       
          break;

        case NAV_STATE_LAND_SETTLE:
          GPS_calc_poshold();
          if (millis()-land_settle_timer > 5000)
            NAV_state = NAV_STATE_LAND_START_DESCENT;
          break;

        case NAV_STATE_LAND_START_DESCENT:
          GPS_calc_poshold();									//Land in position hold 			
          f.THROTTLE_IGNORED = 1;								//Ignore Throtte stick input
          f.GPS_BARO_MODE    = 1;								//Take control of BARO mode
          land_detect = 0;									    //Reset land detector
          f.LAND_COMPLETED = 0;								
          f.LAND_IN_PROGRESS = 1;								// Flag land process
          NAV_state = NAV_STATE_LAND_IN_PROGRESS;
          break;

        case NAV_STATE_LAND_IN_PROGRESS:
          GPS_calc_poshold();
          check_land();									        //Call land detector
          if (f.LAND_COMPLETED) {
            nav_timer_stop = millis() + 5000;
            NAV_state = NAV_STATE_LANDED;
            }
          break;

        case NAV_STATE_LANDED:
          // Disarm if THROTTLE stick is at minimum or 5sec past after land detected
          if (rcData[THROTTLE]<MINCHECK || nav_timer_stop <= millis())	//Throttle at minimum or 5sec passed.
            {
            go_disarm();
            f.OK_TO_ARM = 0;								//Prevent rearming
            NAV_state = NAV_STATE_NONE;						//Disable position holding.... prevent flippover
            f.GPS_BARO_MODE = 0;
            f.LAND_COMPLETED = 0;
            f.LAND_IN_PROGRESS = 0;
            land_detect = 0;
            f.THROTTLE_IGNORED = 0;
            GPS_reset_nav();
            }
          break;

        case NAV_STATE_HOLD_INFINIT:							//Constant position hold, no timer. Only an rcOption change can exit from this
          GPS_calc_poshold();
          break;

        case NAV_STATE_HOLD_TIMED:
          if (nav_timer_stop == 0)							//We are start a timed poshold
            {
            nav_timer_stop = millis() + 1000*nav_hold_time;  //Set when we will continue
            } 
          else if (nav_timer_stop <= millis())				//did we reach our time limit ?
            {
            if (mission_step.flag != MISSION_FLAG_END)
              {
              NAV_state = NAV_STATE_PROCESS_NEXT;		   //if yes then process next mission step
              }
            NAV_error = NAV_ERROR_TIMEWAIT;
            }
          GPS_calc_poshold();									//BTW hold position till next command
          break;

        case NAV_STATE_RTH_START:								
          if ((alt_change_flag == REACHED_ALT) || (!GPS_conf.wait_for_rth_alt))						//Wait until we reach RTH altitude
            {
            GPS_set_next_wp(&GPS_home[LAT],&GPS_home[LON], &GPS_coord[LAT], &GPS_coord[LON]);	//If we reached then change mode and start RTH
            NAV_state = NAV_STATE_RTH_ENROUTE;
            NAV_error = NAV_ERROR_NONE;
            } 
          else
            {
            GPS_calc_poshold();								//hold position till we reach RTH alt
            NAV_error = NAV_ERROR_WAIT_FOR_RTH_ALT;
            }
          break;

        case NAV_STATE_RTH_ENROUTE:								//Doing RTH navigation
          speed = GPS_calc_desired_speed(GPS_conf.nav_speed_max, GPS_conf.slow_nav); 
          GPS_calc_nav_rate(speed);
          GPS_adjust_heading();
          if ((wp_distance <= GPS_conf.wp_radius) || check_missed_wp())
            {         //if yes switch to poshold mode

            if (mission_step.parameter1 == 0) NAV_state = NAV_STATE_HOLD_INFINIT;
            else NAV_state = NAV_STATE_LAND_START;									// if parameter 1 in RTH step is non 0 then land at home
            if (GPS_conf.nav_rth_takeoff_heading) { magHold = nav_takeoff_bearing; }
            } 
          break;

        case NAV_STATE_WP_ENROUTE:
          speed = GPS_calc_desired_speed(GPS_conf.nav_speed_max, GPS_conf.slow_nav); 
          GPS_calc_nav_rate(speed);
          GPS_adjust_heading();

          if ((wp_distance <= GPS_conf.wp_radius) || check_missed_wp())			//This decides what happen when we reached the WP coordinates
            {         
            if (mission_step.action == MISSION_LAND)					//Autoland
              {
              NAV_state = NAV_STATE_LAND_START;							//Start landing
              set_new_altitude(alt.EstAlt);								//Stop any altitude changes
              }
            else if (mission_step.flag == MISSION_FLAG_END)						//If this was the last mission step (flag set by the mission planner), then switch to poshold
              {
              NAV_state = NAV_STATE_HOLD_INFINIT;
              NAV_error = NAV_ERROR_FINISH;
              }
            else if (mission_step.action == MISSION_HOLD_UNLIM)				//If mission_step was POSHOLD_UNLIM and we reached the position then switch to poshold unlimited
              {
              NAV_state = NAV_STATE_HOLD_INFINIT;
              NAV_error = NAV_ERROR_FINISH;

              }
            else if (mission_step.action == MISSION_HOLD_TIME)				//If mission_step was a timed poshold then initiate timed poshold
              {
              nav_hold_time = mission_step.parameter1;
              nav_timer_stop = 0;											//This indicates that we are starting a timed poshold
              NAV_state = NAV_STATE_HOLD_TIMED;	
              }
            else 
              {
              NAV_state = NAV_STATE_PROCESS_NEXT;							//Otherwise process next step
              }
            } 
          break;

        case NAV_STATE_DO_JUMP:
          if (jump_times < 0)	//Jump unconditionally (supposed to be -1) -10 should not be here
            { 
            next_step = mission_step.parameter1;
            NAV_state = NAV_STATE_PROCESS_NEXT;
            }

          if (jump_times == 0)
            {
            jump_times = -10;	//reset jump counter
            if (mission_step.flag == MISSION_FLAG_END)						//If this was the last mission step (flag set by the mission planner), then switch to poshold
              {
              NAV_state = NAV_STATE_HOLD_INFINIT;
              NAV_error = NAV_ERROR_FINISH;
              }
            else NAV_state = NAV_STATE_PROCESS_NEXT;
            }

          if (jump_times > 0)		//if zero not reached do a jump
            {
            next_step = mission_step.parameter1;
            NAV_state = NAV_STATE_PROCESS_NEXT;
            jump_times--;
            }
          break;

        case NAV_STATE_PROCESS_NEXT:			//Processing next mission step
          NAV_error = NAV_ERROR_NONE;
          if (!recallWP(next_step)) 
            { 
            abort_mission(NAV_ERROR_WP_CRC);
            }
          else 
            {
            switch(mission_step.action)
              {
              //Waypoiny and hold commands all starts with an enroute status it includes the LAND command too
              case MISSION_WAYPOINT:
              case MISSION_HOLD_TIME:
              case MISSION_HOLD_UNLIM:
              case MISSION_LAND:
                set_new_altitude(mission_step.altitude);
                GPS_set_next_wp(&mission_step.pos[LAT], &mission_step.pos[LON], &GPS_prev[LAT], &GPS_prev[LON]);	
                if ((wp_distance/100) >= GPS_conf.safe_wp_distance)  abort_mission(NAV_ERROR_TOOFAR);
                else NAV_state = NAV_STATE_WP_ENROUTE;	
                GPS_prev[LAT] = mission_step.pos[LAT];								//Save wp coordinates for precise route calc
                GPS_prev[LON] = mission_step.pos[LON];		
                break;
              case MISSION_RTH:
                f.GPS_head_set = 0;
                if (GPS_conf.rth_altitude == 0 && mission_step.altitude == 0) //if config and mission_step alt is zero 
                  set_new_altitude(alt.EstAlt);     // RTH returns at the actual altitude 
                else {
                  uint32_t rth_alt;
                  if (mission_step.altitude == 0) rth_alt = GPS_conf.rth_altitude * 100;   //altitude in mission step has priority
                  else rth_alt = mission_step.altitude;

                  if (alt.EstAlt < rth_alt) set_new_altitude(rth_alt);				     //BUt only if we are below it.
                  else set_new_altitude(alt.EstAlt);
                  }
                NAV_state = NAV_STATE_RTH_START;
                break;
              case MISSION_JUMP:
                if (jump_times == -10) jump_times = mission_step.parameter2;
                if (mission_step.parameter1 > 0 && mission_step.parameter1 < mission_step.number)
                  NAV_state = NAV_STATE_DO_JUMP;
                else //Error situation, invalid jump target
                  abort_mission(NAV_ERROR_INVALID_JUMP);
                break;
              case MISSION_SET_HEADING:
                GPS_poi[LAT] = 0; GPS_poi[LON] = 0;	// zeroing this out clears the possible pervious set_poi
                if (mission_step.parameter1 < 0) f.GPS_head_set = 0;
                else
                  {
                  f.GPS_head_set = 1;
                  GPS_directionToPoi = mission_step.parameter1;
                  } 
                break;
              case MISSION_SET_POI:
                GPS_poi[LAT] = mission_step.pos[LAT];
                GPS_poi[LON] = mission_step.pos[LON];
                f.GPS_head_set = 1;
                break;
              default:														//if we got an unknown action code abort mission and hold position
                abort_mission(NAV_ERROR_INVALID_DATA);
                break;
              }
            next_step++;														//Prepare for the next step

            }
          break;
        }
      } //end of gps calcs ###0002 
    }

  } // End of GPS_compute

  
  
// Abort current mission with the given error code (switch to poshold_infinit)	
void abort_mission(unsigned char error_code)
  {
  GPS_set_next_wp(&GPS_coord[LAT], &GPS_coord[LON],&GPS_coord[LAT], &GPS_coord[LON]);	
  NAV_error = error_code;
  NAV_state = NAV_STATE_HOLD_INFINIT;
  }
//Adjusting heading according to settings - MAG mode must be enabled
void GPS_adjust_heading()
  {
  //TODO: Add slow windup for large heading change
  //This controls the heading
  if (f.GPS_head_set)					// We have seen a SET_POI or a SET_HEADING command
    {
    if (GPS_poi[LAT] == 0)
      magHold = wrap_18000((GPS_directionToPoi*100)-18000)/100;
    else 
      {
      GPS_bearing(&GPS_coord[LAT],&GPS_coord[LON],&GPS_poi[LAT],&GPS_poi[LON],&GPS_directionToPoi);
      GPS_distance_cm(&GPS_coord[LAT],&GPS_coord[LON],&GPS_poi[LAT],&GPS_poi[LON],&wp_distance);
      magHold = GPS_directionToPoi /100;
      }
    }
  else                                // heading controlled by the standard defines
    if (GPS_conf.nav_controls_heading) 
      {
      if (GPS_conf.nav_tail_first) 
        {
        magHold = wrap_18000(target_bearing-18000)/100;
        } 
      else 
        {
          magHold = wrap_18000(target_bearing)/100;
        }
      }
  }

#define LAND_DETECT_THRESHOLD 40      //Counts of land situation
#define BAROPIDMIN           -180     //BaroPID reach this if we landed.....

//Check if we landed or not
void check_land()
  {
  // detect whether we have landed by watching for low climb rate and throttle control
  if ( (abs(alt.vario) < 20) && (BaroPID < BAROPIDMIN)) {
    if (!f.LAND_COMPLETED) {

      if( land_detect < LAND_DETECT_THRESHOLD) {
        land_detect++;
        }else{
          f.LAND_COMPLETED = 1;
          land_detect = 0;
        }
      }
    }else{
      // we've detected movement up or down so reset land_detector
      land_detect = 0;
      if(f.LAND_COMPLETED) {
        f.LAND_COMPLETED = 0;
        }
    }
  }

int32_t get_altitude_error()
  {
  return alt_to_hold - alt.EstAlt;
  }

void clear_new_altitude()
  {
  alt_change_flag = REACHED_ALT;
  }

void force_new_altitude(int32_t _new_alt)
  {
  alt_to_hold 	= _new_alt;
  target_altitude = _new_alt;
  alt_change_flag = REACHED_ALT;
  }

void set_new_altitude(int32_t _new_alt)
  {

  //Limit maximum altitude command
  if(_new_alt > GPS_conf.nav_max_altitude*100) _new_alt = GPS_conf.nav_max_altitude * 100;

  if(_new_alt == alt.EstAlt){
    force_new_altitude(_new_alt);
    return;
    }

  // We start at the current location altitude and gradually change alt
  alt_to_hold = alt.EstAlt;

  // for calculating the delta time
  alt_change_timer = millis();

  // save the target altitude
  target_altitude = _new_alt;

  // reset our altitude integrator
  alt_change = 0;

  // save the original altitude
  original_altitude = alt.EstAlt;

  // to decide if we have reached the target altitude
  if(target_altitude > original_altitude){
    // we are below, going up
    alt_change_flag = ASCENDING;
    }else if(target_altitude < original_altitude){
      // we are above, going down
      alt_change_flag = DESCENDING;
    }else{
      // No Change
      alt_change_flag = REACHED_ALT;
      }
  }

int32_t get_new_altitude()
  {
  // returns a new altitude which feeded into the alt.hold controller

  if(alt_change_flag == ASCENDING)
    {
    // we are below, going up
    if(alt.EstAlt >=  target_altitude) alt_change_flag = REACHED_ALT; 
    // we shouldn't command past our target
    if(alt_to_hold >=  target_altitude) return target_altitude; 
    }
  else if (alt_change_flag == DESCENDING)
    {
    // we are above, going down
    if(alt.EstAlt <=  target_altitude) alt_change_flag = REACHED_ALT;
    // we shouldn't command past our target
    if(alt_to_hold <=  target_altitude) return target_altitude;
    }

  // if we have reached our target altitude, return the target alt
  if(alt_change_flag == REACHED_ALT) return target_altitude;


  int32_t diff 	= abs(alt_to_hold - target_altitude);
  // scale is how we generate a desired rate from the elapsed time
  // a smaller scale means faster rates
  int8_t			_scale 	= 4;

  if (alt_to_hold < target_altitude)
    {
    // we are below the target alt
    if(diff < 200)	_scale = 4;
    else _scale = 3;
    }
  else
    {
    // we are above the target, going down
    if(diff < 400) _scale = 5;		//Slow down if only 4meters above
    if(diff < 100) _scale = 6;		//Slow down further if within 1meter
    }

  // we use the elapsed time as our altitude offset
  // 1000 = 1 sec
  // 1000 >> 4 = 64cm/s descent by default
  int32_t change = (millis() - alt_change_timer) >> _scale;

  if(alt_change_flag == ASCENDING){
    alt_change += change;
    }else{
      alt_change -= change;
    }
  // for generating delta time
  alt_change_timer = millis();

  return original_altitude + alt_change;
  }

////////////////////////////////////////////////////////////////////////////////////
//PID based GPS navigation functions
//Author : EOSBandi
//Based on code and ideas from the Arducopter team: Jason Short,Randy Mackay, Pat Hickey, Jose Julio, Jani Hirvinen
//Andrew Tridgell, Justin Beech, Adam Rivera, Jean-Louis Naudin, Roberto Navoni

//original constraint does not work with variables
int16_t constrain_int16(int16_t amt, int16_t low, int16_t high) {
  return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
  }
////////////////////////////////////////////////////////////////////////////////////
// this is used to offset the shrinking longitude as we go towards the poles
// It's ok to calculate this once per waypoint setting, since it changes a little within the reach of a multicopter
//
void GPS_calc_longitude_scaling(int32_t lat) {

  GPS_scaleLonDown = cos(lat * 1.0e-7f * 0.01745329251f);
  }

////////////////////////////////////////////////////////////////////////////////////
// Sets the waypoint to navigate, reset neccessary variables and calculate initial values
//
void GPS_set_next_wp(int32_t* lat_to, int32_t* lon_to, int32_t* lat_from, int32_t* lon_from) {

  GPS_WP[LAT] = *lat_to;
  GPS_WP[LON] = *lon_to;

  GPS_FROM[LAT] = *lat_from;
  GPS_FROM[LON] = *lon_from;

  GPS_calc_longitude_scaling(*lat_to);

  GPS_bearing(&GPS_FROM[LAT],&GPS_FROM[LON],&GPS_WP[LAT],&GPS_WP[LON],&target_bearing);
  GPS_distance_cm(&GPS_FROM[LAT],&GPS_FROM[LON],&GPS_WP[LAT],&GPS_WP[LON],&wp_distance);
  GPS_calc_location_error(&GPS_WP[LAT],&GPS_WP[LON],&GPS_FROM[LAT],&GPS_FROM[LON]);
  waypoint_speed_gov = GPS_conf.nav_speed_min;
  original_target_bearing = target_bearing;

  }

////////////////////////////////////////////////////////////////////////////////////
// Check if we missed the destination somehow
//
static bool check_missed_wp(void) {
  int32_t temp;
  temp = target_bearing - original_target_bearing;
  temp = wrap_18000(temp);
  return (abs(temp) > 10000);   // we passed the waypoint by 100 degrees
}

////////////////////////////////////////////////////////////////////////////////////
// Get distance between two points in cm
// Get bearing from pos1 to pos2, returns an 1deg = 100 precision

void GPS_bearing(int32_t* lat1, int32_t* lon1, int32_t* lat2, int32_t* lon2, int32_t* bearing) {

  int32_t off_x = *lon2 - *lon1;
  int32_t off_y = (*lat2 - *lat1) / GPS_scaleLonDown;

  *bearing = 9000 + atan2(-off_y, off_x) * 5729.57795f;      //Convert the output redians to 100xdeg
  if (*bearing < 0) *bearing += 36000;
}

void GPS_distance_cm(int32_t* lat1, int32_t* lon1, int32_t* lat2, int32_t* lon2,uint32_t* dist) {

  float dLat = (float)(*lat2 - *lat1);                                    // difference of latitude in 1/10 000 000 degrees
  float dLon = (float)(*lon2 - *lon1) * GPS_scaleLonDown; //x
  *dist = sqrt(sq(dLat) + sq(dLon)) * 1.11318845f;
  }

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
static void GPS_calc_velocity(void){
  static int16_t speed_old[2] = {0,0};
  static int32_t last[2] = {0,0};
  static uint8_t init = 0;

  if (init) {
    float tmp = 1.0/dTnav;
    actual_speed[_X] = (float)(GPS_coord[LON] - last[LON]) *  GPS_scaleLonDown * tmp;
    actual_speed[_Y] = (float)(GPS_coord[LAT]  - last[LAT])  * tmp;

    //TODO: Check unrealistic speed changes and signal navigation about posibble gps signal degradation

    if (!GPS_conf.lead_filter)
      {
      actual_speed[_X] = (actual_speed[_X] + speed_old[_X]) / 2;
      actual_speed[_Y] = (actual_speed[_Y] + speed_old[_Y]) / 2;

      speed_old[_X] = actual_speed[_X];
      speed_old[_Y] = actual_speed[_Y];
      }
    }
  init=1;

  last[LON] = GPS_coord[LON];
  last[LAT] = GPS_coord[LAT];

  if (GPS_conf.lead_filter)
    {
    GPS_coord_lead[LON] = xLeadFilter.get_position(GPS_coord[LON], actual_speed[_X], GPS_LAG);
    GPS_coord_lead[LAT] = yLeadFilter.get_position(GPS_coord[LAT], actual_speed[_Y], GPS_LAG);
    }

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
// TODO: check that the poshold target speed constraint can be increased for snappier poshold lock
static void GPS_calc_poshold(void) {
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
    //	nav[axis]  = constrain(nav[axis], -NAV_BANK_MAX, NAV_BANK_MAX);
    nav[axis]  = constrain_int16(nav[axis], -GPS_conf.nav_bank_max, GPS_conf.nav_bank_max);
    navPID[axis].integrator = poshold_ratePID[axis].integrator;
    }
  }

////////////////////////////////////////////////////////////////////////////////////
// Calculate the desired nav_lat and nav_lon for distance flying such as RTH and WP
//
static void GPS_calc_nav_rate( uint16_t max_speed)
  {
  float trig[2];
  int32_t target_speed[2];
  int32_t tilt;
  uint8_t axis;

  GPS_update_crosstrack();
  int16_t cross_speed = crosstrack_error * (GPS_conf.crosstrack_gain / 100.0);  //check is it ok ?
  cross_speed = constrain(cross_speed,-200,200);
  cross_speed = -cross_speed;

  float temp = (9000l - target_bearing) * RADX100;
  trig[_X] = cos(temp);
  trig[_Y] = sin(temp);

  target_speed[_X] = max_speed * trig[_X] - cross_speed * trig[_Y];
  target_speed[_Y] = cross_speed * trig[_X] + max_speed * trig[_Y];

  for (axis=0;axis<2;axis++)
    {
    rate_error[axis] = target_speed[axis] - actual_speed[axis];
    rate_error[axis] = constrain(rate_error[axis],-1000,1000);
    nav[axis]      =
        get_P(rate_error[axis],                        &navPID_PARAM)
       +get_I(rate_error[axis], &dTnav, &navPID[axis], &navPID_PARAM)
       +get_D(rate_error[axis], &dTnav, &navPID[axis], &navPID_PARAM);

    //			nav[axis] = constrain(nav[axis],-NAV_BANK_MAX,NAV_BANK_MAX);
    nav[axis]  = constrain_int16(nav[axis], -GPS_conf.nav_bank_max, GPS_conf.nav_bank_max);
    poshold_ratePID[axis].integrator = navPID[axis].integrator;
  }
}

static void GPS_update_crosstrack(void)
  {
  // Crosstrack Error
  // ----------------
  // If we are too far off or too close we don't do track following
  float temp = (target_bearing - original_target_bearing) * RADX100;
  crosstrack_error = sin(temp) * wp_distance;	 // Meters we are off track line
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
//                 |< we should slow to 1 m/s as we hit the target
//
static uint16_t GPS_calc_desired_speed(uint16_t max_speed, bool _slow) {
  if(_slow){
    max_speed = min(max_speed, wp_distance / 2);
    }else{
      max_speed = min(max_speed, wp_distance);
      max_speed = max(max_speed, GPS_conf.nav_speed_min);  // go at least nav_speed_min
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

/*
 * EOS increased the precision here, even if we think that the gps is not precise enough, with 10e5 precision it has 76cm resolution
 * with 10e7 it's around 1 cm now. Increasing it further is irrelevant, since even 1cm resolution is unrealistic, however increased 
 * resolution also increased precision of nav calculations
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
        //GPS_time       = _buffer.posllh.time;
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
      //GPS_time                    = _buffer.msg.utc_time;
      //GPS_hdop                  = _buffer.msg.hdop;
      parsed = true;
      GPS_Present = 1;
    }
  return parsed;
  }
#endif //MTK


//************************************************************************
// Common GPS functions 
//
void init_RTH()
  {
  f.GPS_mode = GPS_MODE_RTH;									// Set GPS_mode to RTH
  f.GPS_BARO_MODE = true;
  GPS_hold[LAT] = GPS_coord[LAT];							//All RTH starts with a poshold 
  GPS_hold[LON] = GPS_coord[LON];							//This allows to raise to rth altitude
  GPS_set_next_wp(&GPS_hold[LAT],&GPS_hold[LON], &GPS_hold[LAT], &GPS_hold[LON]);
  NAV_paused_at = 0;
  if (GPS_conf.rth_altitude == 0) set_new_altitude(alt.EstAlt);	    //Return at actual altitude
  else {													// RTH altitude is defined, but we use it only if we are below it
    if (alt.EstAlt < GPS_conf.rth_altitude * 100) 
      set_new_altitude(GPS_conf.rth_altitude * 100);
    else set_new_altitude(alt.EstAlt);					   
    }
  f.GPS_head_set = 0;										//Allow the RTH ti handle heading
  NAV_state = NAV_STATE_RTH_START;							//NAV engine status is Starting RTH.
  }

void GPS_reset_home_position(void) {
  if (f.GPS_FIX && GPS_numSat >= 5) {
    GPS_home[LAT] = GPS_coord[LAT];
    GPS_home[LON] = GPS_coord[LON];
    GPS_calc_longitude_scaling(GPS_coord[LAT]);    //need an initial value for distance and bearing calc
    nav_takeoff_bearing = att.heading;             //save takeoff heading
    //TODO: Set ground altitude
    f.GPS_FIX_HOME = 1;
    }
  }

//reset navigation (stop the navigation processor, and clear nav)
void GPS_reset_nav(void) {
  uint8_t i;

  for(i=0;i<2;i++) {
    nav[i] = 0;
    reset_PID(&posholdPID[i]);
    reset_PID(&poshold_ratePID[i]);
    reset_PID(&navPID[i]);
    NAV_state = NAV_STATE_NONE;
    //invalidate JUMP counter
    jump_times = -10;
    //reset next step counter
    next_step = 1;
    //Clear poi
    GPS_poi[LAT] = 0; GPS_poi[LON] = 0;
    f.GPS_head_set = 0;
    }
  }

//Get the relevant P I D values and set the PID controllers 
void GPS_set_pids(void) {
  posholdPID_PARAM.kP   = (float)conf.pid[PIDPOS].P8/100.0;
  posholdPID_PARAM.kI   = (float)conf.pid[PIDPOS].I8/100.0;
  posholdPID_PARAM.Imax = POSHOLD_RATE_IMAX * 100;

  poshold_ratePID_PARAM.kP   = (float)conf.pid[PIDPOSR].P8/10.0;
  poshold_ratePID_PARAM.kI   = (float)conf.pid[PIDPOSR].I8/100.0;
  poshold_ratePID_PARAM.kD   = (float)conf.pid[PIDPOSR].D8/1000.0;
  poshold_ratePID_PARAM.Imax = POSHOLD_RATE_IMAX * 100;

  navPID_PARAM.kP   = (float)conf.pid[PIDNAVR].P8/10.0;
  navPID_PARAM.kI   = (float)conf.pid[PIDNAVR].I8/100.0;
  navPID_PARAM.kD   = (float)conf.pid[PIDNAVR].D8/1000.0;
  navPID_PARAM.Imax = POSHOLD_RATE_IMAX * 100;
  }
//It was moved here since even i2cgps code needs it
int32_t wrap_18000(int32_t ang) {
  if (ang > 18000)  ang -= 36000;
  if (ang < -18000) ang += 36000;
  return ang;
  }

#endif // GPS Defined
