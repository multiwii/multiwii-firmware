#ifndef TYPES_H_
#define TYPES_H_

enum rc {
  ROLL,
  PITCH,
  YAW,
  THROTTLE,
  AUX1,
  AUX2,
  AUX3,
  AUX4,
  AUX5,
  AUX6,
  AUX7,
  AUX8
};

enum pid {
  PIDROLL,
  PIDPITCH,
  PIDYAW,
  PIDALT,
  PIDPOS,
  PIDPOSR,
  PIDNAVR,
  PIDLEVEL,
  PIDMAG,
  PIDVEL,     // not used currently
  PIDITEMS
};

enum box {
  BOXARM,
  #if ACC
    BOXANGLE,
    BOXHORIZON,
  #endif
  #if BARO && (!defined(SUPPRESS_BARO_ALTHOLD))
    BOXBARO,
  #endif
  #ifdef VARIOMETER
    BOXVARIO,
  #endif
  BOXMAG,
  #if defined(HEADFREE)
    BOXHEADFREE,
    BOXHEADADJ, // acquire heading for HEADFREE mode
  #endif
  #if defined(SERVO_TILT) || defined(GIMBAL)  || defined(SERVO_MIX_TILT)
    BOXCAMSTAB,
  #endif
  #if defined(CAMTRIG)
    BOXCAMTRIG,
  #endif
  #if GPS
    BOXGPSHOME,
    BOXGPSHOLD,
  #endif
  #if defined(FIXEDWING) || defined(HELICOPTER)
    BOXPASSTHRU,
  #endif
  #if defined(BUZZER)
    BOXBEEPERON,
  #endif
  #if defined(LED_FLASHER)
    BOXLEDMAX, // we want maximum illumination
    BOXLEDLOW, // low/no lights
  #endif
  #if defined(LANDING_LIGHTS_DDR)
    BOXLLIGHTS, // enable landing lights at any altitude
  #endif
  #ifdef INFLIGHT_ACC_CALIBRATION
    BOXCALIB,
  #endif
  #ifdef GOVERNOR_P
    BOXGOV,
  #endif
  #ifdef OSD_SWITCH
    BOXOSD,
  #endif
  #if GPS
    BOXGPSNAV,
    BOXLAND,
  #endif
  CHECKBOXITEMS
};

typedef struct {
  int16_t  accSmooth[3];
  int16_t  gyroData[3];
  int16_t  magADC[3];
  int16_t  gyroADC[3];
  int16_t  accADC[3];
} imu_t;

typedef struct {
  uint8_t  vbat;               // battery voltage in 0.1V steps
  uint16_t intPowerMeterSum;
  uint16_t rssi;              // range: [0;1023]
  uint16_t amperage;          // 1unit == 100mA
  uint16_t watts;             // 1unit == 1W
  uint16_t vbatcells[VBAT_CELLS_NUM];
} analog_t;

typedef struct {
  int32_t  EstAlt;             // in cm
  int16_t  vario;              // variometer in cm/s
} alt_t;

typedef struct {
  int16_t angle[2];            // absolute angle inclination in multiple of 0.1 degree    180 deg = 1800
  int16_t heading;             // variometer in cm/s
} att_t;

typedef struct {
  uint8_t OK_TO_ARM :1 ;
  uint8_t ARMED :1 ;
  uint8_t ACC_CALIBRATED :1 ;
  uint8_t ANGLE_MODE :1 ;
  uint8_t HORIZON_MODE :1 ;
  uint8_t MAG_MODE :1 ;
  uint8_t BARO_MODE :1 ;
#ifdef HEADFREE
  uint8_t HEADFREE_MODE :1 ;
#endif
#if defined(FIXEDWING) || defined(HELICOPTER)
  uint8_t PASSTHRU_MODE :1 ;
#endif
  uint8_t SMALL_ANGLES_25 :1 ;
#if MAG
  uint8_t CALIBRATE_MAG :1 ;
#endif
#ifdef VARIOMETER
  uint8_t VARIO_MODE :1;
#endif
  uint8_t GPS_mode: 2;               // 0-3 NONE,HOLD, HOME, NAV (see GPS_MODE_* defines
#if BARO
  uint8_t THROTTLE_IGNORED : 1;      // If it is 1 then ignore throttle stick movements in baro mode;
#endif
#if GPS
  uint8_t GPS_FIX :1 ;
  uint8_t GPS_FIX_HOME :1 ;
  uint8_t GPS_BARO_MODE : 1;         // This flag is used when GPS controls baro mode instead of user (it will replace rcOptions[BARO]
  uint8_t GPS_head_set: 1;           // it is 1 if the navigation engine got commands to control heading (SET_POI or SET_HEAD) CLEAR_HEAD will zero it
  uint8_t LAND_COMPLETED: 1;
  uint8_t LAND_IN_PROGRESS: 1;
#endif
} flags_struct_t;

typedef struct {
  uint8_t currentSet;
  int16_t accZero[3];
  int16_t magZero[3];
  uint16_t flashsum;
  uint8_t checksum;      // MUST BE ON LAST POSITION OF STRUCTURE !
} global_conf_t;

struct pid_ {
  uint8_t P8;
  uint8_t I8;
  uint8_t D8;
};

struct servo_conf_ {  // this is a generic way to configure a servo, every multi type with a servo should use it
  int16_t min;        // minimum value, must be more than 1020 with the current implementation
  int16_t max;        // maximum value, must be less than 2000 with the current implementation
  int16_t middle;     // default should be 1500
  int8_t  rate;       // range [-100;+100] ; can be used to ajust a rate 0-100% and a direction
};

typedef struct {
  pid_    pid[PIDITEMS];
  uint8_t rcRate8;
  uint8_t rcExpo8;
  uint8_t rollPitchRate;
  uint8_t yawRate;
  uint8_t dynThrPID;
  uint8_t thrMid8;
  uint8_t thrExpo8;
  int16_t angleTrim[2]; 
  #if defined(EXTENDED_AUX_STATES)
   uint32_t activate[CHECKBOXITEMS];  //Extended aux states define six different aux state for each aux channel
  #else
   uint16_t activate[CHECKBOXITEMS];
  #endif 
  uint8_t powerTrigger1;
  #if MAG
    int16_t mag_declination;
  #endif
  servo_conf_ servoConf[8];
  #if defined(GYRO_SMOOTHING)
    uint8_t Smoothing[3];
  #endif
  #if defined (FAILSAFE)
    int16_t failsafe_throttle;
  #endif
  #ifdef VBAT
    uint8_t vbatscale;
    uint8_t vbatlevel_warn1;
    uint8_t vbatlevel_warn2;
    uint8_t vbatlevel_crit;
  #endif
  #ifdef POWERMETER
    uint8_t pint2ma;
  #endif
  #ifdef POWERMETER_HARD
    uint16_t psensornull;
  #endif
  #ifdef MMGYRO
    uint8_t mmgyro;
  #endif
  #ifdef ARMEDTIMEWARNING
    uint16_t armedtimewarning;
  #endif
  int16_t minthrottle;
  #ifdef GOVERNOR_P
   int16_t governorP;
   int16_t governorD;
  #endif
  #ifdef YAW_COLL_PRECOMP
   uint8_t yawCollPrecomp;
   uint16_t yawCollPrecompDeadband;
  #endif
  uint8_t  checksum;      // MUST BE ON LAST POSITION OF CONF STRUCTURE !
} conf_t;

#ifdef LOG_PERMANENT
typedef struct {
  uint16_t arm;           // #arm events
  uint16_t disarm;        // #disarm events
  uint16_t start;         // #powercycle/reset/initialize events
  uint32_t armed_time ;   // copy of armedTime @ disarm
  uint32_t lifetime;      // sum (armed) lifetime in seconds
  uint16_t failsafe;      // #failsafe state @ disarm
  uint16_t i2c;           // #i2c errs state @ disarm
  uint8_t  running;       // toggle on arm & disarm to monitor for clean shutdown vs. powercut
  uint8_t  checksum;      // MUST BE ON LAST POSITION OF CONF STRUCTURE !
} plog_t;
#endif

#if GPS

// TODO: cross check with I2C gps and add relevant defines

//This is the mode what is selected via the remote (NONE, HOLD, RTH and NAV (NAV-> exectute mission)
enum gpsmode {
  GPS_MODE_NONE = 0, 
  GPS_MODE_HOLD, 
  GPS_MODE_RTH, 
  GPS_MODE_NAV 
};

enum navstate {
  NAV_STATE_NONE = 0,
  NAV_STATE_RTH_START,
  NAV_STATE_RTH_ENROUTE,
  NAV_STATE_HOLD_INFINIT,
  NAV_STATE_HOLD_TIMED,
  NAV_STATE_WP_ENROUTE,
  NAV_STATE_PROCESS_NEXT,
  NAV_STATE_DO_JUMP,
  NAV_STATE_LAND_START,
  NAV_STATE_LAND_IN_PROGRESS,
  NAV_STATE_LANDED,
  NAV_STATE_LAND_SETTLE,
  NAV_STATE_LAND_START_DESCENT
};

enum naverror {
  NAV_ERROR_NONE = 0,            //All systems clear
  NAV_ERROR_TOOFAR,              //Next waypoint distance is more than safety distance
  NAV_ERROR_SPOILED_GPS,         //GPS reception is compromised - Nav paused - copter is adrift !
  NAV_ERROR_WP_CRC,              //CRC error reading WP data from EEPROM - Nav stopped
  NAV_ERROR_FINISH,              //End flag detected, navigation finished
  NAV_ERROR_TIMEWAIT,            //Waiting for poshold timer
  NAV_ERROR_INVALID_JUMP,        //Invalid jump target detected, aborting
  NAV_ERROR_INVALID_DATA,        //Invalid mission step action code, aborting, copter is adrift
  NAV_ERROR_WAIT_FOR_RTH_ALT,    //Waiting to reach RTH Altitude
  NAV_ERROR_GPS_FIX_LOST,        //Gps fix lost, aborting mission
  NAV_ERROR_DISARMED,            //NAV engine disabled due disarm
  NAV_ERROR_LANDING              //Landing
};

typedef struct {
  uint8_t  number;     //Waypoint number
  int32_t  pos[2];     //GPS position 
  uint8_t  action;     //Action to follow
  int16_t  parameter1; //Parameter for the action
  int16_t  parameter2; //Parameter for the action
  int16_t  parameter3; //Parameter for the action
  uint32_t altitude;   //Altitude in cm (AGL)
  uint8_t  flag;       //flags the last wp and other fancy things that are not yet defined
  uint8_t  checksum;   //this must be at the last position
} mission_step_struct;


typedef struct {
  //Don't forget to change the reply size in GUI when change this struct;

  // on/off flags
  // First byte 
  uint8_t filtering : 1;
  uint8_t lead_filter : 1;
  uint8_t dont_reset_home_at_arm : 1;
  uint8_t nav_controls_heading : 1;
  uint8_t nav_tail_first : 1;
  uint8_t nav_rth_takeoff_heading : 1;
  uint8_t slow_nav : 1;
  uint8_t wait_for_rth_alt : 1;
  // Second byte
  uint8_t ignore_throttle: 1; // Disable stick controls during mission and RTH
  uint8_t takeover_baro: 1;




  uint16_t wp_radius;           // in cm
  uint16_t safe_wp_distance;    // in meter
  uint16_t nav_max_altitude;    // in meter
  uint16_t nav_speed_max;       // in cm/s
  uint16_t nav_speed_min;       // in cm/s
  uint8_t  crosstrack_gain;     // * 100 (0-2.56)
  uint16_t nav_bank_max;        // degree * 100; (3000 default)
  uint16_t rth_altitude;        // in meter
  uint8_t  land_speed;          // between 50 and 255 (100 approx = 50cm/sec)
  uint16_t fence;               // fence control in meters

  uint8_t  max_wp_number;

  uint8_t  checksum;
} gps_conf_struct;

#endif

#endif /* TYPES_H_ */
