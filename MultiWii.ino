/*
MultiWiiCopter by Alexandre Dubus
www.multiwii.com
March  2013     V2.2
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 any later version. see <http://www.gnu.org/licenses/>
*/

#include <avr/io.h>

#include "config.h"
#include "def.h"


#include <avr/pgmspace.h>
#define  VERSION  221

/*********** RC alias *****************/
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

const char pidnames[] PROGMEM =
  "ROLL;"
  "PITCH;"
  "YAW;"
  "ALT;"
  "Pos;"
  "PosR;"
  "NavR;"
  "LEVEL;"
  "MAG;"
  "VEL;"
;

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
  #if MAG
    BOXMAG,
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
  CHECKBOXITEMS
};

const char boxnames[] PROGMEM = // names for dynamic generation of config GUI
  "ARM;"
  #if ACC
    "ANGLE;"
    "HORIZON;"
  #endif
  #if BARO && (!defined(SUPPRESS_BARO_ALTHOLD))
    "BARO;"
  #endif
  #ifdef VARIOMETER
    "VARIO;"
  #endif
  #if MAG
    "MAG;"
    "HEADFREE;"
    "HEADADJ;"  
  #endif
  #if defined(SERVO_TILT) || defined(GIMBAL)|| defined(SERVO_MIX_TILT)
    "CAMSTAB;"
  #endif
  #if defined(CAMTRIG)
    "CAMTRIG;"
  #endif
  #if GPS
    "GPS HOME;"
    "GPS HOLD;"
  #endif
  #if defined(FIXEDWING) || defined(HELICOPTER)
    "PASSTHRU;"
  #endif
  #if defined(BUZZER)
    "BEEPER;"
  #endif
  #if defined(LED_FLASHER)
    "LEDMAX;"
    "LEDLOW;"
  #endif
  #if defined(LANDING_LIGHTS_DDR)
    "LLIGHTS;"
  #endif
  #ifdef INFLIGHT_ACC_CALIBRATION
    "CALIB;"
  #endif
  #ifdef GOVERNOR_P
    "GOVERNOR;"
  #endif
  #ifdef OSD_SWITCH
    "OSD SW;"
  #endif
;

const uint8_t boxids[] PROGMEM = {// permanent IDs associated to boxes. This way, you can rely on an ID number to identify a BOX function.
  0, //"ARM;"
  #if ACC
    1, //"ANGLE;"
    2, //"HORIZON;"
  #endif
  #if BARO && (!defined(SUPPRESS_BARO_ALTHOLD))
    3, //"BARO;"
  #endif
  #ifdef VARIOMETER
    4, //"VARIO;"
  #endif
  #if MAG
    5, //"MAG;"
    6, //"HEADFREE;"
    7, //"HEADADJ;"  
  #endif
  #if defined(SERVO_TILT) || defined(GIMBAL)|| defined(SERVO_MIX_TILT)
    8, //"CAMSTAB;"
  #endif
  #if defined(CAMTRIG)
    9, //"CAMTRIG;"
  #endif
  #if GPS
    10, //"GPS HOME;"
    11, //"GPS HOLD;"
  #endif
  #if defined(FIXEDWING) || defined(HELICOPTER)
    12, //"PASSTHRU;"
  #endif
  #if defined(BUZZER)
    13, //"BEEPER;"
  #endif
  #if defined(LED_FLASHER)
    14, //"LEDMAX;"
    15, //"LEDLOW;"
  #endif
  #if defined(LANDING_LIGHTS_DDR)
    16, //"LLIGHTS;"
  #endif
  #ifdef INFLIGHT_ACC_CALIBRATION
    17, //"CALIB;"
  #endif
  #ifdef GOVERNOR_P
    18, //"GOVERNOR;"
  #endif
  #ifdef OSD_SWITCH
    19, //"OSD_SWITCH;"
  #endif
};


static uint32_t currentTime = 0;
static uint16_t previousTime = 0;
static uint16_t cycleTime = 0;     // this is the number in micro second to achieve a full loop, it can differ a little and is taken into account in the PID loop
static uint16_t calibratingA = 0;  // the calibration is done in the main loop. Calibrating decreases at each cycle down to 0, then we enter in a normal mode.
static uint16_t calibratingB = 0;  // baro calibration = get new ground pressure value
static uint16_t calibratingG;
static int16_t  magHold,headFreeModeHold; // [-180;+180]
static uint8_t  vbatMin = VBATNOMINAL;  // lowest battery voltage in 0.1V steps
static uint8_t  rcOptions[CHECKBOXITEMS];
static int32_t  BaroAlt,AltHold; // in cm
static int16_t  BaroPID = 0;
static int16_t  errorAltitudeI = 0;

// **************
// gyro+acc IMU
// **************
static int16_t gyroZero[3] = {0,0,0};
static int16_t angle[2]    = {0,0};  // absolute angle inclination in multiple of 0.1 degree    180 deg = 1800

static struct {
  int16_t  accSmooth[3];
  int16_t  gyroData[3];
  int16_t  magADC[3];
  int16_t  gyroADC[3];
  int16_t  accADC[3];
} imu;

static struct {
  uint8_t  vbat;               // battery voltage in 0.1V steps
  uint16_t intPowerMeterSum;
  uint16_t rssi;               // range: [0;1023]
} analog;

static struct {
  int32_t  EstAlt;             // in cm
  int16_t  vario;              // variometer in cm/s
} alt;

static struct {
  int16_t angle[2];
  int16_t heading;              // variometer in cm/s
} att;

#if defined(ARMEDTIMEWARNING)
  static uint32_t  ArmedTimeWarningMicroSeconds = 0;
#endif

static int16_t  debug[4];
static int16_t  sonarAlt; //to think about the unit

struct flags_struct {
  uint8_t OK_TO_ARM :1 ;
  uint8_t ARMED :1 ;
  uint8_t I2C_INIT_DONE :1 ; // For i2c gps we have to now when i2c init is done, so we can update parameters to the i2cgps from eeprom (at startup it is done in setup())
  uint8_t ACC_CALIBRATED :1 ;
  uint8_t NUNCHUKDATA :1 ;
  uint8_t ANGLE_MODE :1 ;
  uint8_t HORIZON_MODE :1 ;
  uint8_t MAG_MODE :1 ;
  uint8_t BARO_MODE :1 ;
  uint8_t GPS_HOME_MODE :1 ;
  uint8_t GPS_HOLD_MODE :1 ;
  uint8_t HEADFREE_MODE :1 ;
  uint8_t PASSTHRU_MODE :1 ;
  uint8_t GPS_FIX :1 ;
  uint8_t GPS_FIX_HOME :1 ;
  uint8_t SMALL_ANGLES_25 :1 ;
  uint8_t CALIBRATE_MAG :1 ;
  uint8_t VARIO_MODE :1;
} f;

//for log
#if defined(LOG_VALUES) || defined(LCD_TELEMETRY)
  static uint16_t cycleTimeMax = 0;       // highest ever cycle timen
  static uint16_t cycleTimeMin = 65535;   // lowest ever cycle timen
  static int32_t  BAROaltMax;             // maximum value
  static uint8_t  GPS_speedMaxKmh = 0;    // maximum speed from gps in km/h
  static uint16_t powerValueMaxMAH = 0;
#endif
#if defined(LOG_VALUES) || defined(LCD_TELEMETRY) || defined(ARMEDTIMEWARNING) || defined(LOG_PERMANENT)
  static uint32_t armedTime = 0;
#endif

static int16_t  i2c_errors_count = 0;
static int16_t  annex650_overrun_count = 0;



#if defined(THROTTLE_ANGLE_CORRECTION)
  static int16_t throttleAngleCorrection = 0;	// correction of throttle in lateral wind,
  static int8_t  cosZ = 100;					// cos(angleZ)*100
#endif



// **********************
//Automatic ACC Offset Calibration
// **********************
#if defined(INFLIGHT_ACC_CALIBRATION)
  static uint16_t InflightcalibratingA = 0;
  static int16_t AccInflightCalibrationArmed;
  static uint16_t AccInflightCalibrationMeasurementDone = 0;
  static uint16_t AccInflightCalibrationSavetoEEProm = 0;
  static uint16_t AccInflightCalibrationActive = 0;
#endif

// **********************
// power meter
// **********************
#if defined(POWERMETER) || ( defined(LOG_VALUES) && (LOG_VALUES >= 3) )
  #define PMOTOR_SUM 8                     // index into pMeter[] for sum
  static uint32_t pMeter[PMOTOR_SUM + 1];  // we use [0:7] for eight motors,one extra for sum
  static uint8_t pMeterV;                  // dummy to satisfy the paramStruct logic in ConfigurationLoop()
  static uint32_t pAlarm;                  // we scale the eeprom value from [0:255] to this value we can directly compare to the sum in pMeter[6]
  static uint16_t powerValue = 0;          // last known current
#endif
static uint16_t intPowerTrigger1;

// **********************
// telemetry
// **********************
#if defined(LCD_TELEMETRY)
  static uint8_t telemetry = 0;
  static uint8_t telemetry_auto = 0;
#endif
#ifdef LCD_TELEMETRY_STEP
  static char telemetryStepSequence []  = LCD_TELEMETRY_STEP;
  static uint8_t telemetryStepIndex = 0;
#endif

// ******************
// rc functions
// ******************
#define MINCHECK 1100
#define MAXCHECK 1900

#define ROL_LO  (1<<(2*ROLL))
#define ROL_CE  (3<<(2*ROLL))
#define ROL_HI  (2<<(2*ROLL))
#define PIT_LO  (1<<(2*PITCH))
#define PIT_CE  (3<<(2*PITCH))
#define PIT_HI  (2<<(2*PITCH))
#define YAW_LO  (1<<(2*YAW))
#define YAW_CE  (3<<(2*YAW))
#define YAW_HI  (2<<(2*YAW))
#define THR_LO  (1<<(2*THROTTLE))
#define THR_CE  (3<<(2*THROTTLE))
#define THR_HI  (2<<(2*THROTTLE))

static int16_t failsafeEvents = 0;
volatile int16_t failsafeCnt = 0;

static int16_t rcData[RC_CHANS];    // interval [1000;2000]
static int16_t rcSerial[8];         // interval [1000;2000] - is rcData coming from MSP 
static int16_t rcCommand[4];        // interval [1000;2000] for THROTTLE and [-500;+500] for ROLL/PITCH/YAW 
static uint8_t rcSerialCount = 0;   // a counter to select legacy RX when there is no more MSP rc serial data 
static int16_t lookupPitchRollRC[5];// lookup table for expo & RC rate PITCH+ROLL
static int16_t lookupThrottleRC[11];// lookup table for expo & mid THROTTLE

#if defined(SPEKTRUM)
  volatile uint8_t  spekFrameFlags;
  volatile uint32_t spekTimeLast;
#endif

#if defined(OPENLRSv2MULTI)
  static uint8_t pot_P,pot_I; // OpenLRS onboard potentiometers for P and I trim or other usages
#endif


// *************************
// motor and servo functions
// *************************
static int16_t axisPID[3];
static int16_t motor[8];
static int16_t servo[8] = {1500,1500,1500,1500,1500,1500,1500,1000};
static uint16_t motorTogglesByte=0; // Is it Possible to change to uint8_t ?... 

// ************************
// EEPROM Layout definition
// ************************
static uint8_t dynP8[3], dynD8[3];

static struct {
  uint8_t currentSet;
  int16_t accZero[3];
  int16_t magZero[3];
  uint16_t flashsum; 
  uint8_t checksum;      // MUST BE ON LAST POSITION OF STRUCTURE ! 
} global_conf;

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

static struct {
  pid_    pid[PIDITEMS];
  uint8_t rcRate8;
  uint8_t rcExpo8;
  uint8_t rollPitchRate;
  uint8_t yawRate;
  uint8_t dynThrPID;
  uint8_t thrMid8;
  uint8_t thrExpo8;
  int16_t angleTrim[2];
  uint16_t activate[CHECKBOXITEMS];
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
   int8_t  governorR;
  #endif
  uint8_t  checksum;      // MUST BE ON LAST POSITION OF CONF STRUCTURE ! 
} conf;

#ifdef LOG_PERMANENT
static struct {
  uint16_t arm;           // #arm events
  uint16_t disarm;        // #disarm events
  uint16_t start;         // #powercycle/reset/initialize events
  uint32_t armed_time ;   // copy of armedTime @ disarm
  uint32_t lifetime;      // sum (armed) lifetime in seconds
  uint16_t failsafe;      // #failsafe state @ disarm
  uint16_t i2c;           // #i2c errs state @ disarm
  uint8_t  running;       // toggle on arm & disarm to monitor for clean shutdown vs. powercut
  uint8_t  checksum;      // MUST BE ON LAST POSITION OF CONF STRUCTURE !
} plog;
#endif

// **********************
// GPS common variables
// **********************
  static int16_t  GPS_angle[2] = { 0, 0};                      // the angles that must be applied for GPS correction
  static int32_t  GPS_coord[2];
  static int32_t  GPS_home[2];
  static int32_t  GPS_hold[2];
  static uint8_t  GPS_numSat;
  static uint16_t GPS_distanceToHome;                          // distance to home  - unit: meter
  static int16_t  GPS_directionToHome;                         // direction to home - unit: degree
  static uint16_t GPS_altitude;                                // GPS altitude      - unit: meter
  static uint16_t GPS_speed;                                   // GPS speed         - unit: cm/s
  static uint8_t  GPS_update = 0;                              // a binary toogle to distinct a GPS position update
  static uint16_t GPS_ground_course = 0;                       //                   - unit: degree*10
  static uint8_t  GPS_Present = 0;                             // Checksum from Gps serial
  static uint8_t  GPS_Enable  = 0;

  #define LAT  0
  #define LON  1
  // The desired bank towards North (Positive) or South (Negative) : latitude
  // The desired bank towards East (Positive) or West (Negative)   : longitude
  static int16_t  nav[2];
  static int16_t  nav_rated[2];    //Adding a rate controller to the navigation to make it smoother

  // default POSHOLD control gains
  #define POSHOLD_P              .11
  #define POSHOLD_I              0.0
  #define POSHOLD_IMAX           20        // degrees

  #define POSHOLD_RATE_P         2.0
  #define POSHOLD_RATE_I         0.08      // Wind control
  #define POSHOLD_RATE_D         0.045     // try 2 or 3 for POSHOLD_RATE 1
  #define POSHOLD_RATE_IMAX      20        // degrees

  // default Navigation PID gains
  #define NAV_P                  1.4
  #define NAV_I                  0.20      // Wind control
  #define NAV_D                  0.08      //
  #define NAV_IMAX               20        // degrees

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Serial GPS only variables
  //navigation mode
  #define NAV_MODE_NONE          0
  #define NAV_MODE_POSHOLD       1
  #define NAV_MODE_WP            2
  static uint8_t nav_mode = NAV_MODE_NONE; // Navigation mode

  static uint8_t alarmArray[16];           // array
 
#if BARO
  static int32_t baroPressure;
  static int32_t baroTemperature;
  static int32_t baroPressureSum;
#endif

void annexCode() { // this code is excetuted at each loop and won't interfere with control loop if it lasts less than 650 microseconds
  static uint32_t calibratedAccTime;
  uint16_t tmp,tmp2;
  uint8_t axis,prop1,prop2;

  // PITCH & ROLL only dynamic PID adjustemnt,  depending on throttle value
  prop2 = 128; // prop2 was 100, is 128 now
  if (rcData[THROTTLE]>1500) { // breakpoint is fix: 1500
    if (rcData[THROTTLE]<2000) {
      prop2 -=  ((uint16_t)conf.dynThrPID*(rcData[THROTTLE]-1500)>>9); //  /512 instead of /500
    } else {
      prop2 -=  conf.dynThrPID;
    }
  }

  for(axis=0;axis<3;axis++) {
    tmp = min(abs(rcData[axis]-MIDRC),500);
    #if defined(DEADBAND)
      if (tmp>DEADBAND) { tmp -= DEADBAND; }
      else { tmp=0; }
    #endif
    if(axis!=2) { //ROLL & PITCH
      tmp2 = tmp>>7; // 500/128 = 3.9  => range [0;3]
      rcCommand[axis] = lookupPitchRollRC[tmp2] + ((tmp-(tmp2<<7)) * (lookupPitchRollRC[tmp2+1]-lookupPitchRollRC[tmp2])>>7);
      prop1 = 128-((uint16_t)conf.rollPitchRate*tmp>>9); // prop1 was 100, is 128 now -- and /512 instead of /500
      prop1 = (uint16_t)prop1*prop2>>7; // prop1: max is 128   prop2: max is 128   result prop1: max is 128
    } else {      // YAW
      rcCommand[axis] = tmp;
      prop1 = 128-((uint16_t)conf.yawRate*tmp>>9); // prop1 was 100, is 128 now -- and /512 instead of /500
    }
    dynP8[axis] = (uint16_t)conf.pid[axis].P8*prop1>>7; // was /100, is /128 now
    dynD8[axis] = (uint16_t)conf.pid[axis].D8*prop1>>7; // was /100, is /128 now
    if (rcData[axis]<MIDRC) rcCommand[axis] = -rcCommand[axis];
  }
  tmp = constrain(rcData[THROTTLE],MINCHECK,2000);
  tmp = (uint32_t)(tmp-MINCHECK)*1000/(2000-MINCHECK); // [MINCHECK;2000] -> [0;1000]
  tmp2 = tmp/100;
  rcCommand[THROTTLE] = lookupThrottleRC[tmp2] + (tmp-tmp2*100) * (lookupThrottleRC[tmp2+1]-lookupThrottleRC[tmp2]) / 100; // [0;1000] -> expo -> [conf.minthrottle;MAXTHROTTLE]

  if(f.HEADFREE_MODE) { //to optimize
    float radDiff = (att.heading - headFreeModeHold) * 0.0174533f; // where PI/180 ~= 0.0174533
    float cosDiff = cos(radDiff);
    float sinDiff = sin(radDiff);
    int16_t rcCommand_PITCH = rcCommand[PITCH]*cosDiff + rcCommand[ROLL]*sinDiff;
    rcCommand[ROLL] =  rcCommand[ROLL]*cosDiff - rcCommand[PITCH]*sinDiff; 
    rcCommand[PITCH] = rcCommand_PITCH;
  }

  // query at most one multiplexed analog channel per MWii cycle
  static uint8_t analogReader =0;
  switch (analogReader++%3) {
  #if defined(POWERMETER_HARD)
  case 0:
  {
    uint16_t pMeterRaw; // used for current reading
    static uint32_t lastRead = currentTime;
    static uint8_t ind = 0;
    static uint16_t pvec[PSENSOR_SMOOTH], psum;
    uint16_t p =  analogRead(PSENSORPIN);
    //LCDprintInt16(p); LCDcrlf();
    //debug[0] = p;
    #if PSENSOR_SMOOTH != 1
      psum += p;
      psum -= pvec[ind];
      pvec[ind++] = p;
      ind %= PSENSOR_SMOOTH;
      p = psum / PSENSOR_SMOOTH;
    #endif
    powerValue = ( conf.psensornull > p ? conf.psensornull - p : p - conf.psensornull); // do not use abs(), it would induce implicit cast to uint and overrun
    if ( powerValue > 307) powerValue = 307;  // only accept reasonable values. 307 is empirical
    pMeter[PMOTOR_SUM] += ((currentTime-lastRead) * (uint32_t)(powerValue*conf.pint2ma))/100000; // [10 mA * msec]
    lastRead = currentTime;
    break;
  }
  #endif // POWERMETER_HARD

  #if defined(VBAT)
  case 1:
  {
      static uint8_t ind = 0;
      static uint16_t vvec[VBAT_SMOOTH], vsum;
      uint16_t v = analogRead(V_BATPIN);
      //debug[1] = v;
      #if VBAT_SMOOTH == 1
        analog.vbat = (v<<4) / conf.vbatscale; // result is Vbatt in 0.1V steps
      #else
        vsum += v;
        vsum -= vvec[ind];
        vvec[ind++] = v;
        ind %= VBAT_SMOOTH;
        #if VBAT_SMOOTH == 16
          analog.vbat = vsum / conf.vbatscale; // result is Vbatt in 0.1V steps
        #elif VBAT_SMOOTH < 16
          analog.vbat = (vsum * (16/VBAT_SMOOTH)) / conf.vbatscale; // result is Vbatt in 0.1V steps
        #else
          analog.vbat = ((vsum /VBAT_SMOOTH) * 16) / conf.vbatscale; // result is Vbatt in 0.1V steps
        #endif
      #endif
      break;
  }
  #endif // VBAT
  #if defined(RX_RSSI)
  case 2:
  {
    static uint8_t ind = 0;
    static uint16_t rvec[RSSI_SMOOTH], rsum;
    uint16_t r = analogRead(RX_RSSI_PIN);
    #if RSSI_SMOOTH == 1
      analog.rssi = r;
    #else
      rsum += r;
      rsum -= rvec[ind];
      rvec[ind++] = r;
      ind %= RSSI_SMOOTH;
      r = rsum / RSSI_SMOOTH;
      analog.rssi = r;
    #endif
    break;
  }
  #endif
  } // end of switch()

  #if defined(BUZZER)
    alarmHandler(); // external buzzer routine that handles buzzer events globally now
  #endif


  if ( (calibratingA>0 && ACC ) || (calibratingG>0) ) { // Calibration phasis
    LEDPIN_TOGGLE;
  } else {
    if (f.ACC_CALIBRATED) {LEDPIN_OFF;}
    if (f.ARMED) {LEDPIN_ON;}
  }

  #if defined(LED_RING)
    static uint32_t LEDTime;
    if ( currentTime > LEDTime ) {
      LEDTime = currentTime + 50000;
      i2CLedRingState();
    }
  #endif

  #if defined(LED_FLASHER)
    auto_switch_led_flasher();
  #endif

  if ( currentTime > calibratedAccTime ) {
    if (! f.SMALL_ANGLES_25) {
      // the multi uses ACC and is not calibrated or is too much inclinated
      f.ACC_CALIBRATED = 0;
      LEDPIN_TOGGLE;
      calibratedAccTime = currentTime + 100000;
    } else {
      f.ACC_CALIBRATED = 1;
    }
  }

  #if !(defined(SPEKTRUM) && defined(PROMINI))  //Only one serial port on ProMini.  Skip serial com if Spektrum Sat in use. Note: Spek code will auto-call serialCom if GUI data detected on serial0.
    #if defined(GPS_PROMINI)
      if(GPS_Enable == 0) {serialCom();}
    #else
      serialCom();
    #endif
  #endif

  #if defined(POWERMETER)
    analog.intPowerMeterSum = (pMeter[PMOTOR_SUM]/PLEVELDIV);
    intPowerTrigger1 = conf.powerTrigger1 * PLEVELSCALE; 
  #endif

  #ifdef LCD_TELEMETRY_AUTO
    static char telemetryAutoSequence []  = LCD_TELEMETRY_AUTO;
    static uint8_t telemetryAutoIndex = 0;
    static uint16_t telemetryAutoTimer = 0;
    if ( (telemetry_auto) && (! (++telemetryAutoTimer % LCD_TELEMETRY_AUTO_FREQ) )  ){
      telemetry = telemetryAutoSequence[++telemetryAutoIndex % strlen(telemetryAutoSequence)];
      LCDclear(); // make sure to clear away remnants
    }
  #endif  
  #ifdef LCD_TELEMETRY
    static uint16_t telemetryTimer = 0;
    if (! (++telemetryTimer % LCD_TELEMETRY_FREQ)) {
      #if (LCD_TELEMETRY_DEBUG+0 > 0)
        telemetry = LCD_TELEMETRY_DEBUG;
      #endif
      if (telemetry) lcd_telemetry();
    }
  #endif

  #if GPS & defined(GPS_LED_INDICATOR)       // modified by MIS to use STABLEPIN LED for number of sattelites indication
    static uint32_t GPSLEDTime;              // - No GPS FIX -> LED blink at speed of incoming GPS frames
    static uint8_t blcnt;                    // - Fix and sat no. bellow 5 -> LED off
    if(currentTime > GPSLEDTime) {           // - Fix and sat no. >= 5 -> LED blinks, one blink for 5 sat, two blinks for 6 sat, three for 7 ...
      if(f.GPS_FIX && GPS_numSat >= 5) {
        if(++blcnt > 2*GPS_numSat) blcnt = 0;
        GPSLEDTime = currentTime + 150000;
        if(blcnt >= 10 && ((blcnt%2) == 0)) {STABLEPIN_ON;} else {STABLEPIN_OFF;}
      }else{
        if((GPS_update == 1) && !f.GPS_FIX) {STABLEPIN_ON;} else {STABLEPIN_OFF;}
        blcnt = 0;
      }
    }
  #endif

  #if defined(LOG_VALUES) && (LOG_VALUES >= 2)
    if (cycleTime > cycleTimeMax) cycleTimeMax = cycleTime; // remember highscore
    if (cycleTime < cycleTimeMin) cycleTimeMin = cycleTime; // remember lowscore
  #endif
  if (f.ARMED)  {
    #if defined(LCD_TELEMETRY) || defined(ARMEDTIMEWARNING) || defined(LOG_PERMANENT)
      armedTime += (uint32_t)cycleTime;
    #endif
    #if defined(VBAT)
      if ( (analog.vbat > NO_VBAT) && (analog.vbat < vbatMin) ) vbatMin = analog.vbat;
    #endif
    #ifdef LCD_TELEMETRY
      #if BARO
        if ( (BaroAlt > BAROaltMax) ) BAROaltMax = BaroAlt;
      #endif
    #endif
  }
}

void setup() {
  #if !defined(GPS_PROMINI)
    SerialOpen(0,SERIAL0_COM_SPEED);
    #if defined(PROMICRO)
      SerialOpen(1,SERIAL1_COM_SPEED);
    #endif
    #if defined(MEGA)
      SerialOpen(1,SERIAL1_COM_SPEED);
      SerialOpen(2,SERIAL2_COM_SPEED);
      SerialOpen(3,SERIAL3_COM_SPEED);
    #endif
  #endif
  LEDPIN_PINMODE;
  POWERPIN_PINMODE;
  BUZZERPIN_PINMODE;
  STABLEPIN_PINMODE;
  POWERPIN_OFF;
  initOutput();
  readGlobalSet();
  #ifndef NO_FLASH_CHECK
    #if defined(MEGA)
      uint16_t i = 65000;                             // only first ~64K for mega board due to pgm_read_byte limitation
    #else
      uint16_t i = 32000;
    #endif
    uint16_t flashsum = 0;
    uint8_t pbyt;
    while(i--) {
      pbyt =  pgm_read_byte(i);        // calculate flash checksum
      flashsum += pbyt;
      flashsum ^= (pbyt<<8);
    }
  #endif
  #ifdef MULTIPLE_CONFIGURATION_PROFILES
    global_conf.currentSet=2;
  #else
    global_conf.currentSet=0;
  #endif
  while(1) {                                                    // check settings integrity
  #ifndef NO_FLASH_CHECK
    if(readEEPROM()) {                                          // check current setting integrity
      if(flashsum != global_conf.flashsum) update_constants();  // update constants if firmware is changed and integrity is OK
    }
  #else
    readEEPROM();                                               // check current setting integrity
  #endif  
    if(global_conf.currentSet == 0) break;                      // all checks is done
    global_conf.currentSet--;                                   // next setting for check
  }
  readGlobalSet();                              // reload global settings for get last profile number
  #ifndef NO_FLASH_CHECK
    if(flashsum != global_conf.flashsum) {
      global_conf.flashsum = flashsum;          // new flash sum
      writeGlobalSet(1);                        // update flash sum in global config
    }
  #endif
  readEEPROM();                                 // load setting data from last used profile
  blinkLED(2,40,global_conf.currentSet+1);          
  configureReceiver();
  #if defined (PILOTLAMP) 
    PL_INIT;
  #endif
  #if defined(OPENLRSv2MULTI)
    initOpenLRS();
  #endif
  initSensors();
  #if defined(I2C_GPS) || defined(GPS_SERIAL) || defined(GPS_FROM_OSD)
    GPS_set_pids();
  #endif
  previousTime = micros();
  #if defined(GIMBAL)
   calibratingA = 512;
  #endif
  calibratingG = 512;
  calibratingB = 200;  // 10 seconds init_delay + 200 * 25 ms = 15 seconds before ground pressure settles
  #if defined(POWERMETER)
    for(uint8_t j=0; j<=PMOTOR_SUM; j++) pMeter[j]=0;
  #endif
  /************************************/
  #if defined(GPS_SERIAL)
    GPS_SerialInit();
    for(uint8_t j=0;j<=5;j++){
      GPS_NewData(); 
      LEDPIN_ON
      delay(20);
      LEDPIN_OFF
      delay(80);
    }
    if(!GPS_Present){
      SerialEnd(GPS_SERIAL);
      SerialOpen(0,SERIAL0_COM_SPEED);
    }
    #if !defined(GPS_PROMINI)
      GPS_Present = 1;
    #endif
    GPS_Enable = GPS_Present;    
  #endif
  /************************************/
 
  #if defined(I2C_GPS) || defined(TINY_GPS) || defined(GPS_FROM_OSD)
   GPS_Enable = 1;
  #endif
  
  #if defined(LCD_ETPP) || defined(LCD_LCD03) || defined(OLED_I2C_128x64) || defined(OLED_DIGOLE) || defined(LCD_TELEMETRY_STEP)
    initLCD();
  #endif
  #ifdef LCD_TELEMETRY_DEBUG
    telemetry_auto = 1;
  #endif
  #ifdef LCD_CONF_DEBUG
    configurationLoop();
  #endif
  #ifdef LANDING_LIGHTS_DDR
    init_landing_lights();
  #endif
  #ifdef FASTER_ANALOG_READS
    ADCSRA |= _BV(ADPS2) ; ADCSRA &= ~_BV(ADPS1); ADCSRA &= ~_BV(ADPS0); // this speeds up analogRead without loosing too much resolution: http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1208715493/11
  #endif
  #if defined(LED_FLASHER)
    init_led_flasher();
    led_flasher_set_sequence(LED_FLASHER_SEQUENCE);
  #endif
  f.SMALL_ANGLES_25=1; // important for gyro only conf
  #ifdef LOG_PERMANENT
    // read last stored set
    readPLog();
    plog.lifetime += plog.armed_time / 1000000;
    plog.start++;         // #powercycle/reset/initialize events
    // dump plog data to terminal
    #ifdef LOG_PERMANENT_SHOW_AT_STARTUP
      dumpPLog(0);
    #endif
    plog.armed_time = 0;   // lifetime in seconds
    //plog.running = 0;       // toggle on arm & disarm to monitor for clean shutdown vs. powercut
  #endif

  debugmsg_append_str("initialization completed\n");
}

void go_arm() {
  if(calibratingG == 0 && f.ACC_CALIBRATED 
  #if defined(FAILSAFE)
    && failsafeCnt < 2
  #endif
    ) {
    if(!f.ARMED) { // arm now!
      f.ARMED = 1;
      headFreeModeHold = att.heading;
      #if defined(VBAT)
        if (analog.vbat > NO_VBAT) vbatMin = analog.vbat;
      #endif
      #ifdef LCD_TELEMETRY // reset some values when arming
        #if BARO
          BAROaltMax = BaroAlt;
        #endif
        #if GPS
          GPS_speedMaxKmh = 0;
        #endif
        #ifdef POWERMETER_HARD
          powerValueMaxMAH = 0;
        #endif
      #endif
      #ifdef LOG_PERMANENT
        plog.arm++;           // #arm events
        plog.running = 1;       // toggle on arm & disarm to monitor for clean shutdown vs. powercut
        // write now.
        writePLog();
      #endif
    }
  } else if(!f.ARMED) { 
    blinkLED(2,255,1);
    alarmArray[8] = 1;
  }
}
void go_disarm() {
  if (f.ARMED) {
    f.ARMED = 0;
    #ifdef LOG_PERMANENT
      plog.disarm++;        // #disarm events
      plog.armed_time = armedTime ;   // lifetime in seconds
      if (failsafeEvents) plog.failsafe++;      // #acitve failsafe @ disarm
      if (i2c_errors_count > 10) plog.i2c++;           // #i2c errs @ disarm
      plog.running = 0;       // toggle @ arm & disarm to monitor for clean shutdown vs. powercut
      // write now.
      writePLog();
    #endif
  }
}

// ******** Main Loop *********
void loop () {
  static uint8_t rcDelayCommand; // this indicates the number of time (multiple of RC measurement at 50Hz) the sticks must be maintained to run or switch off motors
  static uint8_t rcSticks;       // this hold sticks position for command combos
  uint8_t axis,i;
  int16_t error,errorAngle;
  int16_t delta,deltaSum;
  int16_t PTerm,ITerm,DTerm;
  int16_t PTermACC = 0 , ITermACC = 0 , PTermGYRO = 0 , ITermGYRO = 0;
  static int16_t lastGyro[3] = {0,0,0};
  static int16_t delta1[3],delta2[3];
  static int16_t errorGyroI[3] = {0,0,0};
  static int16_t errorAngleI[2] = {0,0};
  static uint32_t rcTime  = 0;
  static int16_t initialThrottleHold;
  static uint32_t timestamp_fixated = 0;

  #if defined(SPEKTRUM)
    if (spekFrameFlags == 0x01) readSpektrum();
  #endif
  
  #if defined(OPENLRSv2MULTI) 
    Read_OpenLRS_RC();
  #endif 

  if (currentTime > rcTime ) { // 50Hz
    rcTime = currentTime + 20000;
    computeRC();
    // Failsafe routine - added by MIS
    #if defined(FAILSAFE)
      if ( failsafeCnt > (5*FAILSAFE_DELAY) && f.ARMED) {                  // Stabilize, and set Throttle to specified level
        for(i=0; i<3; i++) rcData[i] = MIDRC;                               // after specified guard time after RC signal is lost (in 0.1sec)
        rcData[THROTTLE] = conf.failsafe_throttle;
        if (failsafeCnt > 5*(FAILSAFE_DELAY+FAILSAFE_OFF_DELAY)) {          // Turn OFF motors after specified Time (in 0.1sec)
          go_disarm();     // This will prevent the copter to automatically rearm if failsafe shuts it down and prevents
          f.OK_TO_ARM = 0; // to restart accidentely by just reconnect to the tx - you will have to switch off first to rearm
        }
        failsafeEvents++;
      }
      if ( failsafeCnt > (5*FAILSAFE_DELAY) && !f.ARMED) {  //Turn of "Ok To arm to prevent the motors from spinning after repowering the RX with low throttle and aux to arm
          go_disarm();     // This will prevent the copter to automatically rearm if failsafe shuts it down and prevents
          f.OK_TO_ARM = 0; // to restart accidentely by just reconnect to the tx - you will have to switch off first to rearm
      }
      failsafeCnt++;
    #endif
    // end of failsafe routine - next change is made with RcOptions setting

    // ------------------ STICKS COMMAND HANDLER --------------------
    // checking sticks positions
    uint8_t stTmp = 0;
    for(i=0;i<4;i++) {
      stTmp >>= 2;
      if(rcData[i] > MINCHECK) stTmp |= 0x80;      // check for MIN
      if(rcData[i] < MAXCHECK) stTmp |= 0x40;      // check for MAX
    }
    if(stTmp == rcSticks) {
      if(rcDelayCommand<250) rcDelayCommand++;
    } else rcDelayCommand = 0;
    rcSticks = stTmp;
    
    // perform actions    
    if (rcData[THROTTLE] <= MINCHECK) {            // THROTTLE at minimum
      errorGyroI[ROLL] = 0; errorGyroI[PITCH] = 0; errorGyroI[YAW] = 0;
      errorAngleI[ROLL] = 0; errorAngleI[PITCH] = 0;
      if (conf.activate[BOXARM] > 0) {             // Arming/Disarming via ARM BOX
        if ( rcOptions[BOXARM] && f.OK_TO_ARM ) go_arm(); else if (f.ARMED) go_disarm();
      }
    }
    if(rcDelayCommand == 20) {
      if(f.ARMED) {                   // actions during armed
        #ifdef ALLOW_ARM_DISARM_VIA_TX_YAW
          if (conf.activate[BOXARM] == 0 && rcSticks == THR_LO + YAW_LO + PIT_CE + ROL_CE) go_disarm();    // Disarm via YAW
        #endif
        #ifdef ALLOW_ARM_DISARM_VIA_TX_ROLL
          if (conf.activate[BOXARM] == 0 && rcSticks == THR_LO + YAW_CE + PIT_CE + ROL_LO) go_disarm();    // Disarm via ROLL
        #endif
      } else {                        // actions during not armed
        i=0;
        if (rcSticks == THR_LO + YAW_LO + PIT_LO + ROL_CE) {    // GYRO calibration
          calibratingG=512;
          #if GPS 
            GPS_reset_home_position();
          #endif
          #if BARO
            calibratingB=10;  // calibrate baro to new ground level (10 * 25 ms = ~250 ms non blocking)
          #endif
        }
        #if defined(INFLIGHT_ACC_CALIBRATION)  
         else if (rcSticks == THR_LO + YAW_LO + PIT_HI + ROL_HI) {    // Inflight ACC calibration START/STOP
            if (AccInflightCalibrationMeasurementDone){                // trigger saving into eeprom after landing
              AccInflightCalibrationMeasurementDone = 0;
              AccInflightCalibrationSavetoEEProm = 1;
            }else{ 
              AccInflightCalibrationArmed = !AccInflightCalibrationArmed; 
              #if defined(BUZZER)
               if (AccInflightCalibrationArmed) alarmArray[0]=2; else   alarmArray[0]=3;
              #endif
            }
         } 
        #endif
        #ifdef MULTIPLE_CONFIGURATION_PROFILES
          if      (rcSticks == THR_LO + YAW_LO + PIT_CE + ROL_LO) i=1;    // ROLL left  -> Profile 1
          else if (rcSticks == THR_LO + YAW_LO + PIT_HI + ROL_CE) i=2;    // PITCH up   -> Profile 2
          else if (rcSticks == THR_LO + YAW_LO + PIT_CE + ROL_HI) i=3;    // ROLL right -> Profile 3
          if(i) {
            global_conf.currentSet = i-1;
            writeGlobalSet(0);
            readEEPROM();
            blinkLED(2,40,i);
            alarmArray[0] = i;
          }
        #endif
        if (rcSticks == THR_LO + YAW_HI + PIT_HI + ROL_CE) {            // Enter LCD config
          #if defined(LCD_CONF)
            configurationLoop(); // beginning LCD configuration
          #endif
          previousTime = micros();
        }
        #ifdef ALLOW_ARM_DISARM_VIA_TX_YAW
          else if (conf.activate[BOXARM] == 0 && rcSticks == THR_LO + YAW_HI + PIT_CE + ROL_CE) go_arm();      // Arm via YAW
        #endif
        #ifdef ALLOW_ARM_DISARM_VIA_TX_ROLL
          else if (conf.activate[BOXARM] == 0 && rcSticks == THR_LO + YAW_CE + PIT_CE + ROL_HI) go_arm();      // Arm via ROLL
        #endif
        #ifdef LCD_TELEMETRY_AUTO
          else if (rcSticks == THR_LO + YAW_CE + PIT_HI + ROL_LO) {              // Auto telemetry ON/OFF
            if (telemetry_auto) {
              telemetry_auto = 0;
              telemetry = 0;
            } else
              telemetry_auto = 1;
          }
        #endif
        #ifdef LCD_TELEMETRY_STEP
          else if (rcSticks == THR_LO + YAW_CE + PIT_HI + ROL_HI) {              // Telemetry next step
            telemetry = telemetryStepSequence[++telemetryStepIndex % strlen(telemetryStepSequence)];
            #ifdef OLED_I2C_128x64
              if (telemetry != 0) i2c_OLED_init();
            #endif
            LCDclear();
          }
        #endif
        else if (rcSticks == THR_HI + YAW_LO + PIT_LO + ROL_CE) calibratingA=512;     // throttle=max, yaw=left, pitch=min
        else if (rcSticks == THR_HI + YAW_HI + PIT_LO + ROL_CE) f.CALIBRATE_MAG = 1;  // throttle=max, yaw=right, pitch=min  
        i=0;
        if      (rcSticks == THR_HI + YAW_CE + PIT_HI + ROL_CE) {conf.angleTrim[PITCH]+=2; i=1;}
        else if (rcSticks == THR_HI + YAW_CE + PIT_LO + ROL_CE) {conf.angleTrim[PITCH]-=2; i=1;}
        else if (rcSticks == THR_HI + YAW_CE + PIT_CE + ROL_HI) {conf.angleTrim[ROLL] +=2; i=1;}
        else if (rcSticks == THR_HI + YAW_CE + PIT_CE + ROL_LO) {conf.angleTrim[ROLL] -=2; i=1;}
        if (i) {
          writeParams(1);
          rcDelayCommand = 0;    // allow autorepetition
          #if defined(LED_RING)
            blinkLedRing();
          #endif
        }
      }
    }
    #if defined(LED_FLASHER)
      led_flasher_autoselect_sequence();
    #endif
    
    #if defined(INFLIGHT_ACC_CALIBRATION)
      if (AccInflightCalibrationArmed && f.ARMED && rcData[THROTTLE] > MINCHECK && !rcOptions[BOXARM] ){ // Copter is airborne and you are turning it off via boxarm : start measurement
        InflightcalibratingA = 50;
        AccInflightCalibrationArmed = 0;
      }  
      if (rcOptions[BOXCALIB]) {      // Use the Calib Option to activate : Calib = TRUE Meausrement started, Land and Calib = 0 measurement stored
        if (!AccInflightCalibrationActive && !AccInflightCalibrationMeasurementDone){
          InflightcalibratingA = 50;
        }
      }else if(AccInflightCalibrationMeasurementDone && !f.ARMED){
        AccInflightCalibrationMeasurementDone = 0;
        AccInflightCalibrationSavetoEEProm = 1;
      }
    #endif

    uint16_t auxState = 0;
    for(i=0;i<4;i++)
      auxState |= (rcData[AUX1+i]<1300)<<(3*i) | (1300<rcData[AUX1+i] && rcData[AUX1+i]<1700)<<(3*i+1) | (rcData[AUX1+i]>1700)<<(3*i+2);
    for(i=0;i<CHECKBOXITEMS;i++)
      rcOptions[i] = (auxState & conf.activate[i])>0;

    // note: if FAILSAFE is disable, failsafeCnt > 5*FAILSAFE_DELAY is always false
    #if ACC
      if ( rcOptions[BOXANGLE] || (failsafeCnt > 5*FAILSAFE_DELAY) ) { 
        // bumpless transfer to Level mode
        if (!f.ANGLE_MODE) {
          errorAngleI[ROLL] = 0; errorAngleI[PITCH] = 0;
          f.ANGLE_MODE = 1;
        }  
      } else {
        // failsafe support
        f.ANGLE_MODE = 0;
      }
      if ( rcOptions[BOXHORIZON] ) {
        f.ANGLE_MODE = 0;
        if (!f.HORIZON_MODE) {
          errorAngleI[ROLL] = 0; errorAngleI[PITCH] = 0;
          f.HORIZON_MODE = 1;
        }
      } else {
        f.HORIZON_MODE = 0;
      }
    #endif

    if (rcOptions[BOXARM] == 0) f.OK_TO_ARM = 1;
    #if !defined(GPS_LED_INDICATOR)
      if (f.ANGLE_MODE || f.HORIZON_MODE) {STABLEPIN_ON;} else {STABLEPIN_OFF;}
    #endif

    #if BARO
      #if (!defined(SUPPRESS_BARO_ALTHOLD))
        if (rcOptions[BOXBARO]) {
            if (!f.BARO_MODE) {
              f.BARO_MODE = 1;
              AltHold = alt.EstAlt;
              initialThrottleHold = rcCommand[THROTTLE];
              errorAltitudeI = 0;
              BaroPID=0;
            }
        } else {
            f.BARO_MODE = 0;
        }
      #endif
      #ifdef VARIOMETER
        if (rcOptions[BOXVARIO]) {
          if (!f.VARIO_MODE) {
            f.VARIO_MODE = 1;
          }
        } else {
          f.VARIO_MODE = 0;
        }
      #endif
    #endif
    #if MAG
      if (rcOptions[BOXMAG]) {
        if (!f.MAG_MODE) {
          f.MAG_MODE = 1;
          magHold = att.heading;
        }
      } else {
        f.MAG_MODE = 0;
      }
      if (rcOptions[BOXHEADFREE]) {
        if (!f.HEADFREE_MODE) {
          f.HEADFREE_MODE = 1;
        }
	  #if defined(ADVANCED_HEADFREE)
		if ((f.GPS_FIX && GPS_numSat >= 5) && (GPS_distanceToHome > ADV_HEADFREE_RANGE) ) {
            if (GPS_directionToHome < 180)  {headFreeModeHold = GPS_directionToHome + 180;} else {headFreeModeHold = GPS_directionToHome - 180;}
         }
      #endif
      } else {
        f.HEADFREE_MODE = 0;
      }
      if (rcOptions[BOXHEADADJ]) {
        headFreeModeHold = att.heading; // acquire new heading
      }
    #endif
    
    #if GPS
      static uint8_t GPSNavReset = 1;
      if (f.GPS_FIX && GPS_numSat >= 5 ) {
        if (rcOptions[BOXGPSHOME]) {  // if both GPS_HOME & GPS_HOLD are checked => GPS_HOME is the priority
          if (!f.GPS_HOME_MODE)  {
            f.GPS_HOME_MODE = 1;
            f.GPS_HOLD_MODE = 0;
            GPSNavReset = 0;
            #if defined(I2C_GPS)
              GPS_I2C_command(I2C_GPS_COMMAND_START_NAV,0);        //waypoint zero
            #else // SERIAL
              GPS_set_next_wp(&GPS_home[LAT],&GPS_home[LON]);
              nav_mode    = NAV_MODE_WP;
            #endif
          }
        } else {
          f.GPS_HOME_MODE = 0;
          if (rcOptions[BOXGPSHOLD] && abs(rcCommand[ROLL])< AP_MODE && abs(rcCommand[PITCH]) < AP_MODE) {
            if (!f.GPS_HOLD_MODE) {
              f.GPS_HOLD_MODE = 1;
              GPSNavReset = 0;
              #if defined(I2C_GPS)
                GPS_I2C_command(I2C_GPS_COMMAND_POSHOLD,0);
              #else
                GPS_hold[LAT] = GPS_coord[LAT];
                GPS_hold[LON] = GPS_coord[LON];
                GPS_set_next_wp(&GPS_hold[LAT],&GPS_hold[LON]);
                nav_mode = NAV_MODE_POSHOLD;
              #endif
            }
          } else {
            f.GPS_HOLD_MODE = 0;
            // both boxes are unselected here, nav is reset if not already done
            if (GPSNavReset == 0 ) {
              GPSNavReset = 1;
              GPS_reset_nav();
            }
          }
        }
      } else {
        f.GPS_HOME_MODE = 0;
        f.GPS_HOLD_MODE = 0;
        #if !defined(I2C_GPS)
          nav_mode = NAV_MODE_NONE;
        #endif
      }
    #endif
    
    #if defined(FIXEDWING) || defined(HELICOPTER)
      if (rcOptions[BOXPASSTHRU]) {f.PASSTHRU_MODE = 1;}
      else {f.PASSTHRU_MODE = 0;}
    #endif
 
  } else { // not in rc loop
    static uint8_t taskOrder=0; // never call all functions in the same loop, to avoid high delay spikes
    if(taskOrder>4) taskOrder-=5;
    switch (taskOrder) {
      case 0:
        taskOrder++;
        #if MAG
          if (Mag_getADC()) break; // max 350 µs (HMC5883) // only break when we actually did something
        #endif
      case 1:
        taskOrder++;
        #if BARO
          if (Baro_update() != 0 ) break;
        #endif
      case 2:
        taskOrder++;
        #if BARO
          if (getEstimatedAltitude() !=0) break;
        #endif    
      case 3:
        taskOrder++;
        #if GPS
          if(GPS_Enable) GPS_NewData();
          break;
        #endif
      case 4:
        taskOrder++;
        #if SONAR
          Sonar_update(); //debug[2] = sonarAlt;
        #endif
        #ifdef LANDING_LIGHTS_DDR
          auto_switch_landing_lights();
        #endif
        #ifdef VARIOMETER
          if (f.VARIO_MODE) vario_signaling();
        #endif
        break;
    }
  }
 
  computeIMU();
  // Measure loop rate just afer reading the sensors
  currentTime = micros();
  cycleTime = currentTime - previousTime;
  previousTime = currentTime;

  //***********************************
  //**** Experimental FlightModes *****
  //***********************************
  #if defined(ACROTRAINER_MODE)
    if(f.ANGLE_MODE){
      if (abs(rcCommand[ROLL]) + abs(rcCommand[PITCH]) >= ACROTRAINER_MODE ) {
        f.ANGLE_MODE=0;
        f.HORIZON_MODE=0;
        f.MAG_MODE=0;
        f.BARO_MODE=0;
        f.GPS_HOME_MODE=0;
        f.GPS_HOLD_MODE=0;
      }
    }
  #endif

 //*********************************** 
 
  #if MAG
    if (abs(rcCommand[YAW]) <70 && f.MAG_MODE) {
      int16_t dif = att.heading - magHold;
      if (dif <= - 180) dif += 360;
      if (dif >= + 180) dif -= 360;
      if ( f.SMALL_ANGLES_25 ) rcCommand[YAW] -= dif*conf.pid[PIDMAG].P8>>5;
    } else magHold = att.heading;
  #endif

  #if BARO && (!defined(SUPPRESS_BARO_ALTHOLD))
    if (f.BARO_MODE) {
      static uint8_t isAltHoldChanged = 0;
      #if defined(ALTHOLD_FAST_THROTTLE_CHANGE)
        if (abs(rcCommand[THROTTLE]-initialThrottleHold) > ALT_HOLD_THROTTLE_NEUTRAL_ZONE) {
          errorAltitudeI = 0;
          isAltHoldChanged = 1;
          rcCommand[THROTTLE] += (rcCommand[THROTTLE] > initialThrottleHold) ? -ALT_HOLD_THROTTLE_NEUTRAL_ZONE : ALT_HOLD_THROTTLE_NEUTRAL_ZONE;
        } else {
          if (isAltHoldChanged) {
            AltHold = alt.EstAlt;
            isAltHoldChanged = 0;
          }
          rcCommand[THROTTLE] = initialThrottleHold + BaroPID;
        }
      #else
        static int16_t AltHoldCorr = 0;
        if (abs(rcCommand[THROTTLE]-initialThrottleHold)>ALT_HOLD_THROTTLE_NEUTRAL_ZONE) {
          // Slowly increase/decrease AltHold proportional to stick movement ( +100 throttle gives ~ +50 cm in 1 second with cycle time about 3-4ms)
          AltHoldCorr+= rcCommand[THROTTLE] - initialThrottleHold;
          if(abs(AltHoldCorr) > 500) {
            AltHold += AltHoldCorr/500;
            AltHoldCorr %= 500;
          }
          errorAltitudeI = 0;
          isAltHoldChanged = 1;
        } else if (isAltHoldChanged) {
          AltHold = alt.EstAlt;
          isAltHoldChanged = 0;
        }
        rcCommand[THROTTLE] = initialThrottleHold + BaroPID;
      #endif
    }
  #endif

  #if defined(THROTTLE_ANGLE_CORRECTION)
    if(f.ANGLE_MODE || f.HORIZON_MODE) {
       rcCommand[THROTTLE]+= throttleAngleCorrection;
    }
  #endif
  
  #if GPS
    if ( (f.GPS_HOME_MODE || f.GPS_HOLD_MODE) && f.GPS_FIX_HOME ) {
      float sin_yaw_y = sin(att.heading*0.0174532925f);
      float cos_yaw_x = cos(att.heading*0.0174532925f);
      #if defined(NAV_SLEW_RATE)     
        nav_rated[LON]   += constrain(wrap_18000(nav[LON]-nav_rated[LON]),-NAV_SLEW_RATE,NAV_SLEW_RATE);
        nav_rated[LAT]   += constrain(wrap_18000(nav[LAT]-nav_rated[LAT]),-NAV_SLEW_RATE,NAV_SLEW_RATE);
        GPS_angle[ROLL]   = (nav_rated[LON]*cos_yaw_x - nav_rated[LAT]*sin_yaw_y) /10;
        GPS_angle[PITCH]  = (nav_rated[LON]*sin_yaw_y + nav_rated[LAT]*cos_yaw_x) /10;
      #else 
        GPS_angle[ROLL]   = (nav[LON]*cos_yaw_x - nav[LAT]*sin_yaw_y) /10;
        GPS_angle[PITCH]  = (nav[LON]*sin_yaw_y + nav[LAT]*cos_yaw_x) /10;
      #endif
    } else {
      GPS_angle[ROLL]  = 0;
      GPS_angle[PITCH] = 0;
    }
  #endif

  //**** PITCH & ROLL & YAW PID ****
  int16_t prop;
  prop = min(max(abs(rcCommand[PITCH]),abs(rcCommand[ROLL])),500); // range [0;500]

  for(axis=0;axis<3;axis++) {
    if ((f.ANGLE_MODE || f.HORIZON_MODE) && axis<2 ) { // MODE relying on ACC
      // 50 degrees max inclination
      errorAngle = constrain((rcCommand[axis]<<1) + GPS_angle[axis],-500,+500) - att.angle[axis] + conf.angleTrim[axis]; //16 bits is ok here
      PTermACC = ((int32_t)errorAngle*conf.pid[PIDLEVEL].P8)>>7;                          // 32 bits is needed for calculation: errorAngle*P8[PIDLEVEL] could exceed 32768   16 bits is ok for result
      PTermACC = constrain(PTermACC,-conf.pid[PIDLEVEL].D8*5,+conf.pid[PIDLEVEL].D8*5);

      errorAngleI[axis]     = constrain(errorAngleI[axis]+errorAngle,-10000,+10000);    // WindUp     //16 bits is ok here
      ITermACC              = ((int32_t)errorAngleI[axis]*conf.pid[PIDLEVEL].I8)>>12;            // 32 bits is needed for calculation:10000*I8 could exceed 32768   16 bits is ok for result
    }
    if ( !f.ANGLE_MODE || f.HORIZON_MODE || axis == 2 ) { // MODE relying on GYRO or YAW axis
      if (abs(rcCommand[axis])<500) error =          (rcCommand[axis]<<6)/conf.pid[axis].P8 ; // 16 bits is needed for calculation: 500*64 = 32000      16 bits is ok for result if P8>5 (P>0.5)
                               else error = ((int32_t)rcCommand[axis]<<6)/conf.pid[axis].P8 ; // 32 bits is needed for calculation

      error -= imu.gyroData[axis];

      PTermGYRO = rcCommand[axis];
      
      errorGyroI[axis]  = constrain(errorGyroI[axis]+error,-16000,+16000);         // WindUp   16 bits is ok here
      if (abs(imu.gyroData[axis])>640) errorGyroI[axis] = 0;
      ITermGYRO = ((errorGyroI[axis]>>7)*conf.pid[axis].I8)>>6;                        // 16 bits is ok here 16000/125 = 128 ; 128*250 = 32000
    }
    if ( f.HORIZON_MODE && axis<2) {
      PTerm = ((int32_t)PTermACC*(512-prop) + (int32_t)PTermGYRO*prop)>>9;         // the real factor should be 500, but 512 is ok
      ITerm = ((int32_t)ITermACC*(512-prop) + (int32_t)ITermGYRO*prop)>>9;
    } else {
      if ( f.ANGLE_MODE && axis<2) {
        PTerm = PTermACC;
        ITerm = ITermACC;
      } else {
        PTerm = PTermGYRO;
        ITerm = ITermGYRO;
      }
    }

    PTerm -= ((int32_t)imu.gyroData[axis]*dynP8[axis])>>6; // 32 bits is needed for calculation   

    delta          = imu.gyroData[axis] - lastGyro[axis];  // 16 bits is ok here, the dif between 2 consecutive gyro reads is limited to 800
    lastGyro[axis] = imu.gyroData[axis];
    deltaSum       = delta1[axis]+delta2[axis]+delta;
    delta2[axis]   = delta1[axis];
    delta1[axis]   = delta;
 
    DTerm = ((int32_t)deltaSum*dynD8[axis])>>5;        // 32 bits is needed for calculation
                      
    axisPID[axis] =  PTerm + ITerm - DTerm;
  }

  mixTable();
  // do not update servos during unarmed calibration of sensors which are sensitive to vibration
  if ( (f.ARMED) || ((!calibratingG) && (!calibratingA)) ) writeServos();
  writeMotors();
}
