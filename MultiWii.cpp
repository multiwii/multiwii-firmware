/*
MultiWiiCopter by Alexandre Dubus
www.multiwii.com
November  2013     V2.3
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 any later version. see <http://www.gnu.org/licenses/>
*/

#include <avr/io.h>

#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "MultiWii.h"
#include "Alarms.h"
#include "EEPROM.h"
#include "IMU.h"
#include "LCD.h"
#include "Output.h"
#include "RX.h"
#include "Sensors.h"
#include "Serial.h"
#include "GPS.h"
#include "Protocol.h"

#include <avr/pgmspace.h>

/*********** RC alias *****************/

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
  "MAG;"
  #if defined(HEADFREE)
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
#if GPS
  "MISSION;"
  "LAND;"
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
  5, //"MAG;"
#if defined(HEADFREE)
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
#if GPS
  20, //"MISSION;"
  21, //"LAND;"
#endif
  };


uint32_t currentTime = 0;
uint16_t previousTime = 0;
uint16_t cycleTime = 0;     // this is the number in micro second to achieve a full loop, it can differ a little and is taken into account in the PID loop
uint16_t calibratingA = 0;  // the calibration is done in the main loop. Calibrating decreases at each cycle down to 0, then we enter in a normal mode.
uint16_t calibratingB = 0;  // baro calibration = get new ground pressure value
uint16_t calibratingG;
int16_t  magHold,headFreeModeHold; // [-180;+180]
uint8_t  vbatMin = VBATNOMINAL;  // lowest battery voltage in 0.1V steps
uint8_t  rcOptions[CHECKBOXITEMS];
int32_t  AltHold; // in cm
int16_t  sonarAlt;
int16_t  BaroPID = 0;
int16_t  errorAltitudeI = 0;

// **************
// gyro+acc IMU
// **************
int16_t gyroZero[3] = {0,0,0};

imu_t imu;

analog_t analog;

alt_t alt;

att_t att;

#if defined(ARMEDTIMEWARNING)
  uint32_t  ArmedTimeWarningMicroSeconds = 0;
#endif

int16_t  debug[4];

flags_struct_t f;

//for log
#if defined(LOG_VALUES) || defined(LCD_TELEMETRY)
  uint16_t cycleTimeMax = 0;       // highest ever cycle timen
  uint16_t cycleTimeMin = 65535;   // lowest ever cycle timen
  int32_t  BAROaltMax;             // maximum value
  uint16_t GPS_speedMax = 0;       // maximum speed from gps
  #ifdef POWERMETER_HARD
    uint16_t powerValueMaxMAH = 0;
  #endif
  #if defined(WATTS)
    uint16_t wattsMax = 0;
  #endif
#endif
#if defined(LOG_VALUES) || defined(LCD_TELEMETRY) || defined(ARMEDTIMEWARNING) || defined(LOG_PERMANENT)
  uint32_t armedTime = 0;
#endif

int16_t  i2c_errors_count = 0;


#if defined(THROTTLE_ANGLE_CORRECTION)
  int16_t throttleAngleCorrection = 0; // correction of throttle in lateral wind,
  int8_t  cosZ = 100;                  // cos(angleZ)*100
#endif



// **********************
//Automatic ACC Offset Calibration
// **********************
#if defined(INFLIGHT_ACC_CALIBRATION)
  uint16_t InflightcalibratingA = 0;
  int16_t AccInflightCalibrationArmed;
  uint16_t AccInflightCalibrationMeasurementDone = 0;
  uint16_t AccInflightCalibrationSavetoEEProm = 0;
  uint16_t AccInflightCalibrationActive = 0;
#endif

// **********************
// power meter
// **********************
#if defined(POWERMETER) || ( defined(LOG_VALUES) && (LOG_VALUES >= 3) )
  uint32_t pMeter[PMOTOR_SUM + 1];  // we use [0:7] for eight motors,one extra for sum
  uint8_t pMeterV;                  // dummy to satisfy the paramStruct logic in ConfigurationLoop()
  uint32_t pAlarm;                  // we scale the eeprom value from [0:255] to this value we can directly compare to the sum in pMeter[6]
  uint16_t powerValue = 0;          // last known current
#endif
uint16_t intPowerTrigger1;

// **********************
// telemetry
// **********************
#if defined(LCD_TELEMETRY)
  uint8_t telemetry = 0;
  uint8_t telemetry_auto = 0;
  int16_t annex650_overrun_count = 0;
#endif
#ifdef LCD_TELEMETRY_STEP
  char telemetryStepSequence []  = LCD_TELEMETRY_STEP;
  uint8_t telemetryStepIndex = 0;
#endif

// ******************
// rc functions
// ******************
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

int16_t failsafeEvents = 0;
volatile int16_t failsafeCnt = 0;

int16_t rcData[RC_CHANS];    // interval [1000;2000]
int16_t rcSerial[8];         // interval [1000;2000] - is rcData coming from MSP
int16_t rcCommand[4];        // interval [1000;2000] for THROTTLE and [-500;+500] for ROLL/PITCH/YAW
uint8_t rcSerialCount = 0;   // a counter to select legacy RX when there is no more MSP rc serial data
int16_t lookupPitchRollRC[5];// lookup table for expo & RC rate PITCH+ROLL
uint16_t lookupThrottleRC[11];// lookup table for expo & mid THROTTLE

#if defined(SERIAL_RX)
  volatile uint8_t  spekFrameFlags;
  volatile uint32_t spekTimeLast;
  uint8_t  spekFrameDone;
#endif

#if defined(OPENLRSv2MULTI)
  uint8_t pot_P,pot_I; // OpenLRS onboard potentiometers for P and I trim or other usages
#endif


// *************************
// motor and servo functions
// *************************
int16_t axisPID[3];
int16_t motor[8];
int16_t servo[8] = {1500,1500,1500,1500,1500,1500,1500,1000};

// ************************
// EEPROM Layout definition
// ************************
static uint8_t dynP8[2], dynD8[2];

global_conf_t global_conf;

conf_t conf;

#ifdef LOG_PERMANENT
  plog_t plog;
#endif

// **********************
// GPS common variables, no need to put them in defines, since compiller will optimize out unused variables
// **********************
#if GPS
  gps_conf_struct GPS_conf;
#endif
  int16_t  GPS_angle[2] = { 0, 0};                      // the angles that must be applied for GPS correction
  int32_t  GPS_coord[2];
  int32_t  GPS_home[2];
  int32_t  GPS_hold[2];
  int32_t  GPS_prev[2];                                 //previous pos
  int32_t  GPS_poi[2];
  uint8_t  GPS_numSat;
  uint16_t GPS_distanceToHome;                          // distance to home  - unit: meter
  int16_t  GPS_directionToHome;                         // direction to home - unit: degree
  int32_t  GPS_directionToPoi;
  uint16_t GPS_altitude;                                // GPS altitude      - unit: meter
  uint16_t GPS_speed;                                   // GPS speed         - unit: cm/s
  uint8_t  GPS_update = 0;                              // a binary toogle to distinct a GPS position update
  uint16_t GPS_ground_course = 0;                       //                   - unit: degree*10

  //uint8_t GPS_mode  = GPS_MODE_NONE; // contains the current selected gps flight mode --> moved to the f. structure
  uint8_t NAV_state = 0; // NAV_STATE_NONE;  /// State of the nav engine
  uint8_t NAV_error = 0; // NAV_ERROR_NONE;
  uint8_t prv_gps_modes = 0;              /// GPS_checkbox items packed into 1 byte for checking GPS mode changes
  uint32_t nav_timer_stop = 0;            /// common timer used in navigation (contains the desired stop time in millis()
  uint16_t nav_hold_time;                 /// time in seconds to hold position
  uint8_t NAV_paused_at = 0;              // This contains the mission step where poshold paused the runing mission.

  uint8_t next_step = 1;                  /// The mission step which is upcoming it equals with the mission_step stored in EEPROM
  int16_t jump_times = -10;
#if GPS
  mission_step_struct mission_step;
#endif

  // The desired bank towards North (Positive) or South (Negative) : latitude
  // The desired bank towards East (Positive) or West (Negative)   : longitude
  int16_t  nav[2];
  int16_t  nav_rated[2];    //Adding a rate controller to the navigation to make it smoother

  // The orginal altitude used as base our new altitude during nav
  int32_t original_altitude;
  //This is the target what we want to reach 
  int32_t target_altitude;
  //This is the interim value which is feeded into the althold controller
  int32_t alt_to_hold;

  uint32_t alt_change_timer;
  int8_t alt_change_flag;
  uint32_t alt_change;

uint8_t alarmArray[ALRM_FAC_SIZE];           // array

#if BARO
  int32_t baroPressure;
  int16_t baroTemperature;
  int32_t baroPressureSum;
#endif

void annexCode() { // this code is excetuted at each loop and won't interfere with control loop if it lasts less than 650 microseconds
  static uint32_t calibratedAccTime;
  uint16_t tmp,tmp2;
  uint8_t axis,prop1,prop2;

  // PITCH & ROLL only dynamic PID adjustemnt,  depending on throttle value (or collective.pitch value for heli)
  #ifdef HELICOPTER
    #define DYN_THR_PID_CHANNEL COLLECTIVE_PITCH
  #else
    #define DYN_THR_PID_CHANNEL THROTTLE
  #endif
  prop2 = 128; // prop2 was 100, is 128 now
  if (rcData[DYN_THR_PID_CHANNEL]>1500) { // breakpoint is fix: 1500
    if (rcData[DYN_THR_PID_CHANNEL]<2000) {
      prop2 -=  ((uint16_t)conf.dynThrPID*(rcData[DYN_THR_PID_CHANNEL]-1500)>>9); //  /512 instead of /500
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
      dynP8[axis] = (uint16_t)conf.pid[axis].P8*prop1>>7; // was /100, is /128 now
      dynD8[axis] = (uint16_t)conf.pid[axis].D8*prop1>>7; // was /100, is /128 now
    } else {      // YAW
      rcCommand[axis] = tmp;
    }
    if (rcData[axis]<MIDRC) rcCommand[axis] = -rcCommand[axis];
  }
  tmp = constrain(rcData[THROTTLE],MINCHECK,2000);
  tmp = (uint32_t)(tmp-MINCHECK)*2559/(2000-MINCHECK); // [MINCHECK;2000] -> [0;2559]
  tmp2 = tmp/256; // range [0;9]
  rcCommand[THROTTLE] = lookupThrottleRC[tmp2] + (tmp-tmp2*256) * (lookupThrottleRC[tmp2+1]-lookupThrottleRC[tmp2]) / 256; // [0;2559] -> expo -> [conf.minthrottle;MAXTHROTTLE]
  #if defined(HEADFREE)
    if(f.HEADFREE_MODE) { //to optimize
      float radDiff = (att.heading - headFreeModeHold) * 0.0174533f; // where PI/180 ~= 0.0174533
      float cosDiff = cos(radDiff);
      float sinDiff = sin(radDiff);
      int16_t rcCommand_PITCH = rcCommand[PITCH]*cosDiff + rcCommand[ROLL]*sinDiff;
      rcCommand[ROLL] =  rcCommand[ROLL]*cosDiff - rcCommand[PITCH]*sinDiff; 
      rcCommand[PITCH] = rcCommand_PITCH;
    }
  #endif

  // query at most one multiplexed analog channel per MWii cycle
  static uint8_t analogReader =0;
  switch (analogReader++ % (3+VBAT_CELLS_NUM)) {
  case 0:
  {
    #if defined(POWERMETER_HARD)
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
      analog.amperage = ((uint32_t)powerValue * conf.pint2ma) / 100; // [100mA]    //old (will overflow for 65A: powerValue * conf.pint2ma; // [1mA]
      pMeter[PMOTOR_SUM] += ((currentTime-lastRead) * (uint32_t)((uint32_t)powerValue*conf.pint2ma))/100000; // [10 mA * msec]
      lastRead = currentTime;
    #endif // POWERMETER_HARD
    break;
  }

  case 1:
  {
    #if defined(VBAT) && !defined(VBAT_CELLS)
      static uint8_t ind = 0;
      static uint16_t vvec[VBAT_SMOOTH], vsum;
      uint16_t v = analogRead(V_BATPIN);
      #if VBAT_SMOOTH == 1
        analog.vbat = (v<<4) / conf.vbatscale + VBAT_OFFSET; // result is Vbatt in 0.1V steps
      #else
        vsum += v;
        vsum -= vvec[ind];
        vvec[ind++] = v;
        ind %= VBAT_SMOOTH;
        #if VBAT_SMOOTH == 16
          analog.vbat = vsum / conf.vbatscale + VBAT_OFFSET; // result is Vbatt in 0.1V steps
        #elif VBAT_SMOOTH < 16
          analog.vbat = (vsum * (16/VBAT_SMOOTH)) / conf.vbatscale + VBAT_OFFSET; // result is Vbatt in 0.1V steps
        #else
          analog.vbat = ((vsum /VBAT_SMOOTH) * 16) / conf.vbatscale + VBAT_OFFSET; // result is Vbatt in 0.1V steps
        #endif
      #endif
    #endif // VBAT
    break;
  }
  case 2:
  {
  #if defined(RX_RSSI)
    static uint8_t ind = 0;
    static uint16_t rvec[RSSI_SMOOTH], rsum, r;

    // http://www.multiwii.com/forum/viewtopic.php?f=8&t=5530
    #if defined(RX_RSSI_CHAN)
      uint16_t rssi_Input = constrain(rcData[RX_RSSI_CHAN],1000,2000);
      r = map((uint16_t)rssi_Input , 1000, 2000, 0, 1023);
    #else
      r = analogRead(RX_RSSI_PIN);
    #endif 

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
   #endif // RX RSSI
   break;
  }
  default: // here analogReader >=4, because of ++ in switch()
  {
    #if defined(VBAT) && defined(VBAT_CELLS)
      if ( (analogReader<4) || (analogReader>4+VBAT_CELLS_NUM-1) ) break;
      uint8_t ind = analogReader-4;
      static uint16_t vbatcells_pins[VBAT_CELLS_NUM] = VBAT_CELLS_PINS;
      static uint8_t  vbatcells_offset[VBAT_CELLS_NUM] = VBAT_CELLS_OFFSETS;
      static uint8_t  vbatcells_div[VBAT_CELLS_NUM] = VBAT_CELLS_DIVS;
      uint16_t v = analogRead(vbatcells_pins[ind]);
      analog.vbatcells[ind] = vbatcells_offset[ind] + (v << 2) / vbatcells_div[ind]; // result is Vbatt in 0.1V steps
      if (ind == VBAT_CELLS_NUM -1) analog.vbat = analog.vbatcells[ind];
    #endif // VBAT) && defined(VBAT_CELLS)
    break;
  } // end default
  } // end of switch()

#if defined( POWERMETER_HARD ) && (defined(LOG_VALUES) || defined(LCD_TELEMETRY))
  if (analog.amperage > powerValueMaxMAH) powerValueMaxMAH = analog.amperage;
#endif

#if defined(WATTS)
  analog.watts = (analog.amperage * analog.vbat) / 100; // [0.1A] * [0.1V] / 100 = [Watt]
  #if defined(LOG_VALUES) || defined(LCD_TELEMETRY)
    if (analog.watts > wattsMax) wattsMax = analog.watts;
  #endif
#endif

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

  #if !(defined(SERIAL_RX) && defined(PROMINI))  //Only one serial port on ProMini.  Skip serial com if SERIAL RX in use. Note: Spek code will auto-call serialCom if GUI data detected on serial0.
    serialCom();
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
        if ( (alt.EstAlt > BAROaltMax) ) BAROaltMax = alt.EstAlt;
      #endif
      #if GPS
        if ( (GPS_speed > GPS_speedMax) ) GPS_speedMax = GPS_speed;
      #endif
    #endif
  }
}

void setup() {
  SerialOpen(0,SERIAL0_COM_SPEED);
  #if defined(PROMICRO)
    SerialOpen(1,SERIAL1_COM_SPEED);
  #endif
  #if defined(MEGA)
    SerialOpen(1,SERIAL1_COM_SPEED);
    SerialOpen(2,SERIAL2_COM_SPEED);
    SerialOpen(3,SERIAL3_COM_SPEED);
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

  #if GPS
    recallGPSconf();                              //Load GPS configuration parameteres
  #endif

  configureReceiver();
  #if defined (PILOTLAMP) 
    PL_INIT;
  #endif
  #if defined(OPENLRSv2MULTI)
    initOpenLRS();
  #endif
  initSensors();
  #if GPS
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
  #if GPS
    #if defined(GPS_SERIAL)
      GPS_SerialInit();
    #endif
    GPS_conf.max_wp_number = getMaxWPNumber(); 
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
  #ifdef DEBUGMSG
    debugmsg_append_str("initialization completed\n");
  #endif
}

void go_arm() {
  if(calibratingG == 0
  #if defined(ONLYARMWHENFLAT)
    && f.ACC_CALIBRATED 
  #endif
  #if defined(FAILSAFE)
    && failsafeCnt < 2
  #endif
  #if GPS && defined(ONLY_ALLOW_ARM_WITH_GPS_3DFIX)
    && (f.GPS_FIX && GPS_numSat >= 5)
  #endif
    ) {
    if(!f.ARMED && !f.BARO_MODE) { // arm now!
      f.ARMED = 1;
      #if defined(HEADFREE)
        headFreeModeHold = att.heading;
      #endif
      magHold = att.heading;
      #if defined(VBAT)
        if (analog.vbat > NO_VBAT) vbatMin = analog.vbat;
      #endif
      #ifdef ALTITUDE_RESET_ON_ARM
        #if BARO
          calibratingB = 10; // calibrate baro to new ground level (10 * 25 ms = ~250 ms non blocking)
        #endif
      #endif
      #ifdef LCD_TELEMETRY // reset some values when arming
        #if BARO
          BAROaltMax = alt.EstAlt;
        #endif
        #if GPS
          GPS_speedMax = 0;
        #endif
        #if defined( POWERMETER_HARD ) && (defined(LOG_VALUES) || defined(LCD_TELEMETRY))
          powerValueMaxMAH = 0;
        #endif
        #ifdef WATTS
          wattsMax = 0;
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
    SET_ALARM(ALRM_FAC_ACC, ALRM_LVL_ON);
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
  int16_t delta;
  int16_t PTerm = 0,ITerm = 0,DTerm, PTermACC, ITermACC;
  static int16_t lastGyro[2] = {0,0};
  static int16_t errorAngleI[2] = {0,0};
  #if PID_CONTROLLER == 1
  static int32_t errorGyroI_YAW;
  static int16_t delta1[2],delta2[2];
  static int16_t errorGyroI[2] = {0,0};
  #elif PID_CONTROLLER == 2
  static int16_t delta1[3],delta2[3];
  static int32_t errorGyroI[3] = {0,0,0};
  static int16_t lastError[3] = {0,0,0};
  int16_t deltaSum;
  int16_t AngleRateTmp, RateError;
  #endif
  static uint16_t rcTime  = 0;
  static int16_t initialThrottleHold;
  int16_t rc;
  int32_t prop = 0;

  #if defined(SERIAL_RX)
    if (spekFrameFlags == 0x01) readSerial_RX();
  #endif
  #if defined(OPENLRSv2MULTI) 
    Read_OpenLRS_RC();
  #endif 

  #if defined(SERIAL_RX)
  if ((spekFrameDone == 0x01) || ((int16_t)(currentTime-rcTime) >0 )) { 
    spekFrameDone = 0x00;
  #else
  if ((int16_t)(currentTime-rcTime) >0 ) { // 50Hz
  #endif
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
      #if !defined(FIXEDWING)
        errorGyroI[ROLL] = 0; errorGyroI[PITCH] = 0;
        #if PID_CONTROLLER == 1
          errorGyroI_YAW = 0;
        #elif PID_CONTROLLER == 2
          errorGyroI[YAW] = 0;
        #endif
        errorAngleI[ROLL] = 0; errorAngleI[PITCH] = 0;
      #endif
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
                if (AccInflightCalibrationArmed) SET_ALARM_BUZZER(ALRM_FAC_TOGGLE, ALRM_LVL_TOGGLE_2);
                else     SET_ALARM_BUZZER(ALRM_FAC_TOGGLE, ALRM_LVL_TOGGLE_ELSE);
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
            SET_ALARM(ALRM_FAC_TOGGLE, i);
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
            #if defined( OLED_I2C_128x64)
              if (telemetry != 0) i2c_OLED_init();
            #elif defined(OLED_DIGOLE)
              if (telemetry != 0) i2c_OLED_DIGOLE_init();
            #endif
            LCDclear();
          }
        #endif
        #if ACC
          else if (rcSticks == THR_HI + YAW_LO + PIT_LO + ROL_CE) calibratingA=512;     // throttle=max, yaw=left, pitch=min
        #endif
        #if MAG
          else if (rcSticks == THR_HI + YAW_HI + PIT_LO + ROL_CE) f.CALIBRATE_MAG = 1;  // throttle=max, yaw=right, pitch=min
        #endif
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

    #if defined(EXTENDED_AUX_STATES)
    uint32_t auxState = 0;
    for(i=0;i<4;i++)
      auxState |= (rcData[AUX1+i]<1230)<<(6*i) | 
      (1231<rcData[AUX1+i] && rcData[AUX1+i]<1360)<<(6*i+1) |
      (1361<rcData[AUX1+i] && rcData[AUX1+i]<1490)<<(6*i+2) |
      (1491<rcData[AUX1+i] && rcData[AUX1+i]<1620)<<(6*i+3) |
      (1621<rcData[AUX1+i] && rcData[AUX1+i]<1749)<<(6*i+4) |
      (rcData[AUX1+i]>1750)<<(6*i+5);
    #else
    uint16_t auxState = 0;
    for(i=0;i<4;i++)
      auxState |= (rcData[AUX1+i]<1300)<<(3*i) | (1300<rcData[AUX1+i] && rcData[AUX1+i]<1700)<<(3*i+1) | (rcData[AUX1+i]>1700)<<(3*i+2);
    #endif

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
        #if GPS 
        if (GPS_conf.takeover_baro) rcOptions[BOXBARO] = (rcOptions[BOXBARO] || f.GPS_BARO_MODE);
        #endif
        if (rcOptions[BOXBARO]) {
          if (!f.BARO_MODE) {
            f.BARO_MODE = 1;
            AltHold = alt.EstAlt;
            #if defined(ALT_HOLD_THROTTLE_MIDPOINT)
              initialThrottleHold = ALT_HOLD_THROTTLE_MIDPOINT;
            #else
              initialThrottleHold = rcCommand[THROTTLE];
            #endif
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
    if (rcOptions[BOXMAG]) {
      if (!f.MAG_MODE) {
        f.MAG_MODE = 1;
        magHold = att.heading;
      }
    } else {
      f.MAG_MODE = 0;
    }
    #if defined(HEADFREE)
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
    // This handles the three rcOptions boxes 
    // unlike other parts of the multiwii code, it looks for changes and not based on flag settings
    // by this method a priority can be established between gps option

    //Generate a packed byte of all four GPS boxes.
    uint8_t gps_modes_check = (rcOptions[BOXLAND]<< 3) + (rcOptions[BOXGPSHOME]<< 2) + (rcOptions[BOXGPSHOLD]<<1) + (rcOptions[BOXGPSNAV]);

    if (f.ARMED ) {                       //Check GPS status and armed
      //TODO: implement f.GPS_Trusted flag, idea from Dramida - Check for degraded HDOP and sudden speed jumps
      if (f.GPS_FIX) {
        if (GPS_numSat >5 ) {
          if (prv_gps_modes != gps_modes_check) {                           //Check for change since last loop
            NAV_error = NAV_ERROR_NONE;
            if (rcOptions[BOXGPSHOME]) {                                    // RTH has the priotity over everything else
              init_RTH();
            } else if (rcOptions[BOXGPSHOLD]) {                             //Position hold has priority over mission execution  //But has less priority than RTH
              if (f.GPS_mode == GPS_MODE_NAV)
                NAV_paused_at = mission_step.number;
              f.GPS_mode = GPS_MODE_HOLD;
              GPS_set_next_wp(&GPS_coord[LAT], &GPS_coord[LON],&GPS_coord[LAT], & GPS_coord[LON]); //hold at the current position
              set_new_altitude(alt.EstAlt);                                //and current altitude
              NAV_state = NAV_STATE_HOLD_INFINIT;
            } else if (rcOptions[BOXLAND]) {                               //Land now (It has priority over Navigation)
              f.GPS_mode = GPS_MODE_HOLD;
              f.GPS_BARO_MODE = true;
              GPS_set_next_wp(&GPS_coord[LAT], &GPS_coord[LON],&GPS_coord[LAT], & GPS_coord[LON]);
              set_new_altitude(alt.EstAlt);
              NAV_state = NAV_STATE_LAND_START;
            } else if (rcOptions[BOXGPSNAV]) {                             //Start navigation
              f.GPS_mode = GPS_MODE_NAV;                                   //Nav mode start
              f.GPS_BARO_MODE = true;
              GPS_prev[LAT] = GPS_coord[LAT];
              GPS_prev[LON] = GPS_coord[LON];
              if (NAV_paused_at != 0) {
                next_step = NAV_paused_at;
                NAV_paused_at = 0;                                         //Clear paused step 
              } else {
                next_step = 1;
                jump_times = -10;                                          //Reset jump counter
              }
              NAV_state = NAV_STATE_PROCESS_NEXT;
            } else {                                                       //None of the GPS Boxes are switched on
              f.GPS_mode = GPS_MODE_NONE;
              f.GPS_BARO_MODE = false;
              f.THROTTLE_IGNORED = false;
              f.LAND_IN_PROGRESS = 0;
              f.THROTTLE_IGNORED = 0;
              NAV_state = NAV_STATE_NONE;
              GPS_reset_nav();
            }
            prv_gps_modes = gps_modes_check;
          }
        } else { //numSat>5 
          //numSat dropped below 5 during navigation
          if (f.GPS_mode == GPS_MODE_NAV) {
            NAV_paused_at = mission_step.number;
            f.GPS_mode = GPS_MODE_NONE;
            set_new_altitude(alt.EstAlt);                                  //and current altitude
            NAV_state = NAV_STATE_NONE;
            NAV_error = NAV_ERROR_SPOILED_GPS;
            prv_gps_modes = 0xff;                                          //invalidates mode check, to allow re evaluate rcOptions when numsats raised again
          }
          if (f.GPS_mode == GPS_MODE_HOLD || f.GPS_mode == GPS_MODE_RTH) {
            f.GPS_mode = GPS_MODE_NONE;
            NAV_state = NAV_STATE_NONE;
            NAV_error = NAV_ERROR_SPOILED_GPS;
            prv_gps_modes = 0xff;                                          //invalidates mode check, to allow re evaluate rcOptions when numsats raised again
          }
          nav[0] = 0; nav[1] = 0;
        }
      } else { //f.GPS_FIX
        // GPS Fix dissapeared, very unlikely that we will be able to regain it, abort mission
        f.GPS_mode = GPS_MODE_NONE;
        NAV_state = NAV_STATE_NONE;
        NAV_paused_at = 0;
        NAV_error = NAV_ERROR_GPS_FIX_LOST;
        GPS_reset_nav();
        prv_gps_modes = 0xff;                                              //Gives a chance to restart mission when regain fix
      }
    } else { //copter is armed
      //copter is disarmed
      f.GPS_mode = GPS_MODE_NONE;
      f.GPS_BARO_MODE = false;
      f.THROTTLE_IGNORED = false;
      NAV_state = NAV_STATE_NONE;
      NAV_paused_at = 0;
      NAV_error = NAV_ERROR_DISARMED;
      GPS_reset_nav();
    }

    #endif //GPS

    #if defined(FIXEDWING) || defined(HELICOPTER)
      if (rcOptions[BOXPASSTHRU]) {f.PASSTHRU_MODE = 1;}
      else {f.PASSTHRU_MODE = 0;}
    #endif
 
  } else { // not in rc loop
    static uint8_t taskOrder=0; // never call all functions in the same loop, to avoid high delay spikes
    switch (taskOrder) {
      case 0:
        taskOrder++;
        #if MAG
          if (Mag_getADC() != 0) break; // 320 µs
        #endif
      case 1:
        taskOrder++;
        #if BARO
          if (Baro_update() != 0) break; // for MS baro: I2C set and get: 220 us  -  presure and temperature computation 160 us
        #endif
      case 2:
        taskOrder++;
        #if BARO
          if (getEstimatedAltitude() != 0) break; // 280 us
        #endif    
      case 3:
        taskOrder++;
        #if GPS
          if (GPS_Compute() != 0) break;  // performs computation on new frame only if present
          #if defined(I2C_GPS)
          if (GPS_NewData() != 0) break;  // 160 us with no new data / much more with new data 
          #endif
        #endif
      case 4:
        taskOrder=0;
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
 
  while(1) {
    currentTime = micros();
    cycleTime = currentTime - previousTime;
    #if defined(LOOP_TIME)
      if (cycleTime >= LOOP_TIME) break;
    #else
      break;  
    #endif
  }
  previousTime = currentTime;

  computeIMU();

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
      GPS_mode = GPS_MODE_NONE;
      }
    }
  #endif

  //*********************************** 
  // THROTTLE sticks during mission and RTH
  #if GPS
  if (GPS_conf.ignore_throttle == 1) {
    if (f.GPS_mode == GPS_MODE_NAV || f.GPS_mode == GPS_MODE_RTH) {
      //rcCommand[ROLL] = 0;
      //rcCommand[PITCH] = 0;
      //rcCommand[YAW] = 0;
      f.THROTTLE_IGNORED = 1;
    } else 
      f.THROTTLE_IGNORED = 0;
  }

  //Heading manipulation TODO: Do heading manipulation 
  #endif

  if (abs(rcCommand[YAW]) <70 && f.MAG_MODE) {
    int16_t dif = att.heading - magHold;
    if (dif <= - 180) dif += 360;
    if (dif >= + 180) dif -= 360;
    if (f.SMALL_ANGLES_25 || (f.GPS_mode != 0)) rcCommand[YAW] -= dif*conf.pid[PIDMAG].P8 >> 5;  //Always correct maghold in GPS mode
  } else magHold = att.heading;

  #if BARO && (!defined(SUPPRESS_BARO_ALTHOLD))
  /* Smooth alt change routine , for slow auto and aerophoto modes (in general solution from alexmos). It's slowly increase/decrease 
  * altitude proportional to stick movement (+/-100 throttle gives about +/-50 cm in 1 second with cycle time about 3-4ms)
  */
  if (f.BARO_MODE) {
    static uint8_t isAltHoldChanged = 0;
    static int16_t AltHoldCorr = 0;

    #if GPS
    if (f.LAND_IN_PROGRESS) { //If autoland is in progress then take over and decrease alt slowly
      AltHoldCorr -= GPS_conf.land_speed;
      if(abs(AltHoldCorr) > 512) {
        AltHold += AltHoldCorr/512;
        AltHoldCorr %= 512;
      }
    }
    #endif
    //IF Throttle not ignored then allow change altitude with the stick....
    if ( (abs(rcCommand[THROTTLE]-initialThrottleHold)>ALT_HOLD_THROTTLE_NEUTRAL_ZONE) && !f.THROTTLE_IGNORED) {
      // Slowly increase/decrease AltHold proportional to stick movement ( +100 throttle gives ~ +50 cm in 1 second with cycle time about 3-4ms)
      AltHoldCorr+= rcCommand[THROTTLE] - initialThrottleHold;
      if(abs(AltHoldCorr) > 512) {
        AltHold += AltHoldCorr/512;
        AltHoldCorr %= 512;
      }
      isAltHoldChanged = 1;
    } else if (isAltHoldChanged) {
      AltHold = alt.EstAlt;
      isAltHoldChanged = 0;
    }
    rcCommand[THROTTLE] = initialThrottleHold + BaroPID;
  }
  #endif //BARO



  #if defined(THROTTLE_ANGLE_CORRECTION)
  if(f.ANGLE_MODE || f.HORIZON_MODE) {
    rcCommand[THROTTLE]+= throttleAngleCorrection;
  }
  #endif

  #if GPS
  //TODO: split cos_yaw calculations into two phases (X and Y)
  if (( f.GPS_mode != GPS_MODE_NONE ) && f.GPS_FIX_HOME ) {
    float sin_yaw_y = sin(att.heading*0.0174532925f);
    float cos_yaw_x = cos(att.heading*0.0174532925f);
    GPS_angle[ROLL]   = (nav[LON]*cos_yaw_x - nav[LAT]*sin_yaw_y) /10;
    GPS_angle[PITCH]  = (nav[LON]*sin_yaw_y + nav[LAT]*cos_yaw_x) /10;
    } else {
      GPS_angle[ROLL]  = 0;
      GPS_angle[PITCH] = 0;
    }

  //Used to communicate back nav angles to the GPS simulator (for HIL testing)
  #if defined(GPS_SIMULATOR)
    SerialWrite(2,0xa5);
    SerialWrite16(2,nav[LAT]+rcCommand[PITCH]);
    SerialWrite16(2,nav[LON]+rcCommand[ROLL]);
    SerialWrite16(2,(nav[LAT]+rcCommand[PITCH])-(nav[LON]+rcCommand[ROLL])); //check
  #endif

  #endif //GPS

  //**** PITCH & ROLL & YAW PID ****
  #if PID_CONTROLLER == 1 // evolved oldschool
  if ( f.HORIZON_MODE ) prop = min(max(abs(rcCommand[PITCH]),abs(rcCommand[ROLL])),512);

  // PITCH & ROLL
  for(axis=0;axis<2;axis++) {
    rc = rcCommand[axis]<<1;
    error = rc - imu.gyroData[axis];
    errorGyroI[axis]  = constrain(errorGyroI[axis]+error,-16000,+16000);       // WindUp   16 bits is ok here
    if (abs(imu.gyroData[axis])>640) errorGyroI[axis] = 0;

    ITerm = (errorGyroI[axis]>>7)*conf.pid[axis].I8>>6;                        // 16 bits is ok here 16000/125 = 128 ; 128*250 = 32000

    PTerm = mul(rc,conf.pid[axis].P8)>>6;
    
    if (f.ANGLE_MODE || f.HORIZON_MODE) { // axis relying on ACC
      // 50 degrees max inclination
      errorAngle         = constrain(rc + GPS_angle[axis],-500,+500) - att.angle[axis] + conf.angleTrim[axis]; //16 bits is ok here
      errorAngleI[axis]  = constrain(errorAngleI[axis]+errorAngle,-10000,+10000);                                                // WindUp     //16 bits is ok here

      PTermACC           = mul(errorAngle,conf.pid[PIDLEVEL].P8)>>7; // 32 bits is needed for calculation: errorAngle*P8 could exceed 32768   16 bits is ok for result

      int16_t limit      = conf.pid[PIDLEVEL].D8*5;
      PTermACC           = constrain(PTermACC,-limit,+limit);

      ITermACC           = mul(errorAngleI[axis],conf.pid[PIDLEVEL].I8)>>12;   // 32 bits is needed for calculation:10000*I8 could exceed 32768   16 bits is ok for result

      ITerm              = ITermACC + ((ITerm-ITermACC)*prop>>9);
      PTerm              = PTermACC + ((PTerm-PTermACC)*prop>>9);
    }

    PTerm -= mul(imu.gyroData[axis],dynP8[axis])>>6; // 32 bits is needed for calculation   

    delta          = imu.gyroData[axis] - lastGyro[axis];  // 16 bits is ok here, the dif between 2 consecutive gyro reads is limited to 800
    lastGyro[axis] = imu.gyroData[axis];
    DTerm          = delta1[axis]+delta2[axis]+delta;
    delta2[axis]   = delta1[axis];
    delta1[axis]   = delta;
 
    DTerm = mul(DTerm,dynD8[axis])>>5;        // 32 bits is needed for calculation

    axisPID[axis] =  PTerm + ITerm - DTerm;
  }

  //YAW
  #define GYRO_P_MAX 300
  #define GYRO_I_MAX 250

  rc = mul(rcCommand[YAW] , (2*conf.yawRate + 30))  >> 5;

  error = rc - imu.gyroData[YAW];
  errorGyroI_YAW  += mul(error,conf.pid[YAW].I8);
  errorGyroI_YAW  = constrain(errorGyroI_YAW, 2-((int32_t)1<<28), -2+((int32_t)1<<28));
  if (abs(rc) > 50) errorGyroI_YAW = 0;
  
  PTerm = mul(error,conf.pid[YAW].P8)>>6;
  #ifndef COPTER_WITH_SERVO
    int16_t limit = GYRO_P_MAX-conf.pid[YAW].D8;
    PTerm = constrain(PTerm,-limit,+limit);
  #endif
  
  ITerm = constrain((int16_t)(errorGyroI_YAW>>13),-GYRO_I_MAX,+GYRO_I_MAX);
  
  axisPID[YAW] =  PTerm + ITerm;
  
  #elif PID_CONTROLLER == 2 // alexK
  #define GYRO_I_MAX 256
  #define ACC_I_MAX 256
  prop = min(max(abs(rcCommand[PITCH]),abs(rcCommand[ROLL])),500); // range [0;500]

  //----------PID controller----------
  for(axis=0;axis<3;axis++) {
    //-----Get the desired angle rate depending on flight mode
    if ((f.ANGLE_MODE || f.HORIZON_MODE) && axis<2 ) { // MODE relying on ACC
      // calculate error and limit the angle to 50 degrees max inclination
      errorAngle = constrain((rcCommand[axis]<<1) + GPS_angle[axis],-500,+500) - att.angle[axis] + conf.angleTrim[axis]; //16 bits is ok here
    }
    if (axis == 2) {//YAW is always gyro-controlled (MAG correction is applied to rcCommand)
      AngleRateTmp = (((int32_t) (conf.yawRate + 27) * rcCommand[2]) >> 5);
    } else {
      if (!f.ANGLE_MODE) {//control is GYRO based (ACRO and HORIZON - direct sticks control is applied to rate PID
        AngleRateTmp = ((int32_t) (conf.rollPitchRate + 27) * rcCommand[axis]) >> 4;
        if (f.HORIZON_MODE) {
          //mix up angle error to desired AngleRateTmp to add a little auto-level feel
          AngleRateTmp += ((int32_t) errorAngle * conf.pid[PIDLEVEL].I8)>>8;
        }
      } else {//it's the ANGLE mode - control is angle based, so control loop is needed
        AngleRateTmp = ((int32_t) errorAngle * conf.pid[PIDLEVEL].P8)>>4;
      }
    }

    //--------low-level gyro-based PID. ----------
    //Used in stand-alone mode for ACRO, controlled by higher level regulators in other modes
    //-----calculate scaled error.AngleRates
    //multiplication of rcCommand corresponds to changing the sticks scaling here
    RateError = AngleRateTmp  - imu.gyroData[axis];

    //-----calculate P component
    PTerm = ((int32_t) RateError * conf.pid[axis].P8)>>7;

    //-----calculate I component
    //there should be no division before accumulating the error to integrator, because the precision would be reduced.
    //Precision is critical, as I prevents from long-time drift. Thus, 32 bits integrator is used.
    //Time correction (to avoid different I scaling for different builds based on average cycle time)
    //is normalized to cycle time = 2048.
    errorGyroI[axis]  += (((int32_t) RateError * cycleTime)>>11) * conf.pid[axis].I8;
    //limit maximum integrator value to prevent WindUp - accumulating extreme values when system is saturated.
    //I coefficient (I8) moved before integration to make limiting independent from PID settings
    errorGyroI[axis]  = constrain(errorGyroI[axis], (int32_t) -GYRO_I_MAX<<13, (int32_t) +GYRO_I_MAX<<13);
    ITerm = errorGyroI[axis]>>13;

    //-----calculate D-term
    delta          = RateError - lastError[axis];  // 16 bits is ok here, the dif between 2 consecutive gyro reads is limited to 800
    lastError[axis] = RateError;

    //Correct difference by cycle time. Cycle time is jittery (can be different 2 times), so calculated difference
    // would be scaled by different dt each time. Division by dT fixes that.
    delta = ((int32_t) delta * ((uint16_t)0xFFFF / (cycleTime>>4)))>>6;
    //add moving average here to reduce noise
    deltaSum       = delta1[axis]+delta2[axis]+delta;
    delta2[axis]   = delta1[axis];
    delta1[axis]   = delta;

    //DTerm = (deltaSum*conf.pid[axis].D8)>>8;
    //Solve overflow in calculation above...
    DTerm = ((int32_t)deltaSum*conf.pid[axis].D8)>>8;
    //-----calculate total PID output
    axisPID[axis] =  PTerm + ITerm + DTerm;
  }
  #else
    #error "*** you must set PID_CONTROLLER to one existing implementation"
  #endif
  mixTable();
  // do not update servos during unarmed calibration of sensors which are sensitive to vibration
  #if defined(DISABLE_SERVOS_WHEN_UNARMED)
  if (f.ARMED) writeServos();
  #else
  if ( (f.ARMED) || ((!calibratingG) && (!calibratingA)) ) writeServos();
  #endif 
  writeMotors();
}
