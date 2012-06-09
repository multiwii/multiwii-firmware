/*
MultiWiiCopter by Alexandre Dubus
www.multiwii.com
March  2012     V2.0
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 any later version. see <http://www.gnu.org/licenses/>
*/

#include "config.h"
#include "def.h"
#include <avr/pgmspace.h>
#define  VERSION  201

/*********** RC alias *****************/
#define ROLL       0
#define PITCH      1
#define YAW        2
#define THROTTLE   3
#define AUX1       4
#define AUX2       5
#define AUX3       6
#define AUX4       7

#define PIDALT     3
#define PIDPOS     4
#define PIDPOSR    5
#define PIDNAVR    6
#define PIDLEVEL   7
#define PIDMAG     8
#define PIDVEL     9 // not used currently

#define BOXACC       0
#define BOXBARO      1
#define BOXMAG       2
#define BOXCAMSTAB   3
#define BOXCAMTRIG   4
#define BOXARM       5
#define BOXGPSHOME   6
#define BOXGPSHOLD   7
#define BOXPASSTHRU  8
#define BOXHEADFREE  9
#define BOXBEEPERON  10
/* we want maximum illumination */
#define BOXLEDMAX    11
/* enable landing lights at any altitude */
#define BOXLLIGHTS   12
/* acquire heading for HEADFREE mode */
#define BOXHEADADJ   13

#define PIDITEMS 10
#define CHECKBOXITEMS 14

/* names for dynamic generation of config GUI */
const char boxnames[] PROGMEM =
  "ACC;"
  "BARO;"
  "MAG;"
  "CAMSTAB;"
  "CAMTRIG;"
  "ARM;"
  "GPS HOME;"
  "GPS HOLD;"
  "PASSTHRU;"
  "HEADFREE;"
  "BEEPER;"
  "LEDMAX;"
  "LLIGHTS;"
  "HEADADJ;"
;

/* bitmask for all the flags specified in def.h */
uint8_t flag_mask[FLAG_CNT+7 / 8] = {0};

static uint32_t currentTime = 0;
static uint16_t previousTime = 0;
static uint16_t cycleTime = 0;     // this is the number in micro second to achieve a full loop, it can differ a little and is taken into account in the PID loop
static uint16_t calibratingA = 0;  // the calibration is done in the main loop. Calibrating decreases at each cycle down to 0, then we enter in a normal mode.
static uint8_t  calibratingM = 0;
static uint16_t calibratingG;
static uint16_t acc_1G;             // this is the 1G measured acceleration
static int16_t  acc_25deg;
static int16_t  headFreeModeHold;
static int16_t  gyroADC[3],accADC[3],accSmooth[3],magADC[3];
static int16_t  heading,magHold;
static uint8_t  vbat;               // battery voltage in 0.1V steps
static uint8_t  rcOptions[CHECKBOXITEMS];
static int32_t  BaroAlt;
static int32_t  EstAlt;             // in cm
static int16_t  BaroPID = 0;
static int32_t  AltHold;
static int16_t  errorAltitudeI = 0;
#if defined(BUZZER)
static uint8_t  toggleBeep = 0;
#endif
static int16_t  debug1,debug2,debug3,debug4;
static int16_t  sonarAlt; //to think about the unit

//for log
#if defined(LOG_VALUES)
static uint16_t cycleTimeMax = 0;       // highest ever cycle timen
static uint16_t cycleTimeMin = 65535;   // lowest ever cycle timen
static uint16_t powerMax = 0;           // highest ever current
static uint32_t armedTime = 0;
static int32_t  BAROaltStart = 0;       // offset value from powerup
static int32_t	BAROaltMax = 0;	        // maximum value
#endif

static int16_t  i2c_errors_count = 0;
static int16_t  annex650_overrun_count = 0;

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
#if defined(POWERMETER)
#define PMOTOR_SUM 8                     // index into pMeter[] for sum
static uint32_t pMeter[PMOTOR_SUM + 1];  // we use [0:7] for eight motors,one extra for sum
static uint8_t pMeterV;                  // dummy to satisfy the paramStruct logic in ConfigurationLoop()
static uint32_t pAlarm;                  // we scale the eeprom value from [0:255] to this value we can directly compare to the sum in pMeter[6]
static uint16_t powerValue = 0;          // last known current
#endif
static uint16_t intPowerMeterSum, intPowerTrigger1;

// **********************
// telemetry
// **********************
#if defined(LCD_TELEMETRY) || defined(LCD_TELEMETRY_AUTO) || defined(LCD_TELEMETRY_DEBUG)
static uint8_t telemetry = 0;
static uint8_t telemetry_auto = 0;
#endif

// ******************
// rc functions
// ******************
#define MINCHECK 1100
#define MAXCHECK 1900

#if defined(FAILSAFE)
static int16_t failsafeEvents = 0;
#endif
volatile int16_t failsafeCnt = 0;

static int16_t rcData[8];          // interval [1000;2000]
static int16_t rcCommand[4];       // interval [1000;2000] for THROTTLE and [-500;+500] for ROLL/PITCH/YAW 
static int16_t lookupPitchRollRC[6];// lookup table for expo & RC rate PITCH+ROLL
static int16_t lookupThrottleRC[11];// lookup table for expo & mid THROTTLE
volatile uint8_t rcFrameComplete; // for serial rc receiver Spektrum

#if defined(OPENLRSv2MULTI)
static uint8_t pot_P,pot_I; // OpenLRS onboard potentiometers for P and I trim or other usages
#endif

// **************
// gyro+acc IMU
// **************
static int16_t gyroData[3] = {0,0,0};
static int16_t gyroZero[3] = {0,0,0};
static int16_t angle[2]    = {0,0};  // absolute angle inclination in multiple of 0.1 degree    180 deg = 1800
static int8_t  smallAngle25 = 1;

// *************************
// motor and servo functions
// *************************
static int16_t axisPID[3];
static int16_t motor[NUMBER_MOTOR];
#if defined(SERVO)
static int16_t servo[8] = {1500,1500,1500,1500,1500,1500,1500,1500};
#endif

// ************************
// EEPROM Layout definition
// ************************
static uint8_t dynP8[3], dynD8[3];
static struct {
  uint8_t checkNewConf;
  uint8_t P8[PIDITEMS], I8[PIDITEMS], D8[PIDITEMS];
  uint8_t rcRate8;
  uint8_t rcExpo8;
  uint8_t rollPitchRate;
  uint8_t yawRate;
  uint8_t dynThrPID;
  uint8_t thrMid8;
  uint8_t thrExpo8;
  int16_t accZero[3];
  int16_t magZero[3];
  int16_t angleTrim[2];
  uint16_t activate[CHECKBOXITEMS];
  uint8_t powerTrigger1;
  #ifdef FLYING_WING
    uint16_t wing_left_mid;
    uint16_t wing_right_mid;
  #endif
  #ifdef TRI
    uint16_t tri_yaw_middle;
  #endif
  #if defined HELICOPTER || defined(AIRPLANE)
    int16_t servoTrim[8];
  #endif
  #if defined(GYRO_SMOOTHING)
    uint8_t Smoothing[3];
  #endif
} conf;


// **********************
// GPS common variables
// **********************
static int32_t  GPS_coord[2];
static int32_t  GPS_home[2];
static int32_t  GPS_hold[2];
static uint8_t  GPS_numSat;
static uint16_t GPS_distanceToHome;                          // distance to home in meters
static int16_t  GPS_directionToHome;                         // direction to home in degrees
static uint16_t GPS_altitude,GPS_speed;                      // altitude in 0.1m and speed in 0.1m/s
static uint8_t  GPS_update = 0;                              // it's a binary toogle to distinct a GPS position update
static int16_t  GPS_angle[2] = { 0, 0};                      // it's the angles that must be applied for GPS correction
#if defined(I2C_GPS)
static uint16_t GPS_ground_course = 0;                       // degrees*10
#endif
#if defined(GPS_SERIAL)
static uint8_t  GPS_Present = 0;                             // Checksum from Gps serial
#endif
static uint8_t  GPS_Enable  = 0;

#define LAT  0
#define LON  1
// The desired bank towards North (Positive) or South (Negative) : latitude
// The desired bank towards East (Positive) or West (Negative)   : longitude
static int16_t	nav[2];

//////////////////////////////////////////////////////////////////////////////
// POSHOLD control gains
//
#define POSHOLD_P              .11
#define POSHOLD_I              0.0
#define POSHOLD_IMAX           20		// degrees

#define POSHOLD_RATE_P         2.0			//
#define POSHOLD_RATE_I         0.08			// Wind control
#define POSHOLD_RATE_D         0.045			// try 2 or 3 for POSHOLD_RATE 1
#define POSHOLD_RATE_IMAX      20			// degrees
//////////////////////////////////////////////////////////////////////////////
// Navigation PID gains
//
#define NAV_P                  1.4		//
#define NAV_I                  0.20		// Wind control
#define NAV_D                  0.08		//
#define NAV_IMAX               20		// degrees

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Serial GPS only variables
//navigation mode
#define NAV_MODE_NONE          0
#define NAV_MODE_POSHOLD       1
#define NAV_MODE_WP            2
static int8_t  nav_mode = NAV_MODE_NONE;            //Navigation mode


void inline set_flag(enum mwc_flag f, uint8_t v) {
  uint8_t *b = &flag_mask[f/8];
  if (v) {
    *b |= 1<<(f%8);
  } else {
    *b &= ~(1<<(f%8));
  }
}

uint8_t inline get_flag(enum mwc_flag f) {
  uint8_t *b = &flag_mask[f/8];
  return (*b & 1<<(f%8)) != 0;
}

void blinkLED(uint8_t num, uint8_t wait,uint8_t repeat) {
  uint8_t i,r;
  for (r=0;r<repeat;r++) {
    for(i=0;i<num;i++) {
      #if defined(LED_FLASHER)
        switch_led_flasher(1);
      #endif
      #if defined(LANDING_LIGHTS_DDR)
        switch_landing_lights(1);
      #endif
      LEDPIN_TOGGLE; // switch LEDPIN state
      BUZZERPIN_ON;
      delay(wait);
      BUZZERPIN_OFF;
      #if defined(LED_FLASHER)
        switch_led_flasher(0);
      #endif
      #if defined(LANDING_LIGHTS_DDR)
        switch_landing_lights(0);
      #endif
    }
    delay(60);
  }
}

void annexCode() { // this code is excetuted at each loop and won't interfere with control loop if it lasts less than 650 microseconds
  static uint32_t calibratedAccTime;
  uint16_t tmp,tmp2;
  #if defined(BUZZER)
    static uint8_t  buzzerFreq;         // delay between buzzer ring
  #endif
  uint8_t axis,prop1,prop2;

  #define BREAKPOINT 1500
  // PITCH & ROLL only dynamic PID adjustemnt,  depending on throttle value
  if   (rcData[THROTTLE]<BREAKPOINT) {
    prop2 = 100;
  } else {
    if (rcData[THROTTLE]<2000) {
      prop2 = 100 - (uint16_t)conf.dynThrPID*(rcData[THROTTLE]-BREAKPOINT)/(2000-BREAKPOINT);
    } else {
      prop2 = 100 - conf.dynThrPID;
    }
  }

  for(axis=0;axis<3;axis++) {
    tmp = min(abs(rcData[axis]-MIDRC),500);
    #if defined(DEADBAND)
      if (tmp>DEADBAND) { tmp -= DEADBAND; }
      else { tmp=0; }
    #endif
    if(axis!=2) { //ROLL & PITCH
      tmp2 = tmp/100;
      rcCommand[axis] = lookupPitchRollRC[tmp2] + (tmp-tmp2*100) * (lookupPitchRollRC[tmp2+1]-lookupPitchRollRC[tmp2]) / 100;
      prop1 = 100-(uint16_t)conf.rollPitchRate*tmp/500;
      prop1 = (uint16_t)prop1*prop2/100;
    } else {      // YAW
      rcCommand[axis] = tmp;
      prop1 = 100-(uint16_t)conf.yawRate*tmp/500;
    }
    dynP8[axis] = (uint16_t)conf.P8[axis]*prop1/100;
    dynD8[axis] = (uint16_t)conf.D8[axis]*prop1/100;
    if (rcData[axis]<MIDRC) rcCommand[axis] = -rcCommand[axis];
  }
  tmp = constrain(rcData[THROTTLE],MINCHECK,2000);
  tmp = (uint32_t)(tmp-MINCHECK)*1000/(2000-MINCHECK); // [MINCHECK;2000] -> [0;1000]
  tmp2 = tmp/100;
  rcCommand[THROTTLE] = lookupThrottleRC[tmp2] + (tmp-tmp2*100) * (lookupThrottleRC[tmp2+1]-lookupThrottleRC[tmp2]) / 100; // [0;1000] -> expo -> [MINTHROTTLE;MAXTHROTTLE]

  if(get_flag(FLAG_HEADFREE_MODE)) { //to optimize
    float radDiff = (heading - headFreeModeHold) * 0.0174533f; // where PI/180 ~= 0.0174533
    float cosDiff = cos(radDiff);
    float sinDiff = sin(radDiff);
    int16_t rcCommand_PITCH = rcCommand[PITCH]*cosDiff + rcCommand[ROLL]*sinDiff;
    rcCommand[ROLL] =  rcCommand[ROLL]*cosDiff - rcCommand[PITCH]*sinDiff; 
    rcCommand[PITCH] = rcCommand_PITCH;
  }

  #if defined(POWERMETER_HARD)
    uint16_t pMeterRaw;               // used for current reading
    static uint16_t psensorTimer = 0;
    if (! (++psensorTimer % PSENSORFREQ)) {
      pMeterRaw =  analogRead(PSENSORPIN);
      powerValue = ( PSENSORNULL > pMeterRaw ? PSENSORNULL - pMeterRaw : pMeterRaw - PSENSORNULL); // do not use abs(), it would induce implicit cast to uint and overrun
      if ( powerValue < 333) {  // only accept reasonable values. 333 is empirical
      #ifdef LOG_VALUES
        if (powerValue > powerMax) powerMax = powerValue;
      #endif
      } else {
        powerValue = 333;
      }        
      pMeter[PMOTOR_SUM] += (uint32_t) powerValue;
    }
  #endif

  #if defined(VBAT)
    static uint8_t vbatTimer = 0;
    static uint8_t ind = 0;
    uint16_t vbatRaw = 0;
    static uint16_t vbatRawArray[8];
    if (! (++vbatTimer % VBATFREQ)) {
    	vbatRawArray[(ind++)%8] = analogRead(V_BATPIN);
    	for (uint8_t i=0;i<8;i++) vbatRaw += vbatRawArray[i];
    	vbat = vbatRaw / (VBATSCALE/2);                  // result is Vbatt in 0.1V steps
    }
    if ( ( (vbat>VBATLEVEL1_3S) 
    #if defined(POWERMETER)
                         && ( (pMeter[PMOTOR_SUM] < pAlarm) || (pAlarm == 0) )
    #endif
                       )  || (NO_VBAT>vbat)                              ) // ToLuSe
    {                                          // VBAT ok AND powermeter ok, buzzer off
      buzzerFreq = 0;
    #if defined(POWERMETER)
    } else if (pMeter[PMOTOR_SUM] > pAlarm) {                             // sound alarm for powermeter
      buzzerFreq = 4;
    #endif
    } else if (vbat>VBATLEVEL2_3S) buzzerFreq = 1;
    else if (vbat>VBATLEVEL3_3S)   buzzerFreq = 2;
    else                           buzzerFreq = 4;
  #endif
  #if defined(BUZZER)
    buzzer(buzzerFreq); // external buzzer routine that handles buzzer events globally now
  #endif
  
  if ( (calibratingA>0 && ACC ) || (calibratingG>0) ) { // Calibration phasis
    LEDPIN_TOGGLE;
  } else {
    if (get_flag(FLAG_ACC_CALIBRATED)) {LEDPIN_OFF;}
    if (get_flag(FLAG_ARMED)) {LEDPIN_ON;}
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
    if (smallAngle25 == 0) {
      // the multi uses ACC and is not calibrated or is too much inclinated
      set_flag(FLAG_ACC_CALIBRATED, 0);
      LEDPIN_TOGGLE;
      calibratedAccTime = currentTime + 500000;
    } else {
      set_flag(FLAG_ACC_CALIBRATED, 1);
    }
  }

    #if defined(GPS_PROMINI)
      if(GPS_Enable == 0){serialCom();}
    #else
      serialCom();
    #endif

  #if defined(POWERMETER)
    intPowerMeterSum = (pMeter[PMOTOR_SUM]/PLEVELDIV);
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
  
  #if GPS
    static uint32_t GPSLEDTime;
    if ( currentTime > GPSLEDTime && (GPS_numSat >= 5)) {
      GPSLEDTime = currentTime + 150000;
      LEDPIN_TOGGLE;
    }
  #endif

  #if defined(LOG_VALUES) && (LOG_VALUES == 2)
    if (cycleTime > cycleTimeMax) cycleTimeMax = cycleTime; // remember highscore
    if (cycleTime < cycleTimeMin) cycleTimeMin = cycleTime; // remember lowscore
  #endif
  #ifdef LOG_VALUES
    if (get_flag(FLAG_ARMED)) armedTime += (uint32_t)cycleTime;
    #if BARO
      if (!get_flag(FLAG_ARMED)) {
        BAROaltStart = BaroAlt;
        BAROaltMax = BaroAlt;
      } else {
        if (BaroAlt > BAROaltMax) BAROaltMax = BaroAlt;
      }
    #endif
  #endif
}

void setup() {
  #if !defined(GPS_PROMINI)
    SerialOpen(0,SERIAL_COM_SPEED);
  #endif
  LEDPIN_PINMODE;
  POWERPIN_PINMODE;
  BUZZERPIN_PINMODE;
  STABLEPIN_PINMODE;
  POWERPIN_OFF;
  #if defined(ESC_CALIB_CANNOT_FLY) // <- to move in Output.pde, nothing to do here
    /* this turns into a special version of MultiWii. Its only purpose it to try and calib all attached ESCs */
    writeAllMotors(ESC_CALIB_HIGH);
    delay(3000);
    writeAllMotors(ESC_CALIB_LOW);
    delay(500);
    while (1) {
      delay(5000);
      blinkLED(2,20, 2);
    }
    exit; // statement never reached
  #endif
  initOutput();
  readEEPROM();
  checkFirstTime();
  configureReceiver();
  #if defined(OPENLRSv2MULTI)
    initOpenLRS();
  #endif
  initSensors();
  #if defined(I2C_GPS) || defined(GPS_SERIAL)
    GPS_set_pids();
  #endif
  previousTime = micros();
  #if defined(GIMBAL)
   calibratingA = 400;
  #endif
  calibratingG = 400;
  #if defined(POWERMETER)
    for(uint8_t i=0;i<=PMOTOR_SUM;i++)
      pMeter[i]=0;
  #endif
  /************************************/
  #if defined(GPS_SERIAL)
    SerialOpen(GPS_SERIAL,GPS_BAUD);  
    delay(400);  
    for(uint8_t i=0;i<=5;i++){
      GPS_NewData(); 
      LEDPIN_ON
      delay(20);
      LEDPIN_OFF
      delay(80);
    }
    if(!GPS_Present){
      SerialEnd(GPS_SERIAL);
      SerialOpen(0,SERIAL_COM_SPEED);
    }      
    #if !defined(GPS_PROMINI)
      GPS_Present = 1;
    #endif
    GPS_Enable = GPS_Present;    
  #endif
  /************************************/
 
  #if defined(I2C_GPS) || defined(TINY_GPS)
   GPS_Enable = 1;
  #endif
  
  #if defined(LCD_ETPP) || defined(LCD_LCD03) || defined(OLED_I2C_128x64)
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
  ADCSRA |= _BV(ADPS2) ; ADCSRA &= ~_BV(ADPS1); ADCSRA &= ~_BV(ADPS0); // this speeds up analogRead without loosing too much resolution: http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1208715493/11
  #if defined(LED_FLASHER)
    init_led_flasher();
    led_flasher_set_sequence(LED_FLASHER_SEQUENCE);
  #endif
}

// ******** Main Loop *********
void loop () {
  static uint8_t rcDelayCommand; // this indicates the number of time (multiple of RC measurement at RC_FREQ Hz) the sticks must be maintained to run or switch off motors
  uint8_t axis,i;
  int16_t error,errorAngle;
  int16_t delta,deltaSum;
  int16_t PTerm,ITerm,DTerm;
  static int16_t lastGyro[3] = {0,0,0};
  static int16_t delta1[3],delta2[3];
  static int16_t errorGyroI[3] = {0,0,0};
  static int16_t errorAngleI[2] = {0,0};
  static uint32_t rcTime  = 0;
  static int16_t initialThrottleHold;

  #if defined(SPEKTRUM)
    if (rcFrameComplete) computeRC();
  #endif
  #if defined(OPENLRSv2MULTI) 
    Read_OpenLRS_RC();
  #endif 

  #define RC_FREQ 50

  if (currentTime > rcTime ) { // >50Hz
    rcTime = currentTime + (1000000L/RC_FREQ);
    computeRC();
    // Failsafe routine - added by MIS
    #if defined(FAILSAFE)
      if ( failsafeCnt > (5*FAILSAVE_DELAY) && set_flag(FLAG_ARMED, 1)) {                  // Stabilize, and set Throttle to specified level
        for(i=0; i<3; i++) rcData[i] = MIDRC;                               // after specified guard time after RC signal is lost (in 0.1sec)
        rcData[THROTTLE] = FAILSAVE_THROTTLE;
        if (failsafeCnt > 5*(FAILSAVE_DELAY+FAILSAVE_OFF_DELAY)) {          // Turn OFF motors after specified Time (in 0.1sec)
          set_flag(FLAG_ARMED, 0);   // This will prevent the copter to automatically rearm if failsafe shuts it down and prevents
          set_flag(FLAG_OK_TO_ARM, 0); // to restart accidentely by just reconnect to the tx - you will have to switch off first to rearm
        }
        failsafeEvents++;
      }
      failsafeCnt++;
    #endif
    // end of failsave routine - next change is made with RcOptions setting
    if (rcData[THROTTLE] < MINCHECK) {
      errorGyroI[ROLL] = 0; errorGyroI[PITCH] = 0; errorGyroI[YAW] = 0;
      errorAngleI[ROLL] = 0; errorAngleI[PITCH] = 0;
      rcDelayCommand++;
      if (rcData[YAW] < MINCHECK && rcData[PITCH] < MINCHECK && !get_flag(FLAG_ARMED)) {
        if (rcDelayCommand == (20*RC_FREQ/50)) {
          calibratingG=400;
          #if GPS 
            GPS_reset_home_position();
          #endif
        }
      } else if (rcData[YAW] > MAXCHECK && rcData[PITCH] > MAXCHECK && !get_flag(FLAG_ARMED)) {
        if (rcDelayCommand == (20*RC_FREQ/50)) {
          #ifdef TRI
            servo[5] = 1500; // we center the yaw servo in conf mode
            writeServos();
          #endif
          #ifdef FLYING_WING
            servo[0]  = conf.wing_left_mid;
            servo[1]  = conf.wing_right_mid;
            writeServos();
          #endif
          #ifdef AIRPLANE
            for(i = 4; i<7 ;i++) servo[i] = 1500;
            writeServos();
          #endif          
          #if defined(LCD_CONF)
            configurationLoop(); // beginning LCD configuration
          #endif
          previousTime = micros();
        }
      }
      #if defined(INFLIGHT_ACC_CALIBRATION)  
        else if (!get_flag(FLAG_ARMED) && rcData[YAW] < MINCHECK && rcData[PITCH] > MAXCHECK && rcData[ROLL] > MAXCHECK){
          if (rcDelayCommand == (20*RC_FREQ/50)){
            if (AccInflightCalibrationMeasurementDone){                // trigger saving into eeprom after landing
              AccInflightCalibrationMeasurementDone = 0;
              AccInflightCalibrationSavetoEEProm = 1;
            }else{ 
              AccInflightCalibrationArmed = !AccInflightCalibrationArmed; 
              #if defined(BUZZER)
              if (AccInflightCalibrationArmed){
                toggleBeep = 2;
              } else {
                toggleBeep = 3;
              }
              #endif
            }
          }
       } 
     #endif
      else if (conf.activate[BOXARM] > 0) {
        if ( rcOptions[BOXARM] && get_flag(FLAG_OK_TO_ARM) ) {
	  set_flag(FLAG_ARMED, 1);
	  headFreeModeHold = heading;
        } else if (get_flag(FLAG_ARMED)) set_flag(FLAG_ARMED, 0);
        rcDelayCommand = 0;
      #ifdef ALLOW_ARM_DISARM_VIA_TX_YAW
      } else if ( (rcData[YAW] < MINCHECK )  && get_flag(FLAG_ARMED)) {
        if (rcDelayCommand == (20*RC_FREQ/50)) set_flag(FLAG_ARMED, 0); // rcDelayCommand = 20 => 20x20ms = 0.4s = time to wait for a specific RC command to be acknowledged
      } else if ( (rcData[YAW] > MAXCHECK ) && rcData[PITCH] < MAXCHECK && !get_flag(FLAG_ARMED) && calibratingG == 0 && get_flag(FLAG_ACC_CALIBRATED)) {
        if (rcDelayCommand == (20*RC_FREQ/50)) {
	  set_flag(FLAG_ARMED, 1);
	  headFreeModeHold = heading;
        }
      #endif
      #ifdef ALLOW_ARM_DISARM_VIA_TX_ROLL
      } else if ( (rcData[ROLL] < MINCHECK)  && get_flag(FLAG_ARMED)) {
        if (rcDelayCommand == (20*RC_FREQ/50)) set_flag(FLAG_ARMED, 0); // rcDelayCommand = 20 => 20x20ms = 0.4s = time to wait for a specific RC command to be acknowledged
      } else if ( (rcData[ROLL] > MAXCHECK) && rcData[PITCH] < MAXCHECK && !get_flag(FLAG_ARMED) && calibratingG == 0 && get_flag(FLAG_ACC_CALIBRATED)) {
        if (rcDelayCommand == (20*RC_FREQ/50)) {
          set_flag(FLAG_ARMED, 1);
          headFreeModeHold = heading;
        }
      #endif
      #ifdef LCD_TELEMETRY_AUTO
      } else if (rcData[ROLL] < MINCHECK && rcData[PITCH] > MAXCHECK && !get_flag(FLAG_ARMED)) {
        if (rcDelayCommand == (20*RC_FREQ/50)) {
           if (telemetry_auto) {
              telemetry_auto = 0;
              telemetry = 0;
           } else
              telemetry_auto = 1;
        }
     #endif
      } else
        rcDelayCommand = 0;
    } else if (rcData[THROTTLE] > MAXCHECK && !get_flag(FLAG_ARMED)) {
      if (rcData[YAW] < MINCHECK && rcData[PITCH] < MINCHECK) {        // throttle=max, yaw=left, pitch=min
        if (rcDelayCommand == (20*RC_FREQ/50)) calibratingA=400;
        rcDelayCommand++;
      } else if (rcData[YAW] > MAXCHECK && rcData[PITCH] < MINCHECK) { // throttle=max, yaw=right, pitch=min  
        if (rcDelayCommand == (20*RC_FREQ/50)) calibratingM=1; // MAG calibration request
        rcDelayCommand++;
      } else if (rcData[PITCH] > MAXCHECK) {
         conf.angleTrim[PITCH]+=2;writeParams(1);
         #if defined(LED_RING)
           blinkLedRing();
         #endif
      } else if (rcData[PITCH] < MINCHECK) {
         conf.angleTrim[PITCH]-=2;writeParams(1);
         #if defined(LED_RING)
           blinkLedRing();
         #endif
      } else if (rcData[ROLL] > MAXCHECK) {
         conf.angleTrim[ROLL]+=2;writeParams(1);
         #if defined(LED_RING)
           blinkLedRing();
         #endif
      } else if (rcData[ROLL] < MINCHECK) {
         conf.angleTrim[ROLL]-=2;writeParams(1);
         #if defined(LED_RING)
           blinkLedRing();
         #endif
      } else {
        rcDelayCommand = 0;
      }
    }
    #if defined(LED_FLASHER)
      led_flasher_autoselect_sequence();
    #endif
    
    #if defined(INFLIGHT_ACC_CALIBRATION)
      if (AccInflightCalibrationArmed && get_flag(FLAG_ARMED) && rcData[THROTTLE] > MINCHECK && !rcOptions[BOXARM] ){ // Copter is airborne and you are turning it off via boxarm : start measurement
        InflightcalibratingA = 50;
        AccInflightCalibrationArmed = 0;  
      }  
      if (rcOptions[BOXPASSTHRU]) {      // Use the Passthru Option to activate : Passthru = TRUE Meausrement started, Land and passtrhu = 0 measurement stored
        if (!AccInflightCalibrationActive && !AccInflightCalibrationMeasurementDone){
          InflightcalibratingA = 50;
        }
      }else if(AccInflightCalibrationMeasurementDone && !get_flag(FLAG_ARMED)){
        AccInflightCalibrationMeasurementDone = 0;
        AccInflightCalibrationSavetoEEProm = 1;
      }
    #endif

    for(i=0;i<CHECKBOXITEMS;i++) {   
      rcOptions[i] = (
       ((rcData[AUX1]<1300)    | (1300<rcData[AUX1] && rcData[AUX1]<1700)<<1 | (rcData[AUX1]>1700)<<2
       |(rcData[AUX2]<1300)<<3 | (1300<rcData[AUX2] && rcData[AUX2]<1700)<<4 | (rcData[AUX2]>1700)<<5
       |(rcData[AUX3]<1300)<<6 | (1300<rcData[AUX3] && rcData[AUX3]<1700)<<7 | (rcData[AUX3]>1700)<<8
       |(rcData[AUX4]<1300)<<9 | (1300<rcData[AUX4] && rcData[AUX4]<1700)<<10| (rcData[AUX4]>1700)<<11) & conf.activate[i])>0;
    }

    // note: if FAILSAFE is disable, failsafeCnt > 5*FAILSAVE_DELAY is always false
    if (( rcOptions[BOXACC] || (failsafeCnt > 5*FAILSAVE_DELAY) ) && ACC ) { 
      // bumpless transfer to Level mode
      if (!get_flag(FLAG_ACC_MODE)) {
        errorAngleI[ROLL] = 0; errorAngleI[PITCH] = 0;
        set_flag(FLAG_ACC_MODE, 1);
      }  
    } else {
      // failsafe support
      set_flag(FLAG_ACC_MODE, 0);
    }

    if (rcOptions[BOXARM] == 0) set_flag(FLAG_OK_TO_ARM, 1);
    if (get_flag(FLAG_ACC_MODE)) {STABLEPIN_ON;} else {STABLEPIN_OFF;}

    #if BARO
      if (rcOptions[BOXBARO]) {
        if (!get_flag(FLAG_BARO_MODE)) {
          set_flag(FLAG_BARO_MODE, 1);
          AltHold = EstAlt;
          initialThrottleHold = rcCommand[THROTTLE];
          errorAltitudeI = 0;
          BaroPID=0;
        }
      } else {
        set_flag(FLAG_BARO_MODE, 0);
      }
    #endif
    #if MAG
      if (rcOptions[BOXMAG]) {
        if (!get_flag(FLAG_MAG_MODE)) {
          set_flag(FLAG_MAG_MODE, 1);
          magHold = heading;
        }
      } else {
        set_flag(FLAG_MAG_MODE, 0);
      }
      if (rcOptions[BOXHEADFREE]) {
        if (!get_flag(FLAG_HEADFREE_MODE)) {
          set_flag(FLAG_HEADFREE_MODE, 1);
        }
      } else {
        set_flag(FLAG_HEADFREE_MODE, 0);
      }
      if (rcOptions[BOXHEADADJ]) {
        /* acquire new heading */
        headFreeModeHold = heading;
      }
    #endif
    
    #if GPS
      #if defined(I2C_GPS)
      static uint8_t GPSNavReset = 1;
      if (get_flag(FLAG_GPS_FIX) && GPS_numSat >= 5 ) {
        if (!rcOptions[BOXGPSHOME] && !rcOptions[BOXGPSHOLD] )
          {    //Both boxes are unselected
            if (GPSNavReset == 0 ) { 
               GPSNavReset = 1; 
               GPS_I2C_command(I2C_GPS_COMMAND_STOP_NAV,0);
            }
          }  
        if (rcOptions[BOXGPSHOME]) {
         if (!get_flag(FLAG_GPS_HOME_MODE))  {
            set_flag(FLAG_GPS_HOME_MODE, 1);
            GPSNavReset = 0;
            GPS_I2C_command(I2C_GPS_COMMAND_START_NAV,0);        //waypoint zero
          }
        } else {
          set_flag(FLAG_GPS_HOME_MODE, 0);
        }
        if (rcOptions[BOXGPSHOLD]) {
          if (!get_flag(FLAG_GPS_HOLD_MODE) & !get_flag(FLAG_GPS_HOME_MODE)) {
            set_flag(FLAG_GPS_HOLD_MODE, 1);
            GPSNavReset = 0;
            GPS_I2C_command(I2C_GPS_COMMAND_POSHOLD,0);
          }
        } else {
          set_flag(FLAG_GPS_HOLD_MODE, 0);
        }
      }
      #endif 
      #if defined(GPS_SERIAL) || defined(TINY_GPS)
      if (get_flag(FLAG_GPS_FIX) && GPS_numSat >= 5 ) {
        if (rcOptions[BOXGPSHOME]) {
          if (!get_flag(FLAG_GPS_HOME_MODE))  {
            set_flag(FLAG_GPS_HOME_MODE, 1);
            GPS_set_next_wp(&GPS_home[LAT],&GPS_home[LON]);
            nav_mode    = NAV_MODE_WP;
          }
        } else {
          set_flag(FLAG_GPS_HOME_MODE, 0);
        }
        if (rcOptions[BOXGPSHOLD]) {
          if (!get_flag(FLAG_GPS_HOLD_MODE)) {
            set_flag(FLAG_GPS_HOLD_MODE, 1);
            GPS_hold[LAT] = GPS_coord[LAT];
            GPS_hold[LON] = GPS_coord[LON];
            GPS_set_next_wp(&GPS_hold[LAT],&GPS_hold[LON]);
            nav_mode = NAV_MODE_POSHOLD;
          }
        } else {
          set_flag(FLAG_GPS_HOLD_MODE, 0);
        }
      }
      #endif
    #endif
   
    if (rcOptions[BOXPASSTHRU]) {set_flag(FLAG_PASSTHRU_MODE, 1);}
    else {set_flag(FLAG_PASSTHRU_MODE, 0);}
    
    #ifdef FIXEDWING 
      set_flag(FLAG_HEADFREE_MODE, 0);
    #endif
  } else { // not in rc loop
    static uint8_t taskOrder=0; // never call all functions in the same loop, to avoid high delay spikes
    switch (taskOrder++ % 5) {
      case 0:
        #if MAG
          Mag_getADC();
        #endif
        break;
      case 1:
        #if BARO
          Baro_update();
        #endif
        break;
      case 2:
        #if BARO
          getEstimatedAltitude();
        #endif
        break;
      case 3:
        #if GPS
          if(GPS_Enable) GPS_NewData();
        #endif
        break;
      case 4:
        #if SONAR
          Sonar_update();debug3 = sonarAlt;
        #endif
        #ifdef LANDING_LIGHTS_DDR
          auto_switch_landing_lights();
        #endif
        break;
    }
  }
 
  computeIMU();
  // Measure loop rate just afer reading the sensors
  currentTime = micros();
  cycleTime = currentTime - previousTime;
  previousTime = currentTime;

  #if MAG
    if (abs(rcCommand[YAW]) <70 && get_flag(FLAG_MAG_MODE)) {
      int16_t dif = heading - magHold;
      if (dif <= - 180) dif += 360;
      if (dif >= + 180) dif -= 360;
      if ( smallAngle25 ) rcCommand[YAW] -= dif*conf.P8[PIDMAG]/30;  // 18 deg
    } else magHold = heading;
  #endif

  #if BARO
    if (get_flag(FLAG_BARO_MODE)) {
      if (abs(rcCommand[THROTTLE]-initialThrottleHold)>20) {
        set_flag(FLAG_BARO_MODE, 0); // so that a new althold reference is defined
      }
      rcCommand[THROTTLE] = initialThrottleHold + BaroPID;
    }
  #endif
  #if GPS
    //debug2 = GPS_angle[ROLL];
    //debug3 = GPS_angle[PITCH];
    // Check that we really need to navigate ?
    if ( (!get_flag(FLAG_GPS_HOME_MODE) && !get_flag(FLAG_GPS_HOME_MODE)) || !get_flag(FLAG_GPS_FIX_HOME) ) {
      // If not. Reset nav loops and all nav related parameters
      GPS_reset_nav();
    } else {
      float sin_yaw_y = sin(heading*0.0174532925f);
      float cos_yaw_x = cos(heading*0.0174532925f);
      GPS_angle[ROLL]   = (nav[LON]*cos_yaw_x - nav[LAT]*sin_yaw_y) /10;
      GPS_angle[PITCH]  = (nav[LON]*sin_yaw_y + nav[LAT]*cos_yaw_x) /10;
    }
  #endif


  //**** PITCH & ROLL & YAW PID ****    
  for(axis=0;axis<3;axis++) {
    if (get_flag(FLAG_ACC_MODE) && axis<2 ) { //LEVEL MODE
      // 50 degrees max inclination
      errorAngle = constrain(2*rcCommand[axis] + GPS_angle[axis],-500,+500) - angle[axis] + conf.angleTrim[axis]; //16 bits is ok here
      #ifdef LEVEL_PDF
        PTerm      = -(int32_t)angle[axis]*conf.P8[PIDLEVEL]/100 ;
      #else  
        PTerm      = (int32_t)errorAngle*conf.P8[PIDLEVEL]/100 ;                          // 32 bits is needed for calculation: errorAngle*P8[PIDLEVEL] could exceed 32768   16 bits is ok for result
      #endif
      PTerm = constrain(PTerm,-conf.D8[PIDLEVEL]*5,+conf.D8[PIDLEVEL]*5);

      errorAngleI[axis]  = constrain(errorAngleI[axis]+errorAngle,-10000,+10000);    // WindUp     //16 bits is ok here
      ITerm              = ((int32_t)errorAngleI[axis]*conf.I8[PIDLEVEL])>>12;            // 32 bits is needed for calculation:10000*I8 could exceed 32768   16 bits is ok for result
    } else { //ACRO MODE or YAW axis
      if (abs(rcCommand[axis])<350) error =          rcCommand[axis]*10*8/conf.P8[axis] ; // 16 bits is needed for calculation: 350*10*8 = 28000      16 bits is ok for result if P8>2 (P>0.2)
                               else error = (int32_t)rcCommand[axis]*10*8/conf.P8[axis] ; // 32 bits is needed for calculation: 500*5*10*8 = 200000   16 bits is ok for result if P8>2 (P>0.2)
      error -= gyroData[axis];

      PTerm = rcCommand[axis];
      
      errorGyroI[axis]  = constrain(errorGyroI[axis]+error,-16000,+16000);          // WindUp   16 bits is ok here
      if (abs(gyroData[axis])>640) errorGyroI[axis] = 0;
      ITerm = (errorGyroI[axis]/125*conf.I8[axis])>>6;                                   // 16 bits is ok here 16000/125 = 128 ; 128*250 = 32000
    }
    if (abs(gyroData[axis])<160) PTerm -=          gyroData[axis]*dynP8[axis]/10/8; // 16 bits is needed for calculation   160*200 = 32000         16 bits is ok for result
                            else PTerm -= (int32_t)gyroData[axis]*dynP8[axis]/10/8; // 32 bits is needed for calculation   

    delta          = gyroData[axis] - lastGyro[axis];                               // 16 bits is ok here, the dif between 2 consecutive gyro reads is limited to 800
    lastGyro[axis] = gyroData[axis];
    deltaSum       = delta1[axis]+delta2[axis]+delta;
    delta2[axis]   = delta1[axis];
    delta1[axis]   = delta;
 
    if (abs(deltaSum)<640) DTerm = (deltaSum*dynD8[axis])>>5;                       // 16 bits is needed for calculation 640*50 = 32000           16 bits is ok for result 
                      else DTerm = ((int32_t)deltaSum*dynD8[axis])>>5;              // 32 bits is needed for calculation
                      
    axisPID[axis] =  PTerm + ITerm - DTerm;
  }

  mixTable();
  writeServos();
  writeMotors();
}
