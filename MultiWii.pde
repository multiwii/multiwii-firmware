/*
MultiWiiCopter by Alexandre Dubus
www.multiwii.com
August  2011     V1.8
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 any later version. see <http://www.gnu.org/licenses/>
*/

#include "config.h"
#include "def.h"
#define   VERSION  18

/*********** RC alias *****************/
#define ROLL       0
#define PITCH      1
#define YAW        2
#define THROTTLE   3
#define AUX1       4
#define AUX2       5
#define CAMPITCH   6
#define CAMROLL    7

#define PIDALT     3
#define PIDVEL     4
#define PIDLEVEL   5
#define PIDMAG     6

#define BOXACC      0
#define BOXBARO     1
#define BOXMAG      2
#define BOXCAMSTAB  3
#define BOXCAMTRIG  4
#define BOXARM      5


static uint32_t currentTime = 0;
static uint32_t previousTime = 0;
static uint16_t cycleTime = 0;     // this is the number in micro second to achieve a full loop, it can differ a little and is taken into account in the PID loop
static uint16_t calibratingA = 0;  // the calibration is done is the main loop. Calibrating decreases at each cycle down to 0, then we enter in a normal mode.
static uint8_t  calibratingM = 0;
static uint16_t calibratingG;
static uint8_t  armed = 0;
static int16_t  acc_1G;             // this is the 1G measured acceleration
static uint8_t  nunchuk = 0;
static uint8_t  accMode = 0;        // if level mode is a activated
static uint8_t  magMode = 0;        // if compass heading hold is a activated
static uint8_t  baroMode = 0;       // if altitude hold is activated
static int16_t  gyroADC[3],accADC[3],magADC[3];
static int16_t  accSmooth[3];       // projection of smoothed and normalized gravitation force vector on x/y/z axis, as measured by accelerometer
static int16_t  accTrim[2] = {0, 0};
static int16_t  heading,magHold;
static uint8_t  calibratedACC = 0;
static uint8_t  vbat;               // battery voltage in 0.1V steps
static uint8_t  okToArm = 0;
static uint8_t  rcOptions;
static int32_t  pressure = 0;
static float    BaroAlt = 0.0f;
static float    EstVelocity = 0.0f;
static float    EstAlt = 0.0f;

//for log
static uint16_t cycleTimeMax = 0;       // highest ever cycle timen
static uint16_t cycleTimeMin = 65535;   // lowest ever cycle timen
static uint16_t powerMax = 0;           // highest ever current
static uint16_t powerAvg = 0;           // last known current

// **********************
// power meter
// **********************
#define PMOTOR_SUM 8                     // index into pMeter[] for sum
static uint32_t pMeter[PMOTOR_SUM + 1];  //we use [0:7] for eight motors,one extra for sum
static uint8_t pMeterV;                  // dummy to satisfy the paramStruct logic in ConfigurationLoop()
static uint32_t pAlarm;                  // we scale the eeprom value from [0:255] to this value we can directly compare to the sum in pMeter[6]
static uint8_t powerTrigger1 = 0;        // trigger for alarm based on power consumption

// **********************
// telemetry
// **********************
static uint8_t telemetry = 0;
static uint8_t telemetry_auto = 0;

// ******************
// rc functions
// ******************
#define MINCHECK 1100
#define MAXCHECK 1900

volatile int16_t failsafeCnt = 0;

static int16_t rcData[8];    // interval [1000;2000]
static int16_t rcCommand[4]; // interval [1000;2000] for THROTTLE and [-500;+500] for ROLL/PITCH/YAW 

static uint8_t rcRate8;
static uint8_t rcExpo8;
static int16_t lookupRX[7]; //  lookup table for expo & RC rate

// **************
// gyro+acc IMU
// **************
static int16_t gyroData[3] = {0,0,0};
static int16_t gyroZero[3] = {0,0,0};
static int16_t accZero[3]  = {0,0,0};
static int16_t magZero[3]  = {0,0,0};
static int16_t angle[2]    = {0,0};  // absolute angle inclination in multiple of 0.1 degree    180 deg = 1800

// *************************
// motor and servo functions
// *************************
static int16_t axisPID[3];
static int16_t motor[8];
static int16_t servo[4] = {1500,1500,1500,1500};

// **********************
// EEPROM & LCD functions
// **********************
static uint8_t P8[7], I8[7], D8[7]; //8 bits is much faster and the code is much shorter
static uint8_t dynP8[3], dynI8[3], dynD8[3];
static uint8_t rollPitchRate;
static uint8_t yawRate;
static uint8_t dynThrPID;
static uint8_t activate[6];

void blinkLED(uint8_t num, uint8_t wait,uint8_t repeat) {
  uint8_t i,r;
  for (r=0;r<repeat;r++) {
    for(i=0;i<num;i++) {
      LEDPIN_SWITCH //switch LEDPIN state
      BUZZERPIN_ON delay(wait); BUZZERPIN_OFF
    }
    delay(60);
  }
}

void annexCode() { //this code is excetuted at each loop and won't interfere with control loop if it lasts less than 650 microseconds
  static uint32_t serialTime = 0;
  static uint32_t buzzerTime = 0;
  static uint32_t calibratedAccTime;
  static uint8_t  buzzerState = 0;
  static uint32_t vbatRaw = 0;       //used for smoothing voltage reading
  static uint8_t buzzerFreq;         //delay between buzzer ring
  uint8_t axis;
  uint8_t prop1,prop2;

  static uint32_t telemetryTime = 0;
  static uint32_t telemetryAutoTime = 0;
  uint16_t pMeterRaw, powerValue;                //used for current reading
  static uint32_t psensorTime = 0;

  //PITCH & ROLL only dynamic PID adjustemnt,  depending on throttle value
  if      (rcData[THROTTLE]<1500) prop2 = 100;
  else if (rcData[THROTTLE]<2000) prop2 = 100 - (rcData[THROTTLE]-1500)/5 * dynThrPID/100;
  else                            prop2 = 100 - dynThrPID;

  for(axis=0;axis<2;axis++) {
    //PITCH & ROLL dynamic PID adjustemnt, depending on stick deviation
    prop1 = 100-min(abs(rcData[axis]-1500)/5,100)*rollPitchRate/100;
    dynP8[axis] = P8[axis]*prop1/100*prop2/100;
    dynD8[axis] = D8[axis]*prop1/100*prop2/100;
  }
  
  //YAW dynamic PID adjustemnt
  prop1 = 100-min(abs(rcData[YAW]-1500)/5,100)*yawRate/100;
  dynP8[YAW] = P8[YAW]*prop1/100;
  dynD8[YAW] = D8[YAW]*prop1/100;

  #if (POWERMETER == 2)
  if (micros() > psensorTime + 19977 /*20000*/) { // 50Hz, but avoid bulking of timed tasks
     pMeterRaw =  analogRead(PSENSORPIN);
     powerValue = ( PSENSORNULL > pMeterRaw ? PSENSORNULL - pMeterRaw : pMeterRaw - PSENSORNULL); // do not use abs(), it would induce implicit cast to uint and overrun
     #ifdef LOG_VALUES
       if ( powerValue < 256) {  // only accept reasonable values. 256 is empirical
         if (powerValue > powerMax) powerMax = powerValue;
         powerAvg = powerValue;
       }
     #endif
     pMeter[PMOTOR_SUM] += (uint32_t) powerValue;
     psensorTime = micros();
  }
  #endif

  #if defined(VBAT)
    vbatRaw = (vbatRaw*15 + analogRead(V_BATPIN)*16)>>4; // smoothing of vbat readings  
    vbat = vbatRaw / VBATSCALE;                  // result is Vbatt in 0.1V steps
     
    if ( (vbat>VBATLEVEL1_3S) 
    #if defined(POWERMETER)
                         && ( (pMeter[PMOTOR_SUM] < pAlarm) || (pAlarm == 0) )
    #endif
                         || (NO_VBAT>vbat)                              ) // ToLuSe
    {                                          //VBAT ok AND powermeter ok, buzzer off
      buzzerFreq = 0; buzzerState = 0; BUZZERPIN_OFF;
    #if defined(POWERMETER)
    } else if (pMeter[PMOTOR_SUM] > pAlarm) {                             // sound alarm for powermeter
      buzzerFreq = 4;
    #endif
    } else if (vbat>VBATLEVEL2_3S)
      buzzerFreq = 1;
    else if (vbat>VBATLEVEL3_3S)
      buzzerFreq = 2;
    else
      buzzerFreq = 4;
    if (buzzerFreq) {
      if (buzzerState && (currentTime > buzzerTime + 250000) ) {
        buzzerState = 0;BUZZERPIN_OFF;buzzerTime = currentTime;
      } else if ( !buzzerState && (currentTime > (buzzerTime + (2000000>>buzzerFreq))) ) {
         buzzerState = 1;BUZZERPIN_ON;buzzerTime = currentTime;
      }
    }
  #endif

  if ( ( (calibratingA>0 && (ACC || nunchuk) ) || (calibratingG>0) ) ) {  // Calibration phasis
    LEDPIN_SWITCH
  } else {
    if (calibratedACC == 1) {LEDPIN_OFF}
    if (armed) {LEDPIN_ON}
  }

  if ( micros() > calibratedAccTime + 500000 ) {
    if (abs(angle[ROLL])>150 || abs(angle[PITCH])>150) { //more than 15 deg detection
      calibratedACC = 0; //the multi uses ACC and is not calibrated or is too much inclinated
      LEDPIN_SWITCH
      calibratedAccTime = micros();
    } else
      calibratedACC = 1;
  }
  if (micros() > serialTime + 20000) { // 50Hz
    serialCom();
    serialTime = micros();
  }
  #ifdef LCD_TELEMETRY_AUTO
    if ( (telemetry_auto) && (micros() > telemetryAutoTime + LCD_TELEMETRY_AUTO) ) { // every 2 seconds
      telemetry++;
      if ( (telemetry < 'A' ) || (telemetry > 'D' ) ) telemetry = 'A';
      telemetryAutoTime = micros(); // why use micros() and not the variable currentTime ?
    }
  #endif  
  #ifdef LCD_TELEMETRY
    if (micros() > telemetryTime +  LCD_TELEMETRY) { // 10Hz
      if (telemetry) lcd_telemetry();
      telemetryTime = micros();  
    }
  #endif  
  for(axis=0;axis<2;axis++) {
    uint16_t tmp = abs(rcData[axis]-MIDRC);
    uint16_t tmp2 = tmp/100;
    rcCommand[axis] = lookupRX[tmp2] + (tmp-tmp2*100) * (lookupRX[tmp2+1]-lookupRX[tmp2]) / 100;
    if (rcData[axis]<MIDRC) rcCommand[axis] = -rcCommand[axis];
  }
  rcCommand[THROTTLE] = MINTHROTTLE + (int32_t)(MAXTHROTTLE-MINTHROTTLE)* (rcData[THROTTLE]-MINCHECK)/(2000-MINCHECK);
  rcCommand[YAW]      = rcData[YAW]-MIDRC;
  #if defined(DEADBAND)
    if (abs(rcCommand[PITCH])< 5) rcCommand[PITCH] = 0;
    if (abs(rcCommand[ROLL]) < 5) rcCommand[ROLL] = 0;
    if (abs(rcCommand[YAW])  < 10) rcCommand[YAW] = 0;
  #endif
}


void setup() {
  Serial.begin(SERIAL_COM_SPEED);
  LEDPIN_PINMODE
  POWERPIN_PINMODE
  BUZZERPIN_PINMODE
  STABLEPIN_PINMODE
  POWERPIN_OFF  
  initOutput();
  readEEPROM();
  checkFirstTime();
  configureReceiver();
  initSensors();
  previousTime = micros();
  #if defined(GIMBAL) || defined(FLYING_WING)
   calibratingA = 400;
  #endif
  calibratingG = 400;
  #if defined(POWERMETER)
    for(uint8_t i=0;i<=PMOTOR_SUM;i++)
      pMeter[i]=0;
  #endif
}

// ******** Main Loop *********
void loop () {
  static uint8_t rcDelayCommand; // this indicates the number of time (multiple of RC measurement at 50Hz) the sticks must be maintained to run or switch off motors
  uint8_t axis,i;
  int16_t error,errorAngle;
  int16_t delta;
  int16_t PTerm,ITerm,DTerm;
  static int16_t lastGyro[3] = {0,0,0};
  static int16_t delta1[3],delta2[3];
  static int16_t errorGyroI[3] = {0,0,0};
  static int16_t errorAngleI[2] = {0,0};
  static uint8_t camCycle = 0;
  static uint8_t camState = 0;
  static uint32_t camTime = 0;
  static uint32_t rcTime  = 0;
  static int16_t initialThrottleHold;
  static int16_t errorAltitudeI = 0;
  int16_t AltPID = 0;
  static int16_t lastVelError = 0;
  static float AltHold = 0.0;
      
  if (currentTime > (rcTime + 20000) ) { // 50Hz
    rcTime = currentTime; 
    computeRC();
    // Failsafe routine - added by MIS
    #if defined(FAILSAFE)
      if ( failsafeCnt > (5*FAILSAVE_DELAY) && armed==1) {                  // Stabilize, and set Throttle to specified level
        for(i=0; i<3; i++) rcData[i] = MIDRC;                               // after specified guard time after RC signal is lost (in 0.1sec)
        rcData[THROTTLE] = FAILSAVE_THR0TTLE;
        if (failsafeCnt > 5*(FAILSAVE_DELAY+FAILSAVE_OFF_DELAY)) armed = 0; // Turn OFF motors after specified Time (in 0.1sec)
      }
      failsafeCnt++;
    #endif
    // end of failsave routine - next change is made with RcOptions setting
    if (rcData[THROTTLE] < MINCHECK) {
      errorGyroI[ROLL] = 0; errorGyroI[PITCH] = 0; errorGyroI[YAW] = 0;
      errorAngleI[ROLL] = 0; errorAngleI[PITCH] = 0;
      rcDelayCommand++;
      if (rcData[YAW] < MINCHECK && rcData[PITCH] < MINCHECK && armed == 0) {
        if (rcDelayCommand == 20) calibratingG=400;
      } else if (rcData[YAW] > MAXCHECK && rcData[PITCH] > MAXCHECK && armed == 0) {
        if (rcDelayCommand == 20) {
          servo[0] = 1500; //we center the yaw gyro in conf mode
          writeServos();
          #if defined(LCD_CONF)
            configurationLoop(); //beginning LCD configuration
          #endif
          previousTime = micros();
        }
      } else if (activate[BOXARM] > 0) {
        if ((rcOptions & activate[BOXARM]) && okToArm) armed = 1;
        else if (armed) armed = 0;
        rcDelayCommand = 0;
      } else if ( (rcData[YAW] < MINCHECK || rcData[ROLL] < MINCHECK)  && armed == 1) {
        if (rcDelayCommand == 20) armed = 0; // rcDelayCommand = 20 => 20x20ms = 0.4s = time to wait for a specific RC command to be acknowledged
      } else if ( (rcData[YAW] > MAXCHECK || rcData[ROLL] > MAXCHECK) && rcData[PITCH] < MAXCHECK && armed == 0 && calibratingG == 0 && calibratedACC == 1) {
        if (rcDelayCommand == 20) armed = 1;
     #ifdef LCD_TELEMETRY_AUTO
      } else if (rcData[ROLL] < MINCHECK && rcData[PITCH] > MAXCHECK && armed == 0) {
        if (rcDelayCommand == 20) {
           if (telemetry_auto) {
              telemetry_auto = 0;
              telemetry = 0;
           } else
              telemetry_auto = 1;
        }
     #endif
      } else
        rcDelayCommand = 0;
    } else if (rcData[THROTTLE] > MAXCHECK && armed == 0) {
      if (rcData[YAW] < MINCHECK && rcData[PITCH] < MINCHECK) {
        if (rcDelayCommand == 20) calibratingA=400;
        rcDelayCommand++;
      } else if (rcData[PITCH] > MAXCHECK) {
         accTrim[PITCH]++;writeParams();
      } else if (rcData[PITCH] < MINCHECK) {
         accTrim[PITCH]--;writeParams();
      } else if (rcData[ROLL] > MAXCHECK) {
         accTrim[ROLL]++;writeParams();
      } else if (rcData[ROLL] < MINCHECK) {
         accTrim[ROLL]--;writeParams();
      } else {
        rcDelayCommand = 0;
      }
    }
   #ifdef LOG_VALUES
    else if (armed) { // update min and max values here, so do not get cycle time of the motor arming (which is way higher than normal)
      if (cycleTime > cycleTimeMax) cycleTimeMax = cycleTime; // remember highscore
      if (cycleTime < cycleTimeMin) cycleTimeMin = cycleTime; // remember lowscore
    }
   #endif

    rcOptions = (rcData[AUX1]<1300)   + (1300<rcData[AUX1] && rcData[AUX1]<1700)*2  + (rcData[AUX1]>1700)*4
               +(rcData[AUX2]<1300)*8 + (1300<rcData[AUX2] && rcData[AUX2]<1700)*16 + (rcData[AUX2]>1700)*32;
    
    //note: if FAILSAFE is disable, failsafeCnt > 5*FAILSAVE_DELAY is always false
    if (((rcOptions & activate[BOXACC]) || (failsafeCnt > 5*FAILSAVE_DELAY) ) && (ACC || nunchuk)) { 
      // bumpless transfer to Level mode
      if (!accMode) {
        errorAngleI[ROLL] = 0; errorAngleI[PITCH] = 0;
        accMode = 1;
      }  
    } else accMode = 0;  // modified by MIS for failsave support

    if ((rcOptions & activate[BOXARM]) == 0) okToArm = 1;
    if (accMode == 1) STABLEPIN_ON else STABLEPIN_OFF;

    if(BARO) {
      if (rcOptions & activate[BOXBARO]) {
        if (baroMode == 0) {
          baroMode = 1;
          AltHold = EstAlt;
          initialThrottleHold = rcCommand[THROTTLE];
          errorAltitudeI = 0;
          lastVelError = 0;
          EstVelocity = 0;
        }
      } else baroMode = 0;
    }
    if(MAG) {
      if (rcOptions & activate[BOXMAG]) {
        if (magMode == 0) {
          magMode = 1;
          magHold = heading;
        }
      } else magMode = 0;
    }
  }
  if (MAG)  Mag_getADC();
  if (BARO) Baro_update();
    
  computeIMU();
  // Measure loop rate just afer reading the sensors
  currentTime = micros();
  cycleTime = currentTime - previousTime;
  previousTime = currentTime;

  if(MAG) {
    if (abs(rcCommand[YAW]) <70 && magMode) {
      int16_t dif = heading - magHold;
      if (dif <= - 180) dif += 360;
      if (dif >= + 180) dif -= 360;
      if ( (abs(angle[ROLL])<200) && (abs(angle[PITCH])<200) ) //20 deg
        rcCommand[YAW] -= dif*P8[PIDMAG]/30; 
    } else magHold = heading;
  }

  if(BARO) {
    if (baroMode) {
      if (abs(rcCommand[THROTTLE]-initialThrottleHold)>20) {
        baroMode = 0;
        errorAltitudeI = 0;
      }
      //**** Alt. Set Point stabilization PID ****
      error = constrain((AltHold - EstAlt)*10, -100, 100); //  +/-10m,  1 decimeter accuracy
      errorAltitudeI += error;
      errorAltitudeI = constrain(errorAltitudeI,-5000,5000);
      
      PTerm = P8[PIDALT]*error/100;                     // 16 bits is ok here
      ITerm = (int32_t)I8[PIDALT]*errorAltitudeI/4000;  // 
      
      AltPID = PTerm + ITerm ;

      //**** Velocity stabilization PD ****        
      error = constrain(EstVelocity*2000.0f, -30000.0f, 30000.0f);
      delta = error - lastVelError;
      lastVelError = error;

      PTerm = (int32_t)error * P8[PIDVEL]/800;
      DTerm = (int32_t)delta * D8[PIDVEL]/16;
      
      rcCommand[THROTTLE] = initialThrottleHold + constrain(AltPID - (PTerm - DTerm),-100,+100);
    }
  }


  //**** PITCH & ROLL & YAW PID ****    
  for(axis=0;axis<3;axis++) {
    if (accMode == 1 && axis<2 ) { //LEVEL MODE
      errorAngle = constrain(2*rcCommand[axis],-700,+700) - angle[axis] + accTrim[axis]; //16 bits is ok here
      #ifdef LEVEL_PDF
        PTerm      = -(int32_t)angle[axis]*P8[PIDLEVEL]/100 ;
      #else  
        PTerm      = (int32_t)errorAngle*P8[PIDLEVEL]/100 ;                         //32 bits is needed for calculation: errorAngle*P8[PIDLEVEL] could exceed 32768   16 bits is ok for result
      #endif

      errorAngleI[axis] += errorAngle;                                              //16 bits is ok here
      errorAngleI[axis]  = constrain(errorAngleI[axis],-10000,+10000); //WindUp     //16 bits is ok here
      ITerm              = (int32_t)errorAngleI[axis]*I8[PIDLEVEL]/4000;            //32 bits is needed for calculation:10000*I8 could exceed 32768   16 bits is ok for result
    } else { //ACRO MODE or YAW axis
      error = (int32_t)rcCommand[axis]*10*8/P8[axis] - gyroData[axis];              //32 bits is needed for calculation: 500*5*10*8 = 200000   16 bits is ok for result if P8>2 (P>0.2)
      PTerm = rcCommand[axis];

      errorGyroI[axis] += error;                                                    //16 bits is ok here
      errorGyroI[axis]  = constrain(errorGyroI[axis],-16000,+16000); //WindUp       //16 bits is ok here
      if (abs(gyroData[axis])>640) errorGyroI[axis] = 0;
      ITerm = (int32_t)errorGyroI[axis]*I8[axis]/1000/8;                            //32 bits is needed for calculation: 16000*I8  16 bits is ok for result
    }
    PTerm         -= (int32_t)gyroData[axis]*dynP8[axis]/10/8;                      //32 bits is needed for calculation            16 bits is ok for result

    delta          = gyroData[axis] - lastGyro[axis];                               //16 bits is ok here, the dif between 2 consecutive gyro reads is limited to 800
    lastGyro[axis] = gyroData[axis];
    DTerm          = (int32_t)(delta1[axis]+delta2[axis]+delta+1)*dynD8[axis]/3/8;  //32 bits is needed for calculation (800+800+800)*50 = 120000           16 bits is ok for result 
    delta2[axis]   = delta1[axis];
    delta1[axis]   = delta;

    axisPID[axis] =  PTerm + ITerm - DTerm;
  }
  
  #if defined(CAMTRIG)
    if (camCycle==1) {
      if (camState == 0) {
        servo[3] = CAM_SERVO_HIGH;
        camState = 1;
        camTime = millis();
      } else if (camState == 1) {
       if ( (millis() - camTime) > CAM_TIME_HIGH ) {
         servo[3] = CAM_SERVO_LOW;
         camState = 2;
         camTime = millis();
       }
      } else { //camState ==2
       if ( (millis() - camTime) > CAM_TIME_LOW ) {
         camState = 0;
         camCycle = 0;
       }
      }
    }
    if (rcOptions & activate[BOXCAMTRIG]) camCycle=1;
  #endif

  mixTable();
  writeServos();
  writeMotors();
  #if defined(LOG_VALUES) || (POWERMETER == 1)
    logMotorsPower();
  #endif 
}
