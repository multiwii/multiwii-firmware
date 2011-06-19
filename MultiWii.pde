/*
MultiWiiCopter by Alexandre Dubus
www.multiwii.com
June  2011     V1.dev
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 any later version. see <http://www.gnu.org/licenses/>
*/

#include "config.h"
#include "def.h"
#include <EEPROM.h>
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
#define PIDLEVEL   4
#define PIDMAG     5

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
static uint8_t armed = 0;
static int16_t acc_1G;             // this is the 1G measured acceleration
static uint8_t nunchuk = 0;
static uint8_t accMode = 0;        // if level mode is a activated
static uint8_t magMode = 0;        // if compass heading hold is a activated
static uint8_t baroMode = 0;       // if altitude hold is activated
static int16_t gyroADC[3],accADC[3],magADC[3];
static int16_t accSmooth[3];       // projection of smoothed and normalized gravitation force vector on x/y/z axis, as measured by accelerometer
static int16_t heading,magHold;
static int16_t altitudeSmooth = 0;
static uint8_t calibratedACC = 0;
static uint8_t vbat;               // battery voltage in 0.1V steps
static uint8_t okToArm = 0;
static uint8_t rcOptions;
static int16_t acc_z;
static int32_t vel_z;
static int32_t pos_z;

#ifdef LOG_VALUES
  static uint16_t cycleTimeMax = 0;          // highest ever cycle timen 
  static uint16_t cycleTimeMin = 65535;      // lowest ever cycle timen 
#endif
static uint32_t pMeter[7];         //we use [0:5] for six motors,[6] for sum
static uint8_t pMeterV;            // dummy to satisfy the paramStruct logic in ConfigurationLoop()
static uint32_t pAlarm;            // we scale the eeprom value from [0:255] to this value we can directly compare to the sum in pMeter[6]
static uint8_t powerTrigger1 = 0;  // trigger for alarm based on power consumption
#if defined(POWERMETER)
  #ifndef VBAT
	#error "to use powermeter, you must also define and configure VBAT"
  #endif
  #define PARAMMAX 29
  #define PARAMMOTORSTART  20
  #define PARAMMOTOREND    28
  #define PARAMMOTOROFFSET 21
#else
  #define PARAMMAX 20
#endif

#ifdef LCD_TELEMETRY
  static uint8_t telemetry = 0;
#endif
#ifdef LCD_TELEMETRY_AUTO
  #ifndef LCD_TELEMETRY
     #error "to use automatic telemetry, you MUST also define and configure LCD_TELEMETRY"
  #endif
  static uint8_t telemetry_auto = 0;
#endif

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
static int16_t angle[2]    = {0,0};  // absolute angle inclination in multiple of 0.1 degree

// *************************
// motor and servo functions
// *************************
static int16_t axisPID[3];
static int16_t motor[8];
static int16_t servo[4] = {1500,1500,1500,1500};

// **********************
// EEPROM & LCD functions
// **********************
static uint8_t P8[6], I8[5], D8[4]; //8 bits is much faster and the code is much shorter
static uint8_t dynP8[3], dynI8[3], dynD8[3];
static uint8_t rollPitchRate;
static uint8_t yawRate;
static uint8_t dynThrPID;
static uint8_t activate[6];

typedef struct {
  char*    paramText;
  uint8_t* var;
  uint8_t  decimal;
  uint8_t  increment;
} paramStruct;

static paramStruct param[30] = {
  {"PITCH&ROLL P", &P8[ROLL],1,1},
  {"ROLL   P", &P8[ROLL],1,1},     {"ROLL   I", &I8[ROLL],3,5},  {"ROLL   D", &D8[ROLL],0,1},
  {"PITCH  P", &P8[PITCH],1,1},    {"PITCH  I", &I8[PITCH],3,5}, {"PITCH  D", &D8[PITCH],0,1},
  {"YAW    P", &P8[YAW],1,1},      {"YAW    I", &I8[YAW],3,5},   {"YAW    D", &D8[YAW],0,1},
  {"ALT    P", &P8[PIDALT],1,1},   {"ALT    I", &I8[PIDALT],3,5},{"ALT    D", &D8[PIDALT],0,1},
  {"LEVEL  P", &P8[PIDLEVEL],1,1}, {"LEVEL  I", &I8[PIDLEVEL],3,5},
  {"MAG    P", &P8[PIDMAG],1,1},
  {"RC RATE", &rcRate8,2,2},       {"RC EXPO", &rcExpo8,2,2},
  {"PITCH&ROLL RATE", &rollPitchRate,2,2}, {"YAW RATE", &yawRate,2,2},
  {"THROTTLE PID", &dynThrPID,2,2},

  {"pMeter Motor 0", &pMeterV,16,0}, {"pMeter Motor 1", &pMeterV,16,0}, {"pMeter Motor 2", &pMeterV,16,0},
  {"pMeter Motor 3", &pMeterV,16,0}, {"pMeter Motor 4", &pMeterV,16,0}, {"pMeter Motor 5", &pMeterV,16,0},
  {"pMeter Sum", &pMeterV,16,0},
  {"pAlarm /50", &powerTrigger1,0,1}, // change text to represent PLEVELSCALE value
  {"Battery Volt", &vbat,1,0} //29
};

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
  #ifdef LCD_TELEMETRY_AUTO
    static uint32_t telemetryTime = 0;
  #endif
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

  #if defined(VBAT)
    vbatRaw = (vbatRaw*15 + analogRead(V_BATPIN)*16)>>4; // smoothing of vbat readings  
    vbat = vbatRaw / VBATSCALE;                  // result is Vbatt in 0.1V steps
     
    if ( (vbat>VBATLEVEL1_3S) 
    #if defined(POWERMETER)
                         && ( (pMeter[6] < pAlarm) || (pAlarm == 0) )
    #endif
                                                                        )
    {                                          //VBAT ok AND powermeter ok, buzzer off
      buzzerFreq = 0; buzzerState = 0; BUZZERPIN_OFF;
    #if defined(POWERMETER)
    } else if (pMeter[6] > pAlarm) {                             // sound alarm for powermeter
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
    if (calibratedACC == 1) LEDPIN_OFF
    if (armed) LEDPIN_ON
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
    if ( (telemetry_auto) && (micros() > telemetryTime + LCD_TELEMETRY_AUTO) ) { // every 2 seconds
      telemetry++;
      if ( (telemetry < 'A' ) || (telemetry > 'D' ) ) telemetry = 'A';
      telemetryTime = micros(); // why use micros() and not the variable currentTime ?
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
    for(uint8_t i=0;i<7;i++) // 6 is the maximum number of possible motors, array is larger by one to append the sum
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
  static int16_t altitudeHold = 0;
  static int16_t throttleHold;

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
      } else if (rcData[YAW] < MINCHECK && rcData[PITCH] < MINCHECK && armed == 0) {
        if (rcDelayCommand == 20) calibratingG=400;
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
         accZero[PITCH]++;writeParams();
      } else if (rcData[PITCH] < MINCHECK) {
         accZero[PITCH]--;writeParams();
      } else if (rcData[ROLL] > MAXCHECK) {
         accZero[ROLL]++;writeParams();
      } else if (rcData[ROLL] < MINCHECK) {
         accZero[ROLL]--;writeParams();
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
    if (((rcOptions & activate[BOXACC]) || (failsafeCnt > 5*FAILSAVE_DELAY) ) && (ACC || nunchuk) ) accMode = 1; else accMode = 0;  // modified by MIS for failsave support
    if ((rcOptions & activate[BOXARM]) == 0) okToArm = 1;
    if (accMode == 1) STABLEPIN_ON else STABLEPIN_OFF;

    if(BARO) {
      if (rcOptions & activate[BOXBARO]) {
        if (baroMode == 0) {
          baroMode = 1;
          altitudeHold = altitudeSmooth;
          throttleHold = rcCommand[THROTTLE];
          vel_z = 0;
          pos_z = 0;
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
      static uint32_t t1;
      static uint8_t stateHold = 0;
 
      vel_z += acc_z;
      vel_z = constrain(vel_z*50/51,-60000,+60000); //WindUp
      PTerm = vel_z*60/1000;
      DTerm = int32_t(acc_z) * 2;
 
      if (stateHold == 0) {
         if (millis()-t1>(min(abs(altitudeHold-altitudeSmooth)*10*D8[PIDALT]+300,1500))) {
           t1=millis();
           stateHold =1;
         }
      }
      if (stateHold == 1) {
         if (millis()-t1<300) {           
           rcCommand[THROTTLE] = constrain(rcCommand[THROTTLE] + (altitudeHold-altitudeSmooth)*P8[PIDALT]/10,max(rcCommand[THROTTLE]-60,MINTHROTTLE),min(rcCommand[THROTTLE]+60,MAXTHROTTLE))
                               - constrain(PTerm + DTerm,-100,+100);
         } else {
           t1=millis();
           stateHold = 0;
         }
      }

/*      
      vel_z += acc_z;
      vel_z = constrain(vel_z,-60000,+60000); //WindUp
      pos_z += vel_z / 2;
      pos_z = constrain(pos_z,-60000,+60000); //WindUp
      
      PTerm = vel_z*P8[PIDALT]/1000;
      ITerm = pos_z*I8[PIDALT]/50000 + (altitudeSmooth-altitudeHold)*P8[PIDMAG]/10;
      DTerm = int32_t(acc_z) * D8[PIDALT]/10;
      rcCommand[THROTTLE] = constrain(rcCommand[THROTTLE]-(PTerm + ITerm + DTerm),max(rcCommand[THROTTLE]-100,MINTHROTTLE),min(rcCommand[THROTTLE]+200,MAXTHROTTLE));
*/
    }
  }

  //**** PITCH & ROLL & YAW PID ****    
  for(axis=0;axis<3;axis++) {
    if (accMode == 1 && axis<2 ) { //LEVEL MODE
      errorAngle = rcCommand[axis] - angle[axis];                                 //500+180 = 680: 16 bits is ok here
      PTerm      = errorAngle*(P8[PIDLEVEL]/10)/10 ;                              //680*20 = 13600: 16 bits is ok here

      errorAngleI[axis] += errorAngle;                                            //16 bits is ok here
      errorAngleI[axis]  = constrain(errorAngleI[axis],-10000,+10000); //WindUp   //16 bits is ok here
      ITerm              = (int32_t)errorAngleI[axis]*I8[PIDLEVEL]/4000;          //32 bits is needed for calculation:10000*I8 could exceed 32768   16 bits is ok for result
    } else { //ACRO MODE or YAW axis
      error = (int32_t)rcCommand[axis]*10*8/P8[axis] - gyroData[axis];            //32 bits is needed for calculation: 500*10*8 = 40000   16 bits is ok for result if P>2
      PTerm = rcCommand[axis];

      errorGyroI[axis] += error;                                                  //16 bits is ok here
      errorGyroI[axis]  = constrain(errorGyroI[axis],-16000,+16000); //WindUp     //16 bits is ok here
      if (abs(gyroData[axis])>640) errorGyroI[axis] = 0;
      ITerm = (int32_t)errorGyroI[axis]*I8[axis]/1000/8;                          //32 bits is needed for calculation: 16000*I8  16 bits is ok for result
    }
    PTerm         -= (int32_t)gyroData[axis]*dynP8[axis]/10/8;                    //32 bits is needed for calculation            16 bits is ok for result

    delta          = gyroData[axis] - lastGyro[axis];                             //16 bits is ok here, because the dif between 2 consecutive gyro reads is limited
    DTerm          = (delta1[axis]+delta2[axis]+delta+1)*dynD8[axis]/3/8;         //16 bits is ok here
    delta2[axis]   = delta1[axis];
    delta1[axis]   = delta;
    lastGyro[axis] = gyroData[axis];

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
  #if defined(POWERMETER)
    logMotorsPower();
  #endif 
}
