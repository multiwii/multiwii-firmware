#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "MultiWii.h"
#include "LCD.h"
#include "Sensors.h"
#include "Alarms.h"

void alarmPatternComposer();
void patternDecode(uint8_t resource,uint16_t first,uint16_t second,uint16_t third,uint16_t cyclepause, uint16_t endpause);
void setTiming(uint8_t resource, uint16_t pulse, uint16_t pause);
void turnOff(uint8_t resource);
void toggleResource(uint8_t resource, uint8_t activate);
void vario_output(uint16_t d, uint8_t up);
void inline switch_led_flasher(uint8_t on);
void inline switch_landing_lights(uint8_t on);
void PilotLampSequence(uint16_t speed, uint16_t pattern, uint8_t num_patterns);

static uint8_t cycleDone[5]={0,0,0,0,0}, 
               resourceIsOn[5] = {0,0,0,0,0};
static uint32_t LastToggleTime[5] ={0,0,0,0,0};
static int16_t  i2c_errors_count_old = 0;

static uint8_t SequenceActive[5]={0,0,0,0,0};

#if defined(BUZZER)
  uint8_t isBuzzerON(void) { return resourceIsOn[1]; } // returns true while buzzer is buzzing; returns 0 for silent periods
#else
  uint8_t isBuzzerON() { return 0; }
#endif  //end of buzzer define
/********************************************************************/
/****                      Alarm Handling                        ****/
/********************************************************************/
/*
AlarmArray
0: toggle
1: failsafe
2: noGPS
3: beeperOn
4: pMeter
5: runtime
6: vBat
7: confirmation
8: Acc
9: I2C Error
*/
/*
Resources:
0: onboard LED
1: Buzzer
2: PL GREEN
3: PL BLUE
4: PL RED
*/
void alarmHandler(void){
  
  #if defined(RCOPTIONSBEEP)
    static uint8_t i = 0,firstrun = 1, last_rcOptions[CHECKBOXITEMS];
                  
    if (last_rcOptions[i] != rcOptions[i]) alarmArray[0] = 1;
      last_rcOptions[i] = rcOptions[i]; 
      i++;
    if(i >= CHECKBOXITEMS)i=0;
    
    if(firstrun == 1 && alarmArray[7] == 0) {
      alarmArray[0] = 0;    //only enable options beep AFTER gyro init
      alarmArray[3] = 0;
    }        
    else firstrun = 0;
  #endif  
     
  #if defined(FAILSAFE)
    if ( failsafeCnt > (5*FAILSAFE_DELAY) && f.ARMED) {
      alarmArray[1] = 1;                                                                   //set failsafe warning level to 1 while landing
      if (failsafeCnt > 5*(FAILSAFE_DELAY+FAILSAFE_OFF_DELAY)) alarmArray[1] = 2;          //start "find me" signal after landing   
    }
    if ( failsafeCnt > (5*FAILSAFE_DELAY) && !f.ARMED) alarmArray[1] = 2;                  // tx turned off while motors are off: start "find me" signal
    if ( failsafeCnt == 0) alarmArray[1] = 0;                                              // turn off alarm if TX is okay
  #endif
  
  #if GPS
    if ((f.GPS_mode != GPS_MODE_NONE) && !f.GPS_FIX) alarmArray[2] = 2;
    else if (!f.GPS_FIX)alarmArray[2] = 1;
    else alarmArray[2] = 0;
  #endif
  
  #if defined(BUZZER)
    if ( rcOptions[BOXBEEPERON] )alarmArray[3] = 1;
    else alarmArray[3] = 0;
  #endif

  #if defined(POWERMETER)
    if ( (pMeter[PMOTOR_SUM] < pAlarm) || (pAlarm == 0) || !f.ARMED) alarmArray[4] = 0;
    else if (pMeter[PMOTOR_SUM] > pAlarm) alarmArray[4] = 1;                  
  #endif 
  
  #if defined(ARMEDTIMEWARNING)
    if (ArmedTimeWarningMicroSeconds > 0 && armedTime >= ArmedTimeWarningMicroSeconds && f.ARMED) alarmArray[5] = 1;
    else alarmArray[5] = 0;
  #endif
  
  #if defined(VBAT)
    if (vbatMin < conf.vbatlevel_crit) alarmArray[6] = 4;
    else if ( (analog.vbat > conf.vbatlevel_warn1)  || (NO_VBAT > analog.vbat)) alarmArray[6] = 0;
    else if (analog.vbat > conf.vbatlevel_warn2) alarmArray[6] = 1;
    else if (analog.vbat > conf.vbatlevel_crit) alarmArray[6] = 2;
    //else alarmArray[6] = 4;
  #endif
  
  if (i2c_errors_count > i2c_errors_count_old+100 || i2c_errors_count < -1) alarmArray[9] = 1;
  else alarmArray[9] = 0;
   
  alarmPatternComposer();
}

void alarmPatternComposer(){ 
  static char resource = 0;
  // patternDecode(length1,length2,length3,beeppause,endpause,loop)
  #if defined(BUZZER)
    resource = 1;                                                                                  //buzzer selected
    if (alarmArray[1] == 2)       patternDecode(resource,200,0,0,50,2000);                       //failsafe "find me" signal
    else if (alarmArray[1] == 1 || alarmArray[8] == 1) patternDecode(resource,50,200,200,50,50); //failsafe "panic"  or Acc not calibrated                     
    else if (alarmArray[0] == 1)  patternDecode(resource,50,0,0,50,0);                           //toggle 1
    else if (alarmArray[0] == 2)  patternDecode(resource,50,50,0,50,0);                          //toggle 2       
    else if (alarmArray[0] > 2)   patternDecode(resource,50,50,50,50,0);                         //toggle else         
    else if (alarmArray[2] == 2)  patternDecode(resource,50,50,0,50,50);                         //gps installed but no fix    
    else if (alarmArray[3] == 1)  patternDecode(resource,50,50,50,50,50);                        //BeeperOn
    else if (alarmArray[4] == 1)  patternDecode(resource,50,50,0,50,120);                        //pMeter Warning
    else if (alarmArray[5] == 1)  patternDecode(resource,50,50,50,50,0);                         //Runtime warning      
    else if (alarmArray[6] == 4)  patternDecode(resource,50,50,200,50,2000);                     //vbat critical
    else if (alarmArray[6] == 2)  patternDecode(resource,50,200,0,50,2000);                      //vbat warning      
    else if (alarmArray[6] == 1)  patternDecode(resource,200,0,0,50,2000);                       //vbat info
    else if (alarmArray[7] == 1)  patternDecode(resource,200,0,0,50,200);                        //confirmation indicator 1x
    else if (alarmArray[7] == 2)  patternDecode(resource,200,200,0,50,200);                      //confirmation indicator 2x 
    else if (alarmArray[7] > 2)   patternDecode(resource,200,200,200,50,200);                    //confirmation indicator 3x
    else if (SequenceActive[(uint8_t)resource] == 1) patternDecode(resource,0,0,0,0,0);                   // finish last sequence if not finished yet
    else turnOff(resource);                                                                        // turn off the resource 
    alarmArray[8] = 0;                                                                             //reset acc not calibrated
    
  #endif
  #if defined(PILOTLAMP)
    if (alarmArray[9] == 1)   PilotLampSequence(100,B000111,2);                                   //I2C Error
    else if (alarmArray[3] == 1)  PilotLampSequence(100,B0101<<8|B00010001,4);                    //BeeperOn
    else{        
      resource = 2; 
      if (f.ARMED && f.ANGLE_MODE) patternDecode(resource,100,100,100,100,1000);                //Green Slow Blink-->angle
      else if (f.ARMED && f.HORIZON_MODE) patternDecode(resource,200,200,200,100,1000);         //Green mid Blink-->horizon
      else if (f.ARMED) patternDecode(resource,100,100,0,100,1000);                             //Green fast Blink-->acro
      else turnOff(resource);                                                               //switch off
      resource = 3; 
      #if GPS
        if (alarmArray[2]==1) patternDecode(resource,100,100,100,100,100);                      // blue fast blink -->no gps fix
		else if (f.GPS_mode != GPS_MODE_NONE) patternDecode(resource,100,100,100,100,1000); //blue slow blink --> gps active
        else setTiming(resource,100,1000);                                                        //blue short blink -->gps fix ok
      #else
        turnOff(resource);
      #endif   
      resource = 4; 
      if (alarmArray[1] == 1)       setTiming(resource,100,100);                                  //Red fast blink--> failsafe panic
      else if (alarmArray[1] == 2)  patternDecode(resource,1000,0,0,0,2000);                    //red slow blink--> failsafe find me
      else turnOff(resource); 
    }
  #endif 
}

void patternDecode(uint8_t resource,uint16_t first,uint16_t second,uint16_t third,uint16_t cyclepause, uint16_t endpause){
  static uint16_t pattern[5][5];
  static uint8_t icnt[5] = {0,0,0,0,0};
  
  if(SequenceActive[resource] == 0){
    SequenceActive[resource] = 1; 
    pattern[resource][0] = first; 
    pattern[resource][1] = second;
    pattern[resource][2] = third;
    pattern[resource][3] = endpause;
    pattern[resource][4] = cyclepause;
  }
  if(icnt[resource] <3 ){
    if (pattern[resource][icnt[resource]] != 0){
      setTiming(resource,pattern[resource][icnt[resource]],pattern[resource][4]);
     }
  }
  else if (LastToggleTime[resource] < (millis()-pattern[resource][3]))  {  //sequence is over: reset everything
    icnt[resource]=0;
    SequenceActive[resource] = 0;                               //sequence is now done, cycleDone sequence may begin
    alarmArray[0] = 0;                                //reset toggle bit
    alarmArray[7] = 0;                                //reset confirmation bit
    turnOff(resource);   
    return;
  }
  if (cycleDone[resource] == 1 || pattern[resource][icnt[resource]] == 0) {            //single on off cycle is done
    if (icnt[resource] < 3) {
      icnt[resource]++;
    }
    cycleDone[resource] = 0;
    turnOff(resource);    
  }  
}

void turnOff(uint8_t resource){
  if (resource == 1) {
    if (resourceIsOn[1]) {
      BUZZERPIN_OFF;
      resourceIsOn[1] = 0;
    }
  }else if (resource == 0) {
    if (resourceIsOn[0]) {
      resourceIsOn[0] = 0;
      LEDPIN_OFF;
    }
  }else if (resource == 2) {
    if (resourceIsOn[2]) {
      resourceIsOn[2] = 0;
      #if defined (PILOTLAMP)
        PilotLamp(PL_GRN_OFF);
      #endif
    }
  }else if (resource == 3) {
    if (resourceIsOn[3]) {
      resourceIsOn[3] = 0;
      #if defined (PILOTLAMP)
        PilotLamp(PL_BLU_OFF);
      #endif
    }
  }else if (resource == 4) {
    if (resourceIsOn[4]) {
      resourceIsOn[4] = 0;
      #if defined (PILOTLAMP)
        PilotLamp(PL_RED_OFF);
      #endif
    }
  }
}

#if defined (PILOTLAMP) 
  //original code based on mr.rc-cam and jevermeister work
  //modified by doughboy to use timer interrupt

  #define PL_BUF_SIZE 8
  volatile uint8_t queue[PL_BUF_SIZE]; //circular queue
  volatile uint8_t head = 0;
  volatile uint8_t tail = 0;

/********************************************************************/
/****                   Pilot Lamp Handling                      ****/
/********************************************************************/


//define your light pattern by bits, 0=off 1=on
//define up to 5 patterns that cycle using 15 bits, pattern starts at bit 0 in groups of 3
//  14 13 12 11 10 9 8 7 6 5 4 3 2 1 0            
//   R  B  G  R  B G R B G R B G R B G
//parameters speed is the ms between patterns
//           pattern is the 16bit (uses only 15 bits) specifying up to 5 patterns
//           num_patterns is the number of patterns defined
//example to define sequential light where G->B->R then back to G, you need 4 patterns
//           B00000101<<8 | B00010001
//example: to define alternate all on and al off, you only need two patterns
//           B00000111
  void PilotLampSequence(uint16_t speed, uint16_t pattern, uint8_t num_patterns){
    static uint32_t lastswitch = 0;
    static uint8_t seqno = 0;
    static uint16_t lastpattern = 0;  //since variables are static, when switching patterns, the correct pattern will start on the second sequence
    
    if(millis() < (lastswitch + speed))                                 
      return; //not time to change pattern yet
    lastswitch = millis();
   
    for (uint8_t i=0;i<3;i++) {
      uint8_t state = (pattern >>(seqno*3+i)) & 1; //get pattern bit
      //since value is multiple of 25, we can calculate time based on pattern position
      //i=0 is green, 1=blue, 2=red, green ON ticks is calculated as 50*(0+1)-1*25 = 25 ticks
      uint8_t tick = 50*(i+1);
      if (state)
        tick -=25;
      PilotLamp(tick);
      resourceIsOn[i+2]=state;
    }
   seqno++;
   seqno%=num_patterns;
  }

  void PilotLamp(uint8_t count){
    if (((tail+1)%PL_BUF_SIZE)!=head) {
      queue[tail]=count;
      tail++;
      tail=(tail%PL_BUF_SIZE);
    }
  }
  //ISR code sensitive to changes, test thoroughly before flying
  ISR(PL_ISR) { //the interrupt service routine
   static uint8_t state = 0;
   uint8_t h = head;
   uint8_t t = tail;
   if (state==0) {
     if (h!=t) {
       uint8_t c = queue[h];
       PL_PIN_ON;
       PL_CHANNEL+=c;
       h = ((h+1) % PL_BUF_SIZE);
       head = h;
       state=1;
     }
   } else if (state==1) {
     PL_PIN_OFF;
     PL_CHANNEL+=PL_IDLE;
     state=0;
   } 
 }
#endif

/********************************************************************/
/****                         LED Handling                       ****/
/********************************************************************/
//Beware!! Is working with delays, do not use inflight!

void blinkLED(uint8_t num, uint8_t ontime,uint8_t repeat) {
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
      delay(ontime);
      #if defined(LED_FLASHER)
        switch_led_flasher(0);
      #endif
      #if defined(LANDING_LIGHTS_DDR)
        switch_landing_lights(0);
      #endif
    }
    delay(60); //wait 60 ms
  }
}

/********************************************************************/
/****                   Global Resource Handling                 ****/
/********************************************************************/

  void setTiming(uint8_t resource, uint16_t pulse, uint16_t pause){
    if (!resourceIsOn[resource] && (millis() >= (LastToggleTime[resource] + pause))&& pulse != 0) {
      resourceIsOn[resource] = 1;      
      toggleResource(resource,1);
      LastToggleTime[resource]=millis();      
    } else if ( (resourceIsOn[resource] && (millis() >= LastToggleTime[resource] + pulse) ) || (pulse==0 && resourceIsOn[resource]) ) {
      resourceIsOn[resource] = 0;
      toggleResource(resource,0);
      LastToggleTime[resource]=millis();
      cycleDone[resource] = 1;     
    } 
  } 
 
  void toggleResource(uint8_t resource, uint8_t activate){
     switch(resource) {     
        #if defined (BUZZER)   
          case 1:
            if (activate == 1) {BUZZERPIN_ON;}
            else BUZZERPIN_OFF;
            break; 
        #endif
        #if defined (PILOTLAMP) 
          case 2:
            if (activate == 1) PilotLamp(PL_GRN_ON);
            else PilotLamp(PL_GRN_OFF);
            break;
          case 3: 
            if (activate == 1) PilotLamp(PL_BLU_ON);
            else PilotLamp(PL_BLU_OFF);
            break;
          case 4: 
            if (activate == 1) PilotLamp(PL_RED_ON);
            else PilotLamp(PL_RED_OFF);
            break;
        #endif
        case 0:
        default:
          if (activate == 1) {LEDPIN_ON;}
          else LEDPIN_OFF;
          break;
      }
      return;
  }


/********************************************************************/
/****                      LED Ring Handling                     ****/
/********************************************************************/

  #if defined(LED_RING)
  
  #define LED_RING_ADDRESS 0xDA //7 bits
  
  void i2CLedRingState(void) {
    uint8_t b[10];
  
    b[0]='M'; // MultiwII mode
    if (f.ARMED) { // Motors running = flying
      if(!(f.ANGLE_MODE||f.HORIZON_MODE)){ //ACRO
        b[0]= 'x';
      }
	  else if(f.GPS_mode == GPS_MODE_RTH){ //RTH
        b[0]= 'w';
      }   
	  else if(f.GPS_mode == GPS_MODE_HOLD){//Position Hold
        b[0]= 'v';
      } 
      else if(f.HORIZON_MODE){ //HORIZON mode
        b[0]= 'y';
      }   
      else {
        b[0]= 'u'; // ANGLE mode
      }  
      i2c_rep_start(LED_RING_ADDRESS);
      i2c_write(b[0]);
      i2c_stop();  
    } 
    else if (!f.ACC_CALIBRATED) { // Multiwii not stable or uncalibrated 
      b[0]= 't';
      i2c_rep_start(LED_RING_ADDRESS);
      i2c_write(b[0]);
      i2c_stop();   
    }
    else { // Motors not running = on the ground
      b[0]= 's';
      if (f.ANGLE_MODE) b[1]=1; 
      else if (f.HORIZON_MODE) b[1]=2; 
      else b[1]= 0;                  
      if (f.BARO_MODE) b[2]=1; 
      else b[2]= 0;                  
      if (f.MAG_MODE) b[3]=1; 
      else b[3]= 0;                  
  #if GPS
      if (rcOptions[BOXGPSHOME]) b[4]=2;
      else if (rcOptions[BOXGPSHOLD]) b[4]=1;
      else b[4]=0;
  #else
      b[4]=0;
  #endif
      b[5]=(180-att.heading)/2; // 1 unit = 2 degrees;
      b[6]=GPS_numSat;                                      
      i2c_rep_start(LED_RING_ADDRESS);
      for(uint8_t i=0;i<7;i++){
        i2c_write(b[i]);
      }
      i2c_stop();
    }
  #if defined (VBAT)
    if (analog.vbat < conf.vbatlevel_warn1){ // Uh oh - battery low
      i2c_rep_start(LED_RING_ADDRESS);
      i2c_write('r');
      i2c_stop();   
    }
  # endif
  }
  
  void blinkLedRing(void) {
    uint8_t b[3];
    b[0]='g';
    b[1]= 10;
    b[2]= 10;
    i2c_rep_start(LED_RING_ADDRESS<<1);
    for(uint8_t i=0;i<3;i++)
      i2c_write(b[i]);
    i2c_stop();
  }
#endif

/********************************************************************/
/****                    LED flasher Handling                    ****/
/********************************************************************/

#if defined(LED_FLASHER)
  static uint8_t led_flasher_sequence = 0;
  /* if we load a specific sequence and do not want it change, set this flag */
  static enum {
    LED_FLASHER_AUTO,
    LED_FLASHER_CUSTOM
  } led_flasher_control = LED_FLASHER_AUTO;
  
  void init_led_flasher() {
    #if defined(LED_FLASHER_DDR)
    LED_FLASHER_DDR |= (1<<LED_FLASHER_BIT);
    switch_led_flasher(0);
    #endif
  }
  
  void led_flasher_set_sequence(uint8_t s) {
    led_flasher_sequence = s;
  }
  
  void inline switch_led_flasher(uint8_t on) {
    #if defined(LED_FLASHER_DDR)
      #ifndef LED_FLASHER_INVERT
        if (on) {
      #else
        if (!on) {
      #endif
        LED_FLASHER_PORT |= (1<<LED_FLASHER_BIT);
      } else {
        LED_FLASHER_PORT &= ~(1<<LED_FLASHER_BIT);
      }
    #endif
  }
  
  static uint8_t inline led_flasher_on() {
    uint8_t seg = (currentTime/1000/125)%8;
    return (led_flasher_sequence & 1<<seg);
  }
  
  void auto_switch_led_flasher() {
    if (led_flasher_on()) {
      switch_led_flasher(1);
    } else {
      switch_led_flasher(0);
    }
  }
  
  /* auto-select a fitting sequence according to the
   * copter situation
   */
  void led_flasher_autoselect_sequence() {
    if (led_flasher_control != LED_FLASHER_AUTO) return;

    #if defined(LED_FLASHER_SEQUENCE_MAX)
    /* do we want the complete illumination no questions asked? */
    if (rcOptions[BOXLEDMAX]) {
      led_flasher_set_sequence(LED_FLASHER_SEQUENCE_MAX);
      return;
    }
    #endif

    #if defined(LED_FLASHER_SEQUENCE_LOW)
    if (rcOptions[BOXLEDLOW]) {
      led_flasher_set_sequence(LED_FLASHER_SEQUENCE_LOW);
      return;
    }
    #endif

    #if defined(LED_FLASHER_SEQUENCE_ARMED)
    /* do we have a special sequence for armed copters? */
    if (f.ARMED) {
      led_flasher_set_sequence(LED_FLASHER_SEQUENCE_ARMED);
      return;
    }
    #endif

    /* Let's load the plain old boring sequence as a last resort */
    led_flasher_set_sequence(LED_FLASHER_SEQUENCE);
  }
  
  #endif
  
  #if defined(LANDING_LIGHTS_DDR)
  void init_landing_lights(void) {
    LANDING_LIGHTS_DDR |= 1<<LANDING_LIGHTS_BIT;
    switch_landing_lights(0);
  }
  
  void inline switch_landing_lights(uint8_t on) {
    #ifndef LANDING_LIGHTS_INVERT
    if (on) {
    #else
    if (!on) {
    #endif
      LANDING_LIGHTS_PORT |= 1<<LANDING_LIGHTS_BIT;
    } else {
      LANDING_LIGHTS_PORT &= ~(1<<LANDING_LIGHTS_BIT);
    }
  }
  
  void auto_switch_landing_lights(void) {
    if (rcOptions[BOXLLIGHTS]
    #if defined(LANDING_LIGHTS_AUTO_ALTITUDE) & SONAR
        || (sonarAlt >= 0 && sonarAlt <= LANDING_LIGHTS_AUTO_ALTITUDE && f.ARMED)
    #endif
    #if defined(LED_FLASHER_DDR) & defined(LANDING_LIGHTS_ADOPT_LED_FLASHER_PATTERN)
        || (led_flasher_on())
    #endif
    ) {
      switch_landing_lights(1);
    } else {
      switch_landing_lights(0);
    }
  }
#endif

  /********************************************************************/
  /****                    Variometer signaling                    ****/
  /********************************************************************/
#ifdef VARIOMETER
#define TRESHOLD_UP    50           // (m1) treshhold for up velocity
#define TRESHOLD_DOWN  40           // (m1) treshhold for up velocity
#define TRESHOLD_UP_MINUS_DOWN  10  // (m1) you compute: TRESHOLD_UP - TRESHOLD_DOWN
#define ALTITUDE_INTERVAL 400       // (m2) in calls; interval to perodically observe altitude change
#define DELTA_ALT_TRESHOLD 200      // (m2) in cm; treshold for delta altitude after ALTITUDE_INTERVAL
#define DELTA_T 5                   // (m2) divisor for delta_alt to compute vel
#define SIGNAL_SCALE   4       // you compute: (50ms per beep / 5*3ms cycle time)
#define SILENCE_M      200     // max duration of silence in calls
#define SILENCE_SCALE  33      // vario scale: larger -> slower decay of silence
#define SILENCE_A      6600    // you compute: SILENCE_M * SILENCE_SCALE
#define DURATION_SUP   5       // sup duration of signal
#define DURATION_SCALE 100     // vario scale: larger -> slower rise of length

  /* vario_signaling() gets called every 5th cycle (~2ms - 5ms) -> (~10ms - 25ms)
   * modulates silence duration between tones and tone duration
   * higher abs(vario) -> shorther silence & longer signal duration.
   * Utilize two methods for combined short and long term observation
   */
void vario_signaling(void) {
  static int16_t last_v = 0;
  static uint16_t silence = 0;
  static int16_t max_v = 0;
  static uint8_t max_up = 0;

  uint16_t s = 0;
  int16_t v = 0;

  /* method 1: use vario to follow short term up/down movement : */
  #if (VARIOMETER == 1) || (VARIOMETER == 12)
  {
    uint8_t up = (alt.vario > 0 ? 1 : 0 ); //, down = (vario < 0 ? 1 : 0 );
    //int16_t v = abs(vario) - up * TRESHOLD_UP - down * TRESHOLD_DOWN;
    v = abs(alt.vario) - up * (TRESHOLD_UP_MINUS_DOWN) - TRESHOLD_DOWN;
    if (silence>0) silence--; else silence = 0;
    if (v > 0) {
      // going up or down
      if (v > last_v) {
        // current speed greater than speed for last signal,
        // so shorten the remaining silence period
        s = (SILENCE_A) / (SILENCE_SCALE + v);
        if (silence > s)  silence = s;
      }
      // remember interim max v
      if (v > max_v) {
        max_v = v;
        max_up = up;
      }
    } // end of (v>0)
  }
  #endif // end method 1
  /* method 2: use altitude to follow long term up/down movement : */
  #if (VARIOMETER == 2) || (VARIOMETER == 12)
  {
    static uint16_t t = 0;
    if (!(t++ % ALTITUDE_INTERVAL)) {
      static int32_t last_BaroAlt = 0;
      int32_t delta_BaroAlt = alt.EstAlt - last_BaroAlt;
      if (abs(delta_BaroAlt) > DELTA_ALT_TRESHOLD) {
        // inject suitable values
        max_v = abs(delta_BaroAlt / DELTA_T);
        max_up = (delta_BaroAlt > 0 ? 1 : 0);
        silence = 0;
      }
      last_BaroAlt = alt.EstAlt;
    }
  }
  #endif // end method 2
  /* something to signal now? */
  if ( (silence == 0) && (max_v > 0) ) {
    // create new signal
    uint16_t d = (DURATION_SUP * max_v)/(DURATION_SCALE + max_v);
    s = (SILENCE_A) / (SILENCE_SCALE + max_v);
    s+= d * SIGNAL_SCALE;
    vario_output(d, max_up);
    last_v = v;
    max_v = 0;
    max_up = 0;
    silence = s;
   }
} // end of vario_signaling()

void vario_output(uint16_t d, uint8_t up) {
  if (d == 0) return;
  #if defined(SUPPRESS_VARIOMETER_UP)
    if (up) return;
  #elif defined(SUPPRESS_VARIOMETER_DOWN)
    if (!up) return;
  #endif
  #ifdef VARIOMETER_SINGLE_TONE
    uint8_t s1 = 0x07;
    uint8_t d1 = d;
  #else
    uint8_t s1 = (up ? 0x05 : 0x07);
    uint8_t d1 = d/2;
  #endif
  if (d1<1) d1 = 1;
  for (uint8_t i=0; i<d1; i++) LCDprint(s1);
  #ifndef VARIOMETER_SINGLE_TONE
    uint8_t s2 = (up ? 0x07 : 0x05);
    uint8_t d2 = d-d1;
    if (d2<1) d2 = 1;
    for (uint8_t i=0; i<d2; i++) LCDprint(s2);
  #endif
}

#endif

