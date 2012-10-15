static uint8_t cycle_Done[5]={0,0,0,0,0}, 
               channelIsOn[5] = {0,0,0,0,0};
static uint32_t channelLastToggleTime[5] ={0,0,0,0,0};
static int16_t  i2c_errors_count_old = 0;

static uint8_t SequenceActive=0;

#if defined(BUZZER)
  uint8_t isBuzzerON() { return channelIsOn[1]; } // returns true while buzzer is buzzing; returns 0 for silent periods
#endif
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
void alarmHandler(){
  
  #if defined(RCOPTIONSBEEP)
    static uint8_t i = 0,firstrun = 1, last_rcOptions[CHECKBOXITEMS];
                  
    if (last_rcOptions[i] != rcOptions[i])alarmArray[0] = 1;
      last_rcOptions[i] = rcOptions[i]; 
      i++;
    if(i >= CHECKBOXITEMS)i=0;
    
    if(firstrun == 1 && alarmArray[7] == 0){
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
    if ((f.GPS_HOME_MODE || f.GPS_HOLD_MODE) && !f.GPS_FIX)alarmArray[2] = 2;  
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
    if (armedTime >= ArmedTimeWarningMicroSeconds && f.ARMED)alarmArray[5] = 1;
    else alarmArray[5] = 0;
  #endif
  
  #if defined(VBAT)
    if (vbatMin < conf.vbatlevel_crit) alarmArray[6] = 4;
    else if ( (vbat>conf.vbatlevel1_3s)  || (conf.no_vbat > vbat))alarmArray[6] = 0;
    else if (vbat > conf.vbatlevel2_3s) alarmArray[6] = 2;
    else alarmArray[6] = 4;
  #endif
  
  if (i2c_errors_count > i2c_errors_count_old+100 || i2c_errors_count < -1)alarmArray[9] = 1;
  else alarmArray[9] = 0;
    
  
  #if defined(BUZZER)
    buzzerHandler();
  #endif
  #if defined(PILOTLAMP)
    PilotLampHandler();
  #endif
}

#if defined(BUZZER)
  void buzzerHandler(){ 
    static char resource = 1;
    // beepcode(length1,length2,length3,pause)
    if (alarmArray[1] == 2)       patternDecode(resource,200,0,0,2000);                       //failsafe "find me" signal
    else if (alarmArray[1] == 1 || alarmArray[8] == 1) patternDecode(resource,50,200,200,50); //failsafe "panic"  or Acc not calibrated                     
    else if (alarmArray[0] == 1)  patternDecode(resource,50,0,0,0);                           //toggle 1
    else if (alarmArray[0] == 2)  patternDecode(resource,50,50,0,0);                          //toggle 2       
    else if (alarmArray[0] > 2)   patternDecode(resource,50,50,50,0);                         //toggle else         
    else if (alarmArray[2] == 2)  patternDecode(resource,50,50,0,50);                         //gps installed but no fix    
    else if (alarmArray[3] == 1)  patternDecode(resource,50,50,50,50);                        //beeperon
    else if (alarmArray[4] == 1)  patternDecode(resource,50,50,0,120);                        //pMeter Warning
    else if (alarmArray[5] == 1)  patternDecode(resource,50,50,50,0);                         //Runtime warning      
    else if (alarmArray[6] == 4)  patternDecode(resource,50,50,200,2000);                     //vbat critical
    else if (alarmArray[6] == 2)  patternDecode(resource,50,200,0,2000);                      //vbat warning      
    else if (alarmArray[6] == 1)  patternDecode(resource,200,0,0,2000);                       //vbat info
    else if (alarmArray[7] == 1)  patternDecode(resource,200,0,0,200);                        //confirmation indicator 1x
    else if (alarmArray[7] == 2)  patternDecode(resource,200,200,0,200);                      //confirmation indicator 2x 
    else if (alarmArray[7] > 2)   patternDecode(resource,200,200,200,200);                    //confirmation indicator 3x
    else if (SequenceActive == 1) patternDecode(resource,0,0,0,0);                            // finish last sequence if not finished yet
    else{                                                                                     //reset everything and keep quiet
      if (channelIsOn[1]) {
        channelIsOn[1] = 0;
        BUZZERPIN_OFF;
      }
    } 
  alarmArray[8] = 0;     //reset acc not calibrated
  }
#endif 
#if defined(PILOTLAMP)
  void PilotLampHandler(){
    if (alarmArray[9] == 1)   PilotLampSequence(100,B000111,2);                //I2C Error
    else if (alarmArray[3] == 1)  PilotLampSequence(100,B0101<<8|B00010001,4); //BuzzerOn
    else{
      if (alarmArray[1] == 1)  useResource(4,100,100);                         //Red fast blink--> failsafe panic
      else if (alarmArray[1] == 2)  useResource(4,1000,2000);                  //red slow blink--> failsafe find me
      else useResource(4,0,0);
      if (f.ARMED && f.ANGLE_MODE) useResource(2,1000,1000);                   //Green Slow Blink-->angle
      else if (f.ARMED && f.HORIZON_MODE) useResource(2,1000,500);             //Green mid Blink-->horizon
      else if (f.ARMED) useResource(2,1000,100);                               //Green fast Blink-->acro                               //Green fast Blink-->acro
      else useResource(2,0,0);                                                 //switch off
      #if GPS
        if (alarmArray[2]==1) useResource('B',100,100);                           // blue fast blink -->no gps fix
        else if (f.GPS_HOME_MODE || f.GPS_HOLD_MODE) useResource(3,1000,1000); //blue slow blink --> gps active
        else useResource(3,100,1000);                                          //blue short blink -->gps fix ok
      #else
        useResource(3,0,0);
      #endif   
   }
 }
#endif  
void patternDecode(uint8_t channel,uint16_t first,uint16_t second,uint16_t third,uint16_t pause){
  static uint16_t patternInt[4];
  static uint8_t icnt = 0;
  if(icnt == 0){
    patternInt[0] = first; 
    patternInt[1] = second;
    patternInt[2] = third;
    patternInt[3] = pause;
  }
  if(icnt <3 ){
    useResource(channel,patternInt[icnt],50);
  }
  if (icnt >=3 && (channelLastToggleTime[1]<millis()-patternInt[icnt]) ){//sequence is over: reset everything
    icnt=0;
    alarmArray[0] = 0;                                //reset toggle bit
    alarmArray[7] = 0;                                //reset confirmation bit
    SequenceActive = 0;                               //sequence is now done, next sequence may begin
    if (channel == 1){
      if (channelIsOn[1]) {
        BUZZERPIN_OFF;
        channelIsOn[1] = 0;
      }
    }else if (channel == 0){
      if (channelIsOn[0]) {
        channelIsOn[0] = 0;
        LEDPIN_OFF;
      }
    }else if (channel ==2){
      if (channelIsOn[2]) {
        channelIsOn[2] = 0;
        PL_GRN_OFF;
      }
    }
    return;
  }
  if (cycle_Done[channel] == 1 || patternInt[icnt] == 0){//single on off cycle is done
    if (icnt < 3){icnt++;} 
    cycle_Done[channel] = 0;
    if (channel == 1){
      if (channelIsOn[1]) {
        BUZZERPIN_OFF;
        channelIsOn[1] = 0;
      }
    }else if (channel == 0){
      if (channelIsOn[0]) {
        channelIsOn[0] = 0;
        LEDPIN_OFF;
      }
    }else if (channel == 2){
      if (channelIsOn[2]) {
        channelIsOn[2] = 0;
        PL_GRN_OFF;
      }
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
      channelIsOn[i+2]=state;
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
/****                         Global Handling                    ****/
/********************************************************************/

  int useResource(uint8_t channel, uint16_t pulse, uint16_t pause){ 
    if (!channelIsOn[channel] && (millis() >= (channelLastToggleTime[channel] + pause))&& pulse != 0) {	         
      channelIsOn[channel] = 1;      
      ChannelToOutput(channel,1);
      channelLastToggleTime[channel]=millis();      
    } else if (channelIsOn[channel] && (millis() >= channelLastToggleTime[channel] + pulse)|| (pulse==0 && channelIsOn[channel]) ) {       
      channelIsOn[channel] = 0;
      ChannelToOutput(channel,0);
      channelLastToggleTime[channel]=millis();
      cycle_Done[channel] = 1;     
    } 
  } 
 
  void ChannelToOutput(uint8_t channel, uint8_t activate){
     switch(channel) {     
        #if defined (BUZZER)   
          case 1:
            if (activate == 1) {BUZZERPIN_ON;}
            else {BUZZERPIN_OFF;}
            break; 
        #endif
        #if defined (PILOTLAMP) 
          case 2:
            if (activate == 1) PilotLamp(PL_GRN_ON);
            else PilotLamp(PL_GRN_OFF);
            break;
          case 3: 
            if (activate == 1)PilotLamp(PL_BLU_ON);
            else PilotLamp(PL_BLU_OFF);
            break;
          case 4: 
            if (activate == 1)PilotLamp(PL_RED_ON);
            else PilotLamp(PL_RED_OFF);
            break;
        #endif
	case 0:	
        default:
          if (activate == 1){LEDPIN_ON;}
          else {LEDPIN_OFF;}
          break;
      }
      return;
  }


/********************************************************************/
/****                      LED Ring Handling                     ****/
/********************************************************************/

  #if defined(LED_RING)
  
  #define LED_RING_ADDRESS 0xDA //7 bits
  
  void i2CLedRingState() {
    uint8_t b[10];
  
    b[0]='M'; // MultiwII mode
    if (f.ARMED) { // Motors running = flying
      if(!(f.ANGLE_MODE||f.HORIZON_MODE)){ //ACRO
        b[0]= 'x';
      }
      else if(f.GPS_HOME_MODE){ //RTH
        b[0]= 'w';
      }   
      else if(f.GPS_HOLD_MODE){//Position Hold
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
      b[5]=(180-heading)/2; // 1 unit = 2 degrees;
      b[6]=GPS_numSat;                                      
      i2c_rep_start(LED_RING_ADDRESS);
      for(uint8_t i=0;i<7;i++){
        i2c_write(b[i]);
      }
      i2c_stop();
    }
  #if defined (VBAT)
    if (vbat < conf.vbatlevel1_3s){ // Uh oh - battery low
      i2c_rep_start(LED_RING_ADDRESS);
      i2c_write('r');
      i2c_stop();   
    }
  # endif
  }
  
  void blinkLedRing() {
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
    LED_FLASHER_DDR |= (1<<LED_FLASHER_BIT);
    LED_FLASHER_PORT &= ~(1<<LED_FLASHER_BIT);
  }
  
  void led_flasher_set_sequence(uint8_t s) {
    led_flasher_sequence = s;
  }
  
  void inline switch_led_flasher(uint8_t on) {
    if (on) {
      LED_FLASHER_PORT |= (1<<LED_FLASHER_BIT);
    } else {
      LED_FLASHER_PORT &= ~(1<<LED_FLASHER_BIT);
    }
  }
  
  void auto_switch_led_flasher() {
    uint8_t seg = (currentTime/1000/125)%8;
    if (led_flasher_sequence & 1<<seg) {
      switch_led_flasher(!isBuzzerON());
    } else {
      switch_led_flasher(isBuzzerON());
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
    #else
    if (0) {
    #endif
      led_flasher_set_sequence(LED_FLASHER_SEQUENCE_MAX);
    } else {
      /* do we have a special sequence for armed copters? */
      #if defined(LED_FLASHER_SEQUENCE_ARMED)
      led_flasher_set_sequence(f.ARMED ? LED_FLASHER_SEQUENCE_ARMED : LED_FLASHER_SEQUENCE);
      #else
      /* Let's load the plain old boring sequence */
      led_flasher_set_sequence(LED_FLASHER_SEQUENCE);
      #endif
    }
  }
  
  #endif
  
  #if defined(LANDING_LIGHTS_DDR)
  void init_landing_lights(void) {
    LANDING_LIGHTS_DDR |= 1<<LANDING_LIGHTS_BIT;
  }
  
  void inline switch_landing_lights(uint8_t on) {
    if (on) {
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
    ) {
      switch_landing_lights(1);
    } else {
      switch_landing_lights(0);
    }
  }
#endif
