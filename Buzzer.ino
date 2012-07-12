#if defined(BUZZER)
static uint8_t buzzerIsOn = 0,blinkdone =0;
static uint32_t buzzerLastToggleTime;
uint8_t isBuzzerON() { return buzzerIsOn; } // returns true while buzzer is buzzing; returns 0 for silent periods

void buzzer(uint8_t warn_vbat){
  static uint16_t ontime, offtime, beepcount, repeat, repeatcounter;
  static uint8_t beeperOnBox,
                 warn_noGPSfix = 0,
                 warn_failsafe = 0, 
                 warn_runtime = 0;  

  //===================== Beeps for changing rcOptions =====================
  #if defined(RCOPTIONSBEEP)
    static uint8_t i = 0;
    static uint8_t last_rcOptions[CHECKBOXITEMS];
    if (last_rcOptions[i] != rcOptions[i]){toggleBeep = 1;}
    last_rcOptions[i] = rcOptions[i]; 
    i++;
    if(i >= CHECKBOXITEMS)i=0;
  #endif  
  //=====================  BeeperOn via rcOptions =====================
  if ( rcOptions[BOXBEEPERON] ){ // unconditional beeper on via AUXn switch 
    beeperOnBox = 1;
  }else{
    beeperOnBox = 0;
  }
  //===================== Beeps for failsafe =====================
  #if defined(FAILSAFE)
    if ( failsafeCnt > (5*FAILSAVE_DELAY) && f.ARMED) {
      warn_failsafe = 1;                                                                   //set failsafe warning level to 1 while landing
      if (failsafeCnt > 5*(FAILSAVE_DELAY+FAILSAVE_OFF_DELAY)) warn_failsafe = 2;          //start "find me" signal after landing   
    }
    if ( failsafeCnt > (5*FAILSAVE_DELAY) && !f.ARMED) warn_failsafe = 2;                  // tx turned off while motors are off: start "find me" signal
    if ( failsafeCnt == 0) warn_failsafe = 0;                                              // turn off alarm if TX is okay
  #endif
  //===================== GPS fix notification handling =====================
  #if GPS
  if ((rcOptions[BOXGPSHOME] || rcOptions[BOXGPSHOLD]) && !f.GPS_FIX){    //if no fix and gps funtion is activated: do warning beeps.
    warn_noGPSfix = 1;    
  }else{
    warn_noGPSfix = 0;
  }
  #endif
  //===================== Runtime Warning =====================
  #if defined(ARMEDTIMEWARNING)
  if (armedTime >= ArmedTimeWarningMicroSeconds){
    warn_runtime = 1;
  }
  #endif
  //===================== Priority driven Handling =====================
  // beepcode(length1,length2,length3,pause)
  //None: 0, Short:50, Medium: 100, Long: 200, Double: 2000 .
  if (warn_failsafe == 2)      beep_code(200,0,0,2000);                 //failsafe "find me" signal
  else if (warn_failsafe == 1) beep_code(50,100,200,100);                 //failsafe landing active         
  else if (warn_noGPSfix == 1) beep_code(50,50,0,100);       
  else if (beeperOnBox == 1)   beep_code(50,50,50,100);                 //beeperon
  else if (warn_vbat == 4)     beep_code(50,50,100,2000);       
  else if (warn_vbat == 2)     beep_code(50,100,0,2000);       
  else if (warn_vbat == 1)     beep_code(100,0,0,2000);           
  else if (warn_runtime == 1 && f.ARMED == 1)beep_code(50,50,100,0); //Runtime warning            
  else if (toggleBeep > 0)     beep(50);                                   //fast confirmation beep
  else { 
    buzzerIsOn = 0;
    BUZZERPIN_OFF;
  }    
}

void beep_code(uint16_t first, uint16_t second, uint16_t third, uint16_t pause){
  uint16_t patternInt[4];
  static uint8_t icnt = 0;
  
  patternInt[0] = first; 
  patternInt[1] = second;
  patternInt[2] = third;
  patternInt[3] = pause;

  if(icnt <3 && patternInt[icnt]!=0){
    beep(patternInt[icnt]);
  }
  if (icnt >=3 && (buzzerLastToggleTime<millis()-patternInt[3]) ){
    icnt=0;
    toggleBeep =0;
  }
  if (blinkdone == 1 || patternInt[icnt]==0){
    icnt++;
    blinkdone=0;      
    buzzerIsOn = 0;
    BUZZERPIN_OFF;
  }
}

void beep( uint16_t pulse){  
  if ( !buzzerIsOn && (millis() >= (buzzerLastToggleTime + 50)) ) {	          // Buzzer is off and long pause time is up -> turn it on
    buzzerIsOn = 1;
    BUZZERPIN_ON;
    buzzerLastToggleTime=millis();      // save the time the buzer turned on
  } else if (buzzerIsOn && (millis() >= buzzerLastToggleTime + pulse) ) {         //Buzzer is on and time is up -> turn it off
    buzzerIsOn = 0;
    BUZZERPIN_OFF;
    buzzerLastToggleTime=millis();    
    if (toggleBeep >0) toggleBeep--;    
    blinkdone =1;
  }
} 

#endif
