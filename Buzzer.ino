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
  //D: Double, L: Long, M: Middle, S: Short, N: None
  if (warn_failsafe == 2)      beep_code('L','N','N','D');                 //failsafe "find me" signal
  else if (warn_failsafe == 1) beep_code('S','M','L','M');                 //failsafe landing active         
  else if (warn_noGPSfix == 1) beep_code('S','S','N','M');       
  else if (beeperOnBox == 1)   beep_code('S','S','S','M');                 //beeperon
  else if (warn_vbat == 4)     beep_code('S','M','M','D');       
  else if (warn_vbat == 2)     beep_code('S','S','M','D');       
  else if (warn_vbat == 1)     beep_code('S','M','N','D');           
  else if (warn_runtime == 1 && f.ARMED == 1)beep_code('S','S','M','N'); //Runtime warning            
  else if (toggleBeep > 0)     beep(50);                                   //fast confirmation beep
  else { 
    buzzerIsOn = 0;
    BUZZERPIN_OFF;
  }    
}

void beep_code(char first, char second, char third, char pause){
  char patternChar[4];
  uint16_t patternInt[4];
  static uint8_t icnt;
  
  patternChar[0] = first; 
  patternChar[1] = second;
  patternChar[2] = third;
  patternChar[3] = pause;
  
  for (int i = 0; i < 4; i = i+1) {
    switch(patternChar[i]) {
    case 'S':
      patternInt[i] = 50; 
      break;
    case 'M': 
      patternInt[i] = 100; 
      break;
    case 'L': 
      patternInt[i] = 200; 
      break;
    case 'D': 
      patternInt[i] = 2000; 
      break;
    case 'N': 
      patternInt[i] = 0; 
      break;
    default:
      patternInt[i] = 50; 
      break;
    }
  }
  
  if(icnt <3 && patternInt[icnt]!=0){
    beep(patternInt[icnt]);
  }
  if (blinkdone == 1 || patternInt[icnt]==0){
    icnt++;
    blinkdone=0;      
    buzzerIsOn = 0;
    BUZZERPIN_OFF;
  }
  if (icnt >=3 && (buzzerLastToggleTime<millis()-patternInt[3]) ){
    icnt=0;
    toggleBeep =0;
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
