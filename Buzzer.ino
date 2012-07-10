#if defined(BUZZER)
static uint8_t buzzerIsOn = 0;

uint8_t isBuzzerON() { return buzzerIsOn; } // returns true while buzzer is buzzing; returns 0 for silent periods

void buzzer(uint8_t warn_vbat){
  static uint16_t ontime, offtime, beepcount, repeat, repeatcounter;
  static uint32_t buzzerLastToggleTime;
  static uint8_t activateBuzzer,
                 beeperOnBox,
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
  
  if (warn_failsafe == 2)      letsbeep(1, 200, 2000, 1);     //failsafe "find me" signal
  else if (warn_failsafe == 1) letsbeep(1, 100, 50, 1);                        //failsafe landing active         
  else if (warn_noGPSfix == 1) letsbeep(1, 100, 50, 1);       
  else if (beeperOnBox == 1)   letsbeep(1, 100, 50, 1);                            //beeperon
  else if (warn_vbat == 4)     letsbeep(1, 100, 500, 3);       
  else if (warn_vbat == 2)     letsbeep(1, 100, 1000, 2);       
  else if (warn_vbat == 1)     letsbeep(1, 100, 2000, 1);           
  else if (warn_runtime == 1 && f.ARMED == 1)letsbeep(1, 50, 50, 1);                      //Runtime warning            
  else if (toggleBeep > 0)     letsbeep(1, 50, 50, 1);        //fast confirmation beep
  else                         letsbeep(0, 100, 1000, 1);    
 
}

void letsbeep(uint8_t activateBuzzer, uint16_t pulse, uint16_t pause, uint16_t repeat){  
  static uint16_t beepcount, repeatcounter;
  static uint32_t buzzerLastToggleTime;
  
  if (activateBuzzer) { 
    if ( repeatcounter > 1 && !buzzerIsOn && (millis() >= (buzzerLastToggleTime + 80)) ){    // if the buzzer is off and there is a short pause neccessary (multipe buzzes)
      buzzerIsOn = 1;
      BUZZERPIN_ON;
      buzzerLastToggleTime=millis();      // save the time the buzzer turned on
      repeatcounter--;
    } else if ( !buzzerIsOn && (millis() >= (buzzerLastToggleTime + pause)) ) {	          // Buzzer is off and long pause time is up -> turn it on
      buzzerIsOn = 1;
      BUZZERPIN_ON;
      buzzerLastToggleTime=millis();      // save the time the buzer turned on
      repeatcounter = repeat;  //set the amount of repeats after the pause
    } else if (buzzerIsOn && (millis() >= buzzerLastToggleTime + pulse) ) {         //Buzzer is on and time is up -> turn it off
      buzzerIsOn = 0;
      BUZZERPIN_OFF;
      buzzerLastToggleTime=millis();                                 // save the time the buzer turned on
      if (toggleBeep > 0)beepcount++;                   // only increment if confirmation beep, the rest is endless while the condition is given
    }
    if (beepcount >= toggleBeep){      //confirmation flag is 0,1 or 2 
      beepcount = 0;            //reset the counter for the next time
      toggleBeep = 0;    //reset the flag after all beeping is done
    }
  }else{                          //no beeping neccessary:reset everything (just in case)
    beepcount = 0;                //reset the counter for the next time 
    BUZZERPIN_OFF;              
    buzzerIsOn = 0; 
  }   
}

#endif
