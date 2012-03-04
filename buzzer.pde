void buzzer(uint8_t warn_vbat){
  static uint16_t ontime, offtime, beepcount, repeat, repeatcounter;
  static uint32_t buzzerLastToggleTime;
  static uint8_t buzzerIsOn = 0,
                 activate_buzzer,
                 beeper_on,
                 i,
                 last_rcOptions[CHECKBOXITEMS], 
                 warn_noGPSfix = 0,
                 warn_failsafe = 0,
                 confirmation_flag = 0;         

  #define RCOPTIONSBEEP        //uncomment this if you want the buzzer to beep at any rcOptions change on channel Aux1 to Aux4
  
  //=====================  Beeps for changing rcoptions
  #if defined(RCOPTIONSBEEP)
    if (last_rcOptions[i] != rcOptions[i]){confirmation_flag = 1;}
    last_rcOptions[i] = rcOptions[i]; 
    i++;
    if(i >= CHECKBOXITEMS)i=0;
  #endif
  
  //=====================  Beeps for Beeperon via rcOptions
  if ( rcOptions[BOXBEEPERON] ){ // unconditional beeper on via AUXn switch 
    beeper_on = 1;
  }else{
    beeper_on = 0;
  }
  //=====================  Beeps for failsafe 
  #if defined(FAILSAFE)
    if ( failsafeCnt > (5*FAILSAVE_DELAY) && armed==1) {
      warn_failsafe = 1;    //set failsafe warning level to 1
      if (failsafeCnt > 5*(FAILSAVE_DELAY+FAILSAVE_OFF_DELAY)) warn_failsafe = 2;                            //find me signal after landing   
    }
    if ( failsafeCnt > (5*FAILSAVE_DELAY) && armed==0) warn_failsafe = 2;                  // tx turned off while motors are off: start "find me" signal
    if ( failsafeCnt == 0) warn_failsafe = 0;                                              // turn of alarm if TX is okay
  #endif
  
  //=====================  GPS fix notification handling 

  //if no fix and gps funtion is activated: do warning beeps.
  if ((GPSModeHome || GPSModeHold) && !GPS_fix){
    warn_noGPSfix = 1;    
  }else{
    warn_noGPSfix = 0;
  }
  repeat = 1;                                           // set repeat to default
  ontime = 100;                                           // set offtime to default
  
  //the order of the below is the priority from high to low, the last entry has the lowest priority, only one option can be active at the same time
  if (warn_failsafe == 2){activate_buzzer = 1; offtime = 2000;ontime=300;repeat = 1;}    //failsafe "find me" signal
  else if (warn_failsafe == 1){activate_buzzer = 1; offtime = 50;}                       //failsafe landing aktivetical level         
  else if (warn_noGPSfix == 1){activate_buzzer = 1; offtime = 10;}      
  else if (beeper_on == 1){activate_buzzer = 1; offtime = 50;}                           //beeperon
  else if (warn_vbat == 4){activate_buzzer = 1; offtime = 500; repeat = 3;}       
  else if (warn_vbat == 2){activate_buzzer = 1; offtime = 1000; repeat = 2;}      
  else if (warn_vbat == 1){activate_buzzer = 1; offtime = 2000;}                 
  else if (confirmation_flag > 0){activate_buzzer = 1; ontime =50; offtime = 50;}       //fast confirmation beep
  else{activate_buzzer=0;}
  
  if (activate_buzzer) { 
    if ( repeatcounter > 1 && !buzzerIsOn && (millis() >= (buzzerLastToggleTime + 80)) ){    // if the buzzer is off and there is a short pause neccessary (multipe buzzes)
      buzzerIsOn = 1;
      BUZZERPIN_ON;
      buzzerLastToggleTime=millis();      // save the time the buzer turned on
      repeatcounter--;
    } else if ( !buzzerIsOn && (millis() >= (buzzerLastToggleTime + offtime)) ) {	          // Buzzer is off and long pause time is up -> turn it on
      buzzerIsOn = 1;
      BUZZERPIN_ON;
      buzzerLastToggleTime=millis();      // save the time the buzer turned on
      repeatcounter = repeat;  //set the amount of repeats
    } else if (buzzerIsOn && (millis() >= buzzerLastToggleTime + ontime) ) {         //Buzzer is on and time is up -> turn it off
      buzzerIsOn = 0;
      BUZZERPIN_OFF;
      buzzerLastToggleTime=millis();                                 // save the time the buzer turned on
      if (confirmation_flag > 0)beepcount++;                   // only increment if confirmation beep, the rest is endless while the condition is given
    }
    if (beepcount >= confirmation_flag){      //confirmation flag is 0,1 or 2 
      beepcount = 0;            //reset the counter for the next time
      confirmation_flag = 0;    //reset the flag after all beeping is done
    }
  }else{                          //no beeping neccessary:reset everything (just in case)
    beepcount = 0;                //reset the counter for the next time 
    BUZZERPIN_OFF;              
    buzzerIsOn = 0; 
  }	
}
