#if defined(LED_RING)

#define LED_RING_ADDRESS 0x6D //7 bits

void i2CLedRingState() {
  uint8_t b[10];

  b[0]='M'; // MultiwII mode
  if (f.ARMED) { // Motors running = flying
    if(!f.ACC_MODE){ //ACRO
      b[0]= 'x';
    }
    else if(f.GPS_HOME_MODE){ //RTH
      b[0]= 'w';
    }   
    else if(f.GPS_HOLD_MODE){//Position Hold
      b[0]= 'v';
    } 
    else {
      b[0]= 'u'; // stable mode
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
    if (f.ACC_MODE) b[1]=1; 
    else b[1]= 0;                  
    if (f.BARO_MODE) b[2]=1; 
    else b[2]= 0;                  
    if (f.MAG_MODE) b[3]=1; 
    else b[3]= 0;                  
    if (rcOptions[BOXGPSHOME]) b[4]=2;
    else if (rcOptions[BOXGPSHOLD]) b[4]=1;
    else b[4]=0;
    b[5]=(180-heading)/2; // 1 unit = 2 degrees;
    b[6]=GPS_numSat;                                      
    i2c_rep_start(LED_RING_ADDRESS);
    for(uint8_t i=0;i<7;i++){
      i2c_write(b[i]);
    }
    i2c_stop();
  }
#if defined (VBAT)
  if (vbat<VBATLEVEL1_3S){ // Uh oh - battery low
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
