
#if defined(BI) || defined(TRI) || defined(SERVO_TILT) || defined(GIMBAL) || defined(FLYING_WING) || defined(CAMTRIG)
  #define SERVO
#endif

#if defined(GIMBAL)
  #define NUMBER_MOTOR 0
#elif defined(FLYING_WING)
  #define NUMBER_MOTOR 1
#elif defined(BI)
  #define NUMBER_MOTOR 2
#elif defined(TRI)
  #define NUMBER_MOTOR 3
#elif defined(QUADP) || defined(QUADX) || defined(Y4)
  #define NUMBER_MOTOR 4
#elif defined(Y6) || defined(HEX6) || defined(HEX6X)
  #define NUMBER_MOTOR 6
#elif defined(OCTOX8) || defined(OCTOFLATP) || defined(OCTOFLATX)
  #define NUMBER_MOTOR 8
#endif

uint8_t PWM_PIN[8] = {MOTOR_ORDER};
volatile uint8_t atomicServo[4] = {125,125,125,125};

//for HEX Y6 and HEX6/HEX6X flat and for promini
volatile uint8_t atomicPWM_PIN5_lowState;
volatile uint8_t atomicPWM_PIN5_highState;
volatile uint8_t atomicPWM_PIN6_lowState;
volatile uint8_t atomicPWM_PIN6_highState;


void writeServos() {
  #if defined(SERVO)
    atomicServo[0] = (servo[0]-1000)/4;
    atomicServo[1] = (servo[1]-1000)/4;
    atomicServo[2] = (servo[2]-1000)/4;
    atomicServo[3] = (servo[3]-1000)/4;
  #endif
}

void writeMotors() { // [1000;2000] => [125;250]
  #if defined(MEGA)
    for(uint8_t i=0;i<NUMBER_MOTOR;i++)
      analogWrite(PWM_PIN[i], motor[i]>>3);
  #else
    for(uint8_t i=0;i<min(NUMBER_MOTOR,4);i++)
      analogWrite(PWM_PIN[i], motor[i]>>3);
    #if (NUMBER_MOTOR == 6)
      atomicPWM_PIN5_highState = motor[5]/8;
      atomicPWM_PIN5_lowState = 255-atomicPWM_PIN5_highState;
      atomicPWM_PIN6_highState = motor[4]/8;
      atomicPWM_PIN6_lowState = 255-atomicPWM_PIN6_highState;
    #endif
  #endif
}

void writeAllMotors(int16_t mc) {   // Sends commands to all motors
  for (uint8_t i =0;i<NUMBER_MOTOR;i++)
    motor[i]=mc;
  writeMotors();
}

#if defined(LOG_VALUES) || (POWERMETER == 1)
void logMotorsPower() {
  uint32_t amp;
  /* true cubic function; when divided by vbat_max=126 (12.6V) for 3 cell battery this gives maximum value of ~ 1000 */
  const uint32_t amperes[64] =   {0,4,13,31,60,104,165,246,350,481,640,831,1056,1319,1622,1969,2361,2803,3297,3845,4451,5118,5848,6645,
	                             7510,8448,9461,10551,11723,12978,14319,15750,17273,18892,20608,22425,24346,26374,28512,30762,33127,35611,
	                             38215,40944,43799,46785,49903,53156,56548,60081,63759,67583,71558,75685,79968,84410,89013,93781,98716,103821,
	                             109099,114553,120186,126000 };
  if (vbat) { // by all means - must avoid division by zero 
    for (uint8_t i =0;i<NUMBER_MOTOR;i++) {
      amp = amperes[(motor[i] - 1000)>>4] / vbat; // range mapped from [1000:2000] => [0:1000]; then break that up into 64 ranges; lookup amp
      #ifdef LOG_VALUES
         pMeter[i]+= amp; // sum up over time the mapped ESC input 
      #endif
      #if (POWERMETER == 1)
         pMeter[PMOTOR_SUM]+= amp; // total sum over all motors
      #endif
    }
  }
}
#endif

void initOutput() {
  for(uint8_t i=0;i<NUMBER_MOTOR;i++)
    pinMode(PWM_PIN[i],OUTPUT);
  writeAllMotors(1000);
  delay(300);
  #if defined(SERVO)
    initializeServo();
  #elif (NUMBER_MOTOR == 6) && defined(PROMINI)
    initializeSoftPWM();
    #if defined(A0_A1_PIN_HEX)
      pinMode(5,INPUT);pinMode(6,INPUT);     // we reactivate the INPUT affectation for these two PINs
      pinMode(A0,OUTPUT);pinMode(A1,OUTPUT);
    #endif
  #endif
}

#if defined(SERVO)
void initializeServo() {
  #if defined(TRI)
    DIGITAL_SERVO_TRI_PINMODE
  #endif
  #if defined(SERVO_TILT) || defined(GIMBAL) || defined(FLYING_WING)
    DIGITAL_TILT_ROLL_PINMODE
    DIGITAL_TILT_PITCH_PINMODE
  #endif
  #if defined(CAMTRIG)
    DIGITAL_CAM_PINMODE
  #endif
  #if defined(BI)
    DIGITAL_SERVO_TRI_PINMODE
    DIGITAL_BI_LEFT_PINMODE
  #endif
  TCCR0A = 0; // normal counting mode
  TIMSK0 |= (1<<OCIE0A); // Enable CTC interrupt
}

// ****servo yaw with a 50Hz refresh rate****
// prescaler is set by default to 64 on Timer0
// Duemilanove : 16MHz / 64 => 4 us
// 256 steps = 1 counter cycle = 1024 us
// algorithm strategy:
// pulse high servo 0 -> do nothing for 1000 us -> do nothing for [0 to 1000] us -> pulse down servo 0
// pulse high servo 1 -> do nothing for 1000 us -> do nothing for [0 to 1000] us -> pulse down servo 1
// pulse high servo 2 -> do nothing for 1000 us -> do nothing for [0 to 1000] us -> pulse down servo 2
// pulse high servo 3 -> do nothing for 1000 us -> do nothing for [0 to 1000] us -> pulse down servo 3
// do nothing for 14 x 1000 us
ISR(TIMER0_COMPA_vect) {
  static uint8_t state = 0;
  static uint8_t count;
  if (state == 0) {
    //http://billgrundmann.wordpress.com/2009/03/03/to-use-or-not-use-writedigital/
    #if defined(TRI) || defined (BI)
      DIGITAL_SERVO_TRI_HIGH
    #endif
    OCR0A+= 250; // 1000 us
    state++ ;
  } else if (state == 1) {
    OCR0A+= atomicServo[0]; // 1000 + [0-1020] us
    state++;
  } else if (state == 2) {
    #if defined(TRI) || defined (BI)
      DIGITAL_SERVO_TRI_LOW
    #endif
    #if defined(BI)
      DIGITAL_BI_LEFT_HIGH
    #endif
    #if defined(SERVO_TILT) || defined(GIMBAL) || defined(FLYING_WING)
      DIGITAL_TILT_PITCH_HIGH
    #endif
    OCR0A+= 250; // 1000 us
    state++;
  } else if (state == 3) {
    OCR0A+= atomicServo[1]; // 1000 + [0-1020] us
    state++;
  } else if (state == 4) {
    #if defined(SERVO_TILT) || defined(GIMBAL) || defined(FLYING_WING)
      DIGITAL_TILT_PITCH_LOW
      DIGITAL_TILT_ROLL_HIGH
    #endif
    #if defined(BI)
      DIGITAL_BI_LEFT_LOW
    #endif
    state++;
    OCR0A+= 250; // 1000 us
  } else if (state == 5) {
    OCR0A+= atomicServo[2]; // 1000 + [0-1020] us
    state++;
  } else if (state == 6) {
    #if defined(SERVO_TILT) || defined(GIMBAL) || defined(FLYING_WING)
      DIGITAL_TILT_ROLL_LOW
    #endif
    #if defined(CAMTRIG)
      DIGITAL_CAM_HIGH
    #endif
    state++;
    OCR0A+= 250; // 1000 us
  } else if (state == 7) {
    OCR0A+= atomicServo[3]; // 1000 + [0-1020] us
    state++;
  } else if (state == 8) {
    #if defined(CAMTRIG)
      DIGITAL_CAM_LOW
    #endif
    count = 10; // 12 x 1000 us
    state++;
    OCR0A+= 250; // 1000 us
  } else if (state == 9) {
    if (count > 0) count--;
    else state = 0;
    OCR0A+= 250;
  }
}
#endif

#if (NUMBER_MOTOR == 6) && defined(PROMINI)
void initializeSoftPWM() {
  TCCR0A = 0; // normal counting mode
  TIMSK0 |= (1<<OCIE0A); // Enable CTC interrupt
  TIMSK0 |= (1<<OCIE0B);
}

ISR(TIMER0_COMPA_vect) {
  static uint8_t state = 0;
  if (state == 0) {
    #if not defined(A0_A1_PIN_HEX)
      PORTD |= 1<<5; //digital PIN 5 high
    #else
      PORTC |= 1<<0;//PIN A0
    #endif
    OCR0A+= atomicPWM_PIN5_highState; //250 x 4 microsecons = 1ms
    state = 1;
  } else if (state == 1) {
    OCR0A+= atomicPWM_PIN5_highState;
    state = 2;
  } else if (state == 2) {
    #if not defined(A0_A1_PIN_HEX)
      PORTD &= ~(1<<5); //digital PIN 5 low
    #else
      PORTC &= ~(1<<0);
    #endif 
    OCR0A+= atomicPWM_PIN5_lowState;
    state = 0;
  }
}

ISR(TIMER0_COMPB_vect) { //the same with digital PIN 6 and OCR0B counter
  static uint8_t state = 0;
  if (state == 0) {
    #if not defined(A0_A1_PIN_HEX)
      PORTD |= 1<<6;
    #else
      PORTC |= 1<<1;//PIN A1
    #endif
    OCR0B+= atomicPWM_PIN6_highState;state = 1;
  } else if (state == 1) {
    OCR0B+= atomicPWM_PIN6_highState;state = 2;
  } else if (state == 2) {
    #if not defined(A0_A1_PIN_HEX)
      PORTD &= ~(1<<6);
    #else
      PORTC &= ~(1<<1);
    #endif
    OCR0B+= atomicPWM_PIN6_lowState;state = 0;
  }
}
#endif


void mixTable() {
  int16_t maxMotor;
  uint8_t i,axis;
  static uint8_t camCycle = 0;
  static uint8_t camState = 0;
  static uint32_t camTime = 0;
  
  #define PIDMIX(X,Y,Z) rcCommand[THROTTLE] + axisPID[ROLL]*X + axisPID[PITCH]*Y + YAW_DIRECTION * axisPID[YAW]*Z

  #if NUMBER_MOTOR > 3
    //prevent "yaw jump" during yaw correction
    axisPID[YAW] = constrain(axisPID[YAW],-100-abs(rcCommand[YAW]),+100+abs(rcCommand[YAW]));
  #endif
  #ifdef BI
    motor[0] = PIDMIX(+1, 0, 0); //LEFT
    motor[1] = PIDMIX(-1, 0, 0); //RIGHT        
    servo[0]  = constrain(1500 + YAW_DIRECTION * (axisPID[YAW] + axisPID[PITCH]), 1020, 2000); //LEFT
    servo[1]  = constrain(1500 + YAW_DIRECTION * (axisPID[YAW] - axisPID[PITCH]), 1020, 2000); //RIGHT
  #endif
  #ifdef TRI
    motor[0] = PIDMIX( 0,+4/3, 0); //REAR
    motor[1] = PIDMIX(-1,-2/3, 0); //RIGHT
    motor[2] = PIDMIX(+1,-2/3, 0); //LEFT
    servo[0] = constrain(TRI_YAW_MIDDLE + YAW_DIRECTION * axisPID[YAW], TRI_YAW_CONSTRAINT_MIN, TRI_YAW_CONSTRAINT_MAX); //REAR
  #endif
  #ifdef QUADP
    motor[0] = PIDMIX( 0,+1,-1); //REAR
    motor[1] = PIDMIX(-1, 0,+1); //RIGHT
    motor[2] = PIDMIX(+1, 0,+1); //LEFT
    motor[3] = PIDMIX( 0,-1,-1); //FRONT
  #endif
  #ifdef QUADX
    motor[0] = PIDMIX(-1,+1,-1); //REAR_R
    motor[1] = PIDMIX(-1,-1,+1); //FRONT_R
    motor[2] = PIDMIX(+1,+1,+1); //REAR_L
    motor[3] = PIDMIX(+1,-1,-1); //FRONT_L
  #endif
  #ifdef Y4
    motor[0] = PIDMIX(+0,+1,-1);   //REAR_1 CW
    motor[1] = PIDMIX(-1,-1, 0); //FRONT_R CCW
    motor[2] = PIDMIX(+0,+1,+1);   //REAR_2 CCW
    motor[3] = PIDMIX(+1,-1, 0); //FRONT_L CW
  #endif
  #ifdef Y6
    motor[0] = PIDMIX(+0,+4/3,+1); //REAR
    motor[1] = PIDMIX(-1,-2/3,-1); //RIGHT
    motor[2] = PIDMIX(+1,-2/3,-1); //LEFT
    motor[3] = PIDMIX(+0,+4/3,-1); //UNDER_REAR
    motor[4] = PIDMIX(-1,-2/3,+1); //UNDER_RIGHT
    motor[5] = PIDMIX(+1,-2/3,+1); //UNDER_LEFT    
  #endif
  #ifdef HEX6
    motor[0] = PIDMIX(-1/2,+1/2,+1); //REAR_R
    motor[1] = PIDMIX(-1/2,-1/2,-1); //FRONT_R
    motor[2] = PIDMIX(+1/2,+1/2,+1); //REAR_L
    motor[3] = PIDMIX(+1/2,-1/2,-1); //FRONT_L
    motor[4] = PIDMIX(+0  ,-1  ,+1); //FRONT
    motor[5] = PIDMIX(+0  ,+1  ,-1); //REAR
  #endif
  #ifdef HEX6X
    motor[0] = PIDMIX(-1/2,+1/2,+1); //REAR_R
    motor[1] = PIDMIX(-1/2,-1/2,+1); //FRONT_R
    motor[2] = PIDMIX(+1/2,+1/2,-1); //REAR_L
    motor[3] = PIDMIX(+1/2,-1/2,-1); //FRONT_L
    motor[4] = PIDMIX(-1  ,+0  ,-1); //RIGHT
    motor[5] = PIDMIX(+1  ,+0  ,+1); //LEFT
  #endif
  #ifdef OCTOX8
    motor[0] = PIDMIX(-1,+1,-1); //REAR_R
    motor[1] = PIDMIX(-1,-1,+1); //FRONT_R
    motor[2] = PIDMIX(+1,+1,+1); //REAR_L
    motor[3] = PIDMIX(+1,-1,-1); //FRONT_L
    motor[4] = PIDMIX(-1,+1,+1); //UNDER_REAR_R
    motor[5] = PIDMIX(-1,-1,-1); //UNDER_FRONT_R
    motor[6] = PIDMIX(+1,+1,-1); //UNDER_REAR_L
    motor[7] = PIDMIX(+1,-1,+1); //UNDER_FRONT_L
  #endif
  #ifdef OCTOFLATP
    motor[0] = PIDMIX(+7/10,-7/10,+1); //FRONT_L
    motor[1] = PIDMIX(-7/10,-7/10,+1); //FRONT_R
    motor[2] = PIDMIX(-7/10,+7/10,+1); //REAR_R
    motor[3] = PIDMIX(+7/10,+7/10,+1); //REAR_L
    motor[4] = PIDMIX(+0   ,-1   ,-1); //FRONT
    motor[5] = PIDMIX(-1   ,+0   ,-1); //RIGHT
    motor[6] = PIDMIX(+0   ,+1   ,-1); //REAR
    motor[7] = PIDMIX(+1   ,+0   ,-1); //LEFT 
  #endif
  #ifdef OCTOFLATX
    motor[0] = PIDMIX(+1  ,-1/2,+1); //MIDFRONT_L
    motor[1] = PIDMIX(-1/2,-1  ,+1); //FRONT_R
    motor[2] = PIDMIX(-1  ,+1/2,+1); //MIDREAR_R
    motor[3] = PIDMIX(+1/2,+1  ,+1); //REAR_L
    motor[4] = PIDMIX(+1/2,-1  ,-1); //FRONT_L
    motor[5] = PIDMIX(-1  ,-1/2,-1); //MIDFRONT_R
    motor[6] = PIDMIX(-1/2,+1  ,-1); //REAR_R
    motor[7] = PIDMIX(+1  ,+1/2,-1); //MIDREAR_L 
  #endif

  #ifdef SERVO_TILT
    if (rcOptions & activate[BOXCAMSTAB] ) {
      servo[1] = constrain(TILT_PITCH_MIDDLE + TILT_PITCH_PROP * angle[PITCH] /16 + rcData[CAMPITCH]-1500 , TILT_PITCH_MIN, TILT_PITCH_MAX);
      servo[2] = constrain(TILT_ROLL_MIDDLE  + TILT_ROLL_PROP  * angle[ROLL]  /16 + rcData[CAMROLL]-1500, TILT_ROLL_MIN, TILT_ROLL_MAX);
    } else {
      servo[1] = constrain(TILT_PITCH_MIDDLE  + rcData[CAMPITCH]-1500 , TILT_PITCH_MIN, TILT_PITCH_MAX);
      servo[2] = constrain(TILT_ROLL_MIDDLE   + rcData[CAMROLL]-1500,  TILT_ROLL_MIN, TILT_ROLL_MAX);
    }
  #endif
  #ifdef GIMBAL
    servo[1] = constrain(TILT_PITCH_MIDDLE + TILT_PITCH_PROP * angle[PITCH] /16 + rcCommand[PITCH], TILT_PITCH_MIN, TILT_PITCH_MAX);
    servo[2] = constrain(TILT_ROLL_MIDDLE + TILT_ROLL_PROP   * angle[ROLL]  /16 + rcCommand[ROLL], TILT_ROLL_MIN, TILT_ROLL_MAX);
  #endif
  #ifdef FLYING_WING
   motor[0] = rcCommand[THROTTLE];
    //if (passthroughMode) {// use raw stick values to drive output 
    // follow aux1 as being three way switch **NOTE: better to implement via check boxes in GUI 
    if (rcData[AUX1]<1300) { // passthrough

       servo[1]  = constrain(WING_LEFT_MID  + PITCH_DIRECTION_L * (rcData[PITCH]-MIDRC) + ROLL_DIRECTION_L * (rcData[ROLL]-MIDRC), WING_LEFT_MIN,  WING_LEFT_MAX); //LEFT
       servo[2]  = constrain(WING_RIGHT_MID + PITCH_DIRECTION_R * (rcData[PITCH]-MIDRC) + ROLL_DIRECTION_R * (rcData[ROLL]-MIDRC), WING_RIGHT_MIN, WING_RIGHT_MAX); //RIGHT
    } else { // use sensors to correct (gyro only or gyro+acc according to aux1/aux2 configuration
       servo[1]  = constrain(WING_LEFT_MID  + PITCH_DIRECTION_L * axisPID[PITCH]        + ROLL_DIRECTION_L * axisPID[ROLL], WING_LEFT_MIN,  WING_LEFT_MAX); //LEFT
       servo[2]  = constrain(WING_RIGHT_MID + PITCH_DIRECTION_R * axisPID[PITCH]        + ROLL_DIRECTION_R * axisPID[ROLL], WING_RIGHT_MIN, WING_RIGHT_MAX); //RIGHT
    }
  #endif

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

  maxMotor=motor[0];
  for(i=1;i< NUMBER_MOTOR;i++)
    if (motor[i]>maxMotor) maxMotor=motor[i];
  for (i = 0; i < NUMBER_MOTOR; i++) {
    if (maxMotor > MAXTHROTTLE) // this is a way to still have good gyro corrections if at least one motor reaches its max.
      motor[i] -= maxMotor - MAXTHROTTLE;
    motor[i] = constrain(motor[i], MINTHROTTLE, MAXTHROTTLE);    
    if ((rcData[THROTTLE]) < MINCHECK)
      #ifndef MOTOR_STOP
        motor[i] = MINTHROTTLE;
      #else
        motor[i] = MINCOMMAND;
      #endif
    if (armed == 0)
      motor[i] = MINCOMMAND;
  }

  #if defined(LOG_VALUES) || (POWERMETER == 1)
    logMotorsPower();
  #endif
}

