
#if defined(BI) || defined(TRI) || defined(SERVO_TILT) || defined(GIMBAL) || defined(FLYING_WING) || defined(CAMTRIG)
  #define SERVO
#endif

#if defined(GIMBAL) || defined(FLYING_WING)
  #define NUMBER_MOTOR 0
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
  const uint32_t amperes[16] =   {31, 246, 831, 1969, 3845, 6645, 10551, 15750, 22425, 30762, 40944, 53156, 67583, 84410, 103821, 126000 };

  if (vbat) { // by all means - must avoid division by zero 
    for (uint8_t i =0;i<NUMBER_MOTOR;i++) {
      amp = amperes[(motor[i] - 1000)>>6] / vbat; // range mapped from [1000:2000] => [0:1000]; then break that up into 16 ranges; lookup amp
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
    PORTD |= 1<<5; //digital PIN 5 high
    OCR0A+= atomicPWM_PIN5_highState; //250 x 4 microsecons = 1ms
    state = 1;
  } else if (state == 1) {
    OCR0A+= atomicPWM_PIN5_highState;
    state = 2;
  } else if (state == 2) {
    PORTD &= ~(1<<5); //digital PIN 5 low
    OCR0A+= atomicPWM_PIN5_lowState;
    state = 0;
  }
}

ISR(TIMER0_COMPB_vect) { //the same with digital PIN 6 and OCR0B counter
  static uint8_t state = 0;
  if (state == 0) {
    PORTD |= 1<<6;OCR0B+= atomicPWM_PIN6_highState;state = 1;
  } else if (state == 1) {
    OCR0B+= atomicPWM_PIN6_highState;state = 2;
  } else if (state == 2) {
    PORTD &= ~(1<<6);OCR0B+= atomicPWM_PIN6_lowState;state = 0;
  }
}
#endif


void mixTable() {
  int16_t maxMotor;
  uint8_t i;

//  int16_t tmp = 2*max(max(abs(rcCommand[ROLL]),abs(rcCommand[PITCH])),50);
//  #define PIDMIX(X,A,Y,B,Z) rcCommand[THROTTLE] +constrain(X axisPID[ROLL]*A  Y axisPID[PITCH]*B,-tmp,+tmp) Z YAW_DIRECTION * axisPID[YAW];
  #define PIDMIX(X,A,Y,B,Z) rcCommand[THROTTLE] + X axisPID[ROLL]*A  Y axisPID[PITCH]*B Z YAW_DIRECTION * axisPID[YAW];

  #if NUMBER_MOTOR > 3
    //prevent "yaw jump" during yaw correction
    axisPID[YAW] = constrain(axisPID[YAW],-100-abs(rcCommand[YAW]),+100+abs(rcCommand[YAW]));
  #endif
  #ifdef BI
    motor[0] = PIDMIX(+,1,+,0,+0*); //LEFT
    motor[1] = PIDMIX(-,1,+,0,+0*); //RIGHT        
    servo[0]  = constrain(1500 + YAW_DIRECTION * (axisPID[YAW] + axisPID[PITCH]), 1020, 2000); //LEFT
    servo[1]  = constrain(1500 + YAW_DIRECTION * (axisPID[YAW] - axisPID[PITCH]), 1020, 2000); //RIGHT
  #endif
  #ifdef TRI
    motor[0] = PIDMIX(+,0,+,4/6,+0*); //REAR
    motor[1] = PIDMIX(-,1,-,1/6,+0*); //RIGHT
    motor[2] = PIDMIX(+,1,-,1/6,+0*); //LEFT
    servo[0] = constrain(TRI_YAW_MIDDLE + YAW_DIRECTION * axisPID[YAW], TRI_YAW_CONSTRAINT_MIN, TRI_YAW_CONSTRAINT_MAX); //REAR
  #endif
  #ifdef QUADP
    motor[0] = PIDMIX(+,0,+,1,-); //REAR
    motor[1] = PIDMIX(-,1,+,0,+); //RIGHT
    motor[2] = PIDMIX(+,1,+,0,+); //LEFT
    motor[3] = PIDMIX(+,0,-,1,-); //FRONT
  #endif
  #ifdef QUADX
    motor[0] = PIDMIX(-,1,+,1,-); //REAR_R
    motor[1] = PIDMIX(-,1,-,1,+); //FRONT_R
    motor[2] = PIDMIX(+,1,+,1,+); //REAR_L
    motor[3] = PIDMIX(+,1,-,1,-); //FRONT_L
  #endif
  #ifdef Y4
    motor[0] = PIDMIX(+,0,+,1,-);   //REAR_1 CW
    motor[1] = PIDMIX(-,1,-,1,+0*); //FRONT_R CCW
    motor[2] = PIDMIX(+,0,+,1,+);   //REAR_2 CCW
    motor[3] = PIDMIX(+,1,-,1,+0*); //FRONT_L CW
  #endif
  #ifdef Y6
    motor[0] = PIDMIX(+,0,+,4/3,+); //REAR
    motor[1] = PIDMIX(-,1,-,2/3,-); //RIGHT
    motor[2] = PIDMIX(+,1,-,2/3,-); //LEFT
    motor[3] = PIDMIX(+,0,+,4/3,-); //UNDER_REAR
    motor[4] = PIDMIX(-,1,-,2/3,+); //UNDER_RIGHT
    motor[5] = PIDMIX(+,1,-,2/3,+); //UNDER_LEFT
  #endif
  #ifdef HEX6
    motor[0] = PIDMIX(-,1/2,+,1/2,+); //REAR_R
    motor[1] = PIDMIX(-,1/2,-,1/2,-); //FRONT_R
    motor[2] = PIDMIX(+,1/2,+,1/2,+); //REAR_L
    motor[3] = PIDMIX(+,1/2,-,1/2,-); //FRONT_L
    motor[4] = PIDMIX(+,0  ,-,1  ,+); //FRONT
    motor[5] = PIDMIX(+,0  ,+,1  ,-); //REAR
  #endif
  #ifdef HEX6X
    motor[0] = PIDMIX(-,1/2,+,1/2,+); //REAR_R
    motor[1] = PIDMIX(-,1/2,-,1/2,+); //FRONT_R
    motor[2] = PIDMIX(+,1/2,+,1/2,-); //REAR_L
    motor[3] = PIDMIX(+,1/2,-,1/2,-); //FRONT_L
    motor[4] = PIDMIX(-,1  ,+,0  ,-); //RIGHT
    motor[5] = PIDMIX(+,1  ,+,0  ,+); //LEFT
  #endif
  #ifdef OCTOX8
    motor[0] = PIDMIX(-,1,+,1,-); //REAR_R
    motor[1] = PIDMIX(-,1,-,1,+); //FRONT_R
    motor[2] = PIDMIX(+,1,+,1,+); //REAR_L
    motor[3] = PIDMIX(+,1,-,1,-); //FRONT_L
    motor[4] = PIDMIX(-,1,+,1,+); //UNDER_REAR_R
    motor[5] = PIDMIX(-,1,-,1,-); //UNDER_FRONT_R
    motor[6] = PIDMIX(+,1,+,1,-); //UNDER_REAR_L
    motor[7] = PIDMIX(+,1,-,1,+); //UNDER_FRONT_L
  #endif
  #ifdef OCTOFLATP
    motor[0] = PIDMIX(+,7/10,-,7/10,+); //FRONT_L
    motor[1] = PIDMIX(-,7/10,-,7/10,+); //FRONT_R
    motor[2] = PIDMIX(-,7/10,+,7/10,+); //REAR_R
    motor[3] = PIDMIX(+,7/10,+,7/10,+); //REAR_L
    motor[4] = PIDMIX(+,0   ,-,1   ,-); //FRONT
    motor[5] = PIDMIX(-,1   ,+,0   ,-); //RIGHT
    motor[6] = PIDMIX(+,0   ,+,1   ,-); //REAR
    motor[7] = PIDMIX(-,1   ,+,0   ,-); //LEFT 
  #endif
  #ifdef OCTOFLATX
    motor[0] = PIDMIX(+,1  ,-,1/2,+); //MIDFRONT_L
    motor[1] = PIDMIX(-,1/2,-,1  ,+); //FRONT_R
    motor[2] = PIDMIX(-,1  ,+,1/2,+); //MIDREAR_R
    motor[3] = PIDMIX(+,1/2,+,1  ,+); //REAR_L
    motor[4] = PIDMIX(+,1/2,-,1  ,-); //FRONT_L
    motor[5] = PIDMIX(-,1  ,-,1/2,-); //MIDFRONT_R
    motor[6] = PIDMIX(-,1/2,+,1  ,-); //REAR_R
    motor[7] = PIDMIX(+,1  ,+,1/2,-); //MIDREAR_L 
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
    servo[1]  = constrain(1500 + axisPID[PITCH] - axisPID[ROLL], 1020, 2000); //LEFT the direction of the 2 servo can be changed here: invert the sign before axisPID
    servo[2]  = constrain(1500 + axisPID[PITCH] + axisPID[ROLL], 1020, 2000); //RIGHT
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
}

