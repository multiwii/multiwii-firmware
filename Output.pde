#if defined(PROMINI)
  uint8_t PWM_PIN[8] = {9,10,11,3,6,5,A2,12};   //for a quad+: rear,right,left,front
#endif
#if defined(PROMICRO)
  #if !defined(HWPWM6)
    uint8_t PWM_PIN[8] = {9,10,5,6,4,A2,A0,A1};   //for a quad+: rear,right,left,front
  #else
    uint8_t PWM_PIN[8] = {9,10,5,6,11,13,A0,A1};   //for a quad+: rear,right,left,front
  #endif
#endif
#if defined(MEGA)
  uint8_t PWM_PIN[8] = {3,5,6,2,7,8,9,10};      //for a quad+: rear,right,left,front   //+ for y6: 7:under right  8:under left
#endif
#if !defined(PROMICRO) || defined(HWPWM6)
  volatile uint8_t atomicServo[8] = {125,125,125,125,125,125,125,125};
  //for HEX Y6 and HEX6/HEX6X flat for promini
  volatile uint8_t atomicPWM_PIN5_lowState;
  volatile uint8_t atomicPWM_PIN5_highState;
  volatile uint8_t atomicPWM_PIN6_lowState;
  volatile uint8_t atomicPWM_PIN6_highState;
  //for OCTO on promini
  volatile uint8_t atomicPWM_PINA2_lowState;
  volatile uint8_t atomicPWM_PINA2_highState;
  volatile uint8_t atomicPWM_PIN12_lowState;
  volatile uint8_t atomicPWM_PIN12_highState;
#else
  volatile uint16_t atomicServo[8] = {8000,8000,8000,8000,8000,8000,8000,8000};
  //for HEX Y6 and HEX6/HEX6X and for Promicro
  volatile uint16_t atomicPWM_PIN5_lowState;
  volatile uint16_t atomicPWM_PIN5_highState;
  volatile uint16_t atomicPWM_PIN6_lowState;
  volatile uint16_t atomicPWM_PIN6_highState;
  //for OCTO on Promicro
  volatile uint16_t atomicPWM_PINA2_lowState;
  volatile uint16_t atomicPWM_PINA2_highState;
  volatile uint16_t atomicPWM_PIN12_lowState;
  volatile uint16_t atomicPWM_PIN12_highState;
#endif

void writeServos() {
  #if defined(SERVO)
    #if defined(PRI_SERVO_FROM)    // write primary servos
      for(uint8_t i = (PRI_SERVO_FROM-1); i < PRI_SERVO_TO; i++){
        #if !defined(PROMICRO) || defined(HWPWM6)
          atomicServo[i] = (servo[i]-1000)>>2;
        #else
          atomicServo[i] = (servo[i]-1000)<<4;
        #endif
      }
    #endif
    #if defined(SEC_SERVO_FROM)   // write secundary servos
      #if defined(SERVO_TILT) && defined(MMSERVOGIMBAL)
        // Moving Average Servo Gimbal by Magnetron1
        static int16_t mediaMobileServoGimbalADC[3][MMSERVOGIMBALVECTORLENGHT];
        static int32_t mediaMobileServoGimbalADCSum[3];
        static uint8_t mediaMobileServoGimbalIDX;
        uint8_t axis;

        mediaMobileServoGimbalIDX = ++mediaMobileServoGimbalIDX % MMSERVOGIMBALVECTORLENGHT;
        for (axis=(SEC_SERVO_FROM-1); axis < SEC_SERVO_TO; axis++) {
          mediaMobileServoGimbalADCSum[axis] -= mediaMobileServoGimbalADC[axis][mediaMobileServoGimbalIDX];
          mediaMobileServoGimbalADC[axis][mediaMobileServoGimbalIDX] = servo[axis];
          mediaMobileServoGimbalADCSum[axis] += mediaMobileServoGimbalADC[axis][mediaMobileServoGimbalIDX];
          #if !defined(PROMICRO) || defined(HWPWM6)
            atomicServo[axis] = (mediaMobileServoGimbalADCSum[axis] / MMSERVOGIMBALVECTORLENGHT - 1000)>>2;
          #else
            atomicServo[axis] = (mediaMobileServoGimbalADCSum[axis] / MMSERVOGIMBALVECTORLENGHT - 1000)<<4;
          #endif
        }
      #else
        for(uint8_t i = (SEC_SERVO_FROM-1); i < SEC_SERVO_TO; i++){
          #if !defined(PROMICRO) || defined(HWPWM6)
            atomicServo[i] = (servo[i]-1000)>>2;
          #else
            atomicServo[i] = (servo[i]-1000)<<4;
          #endif
        }
      #endif
    #endif
  #endif
}

void writeMotors() { // [1000;2000] => [125;250]
  #if defined(MEGA)// [1000:2000] => [8000:16000] for timer 3 & 4 for mega
    #if (NUMBER_MOTOR > 0) 
      #ifndef EXT_MOTOR_RANGE 
        OCR3C = motor[0]<<3; //  pin 3
      #else
        OCR3C = ((motor[0]<<4) - 16000) + 128;
      #endif
    #endif
    #if (NUMBER_MOTOR > 1)
      #ifndef EXT_MOTOR_RANGE 
        OCR3A = motor[1]<<3; //  pin 5
      #else
        OCR3A = ((motor[1]<<4) - 16000) + 128;
      #endif
    #endif
    #if (NUMBER_MOTOR > 2)
      #ifndef EXT_MOTOR_RANGE 
        OCR4A = motor[2]<<3; //  pin 6
      #else
        OCR4A = ((motor[2]<<4) - 16000) + 128;
      #endif
    #endif
    #if (NUMBER_MOTOR > 3)
      #ifndef EXT_MOTOR_RANGE 
        OCR3B = motor[3]<<3; //  pin 2
      #else
        OCR3B = ((motor[3]<<4) - 16000) + 128;
      #endif
    #endif
    #if (NUMBER_MOTOR > 4)
      #ifndef EXT_MOTOR_RANGE 
        OCR4B = motor[4]<<3; //  pin 7
        OCR4C = motor[5]<<3; //  pin 8
      #else
        OCR4B = ((motor[4]<<4) - 16000) + 128;
        OCR4C = ((motor[5]<<4) - 16000) + 128;
      #endif
    #endif
    #if (NUMBER_MOTOR > 6)
      #ifndef EXT_MOTOR_RANGE 
        OCR2B = motor[6]>>3; //  pin 9
        OCR2A = motor[7]>>3; //  pin 10
      #else
        OCR2B = ((motor[6]>>2) - 250) + 2);
        OCR2A = ((motor[7]>>2) - 250) + 2);
      #endif
    #endif
  #endif
  #if defined(PROMICRO)
    #if (NUMBER_MOTOR > 0) // Timer 1 A & B [1000:2000] => [8000:16000]
      OCR1A = motor[0]<<3; //  pin 9
    #endif
    #if (NUMBER_MOTOR > 1)
      OCR1B = motor[1]<<3; //  pin 10
    #endif
    #if (NUMBER_MOTOR > 2) // Timer 4 A & D [1000:2000] => [1000:2000]
      #if !defined(HWPWM6)
        // to write values > 255 to timer 4 A/B we need to split the bytes
        static uint8_t pwm4_HBA;
        static uint16_t pwm4_LBA; // high and low byte for timer 4 A
        pwm4_LBA = 2047-motor[2]; pwm4_HBA = 0; // channel A is inverted
        while(pwm4_LBA > 255){
          pwm4_HBA++;
          pwm4_LBA-=256;
        }     
        TC4H = pwm4_HBA; OCR4A = pwm4_LBA; //  pin 5
      #else
        OCR3A = motor[2]<<3; //  pin 5
      #endif
    #endif
    #if (NUMBER_MOTOR > 3)
      static uint8_t pwm4_HBD;
      static uint16_t pwm4_LBD; // high and low byte for timer 4 D
      pwm4_LBD = motor[3]; pwm4_HBD = 0;
      while(pwm4_LBD > 255){
        pwm4_HBD++;
        pwm4_LBD-=256;
      }     
      TC4H = pwm4_HBD; OCR4D = pwm4_LBD; //  pin 6
    #endif    
    #if (NUMBER_MOTOR > 4)
      #if !defined(HWPWM6)
        atomicPWM_PIN5_highState = ((motor[4]-1000)<<4)+320;
        atomicPWM_PIN5_lowState = 15743-atomicPWM_PIN5_highState;
        atomicPWM_PIN6_highState = ((motor[5]-1000)<<4)+320;
        atomicPWM_PIN6_lowState = 15743-atomicPWM_PIN6_highState;
      #else
        OCR1C = motor[4]<<3; //  pin 11
        static uint8_t pwm4_HBA;
        static uint16_t pwm4_LBA; // high and low byte for timer 4 A
        pwm4_LBA = motor[5]; pwm4_HBA = 0;
        while(pwm4_LBA > 255){
          pwm4_HBA++;
          pwm4_LBA-=256;
        }     
        TC4H = pwm4_HBA; OCR4A = pwm4_LBA; //  pin 13      
      #endif
    #endif
    #if (NUMBER_MOTOR > 6)
      #if !defined(HWPWM6)
        atomicPWM_PINA2_highState = ((motor[6]-1000)<<4)+320;
        atomicPWM_PINA2_lowState = 15743-atomicPWM_PINA2_highState;
        atomicPWM_PIN12_highState = ((motor[7]-1000)<<4)+320;
        atomicPWM_PIN12_lowState = 15743-atomicPWM_PIN12_highState;
      #else
        atomicPWM_PINA2_highState = ((motor[6]-1000)>>2)+5;
        atomicPWM_PINA2_lowState = 245-atomicPWM_PINA2_highState;
        atomicPWM_PIN12_highState = ((motor[7]-1000)>>2)+5;
        atomicPWM_PIN12_lowState = 245-atomicPWM_PIN12_highState;     
      #endif
    #endif
  #endif
  #if defined(PROMINI)
    #if (NUMBER_MOTOR > 0)
      #ifndef EXT_MOTOR_RANGE 
        OCR1A = motor[0]>>3; //  pin 9
      #else
        OCR1A = ((motor[0]>>2) - 250) + 2;
      #endif
    #endif
    #if (NUMBER_MOTOR > 1)
      #ifndef EXT_MOTOR_RANGE 
        OCR1B = motor[1]>>3; //  pin 10
      #else
        OCR1B = ((motor[1]>>2) - 250) + 2;
      #endif
    #endif
    #if (NUMBER_MOTOR > 2)
      #ifndef EXT_MOTOR_RANGE 
        OCR2A = motor[2]>>3; //  pin 11
      #else
        OCR2A = ((motor[2]>>2) - 250) + 2;
      #endif
    #endif
    #if (NUMBER_MOTOR > 3)
      #ifndef EXT_MOTOR_RANGE 
        OCR2B = motor[3]>>3; //  pin 3
      #else
        OCR2B = ((motor[3]>>2) - 250) + 2;
      #endif
    #endif
    #if (NUMBER_MOTOR > 4) //note: EXT_MOTOR_RANGE not possible here
      atomicPWM_PIN6_highState = ((motor[4]-1000)>>2)+5;
      atomicPWM_PIN6_lowState  = 245-atomicPWM_PIN6_highState;
      atomicPWM_PIN5_highState = ((motor[5]-1000)>>2)+5;
      atomicPWM_PIN5_lowState  = 245-atomicPWM_PIN5_highState;
    #endif
    #if (NUMBER_MOTOR > 6) //note: EXT_MOTOR_RANGE not possible here
      atomicPWM_PINA2_highState = ((motor[6]-1000)>>2)+5;
      atomicPWM_PINA2_lowState  = 245-atomicPWM_PINA2_highState;
      atomicPWM_PIN12_highState = ((motor[7]-1000)>>2)+5;
      atomicPWM_PIN12_lowState  = 245-atomicPWM_PIN12_highState;
    #endif
  #endif
}

void writeAllMotors(int16_t mc) {   // Sends commands to all motors
  for (uint8_t i =0;i<NUMBER_MOTOR;i++)
    motor[i]=mc;
  writeMotors();
}

void initOutput() {
  for(uint8_t i=0;i<NUMBER_MOTOR;i++)
    pinMode(PWM_PIN[i],OUTPUT);
  #if defined(MEGA)
    #if (NUMBER_MOTOR > 0)
      // init 16bit timer 3
      TCCR3A |= (1<<WGM31); // phase correct mode
      TCCR3A &= ~(1<<WGM30);
      TCCR3B |= (1<<WGM33);
      TCCR3B &= ~(1<<CS31); // no prescaler
      ICR3   |= 0x3FFF; // TOP to = 16383;      
      
      TCCR3A |= _BV(COM3C1); // connect pin 3 to timer 3 channel C
    #endif
    #if (NUMBER_MOTOR > 1)
      TCCR3A |= _BV(COM3A1); // connect pin 5 to timer 3 channel A
    #endif
    #if (NUMBER_MOTOR > 2)
      // init 16bit timer 4
      TCCR4A |= (1<<WGM41); // phase correct mode
      TCCR4A &= ~(1<<WGM40);
      TCCR4B |= (1<<WGM43);
      TCCR4B &= ~(1<<CS41); // no prescaler
      ICR4   |= 0x3FFF; // TOP to = 16383;    
      
      TCCR4A |= _BV(COM4A1); // connect pin 6 to timer 4 channel A
    #endif
    #if (NUMBER_MOTOR > 3)
      TCCR3A |= _BV(COM3B1); // connect pin 2 to timer 3 channel B
    #endif
    #if (NUMBER_MOTOR > 4)
      TCCR4A |= _BV(COM4B1); // connect pin 7 to timer 4 channel B
      TCCR4A |= _BV(COM4C1); // connect pin 8 to timer 4 channel C
    #endif
    #if (NUMBER_MOTOR > 6)
      // timer 2 is a 8bit timer so we cant change its range 
      TCCR2A |= _BV(COM2B1); // connect pin 9 to timer 2 channel B
      TCCR2A |= _BV(COM2A1); // connect pin 10 to timer 2 channel A
    #endif
  #endif
  #if defined(PROMICRO)
    #if (NUMBER_MOTOR > 0)
      TCCR1A |= (1<<WGM11); TCCR1A &= ~(1<<WGM10); TCCR1B |= (1<<WGM13);  // phase correct mode
      TCCR1B &= ~(1<<CS11); ICR1 |= 0x3FFF; // no prescaler & TOP to 16383;
      TCCR1A |= _BV(COM1A1); // connect pin 9 to timer 1 channel A
    #endif
    #if (NUMBER_MOTOR > 1)
      TCCR1A |= _BV(COM1B1); // connect pin 10 to timer 1 channel B
    #endif
    #if (NUMBER_MOTOR > 2)
      #if !defined(HWPWM6) // timer 4A
        TCCR4E |= (1<<ENHC4); // enhanced pwm mode
        TCCR4B &= ~(1<<CS41); TCCR4B |= (1<<CS42)|(1<<CS40); // prescaler to 16
        TCCR4D |= (1<<WGM40); TC4H = 0x3; OCR4C = 0xFF; // phase and frequency correct mode & top to 1023 but with enhanced pwm mode we have 2047
        TCCR4A |= (1<<COM4A0)|(1<<PWM4A); // connect pin 5 to timer 4 channel A 
      #else // timer 3A
        TCCR3A |= (1<<WGM31); TCCR3A &= ~(1<<WGM30); TCCR3B |= (1<<WGM33);  // phase correct mode
        TCCR3B &= ~(1<<CS31); ICR3 |= 0x3FFF; // no prescaler & TOP to 16383;
        TCCR3A |= _BV(COM3A1); // connect pin 5 to timer 3 channel A    
      #endif 
    #endif
    #if (NUMBER_MOTOR > 3)
      #if defined(HWPWM6) 
        TCCR4E |= (1<<ENHC4); // enhanced pwm mode
        TCCR4B &= ~(1<<CS41); TCCR4B |= (1<<CS42)|(1<<CS40); // prescaler to 16
        TCCR4D |= (1<<WGM40); TC4H = 0x3; OCR4C = 0xFF; // phase and frequency correct mode & top to 1023 but with enhanced pwm mode we have 2047
      #endif
      TCCR4C |= (1<<COM4D1)|(1<<PWM4D); // connect pin 6 to timer 4 channel D
    #endif
    #if (NUMBER_MOTOR > 4)
      #if defined(HWPWM6) 
        TCCR1A |= _BV(COM1C1); // connect pin 11 to timer 1 channel C
        TCCR4A |= (1<<COM4A1)|(1<<PWM4A); // connect pin 13 to timer 4 channel A 
      #else
        initializeSoftPWM();
      #endif
    #endif
    #if (NUMBER_MOTOR > 6)
      #if defined(HWPWM6) 
        initializeSoftPWM();
      #endif
    #endif
  #endif
  #if defined(PROMINI)
    #if (NUMBER_MOTOR > 0)
      TCCR1A |= _BV(COM1A1); // connect pin 9 to timer 1 channel A
    #endif
    #if (NUMBER_MOTOR > 1)
      TCCR1A |= _BV(COM1B1); // connect pin 10 to timer 1 channel B
    #endif
    #if (NUMBER_MOTOR > 2)
      TCCR2A |= _BV(COM2A1); // connect pin 11 to timer 2 channel A
    #endif
    #if (NUMBER_MOTOR > 3)
      TCCR2A |= _BV(COM2B1); // connect pin 3 to timer 2 channel B
    #endif
    #if (NUMBER_MOTOR > 5)  // PIN 5 & 6 or A0 & A1
      initializeSoftPWM();
      #if defined(A0_A1_PIN_HEX) || (NUMBER_MOTOR > 6)
        pinMode(5,INPUT);pinMode(6,INPUT);     // we reactivate the INPUT affectation for these two PINs
        pinMode(A0,OUTPUT);pinMode(A1,OUTPUT);
      #endif
    #endif
  #endif
  
  writeAllMotors(MINCOMMAND);
  delay(300);
  #if defined(SERVO)
    initializeServo();
  #endif
}

#if defined(SERVO)
void initializeServo() {
  #if (PRI_SERVO_FROM == 1) || (SEC_SERVO_FROM == 1)
    SERVO_1_PINMODE;
  #endif
  #if (PRI_SERVO_FROM <= 2 && PRI_SERVO_TO >= 2) || (SEC_SERVO_FROM <= 2 && SEC_SERVO_TO >= 2) 
    SERVO_2_PINMODE;
  #endif
  #if (PRI_SERVO_FROM <= 3 && PRI_SERVO_TO >= 3) || (SEC_SERVO_FROM <= 3 && SEC_SERVO_TO >= 3) 
    SERVO_3_PINMODE;
  #endif 
  #if (PRI_SERVO_FROM <= 4 && PRI_SERVO_TO >= 4) || (SEC_SERVO_FROM <= 4 && SEC_SERVO_TO >= 4) 
    SERVO_4_PINMODE;
  #endif 
  #if (PRI_SERVO_FROM <= 5 && PRI_SERVO_TO >= 5) || (SEC_SERVO_FROM <= 5 && SEC_SERVO_TO >= 5) 
    SERVO_5_PINMODE;
  #endif 
  #if (PRI_SERVO_FROM <= 6 && PRI_SERVO_TO >= 6) || (SEC_SERVO_FROM <= 6 && SEC_SERVO_TO >= 6) 
    SERVO_6_PINMODE;
  #endif 
  #if (PRI_SERVO_FROM <= 7 && PRI_SERVO_TO >= 7) || (SEC_SERVO_FROM <= 7 && SEC_SERVO_TO >= 7) 
    SERVO_7_PINMODE;
  #endif 
  #if (PRI_SERVO_FROM <= 8 && PRI_SERVO_TO >= 8) || (SEC_SERVO_FROM <= 8 && SEC_SERVO_TO >= 8) 
    SERVO_8_PINMODE;
  #endif
  
  #if !defined(PROMICRO) || defined(HWPWM6)
    TCCR0A = 0; // normal counting mode
    TIMSK0 |= (1<<OCIE0A); // Enable CTC interrupt
  #else
    TCCR3A &= ~(1<<WGM30); // normal counting mode
    TCCR3B &= ~(1<<CS31); // no prescaler
    TIMSK3 |= (1<<OCIE3A); // Enable CTC interrupt   
  #endif
}

#if !defined(PROMICRO) || defined(HWPWM6)
  #define SERVO_ISR TIMER0_COMPA_vect
  #define SERVO_CHANNEL OCR0A
  #define SERVO_1K_US 250
#else
  #define SERVO_ISR TIMER3_COMPA_vect
  #define SERVO_CHANNEL OCR3A
  #define SERVO_1K_US 16000
#endif

// prescaler is set by default to 64 on Timer0
// Duemilanove : 16MHz / 64 => 4 us
// 256 steps = 1 counter cycle = 1024 us
ISR(SERVO_ISR) {
  static uint8_t state = 0;
  static uint8_t count;
  if (state == 0) {
    #if (PRI_SERVO_FROM == 1) || (SEC_SERVO_FROM == 1)
      SERVO_1_PIN_HIGH;
    #endif
    SERVO_CHANNEL+= SERVO_1K_US; // 1000 us
    state++ ;
  } else if (state == 1) {
    SERVO_CHANNEL+= atomicServo[0]; // 1000 + [0-1020] us
    state++;
  } else if (state == 2) {
    #if (PRI_SERVO_FROM == 1) || (SEC_SERVO_FROM == 1)
      SERVO_1_PIN_LOW;
    #endif
    #if (PRI_SERVO_FROM <= 2 && PRI_SERVO_TO >= 2) || (SEC_SERVO_FROM <= 2 && SEC_SERVO_TO >= 2) 
      SERVO_2_PIN_HIGH;
    #endif
    SERVO_CHANNEL+= SERVO_1K_US; // 1000 us
    state++;
  } else if (state == 3) {
    SERVO_CHANNEL+= atomicServo[1]; // 1000 + [0-1020] us
    state++;
  } else if (state == 4) {
    #if (PRI_SERVO_FROM <= 2 && PRI_SERVO_TO >= 2) || (SEC_SERVO_FROM <= 2 && SEC_SERVO_TO >= 2)  
      SERVO_2_PIN_LOW;
    #endif
    #if (PRI_SERVO_FROM <= 3 && PRI_SERVO_TO >= 3) || (SEC_SERVO_FROM <= 3 && SEC_SERVO_TO >= 3)   
      SERVO_3_PIN_HIGH;
    #endif
    state++;
    SERVO_CHANNEL+= SERVO_1K_US; // 1000 us
  } else if (state == 5) {
    SERVO_CHANNEL+= atomicServo[2]; // 1000 + [0-1020] us
    state++;
  } else if (state == 6) {
    #if (PRI_SERVO_FROM <= 3 && PRI_SERVO_TO >= 3) || (SEC_SERVO_FROM <= 3 && SEC_SERVO_TO >= 3)      
      SERVO_3_PIN_LOW;
    #endif
    #if (PRI_SERVO_FROM <= 4 && PRI_SERVO_TO >= 4) || (SEC_SERVO_FROM <= 4 && SEC_SERVO_TO >= 4)   
      SERVO_4_PIN_HIGH;
    #endif
    state++;
    SERVO_CHANNEL+= SERVO_1K_US; // 1000 us
  } else if (state == 7) {
    SERVO_CHANNEL+= atomicServo[3]; // 1000 + [0-1020] us
    state++;
  } else if (state == 8) {
    #if (PRI_SERVO_FROM <= 4 && PRI_SERVO_TO >= 4) || (SEC_SERVO_FROM <= 4 && SEC_SERVO_TO >= 4)    
      SERVO_4_PIN_LOW;
    #endif  
    #if (PRI_SERVO_FROM <= 5 && PRI_SERVO_TO >= 5) || (SEC_SERVO_FROM <= 5 && SEC_SERVO_TO >= 5)   
      SERVO_5_PIN_HIGH;;
    #endif 
    state++;
    SERVO_CHANNEL+= SERVO_1K_US; // 1000 us
  } else if (state == 9) {
    SERVO_CHANNEL+= atomicServo[4]; // 1000 + [0-1020] us
    state++;
  } else if (state == 10) {
    #if (PRI_SERVO_FROM <= 5 && PRI_SERVO_TO >= 5) || (SEC_SERVO_FROM <= 5 && SEC_SERVO_TO >= 5)    
      SERVO_5_PIN_LOW;
    #endif 
    #if (PRI_SERVO_FROM <= 6 && PRI_SERVO_TO >= 6) || (SEC_SERVO_FROM <= 6 && SEC_SERVO_TO >= 6)    
      SERVO_6_PIN_HIGH;;
    #endif 
    state++;
    SERVO_CHANNEL+= SERVO_1K_US; // 1000 us
  } else if (state == 11) {
    SERVO_CHANNEL+= atomicServo[5]; // 1000 + [0-1020] us
    state++;
  } else if (state == 12) {
    #if (PRI_SERVO_FROM <= 6 && PRI_SERVO_TO >= 6) || (SEC_SERVO_FROM <= 6 && SEC_SERVO_TO >= 6)       
      SERVO_6_PIN_LOW;
    #endif 
    #if (PRI_SERVO_FROM <= 7 && PRI_SERVO_TO >= 7) || (SEC_SERVO_FROM <= 7 && SEC_SERVO_TO >= 7)       
      SERVO_7_PIN_HIGH;
    #endif 
    state++;
    SERVO_CHANNEL+= SERVO_1K_US; // 1000 us
  } else if (state == 13) {
    SERVO_CHANNEL+= atomicServo[6]; // 1000 + [0-1020] us
    state++;
  } else if (state == 14) {
    #if (PRI_SERVO_FROM <= 7 && PRI_SERVO_TO >= 7) || (SEC_SERVO_FROM <= 7 && SEC_SERVO_TO >= 7)       
      SERVO_7_PIN_LOW;
    #endif 
    #if (PRI_SERVO_FROM <= 8 && PRI_SERVO_TO >= 8) || (SEC_SERVO_FROM <= 8 && SEC_SERVO_TO >= 8)    
      SERVO_8_PIN_HIGH;
    #endif     
    state++;
    SERVO_CHANNEL+= SERVO_1K_US; // 1000 us
  } else if (state == 15) {
    SERVO_CHANNEL+= atomicServo[7]; // 1000 + [0-1020] us
    state++;
  } else if (state == 16) {
    #if (PRI_SERVO_FROM <= 8 && PRI_SERVO_TO >= 8) || (SEC_SERVO_FROM <= 8 && SEC_SERVO_TO >= 8)     
      SERVO_8_PIN_LOW;
    #endif 
    count = 2;
    state++;
    SERVO_CHANNEL+= SERVO_1K_US; // 1000 us
  } else if (state == 17) {
    if (count > 0) count--;
    else state = 0;
    SERVO_CHANNEL+= SERVO_1K_US;
  }
}
#endif

#if (NUMBER_MOTOR > 4) && (defined(PROMINI) || defined(PROMICRO))

  #if !defined(PROMICRO) || defined(HWPWM6)
    #define SOFT_PWM_ISR1 TIMER0_COMPB_vect
    #define SOFT_PWM_ISR2 TIMER0_COMPA_vect
    #define SOFT_PWM_CHANNEL1 OCR0B
    #define SOFT_PWM_CHANNEL2 OCR0A 
  #else
    #define SOFT_PWM_ISR1 TIMER3_COMPB_vect
    #define SOFT_PWM_ISR2 TIMER3_COMPC_vect
    #define SOFT_PWM_CHANNEL1 OCR3B
    #define SOFT_PWM_CHANNEL2 OCR3C   
  #endif

  void initializeSoftPWM() {
    #if !defined(PROMICRO) || defined(HWPWM6)
      TCCR0A = 0; // normal counting mode
      #if (NUMBER_MOTOR > 4) && !defined(HWPWM6) 
        TIMSK0 |= (1<<OCIE0B); // Enable CTC interrupt  
      #endif
      #if (NUMBER_MOTOR > 6)
        TIMSK0 |= (1<<OCIE0A);
      #endif
    #else
        TCCR3A &= ~(1<<WGM30); // normal counting mode
        TCCR3B &= ~(1<<CS31); // no prescaler
        TIMSK3 |= (1<<OCIE3B); // Enable CTC interrupt  
        #if (NUMBER_MOTOR > 6)
          TIMSK3 |= (1<<OCIE3C);
        #endif   
    #endif
  }
  
  // HEXA with just OCR0B 
  ISR(SOFT_PWM_ISR1) { 
    static uint8_t state = 0;
    if(state == 0){
      #if !defined(A0_A1_PIN_HEX) && (NUMBER_MOTOR < 8)
        SOFT_PWM_1_PIN_HIGH;
      #else
        PORTC |= 1<<0;//PIN A0
      #endif
      SOFT_PWM_CHANNEL1 += atomicPWM_PIN5_highState;
      state = 1;
    }else if(state == 1){
      #if !defined(A0_A1_PIN_HEX) && (NUMBER_MOTOR < 8)
        SOFT_PWM_2_PIN_LOW;
      #else
        PORTC &= ~(1<<1);
      #endif
      SOFT_PWM_CHANNEL1 += atomicPWM_PIN6_lowState;
      state = 2;
    }else if(state == 2){
      #if !defined(A0_A1_PIN_HEX) && (NUMBER_MOTOR < 8)
       SOFT_PWM_2_PIN_HIGH;
      #else
        PORTC |= 1<<1;//PIN A1
      #endif
      SOFT_PWM_CHANNEL1 += atomicPWM_PIN6_highState;
      state = 3;  
    }else if(state == 3){
      #if !defined(A0_A1_PIN_HEX) && (NUMBER_MOTOR < 8)
        SOFT_PWM_1_PIN_LOW;
      #else
        PORTC &= ~(1<<0);
      #endif 
      SOFT_PWM_CHANNEL1 += atomicPWM_PIN5_lowState;
      state = 0;   
    }
  }

  //the same with digital PIN A2 & 12 OCR0A counter for OCTO
  #if (NUMBER_MOTOR > 6) && !defined(HWPWM6)
    ISR(SOFT_PWM_ISR2) {
      static uint8_t state = 0;
      if(state == 0){
        SOFT_PWM_3_PIN_HIGH;
        SOFT_PWM_CHANNEL2 += atomicPWM_PINA2_highState;
        state = 1;
      }else if(state == 1){
        SOFT_PWM_4_PIN_LOW;
        SOFT_PWM_CHANNEL2 += atomicPWM_PIN12_lowState;
        state = 2;
      }else if(state == 2){
        SOFT_PWM_4_PIN_HIGH;
        SOFT_PWM_CHANNEL2 += atomicPWM_PIN12_highState;
        state = 3;  
      }else if(state == 3){
        SOFT_PWM_3_PIN_LOW;
        SOFT_PWM_CHANNEL2 += atomicPWM_PINA2_lowState;
        state = 0;   
      }
    }
  #endif
#endif

void mixTable() {
  int16_t maxMotor;
  uint8_t i;

  #define PIDMIX(X,Y,Z) rcCommand[THROTTLE] + axisPID[ROLL]*X + axisPID[PITCH]*Y + YAW_DIRECTION * axisPID[YAW]*Z

  #if NUMBER_MOTOR > 3
    //prevent "yaw jump" during yaw correction
    axisPID[YAW] = constrain(axisPID[YAW],-100-abs(rcCommand[YAW]),+100+abs(rcCommand[YAW]));
  #endif
  #ifdef BI
    motor[0] = PIDMIX(+1, 0, 0); //LEFT
    motor[1] = PIDMIX(-1, 0, 0); //RIGHT        
    servo[4]  = constrain(1500 + YAW_DIRECTION * (axisPID[YAW] + axisPID[PITCH]), 1020, 2000); //LEFT
    servo[5]  = constrain(1500 + YAW_DIRECTION * (axisPID[YAW] - axisPID[PITCH]), 1020, 2000); //RIGHT
  #endif
  #ifdef TRI
    motor[0] = PIDMIX( 0,+4/3, 0); //REAR
    motor[1] = PIDMIX(-1,-2/3, 0); //RIGHT
    motor[2] = PIDMIX(+1,-2/3, 0); //LEFT
    servo[5] = constrain(tri_yaw_middle + YAW_DIRECTION * axisPID[YAW], TRI_YAW_CONSTRAINT_MIN, TRI_YAW_CONSTRAINT_MAX); //REAR
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
  #ifdef VTAIL4
    motor[0] = PIDMIX(+0,+1, -1/2);      //REAR_R 
    motor[1] = PIDMIX(-1, -1, +2/10); //FRONT_R 
    motor[2] = PIDMIX(+0,+1, +1/2);      //REAR_L 
    motor[3] = PIDMIX(+1, -1, -2/10); //FRONT_L
  #endif

  #ifdef SERVO_TILT
    #if defined(A0_A1_PIN_HEX) && (NUMBER_MOTOR == 6) && defined(PROMINI)
      #define S_PITCH servo[2]
      #define S_ROLL  servo[3]
    #else
      #define S_PITCH servo[0]
      #define S_ROLL  servo[1]
    #endif
    S_PITCH = TILT_PITCH_MIDDLE + rcData[AUX3]-1500;
    S_ROLL  = TILT_ROLL_MIDDLE  + rcData[AUX4]-1500;
    if (rcOptions[BOXCAMSTAB]) {
      S_PITCH += TILT_PITCH_PROP * angle[PITCH] /16 ;
      S_ROLL  += TILT_ROLL_PROP  * angle[ROLL]  /16 ;
    }
    S_PITCH = constrain(S_PITCH, TILT_PITCH_MIN, TILT_PITCH_MAX);
    S_ROLL  = constrain(S_ROLL , TILT_ROLL_MIN, TILT_ROLL_MAX  );   
  #endif
  #ifdef GIMBAL
    servo[0] = constrain(TILT_PITCH_MIDDLE + TILT_PITCH_PROP * angle[PITCH] /16 + rcCommand[PITCH], TILT_PITCH_MIN, TILT_PITCH_MAX);
    servo[1] = constrain(TILT_ROLL_MIDDLE + TILT_ROLL_PROP   * angle[ROLL]  /16 + rcCommand[ROLL], TILT_ROLL_MIN, TILT_ROLL_MAX);
  #endif
  #ifdef FLYING_WING
    motor[0] = rcCommand[THROTTLE];
    if (passThruMode) {// do not use sensors for correction, simple 2 channel mixing
       servo[0]  = PITCH_DIRECTION_L * (rcData[PITCH]-MIDRC) + ROLL_DIRECTION_L * (rcData[ROLL]-MIDRC);
       servo[1]  = PITCH_DIRECTION_R * (rcData[PITCH]-MIDRC) + ROLL_DIRECTION_R * (rcData[ROLL]-MIDRC);
    } else { // use sensors to correct (gyro only or gyro+acc according to aux1/aux2 configuration
       servo[0]  = PITCH_DIRECTION_L * axisPID[PITCH]        + ROLL_DIRECTION_L * axisPID[ROLL];
       servo[1]  = PITCH_DIRECTION_R * axisPID[PITCH]        + ROLL_DIRECTION_R * axisPID[ROLL];
    }
    servo[0]  = constrain(servo[0] + wing_left_mid , WING_LEFT_MIN,  WING_LEFT_MAX );
    servo[1]  = constrain(servo[1] + wing_right_mid, WING_RIGHT_MIN, WING_RIGHT_MAX);
  #endif
  #if defined(CAMTRIG)
    static uint8_t camCycle = 0;
    static uint8_t camState = 0;
    static uint32_t camTime = 0;
    if (camCycle==1) {
      if (camState == 0) {
        servo[2] = CAM_SERVO_HIGH;
        camState = 1;
        camTime = millis();
      } else if (camState == 1) {
       if ( (millis() - camTime) > CAM_TIME_HIGH ) {
         servo[2] = CAM_SERVO_LOW;
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
    if (rcOptions[BOXCAMTRIG]) camCycle=1;
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

  #if (LOG_VALUES == 2) || defined(POWERMETER_SOFT)
    uint32_t amp;
    /* true cubic function; when divided by vbat_max=126 (12.6V) for 3 cell battery this gives maximum value of ~ 500 */
    /* Lookup table moved to PROGMEM 11/21/2001 by Danal */
    static uint16_t amperes[64] =   {   0,  2,  6, 15, 30, 52, 82,123,
                                     175,240,320,415,528,659,811,984,
                                     1181,1402,1648,1923,2226,2559,2924,3322,
                                     3755,4224,4730,5276,5861,6489,7160,7875,
                                     8637 ,9446 ,10304,11213,12173,13187,14256,15381,
                                     16564,17805,19108,20472,21900,23392,24951,26578,
                                     28274,30041,31879,33792,35779,37843,39984,42205,
                                     44507,46890,49358,51910,54549,57276,60093,63000};
  
    if (vbat) { // by all means - must avoid division by zero 
      for (i =0;i<NUMBER_MOTOR;i++) {
        amp = amperes[ ((motor[i] - 1000)>>4) ] / vbat; // range mapped from [1000:2000] => [0:1000]; then break that up into 64 ranges; lookup amp
  	  #if (LOG_VALUES == 2)
           pMeter[i]+= amp; // sum up over time the mapped ESC input 
        #endif
        #if defined(POWERMETER_SOFT)
           pMeter[PMOTOR_SUM]+= amp; // total sum over all motors
        #endif
      }
    }
  #endif
}
