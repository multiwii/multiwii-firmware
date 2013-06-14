/**************************************************************************************/
/***************                  Motor Pin order                  ********************/
/**************************************************************************************/
// since we are uing the PWM generation in a direct way, the pin order is just to inizialie the right pins 
// its not possible to change a PWM output pin just by changing the order
#if defined(PROMINI)
  uint8_t PWM_PIN[8] = {9,10,11,3,6,5,A2,12};   //for a quad+: rear,right,left,front
#endif
#if defined(PROMICRO)
  #if !defined(HWPWM6)
    #if defined(TEENSY20)
      uint8_t PWM_PIN[8] = {14,15,9,12,22,18,16,17};   //for a quad+: rear,right,left,front
    #elif defined(A32U4_4_HW_PWM_SERVOS)
      uint8_t PWM_PIN[8] = {6,9,10,11,5,13,SW_PWM_P3,SW_PWM_P4};   //
    #else
    uint8_t PWM_PIN[8] = {9,10,5,6,4,A2,SW_PWM_P3,SW_PWM_P4};   //for a quad+: rear,right,left,front
    #endif
  #else
    #if defined(TEENSY20)
      uint8_t PWM_PIN[8] = {14,15,9,12,4,10,16,17};   //for a quad+: rear,right,left,front
    #elif defined(A32U4_4_HW_PWM_SERVOS)
      uint8_t PWM_PIN[8] = {6,9,10,11,5,13,SW_PWM_P3,SW_PWM_P4};   //
    #else
      uint8_t PWM_PIN[8] = {9,10,5,6,11,13,SW_PWM_P3,SW_PWM_P4};   //for a quad+: rear,right,left,front
    #endif
  #endif
#endif
#if defined(MEGA)
  uint8_t PWM_PIN[8] = {3,5,6,2,7,8,9,10};      //for a quad+: rear,right,left,front   //+ for y6: 7:under right  8:under left
#endif

/**************************************************************************************/
/***************         Software PWM & Servo variables            ********************/
/**************************************************************************************/
#if defined(PROMINI) || (defined(PROMICRO) && defined(HWPWM6)) || (defined(MEGA) && defined(MEGA_HW_PWM_SERVOS))
  #if (NUMBER_MOTOR > 4)
    //for HEX Y6 and HEX6/HEX6X/HEX6H flat for promini
    volatile uint8_t atomicPWM_PIN5_lowState;
    volatile uint8_t atomicPWM_PIN5_highState;
    volatile uint8_t atomicPWM_PIN6_lowState;
    volatile uint8_t atomicPWM_PIN6_highState;
  #endif
  #if (NUMBER_MOTOR > 6)
    //for OCTO on promini
    volatile uint8_t atomicPWM_PINA2_lowState;
    volatile uint8_t atomicPWM_PINA2_highState;
    volatile uint8_t atomicPWM_PIN12_lowState;
    volatile uint8_t atomicPWM_PIN12_highState;
  #endif
#else
  #if (NUMBER_MOTOR > 4)
    //for HEX Y6 and HEX6/HEX6X/HEX6H and for Promicro
    volatile uint16_t atomicPWM_PIN5_lowState;
    volatile uint16_t atomicPWM_PIN5_highState;
    volatile uint16_t atomicPWM_PIN6_lowState;
    volatile uint16_t atomicPWM_PIN6_highState;
  #endif
  #if (NUMBER_MOTOR > 6)
    //for OCTO on Promicro
    volatile uint16_t atomicPWM_PINA2_lowState;
    volatile uint16_t atomicPWM_PINA2_highState;
    volatile uint16_t atomicPWM_PIN12_lowState;
    volatile uint16_t atomicPWM_PIN12_highState;
  #endif
#endif

#if defined(SERVO)
  #if (defined(PROMICRO) && defined(A32U4_4_HW_PWM_SERVOS))  || (defined(MEGA) && defined(MEGA_HW_PWM_SERVOS))
    // hw servo pwm does not need atomicServo[]
  #elif defined(PROMINI) || (defined(PROMICRO) && defined(HWPWM6))
    #if defined(AIRPLANE) || defined(HELICOPTER)
      // To prevent motor to start at reset. atomicServo[7]=5 or 249 if reversed servo
      volatile uint8_t atomicServo[8] = {125,125,125,125,125,125,125,5};
    #else
      volatile uint8_t atomicServo[8] = {125,125,125,125,125,125,125,125};
    #endif
  #else
    #if defined(AIRPLANE)|| defined(HELICOPTER)
      // To prevent motor to start at reset. atomicServo[7]=5 or 249 if reversed servo
      volatile uint16_t atomicServo[8] = {8000,8000,8000,8000,8000,8000,8000,320};
    #else
      volatile uint16_t atomicServo[8] = {8000,8000,8000,8000,8000,8000,8000,8000};
    #endif
  #endif
#endif

/**************************************************************************************/
/***************       Calculate first and last used servos        ********************/
/**************************************************************************************/
#if defined(SERVO)
  #if defined(PRI_SERVO_FROM) && defined(SEC_SERVO_FROM)
    #if PRI_SERVO_FROM < SEC_SERVO_FROM
      #define SERVO_START PRI_SERVO_FROM
    #else
      #define SERVO_START SEC_SERVO_FROM
    #endif
  #else
    #if defined(PRI_SERVO_FROM)
      #define SERVO_START PRI_SERVO_FROM
    #endif
    #if defined(SEC_SERVO_FROM)
      #define SERVO_START SEC_SERVO_FROM
    #endif
  #endif
  #if defined(PRI_SERVO_TO) && defined(SEC_SERVO_TO)
    #if PRI_SERVO_TO > SEC_SERVO_TO
      #define SERVO_END PRI_SERVO_TO
    #else
      #define SERVO_END SEC_SERVO_TO
    #endif
  #else
    #if defined(PRI_SERVO_TO)
      #define SERVO_END PRI_SERVO_TO
    #endif
    #if defined(SEC_SERVO_TO)
      #define SERVO_END SEC_SERVO_TO
    #endif
  #endif


#endif

/**************************************************************************************/
/***************   Writes the Servos values to the needed format   ********************/
/**************************************************************************************/
void writeServos() {
  #if defined(SERVO)
    #if defined(PRI_SERVO_FROM) && !(defined(PROMICRO)&&defined(A32U4_4_HW_PWM_SERVOS)) && !(defined(MEGA)&&defined(MEGA_HW_PWM_SERVOS))   // write primary servos
      for(uint8_t i = (PRI_SERVO_FROM-1); i < PRI_SERVO_TO; i++){
        #if defined(PROMINI) || (defined(PROMICRO) && defined(HWPWM6)) || (defined(MEGA) && defined(MEGA_HW_PWM_SERVOS))
          atomicServo[i] = (servo[i]-1000)>>2;
        #else
          atomicServo[i] = (servo[i]-1000)<<4;
        #endif
      }
    #endif
    #if defined(SEC_SERVO_FROM)   // write secundary servos
      #if (defined(SERVO_TILT)|| defined(SERVO_MIX_TILT)) && defined(MMSERVOGIMBAL)
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
          #if defined(PROMINI) || (defined(PROMICRO) && defined(HWPWM6))
            atomicServo[axis] = (mediaMobileServoGimbalADCSum[axis] / MMSERVOGIMBALVECTORLENGHT - 1000)>>2;
          #else
            atomicServo[axis] = (mediaMobileServoGimbalADCSum[axis] / MMSERVOGIMBALVECTORLENGHT - 1000)<<4;
          #endif
        }
      #else
        for(uint8_t i = (SEC_SERVO_FROM-1); i < SEC_SERVO_TO; i++){
          #if defined(PROMINI) || (defined(PROMICRO) && defined(HWPWM6)) || (defined(MEGA) && defined(MEGA_HW_PWM_SERVOS))
            atomicServo[i] = (servo[i]-1000)>>2;
          #else
            atomicServo[i] = (servo[i]-1000)<<4;
          #endif
        }
      #endif
    #endif
    // write HW PWM servos for the mega
    #if defined(MEGA) && defined(MEGA_HW_PWM_SERVOS)
      #if (PRI_SERVO_FROM == 1 || SEC_SERVO_FROM == 1) 
        OCR5C = servo[0];
      #endif 
      #if (PRI_SERVO_FROM <= 2 && PRI_SERVO_TO >= 2) || (SEC_SERVO_FROM <= 2 && SEC_SERVO_TO >= 2) 
        OCR5B = servo[1];
      #endif 
      #if (PRI_SERVO_FROM <= 3 && PRI_SERVO_TO >= 3) || (SEC_SERVO_FROM <= 3 && SEC_SERVO_TO >= 3) 
        OCR5A = servo[2];
      #endif 
      #if (PRI_SERVO_FROM <= 4 && PRI_SERVO_TO >= 4) || (SEC_SERVO_FROM <= 4 && SEC_SERVO_TO >= 4) 
        OCR1A = servo[3];
      #endif 
      #if (PRI_SERVO_FROM <= 5 && PRI_SERVO_TO >= 5) || (SEC_SERVO_FROM <= 5 && SEC_SERVO_TO >= 5) 
        OCR1B = servo[4];
      #endif 
      #if (PRI_SERVO_FROM <= 6 && PRI_SERVO_TO >= 6) || (SEC_SERVO_FROM <= 6 && SEC_SERVO_TO >= 6) 
        OCR4A = servo[5];
      #endif 
      #if (PRI_SERVO_FROM <= 7 && PRI_SERVO_TO >= 7) || (SEC_SERVO_FROM <= 7 && SEC_SERVO_TO >= 7) 
        OCR4B = servo[6];
      #endif 
      #if (PRI_SERVO_FROM <= 8 && PRI_SERVO_TO >= 8) || (SEC_SERVO_FROM <= 8 && SEC_SERVO_TO >= 8) 
        OCR4C = servo[7];
      #endif
    #endif
    // write HW PWM servos for the promicro
    #if defined(PROMICRO) && defined(A32U4_4_HW_PWM_SERVOS)
      #if (PRI_SERVO_FROM <= 7 && PRI_SERVO_TO >= 7)
        OCR1A = servo[6];// Pin 9
      #endif
      #if (PRI_SERVO_FROM <= 5 && PRI_SERVO_TO >= 5)
        OCR1B = servo[4];// Pin 10
      #endif
      #if (PRI_SERVO_FROM <= 6 && PRI_SERVO_TO >= 6)
        OCR3A = servo[5];// Pin 5
      #endif
      #if (PRI_SERVO_FROM <= 4 && PRI_SERVO_TO >= 4)
        OCR1C = servo[3];// Pin 11
      #endif
    #endif
  #endif
}

/**************************************************************************************/
/************  Writes the Motors values to the PWM compare register  ******************/
/**************************************************************************************/
void writeMotors() { // [1000;2000] => [125;250]
  /****************  Specific PWM Timers & Registers for the MEGA's   *******************/
  #if defined(MEGA)// [1000:2000] => [8000:16000] for timer 3 & 4 for mega
    #if (NUMBER_MOTOR > 0) 
      #ifndef EXT_MOTOR_RANGE 
        OCR3C = motor[0]<<3; //  pin 3
      #else
        OCR3C = ((motor[0]<<4) - 16000);
      #endif
    #endif
    #if (NUMBER_MOTOR > 1)
      #ifndef EXT_MOTOR_RANGE 
        OCR3A = motor[1]<<3; //  pin 5
      #else
        OCR3A = ((motor[1]<<4) - 16000);
      #endif
    #endif
    #if (NUMBER_MOTOR > 2)
      #ifndef EXT_MOTOR_RANGE 
        OCR4A = motor[2]<<3; //  pin 6
      #else
        OCR4A = ((motor[2]<<4) - 16000);
      #endif
    #endif
    #if (NUMBER_MOTOR > 3)
      #ifndef EXT_MOTOR_RANGE 
        OCR3B = motor[3]<<3; //  pin 2
      #else
        OCR3B = ((motor[3]<<4) - 16000);
      #endif
    #endif
    #if (NUMBER_MOTOR > 4)
      #ifndef EXT_MOTOR_RANGE 
        OCR4B = motor[4]<<3; //  pin 7
        OCR4C = motor[5]<<3; //  pin 8
      #else
        OCR4B = ((motor[4]<<4) - 16000);
        OCR4C = ((motor[5]<<4) - 16000);
      #endif
    #endif
    #if (NUMBER_MOTOR > 6)
      #ifndef EXT_MOTOR_RANGE 
        OCR2B = motor[6]>>3; //  pin 9
        OCR2A = motor[7]>>3; //  pin 10
      #else
        OCR2B = (motor[6]>>2) - 250;
        OCR2A = (motor[7]>>2) - 250;
      #endif
    #endif
  #endif

  /******** Specific PWM Timers & Registers for the atmega32u4 (Promicro)   ************/
  #if defined(PROMICRO)
    #if (NUMBER_MOTOR > 0)
      #if defined(A32U4_4_HW_PWM_SERVOS)
        // write motor0 to pin 6
        // Timer 4 A & D [1000:2000] => [1000:2000]
        #ifndef EXT_MOTOR_RANGE
          TC4H = motor[0]>>8; OCR4D = (motor[0]&0xFF); //  pin 6
        #else
          TC4H = (((motor[0]-1000)<<1)+16)>>8; OCR4D = ((((motor[0]-1000)<<1)+16)&0xFF); //  pin 6
        #endif
      #else
        // write motor0 to pin 9
        // Timer 1 A & B [1000:2000] => [8000:16000]
        #ifndef EXT_MOTOR_RANGE
          OCR1A = motor[0]<<3; //  pin 9
        #else
          OCR1A = ((motor[0]<<4) - 16000) + 128;
        #endif
      #endif
    #endif
    #if (NUMBER_MOTOR > 1)
      #ifndef EXT_MOTOR_RANGE 
        OCR1B = motor[1]<<3; //  pin 10
      #else
        OCR1B = ((motor[1]<<4) - 16000) + 128;
      #endif
    #endif
    #if (NUMBER_MOTOR > 2) // Timer 4 A & D [1000:2000] => [1000:2000]
      #if !defined(HWPWM6)
        // to write values > 255 to timer 4 A/B we need to split the bytes
        #ifndef EXT_MOTOR_RANGE 
          TC4H = (2047-motor[2])>>8; OCR4A = ((2047-motor[2])&0xFF); //  pin 5
        #else
          TC4H = 2047-(((motor[2]-1000)<<1)+16)>>8; OCR4A = (2047-(((motor[2]-1000)<<1)+16)&0xFF); //  pin 5
        #endif
      #else
        #ifndef EXT_MOTOR_RANGE 
          OCR3A = motor[2]<<3; //  pin 5
        #else
          OCR3A = ((motor[2]<<4) - 16000) + 128;
        #endif
      #endif
    #endif
    #if (NUMBER_MOTOR > 3)
      #ifndef EXT_MOTOR_RANGE 
        TC4H = motor[3]>>8; OCR4D = (motor[3]&0xFF); //  pin 6
      #else
        TC4H = (((motor[3]-1000)<<1)+16)>>8; OCR4D = ((((motor[3]-1000)<<1)+16)&0xFF); //  pin 6
      #endif
    #endif    
    #if (NUMBER_MOTOR > 4)
      #if !defined(HWPWM6)
        #if (NUMBER_MOTOR == 6) && !defined(SERVO)
          atomicPWM_PIN5_highState = motor[4]<<3;
          atomicPWM_PIN5_lowState = 16383-atomicPWM_PIN5_highState;
          atomicPWM_PIN6_highState = motor[5]<<3;
          atomicPWM_PIN6_lowState = 16383-atomicPWM_PIN6_highState;      
        #else
          atomicPWM_PIN5_highState = ((motor[4]-1000)<<4)+320;
          atomicPWM_PIN5_lowState = 15743-atomicPWM_PIN5_highState;
          atomicPWM_PIN6_highState = ((motor[5]-1000)<<4)+320;
          atomicPWM_PIN6_lowState = 15743-atomicPWM_PIN6_highState;        
        #endif
      #else
        #ifndef EXT_MOTOR_RANGE 
          OCR1C = motor[4]<<3; //  pin 11
          TC4H = motor[5]>>8; OCR4A = (motor[5]&0xFF); //  pin 13  
        #else
          OCR1C = ((motor[4]<<4) - 16000) + 128;
          TC4H = (((motor[5]-1000)<<1)+16)>>8; OCR4A = ((((motor[5]-1000)<<1)+16)&0xFF); //  pin 13       
        #endif  
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

  /********  Specific PWM Timers & Registers for the atmega328P (Promini)   ************/
  #if defined(PROMINI)
    #if (NUMBER_MOTOR > 0)
      #ifndef EXT_MOTOR_RANGE 
        OCR1A = motor[0]>>3; //  pin 9
      #else
        OCR1A = ((motor[0]>>2) - 250);
      #endif
    #endif
    #if (NUMBER_MOTOR > 1)
      #ifndef EXT_MOTOR_RANGE 
        OCR1B = motor[1]>>3; //  pin 10
      #else
        OCR1B = ((motor[1]>>2) - 250);
      #endif
    #endif
    #if (NUMBER_MOTOR > 2)
      #ifndef EXT_MOTOR_RANGE
        OCR2A = motor[2]>>3; //  pin 11
      #else
        OCR2A = ((motor[2]>>2) - 250);
      #endif
    #endif
    #if (NUMBER_MOTOR > 3)
      #ifndef EXT_MOTOR_RANGE
        OCR2B = motor[3]>>3; //  pin 3
      #else
        OCR2B = ((motor[3]>>2) - 250);
      #endif
    #endif
    #if (NUMBER_MOTOR > 4)
      #if (NUMBER_MOTOR == 6) && !defined(SERVO)
        #ifndef EXT_MOTOR_RANGE 
          atomicPWM_PIN6_highState = motor[4]>>3;
          atomicPWM_PIN5_highState = motor[5]>>3;
        #else
          atomicPWM_PIN6_highState = (motor[4]>>2) - 250;
          atomicPWM_PIN5_highState = (motor[5]>>2) - 250;
        #endif
        atomicPWM_PIN6_lowState  = 255-atomicPWM_PIN6_highState;
        atomicPWM_PIN5_lowState  = 255-atomicPWM_PIN5_highState; 
      #else //note: EXT_MOTOR_RANGE not possible here
        atomicPWM_PIN6_highState = ((motor[4]-1000)>>2)+5;
        atomicPWM_PIN6_lowState  = 245-atomicPWM_PIN6_highState;
        atomicPWM_PIN5_highState = ((motor[5]-1000)>>2)+5;
        atomicPWM_PIN5_lowState  = 245-atomicPWM_PIN5_highState;
      #endif
    #endif
    #if (NUMBER_MOTOR > 6) //note: EXT_MOTOR_RANGE not possible here
      atomicPWM_PINA2_highState = ((motor[6]-1000)>>2)+5;
      atomicPWM_PINA2_lowState  = 245-atomicPWM_PINA2_highState;
      atomicPWM_PIN12_highState = ((motor[7]-1000)>>2)+5;
      atomicPWM_PIN12_lowState  = 245-atomicPWM_PIN12_highState;
    #endif
  #endif
}

/**************************************************************************************/
/************          Writes the mincommand to all Motors           ******************/
/**************************************************************************************/
void writeAllMotors(int16_t mc) {   // Sends commands to all motors
  for (uint8_t i =0;i<NUMBER_MOTOR;i++) {
    motor[i]=mc;
  }
  writeMotors();
}

/**************************************************************************************/
/************        Initialize the PWM Timers and Registers         ******************/
/**************************************************************************************/
void initOutput() {
  /****************            mark all PWM pins as Output             ******************/
  for(uint8_t i=0;i<NUMBER_MOTOR;i++) {
    pinMode(PWM_PIN[i],OUTPUT);
  }
    
  /****************  Specific PWM Timers & Registers for the MEGA's    ******************/
  #if defined(MEGA)
    #if (NUMBER_MOTOR > 0)
      // init 16bit timer 3
      TCCR3A |= (1<<WGM31); // phase correct mode
      TCCR3A &= ~(1<<WGM30);
      TCCR3B |= (1<<WGM33);
      TCCR3B &= ~(1<<CS31); // no prescaler
      ICR3   |= 0x3FFF; // TOP to 16383;      
      
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
      ICR4   |= 0x3FFF; // TOP to 16383;    
      
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
  
  /******** Specific PWM Timers & Registers for the atmega32u4 (Promicro)   ************/
  #if defined(PROMICRO)
    #if (NUMBER_MOTOR > 0) && ( !defined(A32U4_4_HW_PWM_SERVOS) )
      TCCR1A |= (1<<WGM11); // phase correct mode & no prescaler
      TCCR1A &= ~(1<<WGM10);
      TCCR1B &= ~(1<<WGM12) &  ~(1<<CS11) & ~(1<<CS12);
      TCCR1B |= (1<<WGM13) | (1<<CS10); 
      ICR1   |= 0x3FFF; // TOP to 16383;     
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
        TCCR3A |= (1<<WGM31); // phase correct mode & no prescaler
        TCCR3A &= ~(1<<WGM30);
        TCCR3B &= ~(1<<WGM32) &  ~(1<<CS31) & ~(1<<CS32);
        TCCR3B |= (1<<WGM33) | (1<<CS30); 
        ICR3   |= 0x3FFF; // TOP to 16383;     
        TCCR3A |= _BV(COM3A1); // connect pin 5 to timer 3 channel A    
      #endif 
    #endif
    #if (NUMBER_MOTOR > 3) || ( (NUMBER_MOTOR > 0) && defined(A32U4_4_HW_PWM_SERVOS) )
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
  
  /********  Specific PWM Timers & Registers for the atmega328P (Promini)   ************/
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
    #if (NUMBER_MOTOR > 4)  // PIN 5 & 6 or A0 & A1
      initializeSoftPWM();
      #if defined(A0_A1_PIN_HEX) || (NUMBER_MOTOR > 6)
        pinMode(5,INPUT);pinMode(6,INPUT);     // we reactivate the INPUT affectation for these two PINs
        pinMode(A0,OUTPUT);pinMode(A1,OUTPUT);
      #endif
    #endif
  #endif

  /********  special version of MultiWii to calibrate all attached ESCs ************/
  #if defined(ESC_CALIB_CANNOT_FLY)
    writeAllMotors(ESC_CALIB_HIGH);
    blinkLED(2,20, 2);
    delay(4000);
    writeAllMotors(ESC_CALIB_LOW);
    blinkLED(3,20, 2);
    while (1) {
      delay(5000);
      blinkLED(4,20, 2);
    #if defined(BUZZER)
      alarmArray[7] = 2;
    #endif
    }
    exit; // statement never reached
  #endif
  
  writeAllMotors(MINCOMMAND);
  delay(300);
  #if defined(SERVO)
    initializeServo();
  #endif
}


#if defined(SERVO)
/**************************************************************************************/
/************                Initialize the PWM Servos               ******************/
/**************************************************************************************/
void initializeServo() {
  #if !(defined(MEGA) && defined(MEGA_HW_PWM_SERVOS))  &&  !(defined(PROMICRO) && defined(A32U4_4_HW_PWM_SERVOS))
  // do pins init if not with one of (mega & HW PWMs) or (promicro & HW PWMs)
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
  #endif

  #if defined(SERVO_1_HIGH)
    #if defined(PROMINI) || (defined(PROMICRO) && defined(HWPWM6)) // uses timer 0 Comperator A (8 bit)
      TCCR0A = 0; // normal counting mode
      TIMSK0 |= (1<<OCIE0A); // Enable CTC interrupt
      #define SERVO_ISR TIMER0_COMPA_vect
      #define SERVO_CHANNEL OCR0A
      #define SERVO_1K_US 250
    #endif
    #if (defined(PROMICRO) && !defined(HWPWM6)) // uses timer 3 Comperator A (11 bit)
      TCCR3A &= ~(1<<WGM30) & ~(1<<WGM31); //normal counting & no prescaler
      TCCR3B &= ~(1<<WGM32) & ~(1<<CS31) & ~(1<<CS32) & ~(1<<WGM33);
      TCCR3B |= (1<<CS30);   
      TIMSK3 |= (1<<OCIE3A); // Enable CTC interrupt 
      #define SERVO_ISR TIMER3_COMPA_vect
      #define SERVO_CHANNEL OCR3A
      #define SERVO_1K_US 16000 
    #endif
    #if defined(MEGA) // uses timer 5 Comperator A (11 bit)
      TCCR5A &= ~(1<<WGM50) & ~(1<<WGM51); //normal counting & no prescaler
      TCCR5B &= ~(1<<WGM52) & ~(1<<CS51) & ~(1<<CS52) & ~(1<<WGM53);
      TCCR5B |= (1<<CS50);   
      TIMSK5 |= (1<<OCIE5A); // Enable CTC interrupt  
      #define SERVO_ISR TIMER5_COMPA_vect
      #define SERVO_CHANNEL OCR5A
      #define SERVO_1K_US 16000 
    #endif
  #endif

  #if defined(MEGA) && defined(MEGA_HW_PWM_SERVOS)
    #if defined(SERVO_RFR_RATE)
      #if (SERVO_RFR_RATE < 20)
        #define SERVO_RFR_RATE 20
      #endif
      #if (SERVO_RFR_RATE > 400)
        #define SERVO_RFR_RATE 400
      #endif
    #else
      #if defined(SERVO_RFR_50HZ)
        #define SERVO_RFR_RATE 50
      #elif defined(SERVO_RFR_160HZ)
        #define SERVO_RFR_RATE 160
      #elif defined(SERVO_RFR_300HZ)
        #define SERVO_RFR_RATE 300
      #endif
    #endif  
    #define SERVO_TOP_VAL (uint16_t)(1000000L / SERVO_RFR_RATE)
    // init Timer 5, 1 and 4 of the mega for hw PWM
    TIMSK5 &= ~(1<<OCIE5A); // Disable software PWM  
    #if (PRI_SERVO_TO >= 1) || (SEC_SERVO_TO >= 1)
      TCCR5A |= (1<<WGM51);   // phase correct mode & prescaler to 8 = 1us resolution
      TCCR5A &= ~(1<<WGM50);
      TCCR5B &= ~(1<<WGM52) &  ~(1<<CS50) & ~(1<<CS52);
      TCCR5B |= (1<<WGM53) | (1<<CS51);
      ICR5 = SERVO_TOP_VAL;
      #if (PRI_SERVO_FROM == 1 || SEC_SERVO_FROM == 1) 
        pinMode(44,OUTPUT);
        TCCR5A |= (1<<COM5C1); // pin 44
      #endif
      #if (PRI_SERVO_FROM <= 2 && PRI_SERVO_TO >= 2) || (SEC_SERVO_FROM <= 2 && SEC_SERVO_TO >= 2) 
        pinMode(45,OUTPUT);
        TCCR5A |= (1<<COM5B1); // pin 45
      #endif
      #if (PRI_SERVO_FROM <= 3 && PRI_SERVO_TO >= 3) || (SEC_SERVO_FROM <= 3 && SEC_SERVO_TO >= 3) 
        pinMode(46,OUTPUT);
        TCCR5A |= (1<<COM5A1); // pin 46
      #endif
    #endif
    #if (PRI_SERVO_TO >= 4) || (SEC_SERVO_TO >= 4) 
      TCCR1A |= (1<<WGM11); // phase correct mode & prescaler to 8
      TCCR1A &= ~(1<<WGM10);
      TCCR1B &= ~(1<<WGM12) &  ~(1<<CS10) & ~(1<<CS12);
      TCCR1B |= (1<<WGM13) | (1<<CS11);
      ICR1 = SERVO_TOP_VAL;
      #if (PRI_SERVO_FROM <= 4 && PRI_SERVO_TO >= 4) || (SEC_SERVO_FROM <= 4 && SEC_SERVO_TO >= 4) 
        pinMode(11, OUTPUT);
        TCCR1A |= (1<<COM1A1); // pin 11
      #endif
      #if (PRI_SERVO_FROM <= 5 && PRI_SERVO_TO >= 5) || (SEC_SERVO_FROM <= 5 && SEC_SERVO_TO >= 5) 
        pinMode(12,OUTPUT);
        TCCR1A |= (1<<COM1B1); // pin 12
      #endif
    #endif
    #if (PRI_SERVO_TO >= 6) || (SEC_SERVO_TO >= 6) 
      // init 16bit timer 4
      TCCR4A |= (1<<WGM41); // phase correct mode
      TCCR4A &= ~(1<<WGM40);
      TCCR4B &= ~(1<<WGM42) &  ~(1<<CS40) & ~(1<<CS42);
      TCCR4B |= (1<<WGM43) | (1<<CS41);
      ICR4 = SERVO_TOP_VAL;
      #if (PRI_SERVO_FROM <= 6 && PRI_SERVO_TO >= 6) || (SEC_SERVO_FROM <= 6 && SEC_SERVO_TO >= 6) 
        pinMode(6,OUTPUT);
        TCCR4A |= _BV(COM4A1); // connect pin 6 to timer 4 channel A
      #endif
      #if (PRI_SERVO_FROM <= 7 && PRI_SERVO_TO >= 7) || (SEC_SERVO_FROM <= 7 && SEC_SERVO_TO >= 7) 
        pinMode(7,OUTPUT);
        TCCR4A |= _BV(COM4B1); // connect pin 7 to timer 4 channel B
      #endif
      #if (PRI_SERVO_FROM <= 8 && PRI_SERVO_TO >= 8) || (SEC_SERVO_FROM <= 8 && SEC_SERVO_TO >= 8) 
        #if defined(AIRPLANE) || defined(HELICOPTER)
          servo[7] =  MINCOMMAND;    // Trhottle at minimum for airplane and heli
          OCR4C = MINCOMMAND;
        #endif  
        pinMode(8,OUTPUT);
        TCCR4A |= _BV(COM4C1); // connect pin 8 to timer 4 channel C
      #endif
    #endif 
  #endif // mega hw pwm

  #if defined(PROMICRO) && defined(A32U4_4_HW_PWM_SERVOS)
    // atm. always initialize 4 servos to pins 9, 10, 11, 5
    TIMSK1 &= ~(1<<OCIE1A) & ~(1<<OCIE1B) & ~(1<<OCIE1C);
    TCCR1A |= (1<<WGM11); // phase correct mode & prescaler to 8
    TCCR1A &= ~(1<<WGM10);
    TCCR1B &= ~(1<<WGM12) &  ~(1<<CS10) & ~(1<<CS12);
    TCCR1B |= (1<<WGM13) | (1<<CS11);
    pinMode(9,OUTPUT);
    TCCR1A |= (1<<COM1A1); // pin 9
    pinMode(10,OUTPUT);
    TCCR1A |= (1<<COM1B1); // pin 10
    pinMode(11,OUTPUT);
    TCCR1A |= (1<<COM1C1); // pin 11

    TCCR3A |= (1<<WGM31); // phase correct mode & prescaler to 8
    TCCR3A &= ~(1<<WGM30);
    TCCR3B &= ~(1<<WGM32) &  ~(1<<CS30) & ~(1<<CS32);
    TCCR3B |= (1<<WGM33) | (1<<CS31);
    pinMode(5,OUTPUT);
    TCCR3A |= (1<<COM3A1); // pin 5
    #if defined(SERVO_RFR_RATE)
      #if (SERVO_RFR_RATE < 50) || (SERVO_RFR_RATE > 400)
        #error "* invalid SERVO_RFR_RATE specified"
      #endif
      #define SERVO_TOP_VAL (uint16_t)(1000000L / SERVO_RFR_RATE)
    #elif defined(SERVO_RFR_50HZ)
      #define SERVO_TOP_VAL 16700
    #elif defined(SERVO_RFR_160HZ)
      #define SERVO_TOP_VAL 6200
    #elif defined(SERVO_RFR_300HZ)
      #define SERVO_TOP_VAL 3300
    #else
      #error "* must set SERVO_RFR_RATE or one of the fixed refresh rates of 50, 160 or 300 Hz"
    #endif
    #if defined(SERVO_PIN5_RFR_RATE)
      #if (SERVO_PIN5_RFR_RATE < 50) || (SERVO_PIN5_RFR_RATE > 400)
        #error "* invalid SERVO_PIN5_RFR_RATE specified"
      #endif
      #define SERVO_PIN5_TOP_VAL (uint16_t)(1000000L / SERVO_PIN5_RFR_RATE)
    #else
      #define SERVO_PIN5_TOP_VAL SERVO_TOP_VAL
    #endif
    ICR1   = SERVO_TOP_VAL;      // set TOP timer 1
    ICR3   = SERVO_PIN5_TOP_VAL; // set TOP timer 3
  #endif // promicro hw pwm
}

/**************************************************************************************/
/************              Servo software PWM generation             ******************/
/**************************************************************************************/

// prescaler is set by default to 64 on Timer0
// Duemilanove : 16MHz / 64 => 4 us
// 256 steps = 1 counter cycle = 1024 us

// for servo 2-8
// its almost the same as for servo 1
#if defined(SERVO_1_HIGH) && !defined(A32U4_4_HW_PWM_SERVOS)
  #define SERVO_PULSE(PIN_HIGH,ACT_STATE,SERVO_NUM,LAST_PIN_LOW) \
    }else if(state == ACT_STATE){                                \
      LAST_PIN_LOW;                                              \
      PIN_HIGH;                                                  \
      SERVO_CHANNEL+=SERVO_1K_US;                                \
      state++;                                                   \
    }else if(state == ACT_STATE+1){                              \
      SERVO_CHANNEL+=atomicServo[SERVO_NUM];                     \
      state++;                                                   \
  
  ISR(SERVO_ISR) {
    static uint8_t state = 0; // indicates the current state of the chain
    if(state == 0){
      SERVO_1_HIGH; // set servo 1's pin high 
      SERVO_CHANNEL+=SERVO_1K_US; // wait 1000us
      state++; // count up the state
    }else if(state==1){
      SERVO_CHANNEL+=atomicServo[SERVO_1_ARR_POS]; // load the servo's value (0-1000us)
      state++; // count up the state
    #if defined(SERVO_2_HIGH)
      SERVO_PULSE(SERVO_2_HIGH,2,SERVO_2_ARR_POS,SERVO_1_LOW); // the same here
    #endif
    #if defined(SERVO_3_HIGH)
      SERVO_PULSE(SERVO_3_HIGH,4,SERVO_3_ARR_POS,SERVO_2_LOW);
    #endif
    #if defined(SERVO_4_HIGH)
      SERVO_PULSE(SERVO_4_HIGH,6,SERVO_4_ARR_POS,SERVO_3_LOW);
    #endif
    #if defined(SERVO_5_HIGH)
      SERVO_PULSE(SERVO_5_HIGH,8,SERVO_5_ARR_POS,SERVO_4_LOW);
    #endif
    #if defined(SERVO_6_HIGH)
      SERVO_PULSE(SERVO_6_HIGH,10,SERVO_6_ARR_POS,SERVO_5_LOW);
    #endif
    #if defined(SERVO_7_HIGH)
      SERVO_PULSE(SERVO_7_HIGH,12,SERVO_7_ARR_POS,SERVO_6_LOW);
    #endif
    #if defined(SERVO_8_HIGH)
      SERVO_PULSE(SERVO_8_HIGH,14,SERVO_8_ARR_POS,SERVO_7_LOW);
    #endif
    }else{
      LAST_LOW;
      #if defined(SERVO_RFR_300HZ)
        #if defined(SERVO_3_HIGH)  // if there are 3 or more servos we dont need to slow it down
          SERVO_CHANNEL+=(SERVO_1K_US>>3); // 0 would be better but it causes bad jitter
          state=0; 
        #else // if there are less then 3 servos we need to slow it to not go over 300Hz (the highest working refresh rate for the digital servos for what i know..)
          SERVO_CHANNEL+=SERVO_1K_US;
          if(state<4){
            state+=2;
          }else{
            state=0;
          }
        #endif
      #endif
      #if defined(SERVO_RFR_160HZ)
        #if defined(SERVO_4_HIGH)  // if there are 4 or more servos we dont need to slow it down
          SERVO_CHANNEL+=(SERVO_1K_US>>3); // 0 would be better but it causes bad jitter
          state=0; 
        #else // if there are less then 4 servos we need to slow it to not go over ~170Hz (the highest working refresh rate for analog servos)
          SERVO_CHANNEL+=SERVO_1K_US;
          if(state<8){
            state+=2;
          }else{
            state=0;
          }
        #endif
      #endif   
      #if defined(SERVO_RFR_50HZ) // to have ~ 50Hz for all servos
        SERVO_CHANNEL+=SERVO_1K_US;
        if(state<30){
          state+=2;
        }else{
          state=0;
        }     
      #endif
    }
  } 
  #endif
#endif

/**************************************************************************************/
/************             Motor software PWM generation              ******************/
/**************************************************************************************/
// SW PWM is only used if there are not enough HW PWM pins (for exampe hexa on a promini)

#if (NUMBER_MOTOR > 4) && (defined(PROMINI) || defined(PROMICRO))

  /****************    Pre define the used ISR's and Timerchannels     ******************/
  #if !defined(PROMICRO)
    #define SOFT_PWM_ISR1 TIMER0_COMPB_vect
    #define SOFT_PWM_ISR2 TIMER0_COMPA_vect
    #define SOFT_PWM_CHANNEL1 OCR0B
    #define SOFT_PWM_CHANNEL2 OCR0A 
  #elif !defined(HWPWM6)
    #define SOFT_PWM_ISR1 TIMER3_COMPB_vect
    #define SOFT_PWM_ISR2 TIMER3_COMPC_vect
    #define SOFT_PWM_CHANNEL1 OCR3B
    #define SOFT_PWM_CHANNEL2 OCR3C 
  #else
    #define SOFT_PWM_ISR2 TIMER0_COMPB_vect  
    #define SOFT_PWM_CHANNEL2 OCR0B 
  #endif
  
  /****************         Initialize Timers and PWM Channels         ******************/
  void initializeSoftPWM() {
    #if !defined(PROMICRO)
      TCCR0A = 0; // normal counting mode
      #if (NUMBER_MOTOR > 4) && !defined(HWPWM6) 
        TIMSK0 |= (1<<OCIE0B); // Enable CTC interrupt  
      #endif
      #if (NUMBER_MOTOR > 6) || ((NUMBER_MOTOR == 6) && !defined(SERVO))
        TIMSK0 |= (1<<OCIE0A);
      #endif
    #else
      #if !defined(HWPWM6)
        TCCR3A &= ~(1<<WGM30) & ~(1<<WGM31); //normal counting & no prescaler
        TCCR3B &= ~(1<<WGM32) & ~(1<<CS31) & ~(1<<CS32) & ~(1<<WGM33);
        TCCR3B |= (1<<CS30);   
        TIMSK3 |= (1<<OCIE3B); // Enable CTC interrupt  
        #if (NUMBER_MOTOR > 6) || ((NUMBER_MOTOR == 6) && !defined(SERVO))
          TIMSK3 |= (1<<OCIE3C);
        #endif   
      #else
        TCCR0A = 0; // normal counting mode
        TIMSK0 |= (1<<OCIE0B); // Enable CTC interrupt 
      #endif
    #endif
  }
  
  /****************               Motor SW PWM ISR's                 ******************/
  // hexa with old but sometimes better SW PWM method
  // for setups without servos
  #if (NUMBER_MOTOR == 6) && (!defined(SERVO) && !defined(HWPWM6))
    ISR(SOFT_PWM_ISR1) { 
      static uint8_t state = 0;
      if(state == 0){
        if (atomicPWM_PIN5_highState>0) SOFT_PWM_1_PIN_HIGH;
        SOFT_PWM_CHANNEL1 += atomicPWM_PIN5_highState;
        state = 1;
      }else if(state == 1){
        SOFT_PWM_CHANNEL1 += atomicPWM_PIN5_highState;
        state = 2;
      }else if(state == 2){
        SOFT_PWM_1_PIN_LOW;
        SOFT_PWM_CHANNEL1 += atomicPWM_PIN5_lowState;
        state = 3;  
      }else if(state == 3){
        SOFT_PWM_CHANNEL1 += atomicPWM_PIN5_lowState;
        state = 0;   
      }
    }
    ISR(SOFT_PWM_ISR2) { 
      static uint8_t state = 0;
      if(state == 0){
        if (atomicPWM_PIN6_highState>0) SOFT_PWM_2_PIN_HIGH;
        SOFT_PWM_CHANNEL2 += atomicPWM_PIN6_highState;
        state = 1;
      }else if(state == 1){
        SOFT_PWM_CHANNEL2 += atomicPWM_PIN6_highState;
        state = 2;
      }else if(state == 2){
        SOFT_PWM_2_PIN_LOW;
        SOFT_PWM_CHANNEL2 += atomicPWM_PIN6_lowState;
        state = 3;  
      }else if(state == 3){
        SOFT_PWM_CHANNEL2 += atomicPWM_PIN6_lowState;
        state = 0;   
      }
    }
  #else
    #if (NUMBER_MOTOR > 4) && !defined(HWPWM6)
      // HEXA with just OCR0B 
      ISR(SOFT_PWM_ISR1) { 
        static uint8_t state = 0;
        if(state == 0){
          SOFT_PWM_1_PIN_HIGH;
          SOFT_PWM_CHANNEL1 += atomicPWM_PIN5_highState;
          state = 1;
        }else if(state == 1){
          SOFT_PWM_2_PIN_LOW;
          SOFT_PWM_CHANNEL1 += atomicPWM_PIN6_lowState;
          state = 2;
        }else if(state == 2){
          SOFT_PWM_2_PIN_HIGH;
          SOFT_PWM_CHANNEL1 += atomicPWM_PIN6_highState;
          state = 3;  
        }else if(state == 3){
          SOFT_PWM_1_PIN_LOW;
          SOFT_PWM_CHANNEL1 += atomicPWM_PIN5_lowState;
          state = 0;   
        }
      } 
    #endif
    //the same with digital PIN A2 & 12 OCR0A counter for OCTO
    #if (NUMBER_MOTOR > 6)
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
#endif

/**************************************************************************************/
/********** Mixes the Computed stabilize values to the Motors & Servos  ***************/
/**************************************************************************************/

// get servo middle point from Config or from RC-Data
int16_t get_middle(uint8_t nr) {
  return (conf.servoConf[nr].middle < RC_CHANS) ? rcData[conf.servoConf[nr].middle] : conf.servoConf[nr].middle;
}

// int8_t servodir(uint8_t n, uint8_t b) { return ((conf.servoConf[n].rate & b) ? -1 : 1) ; }

void mixTable() {
  int16_t maxMotor;
  uint8_t i;
  #if defined(DYNBALANCE)
    return;
  #endif
  #define PIDMIX(X,Y,Z) rcCommand[THROTTLE] + axisPID[ROLL]*X + axisPID[PITCH]*Y + YAW_DIRECTION * axisPID[YAW]*Z
  #define SERVODIR(n,b) ((conf.servoConf[n].rate & b) ? -1 : 1)
  
  #if NUMBER_MOTOR > 3
    //prevent "yaw jump" during yaw correction
    axisPID[YAW] = constrain(axisPID[YAW],-100-abs(rcCommand[YAW]),+100+abs(rcCommand[YAW]));
  #endif
  /****************                   main Mix Table                ******************/
  #if defined( MY_PRIVATE_MIXING )
    #include MY_PRIVATE_MIXING
  #elif defined( BI )
    motor[0] = PIDMIX(+1, 0, 0); //LEFT
    motor[1] = PIDMIX(-1, 0, 0); //RIGHT
    servo[4] = (SERVODIR(4,2) * axisPID[YAW]) + (SERVODIR(4,1) * axisPID[PITCH]) + get_middle(4); //LEFT
    servo[5] = (SERVODIR(5,2) * axisPID[YAW]) + (SERVODIR(5,1) * axisPID[PITCH]) + get_middle(5); //RIGHT
  #elif defined( TRI )
    motor[0] = PIDMIX( 0,+4/3, 0); //REAR
    motor[1] = PIDMIX(-1,-2/3, 0); //RIGHT
    motor[2] = PIDMIX(+1,-2/3, 0); //LEFT
    servo[5] = (SERVODIR(5, 1) * axisPID[YAW]) + get_middle(5); //REAR
  #elif defined( QUADP )
    motor[0] = PIDMIX( 0,+1,-1); //REAR
    motor[1] = PIDMIX(-1, 0,+1); //RIGHT
    motor[2] = PIDMIX(+1, 0,+1); //LEFT
    motor[3] = PIDMIX( 0,-1,-1); //FRONT
  #elif defined( QUADX )
    motor[0] = PIDMIX(-1,+1,-1); //REAR_R
    motor[1] = PIDMIX(-1,-1,+1); //FRONT_R
    motor[2] = PIDMIX(+1,+1,+1); //REAR_L
    motor[3] = PIDMIX(+1,-1,-1); //FRONT_L
  #elif defined( Y4 )
    motor[0] = PIDMIX(+0,+1,-1);   //REAR_1 CW
    motor[1] = PIDMIX(-1,-1, 0); //FRONT_R CCW
    motor[2] = PIDMIX(+0,+1,+1);   //REAR_2 CCW
    motor[3] = PIDMIX(+1,-1, 0); //FRONT_L CW
  #elif defined( Y6 )
    motor[0] = PIDMIX(+0,+4/3,+1); //REAR
    motor[1] = PIDMIX(-1,-2/3,-1); //RIGHT
    motor[2] = PIDMIX(+1,-2/3,-1); //LEFT
    motor[3] = PIDMIX(+0,+4/3,-1); //UNDER_REAR
    motor[4] = PIDMIX(-1,-2/3,+1); //UNDER_RIGHT
    motor[5] = PIDMIX(+1,-2/3,+1); //UNDER_LEFT
  #elif defined( HEX6 )
    motor[0] = PIDMIX(-7/8,+1/2,+1); //REAR_R
    motor[1] = PIDMIX(-7/8,-1/2,-1); //FRONT_R
    motor[2] = PIDMIX(+7/8,+1/2,+1); //REAR_L
    motor[3] = PIDMIX(+7/8,-1/2,-1); //FRONT_L
    motor[4] = PIDMIX(+0  ,-1  ,+1); //FRONT
    motor[5] = PIDMIX(+0  ,+1  ,-1); //REAR
  #elif defined( HEX6X )
    motor[0] = PIDMIX(-1/2,+7/8,+1); //REAR_R
    motor[1] = PIDMIX(-1/2,-7/8,+1); //FRONT_R
    motor[2] = PIDMIX(+1/2,+7/8,-1); //REAR_L
    motor[3] = PIDMIX(+1/2,-7/8,-1); //FRONT_L
    motor[4] = PIDMIX(-1  ,+0  ,-1); //RIGHT
    motor[5] = PIDMIX(+1  ,+0  ,+1); //LEFT
  #elif defined( HEX6H )
    motor[0] = PIDMIX(-1,+1,-1); //REAR_R
    motor[1] = PIDMIX(-1,-1,+1); //FRONT_R
    motor[2] = PIDMIX(+ 1,+1,+1); //REAR_L
    motor[3] = PIDMIX(+ 1,-1,-1); //FRONT_L
    motor[4] = PIDMIX(0 ,0 ,0); //RIGHT
    motor[5] = PIDMIX(0 ,0 ,0); //LEFT
  #elif defined( OCTOX8 )
    motor[0] = PIDMIX(-1,+1,-1); //REAR_R
    motor[1] = PIDMIX(-1,-1,+1); //FRONT_R
    motor[2] = PIDMIX(+1,+1,+1); //REAR_L
    motor[3] = PIDMIX(+1,-1,-1); //FRONT_L
    motor[4] = PIDMIX(-1,+1,+1); //UNDER_REAR_R
    motor[5] = PIDMIX(-1,-1,-1); //UNDER_FRONT_R
    motor[6] = PIDMIX(+1,+1,-1); //UNDER_REAR_L
    motor[7] = PIDMIX(+1,-1,+1); //UNDER_FRONT_L
  #elif defined( OCTOFLATP )
    motor[0] = PIDMIX(+7/10,-7/10,+1); //FRONT_L
    motor[1] = PIDMIX(-7/10,-7/10,+1); //FRONT_R
    motor[2] = PIDMIX(-7/10,+7/10,+1); //REAR_R
    motor[3] = PIDMIX(+7/10,+7/10,+1); //REAR_L
    motor[4] = PIDMIX(+0   ,-1   ,-1); //FRONT
    motor[5] = PIDMIX(-1   ,+0   ,-1); //RIGHT
    motor[6] = PIDMIX(+0   ,+1   ,-1); //REAR
    motor[7] = PIDMIX(+1   ,+0   ,-1); //LEFT
  #elif defined( OCTOFLATX )
    motor[0] = PIDMIX(+1  ,-1/2,+1); //MIDFRONT_L
    motor[1] = PIDMIX(-1/2,-1  ,+1); //FRONT_R
    motor[2] = PIDMIX(-1  ,+1/2,+1); //MIDREAR_R
    motor[3] = PIDMIX(+1/2,+1  ,+1); //REAR_L
    motor[4] = PIDMIX(+1/2,-1  ,-1); //FRONT_L
    motor[5] = PIDMIX(-1  ,-1/2,-1); //MIDFRONT_R
    motor[6] = PIDMIX(-1/2,+1  ,-1); //REAR_R
    motor[7] = PIDMIX(+1  ,+1/2,-1); //MIDREAR_L
  #elif defined( VTAIL4 )
    motor[0] = PIDMIX(+0,+1, +1); //REAR_R
    motor[1] = PIDMIX(-1, -1, +0); //FRONT_R
    motor[2] = PIDMIX(+0,+1, -1); //REAR_L
    motor[3] = PIDMIX(+1, -1, -0); //FRONT_L
  #elif defined( FLYING_WING )
    /*****************************             FLYING WING                **************************************/
    if (!f.ARMED) {
      servo[7] = MINCOMMAND;  // Kill throttle when disarmed
    } else {
      servo[7] = constrain(rcCommand[THROTTLE], conf.minthrottle, MAXTHROTTLE);
    }
    motor[0] = servo[7];
    if (f.PASSTHRU_MODE) {    // do not use sensors for correction, simple 2 channel mixing
      servo[3] = (SERVODIR(3,1) * rcCommand[PITCH]) + (SERVODIR(3,2) * rcCommand[ROLL]);
      servo[4] = (SERVODIR(4,1) * rcCommand[PITCH]) + (SERVODIR(4,2) * rcCommand[ROLL]);
    } else {                  // use sensors to correct (gyro only or gyro+acc according to aux1/aux2 configuration
      servo[3] = (SERVODIR(3,1) * axisPID[PITCH])   + (SERVODIR(3,2) * axisPID[ROLL]);
      servo[4] = (SERVODIR(4,1) * axisPID[PITCH])   + (SERVODIR(4,2) * axisPID[ROLL]);
    }
    servo[3] += get_middle(3);
    servo[4] += get_middle(4);
  #elif defined( AIRPLANE )
    /*****************************               AIRPLANE                **************************************/
    // servo[7] is programmed with safty features to avoid motorstarts when ardu reset..
    // All other servos go to center at reset..  Half throttle can be dangerus
    // Only use servo[7] as motorcontrol if motor is used in the setup            */
    if (!f.ARMED) {
      servo[7] = MINCOMMAND; // Kill throttle when disarmed
    } else {
      servo[7] = constrain(rcCommand[THROTTLE], conf.minthrottle, MAXTHROTTLE);
    }
    motor[0] = servo[7];

    // Flapperon Controll TODO - optimalisation
    int16_t flapperons[2]={0,0};
    #if  defined(FLAPPERONS) && defined(FLAPPERON_EP)
      int8_t flapinv[2] = FLAPPERON_INVERT;
      static int16_t F_Endpoint[2] = FLAPPERON_EP;
      int16_t flap =MIDRC-constrain(rcData[FLAPPERONS],F_Endpoint[0],F_Endpoint[1]);
      static int16_t slowFlaps= flap;
      #if defined(FLAPSPEED)
        if (slowFlaps < flap ){slowFlaps+=FLAPSPEED;}else if(slowFlaps > flap){slowFlaps-=FLAPSPEED;}
      #else
        slowFlaps = flap;
      #endif
      flap = MIDRC-(constrain(MIDRC-slowFlaps,F_Endpoint[0],F_Endpoint[1]));
    for(i=0; i<2; i++){flapperons[i] = flap * flapinv[i] ;}
    #endif

    // Traditional Flaps on SERVO3
    #if defined(FLAPS)
      // configure SERVO3 middle point in GUI to using an AUX channel for FLAPS control
      // use servo min, servo max and servo rate for propper endpoints adjust
      int16_t lFlap = get_middle(2);
      lFlap = constrain(lFlap, conf.servoConf[2].min, conf.servoConf[2].max);
      lFlap = MIDRC - lFlap;
      static int16_t slow_LFlaps= lFlap;
      #if defined(FLAPSPEED)
        if (slow_LFlaps < lFlap ){slow_LFlaps+=FLAPSPEED;} else if(slow_LFlaps > lFlap){slow_LFlaps-=FLAPSPEED;}
      #else
        slow_LFlaps = lFlap;
      #endif
      servo[2] = ((int32_t)conf.servoConf[2].rate * slow_LFlaps)/100L;
      servo[2] += MIDRC;
    #endif

    if(f.PASSTHRU_MODE){   // Direct passthru from RX
      servo[3] = rcCommand[ROLL] + flapperons[0];     //   Wing 1
      servo[4] = rcCommand[ROLL] + flapperons[1];     //   Wing 2
      servo[5] = rcCommand[YAW];                      //   Rudder
      servo[6] = rcCommand[PITCH];                    //   Elevator
    }else{
      // Assisted modes (gyro only or gyro+acc according to AUX configuration in Gui
      servo[3] = axisPID[ROLL] + flapperons[0];   //   Wing 1
      servo[4] = axisPID[ROLL] + flapperons[1];   //   Wing 2
      servo[5] = axisPID[YAW];                    //   Rudder
      servo[6] = axisPID[PITCH];                  //   Elevator
    }
    for(i=3;i<7;i++) {
      servo[i]  = ((int32_t)conf.servoConf[i].rate * servo[i])/100L;  // servo rates
      servo[i] += get_middle(i);
    }
  #elif defined( SINGLECOPTER )
    /***************************          Single & DualCopter          ******************************/
    // Singlecopter
    // This is a beta requested by  xuant9
    // Assisted modes (gyro only or gyro+acc according to AUX configuration in Gui
    // http://www.kkmulticopter.kr/multicopter/images/blue_single.jpg
    //servo[3]  = (axisPID[YAW]*SERVODIR(3,2)) + (axisPID[PITCH]*SERVODIR(3,1));   //   SideServo  5  D12
    //servo[4]  = (axisPID[YAW]*SERVODIR(4,2)) + (axisPID[PITCH]*SERVODIR(4,1));   //   SideServo  3  D11
    //servo[5]  = (axisPID[YAW]*SERVODIR(5,2)) + (axisPID[ROLL] *SERVODIR(5,1));   //   FrontServo 2  D3
    //servo[6]  = (axisPID[YAW]*SERVODIR(6,2)) + (axisPID[ROLL] *SERVODIR(6,1));   //   RearServo  4  D10
    for(i=3;i<7;i++) {
      servo[i]  = (axisPID[YAW] * SERVODIR(i,2)) + (axisPID[(6-i)>>1] * SERVODIR(i,1));  // mix and setup direction
      servo[i] += get_middle(i);
    }
    motor[0] = rcCommand[THROTTLE];
  #elif defined( DUALCOPTER )
    // Dualcopter
    // This is a beta requested by  xuant9
    // Assisted modes (gyro only or gyro+acc according to AUX configuration in Gui
    //http://www.kkmulticopter.kr/products_pic/index.html?sn=multicopter_v02_du&name=KKMultiCopter%20Flight%20Controller%20Blackboard%3Cbr%3E
    //servo[4] = ((int32_t)conf.servoConf[4].rate * axisPID[PITCH])/100L;  //  PITCHServo   D11 => Wing2 servo
    //servo[5] = ((int32_t)conf.servoConf[5].rate * axisPID[ROLL] )/100L;   //  ROLLServo    D3  => Rudder servo
    for(i=4;i<6;i++) {
      servo[i] =  axisPID[5-i] * SERVODIR(i,1);    // mix and setup direction
      servo[i] += get_middle(i);
    }
    motor[0] = PIDMIX(0,0,-1);                                 //  Pin D9
    motor[1] = PIDMIX(0,0,+1);                                 //  Pin D10

  #elif defined( HELICOPTER )
    /*****************************               HELICOPTERS               **************************************/
   // Common controlls for Helicopters
    int16_t heliRoll,heliNick;
    int16_t collRange[3] = COLLECTIVE_RANGE;
    static int16_t   collective;
    #ifdef GOVERNOR_P
      static int16_t last_collective = 0, delta_collective = 0, governorThrottle = 0;
    #endif
    /***************************
     * servo settings Heli.
     ***************************/

    // Limit Collective range up/down
    int16_t collect = rcData[COLLECTIVE_PITCH] - (1500 + collRange[1]);
    if   (collect>0) {
      collective = collect * (collRange[2]*0.01);
    } else{
      collective = collect * (collRange[0]*0.01);
    }

    // maybe collective range can be replaced replaced by this ?
    //collective = rcData[COLLECTIVE_PITCH] - get_middle(7);               // zero pitch offset
    //collective = ((int32_t)conf.servoConf[7].rate * collective)/100L;    // collective range
    
    #ifdef GOVERNOR_P
      delta_collective = collective - last_collective;
      last_collective = collective;
      if (! f.ARMED || ! rcOptions[BOXGOV] || (rcCommand[THROTTLE] < conf.minthrottle) )
        governorThrottle = 0; // clear subito
      else if (delta_collective > 0) {
        governorThrottle += delta_collective * conf.governorP; // attack
        // avoid overshooting governor (would result in overly long decay phase)
        if (rcCommand[THROTTLE] + governorThrottle > MAXTHROTTLE) governorThrottle = MAXTHROTTLE - rcCommand[THROTTLE];
      } else {
        static uint8_t d = 0;
        if (! (++d % conf.governorD)) governorThrottle -= 10; // decay; signal stepsize 10 should  be smooth on most ESCs
      }
      if (governorThrottle < 0) governorThrottle = 0; // always beware of negative impact of governor on throttle
    #endif

    if(f.PASSTHRU_MODE){ // Use Rcdata Without sensors
      heliRoll = rcCommand[ROLL] ;
      heliNick = rcCommand[PITCH];
    } else{ // Assisted modes
      heliRoll = axisPID[ROLL];
      heliNick = axisPID[PITCH];
    }

    // Limit Maximum Rates for Heli
    int16_t cRange[2] = CONTROL_RANGE;
    heliRoll*= cRange[0]*0.01;
    heliNick*= cRange[1]*0.01;

    /* Throttle & YAW
    ******************** */
    // Yaw control is common for Heli 90 & 120
    servo[5] = (axisPID[YAW] * SERVODIR(5,1)) + conf.servoConf[5].middle;
    #if YAWMOTOR
      servo[5] = constrain(servo[5], conf.servoConf[5].min, conf.servoConf[5].max); // limit the values
      if (rcCommand[THROTTLE]<conf.minthrottle || !f.ARMED) {servo[5] = MINCOMMAND;} // Kill YawMotor
    #endif
    if (!f.ARMED){
      servo[7] = MINCOMMAND;          // Kill throttle when disarmed
    } else {
      servo[7] = rcCommand[THROTTLE]; //   50hz ESC or servo
      #ifdef GOVERNOR_P
        servo[7] += governorThrottle;
      #endif
      servo[7] = constrain(servo[7], conf.minthrottle, MAXTHROTTLE);   //  limit min & max    }
    }
    #ifndef HELI_USE_SERVO_FOR_THROTTLE
      motor[0] = servo[7];     // use real motor output - ESC capable
      #if YAWMOTOR
        motor[1] = servo[5];   // use motor2 output for YAWMOTOR
      #endif
    #endif

    //              ( Collective, Pitch/Nick, Roll ) Change sign to invert
    /************************************************************************************************************/
    #define HeliXPIDMIX(Z,Y,X) ( (collRange[1] + collective)*Z + heliNick*Y +  heliRoll*X)/10
    #ifdef HELI_120_CCPM
      static int8_t nickMix[3] = SERVO_NICK;
      static int8_t leftMix[3] = SERVO_LEFT;
      static int8_t rightMix[3]= SERVO_RIGHT;

      servo[3]  =  HeliXPIDMIX( ( SERVODIR(3,4) * nickMix[0]), SERVODIR(3,2) * nickMix[1], SERVODIR(3,1) * nickMix[2]);   //    NICK  servo
      servo[4]  =  HeliXPIDMIX( ( SERVODIR(4,4) * leftMix[0]), SERVODIR(4,2) * leftMix[1], SERVODIR(4,1) * leftMix[2]);   //    LEFT  servo
      servo[6]  =  HeliXPIDMIX( ( SERVODIR(6,4) * rightMix[0]),SERVODIR(6,2) * rightMix[1],SERVODIR(6,1) * rightMix[2]);  //    RIGHT servo
    #endif
    /************************************************************************************************************/
    #ifdef HELI_90_DEG
      servo[3]  = HeliXPIDMIX( +0, (conf.servoConf[3].rate/10), -0);      //     NICK  servo
      servo[4]  = HeliXPIDMIX( +0, +0, (conf.servoConf[4].rate/10));      //     ROLL servo
      servo[6]  = HeliXPIDMIX( (conf.servoConf[6].rate/10), +0, +0);      //     COLLECTIVE  servo
    #endif
    servo[3] += get_middle(3);
    servo[4] += get_middle(4);
    servo[6] += get_middle(6);
  #elif defined( GIMBAL )
    for(i=0;i<2;i++) {
      servo[i]  = ((int32_t)conf.servoConf[i].rate * att.angle[1-i]) /50L;
      servo[i] += get_middle(i);
    }
  #else
    #error "missing coptertype mixtable entry. Either you forgot to define a copter type or the mixing table is lacking neccessary code"
  #endif // MY_PRIVATE_MIXING

  /************************************************************************************************************/
  /****************************                Cam stabilize Servos             *******************************/

  #if defined(SERVO_TILT)
    servo[0] = get_middle(0);
    servo[1] = get_middle(1);
    if (rcOptions[BOXCAMSTAB]) {
      servo[0] += ((int32_t)conf.servoConf[0].rate * att.angle[PITCH]) /50L;
      servo[1] += ((int32_t)conf.servoConf[1].rate * att.angle[ROLL])  /50L;
    }
  #endif

  #ifdef SERVO_MIX_TILT
    int16_t angleP = get_middle(0) - MIDRC;
    int16_t angleR = get_middle(1) - MIDRC;
    if (rcOptions[BOXCAMSTAB]) {
      angleP += ((int32_t)conf.servoConf[0].rate * att.angle[PITCH]) /50L;
      angleR += ((int32_t)conf.servoConf[1].rate * att.angle[ROLL])  /50L;
    }
    servo[0] = MIDRC+angleP-angleR;
    servo[1] = MIDRC-angleP-angleR;
  #endif

/****************                    Cam trigger Servo                ******************/
  #if defined(CAMTRIG)
    // setup MIDDLE for using as camtrig interval (in msec) or RC channel pointer for interval control
    #define CAM_TIME_LOW  conf.servoConf[2].middle
    static uint8_t camCycle = 0;
    static uint8_t camState = 0;
    static uint32_t camTime = 0;
    static uint32_t ctLow;
    if (camCycle==1) {
      if (camState == 0) {
        camState = 1;
        camTime = millis();
      } else if (camState == 1) {
        if ( (millis() - camTime) > CAM_TIME_HIGH ) {
          camState = 2;
          camTime = millis();
          if(CAM_TIME_LOW < RC_CHANS) {
            ctLow = constrain((rcData[CAM_TIME_LOW]-1000)/4, 30, 250);
            ctLow *= ctLow;
          } else ctLow = CAM_TIME_LOW;
        }
      } else { //camState ==2
        if (((millis() - camTime) > ctLow) || !rcOptions[BOXCAMTRIG] ) {
          camState = 0;
          camCycle = 0;
        }
      }
    }
    if (rcOptions[BOXCAMTRIG]) camCycle=1;
    servo[2] =(camState==1) ? conf.servoConf[2].max : conf.servoConf[2].min;
    servo[2] = (servo[2]-1500)*SERVODIR(2,1)+1500;
  #endif

/************************************************************************************************************/
  // add midpoint offset, then scale and limit servo outputs - except SERVO8 used commonly as Moror output
  // don't add offset for camtrig servo (SERVO3)
  #if defined(SERVO)
    for(i=SERVO_START-1; i<SERVO_END; i++) {
      if(i < 2) {
        servo[i] = map(servo[i], 1020,2000, conf.servoConf[i].min, conf.servoConf[i].max);   // servo travel scaling, only for gimbal servos
      }
    #if defined(HELICOPTER) && (YAWMOTOR)
      if(i != 5) // not limit YawMotor
    #endif
        servo[i] = constrain(servo[i], conf.servoConf[i].min, conf.servoConf[i].max); // limit the values
    }
    #if defined(A0_A1_PIN_HEX) && (NUMBER_MOTOR == 6) && defined(PROMINI)
      servo[3] = servo[0];    // copy CamPitch value to propper output servo for A0_A1_PIN_HEX
      servo[4] = servo[1];    // copy CamRoll  value to propper output servo for A0_A1_PIN_HEX
    #endif
    #if defined(TRI) && defined(MEGA_HW_PWM_SERVOS) && defined(MEGA)
      servo[3] = servo[5];    // copy TRI serwo value to propper output servo for MEGA_HW_PWM_SERVOS
    #endif
  #endif

  /****************                compensate the Motors values                ******************/
  #ifdef VOLTAGEDROP_COMPENSATION
    {
      #if (VBATNOMINAL == 126)
        #define GOV_R_NUM 36
        static int8_t g[] = { 0,3,5,8,11,14,17,19,22,25,28,31,34,38,41,44,47,51,54,58,61,65,68,72,76,79,83,87,91,95,99,104,108,112,117,121,126 };
      #elif (VBATNOMINAL == 84)
        #define GOV_R_NUM 24
        static int8_t g[] = { 0,4,8,12,17,21,25,30,34,39,44,49,54,59,65,70,76,81,87,93,99,106,112,119,126 };
      #else
        #error "VOLTAGEDROP_COMPENSATION requires correction values which fit VBATNOMINAL; not yet defined for your value of VBATNOMINAL"
      #endif
      uint8_t v = constrain( VBATNOMINAL - constrain(analog.vbat, conf.vbatlevel_crit, VBATNOMINAL), 0, GOV_R_NUM);
      for (i = 0; i < NUMBER_MOTOR; i++) {
        motor[i] += ( ( (int32_t)(motor[i]-1000) * (int32_t)g[v] ) )/ 500;
      }
    }
  #endif
  /****************                normalize the Motors values                ******************/
    maxMotor=motor[0];
    for(i=1; i< NUMBER_MOTOR; i++)
      if (motor[i]>maxMotor) maxMotor=motor[i];
    for(i=0; i< NUMBER_MOTOR; i++) {
      if (maxMotor > MAXTHROTTLE) // this is a way to still have good gyro corrections if at least one motor reaches its max.
        motor[i] -= maxMotor - MAXTHROTTLE;
      motor[i] = constrain(motor[i], conf.minthrottle, MAXTHROTTLE);
      #if defined(ALTHOLD_FAST_THROTTLE_CHANGE)
        if (rcData[THROTTLE] < MINCHECK)
      #else
        if ((rcData[THROTTLE] < MINCHECK) && !f.BARO_MODE)
      #endif
        #ifndef MOTOR_STOP
          motor[i] = conf.minthrottle;
        #else
          motor[i] = MINCOMMAND;
        #endif
      if (!f.ARMED)
        motor[i] = MINCOMMAND;
    }

  /****************                      Powermeter Log                    ******************/
  #if (LOG_VALUES >= 3) || defined(POWERMETER_SOFT)
  {
    static uint32_t lastRead = currentTime;
    uint16_t amp;
    uint32_t ampsum, ampus; // pseudo ampere * microseconds
    /* true cubic function;
     * when divided by vbat_max=126 (12.6V) for 3 cell battery this gives maximum value of ~ 500
     * when divided by no_vbat=60 (6V) for 3 cell battery this gives maximum value of ~ 1000
     * */

    static uint16_t amperes[64] =   {   0,  2,  6, 15, 30, 52, 82,123,
                                     175,240,320,415,528,659,811,984,
                                     1181,1402,1648,1923,2226,2559,2924,3322,
                                     3755,4224,4730,5276,5861,6489,7160,7875,
                                     8637 ,9446 ,10304,11213,12173,13187,14256,15381,
                                     16564,17805,19108,20472,21900,23392,24951,26578,
                                     28274,30041,31879,33792,35779,37843,39984,42205,
                                     44507,46890,49358,51910,54549,57276,60093,63000};
  
    if (analog.vbat > NO_VBAT) { // by all means - must avoid division by zero
      ampsum = 0;
      for (i =0;i<NUMBER_MOTOR;i++) {
        amp = amperes[ ((motor[i] - 1000)>>4) ] / analog.vbat; // range mapped from [1000:2000] => [0:1000]; then break that up into 64 ranges; lookup amp
        ampus = ( (currentTime-lastRead) * (uint32_t)amp * (uint32_t)conf.pint2ma ) / PLEVELDIVSOFT;
        #if (LOG_VALUES >= 3)
          pMeter[i]+= ampus; // sum up over time the mapped ESC input
        #endif
        #if defined(POWERMETER_SOFT)
          ampsum += ampus; // total sum over all motors
        #endif
      }
      #if defined(POWERMETER_SOFT)
        pMeter[PMOTOR_SUM]+= ampsum / NUMBER_MOTOR; // total sum over all motors
      #endif
    }
    lastRead = currentTime;
  }
  #endif
}
