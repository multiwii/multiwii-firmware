/**************************************************************************************/
/***************             Proc specific definitions             ********************/
/**************************************************************************************/
// Proc auto detection
#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
  #define PROMINI
#endif
#if defined(__AVR_ATmega32U4__) || defined(TEENSY20)
  #define PROMICRO
#endif
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
  #define MEGA
#endif

/**************************   atmega328P (Promini)  ************************************/
#if defined(PROMINI)
  #if !defined(MONGOOSE1_0)
    #define LEDPIN_PINMODE             pinMode (13, OUTPUT);
    #define LEDPIN_TOGGLE              PINB |= 1<<5;     //switch LEDPIN state (digital PIN 13)
    #define LEDPIN_OFF                 PORTB &= ~(1<<5);
    #define LEDPIN_ON                  PORTB |= (1<<5);
  #endif
  #if !defined(RCAUXPIN8) 
    #if !defined(MONGOOSE1_0)
      #define BUZZERPIN_PINMODE          pinMode (8, OUTPUT);
      #define BUZZERPIN_ON               PORTB |= 1;
      #define BUZZERPIN_OFF              PORTB &= ~1;
    #endif
  #else
    #define BUZZERPIN_PINMODE          ;
    #define BUZZERPIN_ON               ;
    #define BUZZERPIN_OFF              ;
    #define RCAUXPIN
  #endif
  #if !defined(RCAUXPIN12) && !defined(DISABLE_POWER_PIN)
    #define POWERPIN_PINMODE           pinMode (12, OUTPUT);
    #define POWERPIN_ON                PORTB |= 1<<4;
    #define POWERPIN_OFF               PORTB &= ~(1<<4); //switch OFF WMP, digital PIN 12
  #else
    #define POWERPIN_PINMODE           ;
    #define POWERPIN_ON                ;
    #define POWERPIN_OFF               ;
  #endif
  #if defined(RCAUXPIN12)
    #define RCAUXPIN
  #endif
  #define I2C_PULLUPS_ENABLE         PORTC |= 1<<4; PORTC |= 1<<5;   // PIN A4&A5 (SDA&SCL)
  #define I2C_PULLUPS_DISABLE        PORTC &= ~(1<<4); PORTC &= ~(1<<5);
  #if !defined(MONGOOSE1_0)
    #define PINMODE_LCD                pinMode(0, OUTPUT);
    #define LCDPIN_OFF                 PORTD &= ~1; //switch OFF digital PIN 0
    #define LCDPIN_ON                  PORTD |= 1;
    #define STABLEPIN_PINMODE          ;
    #define STABLEPIN_ON               ;
    #define STABLEPIN_OFF              ;
  #endif 
  #define PPM_PIN_INTERRUPT          attachInterrupt(0, rxInt, RISING); //PIN 0
  #define SPEK_SERIAL_VECT           USART_RX_vect
  #define SPEK_DATA_REG              UDR0
  //RX PIN assignment inside the port //for PORTD
  #define THROTTLEPIN                2
  #define ROLLPIN                    4
  #define PITCHPIN                   5
  #define YAWPIN                     6
  #define AUX1PIN                    7
  #define AUX2PIN                    0 // optional PIN 8 or PIN 12
  #define AUX3PIN                    1 // unused 
  #define AUX4PIN                    3 // unused 
    
  #define PCINT_PIN_COUNT            5
  #define PCINT_RX_BITS              (1<<2),(1<<4),(1<<5),(1<<6),(1<<7)
  #define PCINT_RX_PORT              PORTD
  #define PCINT_RX_MASK              PCMSK2
  #define PCIR_PORT_BIT              (1<<2)
  #define RX_PC_INTERRUPT            PCINT2_vect
  #define RX_PCINT_PIN_PORT          PIND
  #define ISR_UART                   ISR(USART_UDRE_vect)
  #define V_BATPIN                   A3    // Analog PIN 3
  #define PSENSORPIN                 A2    // Analog PIN 2
  
  #if defined(A0_A1_PIN_HEX) || (NUMBER_MOTOR > 6)
    #define SOFT_PWM_1_PIN_HIGH        PORTC |= 1<<0;
    #define SOFT_PWM_1_PIN_LOW         PORTC &= ~(1<<0);
    #define SOFT_PWM_2_PIN_HIGH        PORTC |= 1<<1;
    #define SOFT_PWM_2_PIN_LOW         PORTC &= ~(1<<1);  
  #else
    #define SOFT_PWM_1_PIN_HIGH        PORTD |= 1<<5;
    #define SOFT_PWM_1_PIN_LOW         PORTD &= ~(1<<5);
    #define SOFT_PWM_2_PIN_HIGH        PORTD |= 1<<6;
    #define SOFT_PWM_2_PIN_LOW         PORTD &= ~(1<<6);
  #endif
  #define SOFT_PWM_3_PIN_HIGH        PORTC |= 1<<2;
  #define SOFT_PWM_3_PIN_LOW         PORTC &= ~(1<<2);
  #define SOFT_PWM_4_PIN_HIGH        PORTB |= 1<<4;
  #define SOFT_PWM_4_PIN_LOW         PORTB &= ~(1<<4);
  
  #define SERVO_1_PINMODE            pinMode(A0,OUTPUT); // TILT_PITCH - WING left
  #define SERVO_1_PIN_HIGH           PORTC |= 1<<0;
  #define SERVO_1_PIN_LOW            PORTC &= ~(1<<0);
  #define SERVO_2_PINMODE            pinMode(A1,OUTPUT); // TILT_ROLL  - WING right
  #define SERVO_2_PIN_HIGH           PORTC |= 1<<1;
  #define SERVO_2_PIN_LOW            PORTC &= ~(1<<1);
  #define SERVO_3_PINMODE            pinMode(A2,OUTPUT); // CAM TRIG  - alt TILT_PITCH
  #define SERVO_3_PIN_HIGH           PORTC |= 1<<2;
  #define SERVO_3_PIN_LOW            PORTC &= ~(1<<2);
  #if !defined(MONGOOSE1_0)
    #define SERVO_4_PINMODE            pinMode(12,OUTPUT); // new       - alt TILT_ROLL
    #define SERVO_4_PIN_HIGH           PORTB |= 1<<4;
    #define SERVO_4_PIN_LOW            PORTB &= ~(1<<4);
  #endif
  #define SERVO_5_PINMODE            pinMode(11,OUTPUT); // BI LEFT
  #define SERVO_5_PIN_HIGH           PORTB |= 1<<3;
  #define SERVO_5_PIN_LOW            PORTB &= ~(1<<3);
  #define SERVO_6_PINMODE            pinMode(3,OUTPUT);  // TRI REAR - BI RIGHT
  #define SERVO_6_PIN_HIGH           PORTD|= 1<<3;
  #define SERVO_6_PIN_LOW            PORTD &= ~(1<<3);
  #define SERVO_7_PINMODE            pinMode(10,OUTPUT); // new
  #define SERVO_7_PIN_HIGH           PORTB |= 1<<2;
  #define SERVO_7_PIN_LOW            PORTB &= ~(1<<2);
  #define SERVO_8_PINMODE            pinMode(9,OUTPUT); // new
  #define SERVO_8_PIN_HIGH           PORTB |= 1<<1;
  #define SERVO_8_PIN_LOW            PORTB &= ~(1<<1);
#endif

/**************************  atmega32u4 (Promicro)  ***********************************/
#if defined(PROMICRO)
  #if !defined(TEENSY20)
    #define LEDPIN_PINMODE             //
    #define LEDPIN_TOGGLE              PIND |= 1<<5;     //switch LEDPIN state (Port D5)
    #if !defined(PROMICRO10)
      #define LEDPIN_OFF                 PORTD |= (1<<5);
      #define LEDPIN_ON                  PORTD &= ~(1<<5);  
    #else
      #define LEDPIN_OFF                PORTD &= ~(1<<5);
      #define LEDPIN_ON                 PORTD |= (1<<5);
    #endif
  #else
    #define LEDPIN_PINMODE           DDRD |= (1<<6);
    #define LEDPIN_OFF               PORTD &= ~(1<<6);
    #define LEDPIN_ON                PORTD |= (1<<6);   
    #define LEDPIN_TOGGLE            PIND |= 1<<6;     //switch LEDPIN state (Port D6)  
  #endif
  #if defined(D8BUZZER)
    #define BUZZERPIN_PINMODE          DDRB |= (1<<4);
    #define BUZZERPIN_ON               PORTB |= 1<<4;
    #define BUZZERPIN_OFF              PORTB &= ~(1<<4); 
  #elif defined(A32U4ALLPINS)
    #define BUZZERPIN_PINMODE          DDRD |= (1<<4);
    #define BUZZERPIN_ON               PORTD |= 1<<4;
    #define BUZZERPIN_OFF              PORTD &= ~(1<<4);    
  #else
    #define BUZZERPIN_PINMODE          DDRD |= (1<<3);
    #define BUZZERPIN_ON               PORTD |= 1<<3;
    #define BUZZERPIN_OFF              PORTD &= ~(1<<3); 
  #endif
  #define POWERPIN_PINMODE           //
  #define POWERPIN_ON                //
  #define POWERPIN_OFF               //
  #define I2C_PULLUPS_ENABLE         PORTD |= 1<<0; PORTD |= 1<<1;   // PIN 2&3 (SDA&SCL)
  #define I2C_PULLUPS_DISABLE        PORTD &= ~(1<<0); PORTD &= ~(1<<1);
  #define PINMODE_LCD                DDRD |= (1<<2);
  #define LCDPIN_OFF                 PORTD &= ~1;
  #define LCDPIN_ON                  PORTD |= 1;
  #define STABLEPIN_PINMODE          ;
  #define STABLEPIN_ON               ;
  #define STABLEPIN_OFF              ;
  #define PPM_PIN_INTERRUPT          DDRE &= ~(1 << 6);PORTE |= (1 << 6);EIMSK |= (1 << INT6);EICRB |= (1 << ISC61)|(1 << ISC60);
  #define SPEK_SERIAL_VECT           USART1_RX_vect
  #define SPEK_DATA_REG              UDR1
  #define USB_CDC_TX                 3
  #define USB_CDC_RX                 2
  
  //soft PWM Pins  
  #define SOFT_PWM_1_PIN_HIGH        PORTD |= 1<<4;
  #define SOFT_PWM_1_PIN_LOW         PORTD &= ~(1<<4);
  #define SOFT_PWM_2_PIN_HIGH        PORTF |= 1<<5;
  #define SOFT_PWM_2_PIN_LOW         PORTF &= ~(1<<5);
  #if !defined(A32U4ALLPINS)
    #define SOFT_PWM_3_PIN_HIGH        PORTF |= 1<<7;
    #define SOFT_PWM_3_PIN_LOW         PORTF &= ~(1<<7);
    #define SOFT_PWM_4_PIN_HIGH        PORTF |= 1<<6;
    #define SOFT_PWM_4_PIN_LOW         PORTF &= ~(1<<6);
    #define SW_PWM_P3                  A1        
    #define SW_PWM_P4                  A0
  #else
    #define SOFT_PWM_3_PIN_HIGH        PORTF |= 1<<4;
    #define SOFT_PWM_3_PIN_LOW         PORTF &= ~(1<<4);
    #define SOFT_PWM_4_PIN_HIGH        PORTF |= 1<<5;
    #define SOFT_PWM_4_PIN_LOW         PORTF &= ~(1<<5); 
    #define SW_PWM_P3                  A2        
    #define SW_PWM_P4                  A3 
  #endif
  
  // Servos
  #define SERVO_1_PINMODE   DDRF |= (1<<7); // A0
  #define SERVO_1_PIN_HIGH  PORTF|= 1<<7;
  #define SERVO_1_PIN_LOW   PORTF &= ~(1<<7);
  #define SERVO_2_PINMODE   DDRF |= (1<<6); // A1
  #define SERVO_2_PIN_HIGH  PORTF |= 1<<6;
  #define SERVO_2_PIN_LOW   PORTF &= ~(1<<6);
  #define SERVO_3_PINMODE   DDRF |= (1<<5); // A2
  #define SERVO_3_PIN_HIGH  PORTF |= 1<<5;
  #define SERVO_3_PIN_LOW   PORTF &= ~(1<<5);
  #if !defined(A32U4ALLPINS)
    #define SERVO_4_PINMODE   DDRD |= (1<<4); // 4
    #define SERVO_4_PIN_HIGH  PORTD |= 1<<4;
    #define SERVO_4_PIN_LOW   PORTD &= ~(1<<4);
  #else
    #define SERVO_4_PINMODE   DDRF |= (1<<4); // A3
    #define SERVO_4_PIN_HIGH  PORTF |= 1<<4;
    #define SERVO_4_PIN_LOW   PORTF &= ~(1<<4);  
  #endif
  #define SERVO_5_PINMODE   DDRD |= (1<<7); // 6
  #define SERVO_5_PIN_HIGH  PORTD |= 1<<7;
  #define SERVO_5_PIN_LOW   PORTD &= ~(1<<7);
  #define SERVO_6_PINMODE   DDRC |= (1<<6); // 5
  #define SERVO_6_PIN_HIGH  PORTC|= 1<<6;
  #define SERVO_6_PIN_LOW   PORTC &= ~(1<<6);
  #define SERVO_7_PINMODE   DDRB |= (1<<6); // 10
  #define SERVO_7_PIN_HIGH  PORTB |= 1<<6;
  #define SERVO_7_PIN_LOW   PORTB &= ~(1<<6);
  #define SERVO_8_PINMODE   DDRB |= (1<<5); // 9
  #define SERVO_8_PIN_HIGH  PORTB |= 1<<5;
  #define SERVO_8_PIN_LOW   PORTB &= ~(1<<5);
  
  //Standart RX
  #define THROTTLEPIN                  3
  #if defined(A32U4ALLPINS)
    #define ROLLPIN                    6
    #define PITCHPIN                   2
    #define YAWPIN                     4
    #define AUX1PIN                    5
  #else
    #define ROLLPIN                    4
    #define PITCHPIN                   5
    #define YAWPIN                     2
    #define AUX1PIN                    6
  #endif
  #define AUX2PIN                      7 
  #define AUX3PIN                      1 // unused 
  #define AUX4PIN                      0 // unused 
  #if !defined(RCAUX2PIND17)
    #define PCINT_PIN_COUNT          4
    #define PCINT_RX_BITS            (1<<1),(1<<2),(1<<3),(1<<4)
  #else
    #define PCINT_PIN_COUNT          5 // one more bit (PB0) is added in RX code
    #define PCINT_RX_BITS            (1<<1),(1<<2),(1<<3),(1<<4),(1<<0)
  #endif
  #define PCINT_RX_PORT                PORTB
  #define PCINT_RX_MASK                PCMSK0
  #define PCIR_PORT_BIT                (1<<0)
  #define RX_PC_INTERRUPT              PCINT0_vect
  #define RX_PCINT_PIN_PORT            PINB
  
  #define ISR_UART                    ISR(USART_UDRE_vect)
  #if !defined(A32U4ALLPINS) && !defined(TEENSY20)
    #define V_BATPIN                  A3    // Analog PIN 3
  #elif defined(A32U4ALLPINS)
    #define V_BATPIN                  A4    // Analog PIN 4
  #else
    #define V_BATPIN                  A2    // Analog PIN 3
  #endif
  #if !defined(TEENSY20)
    #define PSENSORPIN                A2    // Analog PIN 2 
  #else
    #define PSENSORPIN                A2    // Analog PIN 2 
  #endif
#endif

/**************************  all the Mega types  ***********************************/
#if defined(MEGA)
  #define LEDPIN_PINMODE             pinMode (13, OUTPUT);pinMode (30, OUTPUT);
  #define LEDPIN_TOGGLE              PINB  |= (1<<7); PINC  |= (1<<7);
  #define LEDPIN_ON                  PORTB |= (1<<7); PORTC |= (1<<7);
  #define LEDPIN_OFF                 PORTB &= ~(1<<7);PORTC &= ~(1<<7);
  #define BUZZERPIN_PINMODE          pinMode (32, OUTPUT);
  #define BUZZERPIN_ON               PORTC |= 1<<5;
  #define BUZZERPIN_OFF              PORTC &= ~(1<<5);
  #if !defined(DISABLE_POWER_PIN)
    #define POWERPIN_PINMODE           pinMode (37, OUTPUT);
    #define POWERPIN_ON                PORTC |= 1<<0;
    #define POWERPIN_OFF               PORTC &= ~(1<<0);
  #else
    #define POWERPIN_PINMODE           ;
    #define POWERPIN_ON                ;
    #define POWERPIN_OFF               ;
  #endif
  #define I2C_PULLUPS_ENABLE         PORTD |= 1<<0; PORTD |= 1<<1;       // PIN 20&21 (SDA&SCL)
  #define I2C_PULLUPS_DISABLE        PORTD &= ~(1<<0); PORTD &= ~(1<<1);
  #define PINMODE_LCD                pinMode(0, OUTPUT);
  #define LCDPIN_OFF                 PORTE &= ~1; //switch OFF digital PIN 0
  #define LCDPIN_ON                  PORTE |= 1;
  #define STABLEPIN_PINMODE          pinMode (31, OUTPUT);
  #define STABLEPIN_ON               PORTC |= 1<<6;
  #define STABLEPIN_OFF              PORTC &= ~(1<<6);

  #define PPM_PIN_INTERRUPT          attachInterrupt(4, rxInt, RISING);  //PIN 19, also used for Spektrum satellite option
  #define SPEK_SERIAL_VECT           USART1_RX_vect
  #define SPEK_DATA_REG              UDR1
  //RX PIN assignment inside the port //for PORTK
  #define THROTTLEPIN                0  //PIN 62 =  PIN A8
  #define ROLLPIN                    1  //PIN 63 =  PIN A9
  #define PITCHPIN                   2  //PIN 64 =  PIN A10
  #define YAWPIN                     3  //PIN 65 =  PIN A11
  #define AUX1PIN                    4  //PIN 66 =  PIN A12
  #define AUX2PIN                    5  //PIN 67 =  PIN A13
  #define AUX3PIN                    6  //PIN 68 =  PIN A14
  #define AUX4PIN                    7  //PIN 69 =  PIN A15
  #define V_BATPIN                   A0    // Analog PIN 0
  #define PSENSORPIN                 A2    // Analog PIN 2
  #define PCINT_PIN_COUNT            8
  #define PCINT_RX_BITS              (1<<2),(1<<4),(1<<5),(1<<6),(1<<7),(1<<0),(1<<1),(1<<3)
  #define PCINT_RX_PORT              PORTK
  #define PCINT_RX_MASK              PCMSK2
  #define PCIR_PORT_BIT              (1<<2)
  #define RX_PC_INTERRUPT            PCINT2_vect
  #define RX_PCINT_PIN_PORT          PINK
   
  #define ISR_UART                   ISR(USART0_UDRE_vect)
  
  #define SERVO_1_PINMODE            pinMode(34,OUTPUT);pinMode(44,OUTPUT); // TILT_PITCH - WING left
  #define SERVO_1_PIN_HIGH           PORTC |= 1<<3;PORTL |= 1<<5;
  #define SERVO_1_PIN_LOW            PORTC &= ~(1<<3);PORTL &= ~(1<<5);
  #define SERVO_2_PINMODE            pinMode(35,OUTPUT);pinMode(45,OUTPUT); // TILT_ROLL  - WING right
  #define SERVO_2_PIN_HIGH           PORTC |= 1<<2;PORTL |= 1<<4;
  #define SERVO_2_PIN_LOW            PORTC &= ~(1<<2);PORTL &= ~(1<<4);
  #define SERVO_3_PINMODE            pinMode(33,OUTPUT); pinMode(46,OUTPUT); // CAM TRIG  - alt TILT_PITCH
  #define SERVO_3_PIN_HIGH           PORTC |= 1<<4;PORTL |= 1<<3;
  #define SERVO_3_PIN_LOW            PORTC &= ~(1<<4);PORTL &= ~(1<<3);
  #define SERVO_4_PINMODE            pinMode (37, OUTPUT);                   // new       - alt TILT_ROLL
  #define SERVO_4_PIN_HIGH           PORTC |= 1<<0;
  #define SERVO_4_PIN_LOW            PORTC &= ~(1<<0);
  #define SERVO_5_PINMODE            pinMode(6,OUTPUT);                      // BI LEFT
  #define SERVO_5_PIN_HIGH           PORTH |= 1<<3;
  #define SERVO_5_PIN_LOW            PORTH &= ~(1<<3);
  #define SERVO_6_PINMODE            pinMode(2,OUTPUT);                      // TRI REAR - BI RIGHT
  #define SERVO_6_PIN_HIGH           PORTE |= 1<<4;
  #define SERVO_6_PIN_LOW            PORTE &= ~(1<<4);
  #define SERVO_7_PINMODE            pinMode(5,OUTPUT);                      // new
  #define SERVO_7_PIN_HIGH           PORTE |= 1<<3;
  #define SERVO_7_PIN_LOW            PORTE &= ~(1<<3);
  #define SERVO_8_PINMODE            pinMode(3,OUTPUT);                      // new
  #define SERVO_8_PIN_HIGH           PORTE |= 1<<5;
  #define SERVO_8_PIN_LOW            PORTE &= ~(1<<5);
#endif


// special defines for the Mongose IMU board 
// note: that may be moved to the IMU Orientations because this are board defines .. not Proc

#if defined(MONGOOSE1_0)  // basically it's a PROMINI without some PINS => same code as a PROMINI board except PIN definition
                          // note: to avoid too much dubble code there are now just the differencies
  // http://www.multiwii.com/forum/viewtopic.php?f=6&t=627
  #define LEDPIN_PINMODE             pinMode (4, OUTPUT);
  #define LEDPIN_TOGGLE              PIND |= 1<<4;     //switch LEDPIN state (digital PIN 13)
  #define LEDPIN_OFF                 PORTD &= ~(1<<4);  
  #define LEDPIN_ON                  PORTD |= (1<<4);     
  #define SPEK_BAUD_SET              UCSR0A  = (1<<U2X0); UBRR0H = ((F_CPU  / 4 / 115200 -1) / 2) >> 8; UBRR0L = ((F_CPU  / 4 / 115200 -1) / 2);
  #define SPEK_SERIAL_INTERRUPT      UCSR0B |= (1<<RXEN0)|(1<<RXCIE0);

  /* Unavailable pins on MONGOOSE1_0 */
  #define BUZZERPIN_PINMODE          ; // D8
  #define BUZZERPIN_ON               ;
  #define BUZZERPIN_OFF              ;
  #define POWERPIN_PINMODE           ; // D12
  #define POWERPIN_ON                ;
  #define POWERPIN_OFF               ;
  #define STABLEPIN_PINMODE          ; //
  #define STABLEPIN_ON               ;
  #define STABLEPIN_OFF              ; 
  #define PINMODE_LCD                ; //
  #define LCDPIN_OFF                 ;
  #define LCDPIN_ON                  ; 
  
  
  #define SERVO_4_PINMODE            ;                   // Not available
  #define SERVO_4_PIN_HIGH           ;
  #define SERVO_4_PIN_LOW            ;
#endif




/**************************************************************************************/
/***************      IMU Orientations and Sensor definitions      ********************/
/**************************************************************************************/


//please submit any correction to this list.
#if defined(FFIMUv1)
  #define ITG3200
  #define BMA180
  #define BMP085
  #define HMC5843
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -X; accADC[PITCH]  = -Y; accADC[YAW]  =  Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  Y; gyroADC[PITCH] = -X; gyroADC[YAW] = -Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  X; magADC[PITCH]  =  Y; magADC[YAW]  = -Z;}
#endif

#if defined(FFIMUv2)
  #define ITG3200
  #define BMA180
  #define BMP085
  #define HMC5883
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -X; accADC[PITCH]  = -Y; accADC[YAW]  =  Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  Y; gyroADC[PITCH] = -X; gyroADC[YAW] = -Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  Y; magADC[PITCH]  = -X; magADC[YAW]  = -Z;}
#endif

#if defined(FREEIMUv1)
  #define ITG3200
  #define ADXL345
  #define HMC5843
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -X; accADC[PITCH]  = -Y; accADC[YAW]  =  Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  Y; gyroADC[PITCH] = -X; gyroADC[YAW] = -Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  X;  magADC[PITCH] =  Y; magADC[YAW]  = -Z;}
  #define ADXL345_ADDRESS 0x53
  #undef INTERNAL_I2C_PULLUPS
#endif

#if defined(FREEIMUv03)
  #define ITG3200
  #define ADXL345 // this is actually an ADXL346 but that's just the same as ADXL345
  #define HMC5883
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -X; accADC[PITCH]  = -Y; accADC[YAW]  =  Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  Y; gyroADC[PITCH] = -X; gyroADC[YAW] = -Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  X; magADC[PITCH]  =  Y; magADC[YAW]  = -Z;}
  #define ADXL345_ADDRESS 0x53
  #undef INTERNAL_I2C_PULLUPS
#endif

#if defined(FREEIMUv035) || defined(FREEIMUv035_MS) || defined(FREEIMUv035_BMP)
  #define ITG3200
  #define BMA180
  #define HMC5883
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -X; accADC[PITCH]  = -Y; accADC[YAW]  =  Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  Y; gyroADC[PITCH] = -X; gyroADC[YAW] = -Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  X; magADC[PITCH]  =  Y; magADC[YAW]  = -Z;}
  #undef INTERNAL_I2C_PULLUPS
  #if defined(FREEIMUv035_MS)
    #define MS561101BA
  #elif defined(FREEIMUv035_BMP)
    #define BMP085
  #endif
#endif

#if defined(FREEIMUv04)
 #define FREEIMUv043
#endif

#if defined(FREEIMUv043)
  #define MPU6050
  #define HMC5883
  #define MS561101BA
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -X; accADC[PITCH]  = -Y; accADC[YAW]  =  Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  Y; gyroADC[PITCH] = -X; gyroADC[YAW] = -Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  X; magADC[PITCH]  =  Y; magADC[YAW]  = -Z;}
  #define MPU6050_I2C_AUX_MASTER // MAG connected to the AUX I2C bus of MPU6050
  #undef INTERNAL_I2C_PULLUPS
#endif

#if defined(NANOWII)
  #define MPU6050
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -Y; accADC[PITCH]  =  X; accADC[YAW]  =  Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] = -X; gyroADC[PITCH] = -Y; gyroADC[YAW] = -Z;}
  #undef INTERNAL_I2C_PULLUPS
  #define SOFT_PWM_3_PIN_HIGH        PORTD |= 1<<4;
  #define SOFT_PWM_3_PIN_LOW         PORTD &= ~(1<<4);
  #define SOFT_PWM_4_PIN_HIGH        PORTF |= 1<<5;
  #define SOFT_PWM_4_PIN_LOW         PORTF &= ~(1<<5);
  #define SW_PWM_P3                  4        
  #define SW_PWM_P4                  A2
  #define HWPWM6
#endif

#if defined(PIPO)
  #define L3G4200D
  #define ADXL345
  #define HMC5883
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -X; accADC[PITCH]  = -Y; accADC[YAW]  =  Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  X; gyroADC[PITCH] =  Y; gyroADC[YAW] = -Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  Y; magADC[PITCH]  = -X; magADC[YAW]  =  Z;}
  #define ADXL345_ADDRESS 0x53
#endif

#if defined(QUADRINO)
  #define ITG3200
  #define BMA180
  #define BMP085
  #define HMC5883
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -X; accADC[PITCH]  = -Y; accADC[YAW]  =  Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  Y; gyroADC[PITCH] = -X; gyroADC[YAW] = -Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  X; magADC[PITCH]  =  Y; magADC[YAW]  = -Z;}
#endif

#if defined(QUADRINO_ZOOM)
  #define ITG3200
  #define BMA180
  #define BMP085
  #define HMC5883
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -X; accADC[PITCH]  = -Y; accADC[YAW]  =  Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  Y; gyroADC[PITCH] = -X; gyroADC[YAW] = -Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  X; magADC[PITCH]  =  Y; magADC[YAW]  = -Z;}
  #define STABLEPIN_PINMODE pinMode (A2, OUTPUT);
  #define STABLEPIN_ON PORTC |= (1<<2);
  #define STABLEPIN_OFF PORTC &= ~(1<<2);
#endif

#if defined(QUADRINO_ZOOM_MS)
  #define ITG3200
  #define BMA180
  #define MS561101BA
  #define HMC5883
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -X; accADC[PITCH]  = -Y; accADC[YAW]  =  Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  Y; gyroADC[PITCH] = -X; gyroADC[YAW] = -Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  X; magADC[PITCH]  =  Y; magADC[YAW]  = -Z;}
  #define STABLEPIN_PINMODE pinMode (A2, OUTPUT);
  #define STABLEPIN_ON PORTC |= (1<<2);
  #define STABLEPIN_OFF PORTC &= ~(1<<2);
#endif

#if defined(ALLINONE)
  #define ITG3200
  #define BMA180
  #define BMP085
  #define HMC5883
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -X; accADC[PITCH]  = -Y; accADC[YAW]  =  Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  Y; gyroADC[PITCH] = -X; gyroADC[YAW] = -Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  X; magADC[PITCH]  =  Y; magADC[YAW]  = -Z;}
  #define BMA180_ADDRESS 0x41
#endif

#if defined(AEROQUADSHIELDv2) // to confirm
  #define ITG3200
  #define BMA180
  #define BMP085
  #define HMC5843
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  =  X; accADC[PITCH]  = -Y; accADC[YAW]  =  Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  X; gyroADC[PITCH] =  Y; gyroADC[YAW] = -Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  X; magADC[PITCH]  =  Y; magADC[YAW]  = -Z;}
  #define ITG3200_ADDRESS 0X69
#endif

#if defined(ATAVRSBIN1)
  #define ITG3200
  #define BMA020        //Actually it's a BMA150, but this is a drop in replacement for the discountinued BMA020
  #define AK8975
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  =  Y; accADC[PITCH]  = -X; accADC[YAW]  =  Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  X; gyroADC[PITCH] =  Y; gyroADC[YAW] = -Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  = -Y; magADC[PITCH]  = -X; magADC[YAW]  =  Z;}
#endif

#if defined(SIRIUS)
  #define ITG3200
  #define BMA180
  #define BMP085
  #define HMC5883
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -X; accADC[PITCH]  = -Y; accADC[YAW]  =  Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  Y; gyroADC[PITCH] = -X; gyroADC[YAW] = -Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  X; magADC[PITCH]  =  Y; magADC[YAW]  = -Z;}
#endif

#if defined(SIRIUS600)
  #define WMP
  #define BMA180
  #define BMP085
  #define HMC5883
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -X; accADC[PITCH]  = -Y; accADC[YAW]  =  Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  X; magADC[PITCH]  =  Y; magADC[YAW]  = -Z;}
#endif

#if defined(MINIWII)
  #define ITG3200
  #define BMA180
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -X; accADC[PITCH]  = -Y; accADC[YAW]  =  Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  Y; gyroADC[PITCH] = -X; gyroADC[YAW] = -Z;}
#endif

#if defined(CITRUSv2_1)
  #define ITG3200
  #define ADXL345
  #define BMP085
  #define HMC5883
  #define ACC_ORIENTATION(X, Y, Z) {accADC[ROLL] = -X; accADC[PITCH] = -Y; accADC[YAW] = Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] = Y; gyroADC[PITCH] = -X; gyroADC[YAW] = -Z;}
  #define MAG_ORIENTATION(X, Y, Z) {magADC[ROLL] = X; magADC[PITCH] = Y; magADC[YAW] = -Z;}
  #undef INTERNAL_I2C_PULLUPS
#endif

#if defined(CHERRY6DOFv1_0)
  #define MPU6050
  #define ACC_ORIENTATION(Y, X, Z)  {accADC[ROLL]  = -Y; accADC[PITCH]  = -X; accADC[YAW]  =  Z;}
  #define GYRO_ORIENTATION(Y, X, Z) {gyroADC[ROLL] =  X; gyroADC[PITCH] = -Y; gyroADC[YAW] = -Z;}
  #undef INTERNAL_I2C_PULLUPS
#endif

#if defined(DROTEK_10DOF) || defined(DROTEK_10DOF_MS)
  #define ITG3200
  #define BMA180
  #define HMC5883
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -X; accADC[PITCH]  = -Y; accADC[YAW]  =  Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  Y; gyroADC[PITCH] = -X; gyroADC[YAW] = -Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  X; magADC[PITCH]  =  Y; magADC[YAW]  = -Z;}
  #define ITG3200_ADDRESS 0X69
  #if defined(DROTEK_10DOF_MS)
    #define MS561101BA
  #elif defined(DROTEK_10DOF)
    #define BMP085
  #endif
#endif

#if defined(DROTEK_6DOFv2)
  #define ITG3200
  #define BMA180
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -Y; accADC[PITCH]  =  X; accADC[YAW]  =  Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] = -X; gyroADC[PITCH] = -Y; gyroADC[YAW] = -Z;}
  #define ITG3200_ADDRESS 0X69
#endif

#if defined(DROTEK_6DOF_MPU)
  #define MPU6050
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -Y; accADC[PITCH]  =  X; accADC[YAW]  =  Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] = -X; gyroADC[PITCH] = -Y; gyroADC[YAW] = -Z;}
  #define MPU6050_ADDRESS 0x69
  #undef INTERNAL_I2C_PULLUPS
#endif

#if defined(DROTEK_10DOF_MPU)
  #define MPU6050
  #define HMC5883
  #define MS561101BA
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  =  Y; accADC[PITCH]  = -X; accADC[YAW]  = Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  X; gyroADC[PITCH] = Y; gyroADC[YAW] = -Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  -Y; magADC[PITCH]  = X; magADC[YAW]  = -Z;}
  #define MPU6050_ADDRESS 0X69
  #define MPU6050_I2C_AUX_MASTER // MAG connected to the AUX I2C bus of MPU6050
  #undef INTERNAL_I2C_PULLUPS
#endif

#if defined(FLYDUINO_MPU)
  #define MPU6050
  #define ACC_ORIENTATION(X, Y, Z) {accADC[ROLL] = X; accADC[PITCH] = Y; accADC[YAW] = Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] = -Y; gyroADC[PITCH] = X; gyroADC[YAW] = -Z;}
#endif

#if defined(MONGOOSE1_0)
  #define ITG3200
  #define ADXL345
  #define BMP085
  #define HMC5883
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] = -Y; gyroADC[PITCH] =  X; gyroADC[YAW] = -Z;}
  #define ACC_ORIENTATION(Y, X, Z)  {accADC[ROLL]  =  Y; accADC[PITCH]  =  X; accADC[YAW]  =  Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  = -X; magADC[PITCH]  = -Y; magADC[YAW]  = -Z;}
  #define ADXL345_ADDRESS 0x53
  #undef INTERNAL_I2C_PULLUPS
#endif

#if defined(CRIUS_LITE)
  #define ITG3200
  #define ADXL345
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  Y; gyroADC[PITCH] = -X; gyroADC[YAW] = -Z;}
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -X; accADC[PITCH]  = -Y; accADC[YAW]  =  Z;}
#endif

#if defined(CRIUS_SE)
  #define ITG3200
  #define BMA180
  #define HMC5883
  #define BMP085
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -X; accADC[PITCH]  = -Y; accADC[YAW]  =  Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  Y; gyroADC[PITCH] = -X; gyroADC[YAW] = -Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  X; magADC[PITCH]  =  Y; magADC[YAW]  = -Z;}
#endif

#if defined(BOARD_PROTO_1)
  #define MPU6050
  #define HMC5883
  #define MS561101BA
  #define ACC_ORIENTATION(Y, X, Z)  {accADC[ROLL]  = -Y; accADC[PITCH]  = -X; accADC[YAW]  =  Z;}
  #define GYRO_ORIENTATION(Y, X, Z) {gyroADC[ROLL] =  X; gyroADC[PITCH] = -Y; gyroADC[YAW] = -Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  X; magADC[PITCH]  =  Y; magADC[YAW]  = -Z;}
  #define MS561101BA_ADDRESS 0x76
  #define STABLEPIN_PINMODE pinMode (A2, OUTPUT);
  #define STABLEPIN_ON PORTC |= (1<<2);
  #define STABLEPIN_OFF PORTC &= ~(1<<2);
#endif

#if defined(BOARD_PROTO_2)
  #define MPU6050
  #define MAG3110
  #define MS561101BA
  #define ACC_ORIENTATION(Y, X, Z)  {accADC[ROLL]  = -Y; accADC[PITCH]  = -X; accADC[YAW]  =  Z;}
  #define GYRO_ORIENTATION(Y, X, Z) {gyroADC[ROLL] =  X; gyroADC[PITCH] = -Y; gyroADC[YAW] = -Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  X; magADC[PITCH]  =  Y; magADC[YAW]  =  Z;}
  #define MPU6050_I2C_AUX_MASTER
  #define MS561101BA_ADDRESS 0x76
  #define STABLEPIN_PINMODE pinMode (A2, OUTPUT);
  #define STABLEPIN_ON PORTC |= (1<<2);
  #define STABLEPIN_OFF PORTC &= ~(1<<2);
#endif

#if defined(GY_80)
  #define L3G4200D
  #define ADXL345
  #define HMC5883
  #define BMP085
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -X; accADC[PITCH]  = -Y; accADC[YAW]  =  Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  Y; gyroADC[PITCH] = -X; gyroADC[YAW] = -Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  X; magADC[PITCH]  =  Y; magADC[YAW]  = -Z;}
  #undef INTERNAL_I2C_PULLUPS
  #define ADXL345_ADDRESS 0x53
#endif

#if defined(GY_85)
  #define ITG3200
  #define ADXL345
  #define HMC5883
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -X; accADC[PITCH]  = -Y; accADC[YAW]  =  Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  Y; gyroADC[PITCH] = -X; gyroADC[YAW] = -Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  X; magADC[PITCH]  =  Y; magADC[YAW]  = -Z;}
  #undef INTERNAL_I2C_PULLUPS
  #define ADXL345_ADDRESS 0x53
#endif

#if defined(GY_86)
  #define MPU6050
  #define HMC5883
  #define MS561101BA
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -X; accADC[PITCH]  = -Y; accADC[YAW]  =  Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  Y; gyroADC[PITCH] = -X; gyroADC[YAW] = -Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  X; magADC[PITCH]  =  Y; magADC[YAW]  = -Z;}
  #define MPU6050_I2C_AUX_MASTER // MAG connected to the AUX I2C bus of MPU6050
  #undef INTERNAL_I2C_PULLUPS
#endif

#if defined(INNOVWORKS_10DOF)
  #define ITG3200
  #define BMA180
  #define BMP085
  #define HMC5883
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -X; accADC[PITCH]  = -Y; accADC[YAW]  = Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] = Y; gyroADC[PITCH] = -X; gyroADC[YAW] = -Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  = X; magADC[PITCH]  = Y; magADC[YAW]  = -Z;}
  #undef INTERNAL_I2C_PULLUPS
#endif

#if defined(INNOVWORKS_6DOF)
  #define ITG3200
  #define BMA180
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -X; accADC[PITCH]  = -Y; accADC[YAW]  = Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] = Y; gyroADC[PITCH] = -X; gyroADC[YAW] = -Z;}
  #undef INTERNAL_I2C_PULLUPS
#endif

#if defined(IOI_MINI_MULTIWII)
  #define ITG3200
  #define BMA180
  #define HMC5883
  #define BMP085
  #define ACC_ORIENTATION(X, Y, Z) {accADC[ROLL] = -X; accADC[PITCH] = -Y; accADC[YAW] = Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] = Y; gyroADC[PITCH] = -X; gyroADC[YAW] = -Z;}
  #define MAG_ORIENTATION(X, Y, Z) {magADC[ROLL] = -Y; magADC[PITCH] = X; magADC[YAW] = -Z;} 
#endif

#if defined(Bobs_6DOF_V1)
  #define ITG3200
  #define BMA180
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -X; accADC[PITCH]  = -Y; accADC[YAW]  =  Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  Y; gyroADC[PITCH] = -X; gyroADC[YAW] = -Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  Y; magADC[PITCH]  = -X; magADC[YAW]  = -Z;}
  #undef INTERNAL_I2C_PULLUPS
#endif

#if defined(Bobs_9DOF_V1)
  #define ITG3200
  #define BMA180
  #define BMP085
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -X; accADC[PITCH]  = -Y; accADC[YAW]  =  Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  Y; gyroADC[PITCH] = -X; gyroADC[YAW] = -Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  Y; magADC[PITCH]  = -X; magADC[YAW]  = -Z;}
  #undef INTERNAL_I2C_PULLUPS
#endif

#if defined(Bobs_10DOF_BMP_V1)
  #define ITG3200
  #define BMA180
  #define BMP085  // Bobs 10DOF uses the BMP180 - BMP085 and BMP180 are software compatible
  #define HMC5883
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -X; accADC[PITCH]  = -Y; accADC[YAW]  =  Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  Y; gyroADC[PITCH] = -X; gyroADC[YAW] = -Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  Y; magADC[PITCH]  = -X; magADC[YAW]  = -Z;}
  #undef INTERNAL_IC2_PULLUPS
#endif

#if defined(OPENLRSv2MULTI)
  #define ITG3200
  #define ADXL345
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -X; accADC[PITCH]  = -Y; accADC[YAW]  =  Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  Y; gyroADC[PITCH] = -X; gyroADC[YAW] = -Z;}
  #define ADXL345_ADDRESS 0x53
  
  #define SDO_pin A0
  #define SDI_pin A1        
  #define SCLK_pin A2 
  #define IRQ_pin 2
  #define nSel_pin 4
  #define IRQ_interrupt 0
  
  #define  nIRQ_1 (PIND & 0x04)==0x04 //D2
  #define  nIRQ_0 (PIND & 0x04)==0x00 //D2
  
  #define  nSEL_on PORTD |= 0x10 //D4
  #define  nSEL_off PORTD &= 0xEF //D4
  
  #define  SCK_on PORTC |= 0x04 //C2
  #define  SCK_off PORTC &= 0xFB //C2
  
  #define  SDI_on PORTC |= 0x02 //C1
  #define  SDI_off PORTC &= 0xFD //C1
  
  #define  SDO_1 (PINC & 0x01) == 0x01 //C0
  #define  SDO_0 (PINC & 0x01) == 0x00 //C0
  
  //#### Other interface pinouts ###
  #define GREEN_LED_pin 13
  #define RED_LED_pin A3

  #define Red_LED_ON  PORTC |= _BV(3);
  #define Red_LED_OFF  PORTC &= ~_BV(3);
  
  #define Green_LED_ON  PORTB |= _BV(5);
  #define Green_LED_OFF  PORTB &= ~_BV(5);
  
  #define NOP() __asm__ __volatile__("nop") 
 
  #define RF22B_PWRSTATE_READY    01 
  #define RF22B_PWRSTATE_TX        0x09 
  #define RF22B_PWRSTATE_RX       05 
  #define RF22B_Rx_packet_received_interrupt   0x02 
  #define RF22B_PACKET_SENT_INTERRUPT  04 
  #define RF22B_PWRSTATE_POWERDOWN  00    
  
  unsigned char ItStatus1, ItStatus2;  
  typedef struct   
  { 
   unsigned char reach_1s    : 1; 
  } FlagType; 
  FlagType               Flag;   
#endif



/**************************************************************************************/
/***************              Sensor Type definitions              ********************/
/**************************************************************************************/

#if defined(ADXL345) || defined(BMA020) || defined(BMA180) || defined(NUNCHACK) || defined(MMA7455) || defined(ADCACC) || defined(LIS3LV02) || defined(LSM303DLx_ACC) || defined(MPU6050) || defined(NUNCHUCK)
  #define ACC 1
#else
  #define ACC 0
#endif

#if defined(HMC5883) || defined(HMC5843) || defined(AK8975) || defined(MAG3110)
  #define MAG 1
#else
  #define MAG 0
#endif

#if defined(ITG3200) || defined(L3G4200D) || defined(MPU6050) || defined(WMP)
  #define GYRO 1
#else
  #define GYRO 0
#endif

#if defined(BMP085) || defined(MS561101BA)
  #define BARO 1
#else
  #define BARO 0
#endif

#if defined(GPS_PROMINI_SERIAL)
  #define GPS_SERIAL 0
  #define GPS_PROMINI
  #define GPS_BAUD   GPS_PROMINI_SERIAL
#endif

#if defined(GPS_SERIAL)  || defined(I2C_GPS) || defined(GPS_FROM_OSD) || defined(TINY_GPS)
  #define GPS 1
#else
  #define GPS 0
#endif

#if defined(SRF02) || defined(SRF08) || defined(SRF10) || defined(SRC235) || defined(TINY_GPS_SONAR)
  #define SONAR 1
#else
  #define SONAR 0
#endif


/**************************************************************************************/
/***************      Multitype decleration for the GUI's          ********************/
/**************************************************************************************/
#if defined(TRI)
  #define MULTITYPE 1
#elif defined(QUADP)
  #define MULTITYPE 2
#elif defined(QUADX)
  #define MULTITYPE 3
#elif defined(BI)
  #define MULTITYPE 4
#elif defined(GIMBAL)
  #define MULTITYPE 5
#elif defined(Y6)
  #define MULTITYPE 6
#elif defined(HEX6)
  #define MULTITYPE 7
#elif defined(FLYING_WING)
  #define MULTITYPE 8
#elif defined(Y4)
  #define MULTITYPE 9
#elif defined(HEX6X)
  #define MULTITYPE 10
#elif defined(OCTOX8)
  #define MULTITYPE 11   //the JAVA GUI is the same for all 8 motor configs 
#elif defined(OCTOFLATP)
  #define MULTITYPE 12   //12  for MultiWinGui
#elif defined(OCTOFLATX)
  #define MULTITYPE 13   //13  for MultiWinGui 
#elif defined(AIRPLANE)    
  #define MULTITYPE 14    
#elif defined (HELI_120_CCPM)   
  #define MULTITYPE 15      
#elif defined (HELI_90_DEG)   
  #define MULTITYPE 16      
#elif defined(VTAIL4)
 #define MULTITYPE 17
#endif

/**************************************************************************************/
/***************          Some unsorted "chain" defines            ********************/
/**************************************************************************************/
#if defined (AIRPLANE) || defined(FLYING_WING)
  #define FIXEDWING
#endif

#if defined (AIRPLANE) || defined(HELICOPTER) && defined(PROMINI) 
  #if defined(D12_POWER)
    #define SERVO_4_PINMODE            ;  // D12
    #define SERVO_4_PIN_HIGH           ;
    #define SERVO_4_PIN_LOW            ;
  #else
    #define POWERPIN_PINMODE           ;
    #define POWERPIN_ON                ;
    #define POWERPIN_OFF               ;
  #endif
#endif

#if defined(HELI_120_CCPM) || defined(HELI_90_DEG)
  #define HELICOPTER
#endif

#if defined(POWERMETER_HARD) || defined(POWERMETER_SOFT)
  #define POWERMETER
#endif

//all new Special RX's must be added here
//this is to avoid confusion :)
#if !defined(SERIAL_SUM_PPM) && !defined(SPEKTRUM) && !defined(SBUS) && !defined(RCSERIALERIAL)
  #define STANDARD_RX
#endif


// Spektrum Satellite
#if defined(SPEKTRUM)
  #define SPEK_MAX_CHANNEL 7
  #define SPEK_FRAME_SIZE 16
  #if (SPEKTRUM == 1024)
    #define SPEK_CHAN_SHIFT  2       // Assumes 10 bit frames, that is 1024 mode.
    #define SPEK_CHAN_MASK   0x03    // Assumes 10 bit frames, that is 1024 mode.
  #endif
  #if (SPEKTRUM == 2048)
    #define SPEK_CHAN_SHIFT  3       // Assumes 11 bit frames, that is 2048 mode.
    #define SPEK_CHAN_MASK   0x07    // Assumes 11 bit frames, that is 2048 mode.
  #endif
#endif


/**************************************************************************************/
/***************             motor and servo numbers               ********************/
/**************************************************************************************/
#if defined(BI) || defined(TRI) || defined(SERVO_TILT) || defined(GIMBAL) || defined(FLYING_WING) || defined(AIRPLANE) || defined(CAMTRIG) || defined(HELICOPTER) || defined(SERVO_MIX_TILT)
  #define SERVO
#endif

#if defined(GIMBAL)
  #define NUMBER_MOTOR     0
  #define PRI_SERVO_FROM   1 // use servo from 1 to 2
  #define PRI_SERVO_TO     2
#elif defined(FLYING_WING)
  #define NUMBER_MOTOR     1
  #define PRI_SERVO_FROM   1 // use servo from 1 to 2
  #define PRI_SERVO_TO     2
#elif defined(AIRPLANE)
  #define NUMBER_MOTOR     0
  #define PRI_SERVO_FROM   4 // use servo from 4 to 8
  #define PRI_SERVO_TO     8
#elif defined(BI)
  #define NUMBER_MOTOR     2
  #define PRI_SERVO_FROM   5 // use servo from 5 to 6
  #define PRI_SERVO_TO     6
#elif defined(TRI)
  #define NUMBER_MOTOR     3
  #define PRI_SERVO_FROM   6 // use only servo 6
  #define PRI_SERVO_TO     6
#elif defined(QUADP) || defined(QUADX) || defined(Y4)|| defined(VTAIL4)
  #define NUMBER_MOTOR     4
#elif defined(Y6) || defined(HEX6) || defined(HEX6X)
  #define NUMBER_MOTOR     6
#elif defined(OCTOX8) || defined(OCTOFLATP) || defined(OCTOFLATX)
  #define NUMBER_MOTOR     8
#elif defined(HELICOPTER)
  #define NUMBER_MOTOR     0
  #define PRI_SERVO_FROM   4 // use servo from 4 to 8
  #define PRI_SERVO_TO     8
#endif


#if (defined(SERVO_TILT)|| defined(SERVO_MIX_TILT))&& defined(CAMTRIG)
  #define SEC_SERVO_FROM   1 // use servo from 1 to 3
  #define SEC_SERVO_TO     3
#else
  #if defined(SERVO_TILT)|| defined(SERVO_MIX_TILT)
    // if A0 and A1 is taken by motors, we can use A2 and 12 for Servo tilt
    #if defined(A0_A1_PIN_HEX) && (NUMBER_MOTOR == 6) && defined(PROMINI)
      #define SEC_SERVO_FROM   3 // use servo from 3 to 4
      #define SEC_SERVO_TO     4
    #else
      #define SEC_SERVO_FROM   1 // use servo from 1 to 2
      #define SEC_SERVO_TO     2
    #endif
  #endif
  #if defined(CAMTRIG)
    #define SEC_SERVO_FROM   3 // use servo 3
    #define SEC_SERVO_TO     3
  #endif
#endif
/**********************   Sort the Servos for the moust ideal SW PWM     ************************/
// this define block sorts the above slected servos to be in a simple order from 1 - (count of total servos)
// its pretty fat but its the best way i found to get less compiled code and max speed in the ISR without loosing its flexibility
#if (PRI_SERVO_FROM == 1) || (SEC_SERVO_FROM == 1)
  #define LAST_LOW SERVO_1_PIN_LOW
  #define SERVO_1_HIGH SERVO_1_PIN_HIGH
  #define SERVO_1_LOW SERVO_1_PIN_LOW
  #define SERVO_1_ARR_POS  0
#endif
#if (PRI_SERVO_FROM <= 2 && PRI_SERVO_TO >= 2) || (SEC_SERVO_FROM <= 2 && SEC_SERVO_TO >= 2) 
  #undef LAST_LOW
  #define LAST_LOW SERVO_2_PIN_LOW
  #if !defined(SERVO_1_HIGH)
    #define SERVO_1_HIGH SERVO_2_PIN_HIGH
    #define SERVO_1_LOW SERVO_2_PIN_LOW  
    #define SERVO_1_ARR_POS 1
  #else
    #define SERVO_2_HIGH SERVO_2_PIN_HIGH
    #define SERVO_2_LOW SERVO_2_PIN_LOW   
    #define SERVO_2_ARR_POS 1
  #endif
#endif
#if (PRI_SERVO_FROM <= 3 && PRI_SERVO_TO >= 3) || (SEC_SERVO_FROM <= 3 && SEC_SERVO_TO >= 3) 
  #undef LAST_LOW
  #define LAST_LOW SERVO_3_PIN_LOW
  #if !defined(SERVO_1_HIGH)
    #define SERVO_1_HIGH SERVO_3_PIN_HIGH
    #define SERVO_1_LOW SERVO_3_PIN_LOW
    #define SERVO_1_ARR_POS 2 
  #elif !defined(SERVO_2_HIGH)
    #define SERVO_2_HIGH SERVO_3_PIN_HIGH
    #define SERVO_2_LOW SERVO_3_PIN_LOW 
    #define SERVO_2_ARR_POS 2 
  #else
    #define SERVO_3_HIGH SERVO_3_PIN_HIGH
    #define SERVO_3_LOW SERVO_3_PIN_LOW  
    #define SERVO_3_ARR_POS 2   
  #endif
#endif
#if (PRI_SERVO_FROM <= 4 && PRI_SERVO_TO >= 4) || (SEC_SERVO_FROM <= 4 && SEC_SERVO_TO >= 4) 
  #undef LAST_LOW
  #define LAST_LOW SERVO_4_PIN_LOW
  #if !defined(SERVO_1_HIGH)
    #define SERVO_1_HIGH SERVO_4_PIN_HIGH
    #define SERVO_1_LOW SERVO_4_PIN_LOW
    #define SERVO_1_ARR_POS 3  
  #elif !defined(SERVO_2_HIGH)
    #define SERVO_2_HIGH SERVO_4_PIN_HIGH
    #define SERVO_2_LOW SERVO_4_PIN_LOW
    #define SERVO_2_ARR_POS 3
  #elif !defined(SERVO_3_HIGH)
    #define SERVO_3_HIGH SERVO_4_PIN_HIGH
    #define SERVO_3_LOW SERVO_4_PIN_LOW
    #define SERVO_3_ARR_POS 3    
  #else
    #define SERVO_4_HIGH SERVO_4_PIN_HIGH
    #define SERVO_4_LOW SERVO_4_PIN_LOW 
    #define SERVO_4_ARR_POS 3     
  #endif
#endif
#if (PRI_SERVO_FROM <= 5 && PRI_SERVO_TO >= 5) || (SEC_SERVO_FROM <= 5 && SEC_SERVO_TO >= 5)
  #undef LAST_LOW
  #define LAST_LOW SERVO_5_PIN_LOW
  #if !defined(SERVO_1_HIGH)
    #define SERVO_1_HIGH SERVO_5_PIN_HIGH
    #define SERVO_1_LOW SERVO_5_PIN_LOW
    #define SERVO_1_ARR_POS 4   
  #elif !defined(SERVO_2_HIGH)
    #define SERVO_2_HIGH SERVO_5_PIN_HIGH
    #define SERVO_2_LOW SERVO_5_PIN_LOW
    #define SERVO_2_ARR_POS 4  
  #elif !defined(SERVO_3_HIGH)
    #define SERVO_3_HIGH SERVO_5_PIN_HIGH
    #define SERVO_3_LOW SERVO_5_PIN_LOW
    #define SERVO_3_ARR_POS 4   
  #elif !defined(SERVO_4_HIGH)
    #define SERVO_4_HIGH SERVO_5_PIN_HIGH
    #define SERVO_4_LOW SERVO_5_PIN_LOW
    #define SERVO_4_ARR_POS 4   
  #else
    #define SERVO_5_HIGH SERVO_5_PIN_HIGH
    #define SERVO_5_LOW SERVO_5_PIN_LOW 
    #define SERVO_5_ARR_POS 4     
  #endif
#endif
#if (PRI_SERVO_FROM <= 6 && PRI_SERVO_TO >= 6) || (SEC_SERVO_FROM <= 6 && SEC_SERVO_TO >= 6)
  #undef LAST_LOW
  #define LAST_LOW SERVO_6_PIN_LOW
  #if !defined(SERVO_1_HIGH)
    #define SERVO_1_HIGH SERVO_6_PIN_HIGH
    #define SERVO_1_LOW SERVO_6_PIN_LOW 
    #define SERVO_1_ARR_POS 5 
  #elif !defined(SERVO_2_HIGH)
    #define SERVO_2_HIGH SERVO_6_PIN_HIGH
    #define SERVO_2_LOW SERVO_6_PIN_LOW
    #define SERVO_2_ARR_POS 5 
  #elif !defined(SERVO_3_HIGH)
    #define SERVO_3_HIGH SERVO_6_PIN_HIGH
    #define SERVO_3_LOW SERVO_6_PIN_LOW
    #define SERVO_3_ARR_POS 5   
  #elif !defined(SERVO_4_HIGH)
    #define SERVO_4_HIGH SERVO_6_PIN_HIGH
    #define SERVO_4_LOW SERVO_6_PIN_LOW 
    #define SERVO_4_ARR_POS 5  
  #elif !defined(SERVO_5_HIGH)
    #define SERVO_5_HIGH SERVO_6_PIN_HIGH
    #define SERVO_5_LOW SERVO_6_PIN_LOW 
    #define SERVO_5_ARR_POS 5  
  #else
    #define SERVO_6_HIGH SERVO_6_PIN_HIGH
    #define SERVO_6_LOW SERVO_6_PIN_LOW  
    #define SERVO_6_ARR_POS 5   
  #endif
#endif
#if (PRI_SERVO_FROM <= 7 && PRI_SERVO_TO >= 7) || (SEC_SERVO_FROM <= 7 && SEC_SERVO_TO >= 7)
  #undef LAST_LOW
  #define LAST_LOW SERVO_7_PIN_LOW
  #if !defined(SERVO_1_HIGH)
    #define SERVO_1_HIGH SERVO_7_PIN_HIGH
    #define SERVO_1_LOW SERVO_7_PIN_LOW 
    #define SERVO_1_ARR_POS 6 
  #elif !defined(SERVO_2_HIGH)
    #define SERVO_2_HIGH SERVO_7_PIN_HIGH
    #define SERVO_2_LOW SERVO_7_PIN_LOW
    #define SERVO_2_ARR_POS 6 
  #elif !defined(SERVO_3_HIGH)
    #define SERVO_3_HIGH SERVO_7_PIN_HIGH
    #define SERVO_3_LOW SERVO_7_PIN_LOW
    #define SERVO_3_ARR_POS 6   
  #elif !defined(SERVO_4_HIGH)
    #define SERVO_4_HIGH SERVO_7_PIN_HIGH
    #define SERVO_4_LOW SERVO_7_PIN_LOW 
    #define SERVO_4_ARR_POS 6  
  #elif !defined(SERVO_5_HIGH)
    #define SERVO_5_HIGH SERVO_7_PIN_HIGH
    #define SERVO_5_LOW SERVO_7_PIN_LOW 
    #define SERVO_5_ARR_POS 6  
  #elif !defined(SERVO_6_HIGH)
    #define SERVO_6_HIGH SERVO_7_PIN_HIGH
    #define SERVO_6_LOW SERVO_7_PIN_LOW 
    #define SERVO_6_ARR_POS 6  
  #else
    #define SERVO_7_HIGH SERVO_7_PIN_HIGH
    #define SERVO_7_LOW SERVO_7_PIN_LOW  
    #define SERVO_7_ARR_POS 6   
  #endif
#endif
#if (PRI_SERVO_FROM <= 8 && PRI_SERVO_TO >= 8) || (SEC_SERVO_FROM <= 8 && SEC_SERVO_TO >= 8) 
  #undef LAST_LOW
  #define LAST_LOW SERVO_8_PIN_LOW
  #if !defined(SERVO_1_HIGH)
    #define SERVO_1_HIGH SERVO_8_PIN_HIGH
    #define SERVO_1_LOW SERVO_8_PIN_LOW 
    #define SERVO_1_ARR_POS 7 
  #elif !defined(SERVO_2_HIGH)
    #define SERVO_2_HIGH SERVO_8_PIN_HIGH
    #define SERVO_2_LOW SERVO_8_PIN_LOW
    #define SERVO_2_ARR_POS 7
  #elif !defined(SERVO_3_HIGH)
    #define SERVO_3_HIGH SERVO_8_PIN_HIGH
    #define SERVO_3_LOW SERVO_8_PIN_LOW
    #define SERVO_3_ARR_POS 7  
  #elif !defined(SERVO_4_HIGH)
    #define SERVO_4_HIGH SERVO_8_PIN_HIGH
    #define SERVO_4_LOW SERVO_8_PIN_LOW
    #define SERVO_4_ARR_POS 7  
  #elif !defined(SERVO_5_HIGH)
    #define SERVO_5_HIGH SERVO_8_PIN_HIGH
    #define SERVO_5_LOW SERVO_8_PIN_LOW 
    #define SERVO_5_ARR_POS 7  
  #elif !defined(SERVO_6_HIGH)
    #define SERVO_6_HIGH SERVO_8_PIN_HIGH
    #define SERVO_6_LOW SERVO_8_PIN_LOW 
    #define SERVO_6_ARR_POS 7 
  #elif !defined(SERVO_7_HIGH)
    #define SERVO_7_HIGH SERVO_8_PIN_HIGH
    #define SERVO_7_LOW SERVO_8_PIN_LOW 
    #define SERVO_7_ARR_POS 7  
  #else
    #define SERVO_8_HIGH SERVO_8_PIN_HIGH
    #define SERVO_8_LOW SERVO_8_PIN_LOW  
    #define SERVO_8_ARR_POS 7   
  #endif
#endif

/**************************************************************************************/
/***************                       I2C GPS                     ********************/
/**************************************************************************************/
#if defined(I2C_GPS)
  #define I2C_GPS_ADDRESS                         0x20 //7 bits       
///////////////////////////////////////////////////////////////////////////////////////////////////
// I2C GPS NAV registers
///////////////////////////////////////////////////////////////////////////////////////////////////
//
#define I2C_GPS_STATUS_00                            00 //(Read only)
        #define I2C_GPS_STATUS_NEW_DATA       0x01      // New data is available (after every GGA frame)
        #define I2C_GPS_STATUS_2DFIX          0x02      // 2dfix achieved
        #define I2C_GPS_STATUS_3DFIX          0x04      // 3dfix achieved
        #define I2C_GPS_STATUS_WP_REACHED     0x08      // Active waypoint has been reached (not cleared until new waypoint is set)
        #define I2C_GPS_STATUS_NUMSATS        0xF0      // Number of sats in view

#define I2C_GPS_COMMAND                              01 // (write only)
        #define I2C_GPS_COMMAND_POSHOLD       0x01      // Start position hold at the current gps positon
        #define I2C_GPS_COMMAND_START_NAV     0x02      // get the WP from the command and start navigating toward it
        #define I2C_GPS_COMMAND_SET_WP        0x03      // copy current position to given WP      
        #define I2C_GPS_COMMAND_UPDATE_PIDS   0x04      // update PI and PID controllers from the PID registers, this must be called after a pid register is changed
        #define I2C_GPS_COMMAND_NAV_OVERRIDE  0x05      // do not nav since we tring to controll the copter manually (not implemented yet)
        #define I2C_GPS_COMMAND_STOP_NAV      0x06      // Stop navigation (zeroes out nav_lat and nav_lon
        #define I2C_GPS_COMMAND__7            0x07
        #define I2C_GPS_COMMAND__8            0x08      
        #define I2C_GPS_COMMAND__9            0x09
        #define I2C_GPS_COMMAND__a            0x0a
        #define I2C_GPS_COMMAND__b            0x0b
        #define I2C_GPS_COMMAND__c            0x0c
        #define I2C_GPS_COMMAND__d            0x0d
        #define I2C_GPS_COMMAND__e            0x0e
        #define I2C_GPS_COMMAND__f            0x0f

        #define I2C_GPS_COMMAND_WP_MASK       0xF0       // Waypoint number

#define I2C_GPS_WP_REG                              02   // Waypoint register (Read only)
        #define I2C_GPS_WP_REG_ACTIVE_MASK    0x0F       // Active Waypoint lower 4 bits
        #define I2C_GPS_WP_REG_PERVIOUS_MASK  0xF0       // pervious Waypoint upper 4 bits
        
#define I2C_GPS_REG_VERSION                         03   // Version of the I2C_NAV SW uint8_t
#define I2C_GPS_REG_RES2                            04   // reserved for future use (uint8_t)
#define I2C_GPS_REG_RES3                            05   // reserved for future use (uint8_t)
#define I2C_GPS_REG_RES4                            06   // reserved for future use (uint8_t)


#define I2C_GPS_LOCATION                            07   // current location 8 byte (lat, lon) int32_t
#define I2C_GPS_NAV_LAT                             15   // Desired banking towards north/south int16_t
#define I2C_GPS_NAV_LON                             17   // Desired banking toward east/west    int16_t
#define I2C_GPS_WP_DISTANCE                         19   // Distance to current WP in cm uint32
#define I2C_GPS_WP_TARGET_BEARING                   23   // bearing towards current wp 1deg = 1000 int16_t
#define I2C_GPS_NAV_BEARING                         25   // crosstrack corrected bearing towards current wp 1deg = 1000 int16_t
#define I2C_GPS_HOME_TO_COPTER_BEARING              27   // bearing from home to copter 1deg = 1000 int16_t
#define I2C_GPS_DISTANCE_TO_HOME                    29   // distance to home in m int16_t
        
#define I2C_GPS_GROUND_SPEED                        31   // GPS ground speed in m/s*100 (uint16_t)      (Read Only)
#define I2C_GPS_ALTITUDE                            33   // GPS altitude in meters (uint16_t)           (Read Only)
#define I2C_GPS_GROUND_COURSE			    35   // GPS ground course (uint16_t)
#define I2C_GPS_RES1                                37   // reserved for future use (uint16_t)
#define I2C_GPS_TIME                                39   // UTC Time from GPS in hhmmss.sss * 100 (uint32_t)(unneccesary precision) (Read Only)

//Writeable registers from here

#define I2C_GPS_CROSSTRACK_GAIN                     43    // Crosstrack gain *100 (1 - 0.01 100 - 1) uint8_t
#define I2C_GPS_SPEED_MIN                           44    // Minimum navigation speed cm/s uint8_t
#define I2C_GPS_SPEED_MAX                           45    // Maximum navigation speed cm/s uint16_t
#define I2C_GPS_RESERVED                            47    // Reserved for future use
#define I2C_GPS_WP_RADIUS                           49    // Radius of the wp in cm, within this radius we consider the wp reached (uint16_t)

#define I2C_GPS_NAV_FLAGS                           51    // Controls various functions of the I2C-GPS-NAV module
        #define I2C_NAV_FLAG_GPS_FILTER          0x80     // If this bit set GPS coordinates are filtered via a 5 element moving average filter
        #define I2C_NAV_FLAG_LOW_SPEED_D_FILTER  0x40     // If speed below .5m/s ignore D term in POSHOLD_RATE, this supposed to filter out noise

#define I2C_GPS_HOLD_P                              52    // poshold_P  *100 uint16_t
#define I2C_GPS_HOLD_I                              53    // poshold_I  *100 uint16_t
#define I2C_GPS_HOLD_IMAX                           54    // poshold_IMAX *1 uint8_t

#define I2C_GPS_HOLD_RATE_P                         55    // poshold_rate_P  *10 uint16_t
#define I2C_GPS_HOLD_RATE_I                         56    // poshold_rate_I  *100 uint16_t
#define I2C_GPS_HOLD_RATE_D                         57    // poshold_rate_D  *1000 uint16_t
#define I2C_GPS_HOLD_RATE_IMAX                      58    // poshold_rate_IMAX *1 uint8_t

#define I2C_GPS_NAV_P                               59    // nav_P  *10 uint16_t
#define I2C_GPS_NAV_I                               60    // nav_I  *100 uint16_t
#define I2C_GPS_NAV_D                               61    // nav_D  *1000 uint16_t
#define I2C_GPS_NAV_IMAX                            62    // nav_IMAX *1 uint8_t

#define I2C_GPS_WP0                                 63   //Waypoint 0 used for RTH location      (R/W)
#define	I2C_GPS_WP1		                    74
#define	I2C_GPS_WP2		                    85
#define	I2C_GPS_WP3		                    96
#define	I2C_GPS_WP4		                    107
#define	I2C_GPS_WP5		                    118
#define	I2C_GPS_WP6		                    129
#define	I2C_GPS_WP7		                    140
#define	I2C_GPS_WP8		                    151
#define	I2C_GPS_WP9		                    162
#define	I2C_GPS_WP10		                    173
#define	I2C_GPS_WP11		                    184
#define	I2C_GPS_WP12		                    195
#define	I2C_GPS_WP13		                    206
#define	I2C_GPS_WP14		                    217
#define	I2C_GPS_WP15		                    228
///////////////////////////////////////////////////////////////////////////////////////////////////
// End register definition 
///////////////////////////////////////////////////////////////////////////////////////////////////

#endif

#if !(defined(DISPLAY_2LINES)) && !(defined(DISPLAY_MULTILINE))
  #if (defined(LCD_VT100)) || (defined(OLED_I2C_128x64))
    #define DISPLAY_MULTILINE
  #else
    #define DISPLAY_2LINES
  #endif
#endif

#if (defined(LCD_VT100))
  #if !(defined(MULTILINE_PRE))
    #define MULTILINE_PRE 6
  #endif
  #if !(defined(MULTILINE_POST))
    #define MULTILINE_POST 9
  #endif
#elif (defined(OLED_I2C_128x64))
  #if !(defined(MULTILINE_PRE))
    #define MULTILINE_PRE 3
  #endif
  #if !(defined(MULTILINE_POST))
    #define MULTILINE_POST 5
  #endif
#endif

/**************************************************************************************/
/***************               Error Checking Section              ********************/
/**************************************************************************************/

#if (defined(LCD_CONF) || defined(LCD_TELEMETRY)) && !(defined(LCD_SERIAL3W) || defined(LCD_TEXTSTAR) || defined(LCD_VT100) || defined(LCD_ETPP) || defined(LCD_LCD03) || defined(OLED_I2C_128x64) )
  #error "LCD_CONF or LCD_TELEMETRY defined, and choice of LCD not defined.  Uncomment one of LCD_SERIAL3W or LCD_TEXTSTAR or LCD_VT100 or LCD_ETPP or LCD_LCD03 or OLED_I2C_128x64"
#endif

#if defined(POWERMETER) && !(defined(VBAT))
        #error "to use powermeter, you must also define and configure VBAT"
#endif

#if defined(LCD_TELEMETRY_AUTO) && !(defined(LCD_TELEMETRY))
        #error "to use automatic telemetry, you MUST also define and configure LCD_TELEMETRY"
#endif

enum mwc_flag {
  FLAG_OK_TO_ARM,
  FLAG_ARMED,
  FLAG_I2C_INIT_DONE,
  FLAG_ACC_CALIBRATED,
  FLAG_NUNCHUKDATA,
  FLAG_ACC_MODE,
  FLAG_MAG_MODE,
  FLAG_BARO_MODE,
  FLAG_GPS_HOME_MODE,
  FLAG_GPS_HOLD_MODE,
  FLAG_HEADFREE_MODE,
  FLAG_PASSTHRU_MODE,

  FLAG_GPS_FIX,
  FLAG_GPS_FIX_HOME,

  FLAG_SMALL_ANGLES_25,

  FLAG_CNT
};
