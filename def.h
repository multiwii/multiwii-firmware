#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
  #define PROMINI
#endif
#if defined(__AVR_ATmega32U4__)
  #define PROMICRO
#endif
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
  #define MEGA
#endif

#if defined(PROMINI) && !defined(MONGOOSE1_0)
  #define LEDPIN_PINMODE             pinMode (13, OUTPUT);
  #define LEDPIN_TOGGLE              PINB |= 1<<5;     //switch LEDPIN state (digital PIN 13)
  #define LEDPIN_OFF                 PORTB &= ~(1<<5);
  #define LEDPIN_ON                  PORTB |= (1<<5);
  #if !defined(RCAUXPIN8)
    #define BUZZERPIN_PINMODE          pinMode (8, OUTPUT);
    #define BUZZERPIN_ON               PORTB |= 1;
    #define BUZZERPIN_OFF              PORTB &= ~1;
  #else
    #define BUZZERPIN_PINMODE          ;
    #define BUZZERPIN_ON               ;
    #define BUZZERPIN_OFF              ;
    #define RCAUXPIN
  #endif
  #if !defined(RCAUXPIN12)
    #define POWERPIN_PINMODE           pinMode (12, OUTPUT);
    #define POWERPIN_ON                PORTB |= 1<<4;
    #define POWERPIN_OFF               PORTB &= ~(1<<4); //switch OFF WMP, digital PIN 12
    #define I2C_PULLUPS_ENABLE         PORTC |= 1<<4; PORTC |= 1<<5;   // PIN A4&A5 (SDA&SCL)
  #else
    #define POWERPIN_PINMODE           ;
    #define POWERPIN_ON                ;
    #define POWERPIN_OFF               ;
    #define RCAUXPIN
  #endif
  #define I2C_PULLUPS_DISABLE        PORTC &= ~(1<<4); PORTC &= ~(1<<5);
  #define PINMODE_LCD                pinMode(0, OUTPUT);
  #define LCDPIN_OFF                 PORTD &= ~1; //switch OFF digital PIN 0
  #define LCDPIN_ON                  PORTD |= 1;
  #define STABLEPIN_PINMODE          ;
  #define STABLEPIN_ON               ;
  #define STABLEPIN_OFF              ;
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
  #define ISR_UART                   ISR(USART_UDRE_vect)
  #define V_BATPIN                   A3    // Analog PIN 3
  #define PSENSORPIN                 A2    // Analog PIN 2
  
  #define SOFT_PWM_1_PIN_HIGH        PORTD |= 1<<5;
  #define SOFT_PWM_1_PIN_LOW         PORTD &= ~(1<<5);
  #define SOFT_PWM_2_PIN_HIGH        PORTD |= 1<<6;
  #define SOFT_PWM_2_PIN_LOW         PORTD &= ~(1<<6);
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
  #define SERVO_4_PINMODE            pinMode(12,OUTPUT); // new       - alt TILT_ROLL
  #define SERVO_4_PIN_HIGH           PORTB |= 1<<4;
  #define SERVO_4_PIN_LOW            PORTB &= ~(1<<4);
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

#if defined(PROMICRO)
  #define LEDPIN_PINMODE             //
  #define LEDPIN_TOGGLE              PIND |= 1<<5;     //switch LEDPIN state (Port D5)
  #define LEDPIN_OFF                 PORTD &= ~(1<<5);
  #define LEDPIN_ON                  PORTD |= (1<<5);
  #if !defined(D8BUZZER) && !defined(A32U4ALLPINS)
    #define BUZZERPIN_PINMODE          pinMode (1, OUTPUT);
    #define BUZZERPIN_ON               PORTD |= 1<<3;
    #define BUZZERPIN_OFF              PORTD &= ~(1<<3);
  #else
    #define BUZZERPIN_PINMODE          pinMode (8, OUTPUT);
    #define BUZZERPIN_ON               PORTB |= 1<<4;
    #define BUZZERPIN_OFF              PORTB &= ~(1<<4);  
  #endif
  #define POWERPIN_PINMODE           //
  #define POWERPIN_ON                //
  #define POWERPIN_OFF               //
  #define I2C_PULLUPS_ENABLE         PORTD |= 1<<0; PORTD |= 1<<1;   // PIN 2&3 (SDA&SCL)
  #define I2C_PULLUPS_DISABLE        PORTD &= ~(1<<0); PORTD &= ~(1<<1);
  #define PINMODE_LCD                pinMode(0, OUTPUT);
  #define LCDPIN_OFF                 PORTD &= ~1;
  #define LCDPIN_ON                  PORTD |= 1;
  #define STABLEPIN_PINMODE          ;
  #define STABLEPIN_ON               ;
  #define STABLEPIN_OFF              ;
  #define PPM_PIN_INTERRUPT          //attachInterrupt(3, rxInt, RISING);// not used
  #define SPEK_SERIAL_VECT           USART1_RX_vect
  #define SPEK_DATA_REG              UDR1
  #define USB_CDC_TX                 3
  #define USB_CDC_RX                 2
  
  //soft PWM Pins  
  #define SOFT_PWM_1_PIN_HIGH        PORTD |= 1<<4;
  #define SOFT_PWM_1_PIN_LOW         PORTD &= ~(1<<4);
  #define SOFT_PWM_2_PIN_HIGH        PORTF |= 1<<5;
  #define SOFT_PWM_2_PIN_LOW         PORTF &= ~(1<<5);
  #define SOFT_PWM_3_PIN_HIGH        PORTF |= 1<<7;
  #define SOFT_PWM_3_PIN_LOW         PORTF &= ~(1<<7);
  #define SOFT_PWM_4_PIN_HIGH        PORTF |= 1<<6;
  #define SOFT_PWM_4_PIN_LOW         PORTF &= ~(1<<6);
  
  // Servos
  #define SERVO_1_PINMODE   pinMode(A0,OUTPUT);
  #define SERVO_1_PIN_HIGH  PORTF|= 1<<7;
  #define SERVO_1_PIN_LOW   PORTF &= ~(1<<7);
  #define SERVO_2_PINMODE   pinMode(A1,OUTPUT);
  #define SERVO_2_PIN_HIGH  PORTF |= 1<<6;
  #define SERVO_2_PIN_LOW   PORTF &= ~(1<<6);
  #define SERVO_3_PINMODE   pinMode(A2,OUTPUT);
  #define SERVO_3_PIN_HIGH  PORTF |= 1<<5;
  #define SERVO_3_PIN_LOW   PORTF &= ~(1<<5);
  #if !defined(A32U4ALLPINS)
    #define SERVO_4_PINMODE   pinMode(4,OUTPUT);
    #define SERVO_4_PIN_HIGH  PORTD |= 1<<4;
    #define SERVO_4_PIN_LOW   PORTD &= ~(1<<4);
  #else
    #define SERVO_4_PINMODE   pinMode(A3,OUTPUT);
    #define SERVO_4_PIN_HIGH  PORTF |= 1<<4;
    #define SERVO_4_PIN_LOW   PORTF &= ~(1<<4);  
  #endif
  #define SERVO_5_PINMODE   pinMode(6,OUTPUT);
  #define SERVO_5_PIN_HIGH  PORTD |= 1<<7;
  #define SERVO_5_PIN_LOW   PORTD &= ~(1<<7);
  #define SERVO_6_PINMODE   pinMode(5,OUTPUT);
  #define SERVO_6_PIN_HIGH  PORTC|= 1<<6;
  #define SERVO_6_PIN_LOW   PORTC &= ~(1<<6);
  #define SERVO_7_PINMODE   pinMode(10,OUTPUT);
  #define SERVO_7_PIN_HIGH  PORTB |= 1<<6;
  #define SERVO_7_PIN_LOW   PORTB &= ~(1<<6);
  #define SERVO_8_PINMODE   pinMode(9,OUTPUT);
  #define SERVO_8_PIN_HIGH  PORTB |= 1<<5;
  #define SERVO_8_PIN_LOW   PORTB &= ~(1<<5);
  
  #define THROTTLEPIN                2
  #if !defined(A32U4ALLPINS)
    #define ROLLPIN                    4
    #define PITCHPIN                   5
    #define YAWPIN                     6
    #define AUX1PIN                    7
  #else
    #define ROLLPIN                    7
    #define PITCHPIN                   4
    #define YAWPIN                     6
    #define AUX1PIN                    5   
  #endif
  #define AUX2PIN                    0 
  #define AUX3PIN                    1 // unused 
  #define AUX4PIN                    3 // unused 
  #define ISR_UART                   ISR(USART_UDRE_vect)
  #if !defined(A32U4ALLPINS)
    #define V_BATPIN                   A3    // Analog PIN 3
  #else
    #define V_BATPIN                   A4    // Analog PIN 4
  #endif
  #define PSENSORPIN                 A2    // Analog PIN 2 
#endif

#if defined(MEGA)
  #define LEDPIN_PINMODE             pinMode (13, OUTPUT);pinMode (30, OUTPUT);
  #define LEDPIN_TOGGLE              PINB  |= (1<<7); PINC  |= (1<<7);
  #define LEDPIN_ON                  PORTB |= (1<<7); PORTC |= (1<<7);
  #define LEDPIN_OFF                 PORTB &= ~(1<<7);PORTC &= ~(1<<7);
  #define BUZZERPIN_PINMODE          pinMode (32, OUTPUT);
  #define BUZZERPIN_ON               PORTC |= 1<<5;
  #define BUZZERPIN_OFF              PORTC &= ~(1<<5);
  #define POWERPIN_PINMODE           pinMode (37, OUTPUT);
  #define POWERPIN_ON                PORTC |= 1<<0;
  #define POWERPIN_OFF               PORTC &= ~(1<<0);
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
  #define ISR_UART                   ISR(USART0_UDRE_vect)
  #define V_BATPIN                   A0    // Analog PIN 0
  #define PSENSORPIN                 A2    // Analog PIN 2
  
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

#if defined(MONGOOSE1_0)  // basically it's a PROMINI without some PINS => same code as a PROMINI board except PIN definition
  // http://www.fuzzydrone.org/ 
  // http://www.multiwii.com/forum/viewtopic.php?f=6&t=627
  
  #define LEDPIN_PINMODE             pinMode (4, OUTPUT);
  #define LEDPIN_TOGGLE              PIND |= 1<<4;     //switch LEDPIN state (digital PIN 13)
  #define LEDPIN_OFF                 PORTD &= ~(1<<4);  
  #define LEDPIN_ON                  PORTD |= (1<<4);     
  #define I2C_PULLUPS_ENABLE         PORTC |= 1<<4; PORTC |= 1<<5; 
  #define I2C_PULLUPS_DISABLE        PORTC &= ~(1<<4); PORTC &= ~(1<<5);  
  #define PPM_PIN_INTERRUPT          attachInterrupt(0, rxInt, RISING); //PIN 0
  #define SPEK_SERIAL_VECT           USART_RX_vect
  #define SPEK_BAUD_SET              UCSR0A  = (1<<U2X0); UBRR0H = ((F_CPU  / 4 / 115200 -1) / 2) >> 8; UBRR0L = ((F_CPU  / 4 / 115200 -1) / 2);
  #define SPEK_SERIAL_INTERRUPT      UCSR0B |= (1<<RXEN0)|(1<<RXCIE0);
  #define SPEK_DATA_REG              UDR0

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
  
  //RX PIN assignment inside the port //for PORTD
  #define THROTTLEPIN                2
  #define ROLLPIN                    4
  #define PITCHPIN                   5
  #define YAWPIN                     6
  #define AUX1PIN                    7
  #define AUX2PIN                    0 // optional PIN 8 or PIN 12
  #define AUX3PIN                    1 // unused 
  #define AUX4PIN                    3 // unused 
  #define ISR_UART                   ISR(USART_UDRE_vect)
  #define V_BATPIN                   A3    // Analog PIN 3
  #define PSENSORPIN                 A2    // Analog PIN 2

  #define SERVO_1_PINMODE            pinMode(A0,OUTPUT); // TILT_PITCH
  #define SERVO_1_PIN_HIGH           PORTC |= 1<<0;
  #define SERVO_1_PIN_LOW            PORTC &= ~(1<<0);
  #define SERVO_2_PINMODE            pinMode(A1,OUTPUT); // TILT_ROLL
  #define SERVO_2_PIN_HIGH           PORTC |= 1<<1;
  #define SERVO_2_PIN_LOW            PORTC &= ~(1<<1);
  #define SERVO_3_PINMODE            pinMode(A2,OUTPUT); // CAM TRIG
  #define SERVO_3_PIN_HIGH           PORTC |= 1<<2;
  #define SERVO_3_PIN_LOW            PORTC &= ~(1<<2); 
  #define SERVO_5_PINMODE            pinMode(3,OUTPUT); // BI LEFT
  #define SERVO_5_PIN_HIGH           PORTD|= 1<<3;
  #define SERVO_5_PIN_LOW            PORTD &= ~(1<<3);
  #define SERVO_6_PINMODE            pinMode(11,OUTPUT); // TRI REAR
  #define SERVO_6_PIN_HIGH           PORTB |= 1<<3;
  #define SERVO_6_PIN_LOW            PORTB &= ~(1<<3);
  #define SERVO_7_PINMODE            pinMode(10,OUTPUT); // new motor pin 10
  #define SERVO_7_PIN_HIGH           PORTB |= 1<<2;
  #define SERVO_7_PIN_LOW            PORTB &= ~(1<<2);
  #define SERVO_8_PINMODE            pinMode(9,OUTPUT); //new motor pin 9
  #define SERVO_8_PIN_HIGH           PORTB |= 1<<1;
  #define SERVO_8_PIN_LOW            PORTB &= ~(1<<1);
#endif

//please submit any correction to this list.
#if defined(FFIMUv1)
  #define ITG3200
  #define BMA180
  #define BMP085
  #define HMC5843
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  =  X; accADC[PITCH]  = Y; accADC[YAW]  = Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  X; gyroADC[PITCH] = Y; gyroADC[YAW] = Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  X; magADC[PITCH]  = Y; magADC[YAW]  = Z;}
  #define BMA180_ADDRESS 0x80
  #define ITG3200_ADDRESS 0XD0
#endif

#if defined(FFIMUv2)
  #define ITG3200
  #define BMA180
  #define BMP085
  #define HMC5883
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  =  X; accADC[PITCH]  = Y; accADC[YAW]  = Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  X; gyroADC[PITCH] = Y; gyroADC[YAW] = Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  X; magADC[PITCH]  = Y; magADC[YAW]  = Z;}
  #define BMA180_ADDRESS 0x80
  #define ITG3200_ADDRESS 0XD0
#endif

#if defined(FREEIMUv1)
  #define ITG3200
  #define ADXL345
  #define HMC5843
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  =  -Y; accADC[PITCH]  = X; accADC[YAW]  = Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  X;  gyroADC[PITCH] = Y; gyroADC[YAW] = Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  X;  magADC[PITCH]  = Y; magADC[YAW]  = Z;}
  #define ADXL345_ADDRESS 0xA6
  #undef INTERNAL_I2C_PULLUPS
#endif

#if defined(FREEIMUv03)
  #define ITG3200
  #define ADXL345 // this is actually an ADXL346 but that's just the same as ADXL345
  #define HMC5883
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  =  -Y; accADC[PITCH]  = X; accADC[YAW]  = Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  X;  gyroADC[PITCH] = Y; gyroADC[YAW] = Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  -Y;  magADC[PITCH]  = X; magADC[YAW]  = Z;} 
  #define ADXL345_ADDRESS 0xA6
  #undef INTERNAL_I2C_PULLUPS
#endif

#if defined(FREEIMUv035) || defined(FREEIMUv035_MS) || defined(FREEIMUv035_BMP)
  #define ITG3200
  #define BMA180
  #define HMC5883
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  =  X; accADC[PITCH]  = Y; accADC[YAW]  = Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  X;  gyroADC[PITCH] = Y; gyroADC[YAW] = Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  -Y;  magADC[PITCH]  = X; magADC[YAW]  = Z;} 
  #undef INTERNAL_I2C_PULLUPS
  #if defined(FREEIMUv035_MS)
    #define MS561101BA
  #elif defined(FREEIMUv035_BMP)
    #define BMP085
  #endif
#endif

#if defined(FREEIMUv04)
  #define MPU6050
  #define HMC5883
  #define MS561101BA
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  =  X; accADC[PITCH]  = Y; accADC[YAW]  = Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  X;  gyroADC[PITCH] = Y; gyroADC[YAW] = Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  -Y;  magADC[PITCH]  = X; magADC[YAW]  = Z;}
  #define MPU6050_EN_I2C_BYPASS // MAG connected to the AUX I2C bus of MPU6050
  #undef INTERNAL_I2C_PULLUPS
#endif

#if defined(PIPO)
  #define L3G4200D
  #define ADXL345
  #define HMC5883
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -Y; accADC[PITCH]  = X; accADC[YAW]  = Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] = X; gyroADC[PITCH] = Y; gyroADC[YAW] = Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  X; magADC[PITCH]  = Y; magADC[YAW]  = Z;}
  #define ADXL345_ADDRESS 0xA6
#endif

#if defined(QUADRINO)
  #define ITG3200
  #define BMA180
  #define BMP085
  #define HMC5883
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  =  X; accADC[PITCH]  = Y; accADC[YAW]  = Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  X; gyroADC[PITCH] = Y; gyroADC[YAW] = Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  = -Y; magADC[PITCH]  = X; magADC[YAW]  = Z;}
  #define BMA180_ADDRESS 0x80
  #define ITG3200_ADDRESS 0XD0
#endif

#if defined(QUADRINO_ZOOM)
  #define ITG3200
  #define BMA180
  #define BMP085  // note, can be also #define MS561101BA  on some versions
  //#define MS561101BA
  #define HMC5883
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  =  X; accADC[PITCH]  = Y; accADC[YAW]  = Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  X; gyroADC[PITCH] = Y; gyroADC[YAW] = Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  = -Y; magADC[PITCH]  = X; magADC[YAW]  = Z;}
  #define BMA180_ADDRESS 0x80
  #define ITG3200_ADDRESS 0XD0
  #define STABLEPIN_PINMODE pinMode (A2, OUTPUT);
  #define STABLEPIN_ON PORTC |= (1<<2);
  #define STABLEPIN_OFF PORTC &= ~(1<<2);
#endif

#if defined(ALLINONE)
  #define ITG3200
  #define BMA180
  #define BMP085
  #define HMC5883
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  =  X; accADC[PITCH]  = Y; accADC[YAW]  = Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  X; gyroADC[PITCH] = Y; gyroADC[YAW] = Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  = -Y; magADC[PITCH]  = X; magADC[YAW]  = Z;}
#endif

#if defined(AEROQUADSHIELDv2) // to confirm
  #define ITG3200
  #define BMA180
  #define BMP085
  #define HMC5843
  #define ACC_ORIENTATION(X, Y, Z) {accADC[ROLL] = -Y; accADC[PITCH] = X; accADC[YAW] = Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] = -Y; gyroADC[PITCH] = X; gyroADC[YAW] = Z;}
  #define MAG_ORIENTATION(X, Y, Z) {magADC[ROLL] = -Y; magADC[PITCH] = X; magADC[YAW] = Z;}
  #define ITG3200_ADDRESS 0XD2
#endif

#if defined(ATAVRSBIN1)
  #define ITG3200
  #define BMA020        //Actually it's a BMA150, but this is a drop in replacement for the discountinued BMA020
  #define AK8975
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  =  Y; accADC[PITCH]  = -X; accADC[YAW]  = Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] = -Y; gyroADC[PITCH] =  X; gyroADC[YAW] = Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  = -X; magADC[PITCH]  = -Y; magADC[YAW]  = -Z;}
#endif

#if defined(SIRIUS)
  #define ITG3200
  #define BMA180
  #define BMP085
  #define HMC5883
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  =  X; accADC[PITCH]  = Y; accADC[YAW]  = Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  X; gyroADC[PITCH] = Y; gyroADC[YAW] = Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  = -Y;  magADC[PITCH] = X; magADC[YAW]  = Z;}
  #define BMA180_ADDRESS 0x80
  #define ITG3200_ADDRESS 0XD0
#endif

#if defined(SIRIUS600)
  #define BMA180
  #define BMP085
  #define HMC5883
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  =  X; accADC[PITCH]  = Y; accADC[YAW]  = Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  = -Y;  magADC[PITCH] = X; magADC[YAW]  = Z;}
  #define BMA180_ADDRESS 0x80
#endif

#if defined(MINIWII)
  #define ITG3200
  #define BMA180
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  =  X; accADC[PITCH]  = -Y; accADC[YAW]  = -Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] = -X; gyroADC[PITCH] =  Y; gyroADC[YAW] =  Z;}
#endif

#if defined(CITRUSv1_0)
  #define ITG3200
  #define ADXL345
  #define BMP085
  #define HMC5883
  #define ACC_ORIENTATION(Y, X, Z)  {accADC[ROLL]  =  -X; accADC[PITCH]  = Y; accADC[YAW]  = Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  X; gyroADC[PITCH] = Y; gyroADC[YAW] = Z;}
  #define MAG_ORIENTATION(Y, X, Z)  {magADC[ROLL]  = Y;  magADC[PITCH] = X; magADC[YAW]  = Z;}
  #define ADXL345_ADDRESS 0xA6
  #define ITG3200_ADDRESS 0XD0
#endif

#if defined(DROTEK_IMU10DOF)
  #define ITG3200
  #define BMA180
  #define BMP085
  #define HMC5883
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  =  X; accADC[PITCH]  = Y; accADC[YAW]  = Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  X; gyroADC[PITCH] = Y; gyroADC[YAW] = Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  = -Y; magADC[PITCH]  = X; magADC[YAW]  = Z;}
  #define ITG3200_ADDRESS 0XD2
#endif

#if defined(MONGOOSE1_0)
  #define ITG3200
  #define ADXL345
  #define BMP085
  #define HMC5883
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  -X; gyroADC[PITCH] = -Y; gyroADC[YAW] = Z;}
  #define ACC_ORIENTATION(Y, X, Z)  {accADC[ROLL]  =  X; accADC[PITCH]  = -Y; accADC[YAW]  = Z;}
  #define MAG_ORIENTATION(Y, X, Z)  {magADC[ROLL]  = X;  magADC[PITCH] = -Y; magADC[YAW]  = Z;}
  #define ADXL345_ADDRESS 0xA6
  #define ITG3200_ADDRESS 0XD0
  #define BMP085_ADDRESS 0xEE
  #undef INTERNAL_I2C_PULLUPS
#endif

#if defined(CRIUS_LITE_0_2)
  #define ITG3200
  #define ADXL345
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] = X; gyroADC[PITCH] =  Y; gyroADC[YAW] = Z;}
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  =  -Y; accADC[PITCH]  = X; accADC[YAW]  = Z;}
#endif

#if defined(ADXL345) || defined(BMA020) || defined(BMA180) || defined(NUNCHACK) || defined(MMA7455) || defined(ADCACC) || defined(LSM303DLx_ACC) || defined(MPU6050)
  #define ACC 1
#else
  #define ACC 0
#endif

#if defined(HMC5883) || defined(HMC5843) || defined(AK8975)
  #define MAG 1
#else
  #define MAG 0
#endif

#if defined(ITG3200) || defined(L3G4200D) || defined(MPU6050)
  #define GYRO 1
#else
  #define GYRO 0
#endif

#if defined(BMP085) || defined(MS561101BA)
  #define BARO 1
#else
  #define BARO 0
#endif

#if defined(GPS_SERIAL)  || defined(I2C_GPS)
  #define GPS 1
#else
  #define GPS 0
#endif








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
  #define MULTITYPE 15      // Simple model 
#elif defined (HELI_90_DEG)   
  #define MULTITYPE 16      // Simple model  
#elif defined(VTAIL4)
 #define MULTITYPE 17
#endif

#if defined(POWERMETER_HARD) || defined(POWERMETER_SOFT)
  #define POWERMETER
#endif

/* motor and servo numbers */

#if defined(BI) || defined(TRI) || defined(SERVO_TILT) || defined(GIMBAL) || defined(FLYING_WING) || defined(CAMTRIG)
  #define SERVO
#endif

#if defined(GIMBAL)
  #define NUMBER_MOTOR 0
  #define PRI_SERVO_FROM   1 // use servo from 1 to 2
  #define PRI_SERVO_TO     2
#elif defined(FLYING_WING)
  #define NUMBER_MOTOR 1
  #define PRI_SERVO_FROM   1 // use servo from 1 to 2
  #define PRI_SERVO_TO     2





#elif defined(BI)
  #define NUMBER_MOTOR 2
  #define PRI_SERVO_FROM   5 // use servo from 5 to 6
  #define PRI_SERVO_TO     6
#elif defined(TRI)
  #define NUMBER_MOTOR 3
  #define PRI_SERVO_FROM   6 // use only servo 6
  #define PRI_SERVO_TO     6
#elif defined(QUADP) || defined(QUADX) || defined(Y4)|| defined(VTAIL4)
  #define NUMBER_MOTOR 4
#elif defined(Y6) || defined(HEX6) || defined(HEX6X)
  #define NUMBER_MOTOR 6
#elif defined(OCTOX8) || defined(OCTOFLATP) || defined(OCTOFLATX)
  #define NUMBER_MOTOR 8
#endif


#if defined(SERVO_TILT) && defined(CAMTRIG)
  #define SEC_SERVO_FROM   1 // use servo from 1 to 3
  #define SEC_SERVO_TO     3
#else
  #if defined(SERVO_TILT)
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


#if defined(I2C_GPS)
  #define I2C_GPS_ADDRESS                         0x40       
  /*************** I2C GSP register definitions *********************************/
  
  #define I2C_GPS_STATUS                          0x00   //(Read only)
          #define I2C_GPS_STATUS_NEW_DATA         0x01
          #define I2C_GPS_STATUS_2DFIX            0x02
          #define I2C_GPS_STATUS_3DFIX            0x04
          #define I2C_GPS_STATUS_WP_REACHED       0x08      //Active waypoint has been reached (not cleared until new waypoint is set)
          #define I2C_GPS_STATUS_NUMSATS          0xF0
  
  #define I2C_GPS_COMMAND                         0x01   //(write only)
          #define I2C_GPS_COMMAND_POSHOLD         0x01      //copy current position to internal target location register
          #define I2C_GPS_COMMAND_RESUME          0x02      //copy last active WP to internal target location register
          #define I2C_GPS_COMMAND_SET_WP          0x04      //copy current position to given WP
          #define I2C_GPS_COMMAND_ACTIVATE_WP     0x08      //copy given WP position to internal target location register
          #define I2C_GPS_COMMAND_WP              0xF0      //Waypoint number
  
  #define I2C_GPS_WP_REG                          0x06   //Waypoint register (Read only)
          #define I2C_GPS_WP_REG_ACTIVE           0x0F      //Active Waypoint
          #define I2C_GPS_WP_REG_PERVIOUS         0xF0      //pervious Waypoint
          
  #define I2C_GPS_GROUND_SPEED                    0x07   //GPS ground speed in m/s*100 (uint16_t)      (Read Only)
  #define I2C_GPS_ALTITUDE                        0x09   //GPS altitude in meters (uint16_t)           (Read Only)

  #define I2C_GPS_TIME                            0x0b   //UTC Time from GPS in hhmmss.sss * 100 (uint32_t)(unneccesary precision) (Read Only)
  #define I2C_GPS_DISTANCE                        0x0f   //Distance between current pos and internal target location register in meters (uint16_t) (Read Only)
  #define I2C_GPS_DIRECTION                       0x11   //direction towards interal target location reg from current position (+/- 180 degree)    (read Only)
  #define I2C_GPS_LOCATION                        0x13   //current position (8 bytes, lat and lon, 1 degree = 10 000 000                           (read only)
  #define I2C_GPS_WP0                             0x1B   //Waypoint 0 used for RTH location      (R/W)
  #define I2C_GPS_WP1                             0x23
  #define I2C_GPS_WP2                             0x2B
  #define I2C_GPS_WP3                             0x33
  #define I2C_GPS_WP4                             0x3B
  #define I2C_GPS_WP5                             0x43
  #define I2C_GPS_WP6                             0x4B
  #define I2C_GPS_WP7                             0x53
  #define I2C_GPS_WP8                             0x5B
  #define I2C_GPS_WP9                             0x63
  #define I2C_GPS_WP10                            0x6B
  #define I2C_GPS_WP11                            0x73
  #define I2C_GPS_WP12                            0x7B
  #define I2C_GPS_WP13                            0x83
  #define I2C_GPS_WP14                            0x8B
  #define I2C_GPS_WP15                            0x93
  #define I2C_GPS_WP_NAV_PAR1                     0x9B   //Waypoint navigation parameter 1
          #define I2C_GPS_WP_NAV_PAR1_REACH_LIMIT 0x0F      //lover 4 bit, waypoint reached distance
#endif

/**************************/
/* Error Checking Section */
/**************************/

#if (defined(LCD_CONF) || defined(LCD_TELEMETRY)) && !(defined(LCD_SERIAL3W) || defined(LCD_TEXTSTAR) || defined(LCD_VT100) || defined(LCD_ETPP) || defined(LCD_LCD03))
  #error "LCD_CONF or LCD_TELEMETRY defined, and choice of LCD not defined.  Uncomment one of LCD_SERIAL3W or LCD_TEXTSTAR or LCD_VT100 or LCD_ETPP or LCD_LCD03"
#endif


#if defined(POWERMETER) && !(defined(VBAT))
  	#error "to use powermeter, you must also define and configure VBAT"
#endif

#if defined(LCD_TELEMETRY_AUTO) && !(defined(LCD_TELEMETRY))
 	#error "to use automatic telemetry, you MUST also define and configure LCD_TELEMETRY"
#endif
