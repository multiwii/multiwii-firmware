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

#if defined(FREEIMUv01)
  #define ITG3200
  #define ADXL345
  #define HMC5843
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  =  -Y; accADC[PITCH]  = X; accADC[YAW]  = Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  X; gyroADC[PITCH] = Y; gyroADC[YAW] = Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  X; magADC[PITCH]  = Y; magADC[YAW]  = Z;}
  #define ADXL345_ADDRESS 0xA6
  #undef INTERNAL_I2C_PULLUPS
#endif

#if defined(FREEIMU)
  #define ITG3200
  #define ADXL345
  #define HMC5883
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  =  X; accADC[PITCH]  = Y; accADC[YAW]  = Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  X; gyroADC[PITCH] = Y; gyroADC[YAW] = Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  X; magADC[PITCH]  = Y; magADC[YAW]  = Z;}
  #define ADXL345_ADDRESS 0xA6
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

#if defined(ALLINONE)
  #define ITG3200
  #define BMA180
  #define BMP085
  #define HMC5883
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  =  X; accADC[PITCH]  = Y; accADC[YAW]  = Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  X; gyroADC[PITCH] = Y; gyroADC[YAW] = Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  = -Y; magADC[PITCH]  = X; magADC[YAW]  = Z;}
#endif


#if defined(ADXL345) || defined(BMA020) || defined(BMA180) || defined(NUNCHACK) || defined(ADCACC)
  #define ACC 1
#else
  #define ACC 0
#endif

#if defined(HMC5883) || defined(HMC5843)
  #define MAG 1
#else
  #define MAG 0
#endif

#if defined(ITG3200) || defined(L3G4200D)
  #define GYRO 1
#else
  #define GYRO 0
#endif

#if defined(BMP085)
  #define BARO 1
#else
  #define BARO 0
#endif

#if defined(PROMINI)
  #define LEDPIN_PINMODE             pinMode (13, OUTPUT);
  #define LEDPIN_SWITCH              PINB |= 1<<5;     //switch LEDPIN state (digital PIN 13)
  #define LEDPIN_OFF                 PORTB &= ~(1<<5);
  #define LEDPIN_ON                  PORTB |= (1<<5);
  #define BUZZERPIN_PINMODE          pinMode (8, OUTPUT);
  #define BUZZERPIN_ON               PORTB |= 1;
  #define BUZZERPIN_OFF              PORTB &= ~1;
  #define POWERPIN_PINMODE           pinMode (12, OUTPUT);
  #define POWERPIN_ON                PORTB |= 1<<4;
  #define POWERPIN_OFF               PORTB &= ~(1<<4); //switch OFF WMP, digital PIN 12
  #define I2C_PULLUPS_ENABLE         PORTC |= 1<<4; PORTC |= 1<<5;   // PIN A4&A5 (SDA&SCL)
  #define I2C_PULLUPS_DISABLE        PORTC &= ~(1<<4); PORTC &= ~(1<<5);
  #define PINMODE_LCD                pinMode(0, OUTPUT);
  #define LCDPIN_OFF                 PORTD &= ~1;
  #define LCDPIN_ON                  PORTD |= 1;
  #define STABLEPIN_PINMODE          ;
  #define STABLEPIN_ON               ;
  #define STABLEPIN_OFF              ;
  #define DIGITAL_SERVO_TRI_PINMODE  pinMode(3,OUTPUT); //also right servo for BI COPTER
  #define DIGITAL_SERVO_TRI_HIGH     PORTD |= 1<<3;
  #define DIGITAL_SERVO_TRI_LOW      PORTD &= ~(1<<3);
  #define DIGITAL_TILT_PITCH_PINMODE pinMode(A0,OUTPUT);
  #define DIGITAL_TILT_PITCH_HIGH    PORTC |= 1<<0;
  #define DIGITAL_TILT_PITCH_LOW     PORTC &= ~(1<<0);
  #define DIGITAL_TILT_ROLL_PINMODE  pinMode(A1,OUTPUT);
  #define DIGITAL_TILT_ROLL_HIGH     PORTC |= 1<<1;
  #define DIGITAL_TILT_ROLL_LOW      PORTC &= ~(1<<1);
  #define DIGITAL_BI_LEFT_PINMODE    pinMode(11,OUTPUT); 
  #define DIGITAL_BI_LEFT_HIGH       PORTB |= 1<<3;
  #define DIGITAL_BI_LEFT_LOW        PORTB &= ~(1<<3);
  #define PPM_PIN_INTERRUPT          attachInterrupt(0, rxInt, RISING); //PIN 0
  #define MOTOR_ORDER                9,10,11,3,6,5  //for a quad+: rear,right,left,front
  #define DIGITAL_CAM_PINMODE        pinMode(A2,OUTPUT);
  #define DIGITAL_CAM_HIGH           PORTC |= 1<<2;
  #define DIGITAL_CAM_LOW            PORTC &= ~(1<<2);
  //RX PIN assignment inside the port //for PORTD
  #define THROTTLEPIN                2
  #define ROLLPIN                    4
  #define PITCHPIN                   5
  #define YAWPIN                     6
  #define AUX1PIN                    7
  #define AUX2PIN                    7   //unused just for compatibility with MEGA
  #define CAM1PIN                    7   //unused just for compatibility with MEGA
  #define CAM2PIN                    7   //unused just for compatibility with MEGA
  #define ISR_UART                   ISR(USART_UDRE_vect)
  #define V_BATPIN                   A3    // Analog PIN 3
  #define PSENSORPIN                 A2    // Analog PIN 2
#endif
#if defined(MEGA)
  #define LEDPIN_PINMODE             pinMode (13, OUTPUT);
  #define LEDPIN_SWITCH              PINB |= (1<<7);
  #define LEDPIN_ON                  PORTB |= (1<<7);
  #define LEDPIN_OFF                 PORTB &= ~(1<<7);
  #define BUZZERPIN_PINMODE          pinMode (31, OUTPUT);
  #define BUZZERPIN_ON               PORTC |= 1<<6;
  #define BUZZERPIN_OFF              PORTC &= ~1<<6;
  #define POWERPIN_PINMODE           pinMode (37, OUTPUT);
  #define POWERPIN_ON                PORTC |= 1<<0;
  #define POWERPIN_OFF               PORTC &= ~(1<<0);
  #define I2C_PULLUPS_ENABLE         PORTD |= 1<<0; PORTD |= 1<<1;       // PIN 20&21 (SDA&SCL)
  #define I2C_PULLUPS_DISABLE        PORTD &= ~(1<<0); PORTD &= ~(1<<1);
  #define PINMODE_LCD                pinMode(0, OUTPUT);
  #define LCDPIN_OFF                 PORTE &= ~1;      //switch OFF digital PIN 0
  #define LCDPIN_ON                  PORTE |= 1;       //switch OFF digital PIN 0
  #define STABLEPIN_PINMODE          pinMode (31, OUTPUT);
  #define STABLEPIN_ON               PORTC |= 1<<6;
  #define STABLEPIN_OFF              PORTC &= ~1<<6;
  #define DIGITAL_SERVO_TRI_PINMODE  pinMode(2,OUTPUT); //PIN 2 //also right servo for BI COPTER
  #define DIGITAL_SERVO_TRI_HIGH     PORTE |= 1<<4;
  #define DIGITAL_SERVO_TRI_LOW      PORTE &= ~(1<<4);
  #define DIGITAL_TILT_PITCH_PINMODE pinMode(33,OUTPUT); // 33
  #define DIGITAL_TILT_PITCH_HIGH    PORTC |= 1<<4;
  #define DIGITAL_TILT_PITCH_LOW     PORTC &= ~(1<<4);
  #define DIGITAL_TILT_ROLL_PINMODE  pinMode(34,OUTPUT);pinMode(44,OUTPUT); // 34 + 44
  #define DIGITAL_TILT_ROLL_HIGH     PORTC |= 1<<3;PORTL |= 1<<5;
  #define DIGITAL_TILT_ROLL_LOW      PORTC &= ~(1<<3);PORTL |= 1<<5;
  #define DIGITAL_BI_LEFT_PINMODE    pinMode(6,OUTPUT); 
  #define DIGITAL_BI_LEFT_HIGH       PORTH |= 1<<3;
  #define DIGITAL_BI_LEFT_LOW        PORTH &= ~(1<<3);
  #define PPM_PIN_INTERRUPT          attachInterrupt(4, rxInt, RISING);  //PIN 19, also used for Spektrum satellite option
  #define MOTOR_ORDER                3,5,6,2,7,8,9,10   //for a quad+: rear,right,left,front   //+ for y6: 7:under right  8:under left
  #define DIGITAL_CAM_PINMODE        pinMode(35,OUTPUT);pinMode(45,OUTPUT); // 35 + 45
  #define DIGITAL_CAM_HIGH           PORTC |= 1<<2;PORTL |= 1<<4;
  #define DIGITAL_CAM_LOW            PORTC &= ~(1<<2);PORTL |= 1<<4;
  //RX PIN assignment inside the port //for PORTK
  #define THROTTLEPIN                0  //PIN 62 =  PIN A8
  #define ROLLPIN                    1  //PIN 63 =  PIN A9
  #define PITCHPIN                   2  //PIN 64 =  PIN A10
  #define YAWPIN                     3  //PIN 65 =  PIN A11
  #define AUX1PIN                    4  //PIN 66 =  PIN A12
  #define AUX2PIN                    5  //PIN 67 =  PIN A13
  #define CAM1PIN                    6  //PIN 68 =  PIN A14
  #define CAM2PIN                    7  //PIN 69 =  PIN A15
  #define ISR_UART                   ISR(USART0_UDRE_vect)
  #define V_BATPIN                   A3    // Analog PIN 3
  #define PSENSORPIN                 A2    // Analog PIN 2
#endif

