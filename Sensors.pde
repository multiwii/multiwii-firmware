// ***************************
// board orientation and setup
// ***************************
//default board orientation
#if !defined(ACC_ORIENTATION) 
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = X; accADC[PITCH]  = Y; accADC[YAW]  = Z;}
#endif
#if !defined(GYRO_ORIENTATION) 
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] = X; gyroADC[PITCH] = Y; gyroADC[YAW] = Z;}
#endif
#if !defined(MAG_ORIENTATION) 
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  = X; magADC[PITCH]  = Y; magADC[YAW]  = Z;}
#endif

/*** I2C address ***/
#if !defined(ADXL345_ADDRESS) 
  #define ADXL345_ADDRESS 0x3A
  //#define ADXL345_ADDRESS 0xA6   //WARNING: Conflicts with a Wii Motion plus!
#endif

#if !defined(BMA180_ADDRESS) 
  #define BMA180_ADDRESS 0x80
  //#define BMA180_ADDRESS 0x82
#endif

#if !defined(ITG3200_ADDRESS) 
  #define ITG3200_ADDRESS 0XD0
  //#define ITG3200_ADDRESS 0XD2
#endif

uint8_t rawADC[6];
static uint32_t neutralizeTime = 0;
  
// *********************
// I2C general functions
// *********************

// Mask prescaler bits : only 5 bits of TWSR defines the status of each I2C request
#define TW_STATUS_MASK	(1<<TWS7) | (1<<TWS6) | (1<<TWS5) | (1<<TWS4) | (1<<TWS3)
#define TW_STATUS       (TWSR & TW_STATUS_MASK)

void i2c_init(void) {
  #if defined(INTERNAL_I2C_PULLUPS)
    I2C_PULLUPS_ENABLE
  #else
    I2C_PULLUPS_DISABLE
  #endif
  TWSR = 0;        // no prescaler => prescaler = 1
  TWBR = ((16000000L / I2C_SPEED) - 16) / 2; // change the I2C clock rate
  TWCR = 1<<TWEN;  // enable twi module, no interrupt
}

void i2c_rep_start(uint8_t address) {
  TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN) | (1<<TWSTO); // send REPEAT START condition
  waitTransmissionI2C(); // wait until transmission completed
  checkStatusI2C(); // check value of TWI Status Register
  TWDR = address; // send device address
  TWCR = (1<<TWINT) | (1<<TWEN);
  waitTransmissionI2C(); // wail until transmission completed
  checkStatusI2C(); // check value of TWI Status Register
}

void i2c_rep_stop(void) {
  TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
  waitTransmissionI2C();
  checkStatusI2C();
}

void i2c_write(uint8_t data ) {	
  TWDR = data; // send data to the previously addressed device
  TWCR = (1<<TWINT) | (1<<TWEN);
  waitTransmissionI2C(); // wait until transmission completed
  checkStatusI2C(); // check value of TWI Status Register
}

uint8_t i2c_readAck() {
  TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
  waitTransmissionI2C();
  return TWDR;
}

uint8_t i2c_readNak(void) {
  TWCR = (1<<TWINT) | (1<<TWEN);
  waitTransmissionI2C();
  return TWDR;
}

void waitTransmissionI2C() {
  uint8_t count = 255;
  while (count-->0 && !(TWCR & (1<<TWINT)) );
  if (count<2) { //we are in a blocking state => we don't insist
    TWCR = 0;  //and we force a reset on TWINT register
    neutralizeTime = micros(); //we take a timestamp here to neutralize the value during a short delay after the hard reset
  }
}

void checkStatusI2C() {
  if ( TW_STATUS  == 0xF8) { //TW_NO_INFO : this I2C error status indicates a wrong I2C communication.
    // WMP does not respond anymore => we do a hard reset. I did not find another way to solve it. It takes only 13ms to reset and init to WMP or WMP+NK
    TWCR = 0;
    if (!GYRO) {
      POWERPIN_OFF //switch OFF WMP
      delay(1);  
      POWERPIN_ON  //switch ON WMP
      delay(10);
      WMP_init(0);
    }
    neutralizeTime = micros(); //we take a timestamp here to neutralize the WMP or WMP+NK values during a short delay after the hard reset
  }
}

void i2c_getSixRawADC(uint8_t add, uint8_t reg) {
  i2c_rep_start(add);
  i2c_write(reg);         // Start multiple read at the reg register
  i2c_rep_start(add +1);  // I2C read direction => I2C address + 1
  for(uint8_t i = 0; i < 5; i++) {
    rawADC[i]=i2c_readAck();}
  rawADC[5]= i2c_readNak();
}

void i2c_writeReg(uint8_t add, uint8_t reg, uint8_t val) {
  i2c_rep_start(add+0);  // I2C write direction
  i2c_write(reg);        //   register selection
  i2c_write(val);        //     value to write in register
}

// ****************
// GYRO common part
// ****************
void GYRO_Common() {
  static int16_t previousGyroADC[3] = {0,0,0};
  static int32_t g[3];
  uint8_t axis;
  
  if (calibratingG>0) {
    for (axis = 0; axis < 3; axis++) {
      if (calibratingG>1) {
        if (calibratingG == 400) g[axis]=0;
        g[axis] +=gyroADC[axis];
        gyroADC[axis]=0;
        gyroZero[axis]=0;
      } else {
        gyroZero[axis]=(g[axis]+200)/399;
        blinkLED(10,15,1+3*nunchuk);
      }
    }
    calibratingG--;
  }
  //anti gyro glitch, limit the variation between two consecutive readings
  for (axis = 0; axis < 3; axis++) {
    gyroADC[axis]  -= gyroZero[axis];
    gyroADC[axis] = constrain(gyroADC[axis],previousGyroADC[axis]-100,previousGyroADC[axis]+100);
    previousGyroADC[axis] = gyroADC[axis];
  }
}

// ****************
// ACC common part
// ****************
void ACC_Common() {
  static int32_t a[3];
  
  if (calibratingA>0) {
    if (calibratingA>1) {
      for (uint8_t axis = 0; axis < 3; axis++) {
        if (calibratingA == 400) a[axis]=0;
        a[axis] +=accADC[axis];
        accADC[axis]=0;
        accZero[axis]=0;
      }
    } else {
      accZero[ROLL]  = (a[ROLL]+200)/399;
      accZero[PITCH] = (a[PITCH]+200)/399;
      accZero[YAW]   = (a[YAW]+200)/399-acc_1G; // for nunchuk 200=1G
      writeParams(); // write accZero in EEPROM
    }
    calibratingA--;
  }
  accADC[ROLL]  -=  accZero[ROLL] ;
  accADC[PITCH] -=  accZero[PITCH];
  accADC[YAW]   -=  accZero[YAW] ;
}


// **************************
// I2C Barometer BOSCH BMP085
// **************************
// I2C adress: 0xEE (8bit)   0x77 (7bit)
// principle:
//  1) read the calibration register (only once at the initialization)
//  2) read uncompensated temperature (not mandatory at every cycle)
//  3) read uncompensated pressure
//  4) raw temp + raw pressure => calculation of the adjusted pressure
//  the following code uses the maximum precision setting (oversampling setting 3)

#if defined(BMP085)
/**************************************************************
WinFilter version 0.8
http://www.winfilter.20m.com
akundert@hotmail.com

Filter type: Low Pass
Filter model: Butterworth
Filter order: 2
Sampling Frequency: 333 Hz
Cut Frequency: 12.000000 Hz
Coefficents Quantization: float

Z domain Zeros
z = -1.000000 + j 0.000000
z = -1.000000 + j 0.000000

Z domain Poles
z = 0.840979 + j -0.136993
z = 0.840979 + j 0.136993
***************************************************************/
#define NCoef 2
float baro_iir(float NewSample) {
    float ACoef[NCoef+1] = {
        0.01097303489321129600,
        0.02194606978642259200,
        0.01097303489321129600
    };

    float BCoef[NCoef+1] = {
        1.00000000000000000000,
        -1.68195898127125940000,
        0.72601363563434096000
    };

    static float y[NCoef+1]; //output samples
    static float x[NCoef+1]; //input samples
    int n;

    //shift the old samples
    for(n=NCoef; n>0; n--) {
       x[n] = x[n-1];
       y[n] = y[n-1];
    }

    //Calculate the new output
    x[0] = NewSample;
    y[0] = ACoef[0] * x[0];
    for(n=1; n<=NCoef; n++)
        y[0] += ACoef[n] * x[n] - BCoef[n] * y[n];
    
    return y[0];
}

// sensor registers from the BOSCH BMP085 datasheet
static int16_t ac1,ac2,ac3,b1,b2,mb,mc,md;
static uint16_t  ac4,ac5,ac6;

static uint16_t ut; //uncompensated T
static uint32_t up; //uncompensated P
static int32_t pressure = 0;
static int16_t altitude = 0;
static int16_t altitudeZero;
static int16_t altitudeHold;
#define OSS 3

void i2c_BMP085_readCalibration(){
  delay(10);
  ac1 = i2c_BMP085_readIntRegister(0xAA);
  ac2 = i2c_BMP085_readIntRegister(0xAC);
  ac3 = i2c_BMP085_readIntRegister(0xAE);
  ac4 = i2c_BMP085_readIntRegister(0xB0);
  ac5 = i2c_BMP085_readIntRegister(0xB2);
  ac6 = i2c_BMP085_readIntRegister(0xB4);
  b1  = i2c_BMP085_readIntRegister(0xB6);
  b2  = i2c_BMP085_readIntRegister(0xB8);
  mb  = i2c_BMP085_readIntRegister(0xBA);
  mc  = i2c_BMP085_readIntRegister(0xBC);
  md  = i2c_BMP085_readIntRegister(0xBE);
}


void  Baro_init() {
  delay(10);
  i2c_BMP085_readCalibration();
  i2c_BMP085_UT_Start(); 
  delay(5);
  i2c_BMP085_UT_Read();
  altitudeZero = 0;
}


// read a 16 bit register
int16_t i2c_BMP085_readIntRegister(unsigned char r) {
  uint8_t msb, lsb;
  
  i2c_rep_start(0xEE + 0);
  i2c_write(r);
  i2c_rep_start(0xEE + 1);//I2C read direction => 1
  msb=i2c_readAck();
  lsb=i2c_readNak();
  i2c_rep_stop();
  return (((int16_t)msb<<8) | ((int16_t)lsb));
}

// read uncompensated temperature value: send command first
void i2c_BMP085_UT_Start() {
  i2c_writeReg(0xEE,0xf4,0x2e);
  i2c_rep_start(0xEE + 0);
  i2c_write(0xF6);
  i2c_rep_stop();
}

// read uncompensated pressure value: send command first
void i2c_BMP085_UP_Start () {
  i2c_writeReg(0xEE,0xf4,0x34+(OSS<<6)); // control register value for oversampling setting 3
  i2c_rep_start(0xEE + 0); //I2C write direction => 0
  i2c_write(0xf6);
  i2c_rep_stop();
}

// read uncompensated pressure value: read result bytes
// the datasheet suggests a delay of 25.5 ms (oversampling settings 3) after the send command
void i2c_BMP085_UP_Read () {
  uint8_t msb, lsb, xlsb;
  i2c_rep_start(0xEE + 1);//I2C read direction => 1
  msb = i2c_readAck();
  lsb = i2c_readAck();
  xlsb = i2c_readNak();
  i2c_rep_stop();
  up = (((uint32_t)msb<<16) | ((uint32_t)lsb<<8) | ((uint32_t)xlsb)) >> (8-OSS);
}

// read uncompensated temperature value: read result bytes
// the datasheet suggests a delay of 4.5 ms after the send command
void i2c_BMP085_UT_Read() {
  uint8_t msb, lsb;
  i2c_rep_start(0xEE + 1);//I2C read direction => 1
  msb=i2c_readAck();
  lsb=i2c_readNak();
  i2c_rep_stop();
  ut = (((uint16_t)msb<<8) | ((uint16_t)lsb));
}

void i2c_BMP085_Calculate() {
  long x1, x2, x3, b3, b5, b6, p;
  unsigned long b4, b7;
  int32_t tmp;
  
  // See Datasheet page 13 for this formulas
  // Based also on Jee Labs BMP085 example code. Thanks for share.
  // Temperature calculations
  x1 = ((long)ut - ac6) * ac5 >> 15;
  x2 = ((long) mc << 11) / (x1 + md);
  b5 = x1 + x2;
  //Temp = (b5 + 8) >> 4;

  // Pressure calculations
  b6 = b5 - 4000;
  x1 = (b2 * (b6 * b6 >> 12)) >> 11; 
  x2 = ac2 * b6 >> 11;
  x3 = x1 + x2;
  //b3 = (((int32_t) ac1 * 4 + x3)<<oss + 2) >> 2; // BAD
  //b3 = ((int32_t) ac1 * 4 + x3 + 2) >> 2;  //OK for oss=0
  tmp = ac1;
  tmp = (tmp*4 + x3) << OSS;
  b3 = (tmp+2)/4;
  x1 = ac3 * b6 >> 13;
  x2 = (b1 * (b6 * b6 >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  b4 = (ac4 * (uint32_t) (x3 + 32768)) >> 15;
  b7 = ((uint32_t) up - b3) * (50000 >> OSS);
  p = b7 < 0x80000000 ? (b7 * 2) / b4 : (b7 / b4) * 2;
  
  x1 = (p >> 8) * (p >> 8);
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * p) >> 16;
  pressure = p + ((x1 + x2 + 3791) >> 4);
}

void Baro_update() {
  static uint32_t t;
  static uint8_t state = 0;
  static uint8_t cycle_cnt = 0;
  static uint8_t zeroCount = 0;
  
  if (micros() < t) return; 
  t = micros();
  TWBR = ((16000000L / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz, BMP085 is ok with this speed
  switch (state) {
    case 0: 
      i2c_BMP085_UT_Start(); 
      state++; t += 4600; 
      break;
    case 1: 
      i2c_BMP085_UT_Read(); 
      state++; 
      break;
    case 2: 
      i2c_BMP085_UP_Start(); 
      state++; t += 30000; 
      break;
    case 3: 
      i2c_BMP085_UP_Read(); 
      i2c_BMP085_Calculate(); 
      state = 2; 
      cycle_cnt++;
      if (cycle_cnt == 33) {state = 0; cycle_cnt = 0;}
      //aitude = baro_iir((1.0 - pow(float(up)/101325.0, 0.190295)) * 4433000.0); // altitude in decimeter from starting point
      altitude = baro_iir(101325.0-pressure) - altitudeZero ;
      altitudeSmooth = altitude;
      if (altitudeZero == 0) {zeroCount++; if (zeroCount > 100) altitudeZero = altitudeSmooth; altitudeSmooth=0;}
      break;
  }
}
#endif


// **************************
// I2C Accelerometer ADXL345 
// **************************
// I2C adress: 0x3A (8bit)    0x1D (7bit)
// principle:
//  1) CS PIN must be linked to VCC to select the I2C mode
//  2) SD0 PIN must be linked to VCC to select the right I2C adress
//  3) bit  b00000100 must be set on register 0x2D to read data (only once at the initialization)
//  4) bits b00001011 must be set on register 0x31 to select the data format (only once at the initialization)
#if defined(ADXL345)
void ACC_init () {
  delay(10);
  i2c_writeReg(ADXL345_ADDRESS,0x2D,1<<3); //  register: Power CTRL  -- value: Set measure bit 3 on
  i2c_writeReg(ADXL345_ADDRESS,0x31,0x0B); //  register: DATA_FORMAT -- value: Set bits 3(full range) and 1 0 on (+/- 16g-range)
  i2c_writeReg(ADXL345_ADDRESS,0x2C,8+2+1); // register: BW_RATE     -- value: 200Hz sampling (see table 5 of the spec)

  acc_1G = 250;
}

void ACC_getADC () {
  TWBR = ((16000000L / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz, ADXL435 is ok with this speed
  i2c_getSixRawADC(ADXL345_ADDRESS,0x32);

  ACC_ORIENTATION( - ((rawADC[3]<<8) | rawADC[2]) ,
                     ((rawADC[1]<<8) | rawADC[0]) ,
                     ((rawADC[5]<<8) | rawADC[4]) );
  ACC_Common();
}
#endif


// **************************
// contribution initially from opie11 (rc-grooups)
// adaptation from C2po (may 2011)
// I2C Accelerometer BMA180
// **************************
// I2C adress: 0x80 (8bit)    0x40 (7bit) (SDO connection to VCC) 
// I2C adress: 0x82 (8bit)    0x41 (7bit) (SDO connection to VDDIO)
#if defined(BMA180)
void ACC_init () {
  uint8_t reg;
  delay(10);
  //default range 2G: 1G = 4096 unit.
  i2c_writeReg(BMA180_ADDRESS,0x0D,1<<4); // register: ctrl_reg0  -- value: set bit ee_w to 1 to enable writing
  i2c_rep_start(BMA180_ADDRESS+0);
  i2c_write(0x20); // register bw_tcs
  i2c_rep_start(BMA180_ADDRESS+1);
  reg = i2c_readNak(); // read bw_tcs (bw bits 7:4)
  delay(5);
  i2c_writeReg(BMA180_ADDRESS,0x20,reg & 0x0F); // register: bw_tcs reg: bits 4-7 to set bw -- value: set low pass filter to 10Hz (bits value = 0000xxxx)
  acc_1G = 205;
}

void ACC_getADC () {
  TWBR = ((16000000L / 400000L) - 16) / 2;  // Optional line.  Sensor is good for it in the spec.
  i2c_getSixRawADC(BMA180_ADDRESS,0x02);
  
  //usefull info is on the 14 bits  [2-15] bits  /4 => [0-13] bits  /20 => 4096/20 = 205 for 1G
  ACC_ORIENTATION(  - ((rawADC[1]<<8) | rawADC[0])/80 ,
                    - ((rawADC[3]<<8) | rawADC[2])/80 ,
                      ((rawADC[5]<<8) | rawADC[4])/80 );
  ACC_Common();
}
#endif

// **************
// contribution from Point65 and mgros (rc-grooups)
// I2C Accelerometer BMA020
// **************
// I2C adress: 0x70 (8bit)
#if defined(BMA020)
void ACC_init(){
  byte control;
  i2c_writeReg(0x70,0x15,0x80); // Write B10000000 at 0x15 init BMA020
  i2c_writeReg(0x70,0x14,0x71); 

  i2c_rep_start(0x71);
  control = i2c_readNak();
 
  control = control >> 5;  //ensure the value of three fist bits of reg 0x14 see BMA020 documentation page 9
  control = control << 2;
  control = control | 0x00; //Range 2G 00
  control = control << 3;
  control = control | 0x00; //Bandwidth 25 Hz 000

  i2c_writeReg(0x70,0x14,control); 

  acc_1G = 240;
}

void ACC_getADC(){
  TWBR = ((16000000L / 400000L) - 16) / 2;
  i2c_getSixRawADC(0x70,0x02);

  ACC_ORIENTATION(  (((rawADC[1])<<8) | ((rawADC[0]>>1)<<1))/64 ,
                    (((rawADC[3])<<8) | ((rawADC[2]>>1)<<1))/64 ,
                    (((rawADC[5])<<8) | ((rawADC[4]>>1)<<1))/64 );
  ACC_Common();
}
#endif

// **************************
// standalone I2C Nunchuk
// **************************
#if defined(NUNCHACK)
void ACC_init() {
  i2c_writeReg(0xA4 ,0xF0 ,0x55 );
  i2c_writeReg(0xA4 ,0xFB ,0x00 );
  delay(250);
  acc_1G = 200;
}

void ACC_getADC() {
  TWBR = ((16000000L / I2C_SPEED) - 16) / 2; // change the I2C clock rate. !! you must check if the nunchuk is ok with this freq
  i2c_getSixRawADC(0xA4,0x00);

  ACC_ORIENTATION(  ( (rawADC[3]<<2)        + ((rawADC[5]>>4)&0x2) ) ,
                  - ( (rawADC[2]<<2)        + ((rawADC[5]>>3)&0x2) ) ,
                    ( ((rawADC[4]&0xFE)<<2) + ((rawADC[5]>>5)&0x6) ));
  ACC_Common();
}
#endif

// **************************
// ADC ACC
// **************************
#if defined(ADCACC)
void ACC_init(){
  pinMode(A1,INPUT);
  pinMode(A2,INPUT);
  pinMode(A3,INPUT);
  
  acc_1G = 75;
}

void ACC_getADC() {
  ACC_ORIENTATION( -analogRead(A1) ,
                   -analogRead(A2) ,
                    analogRead(A3) );
  ACC_Common();
}
#endif

// **************************
// contribution from Ciskje
// I2C Gyroscope L3G4200D 
// **************************
#if defined(L3G4200D)
void Gyro_init() {
  delay(100);
  i2c_writeReg(0XD2+0 ,0x20 ,0x8F ); // CTRL_REG1   400Hz ODR, 20hz filter, run!
  delay(5);
  i2c_writeReg(0XD2+0 ,0x24 ,0x02 ); // CTRL_REG5   low pass filter enable
}

void Gyro_getADC () {
  TWBR = ((16000000L / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz
  i2c_getSixRawADC(0XD2,0x80|0x28);

  GYRO_ORIENTATION(  ((rawADC[1]<<8) | rawADC[0])/20  ,
                     ((rawADC[3]<<8) | rawADC[2])/20  ,
                    -((rawADC[5]<<8) | rawADC[4])/20  );
  GYRO_Common();
}
#endif

// **************************
// I2C Gyroscope ITG3200 
// **************************
// I2C adress: 0xD2 (8bit)   0x69 (7bit)
// I2C adress: 0xD0 (8bit)   0x68 (7bit)
// principle:
// 1) VIO is connected to VDD
// 2) I2C adress is set to 0x69 (AD0 PIN connected to VDD)
// or 2) I2C adress is set to 0x68 (AD0 PIN connected to GND)
// 3) sample rate = 1000Hz ( 1kHz/(div+1) )
#if defined(ITG3200)
void Gyro_init() {
  delay(100);
  i2c_writeReg(ITG3200_ADDRESS ,0x3E ,0x80 ); //register: Power Management  --  value: reset device
  delay(5);
  i2c_writeReg(ITG3200_ADDRESS ,0x15 ,0x7 ); //register: Sample Rate Divider  --  value: 7: 8000Hz/(7+1) = 1000Hz . more than twice the need
  delay(5);
  i2c_writeReg(ITG3200_ADDRESS ,0x16 ,0x18 ); //register: DLPF_CFG - low pass filter configuration & sample rate  --  value: 256Hz Low Pass Filter Bandwidth - Internal Sample Rate 8kHz
  delay(5);
  i2c_writeReg(ITG3200_ADDRESS ,0x3E ,0x03 ); //register: Power Management  --  value: PLL with Z Gyro reference
  delay(100);
}

void Gyro_getADC () {
  TWBR = ((16000000L / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz
  i2c_getSixRawADC(ITG3200_ADDRESS,0X1D);
  
  GYRO_ORIENTATION(  + ( ((rawADC[2]<<8) | rawADC[3])/4 ) , // range: +/- 10922
                     - ( ((rawADC[0]<<8) | rawADC[1])/4 ) ,
                     - ( ((rawADC[4]<<8) | rawADC[5])/4 ) );

  GYRO_Common();
}
#endif


// **************************
// I2C Compass HMC5843 & HMC5883
// **************************
// I2C adress: 0x3C (8bit)   0x1E (7bit)
#if MAG
void Mag_init() { 
  delay(100);
  i2c_writeReg(0X3C ,0x02 ,0x00 ); //register: Mode register  --  value: Continuous-Conversion Mode
}

void Mag_getADC() {
  static uint32_t t,tCal = 0;
  uint8_t axis;
  static int16_t magZeroTempMin[3];
  static int16_t magZeroTempMax[3];

  if ( (micros()-t )  < 100000 ) return; //each read is spaced by 100ms
  t = micros();
  TWBR = ((16000000L / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz
  i2c_getSixRawADC(0X3C,0X03);

  #if defined(HMC5843)
    MAG_ORIENTATION( ((rawADC[0]<<8) | rawADC[1]) ,
                     ((rawADC[2]<<8) | rawADC[3]) ,
                    -((rawADC[4]<<8) | rawADC[5]) );
  #endif
  #if defined (HMC5883)
    MAG_ORIENTATION( ((rawADC[4]<<8) | rawADC[5]) ,
                    -((rawADC[0]<<8) | rawADC[1]) ,
                    -((rawADC[2]<<8) | rawADC[3]) );
  #endif

  if (calibratingM == 1) {
    tCal = t;
    for(axis=0;axis<3;axis++) {magZero[axis] = 0;magZeroTempMin[axis] = 0; magZeroTempMax[axis] = 0;}
    calibratingM = 0;
  }
  magADC[ROLL]  -= magZero[ROLL];
  magADC[PITCH] -= magZero[PITCH];
  magADC[YAW]   -= magZero[YAW];

  if (tCal != 0) {
    if ((t - tCal) < 30000000) { // 30s: you have 30s to turn the multi in all directions
      LEDPIN_SWITCH
      for(axis=0;axis<3;axis++) {
        if (magADC[axis] < magZeroTempMin[axis]) magZeroTempMin[axis] = magADC[axis];
        if (magADC[axis] > magZeroTempMax[axis]) magZeroTempMax[axis] = magADC[axis];
      }
    } else {
      tCal = 0;
      for(axis=0;axis<3;axis++)
        magZero[axis] = (magZeroTempMin[axis] + magZeroTempMax[axis])/2;
      writeParams();
    }
  }
}
#endif

// **************************
// I2C Wii Motion Plus + optional Nunchuk
// **************************
// I2C adress 1: 0xA6 (8bit)    0x53 (7bit)
// I2C adress 2: 0xA4 (8bit)    0x52 (7bit)

void WMP_init(uint8_t d) {
  if(GYRO) return;
  delay(d);
  i2c_writeReg(0xA6 ,0xF0 ,0x55 );
  delay(d);
  i2c_writeReg(0xA6 ,0xFE ,0x05 );
  delay(d);
  if (d>0) {
    uint8_t numberAccRead = 0;
    for(uint8_t i=0;i<100;i++) {
      delay(4);
      if (WMP_getRawADC() == 0) numberAccRead++; // we detect here is nunchuk extension is available
    }
    if (numberAccRead>25) {
      nunchuk = 1;
      acc_1G = 200;
    }
    delay(10);
  }
}

uint8_t WMP_getRawADC() {
  uint8_t axis;
  TWBR = ((16000000L / I2C_SPEED) - 16) / 2; // change the I2C clock rate
  i2c_getSixRawADC(0xA4,0x00);

  if (micros() < (neutralizeTime + NEUTRALIZE_DELAY)) {//we neutralize data in case of blocking+hard reset state
    for (axis = 0; axis < 3; axis++) {gyroADC[axis]=0;accADC[axis]=0;}
    accADC[YAW] = acc_1G;
    return 1;
  } 

  if ( (rawADC[5]&0x02) == 0x02 && (rawADC[5]&0x01) == 0 ) {// motion plus data
    GYRO_ORIENTATION( - ( ((rawADC[5]>>2)<<8) + rawADC[2] ) , //range: +/- 8192
                      - ( ((rawADC[4]>>2)<<8) + rawADC[1] ) ,
                      - ( ((rawADC[3]>>2)<<8) + rawADC[0] ) );
    GYRO_Common();
    GYRO_ORIENTATION(   (rawADC[3]&0x01)     ? gyroADC[ROLL]/5  : gyroADC[ROLL] ,   //the ratio 1/5 is not exactly the IDG600 or ISZ650 specification 
                        (rawADC[4]&0x02)>>1  ? gyroADC[PITCH]/5 : gyroADC[PITCH] ,  //we detect here the slow of fast mode WMP gyros values (see wiibrew for more details)
                        (rawADC[3]&0x02)>>1  ? gyroADC[YAW]/5   : gyroADC[YAW]   ); // this step must be done after zero compensation    
    return 1;
  } else if ( (rawADC[5]&0x02) == 0 && (rawADC[5]&0x01) == 0) { //nunchuk data
    ACC_ORIENTATION(  ( (rawADC[3]<<2)        + ((rawADC[5]>>4)&0x2) ) ,
                    - ( (rawADC[2]<<2)        + ((rawADC[5]>>3)&0x2) ) ,
                      ( ((rawADC[4]&0xFE)<<2) + ((rawADC[5]>>5)&0x6) ) );
    ACC_Common();
    return 0;
  } else
    return 2;
}

void initSensors() {
  delay(200);
  POWERPIN_ON
  delay(100);
  i2c_init();
  delay(100);
  if (GYRO) Gyro_init();
  else WMP_init(250);
  if (BARO) Baro_init();
  if (ACC) ACC_init();
  if (MAG) Mag_init();
}
