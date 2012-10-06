// ************************************************************************************************************
// board orientation and setup
// ************************************************************************************************************
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
#if !defined(MMA7455_ADDRESS)
  #define MMA7455_ADDRESS 0x1D
#endif

#if !defined(ADXL345_ADDRESS) 
  #define ADXL345_ADDRESS 0x1D
  //#define ADXL345_ADDRESS 0x53   //WARNING: Conflicts with a Wii Motion plus!
#endif

#if !defined(BMA180_ADDRESS) 
  #define BMA180_ADDRESS 0x40
  //#define BMA180_ADDRESS 0x41
#endif

#if !defined(ITG3200_ADDRESS) 
  #define ITG3200_ADDRESS 0X68
  //#define ITG3200_ADDRESS 0X69
#endif

#if !defined(MPU6050_ADDRESS)
  #define MPU6050_ADDRESS     0x68 // address pin AD0 low (GND), default for FreeIMU v0.4 and InvenSense evaluation board
  //#define MPU6050_ADDRESS     0x69 // address pin AD0 high (VCC)
#endif

#if !defined(MPU3050_ADDRESS)
  #define MPU3050_ADDRESS     0x68 // Switch in "ON" position
  //#define MPU3050_ADDRESS     0x69 // Switch in "1" position
#endif

#if !defined(MS561101BA_ADDRESS) 
  #define MS561101BA_ADDRESS 0x77 //CBR=0 0xEE I2C address when pin CSB is connected to LOW (GND)
  //#define MS561101BA_ADDRESS 0x76 //CBR=1 0xEC I2C address when pin CSB is connected to HIGH (VCC)
#endif

//ITG3200 and ITG3205 Gyro LPF setting
#if defined(ITG3200_LPF_256HZ) || defined(ITG3200_LPF_188HZ) || defined(ITG3200_LPF_98HZ) || defined(ITG3200_LPF_42HZ) || defined(ITG3200_LPF_20HZ) || defined(ITG3200_LPF_10HZ)
  #if defined(ITG3200_LPF_256HZ)
    #define ITG3200_SMPLRT_DIV 0  //8000Hz
    #define ITG3200_DLPF_CFG   0
  #endif
  #if defined(ITG3200_LPF_188HZ)
    #define ITG3200_SMPLRT_DIV 0  //1000Hz
    #define ITG3200_DLPF_CFG   1
  #endif
  #if defined(ITG3200_LPF_98HZ)
    #define ITG3200_SMPLRT_DIV 0
    #define ITG3200_DLPF_CFG   2
  #endif
  #if defined(ITG3200_LPF_42HZ)
    #define ITG3200_SMPLRT_DIV 0
    #define ITG3200_DLPF_CFG   3
  #endif
  #if defined(ITG3200_LPF_20HZ)
    #define ITG3200_SMPLRT_DIV 0
    #define ITG3200_DLPF_CFG   4
  #endif
  #if defined(ITG3200_LPF_10HZ)
    #define ITG3200_SMPLRT_DIV 0
    #define ITG3200_DLPF_CFG   5
  #endif
#else
    //Default settings LPF 256Hz/8000Hz sample
    #define ITG3200_SMPLRT_DIV 0  //8000Hz
    #define ITG3200_DLPF_CFG   0
#endif

//MPU6050 Gyro LPF setting
#if defined(MPU6050_LPF_256HZ) || defined(MPU6050_LPF_188HZ) || defined(MPU6050_LPF_98HZ) || defined(MPU6050_LPF_42HZ) || defined(MPU6050_LPF_20HZ) || defined(MPU6050_LPF_10HZ) || defined(MPU6050_LPF_5HZ)
  #if defined(MPU6050_LPF_256HZ)
    #define MPU6050_DLPF_CFG   0
  #endif
  #if defined(MPU6050_LPF_188HZ)
    #define MPU6050_DLPF_CFG   1
  #endif
  #if defined(MPU6050_LPF_98HZ)
    #define MPU6050_DLPF_CFG   2
  #endif
  #if defined(MPU6050_LPF_42HZ)
    #define MPU6050_DLPF_CFG   3
  #endif
  #if defined(MPU6050_LPF_20HZ)
    #define MPU6050_DLPF_CFG   4
  #endif
  #if defined(MPU6050_LPF_10HZ)
    #define MPU6050_DLPF_CFG   5
  #endif
  #if defined(MPU6050_LPF_5HZ)
    #define MPU6050_DLPF_CFG   6
  #endif
#else
    //Default settings LPF 256Hz/8000Hz sample
    #define MPU6050_DLPF_CFG   0
#endif

//MPU3050 Gyro LPF setting
#if defined(MPU3050_LPF_256HZ) || defined(MPU3050_LPF_188HZ) || defined(MPU3050_LPF_98HZ) || defined(MPU3050_LPF_42HZ) || defined(MPU3050_LPF_20HZ) || defined(MPU3050_LPF_10HZ) || defined(MPU3050_LPF_5HZ)
  #if defined(MPU3050_LPF_256HZ)
    #define MPU3050_DLPF_CFG   0
  #endif
  #if defined(MPU3050_LPF_188HZ)
    #define MPU3050_DLPF_CFG   1
  #endif
  #if defined(MPU3050_LPF_98HZ)
    #define MPU3050_DLPF_CFG   2
  #endif
  #if defined(MPU3050_LPF_42HZ)
    #define MPU3050_DLPF_CFG   3
  #endif
  #if defined(MPU3050_LPF_20HZ)
    #define MPU3050_DLPF_CFG   4
  #endif
  #if defined(MPU3050_LPF_10HZ)
    #define MPU3050_DLPF_CFG   5
  #endif
  #if defined(MPU3050_LPF_5HZ)
    #define MPU3050_DLPF_CFG   6
  #endif
#else
    //Default settings LPF 256Hz/8000Hz sample
    #define MPU3050_DLPF_CFG   0
#endif

#if defined(TINY_GPS) | defined(TINY_GPS_SONAR)
#define TINY_GPS_TWI_ADD 0x11
#include "tinygps.h"
#endif

uint8_t rawADC[6];
static uint32_t neutralizeTime = 0;
  
// ************************************************************************************************************
// I2C general functions
// ************************************************************************************************************

void i2c_init(void) {
  #if defined(INTERNAL_I2C_PULLUPS)
    I2C_PULLUPS_ENABLE
  #else
    I2C_PULLUPS_DISABLE
  #endif
  TWSR = 0;                                    // no prescaler => prescaler = 1
  TWBR = ((F_CPU / I2C_SPEED) - 16) / 2;   // change the I2C clock rate
  TWCR = 1<<TWEN;                              // enable twi module, no interrupt
}

void i2c_rep_start(uint8_t address) {
  TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN) ; // send REPEAT START condition
  waitTransmissionI2C();                       // wait until transmission completed
  TWDR = address;                              // send device address
  TWCR = (1<<TWINT) | (1<<TWEN);
  waitTransmissionI2C();                       // wail until transmission completed
}

void i2c_stop(void) {
  TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
  //  while(TWCR & (1<<TWSTO));                // <- can produce a blocking state with some WMP clones
}

void i2c_write(uint8_t data ) {
  TWDR = data;                                 // send data to the previously addressed device
  TWCR = (1<<TWINT) | (1<<TWEN);
  waitTransmissionI2C();
}

uint8_t i2c_read(uint8_t ack) {
  TWCR = (1<<TWINT) | (1<<TWEN) | (ack? (1<<TWEA) : 0);
  waitTransmissionI2C();
  uint8_t r = TWDR;
  if (!ack) i2c_stop();
  return r;
}

uint8_t i2c_readAck() {
  return i2c_read(1);
}

uint8_t i2c_readNak(void) {
  return i2c_read(0);
}

void waitTransmissionI2C() {
  uint16_t count = 255;
  while (!(TWCR & (1<<TWINT))) {
    count--;
    if (count==0) {              //we are in a blocking state => we don't insist
      TWCR = 0;                  //and we force a reset on TWINT register
      neutralizeTime = micros(); //we take a timestamp here to neutralize the value during a short delay
      i2c_errors_count++;
      break;
    }
  }
}

size_t i2c_read_to_buf(uint8_t add, void *buf, size_t size) {
  i2c_rep_start((add<<1) | 1);  // I2C read direction
  size_t bytes_read = 0;
  uint8_t *b = (uint8_t*)buf;
  while (size--) {
    /* acknowledge all but the final byte */
    *b++ = i2c_read(size > 0);
    /* TODO catch I2C errors here and abort */
    bytes_read++;
  }
  return bytes_read;
}

size_t i2c_read_reg_to_buf(uint8_t add, uint8_t reg, void *buf, size_t size) {
  i2c_rep_start(add<<1); // I2C write direction
  i2c_write(reg);        // register selection
  return i2c_read_to_buf(add, buf, size);
}

/* transform a series of bytes from big endian to little
   endian and vice versa. */
void swap_endianness(void *buf, size_t size) {
  /* we swap in-place, so we only have to
  * place _one_ element on a temporary tray
  */
  uint8_t tray;
  uint8_t *from;
  uint8_t *to;
  /* keep swapping until the pointers have assed each other */
  for (from = (uint8_t*)buf, to = &from[size-1]; from < to; from++, to--) {
    tray = *from;
    *from = *to;
    *to = tray;
  }
}

void i2c_getSixRawADC(uint8_t add, uint8_t reg) {
  i2c_read_reg_to_buf(add, reg, &rawADC, 6);
}

void i2c_writeReg(uint8_t add, uint8_t reg, uint8_t val) {
  i2c_rep_start(add<<1); // I2C write direction
  i2c_write(reg);        // register selection
  i2c_write(val);        // value to write in register
  i2c_stop();
}

uint8_t i2c_readReg(uint8_t add, uint8_t reg) {
  uint8_t val;
  i2c_read_reg_to_buf(add, reg, &val, 1);
  return val;
}

// ****************
// GYRO common part
// ****************
void GYRO_Common() {
  static int16_t previousGyroADC[3] = {0,0,0};
  static int32_t g[3];
  uint8_t axis;

#if defined MMGYRO       
  // Moving Average Gyros by Magnetron1
  //---------------------------------------------------
  static int16_t mediaMobileGyroADC[3][MMGYROVECTORLENGHT];
  static int32_t mediaMobileGyroADCSum[3];
  static uint8_t mediaMobileGyroIDX;
  //---------------------------------------------------
#endif

  if (calibratingG>0) {
    for (axis = 0; axis < 3; axis++) {
      // Reset g[axis] at start of calibration
      if (calibratingG == 400) g[axis]=0;
      // Sum up 400 readings
      g[axis] +=gyroADC[axis];
      // Clear global variables for next reading
      gyroADC[axis]=0;
      gyroZero[axis]=0;
      if (calibratingG == 1) {
        gyroZero[axis]=g[axis]/400;
        blinkLED(10,15,1);
      #if defined(BUZZER)
        notification_confirmation = 4;
      #endif
      }
    }
    calibratingG--;
  }

#ifdef MMGYRO       
  mediaMobileGyroIDX = ++mediaMobileGyroIDX % MMGYROVECTORLENGHT;
  for (axis = 0; axis < 3; axis++) {
    gyroADC[axis]  -= gyroZero[axis];
    mediaMobileGyroADCSum[axis] -= mediaMobileGyroADC[axis][mediaMobileGyroIDX];
    //anti gyro glitch, limit the variation between two consecutive readings
    mediaMobileGyroADC[axis][mediaMobileGyroIDX] = constrain(gyroADC[axis],previousGyroADC[axis]-800,previousGyroADC[axis]+800);
    mediaMobileGyroADCSum[axis] += mediaMobileGyroADC[axis][mediaMobileGyroIDX];
    gyroADC[axis] = mediaMobileGyroADCSum[axis] / MMGYROVECTORLENGHT;
#else 
  for (axis = 0; axis < 3; axis++) {
    gyroADC[axis]  -= gyroZero[axis];
    //anti gyro glitch, limit the variation between two consecutive readings
    gyroADC[axis] = constrain(gyroADC[axis],previousGyroADC[axis]-800,previousGyroADC[axis]+800);
#endif    
    previousGyroADC[axis] = gyroADC[axis];
  }

  #if defined(SENSORS_TILT_45DEG_LEFT)
    int16_t temp  = ((gyroADC[PITCH] - gyroADC[ROLL] )*7)/10;
    gyroADC[ROLL] = ((gyroADC[ROLL]  + gyroADC[PITCH])*7)/10;
    gyroADC[PITCH]= temp;
  #endif
  #if defined(SENSORS_TILT_45DEG_RIGHT)
    int16_t temp  = ((gyroADC[PITCH] + gyroADC[ROLL] )*7)/10;
    gyroADC[ROLL] = ((gyroADC[ROLL]  - gyroADC[PITCH])*7)/10;
    gyroADC[PITCH]= temp;
  #endif
}

// ****************
// ACC common part
// ****************
void ACC_Common() {
  static int32_t a[3];
  
  if (calibratingA>0) {
    for (uint8_t axis = 0; axis < 3; axis++) {
      // Reset a[axis] at start of calibration
      if (calibratingA == 400) a[axis]=0;
      // Sum up 400 readings
      a[axis] +=accADC[axis];
      // Clear global variables for next reading
      accADC[axis]=0;
      global_conf.accZero[axis]=0;
    }
    // Calculate average, shift Z down by acc_1G and store values in EEPROM at end of calibration
    if (calibratingA == 1) {
      global_conf.accZero[ROLL]  = a[ROLL]/400;
      global_conf.accZero[PITCH] = a[PITCH]/400;
      global_conf.accZero[YAW]   = a[YAW]/400-acc_1G; // for nunchuk 200=1G
      conf.angleTrim[ROLL]   = 0;
      conf.angleTrim[PITCH]  = 0;
      writeGlobalSet(1); // write accZero in EEPROM
    }
    calibratingA--;
  }
  #if defined(INFLIGHT_ACC_CALIBRATION)
      static int32_t b[3];
      static int16_t accZero_saved[3]  = {0,0,0};
      static int16_t  angleTrim_saved[2] = {0, 0};
      //Saving old zeropoints before measurement
      if (InflightcalibratingA==50) {
         accZero_saved[ROLL]  = global_conf.accZero[ROLL] ;
         accZero_saved[PITCH] = global_conf.accZero[PITCH];
         accZero_saved[YAW]   = global_conf.accZero[YAW] ;
         angleTrim_saved[ROLL]  = conf.angleTrim[ROLL] ;
         angleTrim_saved[PITCH] = conf.angleTrim[PITCH] ;
      }
      if (InflightcalibratingA>0) {
        for (uint8_t axis = 0; axis < 3; axis++) {
          // Reset a[axis] at start of calibration
          if (InflightcalibratingA == 50) b[axis]=0;
          // Sum up 50 readings
          b[axis] +=accADC[axis];
          // Clear global variables for next reading
          accADC[axis]=0;
          global_conf.accZero[axis]=0;
        }
        //all values are measured
        if (InflightcalibratingA == 1) {
          AccInflightCalibrationActive = 0;
          AccInflightCalibrationMeasurementDone = 1;
          #if defined(BUZZER)
            notification_confirmation = 1;      //buzzer for indicatiing the end of calibration
          #endif
          // recover saved values to maintain current flight behavior until new values are transferred
          global_conf.accZero[ROLL]  = accZero_saved[ROLL] ;
          global_conf.accZero[PITCH] = accZero_saved[PITCH];
          global_conf.accZero[YAW]   = accZero_saved[YAW] ;
          conf.angleTrim[ROLL]  = angleTrim_saved[ROLL] ;
          conf.angleTrim[PITCH] = angleTrim_saved[PITCH] ;
        }
        InflightcalibratingA--;
      }
      // Calculate average, shift Z down by acc_1G and store values in EEPROM at end of calibration
      if (AccInflightCalibrationSavetoEEProm == 1){  //the copter is landed, disarmed and the combo has been done again
        AccInflightCalibrationSavetoEEProm = 0;
        global_conf.accZero[ROLL]  = b[ROLL]/50;
        global_conf.accZero[PITCH] = b[PITCH]/50;
        global_conf.accZero[YAW]   = b[YAW]/50-acc_1G; // for nunchuk 200=1G
        conf.angleTrim[ROLL]   = 0;
        conf.angleTrim[PITCH]  = 0;
        writeGlobalSet(1); // write accZero in EEPROM
      }
  #endif
  accADC[ROLL]  -=  global_conf.accZero[ROLL] ;
  accADC[PITCH] -=  global_conf.accZero[PITCH];
  accADC[YAW]   -=  global_conf.accZero[YAW] ;

  #if defined(SENSORS_TILT_45DEG_LEFT)
    int16_t temp = ((accADC[PITCH] - accADC[ROLL] )*7)/10;
    accADC[ROLL] = ((accADC[ROLL]  + accADC[PITCH])*7)/10;
    accADC[PITCH] = temp;
  #endif
  #if defined(SENSORS_TILT_45DEG_RIGHT)
    int16_t temp = ((accADC[PITCH] + accADC[ROLL] )*7)/10;
    accADC[ROLL] = ((accADC[ROLL]  - accADC[PITCH])*7)/10;
    accADC[PITCH] = temp;
  #endif
}


// ************************************************************************************************************
// I2C Barometer BOSCH BMP085
// ************************************************************************************************************
// I2C adress: 0x77 (7bit)
// principle:
//  1) read the calibration register (only once at the initialization)
//  2) read uncompensated temperature (not mandatory at every cycle)
//  3) read uncompensated pressure
//  4) raw temp + raw pressure => calculation of the adjusted pressure
//  the following code uses the maximum precision setting (oversampling setting 3)
// ************************************************************************************************************

#if defined(BMP085)
#define BMP085_ADDRESS 0x77
static int32_t  pressure;

static struct {
  // sensor registers from the BOSCH BMP085 datasheet
  int16_t  ac1, ac2, ac3;
  uint16_t ac4, ac5, ac6;
  int16_t  b1, b2, mb, mc, md;
  union {uint16_t val; uint8_t raw[2]; } ut; //uncompensated T
  union {uint32_t val; uint8_t raw[4]; } up; //uncompensated P
  uint8_t  state;
  uint32_t deadline;
} bmp085_ctx;  
#define OSS 2 //we can get more unique samples and get better precision using average

void i2c_BMP085_readCalibration(){
  delay(10);
  //read calibration data in one go
  size_t s_bytes = (uint8_t*)&bmp085_ctx.md - (uint8_t*)&bmp085_ctx.ac1 + sizeof(bmp085_ctx.ac1);
  i2c_read_reg_to_buf(BMP085_ADDRESS, 0xAA, &bmp085_ctx.ac1, s_bytes);
  // now fix endianness
  int16_t *p;
  for (p = &bmp085_ctx.ac1; p <= &bmp085_ctx.md; p++) {
    swap_endianness(p, sizeof(*p));
  }
}

void  Baro_init() {
  delay(10);
  i2c_BMP085_readCalibration();
  i2c_BMP085_UT_Start(); 
  delay(5);
  i2c_BMP085_UT_Read();
}

// read uncompensated temperature value: send command first
void i2c_BMP085_UT_Start() {
  i2c_writeReg(BMP085_ADDRESS,0xf4,0x2e);
  i2c_rep_start(BMP085_ADDRESS<<1);
  i2c_write(0xF6);
  i2c_stop();
}

// read uncompensated pressure value: send command first
void i2c_BMP085_UP_Start () {
  i2c_writeReg(BMP085_ADDRESS,0xf4,0x34+(OSS<<6)); // control register value for oversampling setting 3
  i2c_rep_start(BMP085_ADDRESS<<1); //I2C write direction => 0
  i2c_write(0xF6);
  i2c_stop();
}

// read uncompensated pressure value: read result bytes
// the datasheet suggests a delay of 25.5 ms (oversampling settings 3) after the send command
void i2c_BMP085_UP_Read () {
  i2c_rep_start((BMP085_ADDRESS<<1) | 1);//I2C read direction => 1
  bmp085_ctx.up.raw[2] = i2c_readAck();
  bmp085_ctx.up.raw[1] = i2c_readAck();
  bmp085_ctx.up.raw[0] = i2c_readNak();
}

// read uncompensated temperature value: read result bytes
// the datasheet suggests a delay of 4.5 ms after the send command
void i2c_BMP085_UT_Read() {
  i2c_rep_start((BMP085_ADDRESS<<1) | 1);//I2C read direction => 1
  bmp085_ctx.ut.raw[1] = i2c_readAck();
  bmp085_ctx.ut.raw[0] = i2c_readNak();
}

void i2c_BMP085_Calculate() {
  int32_t  x1, x2, x3, b3, b5, b6, p, tmp;
  uint32_t b4, b7;
  // Temperature calculations
  x1 = ((int32_t)bmp085_ctx.ut.val - bmp085_ctx.ac6) * bmp085_ctx.ac5 >> 15;
  x2 = ((int32_t)bmp085_ctx.mc << 11) / (x1 + bmp085_ctx.md);
  b5 = x1 + x2;
  // Pressure calculations
  b6 = b5 - 4000;
  x1 = (bmp085_ctx.b2 * (b6 * b6 >> 12)) >> 11; 
  x2 = bmp085_ctx.ac2 * b6 >> 11;
  x3 = x1 + x2;
  tmp = bmp085_ctx.ac1;
  tmp = (tmp*4 + x3) << OSS;
  b3 = (tmp+2)/4;
  x1 = bmp085_ctx.ac3 * b6 >> 13;
  x2 = (bmp085_ctx.b1 * (b6 * b6 >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  b4 = (bmp085_ctx.ac4 * (uint32_t)(x3 + 32768)) >> 15;
  b7 = ((uint32_t) (bmp085_ctx.up.val >> (8-OSS)) - b3) * (50000 >> OSS);
  p = b7 < 0x80000000 ? (b7 * 2) / b4 : (b7 / b4) * 2;
  x1 = (p >> 8) * (p >> 8);
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * p) >> 16;
  pressure = p + ((x1 + x2 + 3791) >> 4);
}

void Baro_update() {
  if (currentTime < bmp085_ctx.deadline) return; 
  bmp085_ctx.deadline = currentTime;
  TWBR = ((F_CPU / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz, BMP085 is ok with this speed
  switch (bmp085_ctx.state) {
    case 0: 
      i2c_BMP085_UT_Start(); 
      bmp085_ctx.state++; bmp085_ctx.deadline += 4600; 
      break;
    case 1: 
      i2c_BMP085_UT_Read(); 
      bmp085_ctx.state++; 
      break;
    case 2: 
      i2c_BMP085_UP_Start(); 
      bmp085_ctx.state++; bmp085_ctx.deadline += 14000; 
      break;
    case 3: 
      i2c_BMP085_UP_Read(); 
      i2c_BMP085_Calculate(); 
      BaroAlt = (1.0f - pow(pressure/101325.0f, 0.190295f)) * 4433000.0f; //centimeter
      bmp085_ctx.state = 0; bmp085_ctx.deadline += 5000; 
      break;
  } 
}
#endif

// ************************************************************************************************************
// I2C Barometer MS561101BA
// ************************************************************************************************************
//
// specs are here: http://www.meas-spec.com/downloads/MS5611-01BA03.pdf
// useful info on pages 7 -> 12
#if defined(MS561101BA)

// registers of the device
#define MS561101BA_PRESSURE    0x40
#define MS561101BA_TEMPERATURE 0x50
#define MS561101BA_RESET       0x1E

// OSR (Over Sampling Ratio) constants
#define MS561101BA_OSR_256  0x00
#define MS561101BA_OSR_512  0x02
#define MS561101BA_OSR_1024 0x04
#define MS561101BA_OSR_2048 0x06
#define MS561101BA_OSR_4096 0x08

#define OSR MS561101BA_OSR_4096
static int32_t  pressure;

static struct {
  // sensor registers from the MS561101BA datasheet
  uint16_t c[7];
  union {uint32_t val; uint8_t raw[4]; } ut; //uncompensated T
  union {uint32_t val; uint8_t raw[4]; } up; //uncompensated P
  uint8_t  state;
  uint32_t deadline;
} ms561101ba_ctx;

void i2c_MS561101BA_reset(){
  i2c_writeReg(MS561101BA_ADDRESS, MS561101BA_RESET, 0);
}

void i2c_MS561101BA_readCalibration(){
  union {uint16_t val; uint8_t raw[2]; } data;
  for(uint8_t i=0;i<6;i++) {
    i2c_rep_start(MS561101BA_ADDRESS<<1);
    i2c_write(0xA2+2*i);
    delay(10);
    i2c_rep_start((MS561101BA_ADDRESS<<1) | 1);//I2C read direction => 1
    delay(10);
    data.raw[1] = i2c_readAck();  // read a 16 bit register
    data.raw[0] = i2c_readNak();
    ms561101ba_ctx.c[i+1] = data.val;
  }
}

void  Baro_init() {
  delay(10);
  i2c_MS561101BA_reset();
  delay(100);
  i2c_MS561101BA_readCalibration();
}

// read uncompensated temperature value: send command first
void i2c_MS561101BA_UT_Start() {
  i2c_rep_start(MS561101BA_ADDRESS<<1);      // I2C write direction
  i2c_write(MS561101BA_TEMPERATURE + OSR);  // register selection
  i2c_stop();
}

// read uncompensated pressure value: send command first
void i2c_MS561101BA_UP_Start () {
  i2c_rep_start(MS561101BA_ADDRESS<<1);      // I2C write direction
  i2c_write(MS561101BA_PRESSURE + OSR);     // register selection
  i2c_stop();
}

// read uncompensated pressure value: read result bytes
void i2c_MS561101BA_UP_Read () {
  i2c_rep_start(MS561101BA_ADDRESS<<1);
  i2c_write(0);
  i2c_rep_start((MS561101BA_ADDRESS<<1) | 1);
  ms561101ba_ctx.up.raw[2] = i2c_readAck();
  ms561101ba_ctx.up.raw[1] = i2c_readAck();
  ms561101ba_ctx.up.raw[0] = i2c_readNak();
}

// read uncompensated temperature value: read result bytes
void i2c_MS561101BA_UT_Read() {
  i2c_rep_start(MS561101BA_ADDRESS<<1);
  i2c_write(0);
  i2c_rep_start((MS561101BA_ADDRESS<<1) | 1);
  ms561101ba_ctx.ut.raw[2] = i2c_readAck();
  ms561101ba_ctx.ut.raw[1] = i2c_readAck();
  ms561101ba_ctx.ut.raw[0] = i2c_readNak();
}

void i2c_MS561101BA_Calculate() {
  int32_t temperature,off2=0,sens2=0,delt;

  int32_t dT   = ms561101ba_ctx.ut.val - ((uint32_t)ms561101ba_ctx.c[5] << 8);
  int64_t off  = ((uint32_t)ms561101ba_ctx.c[2] <<16) + (((int64_t)dT * ms561101ba_ctx.c[4]) >> 7);
  int64_t sens = ((uint32_t)ms561101ba_ctx.c[1] <<15) + (((int64_t)dT * ms561101ba_ctx.c[3]) >> 8);
  temperature  = 2000 + (((int64_t)dT * ms561101ba_ctx.c[6])>>23);

  if (temperature < 2000) { // temperature lower than 20st.C 
    delt = temperature-2000;
    delt  = delt*delt;
    off2  = (5 * delt)>>1; 
    sens2 = (5 * delt)>>2; 
    if (temperature < -1500) { // temperature lower than -15st.C
      delt  = temperature+1500;
      delt  = delt*delt;
      off2  += 7 * delt; 
      sens2 += (11 * delt)>>1; 
    }
  } 
  off  -= off2; 
  sens -= sens2;
  pressure     = (( (ms561101ba_ctx.up.val * sens ) >> 21) - off) >> 15;
}

void Baro_update() {
  if (currentTime < ms561101ba_ctx.deadline) return; 
  ms561101ba_ctx.deadline = currentTime;
  TWBR = ((F_CPU / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz, MS5611 is ok with this speed
  switch (ms561101ba_ctx.state) {
    case 0: 
      i2c_MS561101BA_UT_Start(); 
      ms561101ba_ctx.state++; ms561101ba_ctx.deadline += 10000; //according to the specs, the pause should be at least 8.22ms
      break;
    case 1: 
      i2c_MS561101BA_UT_Read(); 
      ms561101ba_ctx.state++;
      break;
    case 2: 
      i2c_MS561101BA_UP_Start(); 
      ms561101ba_ctx.state++; ms561101ba_ctx.deadline += 10000; //according to the specs, the pause should be at least 8.22ms
      break;
    case 3: 
      i2c_MS561101BA_UP_Read();
      i2c_MS561101BA_Calculate();
      BaroAlt = (1.0f - pow(pressure/101325.0f, 0.190295f)) * 4433000.0f; //centimeter
      ms561101ba_ctx.state = 0; ms561101ba_ctx.deadline += 4000;
      break;
  } 
}
#endif

// ************************************************************************************************************
// I2C Accelerometer MMA7455 
// ************************************************************************************************************
#if defined(MMA7455)
void ACC_init () {
  delay(10);
  i2c_writeReg(MMA7455_ADDRESS,0x16,0x21);
  acc_1G = 64;
}

void ACC_getADC () {
  TWBR = ((F_CPU / 400000L) - 16) / 2;
  i2c_getSixRawADC(MMA7455_ADDRESS,0x00);

  ACC_ORIENTATION( ((int8_t(rawADC[1])<<8) | int8_t(rawADC[0])) ,
                   ((int8_t(rawADC[3])<<8) | int8_t(rawADC[2])) ,
                   ((int8_t(rawADC[5])<<8) | int8_t(rawADC[4])) );
  ACC_Common();
}
#endif

// ************************************************************************************************************
// I2C Accelerometer ADXL345 
// ************************************************************************************************************
// I2C adress: 0x3A (8bit)    0x1D (7bit)
// Resolution: 10bit (Full range - 14bit, but this is autoscaling 10bit ADC to the range +- 16g)
// principle:
//  1) CS PIN must be linked to VCC to select the I2C mode
//  2) SD0 PIN must be linked to VCC to select the right I2C adress
//  3) bit  b00000100 must be set on register 0x2D to read data (only once at the initialization)
//  4) bits b00001011 must be set on register 0x31 to select the data format (only once at the initialization)
// ************************************************************************************************************
#if defined(ADXL345)
void ACC_init () {
  delay(10);
  i2c_writeReg(ADXL345_ADDRESS,0x2D,1<<3); //  register: Power CTRL  -- value: Set measure bit 3 on
  i2c_writeReg(ADXL345_ADDRESS,0x31,0x0B); //  register: DATA_FORMAT -- value: Set bits 3(full range) and 1 0 on (+/- 16g-range)
  i2c_writeReg(ADXL345_ADDRESS,0x2C,0x09); //  register: BW_RATE     -- value: rate=50hz, bw=20hz
  acc_1G = 265;
}

void ACC_getADC () {
  TWBR = ((F_CPU / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz, ADXL435 is ok with this speed
  i2c_getSixRawADC(ADXL345_ADDRESS,0x32);

  ACC_ORIENTATION( ((rawADC[1]<<8) | rawADC[0]) ,
                   ((rawADC[3]<<8) | rawADC[2]) ,
                   ((rawADC[5]<<8) | rawADC[4]) );
  ACC_Common();
}
#endif

// ************************************************************************************************************
// I2C Accelerometer BMA180
// ************************************************************************************************************
// I2C adress: 0x80 (8bit)    0x40 (7bit) (SDO connection to VCC) 
// I2C adress: 0x82 (8bit)    0x41 (7bit) (SDO connection to VDDIO)
// Resolution: 14bit
//
// Control registers:
//
// 0x20    bw_tcs:      |                                           bw<3:0> |                        tcs<3:0> |
//                      |                                             150Hz |                        xxxxxxxx |
// 0x30    tco_z:       |                                                tco_z<5:0>    |     mode_config<1:0> |
//                      |                                                xxxxxxxxxx    |                   00 |
// 0x35    offset_lsb1: |          offset_x<3:0>              |                   range<2:0>       | smp_skip |
//                      |          xxxxxxxxxxxxx              |                    8G:   101       | xxxxxxxx |
// ************************************************************************************************************
#if defined(BMA180)
void ACC_init () {
  delay(10);
  //default range 2G: 1G = 4096 unit.
  i2c_writeReg(BMA180_ADDRESS,0x0D,1<<4); // register: ctrl_reg0  -- value: set bit ee_w to 1 to enable writing
  delay(5);
  uint8_t control = i2c_readReg(BMA180_ADDRESS, 0x20);
  control = control & 0x0F;        // save tcs register
  control = control | (0x01 << 4); // register: bw_tcs reg: bits 4-7 to set bw -- value: set low pass filter to 20Hz
  i2c_writeReg(BMA180_ADDRESS, 0x20, control);
  delay(5);
  control = i2c_readReg(BMA180_ADDRESS, 0x30);
  control = control & 0xFC;        // save tco_z register
  control = control | 0x00;        // set mode_config to 0
  i2c_writeReg(BMA180_ADDRESS, 0x30, control);
  delay(5); 
  control = i2c_readReg(BMA180_ADDRESS, 0x35);
  control = control & 0xF1;        // save offset_x and smp_skip register
  control = control | (0x05 << 1); // set range to 8G
  i2c_writeReg(BMA180_ADDRESS, 0x35, control);
  delay(5); 
  acc_1G = 255;
}

void ACC_getADC () {
  TWBR = ((F_CPU / 400000L) - 16) / 2;  // Optional line.  Sensor is good for it in the spec.
  i2c_getSixRawADC(BMA180_ADDRESS,0x02);
  //usefull info is on the 14 bits  [2-15] bits  /4 => [0-13] bits  /4 => 12 bit resolution
  ACC_ORIENTATION( ((rawADC[1]<<8) | rawADC[0])/16 ,
                   ((rawADC[3]<<8) | rawADC[2])/16 ,
                   ((rawADC[5]<<8) | rawADC[4])/16 );
  ACC_Common();
}
#endif

// ************************************************************************************************************
// I2C Accelerometer BMA020
// ************************************************************************************************************
// I2C adress: 0x70 (8bit)
// Resolution: 10bit
// Control registers:
//
// Datasheet: After power on reset or soft reset it is recommended to set the SPI4-bit to the correct value.
//            0x80 = SPI four-wire = Default setting
// | 0x15: | SPI4 | enable_adv_INT | new_data_INT | latch_INT | shadow_dis | wake_up_pause<1:0> | wake_up |
// |       |    1 |              0 |            0 |         0 |          0 |                 00 |       0 |
//
// | 0x14: |                       reserved <2:0> |            range <1:0> |               bandwith <2:0> |
// |       |                      !!Calibration!! |                     2g |                         25Hz |
//
// ************************************************************************************************************
#if defined(BMA020)
void ACC_init(){
  i2c_writeReg(0x38,0x15,0x80);    // set SPI4 bit
  uint8_t control = i2c_readReg(0x70, 0x14);
  control = control & 0xE0;        // save bits 7,6,5
  control = control | (0x02 << 3); // Range 8G (10)
  control = control | 0x00;        // Bandwidth 25 Hz 000
  i2c_writeReg(0x38,0x14,control); 
  acc_1G = 63;
}

void ACC_getADC(){
  TWBR = ((F_CPU / 400000L) - 16) / 2;
  i2c_getSixRawADC(0x38,0x02);
  ACC_ORIENTATION( ((rawADC[1]<<8) | rawADC[0])/64 ,
                   ((rawADC[3]<<8) | rawADC[2])/64 ,
                   ((rawADC[5]<<8) | rawADC[4])/64 );
  ACC_Common();
}
#endif

// ************************************************************************************************************
// standalone I2C Nunchuk
// ************************************************************************************************************
#if defined(NUNCHACK)
#define NUNCHACK_ADDRESS 0x52

void ACC_init() {
  i2c_writeReg(NUNCHACK_ADDRESS ,0xF0 ,0x55 );
  i2c_writeReg(NUNCHACK_ADDRESS ,0xFB ,0x00 );
  delay(250);
  acc_1G = 200;
}

void ACC_getADC() {
  TWBR = ((F_CPU / I2C_SPEED) - 16) / 2; // change the I2C clock rate. !! you must check if the nunchuk is ok with this freq
  i2c_getSixRawADC(NUNCHACK_ADDRESS,0x00);

  ACC_ORIENTATION(  ( (rawADC[3]<<2)        + ((rawADC[5]>>4)&0x2) ) ,
                  - ( (rawADC[2]<<2)        + ((rawADC[5]>>3)&0x2) ) ,
                    ( ((rawADC[4]&0xFE)<<2) + ((rawADC[5]>>5)&0x6) ));
  ACC_Common();
}
#endif

// ************************************************************************
// LIS3LV02 I2C Accelerometer
// ************************************************************************
#if defined(LIS3LV02)
#define LIS3A  0x1D

void ACC_init(){
  i2c_writeReg(LIS3A ,0x20 ,0xD7 ); // CTRL_REG1   1101 0111 Pwr on, 160Hz 
  i2c_writeReg(LIS3A ,0x21 ,0x50 ); // CTRL_REG2   0100 0000 Littl endian, 12 Bit, Boot
  acc_1G = 256;
}

void ACC_getADC(){
  TWBR = ((F_CPU / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz
  i2c_getSixRawADC(LIS3A,0x28+0x80);
  ACC_ORIENTATION( ((rawADC[1]<<8) | rawADC[0])/4 ,
                   ((rawADC[3]<<8) | rawADC[2])/4 ,
                   ((rawADC[5]<<8) | rawADC[4])/4);
  ACC_Common();
}
#endif

// ************************************************************************************************************
// I2C Accelerometer LSM303DLx
// ************************************************************************************************************
#if defined(LSM303DLx_ACC)
void ACC_init () {
  delay(10);
  i2c_writeReg(0x18,0x20,0x27);
  i2c_writeReg(0x18,0x23,0x30);
  i2c_writeReg(0x18,0x21,0x00);

  acc_1G = 256;
}

  void ACC_getADC () {
  TWBR = ((F_CPU / 400000L) - 16) / 2;
  i2c_getSixRawADC(0x18,0xA8);

  ACC_ORIENTATION( ((rawADC[1]<<8) | rawADC[0])/16 ,
                   ((rawADC[3]<<8) | rawADC[2])/16 ,
                   ((rawADC[5]<<8) | rawADC[4])/16 );
  ACC_Common();
}
#endif

// ************************************************************************************************************
// ADC ACC
// ************************************************************************************************************
#if defined(ADCACC)
void ACC_init(){
  pinMode(A1,INPUT);
  pinMode(A2,INPUT);
  pinMode(A3,INPUT);
  acc_1G = 75;
}

void ACC_getADC() {
  ACC_ORIENTATION(  analogRead(A1) ,
                    analogRead(A2) ,
                    analogRead(A3) );
  ACC_Common();
}
#endif

// ************************************************************************************************************
// I2C Gyroscope L3G4200D 
// ************************************************************************************************************
#if defined(L3G4200D)
#define L3G4200D_ADDRESS 0x69
void Gyro_init() {
  delay(100);
  i2c_writeReg(L3G4200D_ADDRESS ,0x20 ,0x8F ); // CTRL_REG1   400Hz ODR, 20hz filter, run!
  delay(5);
  i2c_writeReg(L3G4200D_ADDRESS ,0x24 ,0x02 ); // CTRL_REG5   low pass filter enable
}

void Gyro_getADC () {
  TWBR = ((F_CPU / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz
  i2c_getSixRawADC(L3G4200D_ADDRESS,0x80|0x28);

  GYRO_ORIENTATION( ((rawADC[1]<<8) | rawADC[0])/20  ,
                    ((rawADC[3]<<8) | rawADC[2])/20  ,
                    ((rawADC[5]<<8) | rawADC[4])/20  );
  GYRO_Common();
}
#endif

// ************************************************************************************************************
// I2C Gyroscope ITG3200 
// ************************************************************************************************************
// I2C adress: 0xD2 (8bit)   0x69 (7bit)
// I2C adress: 0xD0 (8bit)   0x68 (7bit)
// principle:
// 1) VIO is connected to VDD
// 2) I2C adress is set to 0x69 (AD0 PIN connected to VDD)
// or 2) I2C adress is set to 0x68 (AD0 PIN connected to GND)
// 3) sample rate = 1000Hz ( 1kHz/(div+1) )
// ************************************************************************************************************
#if defined(ITG3200)
void Gyro_init() {
  delay(100);
  i2c_writeReg(ITG3200_ADDRESS, 0x3E, 0x80); //register: Power Management  --  value: reset device
//  delay(5);
//  i2c_writeReg(ITG3200_ADDRESS, 0x15, ITG3200_SMPLRT_DIV); //register: Sample Rate Divider  -- default value = 0: OK
  delay(5);
  i2c_writeReg(ITG3200_ADDRESS, 0x16, 0x18 + ITG3200_DLPF_CFG); //register: DLPF_CFG - low pass filter configuration
  delay(5);
  i2c_writeReg(ITG3200_ADDRESS, 0x3E, 0x03); //register: Power Management  --  value: PLL with Z Gyro reference
  delay(100);
}

void Gyro_getADC () {
  TWBR = ((F_CPU / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz
  i2c_getSixRawADC(ITG3200_ADDRESS,0X1D);
  GYRO_ORIENTATION( ((rawADC[0]<<8) | rawADC[1])/4 , // range: +/- 8192; +/- 2000 deg/sec
                    ((rawADC[2]<<8) | rawADC[3])/4 ,
                    ((rawADC[4]<<8) | rawADC[5])/4 );
  GYRO_Common();
}
#endif


// ************************************************************************************************************
// I2C Compass common function
// ************************************************************************************************************
#if MAG
static float   magCal[3] = {1.0,1.0,1.0};  // gain for each axis, populated at sensor init
static uint8_t magInit = 0;

void Mag_getADC() {
  static uint32_t t,tCal = 0;
  static int16_t magZeroTempMin[3];
  static int16_t magZeroTempMax[3];
  uint8_t axis;
  if ( currentTime < t ) return; //each read is spaced by 100ms
  t = currentTime + 100000;
  TWBR = ((F_CPU / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz
  Device_Mag_getADC();
  magADC[ROLL]  = magADC[ROLL]  * magCal[ROLL];
  magADC[PITCH] = magADC[PITCH] * magCal[PITCH];
  magADC[YAW]   = magADC[YAW]   * magCal[YAW];
  if (f.CALIBRATE_MAG) {
    tCal = t;
    for(axis=0;axis<3;axis++) {
      global_conf.magZero[axis] = 0;
      magZeroTempMin[axis] = magADC[axis];
      magZeroTempMax[axis] = magADC[axis];
    }
    f.CALIBRATE_MAG = 0;
  }
  if (magInit) { // we apply offset only once mag calibration is done
    magADC[ROLL]  -= global_conf.magZero[ROLL];
    magADC[PITCH] -= global_conf.magZero[PITCH];
    magADC[YAW]   -= global_conf.magZero[YAW];
  }
 
  if (tCal != 0) {
    if ((t - tCal) < 30000000) { // 30s: you have 30s to turn the multi in all directions
      LEDPIN_TOGGLE;
      for(axis=0;axis<3;axis++) {
        if (magADC[axis] < magZeroTempMin[axis]) magZeroTempMin[axis] = magADC[axis];
        if (magADC[axis] > magZeroTempMax[axis]) magZeroTempMax[axis] = magADC[axis];
      }
    } else {
      tCal = 0;
      for(axis=0;axis<3;axis++)
        global_conf.magZero[axis] = (magZeroTempMin[axis] + magZeroTempMax[axis])/2;
      writeGlobalSet(1);
    }
  } else {
    #if defined(SENSORS_TILT_45DEG_LEFT)
      int16_t temp = ((magADC[PITCH] - magADC[ROLL] )*7)/10;
      magADC[ROLL] = ((magADC[ROLL]  + magADC[PITCH])*7)/10;
      magADC[PITCH] = temp;
    #endif
    #if defined(SENSORS_TILT_45DEG_RIGHT)
      int16_t temp = ((magADC[PITCH] + magADC[ROLL] )*7)/10;
      magADC[ROLL] = ((magADC[ROLL]  - magADC[PITCH])*7)/10;
      magADC[PITCH] = temp;
    #endif
  }
}
#endif

// ************************************************************************************************************
// I2C Compass MAG3110
// ************************************************************************************************************
// I2C adress: 0x0E (7bit)
// ************************************************************************************************************
#if defined(MAG3110)
  #define MAG_ADDRESS 0x0E
  #define MAG_DATA_REGISTER 0x01
  #define MAG_CTRL_REG1 0x10
  #define MAG_CTRL_REG2 0x11
  
  void Mag_init() {
    delay(100);
    i2c_writeReg(MAG_ADDRESS,MAG_CTRL_REG2,0x80);  //Automatic Magnetic Sensor Reset
    delay(100);
    i2c_writeReg(MAG_ADDRESS,MAG_CTRL_REG1,0x11); // DR = 20Hz ; OS ratio = 64 ; mode = Active
    delay(100);
    magInit = 1;
  }
  
  #if not defined(MPU6050_I2C_AUX_MASTER)
    void Device_Mag_getADC() {
      i2c_getSixRawADC(MAG_ADDRESS,MAG_DATA_REGISTER);
      MAG_ORIENTATION( ((rawADC[0]<<8) | rawADC[1]) ,          
                       ((rawADC[2]<<8) | rawADC[3]) ,     
                       ((rawADC[4]<<8) | rawADC[5]) );
    }
  #endif
#endif

// ************************************************************************************************************
// I2C Compass HMC5843 & HMC5883
// ************************************************************************************************************
// I2C adress: 0x3C (8bit)   0x1E (7bit)
// ************************************************************************************************************
#if defined(HMC5843) || defined(HMC5883)
  #define MAG_ADDRESS 0x1E
  #define MAG_DATA_REGISTER 0x03
  
  void Mag_init() { 
    delay(100);
    // force positiveBias
    i2c_writeReg(MAG_ADDRESS ,0x00 ,0x71 ); //Configuration Register A  -- 0 11 100 01  num samples: 8 ; output rate: 15Hz ; positive bias
    delay(50);
    // set gains for calibration
    i2c_writeReg(MAG_ADDRESS ,0x01 ,0x60 ); //Configuration Register B  -- 011 00000    configuration gain 2.5Ga
    i2c_writeReg(MAG_ADDRESS ,0x02 ,0x01 ); //Mode register             -- 000000 01    single Conversion Mode

    // read values from the compass -  self test operation
    // by placing the mode register into single-measurement mode (0x01), two data acquisition cycles will be made on each magnetic vector.
    // The first acquisition values will be subtracted from the second acquisition, and the net measurement will be placed into the data output registers
    delay(100);
      getADC();
    delay(10);
    #if defined(HMC5883)
      magCal[ROLL]  =  1160.0 / abs(magADC[ROLL]);
      magCal[PITCH] =  1160.0 / abs(magADC[PITCH]);
      magCal[YAW]   =  1080.0 / abs(magADC[YAW]);
    #else
      magCal[ROLL]  =  1000.0 / abs(magADC[ROLL]);
      magCal[PITCH] =  1000.0 / abs(magADC[PITCH]);
      magCal[YAW]   =  1000.0 / abs(magADC[YAW]);
    #endif

    // leave test mode
    i2c_writeReg(MAG_ADDRESS ,0x00 ,0x70 ); //Configuration Register A  -- 0 11 100 00  num samples: 8 ; output rate: 15Hz ; normal measurement mode
    i2c_writeReg(MAG_ADDRESS ,0x01 ,0x20 ); //Configuration Register B  -- 001 00000    configuration gain 1.3Ga
    i2c_writeReg(MAG_ADDRESS ,0x02 ,0x00 ); //Mode register             -- 000000 00    continuous Conversion Mode

    magInit = 1;
  }

void getADC() {
  i2c_getSixRawADC(MAG_ADDRESS,MAG_DATA_REGISTER);
  #if defined(HMC5843)
    MAG_ORIENTATION( ((rawADC[0]<<8) | rawADC[1]) ,
                     ((rawADC[2]<<8) | rawADC[3]) ,
                     ((rawADC[4]<<8) | rawADC[5]) );
  #endif
  #if defined (HMC5883)  
    MAG_ORIENTATION( ((rawADC[0]<<8) | rawADC[1]) ,
                     ((rawADC[4]<<8) | rawADC[5]) ,
                     ((rawADC[2]<<8) | rawADC[3]) );
  #endif
}

#if not defined(MPU6050_I2C_AUX_MASTER)
void Device_Mag_getADC() {
  getADC();
}
#endif
#endif

// ************************************************************************************************************
// I2C Compass AK8975
// ************************************************************************************************************
// I2C adress: 0x0C (7bit)
// ************************************************************************************************************
#if defined(AK8975)
  #define MAG_ADDRESS 0x0C
  #define MAG_DATA_REGISTER 0x03
  
  void Mag_init() {
    delay(100);
    i2c_writeReg(MAG_ADDRESS,0x0a,0x01);  //Start the first conversion
    delay(100);
    magInit = 1;
  }

  void Device_Mag_getADC() {
    i2c_getSixRawADC(MAG_ADDRESS,MAG_DATA_REGISTER);
    MAG_ORIENTATION( ((rawADC[1]<<8) | rawADC[0]) ,          
                     ((rawADC[3]<<8) | rawADC[2]) ,     
                     ((rawADC[5]<<8) | rawADC[4]) );
    //Start another meassurement
    i2c_writeReg(MAG_ADDRESS,0x0a,0x01);
  }
#endif

// ************************************************************************************************************
// I2C Gyroscope and Accelerometer MPU6050
// ************************************************************************************************************
#if defined(MPU6050)

void Gyro_init() {
  TWBR = ((F_CPU / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz
  i2c_writeReg(MPU6050_ADDRESS, 0x6B, 0x80);             //PWR_MGMT_1    -- DEVICE_RESET 1
  delay(5);
  i2c_writeReg(MPU6050_ADDRESS, 0x6B, 0x03);             //PWR_MGMT_1    -- SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 3 (PLL with Z Gyro reference)
  i2c_writeReg(MPU6050_ADDRESS, 0x1A, MPU6050_DLPF_CFG); //CONFIG        -- EXT_SYNC_SET 0 (disable input pin for data sync) ; default DLPF_CFG = 0 => ACC bandwidth = 260Hz  GYRO bandwidth = 256Hz)
  i2c_writeReg(MPU6050_ADDRESS, 0x1B, 0x18);             //GYRO_CONFIG   -- FS_SEL = 3: Full scale set to 2000 deg/sec
  // enable I2C bypass for AUX I2C
  #if defined(MAG)
    i2c_writeReg(MPU6050_ADDRESS, 0x37, 0x02);           //INT_PIN_CFG   -- INT_LEVEL=0 ; INT_OPEN=0 ; LATCH_INT_EN=0 ; INT_RD_CLEAR=0 ; FSYNC_INT_LEVEL=0 ; FSYNC_INT_EN=0 ; I2C_BYPASS_EN=1 ; CLKOUT_EN=0
  #endif
}

void Gyro_getADC () {
  i2c_getSixRawADC(MPU6050_ADDRESS, 0x43);
  GYRO_ORIENTATION( ((rawADC[0]<<8) | rawADC[1])/4 , // range: +/- 8192; +/- 2000 deg/sec
                    ((rawADC[2]<<8) | rawADC[3])/4 ,
                    ((rawADC[4]<<8) | rawADC[5])/4 );
  GYRO_Common();
}

void ACC_init () {
  i2c_writeReg(MPU6050_ADDRESS, 0x1C, 0x10);             //ACCEL_CONFIG  -- AFS_SEL=2 (Full Scale = +/-8G)  ; ACCELL_HPF=0   //note something is wrong in the spec.
  //note: something seems to be wrong in the spec here. With AFS=2 1G = 4096 but according to my measurement: 1G=2048 (and 2048/8 = 256)
  //confirmed here: http://www.multiwii.com/forum/viewtopic.php?f=8&t=1080&start=10#p7480
  #if defined(FREEIMUv04)
    acc_1G = 255;
  #else
    acc_1G = 512;
  #endif

  #if defined(MPU6050_I2C_AUX_MASTER)
    //at this stage, the MAG is configured via the original MAG init function in I2C bypass mode
    //now we configure MPU as a I2C Master device to handle the MAG via the I2C AUX port (done here for HMC5883)
    i2c_writeReg(MPU6050_ADDRESS, 0x6A, 0b00100000);       //USER_CTRL     -- DMP_EN=0 ; FIFO_EN=0 ; I2C_MST_EN=1 (I2C master mode) ; I2C_IF_DIS=0 ; FIFO_RESET=0 ; I2C_MST_RESET=0 ; SIG_COND_RESET=0
    i2c_writeReg(MPU6050_ADDRESS, 0x37, 0x00);             //INT_PIN_CFG   -- INT_LEVEL=0 ; INT_OPEN=0 ; LATCH_INT_EN=0 ; INT_RD_CLEAR=0 ; FSYNC_INT_LEVEL=0 ; FSYNC_INT_EN=0 ; I2C_BYPASS_EN=0 ; CLKOUT_EN=0
    i2c_writeReg(MPU6050_ADDRESS, 0x24, 0x0D);             //I2C_MST_CTRL  -- MULT_MST_EN=0 ; WAIT_FOR_ES=0 ; SLV_3_FIFO_EN=0 ; I2C_MST_P_NSR=0 ; I2C_MST_CLK=13 (I2C slave speed bus = 400kHz)
    i2c_writeReg(MPU6050_ADDRESS, 0x25, 0x80|MAG_ADDRESS);//I2C_SLV0_ADDR -- I2C_SLV4_RW=1 (read operation) ; I2C_SLV4_ADDR=MAG_ADDRESS
    i2c_writeReg(MPU6050_ADDRESS, 0x26, MAG_DATA_REGISTER);//I2C_SLV0_REG  -- 6 data bytes of MAG are stored in 6 registers. First register address is MAG_DATA_REGISTER
    i2c_writeReg(MPU6050_ADDRESS, 0x27, 0x86);             //I2C_SLV0_CTRL -- I2C_SLV0_EN=1 ; I2C_SLV0_BYTE_SW=0 ; I2C_SLV0_REG_DIS=0 ; I2C_SLV0_GRP=0 ; I2C_SLV0_LEN=3 (3x2 bytes)
  #endif
}

void ACC_getADC () {
  i2c_getSixRawADC(MPU6050_ADDRESS, 0x3B);
  ACC_ORIENTATION( ((rawADC[0]<<8) | rawADC[1])/8 ,
                   ((rawADC[2]<<8) | rawADC[3])/8 ,
                   ((rawADC[4]<<8) | rawADC[5])/8 );
  ACC_Common();
}

//The MAG acquisition function must be replaced because we now talk to the MPU device
  #if defined(MPU6050_I2C_AUX_MASTER)
    void Device_Mag_getADC() {
      i2c_getSixRawADC(MPU6050_ADDRESS, 0x49);               //0x49 is the first memory room for EXT_SENS_DATA
      #if defined(HMC5843)
        MAG_ORIENTATION( ((rawADC[0]<<8) | rawADC[1]) ,
                         ((rawADC[2]<<8) | rawADC[3]) ,
                         ((rawADC[4]<<8) | rawADC[5]) );
      #endif
      #if defined (HMC5883)  
        MAG_ORIENTATION( ((rawADC[0]<<8) | rawADC[1]) ,
                         ((rawADC[4]<<8) | rawADC[5]) ,
                         ((rawADC[2]<<8) | rawADC[3]) );
      #endif
      #if defined (MAG3110)
        MAG_ORIENTATION( ((rawADC[0]<<8) | rawADC[1]) ,          
                         ((rawADC[2]<<8) | rawADC[3]) ,     
                         ((rawADC[4]<<8) | rawADC[5]) );
      #endif
    }
  #endif
#endif

// ************************************************************************************************************
// I2C Gyroscope MPU3050
// ************************************************************************************************************
#if defined(MPU3050)

void Gyro_init() {
  TWBR = ((F_CPU / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz
  i2c_writeReg(MPU3050_ADDRESS, 0x3E, 0x80);             //PWR_MGMT_1    -- DEVICE_RESET 1
  delay(5);
  i2c_writeReg(MPU3050_ADDRESS, 0x3E, 0x03);             //PWR_MGMT_1    -- SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 3 (PLL with Z Gyro reference)
  i2c_writeReg(MPU3050_ADDRESS, 0x16, MPU3050_DLPF_CFG + 0x18); // Gyro CONFIG   -- EXT_SYNC_SET 0 (disable input pin for data sync) ; default DLPF_CFG = 0 => GYRO bandwidth = 256Hz); -- FS_SEL = 3: Full scale set to 2000 deg/sec
}

void Gyro_getADC () {
  i2c_getSixRawADC(MPU3050_ADDRESS, 0x1D);
  GYRO_ORIENTATION( ((rawADC[0]<<8) | rawADC[1])/4 , // range: +/- 8192; +/- 2000 deg/sec
                    ((rawADC[2]<<8) | rawADC[3])/4 ,
                    ((rawADC[4]<<8) | rawADC[5])/4 );
  GYRO_Common();
}

#endif


#if defined(WMP) || defined(NUNCHUCK)
// ************************************************************************************************************
// I2C Wii Motion Plus + optional Nunchuk
// ************************************************************************************************************
// I2C adress 1: 0x53 (7bit)
// I2C adress 2: 0x52 (7bit)
// ************************************************************************************************************
#define WMP_ADDRESS_1 0x53
#define WMP_ADDRESS_2 0x52

void Gyro_init() {
  delay(250);
  i2c_writeReg(WMP_ADDRESS_1, 0xF0, 0x55); // Initialize Extension
  delay(250);
  i2c_writeReg(WMP_ADDRESS_1, 0xFE, 0x05); // Activate Nunchuck pass-through mode
  delay(250);
}

void Gyro_getADC() {
  uint8_t axis;
  TWBR = ((F_CPU / I2C_SPEED) - 16) / 2; // change the I2C clock rate
  i2c_getSixRawADC(WMP_ADDRESS_2,0x00);

  if (micros() < (neutralizeTime + NEUTRALIZE_DELAY)) {//we neutralize data in case of blocking+hard reset state
    for (axis = 0; axis < 3; axis++) {gyroADC[axis]=0;accADC[axis]=0;}
    accADC[YAW] = acc_1G;
    f.NUNCHUKDATA = 0;
  } 

  // Wii Motion Plus Data
  if ( (rawADC[5]&0x03) == 0x02 ) {
    // Assemble 14bit data 
    gyroADC[ROLL]  = - ( ((rawADC[5]>>2)<<8) | rawADC[2] ); //range: +/- 8192
    gyroADC[PITCH] = - ( ((rawADC[4]>>2)<<8) | rawADC[1] );
    gyroADC[YAW]  =  - ( ((rawADC[3]>>2)<<8) | rawADC[0] );
    GYRO_Common();
    // Check if slow bit is set and normalize to fast mode range
    gyroADC[ROLL]  = (rawADC[3]&0x01)     ? gyroADC[ROLL]/5  : gyroADC[ROLL];  //the ratio 1/5 is not exactly the IDG600 or ISZ650 specification 
    gyroADC[PITCH] = (rawADC[4]&0x02)>>1  ? gyroADC[PITCH]/5 : gyroADC[PITCH]; //we detect here the slow of fast mode WMP gyros values (see wiibrew for more details)
    gyroADC[YAW]   = (rawADC[3]&0x02)>>1  ? gyroADC[YAW]/5   : gyroADC[YAW];   // this step must be done after zero compensation    
    f.NUNCHUKDATA = 0;
  #if defined(NUNCHUCK)
    } else if ( (rawADC[5]&0x03) == 0x00 ) { // Nunchuk Data
      ACC_ORIENTATION(  ( (rawADC[3]<<2)      | ((rawADC[5]>>4)&0x02) ) ,
                      - ( (rawADC[2]<<2)      | ((rawADC[5]>>3)&0x02) ) ,
                        ( ((rawADC[4]>>1)<<3) | ((rawADC[5]>>5)&0x06) ) );
      ACC_Common();
      f.NUNCHUKDATA = 1;
  #endif
  }
}

#if defined(NUNCHUCK)
  void ACC_init () {
    // We need to set acc_1G for the Nunchuk beforehand
    // If a different accelerometer is used, it will be overwritten by its ACC_init() later.
    acc_1G = 200;
  }
  void ACC_getADC () { // it's done ine the WMP gyro part
    Gyro_getADC();
  }
#endif

#endif

#if defined(TINY_GPS) | defined(TINY_GPS_SONAR)
void tinygps_query(void) {
  struct nav_data_t navi;
  int16_t i2c_errors = i2c_errors_count;
  /* copy GPS data to local struct */
  i2c_read_to_buf(TINY_GPS_TWI_ADD, &navi, sizeof(navi));
  /* did we generate any errors? */
  if (i2c_errors == i2c_errors_count) {
    #if defined(TINY_GPS)
    GPS_numSat = navi.gps.sats;
    f.GPS_FIX = (navi.gps.quality > 0);
    GPS_coord[LAT] = (navi.gps.flags & 1<<NMEA_RMC_FLAGS_LAT_NORTH ? 1 : -1) * GPS_coord_to_decimal(&navi.gps.lat);
    GPS_coord[LON] = (navi.gps.flags & 1<<NMEA_RMC_FLAGS_LON_EAST ? 1 : -1) * GPS_coord_to_decimal(&navi.gps.lon);
    GPS_altitude = navi.gps.alt.m;
    #endif

    #if defined(TINY_GPS_SONAR)
    sonarAlt = navi.sonar.distance;
    #endif
  }
}
#endif

// ************************************************************************************************************
// I2C Sonar SRF08
// ************************************************************************************************************
// first contribution from guru_florida (02-25-2012)
//
// specs are here: http://www.meas-spec.com/downloads/MS5611-01BA03.pdf
// useful info on pages 7 -> 12
#if defined(SRF02) || defined(SRF08) || defined(SRF10) || defined(SRC235)

// the default address for any new sensor found on the bus
// the code will move new sonars to the next available sonar address in range of F0-FE so that another
// sonar sensor can be added again.
// Thus, add only 1 sonar sensor at a time, poweroff, then wire the next, power on, wait for flashing light and repeat
#if !defined(SRF08_DEFAULT_ADDRESS) 
  #define SRF08_DEFAULT_ADDRESS 0x70
#endif

#if !defined(SRF08_RANGE_WAIT) 
  #define SRF08_RANGE_WAIT     80000      // delay between Ping and Range Read commands
#endif

#if !defined(SRF08_RANGE_SLEEP) 
  #define SRF08_RANGE_SLEEP    35000      // sleep this long before starting another Ping
#endif

#if !defined(SRF08_SENSOR_FIRST) 
  #define SRF08_SENSOR_FIRST    0xF0    // the first sensor i2c address (after it has been moved)
#endif

#if !defined(SRF08_MAX_SENSORS) 
  #define SRF08_MAX_SENSORS    4        // maximum number of sensors we'll allow (can go up to 8)
#endif

#define SONAR_MULTICAST_PING

// registers of the device
#define SRF08_REV_COMMAND    0
#define SRF08_LIGHT_GAIN     1
#define SRF08_ECHO_RANGE     2


static struct {
  // sensor registers from the MS561101BA datasheet
  int32_t  range[SRF08_MAX_SENSORS];
  int8_t   sensors;              // the number of sensors present
  int8_t   current;              // the current sensor being read
  uint8_t  state;
  uint32_t deadline;
} srf08_ctx;


// read uncompensated temperature value: send command first
void Sonar_init() {
  memset(&srf08_ctx, 0, sizeof(srf08_ctx));
  srf08_ctx.deadline = 4000000;
}

// this function works like readReg accept a failed read is a normal expectation
// use for testing the existence of sensors on the i2c bus
// a 0xffff code is returned if the read failed
uint16_t i2c_try_readReg(uint8_t add, uint8_t reg) {
  uint16_t count = 255;
  i2c_rep_start(add<<1);  // I2C write direction
  i2c_write(reg);        // register selection
  i2c_rep_start((add<<1)|1);  // I2C read direction
  
  TWCR = (1<<TWINT) | (1<<TWEN);
  while (!(TWCR & (1<<TWINT))) {
    count--;
    if (count==0) {              //we are in a blocking state => we don't insist
      TWCR = 0;                  //and we force a reset on TWINT register
      return 0xffff;  // return failure to read
    }
  }
  
  uint8_t r = TWDR;
  i2c_stop();
  return r;  
}

// read a 16bit unsigned int from the i2c bus
uint16_t i2c_readReg16(int8_t addr, int8_t reg) {
  uint8_t b[2];
  i2c_read_reg_to_buf(addr, reg, &b, sizeof(b));
  return (b[0]<<8) | b[1];
}

void i2c_srf08_change_addr(int8_t current, int8_t moveto) {
  // to change a srf08 address, we must write the following sequence to the command register
  // this sequence must occur as 4 seperate i2c transactions!!
  //   A0 AA A5 [addr]
  i2c_writeReg(current, SRF08_REV_COMMAND, 0xA0);  delay(30);
  i2c_writeReg(current, SRF08_REV_COMMAND, 0xAA);  delay(30);
  i2c_writeReg(current, SRF08_REV_COMMAND, 0xA5);  delay(30);
  i2c_writeReg(current, SRF08_REV_COMMAND, moveto);  delay(30); // now change i2c address
  blinkLED(5,1,2);
  #if defined(BUZZER)
   notification_confirmation = 2;
  #endif
}

// discover previously known sensors and any new sensor (move new sensors to assigned area)
void i2c_srf08_discover() {
  uint8_t addr;
  uint16_t x;

  // determine how many sensors are plugged in
  srf08_ctx.sensors=0;
  addr = SRF08_SENSOR_FIRST;
  for(int i=0; i<SRF08_MAX_SENSORS && x!=0xff; i++) {
    // read the revision as a way to check if sensor exists at this location
    x = i2c_try_readReg(addr, SRF08_REV_COMMAND);
    if(x!=0xffff) {
      // detected a sensor at this address
      srf08_ctx.sensors++;
      addr += 2;
    }
  }
  
  // do not add sensors if we are already maxed
  if(srf08_ctx.sensors < SRF08_MAX_SENSORS) {
    // now determine if any sensor is on the 'new sensor' address (srf08 default address)
    // we try to read the revision number
    x = i2c_try_readReg(SRF08_DEFAULT_ADDRESS, SRF08_REV_COMMAND);
    if(x!=0xffff) {
      // new sensor detected at SRF08 default address
      i2c_srf08_change_addr(SRF08_DEFAULT_ADDRESS, addr);  // move sensor to the next address
      srf08_ctx.sensors++;
    }
  }
}

void Sonar_update() {
  if (currentTime < srf08_ctx.deadline || (srf08_ctx.state==0 && f.ARMED)) return; 
  srf08_ctx.deadline = currentTime;
  TWBR = ((F_CPU / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz, SRF08 is ok with this speed
  switch (srf08_ctx.state) {
    case 0: 
      i2c_srf08_discover();
      if(srf08_ctx.sensors>0)
        srf08_ctx.state++; 
      else
        srf08_ctx.deadline += 5000000; // wait 5 secs before trying search again
      break;
    case 1: 
      srf08_ctx.current=0;
      srf08_ctx.state++;
      srf08_ctx.deadline += SRF08_RANGE_SLEEP;
      break;
#if defined(SONAR_MULTICAST_PING)
    case 2:
      // send a ping via the general broadcast address
      i2c_writeReg(0, SRF08_REV_COMMAND, 0x51);  // start ranging, result in centimeters
      srf08_ctx.state++;
      srf08_ctx.deadline += SRF08_RANGE_WAIT;
      break;
    case 3: 
      srf08_ctx.range[srf08_ctx.current] = i2c_readReg16( SRF08_SENSOR_FIRST+(srf08_ctx.current<<1), SRF08_ECHO_RANGE);
      srf08_ctx.current++;
      if(srf08_ctx.current >= srf08_ctx.sensors)
        srf08_ctx.state=1;
      break;
#else
    case 2:
      // send a ping to the current sensor
      i2c_writeReg(SRF08_SENSOR_FIRST+(srf08_ctx.current<<1), SRF08_REV_COMMAND, 0x51);  // start ranging, result in centimeters
      srf08_ctx.state++;
      srf08_ctx.deadline += SRF08_RANGE_WAIT;
      break;
    case 3: 
      srf08_ctx.range[srf08_ctx.current] = i2c_readReg16(SRF08_SENSOR_FIRST+(srf08_ctx.current<<1), SRF08_ECHO_RANGE);
      srf08_ctx.current++;
      if(srf08_ctx.current >= srf08_ctx.sensors)
        srf08_ctx.state=1;
      else
        srf08_ctx.state=2; 
      break;
#endif
  } 
sonarAlt = srf08_ctx.range[0]; //tmp
}
#elif defined(TINY_GPS_SONAR)
inline void Sonar_init() {}
void Sonar_update() {
  /* do not query the module again if the GPS loop already did */
  #if defined(TINY_GPS)
    if (!GPS_Enable) {
  #else
    {
  #endif
      tinygps_query();
    }
}
#else
inline void Sonar_init() {}
inline void Sonar_update() {}
#endif



void initSensors() {
  delay(200);
  POWERPIN_ON;
  delay(100);
  i2c_init();
  delay(100);
  if (GYRO) Gyro_init();
  if (BARO) Baro_init();
  if (MAG) Mag_init();
  if (ACC) {ACC_init();acc_25deg = acc_1G * 0.423;}
  if (SONAR) Sonar_init();
  f.I2C_INIT_DONE = 1;
}
