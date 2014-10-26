#ifndef SENSORS_H_
#define SENSORS_H_

void ACC_getADC ();
void Gyro_getADC ();
uint8_t Mag_getADC();
uint8_t Baro_update();
void Sonar_update();

void initSensors();
void i2c_rep_start(uint8_t address);
void i2c_write(uint8_t data );
void i2c_stop(void);
void i2c_write(uint8_t data );
void i2c_writeReg(uint8_t add, uint8_t reg, uint8_t val);
uint8_t i2c_readReg(uint8_t add, uint8_t reg);
uint8_t i2c_readAck();
uint8_t i2c_readNak();

void i2c_read_reg_to_buf(uint8_t add, uint8_t reg, uint8_t *buf, uint8_t size);

#if defined(MMA7455)
  #define ACC_1G 64
#endif
#if defined(MMA8451Q)
  #define ACC_1G 512
#endif
#if defined(ADXL345)
  #define ACC_1G 265
#endif
#if defined(BMA180) || defined(BMA280) || defined(LIS3LV02) || defined(LSM303DLx_ACC) || defined(LSM330)
  #define ACC_1G 255
#endif
#if defined(BMA020)
  #define ACC_1G 63
#endif
#if defined(ADCACC)
  #define ACC_1G 75
#endif
#if defined(MPU6050)
  #if defined(FREEIMUv04)
    #define ACC_1G 255
  #else
    #define ACC_1G 512
  #endif
#endif
#if !defined(ACC_1G)
  #define ACC_1G 256
#endif
#define ACCZ_25deg   (int16_t)(ACC_1G * 0.90631) // 0.90631 = cos(25deg) (cos(theta) of accZ comparison)
#define ACC_VelScale (9.80665f / 10000.0f / ACC_1G)

// GYRO SCALE: we ignore the last 2 bits and convert it for rad/s
#if defined(ITG3050) || defined(MPU6050) || defined(MPU3050)
  #define GYRO_SCALE (4 / 16.4 * PI / 180.0 / 1000000.0) //16.4 LSB = 1 deg/s
#endif
#if defined(ITG3200)
  #define GYRO_SCALE (4 / 14.375 * PI / 180.0 / 1000000.0) //ITG3200   14.375 LSB = 1 deg/s
#endif
#if defined(L3G4200D) || defined(LSM330)
  #define GYRO_SCALE ((4.0f * PI * 70.0f)/(1000.0f * 180.0f * 1000000.0f)) // 70 milli deg/s /digit => 1 deg/s = 1000/70 LSB
#endif
#if defined(WMP)
  #define GYRO_SCALE (1.0f/200e6f)
#endif

#endif /* SENSORS_H_ */
