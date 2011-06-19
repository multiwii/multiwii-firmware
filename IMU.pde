void computeIMU () {
  uint8_t axis;
  static int16_t gyroADCprevious[3] = {0,0,0};
  static int16_t gyroADCp[3] = {0,0,0};
  int16_t gyroADCinter[3];
  static int16_t lastAccADC[3] = {0,0,0};
  static uint32_t timeInterleave = 0;
  static int16_t gyroYawSmooth = 0;

  //we separate the 2 situations because reading gyro values with a gyro only setup can be acchieved at a higher rate
  //gyro+nunchuk: we must wait for a quite high delay betwwen 2 reads to get both WM+ and Nunchuk data. It works with 3ms
  //gyro only: the delay to read 2 consecutive values can be reduced to only 0.65ms
  if (!ACC && nunchuk) {
    annexCode();
    while((micros()-timeInterleave)<INTERLEAVING_DELAY) ; //interleaving delay between 2 consecutive reads
    timeInterleave=micros();
    WMP_getRawADC();
    getEstimatedAttitude(); // computation time must last less than one interleaving delay
    while((micros()-timeInterleave)<INTERLEAVING_DELAY) ; //interleaving delay between 2 consecutive reads
    timeInterleave=micros();
    while(WMP_getRawADC() != 1) ; // For this interleaving reading, we must have a gyro update at this point (less delay)

    for (axis = 0; axis < 3; axis++) {
      // empirical, we take a weighted value of the current and the previous values
      // /4 is to average 4 values, note: overflow is not possible for WMP gyro here
      gyroData[axis] = (gyroADC[axis]*3+gyroADCprevious[axis]+2)/4;
      gyroADCprevious[axis] = gyroADC[axis];
    }
  } else {
    if (ACC) {
      getEstimatedAttitude();
      ACC_getADC();
    }
    if (GYRO) Gyro_getADC(); else WMP_getRawADC();
    for (axis = 0; axis < 3; axis++)
      gyroADCp[axis] =  gyroADC[axis];
    timeInterleave=micros();
    annexCode();
    while((micros()-timeInterleave)<650) ; //empirical, interleaving delay between 2 consecutive reads
    if (GYRO) Gyro_getADC(); else WMP_getRawADC();
    for (axis = 0; axis < 3; axis++) {
      gyroADCinter[axis] =  gyroADC[axis]+gyroADCp[axis];
      // empirical, we take a weighted value of the current and the previous values
      gyroData[axis] = (gyroADCinter[axis]+gyroADCprevious[axis]+1)/3;
      gyroADCprevious[axis] = gyroADCinter[axis]/2;
      if (!ACC) accADC[axis]=0;
    }
  }
  #if defined(TRI)
    gyroData[YAW] = (gyroYawSmooth*2+gyroData[YAW]+1)/3;
    gyroYawSmooth = gyroData[YAW];
  #endif
}


// **************************************************
// Simplified IMU based on "Complementary Filter"
// Inspired by http://starlino.com/imu_guide.html
//
// adapted by ziss_dm : http://wbb.multiwii.com/viewtopic.php?f=8&t=198
//
// The following ideas was used in this project:
// 1) Rotation matrix: http://en.wikipedia.org/wiki/Rotation_matrix
// 2) Small-angle approximation: http://en.wikipedia.org/wiki/Small-angle_approximation
// 3) C. Hastings approximation for atan2()
// 4) Optimization tricks: http://www.hackersdelight.org/
//
// Currently Magnetometer uses separate CF which is used only
// for heading approximation.
//
// Last Modified: 19/04/2011
// Version: V1.1   
// **************************************************

//******  advanced users settings *******************
/* Set the Low Pass Filter factor for ACC */
/* Increasing this value would reduce ACC noise (visible in GUI), but would increase ACC lag time*/
/* Comment this if  you do not want filter at all.*/
/* Default WMC value: 8*/
#define ACC_LPF_FACTOR 8

/* Set the Low Pass Filter factor for Magnetometer */
/* Increasing this value would reduce Magnetometer noise (not visible in GUI), but would increase Magnetometer lag time*/
/* Comment this if  you do not want filter at all.*/
/* Default WMC value: n/a*/
//#define MG_LPF_FACTOR 4

/* Set the Gyro Weight for Gyro/Acc complementary filter */
/* Increasing this value would reduce and delay Acc influence on the output of the filter*/
/* Default WMC value: 300*/
#define GYR_CMPF_FACTOR 310.0f

/* Set the Gyro Weight for Gyro/Magnetometer complementary filter */
/* Increasing this value would reduce and delay Magnetometer influence on the output of the filter*/
/* Default WMC value: n/a*/
#define GYR_CMPFM_FACTOR 200.0f

//****** end of advanced users settings *************

#define INV_GYR_CMPF_FACTOR   (1.0f / (GYR_CMPF_FACTOR  + 1.0f))
#define INV_GYR_CMPFM_FACTOR  (1.0f / (GYR_CMPFM_FACTOR + 1.0f))
#if GYRO
  #define GYRO_SCALE ((2000.0f * PI)/((32767.0f / 4.0f ) * 180.0f * 1000000.0f) * 1.155f)  
  // +-2000/sec deg scale
  //#define GYRO_SCALE ((200.0f * PI)/((32768.0f / 5.0f / 4.0f ) * 180.0f * 1000000.0f) * 1.5f)     
  // +- 200/sec deg scale
  // 1.5 is emperical, not sure what it means
  // should be in rad/sec
#else
  #define GYRO_SCALE (1.0f/200e6f)
  // empirical, depends on WMP on IDG datasheet, tied of deg/ms sensibility
  // !!!!should be adjusted to the rad/sec
#endif 
// Small angle approximation
#define ssin(val) (val)
#define scos(val) 1.0f

typedef struct {
  float X;
  float Y;
  float Z;
} t_fp_vector_def;

typedef union {
  float   A[3];
  t_fp_vector_def V;
} t_fp_vector;


int16_t _atan2(float y, float x){
  #define fp_is_neg(val) ((((byte*)&val)[3] & 0x80) != 0)
  float z = y / x;
  int16_t zi = abs(int16_t(z * 100)); 
  int8_t y_neg = fp_is_neg(y);
  if ( zi < 100 ){
    if (zi > 10) 
     z = z / (1.0f + 0.28f * z * z);
   if (fp_is_neg(x)) {
     if (y_neg) z -= PI;
     else z += PI;
   }
  } else {
   z = (PI / 2.0f) - z / (z * z + 0.28f);
   if (y_neg) z -= PI;
  }
  z *= (180.0f / PI * 10); 
  return z;
}


void getEstimatedAttitude(){
  uint8_t axis;
  int16_t  AccMag = 0;
  static t_fp_vector GEstG = {0,0,200};
  t_fp_vector EstG = GEstG;
  static t_fp_vector EstM = {10,10,200};
  float deltaGyroAngle;
  static uint16_t PreviousTime;
  static int16_t mgSmooth[3];  //projection of smoothed and normalized magnetic vector on x/y/z axis, as measured by magnetometer
  uint16_t CurrentTime  = micros();
  float deltaTime = (CurrentTime - PreviousTime) * GYRO_SCALE;
  PreviousTime = CurrentTime;
  // Initialization
  for (axis = 0; axis < 3; axis++) {
    #if defined(ACC_LPF_FACTOR)
      // LPF for ACC values
      accSmooth[axis] = (accSmooth[axis] * (ACC_LPF_FACTOR - 1) + accADC[axis]) / ACC_LPF_FACTOR;
      #define ACC_VALUE accSmooth[axis]
    #else  
      accSmooth[axis] = accADC[axis];
      #define ACC_VALUE accADC[axis]
    #endif
    AccMag += (ACC_VALUE * 10 / acc_1G) * (ACC_VALUE * 10 / acc_1G);
    #if MAG
      #if defined(MG_LPF_FACTOR)
        // LPF for Magnetometer values
        mgSmooth[axis] = (mgSmooth[axis] * (MG_LPF_FACTOR - 1) + magADC[axis]) / MG_LPF_FACTOR;
        #define MAG_VALUE mgSmooth[axis]
      #else  
        #define MAG_VALUE magADC[axis]
      #endif
    #endif
  }
  // Rotate Estimated vector(s), ROLL
  deltaGyroAngle  = gyroADC[ROLL] * deltaTime;
  EstG.V.Z =  scos(deltaGyroAngle) * EstG.V.Z - ssin(deltaGyroAngle) * EstG.V.X;
  EstG.V.X =  ssin(deltaGyroAngle) * EstG.V.Z + scos(deltaGyroAngle) * EstG.V.X;
  #if MAG
    EstM.V.Z =  scos(deltaGyroAngle) * EstM.V.Z - ssin(deltaGyroAngle) * EstM.V.X;
    EstM.V.X =  ssin(deltaGyroAngle) * EstM.V.Z + scos(deltaGyroAngle) * EstM.V.X;
  #endif 
  // Rotate Estimated vector(s), PITCH
  deltaGyroAngle  = gyroADC[PITCH] * deltaTime;
  EstG.V.Y =  scos(deltaGyroAngle) * EstG.V.Y + ssin(deltaGyroAngle) * EstG.V.Z;
  EstG.V.Z = -ssin(deltaGyroAngle) * EstG.V.Y + scos(deltaGyroAngle) * EstG.V.Z;
  #if MAG
    EstM.V.Y =  scos(deltaGyroAngle) * EstM.V.Y + ssin(deltaGyroAngle) * EstM.V.Z;
    EstM.V.Z = -ssin(deltaGyroAngle) * EstM.V.Y + scos(deltaGyroAngle) * EstM.V.Z;
  #endif 
  // Rotate Estimated vector(s), YAW
  deltaGyroAngle  = gyroADC[YAW] * deltaTime;
  EstG.V.X =  scos(deltaGyroAngle) * EstG.V.X - ssin(deltaGyroAngle) * EstG.V.Y;
  EstG.V.Y =  ssin(deltaGyroAngle) * EstG.V.X + scos(deltaGyroAngle) * EstG.V.Y;
  #if MAG
    EstM.V.X =  scos(deltaGyroAngle) * EstM.V.X - ssin(deltaGyroAngle) * EstM.V.Y;
    EstM.V.Y =  ssin(deltaGyroAngle) * EstM.V.X + scos(deltaGyroAngle) * EstM.V.Y;
  #endif 
  // Apply complimentary filter (Gyro drift correction)
  // If accel magnitude >1.4G or <0.6G => we neutralize the effect of accelerometers in the angle estimation.
  // To do that, we just skip filter, as EstV already rotated by Gyro
  if (!((36 > AccMag) or (AccMag > 196))) {
    for (axis = 0; axis < 3; axis++)
      EstG.A[axis] = (EstG.A[axis] * GYR_CMPF_FACTOR + ACC_VALUE) * INV_GYR_CMPF_FACTOR;
  }
  // Attitude of the estimated vector  
  angle[ROLL]  =  _atan2(EstG.V.X, EstG.V.Z) ;
  angle[PITCH] =  _atan2(EstG.V.Y, EstG.V.Z) ;
  GEstG = EstG;
  #if MAG
    // Apply complimentary filter (Gyro drift correction)
    for (axis = 0; axis < 3; axis++)
      EstM.A[axis] = (EstM.A[axis] * GYR_CMPFM_FACTOR + MAG_VALUE) * INV_GYR_CMPFM_FACTOR;
    // Attitude of the cross product vector GxM
    heading = _atan2(EstG.V.Z * EstM.V.X - EstG.V.X * EstM.V.Z, EstG.V.Y * EstM.V.Z - EstG.V.Z * EstM.V.Y) / 10;
  #endif
  #if BARO
    acc_z = (acc_z * 2 +  (accSmooth[YAW] - GEstG.V.Z) + 1) / 3;
  #endif 
}
