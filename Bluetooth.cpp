#define REMOTEXY_MODE__SOFTSERIAL

#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "MultiWii.h"

#include <SoftwareSerial.h> 
#include <RemoteXY.h> 

// RemoteXY connection settings  
#define REMOTEXY_SERIAL_RX 8 
#define REMOTEXY_SERIAL_TX 7 
#define REMOTEXY_SERIAL_SPEED 9600

#if defined(Bluetooth)

#pragma pack(push, 1) 
uint8_t RemoteXY_CONF[] = 
  { 255,7,0,1,0,80,0,8,162,0,
  5,37,68,12,42,42,38,26,31,5,
  2,-11,13,41,41,64,26,16,2,1,
  52,4,22,11,38,26,31,31,79,78,
  0,79,70,70,0,66,0,42,18,13,
  28,246,26,2,1,22,4,22,11,2,
  26,31,31,79,78,0,79,70,70,0,
  2,0,38,51,21,7,121,26,31,31,
  79,78,0,79,70,70,0 };

struct { 

    // input variable
  int8_t right_x; // =-100..100 x-coordinate joystick position 
  int8_t right_y; // =-100..100 y-coordinate joystick position 
  int8_t left_x; // =-100..100 x-coordinate joystick position 
  int8_t left_y; // =-100..100 y-coordinate joystick position 
  uint8_t aux2; // =1 if switch ON and =0 if OFF 
  uint8_t aux1; // =1 if switch ON and =0 if OFF 
  uint8_t aux3; // =1 if switch ON and =0 if OFF
   
    // output variable
  int8_t height; // =0..100 level position 

    // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0 

} RemoteXY; 
#pragma pack(pop)   


int16_t bluetooth_rcData[RC_CHANS];// defined in RX.cpp


void resetbluetoothData() 
{
  RemoteXY.left_y = -100;
  RemoteXY.left_x = 0;
  RemoteXY.right_y = 0;
  RemoteXY.right_x = 0;

  RemoteXY.aux1 = 0;
  RemoteXY.aux2 = 0;
  RemoteXY.aux3 = 0; 
}


void bluetooth_Init() { //setup

  RemoteXY_Init (); 
  resetbluetoothData();
  
}

void bluetooth_Read_RC() { //loop
  static unsigned long lastRecvTime = 0;
  RemoteXY_Handler ();
   //unsigned long now = millis();
 // if ( now - lastRecvTime > 1000 ) {
    // signal lost?
   // resetbluetoothData();
//  }
   
  bluetooth_rcData[THROTTLE] =  map(RemoteXY.left_y,-100,100, 1000, 2000); //If your channels are inverted, reverse the map value. Example. From 1000 to 2000 ---> 2000 to 1000
  bluetooth_rcData[ROLL] =      map(RemoteXY.right_x,-100,100, 1000, 2000);
  bluetooth_rcData[PITCH] =     map(RemoteXY.right_y,-100,100, 1000, 2000);
  bluetooth_rcData[YAW] =       map(RemoteXY.left_x,-100,100, 1000, 2000);

  bluetooth_rcData[AUX1] =       map(RemoteXY.aux1,     0, 1, 1000, 2000);
  bluetooth_rcData[AUX2] =       map(RemoteXY.aux2,     0, 1, 1000, 2000);
  bluetooth_rcData[AUX3] =       map(RemoteXY.aux3,     0, 1, 1000, 2000);
}

#endif
