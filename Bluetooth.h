
#ifndef Bluetooth_H_
#define Bluetooth_H_

#include "config.h"

#if defined(Bluetooth)

// The sizeof this struct should not exceed 32 bytes
struct bluetoothData {
  byte throttle;
  byte yaw;
  byte pitch;
  byte roll;
  byte AUX1;
  byte AUX2;
  byte switches;
};

struct bluetoothAckPayload {
  float lat;
  float lon;
  int16_t heading;
  int16_t pitch;
  int16_t roll;  
  int32_t alt;
  byte flags;
};

extern bluetoothData bluetoothData;
extern bluetoothAckPayload bluetoothAckPayload;
extern int16_t bluetooth_rcData[RC_CHANS];

void bluetooth_Init();
void bluetooth_Read_RC();

#endif

#endif /* Bluetooth_H_ */
