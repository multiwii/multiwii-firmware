#include <avr/eeprom.h>

static uint8_t checkNewConf = 153;

struct eep_entry_t{
  void *  var;
  uint8_t size;
};

// ************************************************************************************************************
// EEPROM Layout definition
// ************************************************************************************************************
static eep_entry_t eep_entry[] = {
  {&checkNewConf, sizeof(checkNewConf)}
, {&P8, sizeof(P8)}
, {&I8, sizeof(I8)} 
, {&D8, sizeof(D8)} 
, {&rcRate8, sizeof(rcRate8)}
, {&rcExpo8, sizeof(rcExpo8)}
, {&rollPitchRate, sizeof(rollPitchRate)}
, {&yawRate, sizeof(yawRate)}
, {&dynThrPID, sizeof(dynThrPID)}
, {&accZero, sizeof(accZero)}
, {&magZero, sizeof(magZero)}
, {&accTrim, sizeof(accTrim)}
, {&activate, sizeof(activate)}
, {&powerTrigger1, sizeof(powerTrigger1)}
#ifdef FLYING_WING
, {&wing_left_mid,  sizeof(wing_left_mid)}
, {&wing_right_mid, sizeof(wing_right_mid)}
#endif
#ifdef TRI
, {&tri_yaw_middle,  sizeof(tri_yaw_middle)}
#endif

};  
#define EEBLOCK_SIZE sizeof(eep_entry)/sizeof(eep_entry_t)
// ************************************************************************************************************

void readEEPROM() {
  uint8_t i, _address = eep_entry[0].size;
  for(i=1; i<EEBLOCK_SIZE; i++) {
    eeprom_read_block(eep_entry[i].var, (void*)(_address), eep_entry[i].size); _address += eep_entry[i].size;
  }  
  #if defined(POWERMETER)
    pAlarm = (uint32_t) powerTrigger1 * (uint32_t) PLEVELSCALE * (uint32_t) PLEVELDIV; // need to cast before multiplying
  #endif
  for(i=0;i<7;i++) lookupRX[i] = (2500+rcExpo8*(i*i-25))*i*(int32_t)rcRate8/2500;
  #ifdef FLYING_WING
    wing_left_mid  = constrain(wing_left_mid, WING_LEFT_MIN,  WING_LEFT_MAX); //LEFT 
    wing_right_mid = constrain(wing_right_mid, WING_RIGHT_MIN, WING_RIGHT_MAX); //RIGHT
  #endif
  #ifdef TRI
    tri_yaw_middle = constrain(tri_yaw_middle, TRI_YAW_CONSTRAINT_MIN, TRI_YAW_CONSTRAINT_MAX); //REAR
  #endif
}

void writeParams(uint8_t b) {
  uint8_t i, _address = 0;
  for(i=0; i<EEBLOCK_SIZE; i++) {
    eeprom_write_block(eep_entry[i].var, (void*)(_address), eep_entry[i].size); _address += eep_entry[i].size;
  }  
  readEEPROM();
  if (b == 1) blinkLED(15,20,1);
}

void checkFirstTime() {
  uint8_t test_val; eeprom_read_block((void*)&test_val, (void*)(0), sizeof(test_val));
  if (test_val == checkNewConf) return;
  P8[ROLL] = 40; I8[ROLL] = 30; D8[ROLL] = 23;
  P8[PITCH] = 40; I8[PITCH] = 30; D8[PITCH] = 23;
  P8[YAW]  = 85; I8[YAW]  = 0;  D8[YAW]  = 0;
  P8[PIDALT]   = 16; I8[PIDALT]   = 15;  D8[PIDALT]   = 7;
  P8[PIDGPS]   = 50; I8[PIDGPS]   = 0;  D8[PIDGPS]   = 15;
  P8[PIDVEL]   =  0; I8[PIDVEL]   = 0;  D8[PIDVEL]   = 0;
  P8[PIDLEVEL] = 90; I8[PIDLEVEL] = 45; D8[PIDLEVEL] = 100;
  P8[PIDMAG] = 40;
  rcRate8 = 90;
  rcExpo8 = 65;
  rollPitchRate = 0;
  yawRate = 0;
  dynThrPID = 0;
  for(uint8_t i=0;i<CHECKBOXITEMS;i++) {activate[i] = 0;}
  accTrim[0] = 0; accTrim[1] = 0;
  powerTrigger1 = 0;
  #ifdef FLYING_WING
    wing_left_mid  = WING_LEFT_MID; 
    wing_right_mid = WING_RIGHT_MID; 
  #endif
  #ifdef TRI
    tri_yaw_middle = TRI_YAW_MIDDLE; 
  #endif
  writeParams(0);
}

