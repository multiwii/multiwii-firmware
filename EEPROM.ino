#include <avr/eeprom.h>


uint8_t calculate_sum(uint8_t *cb , uint8_t siz) {
  uint8_t sum=0x55;  // checksum init
  while(--siz) sum += *cb++;  // calculate checksum (without checksum byte)
  return sum;
}

void readGlobalSet() {
  eeprom_read_block((void*)&global_conf, (void*)0, sizeof(global_conf));
  if(calculate_sum((uint8_t*)&global_conf, sizeof(global_conf)) != global_conf.checksum) {
    global_conf.currentSet = 0;
    global_conf.accZero[ROLL] = 5000;    // for config error signalization
  }
}
 
bool readEEPROM() {
  uint8_t i;
  #ifdef MULTIPLE_CONFIGURATION_PROFILES
    if(global_conf.currentSet>2) global_conf.currentSet=0;
  #else
    global_conf.currentSet=0;
  #endif
  eeprom_read_block((void*)&conf, (void*)(global_conf.currentSet * sizeof(conf) + sizeof(global_conf)), sizeof(conf));
  if(calculate_sum((uint8_t*)&conf, sizeof(conf)) != conf.checksum) {
    blinkLED(6,100,3);    
    #if defined(BUZZER)
      alarmArray[7] = 3;
    #endif
    LoadDefaults();                 // force load defaults 
    return false;                   // defaults loaded, don't reload constants (EEPROM life saving)
  }
  // 500/128 = 3.90625    3.9062 * 3.9062 = 15.259   1526*100/128 = 1192
  for(i=0;i<5;i++) {
    lookupPitchRollRC[i] = (1526+conf.rcExpo8*(i*i-15))*i*(int32_t)conf.rcRate8/1192;
  }
  for(i=0;i<11;i++) {
    int16_t tmp = 10*i-conf.thrMid8;
    uint8_t y = 1;
    if (tmp>0) y = 100-conf.thrMid8;
    if (tmp<0) y = conf.thrMid8;
    lookupThrottleRC[i] = 10*conf.thrMid8 + tmp*( 100-conf.thrExpo8+(int32_t)conf.thrExpo8*(tmp*tmp)/(y*y) )/10; // [0;1000]
    lookupThrottleRC[i] = conf.minthrottle + (int32_t)(MAXTHROTTLE-conf.minthrottle)* lookupThrottleRC[i]/1000;  // [0;1000] -> [conf.minthrottle;MAXTHROTTLE]
  }

  #if defined(POWERMETER)
    pAlarm = (uint32_t) conf.powerTrigger1 * (uint32_t) PLEVELSCALE * (uint32_t) PLEVELDIV; // need to cast before multiplying
  #endif
  #if GPS
    GPS_set_pids();    // at this time we don't have info about GPS init done
  #endif
  #if defined(ARMEDTIMEWARNING)
    ArmedTimeWarningMicroSeconds = (conf.armedtimewarning *1000000);
  #endif
  return true;    // setting is OK
}

void writeGlobalSet(uint8_t b) {
  global_conf.checksum = calculate_sum((uint8_t*)&global_conf, sizeof(global_conf));
  eeprom_write_block((const void*)&global_conf, (void*)0, sizeof(global_conf));
  if (b == 1) blinkLED(15,20,1);
  #if defined(BUZZER)
    alarmArray[7] = 1; 
  #endif

}
 
void writeParams(uint8_t b) {
  #ifdef MULTIPLE_CONFIGURATION_PROFILES
    if(global_conf.currentSet>2) global_conf.currentSet=0;
  #else
    global_conf.currentSet=0;
  #endif
  conf.checksum = calculate_sum((uint8_t*)&conf, sizeof(conf));
  eeprom_write_block((const void*)&conf, (void*)(global_conf.currentSet * sizeof(conf) + sizeof(global_conf)), sizeof(conf));
  readEEPROM();
  if (b == 1) blinkLED(15,20,1);
  #if defined(BUZZER)
    alarmArray[7] = 1; //beep if loaded from gui or android
  #endif
}

void update_constants() { 
  #if defined(GYRO_SMOOTHING)
    {
      uint8_t s[3] = GYRO_SMOOTHING;
      for(uint8_t i=0;i<3;i++) conf.Smoothing[i] = s[i];
    }
  #endif
  #if defined (FAILSAFE)
    conf.failsafe_throttle = FAILSAFE_THROTTLE;
  #endif
  #ifdef VBAT
    conf.vbatscale = VBATSCALE;
    conf.vbatlevel_warn1 = VBATLEVEL_WARN1;
    conf.vbatlevel_warn2 = VBATLEVEL_WARN2;
    conf.vbatlevel_crit = VBATLEVEL_CRIT;
  #endif
  #ifdef POWERMETER
    conf.pint2ma = PINT2mA;
  #endif
  #ifdef POWERMETER_HARD
    conf.psensornull = PSENSORNULL;
  #endif
  #ifdef MMGYRO
    conf.mmgyro = MMGYRO;
  #endif
  #if defined(ARMEDTIMEWARNING)
    conf.armedtimewarning = ARMEDTIMEWARNING;
  #endif
  conf.minthrottle = MINTHROTTLE;
  #if MAG
    conf.mag_declination = (int16_t)(MAG_DECLINATION * 10);
  #endif
  #ifdef GOVERNOR_P
    conf.governorP = GOVERNOR_P;
    conf.governorD = GOVERNOR_D;
  #endif
  writeParams(0); // this will also (p)reset checkNewConf with the current version number again.
}

void LoadDefaults() {
  uint8_t i;
  #ifndef SUPPRESS_DEFAULTS_FROM_GUI
    conf.pid[ROLL].P8     = 33;  conf.pid[ROLL].I8    = 30; conf.pid[ROLL].D8     = 23;
    conf.pid[PITCH].P8    = 33; conf.pid[PITCH].I8    = 30; conf.pid[PITCH].D8    = 23;
    conf.pid[YAW].P8      = 68;  conf.pid[YAW].I8     = 45;  conf.pid[YAW].D8     = 0;
    conf.pid[PIDALT].P8   = 64; conf.pid[PIDALT].I8   = 25; conf.pid[PIDALT].D8   = 24;

    conf.pid[PIDPOS].P8  = POSHOLD_P * 100;     conf.pid[PIDPOS].I8    = POSHOLD_I * 100;       conf.pid[PIDPOS].D8    = 0;
    conf.pid[PIDPOSR].P8 = POSHOLD_RATE_P * 10; conf.pid[PIDPOSR].I8   = POSHOLD_RATE_I * 100;  conf.pid[PIDPOSR].D8   = POSHOLD_RATE_D * 1000;
    conf.pid[PIDNAVR].P8 = NAV_P * 10;          conf.pid[PIDNAVR].I8   = NAV_I * 100;           conf.pid[PIDNAVR].D8   = NAV_D * 1000;
  
    conf.pid[PIDLEVEL].P8 = 90; conf.pid[PIDLEVEL].I8 = 10; conf.pid[PIDLEVEL].D8 = 100;
    conf.pid[PIDMAG].P8   = 40;

    conf.pid[PIDVEL].P8 = 0;      conf.pid[PIDVEL].I8 = 0;    conf.pid[PIDVEL].D8 = 0;

    conf.rcRate8 = 90; conf.rcExpo8 = 65;
    conf.rollPitchRate = 0;
    conf.yawRate = 0;
    conf.dynThrPID = 0;
    conf.thrMid8 = 50; conf.thrExpo8 = 0;
    for(i=0;i<CHECKBOXITEMS;i++) {conf.activate[i] = 0;}
    conf.angleTrim[0] = 0; conf.angleTrim[1] = 0;
    conf.powerTrigger1 = 0;
  #endif
  #if defined(SERVO)
    static int8_t sr[8] = SERVO_RATES;
    #ifdef SERVO_MIN
      static int16_t smin[8] = SERVO_MIN;
      static int16_t smax[8] = SERVO_MAX;
      static int16_t smid[8] = SERVO_MID;
    #endif
    for(i=0;i<8;i++) {
      #ifdef SERVO_MIN
        conf.servoConf[i].min = smin[i];
        conf.servoConf[i].max = smax[i];
        conf.servoConf[i].middle = smid[i];
      #else
        conf.servoConf[i].min = 1020;
        conf.servoConf[i].max = 2000;
        conf.servoConf[i].middle = 1500;
      #endif
      conf.servoConf[i].rate = sr[i];
    }
  #endif
  #ifdef FIXEDWING
    conf.dynThrPID = 50;
    conf.rcExpo8   =  0;
  #endif
  update_constants();
}

#ifdef LOG_PERMANENT
void readPLog() {
  eeprom_read_block((void*)&plog, (void*)(E2END - 4 - sizeof(plog)), sizeof(plog));
  if(calculate_sum((uint8_t*)&plog, sizeof(plog)) != plog.checksum) {
    blinkLED(9,100,3);
    #if defined(BUZZER)
      alarmArray[7] = 3;
    #endif
    // force load defaults
    plog.arm = plog.disarm = plog.start = plog.failsafe = plog.i2c = 0;
    plog.running = 1;
    plog.lifetime = plog.armed_time = 0;
    writePLog();
  }
}
void writePLog() {
  plog.checksum = calculate_sum((uint8_t*)&plog, sizeof(plog));
  eeprom_write_block((const void*)&plog, (void*)(E2END - 4 - sizeof(plog)), sizeof(plog));
}
#endif

#if defined(GPS_NAV)
//Stores the WP data in the wp struct in the EEPROM
void storeWP() {
#ifdef MULTIPLE_CONFIGURATION_PROFILES
    #define PROFILES 3
#else
    #define PROFILES 1
#endif
	if (mission_step.number >254) return;
	mission_step.checksum = calculate_sum((uint8_t*)&mission_step, sizeof(mission_step));
	eeprom_write_block((void*)&mission_step, (void*)(PROFILES * sizeof(conf) + sizeof(global_conf)+(sizeof(mission_step)*mission_step.number)),sizeof(mission_step));
}

// Read the given number of WP from the eeprom, supposedly we can use this during flight.
// Returns true when reading is successfull and returns false if there were some error (for example checksum)
bool recallWP(uint8_t wp_number) {
#ifdef MULTIPLE_CONFIGURATION_PROFILES
    #define PROFILES 3
#else
    #define PROFILES 1
#endif
	if (wp_number > 254) return false;

	eeprom_read_block((void*)&mission_step, (void*)(PROFILES * sizeof(conf) + sizeof(global_conf)+(sizeof(mission_step)*wp_number)), sizeof(mission_step));
	if(calculate_sum((uint8_t*)&mission_step, sizeof(mission_step)) != mission_step.checksum) return false;

	return true;
}

// Returns the maximum WP number that can be stored in the EEPROM, calculated from conf and plog sizes, and the eeprom size
uint8_t getMaxWPNumber() {
#ifdef LOG_PERMANENT
    #define PLOG_SIZE sizeof(plog)
#else 
    #define PLOG_SIZE 0
#endif
#ifdef MULTIPLE_CONFIGURATION_PROFILES
    #define PROFILES 3
#else
	#define PROFILES 1
#endif

	uint16_t first_avail = PROFILES*sizeof(conf) + sizeof(global_conf)+ 1; //Add one byte for addtnl separation
	uint16_t last_avail  = E2END - PLOG_SIZE - 4;										  //keep the last 4 bytes intakt
	uint16_t wp_num = (last_avail-first_avail)/sizeof(mission_step);
	if (wp_num>254) wp_num = 254;
	return wp_num;
}

#endif
