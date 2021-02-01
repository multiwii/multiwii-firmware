#include <avr/eeprom.h>
#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "EEPROM.h"
#include "MultiWii.h"
#include "Alarms.h"
#include "GPS.h"

void LoadDefaults(void);

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
  int8_t tmp;
  uint8_t y;

  #ifdef MULTIPLE_CONFIGURATION_PROFILES
    if(global_conf.currentSet>2) global_conf.currentSet=0;
  #else
    global_conf.currentSet=0;
  #endif
  eeprom_read_block((void*)&conf, (void*)(global_conf.currentSet * sizeof(conf) + sizeof(global_conf)), sizeof(conf));
  if(calculate_sum((uint8_t*)&conf, sizeof(conf)) != conf.checksum) {
    blinkLED(6,100,3);    
    SET_ALARM_BUZZER(ALRM_FAC_CONFIRM, ALRM_LVL_CONFIRM_ELSE);
    LoadDefaults();                 // force load defaults 
    return false;                   // defaults loaded, don't reload constants (EEPROM life saving)
  }
  // 500/128 = 3.90625    3.9062 * 3.9062 = 15.259   1526*100/128 = 1192
  for(i=0;i<5;i++) {
    lookupPitchRollRC[i] = (1526+conf.rcExpo8*(i*i-15))*i*(int32_t)conf.rcRate8/1192;
  }
  for(i=0;i<11;i++) {
    tmp = 10*i-conf.thrMid8;
    y = conf.thrMid8;
    if (tmp>0) y = 100-y;
    lookupThrottleRC[i] = 100*conf.thrMid8 + tmp*( (int32_t)conf.thrExpo8*(tmp*tmp)/((uint16_t)y*y)+100-conf.thrExpo8 );       // [0;10000]
    lookupThrottleRC[i] = conf.minthrottle + (uint32_t)((uint16_t)(MAXTHROTTLE-conf.minthrottle))* lookupThrottleRC[i]/10000;  // [0;10000] -> [conf.minthrottle;MAXTHROTTLE]
  }
  #if defined(POWERMETER)
    pAlarm = (uint32_t) conf.powerTrigger1 * (uint32_t) PLEVELSCALE * (uint32_t) PLEVELDIV; // need to cast before multiplying
  #endif
  #if GPS
    GPS_set_pids();    // at this time we don't have info about GPS init done
    recallGPSconf();   // Load gps parameters
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
  SET_ALARM_BUZZER(ALRM_FAC_CONFIRM, ALRM_LVL_CONFIRM_1);

}
 
void writeParams(uint8_t b) {
  #ifdef MULTIPLE_CONFIGURATION_PROFILES
    if(global_conf.currentSet>2) global_conf.currentSet=0;
  #else
    global_conf.currentSet=0;
  #endif
  conf.checksum = calculate_sum((uint8_t*)&conf, sizeof(conf));
  eeprom_write_block((const void*)&conf, (void*)(global_conf.currentSet * sizeof(conf) + sizeof(global_conf)), sizeof(conf));

#if GPS
  writeGPSconf();  //Write GPS parameters
  recallGPSconf(); //Read it to ensure correct eeprom content
#endif

  readEEPROM();
  if (b == 1) blinkLED(15,20,1);
  SET_ALARM_BUZZER(ALRM_FAC_CONFIRM, ALRM_LVL_CONFIRM_1);
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
  #ifdef YAW_COLL_PRECOMP
    conf.yawCollPrecomp = YAW_COLL_PRECOMP;
    conf.yawCollPrecompDeadband = YAW_COLL_PRECOMP_DEADBAND;
  #endif
  #if defined(MY_PRIVATE_DEFAULTS)
    #include MY_PRIVATE_DEFAULTS
  #endif

#if GPS
  loadGPSdefaults();
#endif

  writeParams(0); // this will also (p)reset checkNewConf with the current version number again.
}

void LoadDefaults() {
  uint8_t i;
  #ifdef SUPPRESS_DEFAULTS_FROM_GUI
    // do nothing
  #elif defined(MY_PRIVATE_DEFAULTS)
    // #include MY_PRIVATE_DEFAULTS
    // do that at the last possible moment, so we can override virtually all defaults and constants
  #else
    #if PID_CONTROLLER == 1
      conf.pid[ROLL].P8     = 33;  conf.pid[ROLL].I8    = 30; conf.pid[ROLL].D8     = 23;
      conf.pid[PITCH].P8    = 33; conf.pid[PITCH].I8    = 30; conf.pid[PITCH].D8    = 23;
      conf.pid[PIDLEVEL].P8 = 90; conf.pid[PIDLEVEL].I8 = 10; conf.pid[PIDLEVEL].D8 = 100;
    #elif PID_CONTROLLER == 2
      conf.pid[ROLL].P8     = 28;  conf.pid[ROLL].I8    = 10; conf.pid[ROLL].D8     = 7;
      conf.pid[PITCH].P8    = 28; conf.pid[PITCH].I8    = 10; conf.pid[PITCH].D8    = 7;
      conf.pid[PIDLEVEL].P8 = 30; conf.pid[PIDLEVEL].I8 = 32; conf.pid[PIDLEVEL].D8 = 0;
    #endif
    conf.pid[YAW].P8      = 68;  conf.pid[YAW].I8     = 45;  conf.pid[YAW].D8     = 0;
    conf.pid[PIDALT].P8   = 64; conf.pid[PIDALT].I8   = 25; conf.pid[PIDALT].D8   = 24;

    conf.pid[PIDPOS].P8  = POSHOLD_P * 100;     conf.pid[PIDPOS].I8    = POSHOLD_I * 100;       conf.pid[PIDPOS].D8    = 0;
    conf.pid[PIDPOSR].P8 = POSHOLD_RATE_P * 10; conf.pid[PIDPOSR].I8   = POSHOLD_RATE_I * 100;  conf.pid[PIDPOSR].D8   = POSHOLD_RATE_D * 1000;
    conf.pid[PIDNAVR].P8 = NAV_P * 10;          conf.pid[PIDNAVR].I8   = NAV_I * 100;           conf.pid[PIDNAVR].D8   = NAV_D * 1000;
  
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
  #endif // SUPPRESS_DEFAULTS_FROM_GUI
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
  #else                   //if no servo defined then zero out the config variables to prevent passing false data to the gui.
//    for(i=0;i<8;i++) {
//        conf.servoConf[i].min = 0;
//        conf.servoConf[i].max = 0;
//        conf.servoConf[i].middle = 0;
//        conf.servoConf[i].rate = 0;
//      }
  #endif
  #ifdef FIXEDWING
    conf.dynThrPID = 50;
    conf.rcExpo8   =  0;
  #endif
  update_constants(); // this will also write to eeprom
}

#ifdef LOG_PERMANENT
void readPLog(void) {
  eeprom_read_block((void*)&plog, (void*)(E2END - 4 - sizeof(plog)), sizeof(plog));
  if(calculate_sum((uint8_t*)&plog, sizeof(plog)) != plog.checksum) {
    blinkLED(9,100,3);
    SET_ALARM_BUZZER(ALRM_FAC_CONFIRM, ALRM_LVL_CONFIRM_ELSE);
    // force load defaults
    plog.arm = plog.disarm = plog.start = plog.failsafe = plog.i2c = 0;
    plog.running = 1;
    plog.lifetime = plog.armed_time = 0;
    writePLog();
  }
}
void writePLog(void) {
  plog.checksum = calculate_sum((uint8_t*)&plog, sizeof(plog));
  eeprom_write_block((const void*)&plog, (void*)(E2END - 4 - sizeof(plog)), sizeof(plog));
}
#endif

#if GPS

//Define variables for calculations of EEPROM positions
#ifdef MULTIPLE_CONFIGURATION_PROFILES
    #define PROFILES 3
#else
    #define PROFILES 1
#endif
#ifdef LOG_PERMANENT
    #define PLOG_SIZE sizeof(plog)
#else 
    #define PLOG_SIZE 0
#endif

//Store gps_config

void writeGPSconf(void) {
  GPS_conf.checksum = calculate_sum((uint8_t*)&GPS_conf, sizeof(GPS_conf));
  eeprom_write_block( (void*)&GPS_conf, (void*) (PROFILES * sizeof(conf) + sizeof(global_conf)), sizeof(GPS_conf) );
}

//Recall gps_configuration
bool recallGPSconf(void) {
  eeprom_read_block((void*)&GPS_conf, (void*)(PROFILES * sizeof(conf) + sizeof(global_conf)), sizeof(GPS_conf));
  if(calculate_sum((uint8_t*)&GPS_conf, sizeof(GPS_conf)) != GPS_conf.checksum) {
    loadGPSdefaults();
    return false;
  }
  return true;
}

//Load gps_config_defaults and writes back to EEPROM just to make it sure
void loadGPSdefaults(void) {
  //zero out the conf struct
  uint8_t *ptr = (uint8_t *) &GPS_conf;
  for (int i=0;i<sizeof(GPS_conf);i++) *ptr++ = 0;

#if defined(GPS_FILTERING)
  GPS_conf.filtering = 1;
#endif
#if defined (GPS_LEAD_FILTER)
  GPS_conf.lead_filter = 1;
#endif
#if defined (DONT_RESET_HOME_AT_ARM)
  GPS_conf.dont_reset_home_at_arm = 1;
#endif
  GPS_conf.nav_controls_heading = NAV_CONTROLS_HEADING;
  GPS_conf.nav_tail_first       = NAV_TAIL_FIRST;
  GPS_conf.nav_rth_takeoff_heading = NAV_SET_TAKEOFF_HEADING;
  GPS_conf.slow_nav                = NAV_SLOW_NAV;
  GPS_conf.wait_for_rth_alt        = WAIT_FOR_RTH_ALT;

  GPS_conf.ignore_throttle         = IGNORE_THROTTLE;
  GPS_conf.takeover_baro           = NAV_TAKEOVER_BARO;

  GPS_conf.wp_radius               = GPS_WP_RADIUS;
  GPS_conf.safe_wp_distance        = SAFE_WP_DISTANCE;
  GPS_conf.nav_max_altitude        = MAX_NAV_ALTITUDE;
  GPS_conf.nav_speed_max           = NAV_SPEED_MAX;
  GPS_conf.nav_speed_min           = NAV_SPEED_MIN;
  GPS_conf.crosstrack_gain         = CROSSTRACK_GAIN * 100;
  GPS_conf.nav_bank_max            = NAV_BANK_MAX;
  GPS_conf.rth_altitude            = RTH_ALTITUDE;
  GPS_conf.fence                   = FENCE_DISTANCE;
  GPS_conf.land_speed              = LAND_SPEED;
  GPS_conf.max_wp_number           = getMaxWPNumber();
  writeGPSconf();
}


//Stores the WP data in the wp struct in the EEPROM
void storeWP() {
  if (mission_step.number >254) return;
  mission_step.checksum = calculate_sum((uint8_t*)&mission_step, sizeof(mission_step));
  eeprom_write_block((void*)&mission_step, (void*)(PROFILES * sizeof(conf) + sizeof(global_conf) + sizeof(GPS_conf) +(sizeof(mission_step)*mission_step.number)),sizeof(mission_step));
}

// Read the given number of WP from the eeprom, supposedly we can use this during flight.
// Returns true when reading is successfull and returns false if there were some error (for example checksum)
bool recallWP(uint8_t wp_number) {
  if (wp_number > 254) return false;
  eeprom_read_block((void*)&mission_step, (void*)(PROFILES * sizeof(conf) + sizeof(global_conf)+sizeof(GPS_conf)+(sizeof(mission_step)*wp_number)), sizeof(mission_step));
  if(calculate_sum((uint8_t*)&mission_step, sizeof(mission_step)) != mission_step.checksum) return false;
  return true;
}

// Returns the maximum WP number that can be stored in the EEPROM, calculated from conf and plog sizes, and the eeprom size
uint8_t getMaxWPNumber() {
  uint16_t first_avail = PROFILES*sizeof(conf) + sizeof(global_conf)+sizeof(GPS_conf)+ 1; //Add one byte for addtnl separation
  uint16_t last_avail  = E2END - PLOG_SIZE - 4; //keep the last 4 bytes intakt
  uint16_t wp_num = (last_avail-first_avail)/sizeof(mission_step);
  if (wp_num>254) wp_num = 254;
  return wp_num;
}
#endif
