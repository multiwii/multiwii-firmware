#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "EEPROM.h"
#include "LCD.h"
#include "Output.h"
#include "GPS.h"
#include "MultiWii.h"
#include "Serial.h"
#include "Protocol.h"
#include "RX.h"

/************************************** MultiWii Serial Protocol *******************************************************/
// Multiwii Serial Protocol 0 
#define MSP_VERSION              0

//to multiwii developpers/committers : do not add new MSP messages without a proper argumentation/agreement on the forum
//range id [50-99] won't be assigned and can therefore be used for any custom multiwii fork without further MSP id conflict

#define MSP_PRIVATE              1     //in+out message      to be used for a generic framework : MSP + function code (LIST/GET/SET) + data. no code yet

#define MSP_IDENT                100   //out message         multitype + multiwii version + protocol version + capability variable
#define MSP_STATUS               101   //out message         cycletime & errors_count & sensor present & box activation & current setting number
#define MSP_RAW_IMU              102   //out message         9 DOF
#define MSP_SERVO                103   //out message         8 servos
#define MSP_MOTOR                104   //out message         8 motors
#define MSP_RC                   105   //out message         8 rc chan and more
#define MSP_RAW_GPS              106   //out message         fix, numsat, lat, lon, alt, speed, ground course
#define MSP_COMP_GPS             107   //out message         distance home, direction home
#define MSP_ATTITUDE             108   //out message         2 angles 1 heading
#define MSP_ALTITUDE             109   //out message         altitude, variometer
#define MSP_ANALOG               110   //out message         vbat, powermetersum, rssi if available on RX
#define MSP_RC_TUNING            111   //out message         rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_PID                  112   //out message         P I D coeff (9 are used currently)
#define MSP_BOX                  113   //out message         BOX setup (number is dependant of your setup)
#define MSP_MISC                 114   //out message         powermeter trig
#define MSP_MOTOR_PINS           115   //out message         which pins are in use for motors & servos, for GUI 
#define MSP_BOXNAMES             116   //out message         the aux switch names
#define MSP_PIDNAMES             117   //out message         the PID names
#define MSP_WP                   118   //out message         get a WP, WP# is in the payload, returns (WP#, lat, lon, alt, flags) WP#0-home, WP#16-poshold
#define MSP_BOXIDS               119   //out message         get the permanent IDs associated to BOXes
#define MSP_SERVO_CONF           120   //out message         Servo settings
#define MSP_ACC_TRIM             121   //out message         get acc angle trim values

#define MSP_SET_RAW_RC           200   //in message          8 rc chan
#define MSP_SET_RAW_GPS          201   //in message          fix, numsat, lat, lon, alt, speed
#define MSP_SET_PID              202   //in message          P I D coeff (9 are used currently)
#define MSP_SET_BOX              203   //in message          BOX setup (number is dependant of your setup)
#define MSP_SET_RC_TUNING        204   //in message          rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_ACC_CALIBRATION      205   //in message          no param
#define MSP_MAG_CALIBRATION      206   //in message          no param
#define MSP_SET_MISC             207   //in message          powermeter trig + 8 free for future use
#define MSP_RESET_CONF           208   //in message          no param
#define MSP_SET_WP               209   //in message          sets a given WP (WP#,lat, lon, alt, flags)
#define MSP_SELECT_SETTING       210   //in message          Select Setting Number (0-2)
#define MSP_SET_HEAD             211   //in message          define a new heading hold direction
#define MSP_SET_SERVO_CONF       212   //in message          Servo settings
#define MSP_SET_MOTOR            214   //in message          PropBalance function
#define MSP_SET_ACC_TRIM         215   //in message          set acc angle trim values

#define MSP_BIND                 240   //in message          no param

#define MSP_EEPROM_WRITE         250   //in message          no param

#define MSP_DEBUGMSG             253   //out message         debug string buffer
#define MSP_DEBUG                254   //out message         debug1,debug2,debug3,debug4

#ifdef DEBUGMSG
  #define DEBUG_MSG_BUFFER_SIZE 128
  static char debug_buf[DEBUG_MSG_BUFFER_SIZE];
  static uint8_t head_debug;
  static uint8_t tail_debug;
  static uint8_t debugmsg_available();
  static void debugmsg_serialize(uint8_t l);
#endif

static uint8_t CURRENTPORT=0;

#define INBUF_SIZE 64
static uint8_t inBuf[INBUF_SIZE][UART_NUMBER];
static uint8_t checksum[UART_NUMBER];
static uint8_t indRX[UART_NUMBER];
static uint8_t cmdMSP[UART_NUMBER];

void evaluateOtherData(uint8_t sr);
#ifndef SUPPRESS_ALL_SERIAL_MSP
void evaluateCommand();
#endif

#define BIND_CAPABLE 0;  //Used for Spektrum today; can be used in the future for any RX type that needs a bind and has a MultiWii module. 
#if defined(SPEK_BIND)
  #define BIND_CAPABLE 1;
#endif
// Capability is bit flags; next defines should be 2, 4, 8...

const uint32_t capability = 0+BIND_CAPABLE;

uint8_t read8()  {
  return inBuf[indRX[CURRENTPORT]++][CURRENTPORT]&0xff;
}
uint16_t read16() {
  uint16_t t = read8();
  t+= (uint16_t)read8()<<8;
  return t;
}
uint32_t read32() {
  uint32_t t = read16();
  t+= (uint32_t)read16()<<16;
  return t;
}

void serialize8(uint8_t a) {
  SerialSerialize(CURRENTPORT,a);
  checksum[CURRENTPORT] ^= a;
}
void serialize16(int16_t a) {
  serialize8((a   ) & 0xFF);
  serialize8((a>>8) & 0xFF);
}
void serialize32(uint32_t a) {
  serialize8((a    ) & 0xFF);
  serialize8((a>> 8) & 0xFF);
  serialize8((a>>16) & 0xFF);
  serialize8((a>>24) & 0xFF);
}

void headSerialResponse(uint8_t err, uint8_t s) {
  serialize8('$');
  serialize8('M');
  serialize8(err ? '!' : '>');
  checksum[CURRENTPORT] = 0; // start calculating a new checksum
  serialize8(s);
  serialize8(cmdMSP[CURRENTPORT]);
}

void headSerialReply(uint8_t s) {
  headSerialResponse(0, s);
}

void inline headSerialError(uint8_t s) {
  headSerialResponse(1, s);
}

void tailSerialReply() {
  serialize8(checksum[CURRENTPORT]);UartSendData(CURRENTPORT);
}

void serializeNames(PGM_P s) {
  headSerialReply(strlen_P(s));
  for (PGM_P c = s; pgm_read_byte(c); c++) {
    serialize8(pgm_read_byte(c));
  }
}

void serialCom() {
  uint8_t c,n;  
  static uint8_t offset[UART_NUMBER];
  static uint8_t dataSize[UART_NUMBER];
  static enum _serial_state {
    IDLE,
    HEADER_START,
    HEADER_M,
    HEADER_ARROW,
    HEADER_SIZE,
    HEADER_CMD,
  } c_state[UART_NUMBER];// = IDLE;

  for(n=0;n<UART_NUMBER;n++) {
    #if !defined(PROMINI)
      CURRENTPORT=n;
    #endif
    #define GPS_COND
    #if defined(GPS_SERIAL)
      #if defined(GPS_PROMINI)
        #define GPS_COND       
      #else
        #undef GPS_COND
        #define GPS_COND  && (GPS_SERIAL != CURRENTPORT)
      #endif      
    #endif
    #define RX_COND
    #if (defined(SPEKTRUM) || defined(SBUS)) && (UART_NUMBER > 1)
      #define RX_COND && (RX_SERIAL_PORT != CURRENTPORT)
    #endif
    uint8_t cc = SerialAvailable(CURRENTPORT);
    while (cc-- GPS_COND RX_COND) {
      uint8_t bytesTXBuff = SerialUsedTXBuff(CURRENTPORT); // indicates the number of occupied bytes in TX buffer
      if (bytesTXBuff > TX_BUFFER_SIZE - 50 ) return; // ensure there is enough free TX buffer to go further (50 bytes margin)
      c = SerialRead(CURRENTPORT);
      #ifdef SUPPRESS_ALL_SERIAL_MSP
        // no MSP handling, so go directly
        evaluateOtherData(c);
      #else
        // regular data handling to detect and handle MSP and other data
        if (c_state[CURRENTPORT] == IDLE) {
          c_state[CURRENTPORT] = (c=='$') ? HEADER_START : IDLE;
          if (c_state[CURRENTPORT] == IDLE) evaluateOtherData(c); // evaluate all other incoming serial data
        } else if (c_state[CURRENTPORT] == HEADER_START) {
          c_state[CURRENTPORT] = (c=='M') ? HEADER_M : IDLE;
        } else if (c_state[CURRENTPORT] == HEADER_M) {
          c_state[CURRENTPORT] = (c=='<') ? HEADER_ARROW : IDLE;
        } else if (c_state[CURRENTPORT] == HEADER_ARROW) {
          if (c > INBUF_SIZE) {  // now we are expecting the payload size
            c_state[CURRENTPORT] = IDLE;
            continue;
          }
          dataSize[CURRENTPORT] = c;
          offset[CURRENTPORT] = 0;
          checksum[CURRENTPORT] = 0;
          indRX[CURRENTPORT] = 0;
          checksum[CURRENTPORT] ^= c;
          c_state[CURRENTPORT] = HEADER_SIZE;  // the command is to follow
        } else if (c_state[CURRENTPORT] == HEADER_SIZE) {
          cmdMSP[CURRENTPORT] = c;
          checksum[CURRENTPORT] ^= c;
          c_state[CURRENTPORT] = HEADER_CMD;
        } else if (c_state[CURRENTPORT] == HEADER_CMD && offset[CURRENTPORT] < dataSize[CURRENTPORT]) {
          checksum[CURRENTPORT] ^= c;
          inBuf[offset[CURRENTPORT]++][CURRENTPORT] = c;
        } else if (c_state[CURRENTPORT] == HEADER_CMD && offset[CURRENTPORT] >= dataSize[CURRENTPORT]) {
          if (checksum[CURRENTPORT] == c) {  // compare calculated and transferred checksum
            evaluateCommand();  // we got a valid packet, evaluate it
          }
          c_state[CURRENTPORT] = IDLE;
          cc = 0; // no more than one MSP per port and per cycle
        }
      #endif // SUPPRESS_ALL_SERIAL_MSP
    }
  }
}

void  s_struct(uint8_t *cb,uint8_t siz) {
  headSerialReply(siz);
  while(siz--) serialize8(*cb++);
}

void s_struct_w(uint8_t *cb,uint8_t siz) {
 headSerialReply(0);
  while(siz--) *cb++ = read8();
}

#ifndef SUPPRESS_ALL_SERIAL_MSP
void evaluateCommand() {
  uint32_t tmp=0; 

  switch(cmdMSP[CURRENTPORT]) {
   case MSP_PRIVATE:
     headSerialError(0); // we don't have any custom msp currently, so tell the gui we do not use that
     break;
   case MSP_SET_RAW_RC:
     s_struct_w((uint8_t*)&rcSerial,16);
     rcSerialCount = 50; // 1s transition 
     break;
   #if GPS
   case MSP_SET_RAW_GPS:
     f.GPS_FIX = read8();
     GPS_numSat = read8();
     GPS_coord[LAT] = read32();
     GPS_coord[LON] = read32();
     GPS_altitude = read16();
     GPS_speed = read16();
     GPS_update |= 2;              // New data signalisation to GPS functions
     headSerialReply(0);
     break;
   #endif
   case MSP_SET_PID:
     s_struct_w((uint8_t*)&conf.pid[0].P8,3*PIDITEMS);
     break;
   case MSP_SET_BOX:
     s_struct_w((uint8_t*)&conf.activate[0],CHECKBOXITEMS*2);
     break;
   case MSP_SET_RC_TUNING:
     s_struct_w((uint8_t*)&conf.rcRate8,7);
     break;
   #if !defined(DISABLE_SETTINGS_TAB)
   case MSP_SET_MISC:
     struct {
       uint16_t a,b,c,d,e,f;
       uint32_t g;
       uint16_t h;
       uint8_t  i,j,k,l;
     } set_misc;
     s_struct_w((uint8_t*)&set_misc,22);
     #if defined(POWERMETER)
       conf.powerTrigger1 = set_misc.a / PLEVELSCALE;
     #endif
     conf.minthrottle = set_misc.b;
     #ifdef FAILSAFE 
       conf.failsafe_throttle = set_misc.e;
     #endif  
     #if MAG
       conf.mag_declination = set_misc.h;
     #endif
     #if defined(VBAT)
       conf.vbatscale        = set_misc.i;
       conf.vbatlevel_warn1  = set_misc.j;
       conf.vbatlevel_warn2  = set_misc.k;
       conf.vbatlevel_crit   = set_misc.l;
     #endif
     break;
   case MSP_MISC:
     struct {
       uint16_t a,b,c,d,e,f;
       uint32_t g;
       uint16_t h;
       uint8_t  i,j,k,l;
     } misc;
     misc.a = intPowerTrigger1;
     misc.b = conf.minthrottle;
     misc.c = MAXTHROTTLE;
     misc.d = MINCOMMAND;
     #ifdef FAILSAFE 
       misc.e = conf.failsafe_throttle;
     #else  
       misc.e = 0;
     #endif
     #ifdef LOG_PERMANENT
       misc.f = plog.arm;
       misc.g = plog.lifetime + (plog.armed_time / 1000000); // <- computation must be moved outside from serial
     #else
       misc.f = 0; misc.g =0;
     #endif
     #if MAG
       misc.h = conf.mag_declination;
     #else
       misc.h = 0;
     #endif
     #ifdef VBAT
       misc.i = conf.vbatscale;
       misc.j = conf.vbatlevel_warn1;
       misc.k = conf.vbatlevel_warn2;
       misc.l = conf.vbatlevel_crit;
     #else
       misc.i = 0;misc.j = 0;misc.k = 0;misc.l = 0;
     #endif
     s_struct((uint8_t*)&misc,22);
     break;
   #endif
   #if defined (DYNBALANCE)
     case MSP_SET_MOTOR:
       s_struct_w((uint8_t*)&motor,16);
     break;
   #endif
   #ifdef MULTIPLE_CONFIGURATION_PROFILES
   case MSP_SELECT_SETTING:
     if(!f.ARMED) {
       global_conf.currentSet = read8();
       if(global_conf.currentSet>2) global_conf.currentSet = 0;
       writeGlobalSet(0);
       readEEPROM();
     }
     headSerialReply(0);
     break;
   #endif
   case MSP_SET_HEAD:
     s_struct_w((uint8_t*)&magHold,2);
     break;
   case MSP_IDENT:
     struct {
       uint8_t v,t,msp_v;
       uint32_t cap;
     } id;
     id.v     = VERSION;
     id.t     = MULTITYPE;
     id.msp_v = MSP_VERSION;
     id.cap   = capability|DYNBAL<<2|FLAP<<3;
     s_struct((uint8_t*)&id,7);
     break;
   case MSP_STATUS:
     struct {
       uint16_t cycleTime,i2c_errors_count,sensor;
       uint32_t flag;
       uint8_t set;
     } st;
     st.cycleTime        = cycleTime;
     st.i2c_errors_count = i2c_errors_count;
     st.sensor           = ACC|BARO<<1|MAG<<2|GPS<<3|SONAR<<4;
     #if ACC
       if(f.ANGLE_MODE)   tmp |= 1<<BOXANGLE;
       if(f.HORIZON_MODE) tmp |= 1<<BOXHORIZON;
     #endif
     #if BARO && (!defined(SUPPRESS_BARO_ALTHOLD))
       if(f.BARO_MODE) tmp |= 1<<BOXBARO;
     #endif
     #if MAG
       if(f.MAG_MODE) tmp |= 1<<BOXMAG;
       #if !defined(FIXEDWING)
         if(f.HEADFREE_MODE)       tmp |= 1<<BOXHEADFREE;
         if(rcOptions[BOXHEADADJ]) tmp |= 1<<BOXHEADADJ;
       #endif
     #endif
     #if defined(SERVO_TILT) || defined(GIMBAL)|| defined(SERVO_MIX_TILT)
       if(rcOptions[BOXCAMSTAB]) tmp |= 1<<BOXCAMSTAB;
     #endif
     #if defined(CAMTRIG)
       if(rcOptions[BOXCAMTRIG]) tmp |= 1<<BOXCAMTRIG;
     #endif
     #if GPS
       if(f.GPS_HOME_MODE) tmp |= 1<<BOXGPSHOME; 
       if(f.GPS_HOLD_MODE) tmp |= 1<<BOXGPSHOLD;
     #endif
     #if defined(FIXEDWING) || defined(HELICOPTER)
       if(f.PASSTHRU_MODE) tmp |= 1<<BOXPASSTHRU;
     #endif
     #if defined(BUZZER)
       if(rcOptions[BOXBEEPERON]) tmp |= 1<<BOXBEEPERON;
     #endif
     #if defined(LED_FLASHER)
       if(rcOptions[BOXLEDMAX]) tmp |= 1<<BOXLEDMAX;
       if(rcOptions[BOXLEDLOW]) tmp |= 1<<BOXLEDLOW;
     #endif
     #if defined(LANDING_LIGHTS_DDR)
       if(rcOptions[BOXLLIGHTS]) tmp |= 1<<BOXLLIGHTS;
     #endif
     #if defined(VARIOMETER)
       if(rcOptions[BOXVARIO]) tmp |= 1<<BOXVARIO;
     #endif
     #if defined(INFLIGHT_ACC_CALIBRATION)
       if(rcOptions[BOXCALIB]) tmp |= 1<<BOXCALIB;
     #endif
     #if defined(GOVERNOR_P)
       if(rcOptions[BOXGOV]) tmp |= 1<<BOXGOV;
     #endif
     #if defined(OSD_SWITCH)
       if(rcOptions[BOXOSD]) tmp |= 1<<BOXOSD;
     #endif
     if(f.ARMED) tmp |= 1<<BOXARM;
     st.flag             = tmp;
     st.set              = global_conf.currentSet;
     s_struct((uint8_t*)&st,11);
     break;
   case MSP_RAW_IMU:
     #if defined(DYNBALANCE)
       for(uint8_t axis=0;axis<3;axis++) {imu.gyroData[axis]=imu.gyroADC[axis];imu.accSmooth[axis]= imu.accADC[axis];} // Send the unfiltered Gyro & Acc values to gui.
     #endif 
     s_struct((uint8_t*)&imu,18);
     break;
   case MSP_SERVO:
     s_struct((uint8_t*)&servo,16);
     break;
   case MSP_SERVO_CONF:
     s_struct((uint8_t*)&conf.servoConf[0].min,56); // struct servo_conf_ is 7 bytes length: min:2 / max:2 / middle:2 / rate:1    ----     8 servo =>  8x7 = 56
     break;
   case MSP_SET_SERVO_CONF:
     s_struct_w((uint8_t*)&conf.servoConf[0].min,56);
     break;
   case MSP_MOTOR:
     s_struct((uint8_t*)&motor,16);
     break;
   case MSP_ACC_TRIM:
     s_struct((uint8_t*)&conf.angleTrim[0],4);
     break;
   case MSP_SET_ACC_TRIM:
     s_struct_w((uint8_t*)&conf.angleTrim[0],4);
     break;
   case MSP_RC:
     s_struct((uint8_t*)&rcData,RC_CHANS*2);
     break;
   #if GPS
   case MSP_RAW_GPS:
     headSerialReply(16);
     serialize8(f.GPS_FIX);
     serialize8(GPS_numSat);
     serialize32(GPS_coord[LAT]);
     serialize32(GPS_coord[LON]);
     serialize16(GPS_altitude);
     serialize16(GPS_speed);
     serialize16(GPS_ground_course);        // added since r1172
     break;
   case MSP_COMP_GPS:
     headSerialReply(5);
     serialize16(GPS_distanceToHome);
     serialize16(GPS_directionToHome);
     serialize8(GPS_update & 1);
     break;
   #endif
   case MSP_ATTITUDE:
     s_struct((uint8_t*)&att,6);
     break;
   case MSP_ALTITUDE:
     s_struct((uint8_t*)&alt,6);
     break;
   case MSP_ANALOG:
     s_struct((uint8_t*)&analog,7);
     break;
   case MSP_RC_TUNING:
     s_struct((uint8_t*)&conf.rcRate8,7);
     break;
   case MSP_PID:
     s_struct((uint8_t*)&conf.pid[0].P8,3*PIDITEMS);
     break;
   case MSP_PIDNAMES:
     serializeNames(pidnames);
     break;
   case MSP_BOX:
     s_struct((uint8_t*)&conf.activate[0],2*CHECKBOXITEMS);
     break;
   case MSP_BOXNAMES:
     serializeNames(boxnames);
     break;
   case MSP_BOXIDS:
     headSerialReply(CHECKBOXITEMS);
     for(uint8_t i=0;i<CHECKBOXITEMS;i++) {
       serialize8(pgm_read_byte(&(boxids[i])));
     }
     break;
   case MSP_MOTOR_PINS:
     s_struct((uint8_t*)&PWM_PIN,8);
     break;
   #if defined(USE_MSP_WP)    
   case MSP_WP:
     {
       int32_t lat = 0,lon = 0;
       uint8_t wp_no = read8();        //get the wp number  
       headSerialReply(18);
       if (wp_no == 0) {
         lat = GPS_home[LAT];
         lon = GPS_home[LON];
       } else if (wp_no == 16) {
         lat = GPS_hold[LAT];
         lon = GPS_hold[LON];
       }
       serialize8(wp_no);
       serialize32(lat);
       serialize32(lon);
       serialize32(AltHold);           //altitude (cm) will come here -- temporary implementation to test feature with apps
       serialize16(0);                 //heading  will come here (deg)
       serialize16(0);                 //time to stay (ms) will come here 
       serialize8(0);                  //nav flag will come here
     }
     break;
   case MSP_SET_WP:
     {
       int32_t lat = 0,lon = 0,alt = 0;
       uint8_t wp_no = read8();        //get the wp number
       lat = read32();
       lon = read32();
       alt = read32();                 // to set altitude (cm)
       read16();                       // future: to set heading (deg)
       read16();                       // future: to set time to stay (ms)
       read8();                        // future: to set nav flag
       if (wp_no == 0) {
         GPS_home[LAT] = lat;
         GPS_home[LON] = lon;
         f.GPS_HOME_MODE = 0;          // with this flag, GPS_set_next_wp will be called in the next loop -- OK with SERIAL GPS / OK with I2C GPS
         f.GPS_FIX_HOME  = 1;
         if (alt != 0) AltHold = alt;  // temporary implementation to test feature with apps
       } else if (wp_no == 16) {       // OK with SERIAL GPS  --  NOK for I2C GPS / needs more code dev in order to inject GPS coord inside I2C GPS
         GPS_hold[LAT] = lat;
         GPS_hold[LON] = lon;
         if (alt != 0) AltHold = alt;  // temporary implementation to test feature with apps
         #if !defined(I2C_GPS)
           nav_mode      = NAV_MODE_WP;
           GPS_set_next_wp(&GPS_hold[LAT],&GPS_hold[LON]);
         #endif
       }
     }
     headSerialReply(0);
     break;
   #endif
   case MSP_RESET_CONF:
     if(!f.ARMED) LoadDefaults();
     headSerialReply(0);
     break;
   case MSP_ACC_CALIBRATION:
     if(!f.ARMED) calibratingA=512;
     headSerialReply(0);
     break;
   case MSP_MAG_CALIBRATION:
     if(!f.ARMED) f.CALIBRATE_MAG = 1;
     headSerialReply(0);
     break;
   #if defined(SPEK_BIND)
   case MSP_BIND:
     spekBind();  
     headSerialReply(0);
     break;
   #endif
   case MSP_EEPROM_WRITE:
     writeParams(0);
     headSerialReply(0);
     break;
   case MSP_DEBUG:
     s_struct((uint8_t*)&debug,8);
     break;
   #ifdef DEBUGMSG
   case MSP_DEBUGMSG:
     {
       uint8_t size = debugmsg_available();
       if (size > 16) size = 16;
       headSerialReply(size);
       debugmsg_serialize(size);
     }
     break;
   #endif
   default:  // we do not know how to handle the (valid) message, indicate error MSP $M!
     headSerialError(0);
     break;
  }
  tailSerialReply();
}
#endif // SUPPRESS_ALL_SERIAL_MSP

// evaluate all other incoming serial data
void evaluateOtherData(uint8_t sr) {
  #ifndef SUPPRESS_OTHER_SERIAL_COMMANDS
    switch (sr) {
    // Note: we may receive weird characters here which could trigger unwanted features during flight.
    //       this could lead to a crash easily.
    //       Please use if (!f.ARMED) where neccessary
      #ifdef LCD_CONF
        case 's':
        case 'S':
          if (!f.ARMED) configurationLoop();
          break;
      #endif
      #ifdef LOG_PERMANENT_SHOW_AT_L
        case 'L':
          if (!f.ARMED) dumpPLog(1);
          break;
        #endif
        #if defined(LCD_TELEMETRY) && defined(LCD_TEXTSTAR)
        case 'A': // button A press
          toggle_telemetry(1);
          break;
        case 'B': // button B press
          toggle_telemetry(2);
          break;
        case 'C': // button C press
          toggle_telemetry(3);
          break;
        case 'D': // button D press
          toggle_telemetry(4);
          break;
        case 'a': // button A release
        case 'b': // button B release
        case 'c': // button C release
        case 'd': // button D release
          break;
      #endif
      #ifdef LCD_TELEMETRY
        case '0':
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
        case '9':
      #ifndef SUPPRESS_TELEMETRY_PAGE_R
        case 'R':
      #endif
      #if defined(DEBUG) || defined(DEBUG_FREE)
        case 'F':
      #endif
          toggle_telemetry(sr);
          break;
      #endif // LCD_TELEMETRY
    }
  #endif // SUPPRESS_OTHER_SERIAL_COMMANDS
}

#ifdef DEBUGMSG
void debugmsg_append_str(const char *str) {
  while(*str) {
    debug_buf[head_debug++] = *str++;
    if (head_debug == DEBUG_MSG_BUFFER_SIZE) {
      head_debug = 0;
    }
  }
}

static uint8_t debugmsg_available() {
  if (head_debug >= tail_debug) {
    return head_debug-tail_debug;
  } else {
    return head_debug + (DEBUG_MSG_BUFFER_SIZE-tail_debug);
  }
}

static void debugmsg_serialize(uint8_t l) {
  for (uint8_t i=0; i<l; i++) {
    if (head_debug != tail_debug) {
      serialize8(debug_buf[tail_debug++]);
      if (tail_debug == DEBUG_MSG_BUFFER_SIZE) {
        tail_debug = 0;
      }
    } else {
      serialize8('\0');
    }
  }
}
#else
void debugmsg_append_str(const char *str) {};
#endif
