#if defined(MEGA)
  #define UART_NUMBER 4
#elif defined(PROMICRO)
  #define UART_NUMBER 2
#else
  #define UART_NUMBER 1
#endif
#if defined(GPS_SERIAL)
  #define RX_BUFFER_SIZE 256 // 256 RX buffer is needed for GPS communication (64 or 128 was too short)
#else
  #define RX_BUFFER_SIZE 64
#endif
#define TX_BUFFER_SIZE 128
#define INBUF_SIZE 64

static volatile uint8_t serialHeadRX[UART_NUMBER],serialTailRX[UART_NUMBER];
static uint8_t serialBufferRX[RX_BUFFER_SIZE][UART_NUMBER];
static volatile uint8_t serialHeadTX[UART_NUMBER],serialTailTX[UART_NUMBER];
static uint8_t serialBufferTX[TX_BUFFER_SIZE][UART_NUMBER];
static uint8_t inBuf[INBUF_SIZE][UART_NUMBER];

#define BIND_CAPABLE 0;  //Used for Spektrum today; can be used in the future for any RX type that needs a bind and has a MultiWii module. 
#if defined(SPEK_BIND)
  #define BIND_CAPABLE 1;
#endif
// Capability is bit flags; next defines should be 2, 4, 8...

const uint32_t capability = 0+BIND_CAPABLE;

#ifdef DEBUGMSG
  #define DEBUG_MSG_BUFFER_SIZE 128
  static char debug_buf[DEBUG_MSG_BUFFER_SIZE];
  static uint8_t head_debug;
  static uint8_t tail_debug;
#endif

// Multiwii Serial Protocol 0 
#define MSP_VERSION              0

//to multiwii developpers/committers : do not add new MSP messages without a proper argumentation/agreement on the forum
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

#define MSP_BIND                 240   //in message          no param

#define MSP_EEPROM_WRITE         250   //in message          no param

#define MSP_DEBUGMSG             253   //out message         debug string buffer
#define MSP_DEBUG                254   //out message         debug1,debug2,debug3,debug4

static uint8_t checksum[UART_NUMBER];
static uint8_t indRX[UART_NUMBER];
static uint8_t cmdMSP[UART_NUMBER];

#if defined(PROMINI)
  #define CURRENTPORT 0
#else
  static uint8_t CURRENTPORT=0;
#endif

uint32_t read32() {
  uint32_t t = read16();
  t+= (uint32_t)read16()<<16;
  return t;
}
uint16_t read16() {
  uint16_t t = read8();
  t+= (uint16_t)read8()<<8;
  return t;
}
uint8_t read8()  {
  return inBuf[indRX[CURRENTPORT]++][CURRENTPORT]&0xff;
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
  serialize8(checksum[CURRENTPORT]);UartSendData();
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
        #define GPS_COND  && (GPS_SERIAL != CURRENTPORT)
      #endif      
    #endif
    #define SPEK_COND
    #if defined(SPEKTRUM) && (UART_NUMBER > 1)
      #define SPEK_COND  && (SPEK_SERIAL_PORT != CURRENTPORT)
    #endif
    uint8_t cc = SerialAvailable(CURRENTPORT);
    while (cc-- GPS_COND SPEK_COND) {
      uint8_t bytesTXBuff = ((uint8_t)(serialHeadTX[CURRENTPORT]-serialTailTX[CURRENTPORT]))%TX_BUFFER_SIZE; // indicates the number of occupied bytes in TX buffer
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
     case MSP_SET_MOTOR: // <- must be rewritten to send 8x motors values with no toggle principle. it should be a mirror of MSP_MOTOR
       motorTogglesByte = read8();
       f.ARMED = 0;
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
     id.cap   = capability;
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
       // Send the unfiltered Gyro & Acc values to gui.
       for(uint8_t axis=0;axis<3;axis++) {imu.gyroData[axis]=imu.gyroADC[axis];imu.accSmooth[axis]= imu.accADC[axis];}
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
     s_struct((uint8_t*)&analog,5);
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
      #if defined(LOG_VALUES) || defined(DEBUG)
        case 'R':
      #endif
      #ifdef DEBUG
        case 'F':
      #endif
          toggle_telemetry(sr);
          break;
      #endif // LCD_TELEMETRY
    }
  #endif // SUPPRESS_OTHER_SERIAL_COMMANDS
}

// *******************************************************
// For Teensy 2.0, these function emulate the API used for ProMicro
// it cant have the same name as in the arduino API because it wont compile for the promini (eaven if it will be not compiled)
// *******************************************************
#if defined(TEENSY20)
  unsigned char T_USB_Available(){
    int n = Serial.available();
    if (n > 255) n = 255;
    return n;
  }
#endif

// *******************************************************
// Interrupt driven UART transmitter - using a ring buffer
// *******************************************************

void serialize32(uint32_t a) {
  serialize8((a    ) & 0xFF);
  serialize8((a>> 8) & 0xFF);
  serialize8((a>>16) & 0xFF);
  serialize8((a>>24) & 0xFF);
}

void serialize16(int16_t a) {
  serialize8((a   ) & 0xFF);
  serialize8((a>>8) & 0xFF);
}

void serialize8(uint8_t a) {
  uint8_t t = serialHeadTX[CURRENTPORT];
  if (++t >= TX_BUFFER_SIZE) t = 0;
  serialBufferTX[t][CURRENTPORT] = a;
  checksum[CURRENTPORT] ^= a;
  serialHeadTX[CURRENTPORT] = t;
}

#if defined(PROMINI) || defined(MEGA)
  #if defined(PROMINI)
  ISR(USART_UDRE_vect) {  // Serial 0 on a PROMINI
  #endif
  #if defined(MEGA)
  ISR(USART0_UDRE_vect) { // Serial 0 on a MEGA
  #endif
    uint8_t t = serialTailTX[0];
    if (serialHeadTX[0] != t) {
      if (++t >= TX_BUFFER_SIZE) t = 0;
      UDR0 = serialBufferTX[t][0];  // Transmit next byte in the ring
      serialTailTX[0] = t;
    }
    if (t == serialHeadTX[0]) UCSR0B &= ~(1<<UDRIE0); // Check if all data is transmitted . if yes disable transmitter UDRE interrupt
  }
#endif
#if defined(MEGA) || defined(PROMICRO)
  ISR(USART1_UDRE_vect) { // Serial 1 on a MEGA or on a PROMICRO
    uint8_t t = serialTailTX[1];
    if (serialHeadTX[1] != t) {
      if (++t >= TX_BUFFER_SIZE) t = 0;
      UDR1 = serialBufferTX[t][1];  // Transmit next byte in the ring
      serialTailTX[1] = t;
    }
    if (t == serialHeadTX[1]) UCSR1B &= ~(1<<UDRIE1);
  }
#endif
#if defined(MEGA)
  ISR(USART2_UDRE_vect) { // Serial 2 on a MEGA
    uint8_t t = serialTailTX[2];
    if (serialHeadTX[2] != t) {
      if (++t >= TX_BUFFER_SIZE) t = 0;
      UDR2 = serialBufferTX[t][2];
      serialTailTX[2] = t;
    }
    if (t == serialHeadTX[2]) UCSR2B &= ~(1<<UDRIE2);
  }
  ISR(USART3_UDRE_vect) { // Serial 3 on a MEGA
    uint8_t t = serialTailTX[3];
    if (serialHeadTX[3] != t) {
      if (++t >= TX_BUFFER_SIZE) t = 0;
      UDR3 = serialBufferTX[t][3];
      serialTailTX[3] = t;
    }
    if (t == serialHeadTX[3]) UCSR3B &= ~(1<<UDRIE3);
  }
#endif

void UartSendData() {
  #if defined(PROMINI)
    UCSR0B |= (1<<UDRIE0);
  #endif
  #if defined(PROMICRO)
    switch (CURRENTPORT) {
      case 0:
        while(serialHeadTX[0] != serialTailTX[0]) {
           if (++serialTailTX[0] >= TX_BUFFER_SIZE) serialTailTX[0] = 0;
           #if !defined(TEENSY20)
             USB_Send(USB_CDC_TX,serialBufferTX[serialTailTX[0]],1);
           #else
             Serial.write(serialBufferTX[serialTailTX[0]],1);
           #endif
         }
        break;
      case 1: UCSR1B |= (1<<UDRIE1); break;
    }
  #endif
  #if defined(MEGA)
    switch (CURRENTPORT) {
      case 0: UCSR0B |= (1<<UDRIE0); break;
      case 1: UCSR1B |= (1<<UDRIE1); break;
      case 2: UCSR2B |= (1<<UDRIE2); break;
      case 3: UCSR3B |= (1<<UDRIE3); break;
    }
  #endif
}

#if defined(GPS_SERIAL)
  bool SerialTXfree(uint8_t port) {
    return (serialHeadTX[port] == serialTailTX[port]);
  }
#endif

static void inline SerialOpen(uint8_t port, uint32_t baud) {
  uint8_t h = ((F_CPU  / 4 / baud -1) / 2) >> 8;
  uint8_t l = ((F_CPU  / 4 / baud -1) / 2);
  switch (port) {
    #if defined(PROMINI)
      case 0: UCSR0A  = (1<<U2X0); UBRR0H = h; UBRR0L = l; UCSR0B |= (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0); break;
    #endif
    #if defined(PROMICRO)
      #if (ARDUINO >= 100) && !defined(TEENSY20)
        case 0: UDIEN &= ~(1<<SOFE); break;// disable the USB frame interrupt of arduino (it causes strong jitter and we dont need it)
      #endif
      case 1: UCSR1A  = (1<<U2X1); UBRR1H = h; UBRR1L = l; UCSR1B |= (1<<RXEN1)|(1<<TXEN1)|(1<<RXCIE1); break;
    #endif
    #if defined(MEGA)
      case 0: UCSR0A  = (1<<U2X0); UBRR0H = h; UBRR0L = l; UCSR0B |= (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0); break;
      case 1: UCSR1A  = (1<<U2X1); UBRR1H = h; UBRR1L = l; UCSR1B |= (1<<RXEN1)|(1<<TXEN1)|(1<<RXCIE1); break;
      case 2: UCSR2A  = (1<<U2X2); UBRR2H = h; UBRR2L = l; UCSR2B |= (1<<RXEN2)|(1<<TXEN2)|(1<<RXCIE2); break;
      case 3: UCSR3A  = (1<<U2X3); UBRR3H = h; UBRR3L = l; UCSR3B |= (1<<RXEN3)|(1<<TXEN3)|(1<<RXCIE3); break;
    #endif
  }
}

static void inline SerialEnd(uint8_t port) {
  switch (port) {
    #if defined(PROMINI)
      case 0: UCSR0B &= ~((1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0)|(1<<UDRIE0)); break;
    #endif
    #if defined(PROMICRO)
      case 1: UCSR1B &= ~((1<<RXEN1)|(1<<TXEN1)|(1<<RXCIE1)|(1<<UDRIE1)); break;
    #endif
    #if defined(MEGA)
      case 0: UCSR0B &= ~((1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0)|(1<<UDRIE0)); break;
      case 1: UCSR1B &= ~((1<<RXEN1)|(1<<TXEN1)|(1<<RXCIE1)|(1<<UDRIE1)); break;
      case 2: UCSR2B &= ~((1<<RXEN2)|(1<<TXEN2)|(1<<RXCIE2)|(1<<UDRIE2)); break;
      case 3: UCSR3B &= ~((1<<RXEN3)|(1<<TXEN3)|(1<<RXCIE3)|(1<<UDRIE3)); break;
    #endif
  }
}

static void inline store_uart_in_buf(uint8_t data, uint8_t portnum) {
  #if defined(SPEKTRUM)
    if (portnum == SPEK_SERIAL_PORT) {
      if (!spekFrameFlags) { 
        sei();
        uint32_t spekTimeNow = (timer0_overflow_count << 8) * (64 / clockCyclesPerMicrosecond()); //Move timer0_overflow_count into registers so we don't touch a volatile twice
        uint32_t spekInterval = spekTimeNow - spekTimeLast;                                       //timer0_overflow_count will be slightly off because of the way the Arduino core timer interrupt handler works; that is acceptable for this use. Using the core variable avoids an expensive call to millis() or micros()
        spekTimeLast = spekTimeNow;
        if (spekInterval > 5000) {  //Potential start of a Spektrum frame, they arrive every 11 or every 22 ms. Mark it, and clear the buffer. 
          serialTailRX[portnum] = 0;
          serialHeadRX[portnum] = 0;
          spekFrameFlags = 0x01;
        }
        cli();
      }
    }
  #endif

  uint8_t h = serialHeadRX[portnum];
  if (++h >= RX_BUFFER_SIZE) h = 0;
  if (h == serialTailRX[portnum]) return; // we did not bite our own tail?
  serialBufferRX[serialHeadRX[portnum]][portnum] = data;  
  serialHeadRX[portnum] = h;
}

#if defined(PROMINI)
  ISR(USART_RX_vect)  { store_uart_in_buf(UDR0, 0); }
#endif
#if defined(PROMICRO)
  ISR(USART1_RX_vect)  { store_uart_in_buf(UDR1, 1); }
#endif
#if defined(MEGA)
  ISR(USART0_RX_vect)  { store_uart_in_buf(UDR0, 0); }
  ISR(USART1_RX_vect)  { store_uart_in_buf(UDR1, 1); }
  ISR(USART2_RX_vect)  { store_uart_in_buf(UDR2, 2); }
  ISR(USART3_RX_vect)  { store_uart_in_buf(UDR3, 3); }
#endif

uint8_t SerialRead(uint8_t port) {
  #if defined(PROMICRO)
    #if defined(TEENSY20)
      if(port == 0) return Serial.read();
    #else
      #if (ARDUINO >= 100)
        if(port == 0) USB_Flush(USB_CDC_TX);
      #endif
      if(port == 0) return USB_Recv(USB_CDC_RX);      
    #endif
  #endif
  uint8_t t = serialTailRX[port];
  uint8_t c = serialBufferRX[t][port];
  if (serialHeadRX[port] != t) {
    if (++t >= RX_BUFFER_SIZE) t = 0;
    serialTailRX[port] = t;
  }
  return c;
}

#if defined(SPEKTRUM)
  uint8_t SerialPeek(uint8_t port) {
    uint8_t c = serialBufferRX[serialTailRX[port]][port];
    if ((serialHeadRX[port] != serialTailRX[port])) return c; else return 0;
  }
#endif

uint8_t SerialAvailable(uint8_t port) {
  #if defined(PROMICRO)
    #if !defined(TEENSY20)
      if(port == 0) return USB_Available(USB_CDC_RX);
    #else
      if(port == 0) return T_USB_Available();
    #endif
  #endif
  return ((uint8_t)(serialHeadRX[port] - serialTailRX[port]))%RX_BUFFER_SIZE;
}

void SerialWrite(uint8_t port,uint8_t c){
  #if !defined(PROMINI)
    CURRENTPORT=port;
  #endif
  serialize8(c);UartSendData();
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
