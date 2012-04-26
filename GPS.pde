#if GPS

void GPS_NewData() {
  #if defined(I2C_GPS)
    static uint8_t _i2c_gps_status;
  
    //Do not use i2c_writereg, since writing a register does not work if an i2c_stop command is issued at the end
    //Still investigating, however with separated i2c_repstart and i2c_write commands works... and did not caused i2c errors on a long term test.
  
    GPS_numSat = (_i2c_gps_status & 0xf0) >> 4;
    _i2c_gps_status = i2c_readReg(I2C_GPS_ADDRESS,I2C_GPS_STATUS);                    //Get status register 
    if (_i2c_gps_status & I2C_GPS_STATUS_3DFIX) {                                     //Check is we have a good 3d fix (numsats>5)
       GPS_fix = 1;                                                                   //Num of sats is stored the upmost 4 bits of status
       if (!GPS_fix_home) {        //if home is not set set home position to WP#0 and activate it
          i2c_rep_start(I2C_GPS_ADDRESS);i2c_write(I2C_GPS_COMMAND);i2c_write(I2C_GPS_COMMAND_SET_WP);//Store current position to WP#0 (this is used for RTH)
          i2c_rep_start(I2C_GPS_ADDRESS);i2c_write(I2C_GPS_COMMAND);i2c_write(I2C_GPS_COMMAND_ACTIVATE_WP);//Set WP#0 as the active WP
          GPS_fix_home = 1;                                                           //Now we have a home   
       }
       if (_i2c_gps_status & I2C_GPS_STATUS_NEW_DATA) {                               //Check about new data
          if (GPS_update) { GPS_update = 0;} else { GPS_update = 1;}                  //Fancy flash on GUI :D
          //Read GPS data for distance, heading and gps position 
          i2c_rep_start(I2C_GPS_ADDRESS);
          i2c_write(I2C_GPS_DISTANCE);                                                //Start read from here 2x2 bytes distance and direction
          i2c_rep_start(I2C_GPS_ADDRESS+1);
          uint8_t *varptr = (uint8_t *)&GPS_distanceToHome;
          *varptr++ = i2c_readAck();
          *varptr   = i2c_readAck();

          varptr = (uint8_t *)&GPS_directionToHome;
          *varptr++ = i2c_readAck();
          *varptr   = i2c_readAck();

          varptr = (uint8_t *)&GPS_latitude;		// for OSD latitude displaying
          *varptr++ = i2c_readAck();
          *varptr++ = i2c_readAck();
          *varptr++ = i2c_readAck();
          *varptr   = i2c_readAck();

          varptr = (uint8_t *)&GPS_longitude;		// for OSD longitude displaying
          *varptr++ = i2c_readAck();
          *varptr++ = i2c_readAck();
          *varptr++ = i2c_readAck();
          *varptr   = i2c_readNak();

          i2c_rep_start(I2C_GPS_ADDRESS);
          i2c_write(I2C_GPS_GROUND_SPEED);          //Start read from here 2x2 bytes speed and altitude
          i2c_rep_start(I2C_GPS_ADDRESS+1);

          varptr = (uint8_t *)&GPS_speed;			// speed in cm/s for OSD
          *varptr++ = i2c_readAck();
          *varptr   = i2c_readAck();

          varptr = (uint8_t *)&GPS_altitude;       // altitude in meters for OSD
          *varptr++ = i2c_readAck();
          *varptr   = i2c_readNak();

          //GPS_ground_course
          i2c_rep_start(I2C_GPS_ADDRESS);
          i2c_write(I2C_GPS_COURSE);             //0x9C
          i2c_rep_start(I2C_GPS_ADDRESS+1);
          
          varptr = (uint8_t *)&GPS_ground_course;
          *varptr++ = i2c_readAck();
          *varptr   = i2c_readNak();
        }
    } else {                                                                          //We don't have a fix zero out distance and bearing (for safety reasons)
      GPS_distanceToHome = 0;
      GPS_directionToHome = 0;
      GPS_numSat = 0;
    }
  #endif     

  #if defined(GPS_SERIAL)
    while (SerialAvailable(GPS_SERIAL)) {
     if (GPS_newFrame(SerialRead(GPS_SERIAL))) {
        if (GPS_update == 1) GPS_update = 0; else GPS_update = 1;
        if (GPS_fix == 1 && GPS_numSat > 3) {
          if (GPS_fix_home == 0) {
            GPS_fix_home = 1;
            GPS_latitude_home  = GPS_latitude;
            GPS_longitude_home = GPS_longitude;
          }
          if (GPSModeHold == 1)
            GPS_distance(GPS_latitude_hold,GPS_longitude_hold,GPS_latitude,GPS_longitude, &GPS_distanceToHold, &GPS_directionToHold);
          else
            GPS_distance(GPS_latitude_home,GPS_longitude_home,GPS_latitude,GPS_longitude, &GPS_distanceToHome, &GPS_directionToHome);
        }
      }
    }
  #endif

  #if defined(GPS_FROM_OSD)
    if(GPS_update) {
      if (GPS_fix  && GPS_numSat > 3) {
        if (GPS_fix_home == 0) {
          GPS_fix_home = 1;
          GPS_latitude_home  = GPS_latitude;
          GPS_longitude_home = GPS_longitude;
        }
        if (GPSModeHold == 1)
          GPS_distance(GPS_latitude_hold,GPS_longitude_hold,GPS_latitude,GPS_longitude, &GPS_distanceToHold, &GPS_directionToHold);
        else
          GPS_distance(GPS_latitude_home,GPS_longitude_home,GPS_latitude,GPS_longitude, &GPS_distanceToHome, &GPS_directionToHome);
        }
        GPS_update = 0;
    }
  #endif
}

void GPS_reset_home_position() {
  #if defined(I2C_GPS)
    //set current position as home
    i2c_rep_start(I2C_GPS_ADDRESS);i2c_write(I2C_GPS_COMMAND);i2c_write(I2C_GPS_COMMAND_SET_WP);//Store current position to WP#0 (this is used for RTH)
    i2c_rep_start(I2C_GPS_ADDRESS);i2c_write(I2C_GPS_COMMAND);i2c_write(I2C_GPS_COMMAND_ACTIVATE_WP);//Set WP#0 as the active WP
  #else
    GPS_latitude_home  = GPS_latitude;
    GPS_longitude_home = GPS_longitude;
  #endif
}

/* this is an equirectangular approximation to calculate distance and bearing between 2 GPS points (lat/long)
   it's much more faster than an exact calculation
   the error is neglectible for few kilometers assuming a constant R for earth
   input: lat1/long1 <-> lat2/long2      unit: 1/100000 degree
   output: distance in meters, bearing in degrees
*/
void GPS_distance(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2, uint16_t* dist, int16_t* bearing) {
  float dLat = (lat2 - lat1);                                    // difference of latitude in 1/100000 degrees
  float dLon = (lon2 - lon1) * cos(lat1*(PI/180/100000.0));      // difference of longitude in 1/100000 degrees
  *dist = 6372795 / 100000.0 * PI/180*(sqrt(sq(dLat) + sq(dLon)));
  *bearing = 180/PI*(atan2(dLon,dLat));
}

#if defined(GPS_SERIAL)

/* The latitude or longitude is coded this way in NMEA frames
  dm.m   coded as degrees + minutes + minute decimal
  Where:
    - d can be 1 or more char long. generally: 2 char long for latitude, 3 char long for longitude
    - m is always 2 char long
    - m can be 1 or more char long
  This function converts this format in a unique unsigned long where 1 degree = 100 000
*/
uint32_t GPS_coord_to_degrees(char* s) {
  char *p , *d = s;
  uint32_t sec , m = 1000;
  uint16_t min , dec = 0;
  
  if(!*s) return 0;
  for(p=s; *p!=0; p++) {
    if (d != s) { *p-='0'; dec+=*p*m; m/=10; }
    if (*p == '.') d=p;
  }
  m=10000;
  min = *--d-'0';
  min += (*--d-'0')*10;
  sec = (m*min+dec)/6;
  while (d != s) { m*=10; *--d-='0'; sec+=*d*m; }
  return sec ;
}

// helper functions 
uint16_t grab_fields(char* src, uint8_t mult) {  // convert string to uint16
  uint8_t i;
  uint16_t tmp = 0;
  for(i=0; src[i]!=0; i++) {
    if(src[i] == '.') {
      i++;
      if(mult==0)   break;
      else  src[i+mult] = 0;
    }
    tmp *= 10;
    if(src[i] >='0' && src[i] <='9')	tmp += src[i]-'0';
  }
  return tmp;
}

uint8_t hex_c(uint8_t n) {		// convert '0'..'9','A'..'F' to 0..15
  n -= '0';
  if(n>9)  n -= 7;
  n &= 0x0F;
  return n;
} 

/* This is a light implementation of a GPS frame decoding
   This should work with most of modern GPS devices configured to output NMEA frames.
   It assumes there are some NMEA GGA frames to decode on the serial bus
   Here we use only the following data :
     - latitude
     - longitude
     - GPS fix is/is not ok
     - GPS num sat (4 is enough to be +/- reliable)
     // added by Mis
     - GPS altitude (for OSD displaying)
     - GPS speed (for OSD displaying)
*/
#define FRAME_GGA  1
#define FRAME_RMC  2

bool GPS_newFrame(char c) {
  uint8_t frameOK = 0;
  static uint8_t param = 0, offset = 0, parity = 0;
  static char string[15];
  static uint8_t checksum_param, frame = 0;

  if (c == '$') {
    param = 0; offset = 0; parity = 0;
  } else if (c == ',' || c == '*') {
    string[offset] = 0;
    if (param == 0) { //frame identification
      frame = 0;
      if (string[0] == 'G' && string[1] == 'P' && string[2] == 'G' && string[3] == 'G' && string[4] == 'A') frame = FRAME_GGA;
      if (string[0] == 'G' && string[1] == 'P' && string[2] == 'R' && string[3] == 'M' && string[4] == 'C') frame = FRAME_RMC;
    } else if (frame == FRAME_GGA) {
      if      (param == 2)                     {GPS_latitude = GPS_coord_to_degrees(string);}
      else if (param == 3 && string[0] == 'S') GPS_latitude = -GPS_latitude;
      else if (param == 4)                     {GPS_longitude = GPS_coord_to_degrees(string);}
      else if (param == 5 && string[0] == 'W') GPS_longitude = -GPS_longitude;
      else if (param == 6)                     {GPS_fix = string[0]  > '0' ;}
      else if (param == 7)                     {GPS_numSat = grab_fields(string,0);}
      else if (param == 9)                     {GPS_altitude = grab_fields(string,0);}	// altitude in meters added by Mis
    } else if (frame == FRAME_RMC) {
      if      (param == 7)                     {GPS_speed = ((uint32_t)grab_fields(string,1)*514444L)/100000L;}	// speed in cm/s added by Mis
    }
    param++; offset = 0;
    if (c == '*') checksum_param=1;
    else parity ^= c;
  } else if (c == '\r' || c == '\n') {
    if (checksum_param) { //parity checksum
      uint8_t checksum = hex_c(string[0]);
      checksum <<= 4;
      checksum += hex_c(string[1]);
      if (checksum == parity) frameOK = 1;
    }
    checksum_param=0;
  } else {
     if (offset < 15) string[offset++] = c;
     if (!checksum_param) parity ^= c;
  }
  return frameOK && (frame==FRAME_GGA);
}

#endif
#endif
