#if GPS

void GPS_NewData() {
  #if defined(I2C_GPS)
    static uint8_t _i2c_gps_status;
  
    //Do not use i2c_writereg, since writing a register does not work if an i2c_stop command is issued at the end
    //Still investigating, however with separated i2c_repstart and i2c_write commands works... and did not caused i2c errors on a long term test.
  
    _i2c_gps_status = i2c_readReg(I2C_GPS_ADDRESS,I2C_GPS_STATUS);                    //Get status register 
    if (_i2c_gps_status & I2C_GPS_STATUS_3DFIX) {                                     //Check is we have a good 3d fix (numsats>5)
       GPS_fix = 1;                                                                   //Set fix
       GPS_numSat = (_i2c_gps_status & 0xf0) >> 4;                                    //Num of sats is stored the upmost 4 bits of status
       if (!GPS_fix_home) {        //if home is not set set home position to WP#0 and activate it
          i2c_rep_start(I2C_GPS_ADDRESS);i2c_write(I2C_GPS_COMMAND);i2c_write(I2C_GPS_COMMAND_SET_WP);//Store current position to WP#0 (this is used for RTH)
          i2c_rep_start(I2C_GPS_ADDRESS);i2c_write(I2C_GPS_COMMAND);i2c_write(I2C_GPS_COMMAND_ACTIVATE_WP);//Set WP#0 as the active WP
          GPS_fix_home = 1;                                                           //Now we have a home   
       }
       if (_i2c_gps_status & I2C_GPS_STATUS_NEW_DATA) {                               //Check about new data
          if (GPS_update) { GPS_update = 0;} else { GPS_update = 1;}                  //Fancy flash on GUI :D
          //Read GPS data for distance and heading
          i2c_rep_start(I2C_GPS_ADDRESS);
          i2c_write(I2C_GPS_DISTANCE);                                                //Start read from here 2x2 bytes distance and direction
          i2c_rep_start(I2C_GPS_ADDRESS+1);
          uint8_t *varptr = (uint8_t *)&GPS_distanceToHome;
          *varptr++ = i2c_readAck();
          *varptr   = i2c_readAck();
          varptr = (uint8_t *)&GPS_directionToHome;
          *varptr++ = i2c_readAck();
          *varptr   = i2c_readNak();
        }
  
    } else {                                                                          //We don't have a fix zero out distance and bearing (for safety reasons)
      GPS_distanceToHome = 0;
      GPS_directionToHome = 0;
      GPS_numSat = 0;
    }  

    if (rcData[AUX4]>1800 || GPS_fix_home == 0) {
      i2c_rep_start(I2C_GPS_ADDRESS);i2c_write(I2C_GPS_COMMAND);i2c_write(I2C_GPS_COMMAND_SET_WP);//Store current position to WP#0 (this is used for RTH)
      i2c_rep_start(I2C_GPS_ADDRESS);i2c_write(I2C_GPS_COMMAND);i2c_write(I2C_GPS_COMMAND_ACTIVATE_WP);//Set WP#0 as the active WP
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
}

/* this is an equirectangular approximation to calculate distance and bearing between 2 GPS points (lat/long)
   it's much more faster than an exact calculation
   the error is neglectible for few kilometers assuming a constant R for earth
   input: lat1/long1 <-> lat2/long2      unit: 1/100000 degree
   output: distance in meters, bearing in degrees
*/
void GPS_distance(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2, uint16_t* dist, int16_t* bearing) {
  float dLat = ((lat2 - lat1));                                    // difference of latitude in 1/100000 degrees
  float dLon = ((lon2 - lon1)) * cos(lat1*(PI/180/100000.0));      // difference of longitude in 1/100000 degrees
  *dist = 6372795 / 100000.0 * PI/180*(sqrt(sq(dLat) + sq(dLon)));
  if (lat1 != lat2)
    *bearing = 180/PI*(atan2(dLon,dLat));
  else
    *bearing = 0;
}

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


/* This is a light implementation of a GPS frame decoding
   This should work with most of modern GPS devices configured to output NMEA frames.
   It assumes there are some NMEA GGA frames to decode on the serial bus
   Here we use only the following data :
     - latitude
     - longitude
     - GPS fix is/is not ok
     - GPS num sat (4 is enough to be +/- reliable)
*/
bool GPS_newFrame(char c) {
  uint8_t frameOK = 0;
  static uint8_t param = 0, offset = 0, parity = 0;
  static char string[15];
  static uint8_t checksum_param, GPGGA_frame = 0;

  if (c == '$') {
    param = 0; offset = 0; parity = 0;
  } else if (c == ',' || c == '*') {
    string[offset] = 0;
    if (param == 0) { //frame identification
      if (string[0] == 'G' && string[1] == 'P' && string[2] == 'G' && string[3] == 'G' && string[4] == 'A') GPGGA_frame = 1;
      else GPGGA_frame = 0;
    } else if (GPGGA_frame == 1) {
      if      (param == 2)                     {GPS_latitude = GPS_coord_to_degrees(string);}
      else if (param == 3 && string[0] == 'S') GPS_latitude = -GPS_latitude;
      else if (param == 4)                     {GPS_longitude = GPS_coord_to_degrees(string);}
      else if (param == 5 && string[0] == 'W') GPS_longitude = -GPS_longitude;
      else if (param == 6)                     {GPS_fix = string[0]  > '0' ;}
      else if (param == 7)                     {if (offset>1) GPS_numSat = (string[0]-'0') * 10 + string[1]-'0'; else GPS_numSat = string[0]-'0';}
    }
    param++; offset = 0;
    if (c == '*') checksum_param=1;
    else parity ^= c;
  } else if (c == '\r' || c == '\n') {
    if (checksum_param) { //parity checksum
      uint8_t checksum = 16 * ((string[0]>='A') ? string[0] - 'A'+10: string[0] - '0') + ((string[1]>='A') ? string[1] - 'A'+10: string[1]-'0');
      if (checksum == parity) frameOK = 1;
    }
    checksum_param=0;
  } else {
     if (offset < 15) string[offset++] = c;
     if (!checksum_param) parity ^= c;
  }
  return frameOK && GPGGA_frame;
}
#endif
