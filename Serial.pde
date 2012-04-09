// 256 RX buffer is needed for GPS communication (64 or 128 was too short)
// it avoids also modulo operations

#if defined(PROMINI) 
  uint8_t serialBufferRX[256][1];
  volatile uint8_t serialHeadRX[1],serialTailRX[1];
#endif
#if defined(PROMICRO)
  uint8_t serialBufferRX[256][2];
  volatile uint8_t serialHeadRX[2],serialTailRX[2];
  uint8_t usb_use_buf = 0;
#endif
#if defined(MEGA)
  uint8_t serialBufferRX[256][4];
  volatile uint8_t serialHeadRX[4],serialTailRX[4];
#endif


#define MSP_IDENT                100   //out message         multitype + version
#define MSP_STATUS               101   //out message         cycletime & errors_count & sensor present & box activation
#define MSP_RAW_IMU              102   //out message         9 DOF
#define MSP_SERVO                103   //out message         8 servos
#define MSP_MOTOR                104   //out message         8 motors
#define MSP_RC                   105   //out message         8 rc chan
#define MSP_RAW_GPS              106   //out message         fix, numsat, lat, lon, alt, speed
#define MSP_COMP_GPS             107   //out message         distance home, direction home
#define MSP_ATTITUDE             108   //out message         2 angles 1 heading
#define MSP_ALTITUDE             109   //out message         1 altitude
#define MSP_BAT                  110   //out message         vbat, powermetersum
#define MSP_RC_TUNING            111   //out message         rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_PID                  112   //out message         up to 16 P I D (8 are used)
#define MSP_BOX                  113   //out message         up to 16 checkbox (11 are used)
#define MSP_MISC                 114   //out message         powermeter trig + 8 free for future use

#define MSP_SET_RAW_RC           205   //in message          8 rc chan
#define MSP_SET_RAW_GPS          206   //in message          fix, numsat, lat, lon, alt, speed
#define MSP_SET_PID              212   //in message          up to 16 P I D (8 are used)
#define MSP_SET_BOX              213   //in message          up to 16 checkbox (11 are used)
#define MSP_SET_RC_TUNING        214   //in message          rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_ACC_CALIBRATION      215   //in message          no param
#define MSP_MAG_CALIBRATION      216   //in message          no param
#define MSP_SET_MISC             217   //in message          powermeter trig + 8 free for future use
#define MSP_RESET_CONF           218   //in message          no param

#define MSP_EEPROM_WRITE         220   //in message          no param

#define MSP_DEBUG                254   //out message         debug1,debug2,debug3,debug4


static uint8_t checksum,stateMSP,indRX,inBuf[64];

uint32_t read32() {
  uint32_t t = inBuf[indRX++];
  t+= inBuf[indRX++]<<8;
  t+= inBuf[indRX++]<<16;
  t+= inBuf[indRX++]<<24;
  return t;
}

uint16_t read16() {
  uint16_t t = inBuf[indRX++];
  t+= inBuf[indRX++]<<8;
  return t;
}
uint8_t read8()  {return inBuf[indRX++]&0xff;}

void headSerialReply(uint8_t c) {
  serialize8('$');serialize8('M');serialize8('>');serialize8(c);checksum = 0;
}

void tailSerialReply() {
  serialize8(checksum);UartSendData();stateMSP = 0;
}

void serialCom() {
  uint8_t i, c;
  static uint8_t offset,dataSize;
  
  while (SerialAvailable(0)) {
    c = SerialRead(0);

    if (stateMSP > 3) {
      if (offset < dataSize+1) {
        inBuf[offset++] = c;
        if ( offset < dataSize+1 ) checksum ^= c;
      } else {stateMSP = 0;}
      if ( offset == dataSize+1  && checksum == inBuf[offset-1] && stateMSP > 0) {
        #if defined(PROMICRO)
          usb_use_buf = 1; // enable USB buffer
          serialHeadRX[0] = 0; // reset tail and head
          serialTailRX[0] = 0;     
        #endif
        switch(stateMSP) {
          case MSP_SET_RAW_RC:
            stateMSP = 0;
            for(i=0;i<8;i++) {rcData[i] = read16();}
            break;
          case MSP_SET_RAW_GPS:
            stateMSP = 0;
            GPS_fix = read8();
            GPS_numSat = read8();
            GPS_latitude = read32();
            GPS_longitude = read32();
            GPS_altitude = read16();
            GPS_speed = read16();
            GPS_update = 1;
            break;
          case MSP_SET_PID:
            stateMSP = 0;
            for(i=0;i<PIDITEMS;i++) {P8[i]=read8();I8[i]=read8();D8[i]=read8();}
            break;
          case MSP_SET_BOX:
            stateMSP = 0;
            for(i=0;i<CHECKBOXITEMS;i++) {activate[i]=read16();}
            break;
          case MSP_SET_RC_TUNING:
            stateMSP = 0;
            rcRate8 = read8();
            rcExpo8 = read8();
            rollPitchRate = read8();
            yawRate = read8();
            dynThrPID = read8();
            break;
          case MSP_SET_MISC:
            stateMSP = 0;
            #if defined(POWERMETER)
              powerTrigger1 = read16() / PLEVELSCALE; // we rely on writeParams() to compute corresponding pAlarm value
            #endif
            break;
        }
        #if defined(PROMICRO)
          usb_use_buf = 0; // disable USB buffer
        #endif
      }
      return;
    } else {
      offset = 0;checksum = 0;indRX=0;
    }

    if (stateMSP == 3) {
      switch(c) {
        case MSP_IDENT:
          headSerialReply(c);
          serialize8(VERSION);
          serialize8(MULTITYPE);
          tailSerialReply();break;
        case MSP_STATUS:
          headSerialReply(c);
          serialize16(cycleTime);
          serialize16(i2c_errors_count);
          serialize16((ACC|nunchuk)|BARO<<1|MAG<<2|GPS<<3|SONAR<<4);
          serialize16(accMode<<BOXACC|baroMode<<BOXBARO|magMode<<BOXMAG|armed<<BOXARM|
                      GPSModeHome<<BOXGPSHOME|GPSModeHold<<BOXGPSHOLD|headFreeMode<<BOXHEADFREE);
          tailSerialReply();break;
        case MSP_RAW_IMU:
          headSerialReply(c);
          for(i=0;i<3;i++) serialize16(accSmooth[i]);
          for(i=0;i<3;i++) serialize16(gyroData[i]);
          for(i=0;i<3;i++) serialize16(magADC[i]);
          tailSerialReply();break;
        case MSP_SERVO:
          headSerialReply(c);
          for(i=0;i<8;i++) serialize16(servo[i]);
          tailSerialReply();break;
        case MSP_MOTOR:
          headSerialReply(c);
          for(i=0;i<8;i++) serialize16(motor[i]);
          tailSerialReply();break;
        case MSP_RC:
          headSerialReply(c);
          for(i=0;i<8;i++) serialize16(rcData[i]);
          tailSerialReply();break;
        case MSP_RAW_GPS:
          headSerialReply(c);
          serialize8(GPS_fix);
          serialize8(GPS_numSat);
          serialize32(GPS_latitude);
          serialize32(GPS_longitude);
          serialize16(GPS_altitude);
          serialize16(GPS_speed);
          tailSerialReply();break;
        case MSP_COMP_GPS:
          headSerialReply(c);
          serialize16(GPS_distanceToHome);
          serialize16(GPS_directionToHome+180);
          serialize8(GPS_update);
          tailSerialReply();break;
        case MSP_ATTITUDE:
          headSerialReply(c);
          for(i=0;i<2;i++) serialize16(angle[i]);
          serialize16(heading);
          tailSerialReply();break;
        case MSP_ALTITUDE:
          headSerialReply(c);
          serialize16(EstAlt/10);
          tailSerialReply();break;
        case MSP_BAT:
          headSerialReply(c);
          serialize8(vbat);
          serialize16(intPowerMeterSum);
          tailSerialReply();break;
        case MSP_RC_TUNING:
          headSerialReply(c);
          serialize8(rcRate8);
          serialize8(rcExpo8);
          serialize8(rollPitchRate);
          serialize8(yawRate);
          serialize8(dynThrPID);
          tailSerialReply();break;
        case MSP_PID:
          headSerialReply(c);
          for(i=0;i<PIDITEMS;i++)    {serialize8(P8[i]);serialize8(I8[i]);serialize8(D8[i]);}
          for(i=0;i<16-PIDITEMS;i++) {serialize8(0);serialize8(0);serialize8(0);} //future use
          tailSerialReply();break;
        case MSP_BOX:
          headSerialReply(c);
          for(i=0;i<CHECKBOXITEMS;i++)    {serialize16(activate[i]);}
          for(i=0;i<16-CHECKBOXITEMS;i++) {serialize16(0);} //future use
          tailSerialReply();break;
        case MSP_MISC:
          headSerialReply(c);
          serialize16(intPowerTrigger1);
          for(i=0;i<8;i++) {serialize8(0);} //future use
          tailSerialReply();break;
        case MSP_SET_RAW_RC:
          stateMSP = MSP_SET_RAW_RC;
          dataSize = 16;break;
        case MSP_SET_RAW_GPS:
          stateMSP = MSP_SET_RAW_GPS;
          dataSize = 14;break;
        case MSP_SET_PID:
          stateMSP = MSP_SET_PID;
          dataSize = 48;break;
        case MSP_SET_BOX:
          stateMSP = MSP_SET_BOX;
          dataSize = 32;break;
        case MSP_SET_RC_TUNING:
          stateMSP = MSP_SET_RC_TUNING;
          dataSize = 5;break;
        case MSP_SET_MISC:
          stateMSP = MSP_SET_MISC;
          dataSize = 10;break;
        case MSP_RESET_CONF:
          stateMSP = 0;
          checkNewConf++;checkFirstTime();
          checkNewConf--;checkFirstTime();
          break;
        case MSP_ACC_CALIBRATION:
          stateMSP = 0;
          calibratingA=400;
          break;
        case MSP_MAG_CALIBRATION:
          stateMSP = 0;
          calibratingM=1;
          break;
        case MSP_EEPROM_WRITE:
          stateMSP = 0;
          writeParams();
          break;
        case MSP_DEBUG:
          headSerialReply(c);
          serialize16(debug1);             // 4 variables are here for general monitoring purpose
          serialize16(debug2);
          serialize16(debug3);
          serialize16(debug4);
          tailSerialReply();break;
      }
      return;
    }
    if (stateMSP < 3) {
      switch(c) {
        case '$':                                         //header detection $MW<
          if (stateMSP == 0) stateMSP++;break;
        case 'M':
          if (stateMSP == 1) stateMSP++;break;
        case '<':
          if (stateMSP == 2) stateMSP++;break;
      }
    }
    if (stateMSP == 0) {
      oldSerialCom(c);
    }
  }
}

void oldSerialCom(uint8_t sr) {
  //last things to clean
  switch (sr) {
    #ifdef LCD_TELEMETRY
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
    case '1':
    case '2':
    case '3':
    case '4':
    case '5':
    case '6':
    case '7':
    case '8':
    case '9':
    #if defined(LOG_VALUES) && defined(DEBUG)
    case 'R':
    #endif
    #ifdef DEBUG
    case 'F':
    #endif
      toggle_telemetry(sr);
      break;
    case 'a': // button A release
    case 'b': // button B release
    case 'c': // button C release
    case 'd': // button D release
      break;      
    #endif // LCD_TELEMETRY
  }
}


// *******************************************************
// Interrupt driven UART transmitter - using a ring buffer
// *******************************************************
static uint8_t headTX,tailTX;
static uint8_t bufTX[256];      // 256 is choosen to avoid modulo operations on 8 bits pointers

void serialize32(uint32_t a) {
  static uint8_t t;
  t = a;       bufTX[headTX++] = t ; checksum ^= t;
  t = a>>8;    bufTX[headTX++] = t ; checksum ^= t;
  t = a>>16;   bufTX[headTX++] = t ; checksum ^= t;
  t = a>>24;   bufTX[headTX++] = t ; checksum ^= t;
  #if !defined(PROMICRO)
    UCSR0B |= (1<<UDRIE0);      // in case ISR_UART desactivates the interrupt, we force its reactivation anyway
  #endif 
}
void serialize16(int16_t a) {
  static uint8_t t;
  t = a;          bufTX[headTX++] = t ; checksum ^= t;
  t = a>>8&0xff;  bufTX[headTX++] = t ; checksum ^= t;
  #if !defined(PROMICRO)
    UCSR0B |= (1<<UDRIE0);
  #endif 
}
void serialize8(uint8_t a)  {
  bufTX[headTX++]  = a; checksum ^= a;
  #if !defined(PROMICRO)
    UCSR0B |= (1<<UDRIE0);
  #endif
}

#if !defined(PROMICRO)
  ISR_UART {
    if( headTX != tailTX )
      UDR0 = bufTX[tailTX++];       // Transmit next byte in the ring
    if ( tailTX == headTX )         // Check if all data is transmitted
      UCSR0B &= ~(1<<UDRIE0);       // Disable transmitter UDRE interrupt
  }
#endif

void UartSendData() {               // Data transmission acivated when the ring is not empty
  #if !defined(PROMICRO)
    UCSR0B |= (1<<UDRIE0);          // Enable transmitter UDRE interrupt
  #else
    USB_Send(USB_CDC_TX,bufTX,headTX);
    headTX = 0;
  #endif
}


void SerialOpen(uint8_t port, uint32_t baud) {
  uint8_t h = ((F_CPU  / 4 / baud -1) / 2) >> 8;
  uint8_t l = ((F_CPU  / 4 / baud -1) / 2);
  switch (port) {
    #if !defined(PROMICRO)
    case 0: UCSR0A  = (1<<U2X0); UBRR0H = h; UBRR0L = l; UCSR0B |= (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0); break;
    #endif
    #if defined(MEGA) || defined(PROMICRO)
    case 1: UCSR1A  = (1<<U2X1); UBRR1H = h; UBRR1L = l; UCSR1B |= (1<<RXEN1)|(1<<TXEN1)|(1<<RXCIE1); break;
    #endif
    #if defined(MEGA)
    case 2: UCSR2A  = (1<<U2X2); UBRR2H = h; UBRR2L = l; UCSR2B |= (1<<RXEN2)|(1<<TXEN2)|(1<<RXCIE2); break;
    case 3: UCSR3A  = (1<<U2X3); UBRR3H = h; UBRR3L = l; UCSR3B |= (1<<RXEN3)|(1<<TXEN3)|(1<<RXCIE3); break;
    #endif
  }
}

void SerialEnd(uint8_t port) {
  switch (port) {
    #if !defined(PROMICRO)
    case 0: UCSR0B &= ~((1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0)|(1<<UDRIE0)); break;
    #endif
    #if defined(MEGA) || defined(PROMICRO)
    case 1: UCSR1B &= ~((1<<RXEN1)|(1<<TXEN1)|(1<<RXCIE1)); break;
    #endif
    #if defined(MEGA) 
    case 2: UCSR2B &= ~((1<<RXEN2)|(1<<TXEN2)|(1<<RXCIE2)); break;
    case 3: UCSR3B &= ~((1<<RXEN3)|(1<<TXEN3)|(1<<RXCIE3)); break;
    #endif
  }
}

#if defined(PROMINI) && !(defined(SPEKTRUM))
ISR(USART_RX_vect){
  uint8_t d = UDR0;
  uint8_t i = serialHeadRX[0] + 1;
  if (i != serialTailRX[0]) {serialBufferRX[serialHeadRX[0]][0] = d; serialHeadRX[0] = i;}
}
#endif
#if defined(MEGA)
ISR(USART0_RX_vect){
  uint8_t d = UDR0;
  uint8_t i = serialHeadRX[0] + 1;
  if (i != serialTailRX[0]) {serialBufferRX[serialHeadRX[0]][0] = d; serialHeadRX[0] = i;}
}
#endif
#if defined(MEGA) || defined(PROMICRO)
  #if !(defined(SPEKTRUM))
  ISR(USART1_RX_vect){
    uint8_t d = UDR1;
    uint8_t i = serialHeadRX[1] + 1;
    if (i != serialTailRX[1]) {serialBufferRX[serialHeadRX[1]][1] = d; serialHeadRX[1] = i;}
  }
  #endif
#endif
#if defined(MEGA)
ISR(USART2_RX_vect){
  uint8_t d = UDR2;
  uint8_t i = serialHeadRX[2] + 1;
  if (i != serialTailRX[2]) {serialBufferRX[serialHeadRX[2]][2] = d; serialHeadRX[2] = i;}
}
ISR(USART3_RX_vect){
  uint8_t d = UDR3;
  uint8_t i = serialHeadRX[3] + 1;
  if (i != serialTailRX[3]) {serialBufferRX[serialHeadRX[3]][3] = d; serialHeadRX[3] = i;}
}
#endif

uint8_t SerialRead(uint8_t port) {
  #if !defined(PROMICRO)
      uint8_t c = serialBufferRX[serialTailRX[port]][port];
      if ((serialHeadRX[port] != serialTailRX[port])) serialTailRX[port] = serialTailRX[port] + 1;
    #else
      uint8_t c;
      if(port == 0){
        if(usb_use_buf && serialTailRX[0]<=serialHeadRX[0]){
          c = serialBufferRX[0][serialTailRX[0]++]; // if USB buffer is enabled we give the stored values
        }else{
          // the direckt way without Serial... the usb serial stuff is uploaded anyway
          c = USB_Recv(USB_CDC_RX);
        }
      }else{
        c = serialBufferRX[serialTailRX[port]][port];
        if ((serialHeadRX[port] != serialTailRX[port])) serialTailRX[port] = serialTailRX[port] + 1;        
      }
    #endif    
    return c;
}

uint8_t SerialAvailable(uint8_t port) {
 #if !defined(PROMICRO)
    return serialHeadRX[port] - serialTailRX[port];
  #else
    if(port == 0){
      if(usb_use_buf){
        if(USB_Available(USB_CDC_RX)){
          serialBufferRX[0][serialHeadRX[0]++] = USB_Recv(USB_CDC_RX); // if USB buffer is enabled we store all readings to it 
        }
        return serialHeadRX[0];
      }else{
        return USB_Available(USB_CDC_RX);
      }
    }else{
      return serialHeadRX[port] - serialTailRX[port];
    }
  #endif
}

void SerialWrite(uint8_t port,uint8_t c){
 switch (port) {
    case 0: serialize8(c);UartSendData(); break;                 // Serial0 TX is driven via a buffer and a background intterupt
    #if defined(MEGA) || #defined(PROMICRO)
    case 1: while (!(UCSR1A & (1 << UDRE1))) ; UDR1 = c; break;  // Serial1 Serial2 and Serial3 TX are not driven via interrupts
    #endif
    #if defined(MEGA)
    case 2: while (!(UCSR2A & (1 << UDRE2))) ; UDR2 = c; break;
    case 3: while (!(UCSR3A & (1 << UDRE3))) ; UDR3 = c; break;
    #endif
  }
}
