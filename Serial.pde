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

void serialCom() {
  uint8_t i, sr;
  #if defined(GPS_FROM_OSD)
    uint8_t *rptr;
  #endif

  if (SerialAvailable(0)) {
    switch (sr = SerialRead(0)) {
    #ifdef BTSERIAL
    case 'K': //receive RC data from Bluetooth Serial adapter as a remote
      rcData[THROTTLE] = (SerialRead(0) * 4) + 1000;
      rcData[ROLL]     = (SerialRead(0) * 4) + 1000;
      rcData[PITCH]    = (SerialRead(0) * 4) + 1000;
      rcData[YAW]      = (SerialRead(0) * 4) + 1000;
      rcData[AUX1]     = (SerialRead(0) * 4) + 1000;
      break;
    #endif
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
    case 'M': // Multiwii @ arduino to GUI all data
      serialize8('M');
      serialize8(VERSION);
      for(i=0;i<3;i++) serialize16(accSmooth[i]);
      for(i=0;i<3;i++) serialize16(gyroData[i]);
      for(i=0;i<3;i++) serialize16(magADC[i]);
      serialize16(EstAlt/10);
      serialize16(heading);
      for(i=0;i<8;i++) serialize16(servo[i]);
      for(i=0;i<8;i++) serialize16(motor[i]);
      for(i=0;i<8;i++) serialize16(rcData[i]);
      serialize8(nunchuk|ACC<<1|BARO<<2|MAG<<3|GPS<<4);
      serialize8(accMode|baroMode<<1|magMode<<2|GPSModeHome<<3|GPSModeHold<<4|armed<<5);
      #if defined(LOG_VALUES)
        serialize16(cycleTimeMax);
        cycleTimeMax = 0;
      #else
        serialize16(cycleTime);
      #endif
      serialize16(i2c_errors_count);
      for(i=0;i<2;i++) serialize16(angle[i]);
      serialize8(MULTITYPE);
      for(i=0;i<PIDITEMS;i++) {serialize8(P8[i]);serialize8(I8[i]);serialize8(D8[i]);}
      serialize8(rcRate8);
      serialize8(rcExpo8);
      serialize8(rollPitchRate);
      serialize8(yawRate);
      serialize8(dynThrPID);
      for(i=0;i<CHECKBOXITEMS;i++) {
        serialize8(activate1[i]);
        serialize8(activate2[i] | (rcOptions[i]<<7) ); // use highest bit to transport state in mwc
      }
      serialize16(GPS_distanceToHome);
      serialize16(GPS_directionToHome+180);
      serialize8(GPS_numSat);
      serialize8(GPS_fix);
      serialize8(GPS_update);
      serialize16(intPowerMeterSum);
      serialize16(intPowerTrigger1);
      serialize8(vbat);
      serialize16(BaroAlt/10);             // 4 variables are here for general monitoring purpose
      serialize16(debug2);
      serialize16(debug3);
      serialize16(debug4);
      serialize8('M');
      UartSendData();
      break;
    case 'O':  // arduino to OSD data - contribution from MIS
      serialize8('O');
      for(i=0;i<3;i++) serialize16(accSmooth[i]);
      for(i=0;i<3;i++) serialize16(gyroData[i]);
      serialize16(EstAlt*10.0f);
      serialize16(heading); // compass - 16 bytes
      for(i=0;i<2;i++) serialize16(angle[i]); //20
      for(i=0;i<6;i++) serialize16(motor[i]); //32
      for(i=0;i<6;i++) {serialize16(rcData[i]);} //44
      serialize8(nunchuk|ACC<<1|BARO<<2|MAG<<3|GPS<<4);    // added GPS info
      serialize8(accMode|baroMode<<1|magMode<<2|GPSModeHome<<3|GPSModeHold<<4|armed<<5);  // added GPS modes
      serialize8(vbat);     // Vbatt 47
      serialize8(VERSION);  // MultiWii Firmware version
      // new fields for using arduino GPS for OSD (OSD f/w >= 0.66)
      #if defined(GPS_FROM_OSD)
        serialize8(0x04);                 // Signalisation for OSD that MWC want GPS data from OSD
      #else	
        serialize8(GPS_fix);              // Fix indicator for OSD
      #endif
      serialize8(GPS_numSat);
      serialize16(GPS_latitude);
      serialize16(GPS_latitude >> 16);
      serialize16(GPS_longitude);
      serialize16(GPS_longitude >> 16);
      serialize16(GPS_altitude);
      serialize16(GPS_speed);            // Speed for OSD
      serialize8('O');
      UartSendData();
      break;
    #if defined(GPS_FROM_OSD)
    case 'G':                      // OSD to arduino data for using GPS from OSD for navigation - added by Mis
      GPS_fix = SerialRead(0);     // get GPS Fix status
      GPS_numSat = SerialRead(0);  // get number of sat
      rptr = (uint8_t *)&GPS_latitude;
      *rptr++ = SerialRead(0);	 // get latitude byte 0
      *rptr++ = SerialRead(0);	 // get latitude byte 1
      *rptr++ = SerialRead(0);	 // get latitude byte 2
      *rptr   = SerialRead(0);	 // get latitude byte 3
      rptr = (uint8_t *)&GPS_longitude;
      *rptr++ = SerialRead(0);	 // get longitude byte 0
      *rptr++ = SerialRead(0);	 // get longitude byte 1
      *rptr++ = SerialRead(0);	 // get longitude byte 2
      *rptr   = SerialRead(0);   // get longitude byte 3
      GPS_update = 1;            // new data indicator
      break;
    #endif 
    case 'W': //GUI write params to eeprom @ arduino
     #if defined(PROMICRO)
      usb_use_buf = 1; // enable USB buffer
      serialHeadRX[0] = 0; // reset tail and head
      serialTailRX[0] = 0;     
     #endif
      while (SerialAvailable(0)<(7+3*PIDITEMS+2*CHECKBOXITEMS)) {}
      for(i=0;i<PIDITEMS;i++) {P8[i]= SerialRead(0); I8[i]= SerialRead(0); D8[i]= SerialRead(0);}
      rcRate8 = SerialRead(0); rcExpo8 = SerialRead(0); //2
      rollPitchRate = SerialRead(0); yawRate = SerialRead(0); //4
      dynThrPID = SerialRead(0); //5
      for(i=0;i<CHECKBOXITEMS;i++) {activate1[i] = SerialRead(0);activate2[i] = SerialRead(0);}
     #if defined(POWERMETER)
      powerTrigger1 = (SerialRead(0) + 256* SerialRead(0) ) / PLEVELSCALE; // we rely on writeParams() to compute corresponding pAlarm value
     #else
      SerialRead(0);SerialRead(0); //7 so we unload the two bytes
     #endif
     #if defined(PROMICRO)
      usb_use_buf = 0; // disable USB buffer
     #endif
      writeParams();
      break;
    case 'S': //GUI to arduino ACC calibration request
      calibratingA=400;
      break;
    case 'E': //GUI to arduino MAG calibration request
      calibratingM=1;
      break;
    }
  }
}

// *******************************************************
// Interrupt driven UART transmitter - using a ring buffer
// *******************************************************
static uint8_t headTX,tailTX;
static uint8_t bufTX[256];      // 256 is choosen to avoid modulo operations on 8 bits pointers
void serialize16(int16_t a) {bufTX[headTX++]  = a; bufTX[headTX++]  = a>>8&0xff;}
void serialize8(uint8_t a)  {bufTX[headTX++]  = a;}

#if !defined(PROMICRO)
  ISR_UART {
    UDR0 = bufTX[tailTX++];         // Transmit next byte in the ring
    if ( tailTX == headTX )         // Check if all data is transmitted
      UCSR0B &= ~(1<<UDRIE0);       // Disable transmitter UDRE interrupt
  }
#endif

void UartSendData() {         // Data transmission acivated when the ring is not empty
  #if !defined(PROMICRO)
    UCSR0B |= (1<<UDRIE0);      // Enable transmitter UDRE interrupt
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
