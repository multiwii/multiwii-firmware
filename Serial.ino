// 256 RX buffer is needed for GPS communication (64 or 128 was too short)
#if defined(GPS_SERIAL)
#define RX_BUFFER_SIZE 256
#else
#define RX_BUFFER_SIZE 64
#endif

#if defined(MEGA)
  uint8_t serialBufferRX[RX_BUFFER_SIZE][4];
  volatile uint8_t serialHeadRX[4],serialTailRX[4];
#else
  uint8_t serialBufferRX[RX_BUFFER_SIZE][1];
  volatile uint8_t serialHeadRX[1],serialTailRX[1];
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
#define MSP_MOTOR_PINS           115   //out message         which pins are in use for motors & servos, for GUI 
#define MSP_BOXNAMES             116   //out message         the aux switch names
#define MSP_PIDNAMES             117   //out message         the PID names

#define MSP_SET_RAW_RC           200   //in message          8 rc chan
#define MSP_SET_RAW_GPS          201   //in message          fix, numsat, lat, lon, alt, speed
#define MSP_SET_PID              202   //in message          up to 16 P I D (8 are used)
#define MSP_SET_BOX              203   //in message          up to 16 checkbox (11 are used)
#define MSP_SET_RC_TUNING        204   //in message          rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_ACC_CALIBRATION      205   //in message          no param
#define MSP_MAG_CALIBRATION      206   //in message          no param
#define MSP_SET_MISC             207   //in message          powermeter trig + 8 free for future use
#define MSP_RESET_CONF           208   //in message          no param

#define MSP_EEPROM_WRITE         250   //in message          no param

#define MSP_DEBUG                254   //out message         debug1,debug2,debug3,debug4

static uint8_t checksum;
static uint8_t indRX;
#define INBUF_SIZE 64
static uint8_t inBuf[INBUF_SIZE];

uint32_t read32() {
  uint32_t t = inBuf[indRX++];
  t+= inBuf[indRX++]<<8;
  t+= (uint32_t)inBuf[indRX++]<<16;
  t+= (uint32_t)inBuf[indRX++]<<24;
  return t;
}

uint16_t read16() {
  uint16_t t = inBuf[indRX++];
  t+= inBuf[indRX++]<<8;
  return t;
}
uint8_t read8()  {return inBuf[indRX++]&0xff;}

void headSerialResponse(uint8_t err, uint8_t c,uint8_t s) {
  serialize8('$');
  serialize8('M');
  serialize8(err ? '!' : '>');
  /* start calculating a new checksum */
  checksum = 0;
  serialize8(s);
  serialize8(c);
}

void headSerialReply(uint8_t c,uint8_t s) {
  headSerialResponse(0, c, s);
}

void headSerialError(uint8_t c,uint8_t s) {
  headSerialResponse(1, c, s);
}

void tailSerialReply() {
  serialize8(checksum);UartSendData();
}

uint8_t getNameLength(PGM_P s) {
  return strlen_P(s);
}

void serializeNames(PGM_P s) {
  for (PGM_P c = s; pgm_read_byte(c); c++) {
    serialize8(pgm_read_byte(c));
  }
}

void serialCom() {
  uint8_t c;
  static uint8_t cmd;
  static uint8_t offset;
  static uint8_t dataSize;
  static enum _serial_state {
    IDLE,
    HEADER_START,
    HEADER_M,
    HEADER_ARROW,
    HEADER_SIZE,
    HEADER_CMD,
  } c_state = IDLE;
  
  while (SerialAvailable(0)) {
    c = SerialRead(0);

    if (c_state == IDLE) {
      c_state = (c=='$') ? HEADER_START : IDLE;
    } else if (c_state == HEADER_START) {
      c_state = (c=='M') ? HEADER_M : IDLE;
    } else if (c_state == HEADER_M) {
      c_state = (c=='<') ? HEADER_ARROW : IDLE;
    } else if (c_state == HEADER_ARROW) {
      /* now we are expecting the payload size */
      if (c > INBUF_SIZE) {
        c_state = IDLE;
        continue;
      }
      dataSize = c;
      offset = 0;
      checksum = 0;
      indRX = 0;
      checksum ^= c;
      /* the command is to follow */
      c_state = HEADER_SIZE;
    } else if (c_state == HEADER_SIZE) {
      cmd = c;
      checksum ^= c;
      c_state = HEADER_CMD;
    } else if (c_state == HEADER_CMD && offset < dataSize) {
      checksum ^= c;
      inBuf[offset++] = c;
    } else if (c_state == HEADER_CMD && offset >= dataSize) {
      /* compare calculated and transferred checksum */
      if (checksum == c) {
        /* we got a valid packet, evaluate it */
        evaluateCommand(cmd, dataSize);
      } else {
      }
      c_state = IDLE;
    }
  }
}

void evaluateCommand(uint8_t c, uint8_t dataSize) {
  #if !defined(PROMICRO)
    UCSR0B &= ~(1<<UDRIE0); // disable transmitter UDRE interrupt during the serialization
  #endif
  switch(c) {
   case MSP_SET_RAW_RC:
     for(uint8_t i=0;i<8;i++) {
       rcData[i] = read16();
     }
     headSerialReply(c,0);
     break;
   case MSP_SET_RAW_GPS:
     set_flag(FLAG_GPS_FIX, read8());
     GPS_numSat = read8();
     GPS_coord[LAT] = read32();
     GPS_coord[LON] = read32();
     GPS_altitude = read16();
     GPS_speed = read16();
     GPS_update = 1;
     headSerialReply(c,0);
     break;
   case MSP_SET_PID:
     for(uint8_t i=0;i<PIDITEMS;i++) {
       conf.P8[i]=read8();
       conf.I8[i]=read8();
       conf.D8[i]=read8();
     }
     headSerialReply(c,0);
     break;
   case MSP_SET_BOX:
     for(uint8_t i=0;i<CHECKBOXITEMS;i++) {
       conf.activate[i]=read16();
     }
     headSerialReply(c,0);
     break;
   case MSP_SET_RC_TUNING:
     conf.rcRate8 = read8();
     conf.rcExpo8 = read8();
     conf.rollPitchRate = read8();
     conf.yawRate = read8();
     conf.dynThrPID = read8();
     conf.thrMid8 = read8();
     conf.thrExpo8 = read8();
     headSerialReply(c,0);
     break;
   case MSP_SET_MISC:
     #if defined(POWERMETER)
       conf.powerTrigger1 = read16() / PLEVELSCALE; // we rely on writeParams() to compute corresponding pAlarm value
     #endif
     headSerialReply(c,0);
     break;
   case MSP_IDENT:                          // and we check message code to execute the relative code
     headSerialReply(c,2);                  // we reply with an header indicating a payload lenght of 2 octets
     serialize8(VERSION);                   // the first octet. serialize8/16/32 is used also to compute a checksum
     serialize8(MULTITYPE);                 // the second one
     break;
   case MSP_STATUS:
     headSerialReply(c,8);
     serialize16(cycleTime);
     serialize16(i2c_errors_count);
     serialize16(ACC|BARO<<1|MAG<<2|GPS<<3|SONAR<<4);
     serialize16(get_flag(FLAG_ACC_MODE)<<BOXACC|get_flag(FLAG_BARO_MODE)<<BOXBARO|get_flag(FLAG_MAG_MODE)<<BOXMAG|get_flag(FLAG_ARMED)<<BOXARM|
                 get_flag(FLAG_GPS_HOME_MODE)<<BOXGPSHOME|get_flag(FLAG_GPS_HOLD_MODE)<<BOXGPSHOLD|get_flag(FLAG_HEADFREE_MODE)<<BOXHEADFREE|
                 get_flag(FLAG_PASSTHRU_MODE)<<BOXPASSTHRU|rcOptions[BOXBEEPERON]<<BOXBEEPERON|rcOptions[BOXLEDMAX]<<BOXLEDMAX|rcOptions[BOXLLIGHTS]<<BOXLLIGHTS|rcOptions[BOXHEADADJ]<<BOXHEADADJ);
     break;
   case MSP_RAW_IMU:
     headSerialReply(c,18);
     for(uint8_t i=0;i<3;i++) serialize16(accSmooth[i]);
     for(uint8_t i=0;i<3;i++) serialize16(gyroData[i]);
     for(uint8_t i=0;i<3;i++) serialize16(magADC[i]);
     break;
   case MSP_SERVO:
     headSerialReply(c,16);
     for(uint8_t i=0;i<8;i++)
       #if defined(SERVO)
       serialize16(servo[i]);
       #else
       serialize16(0);
       #endif
     break;
   case MSP_MOTOR:
     headSerialReply(c,16);
     for(uint8_t i=0;i<8;i++) {
       serialize16( (i < NUMBER_MOTOR) ? motor[i] : 0 );
     }
     break;
   case MSP_RC:
     headSerialReply(c,16);
     for(uint8_t i=0;i<8;i++) serialize16(rcData[i]);
     break;
   case MSP_RAW_GPS:
     headSerialReply(c,14);
     serialize8(get_flag(FLAG_GPS_FIX));
     serialize8(GPS_numSat);
     serialize32(GPS_coord[LAT]);
     serialize32(GPS_coord[LON]);
     serialize16(GPS_altitude);
     serialize16(GPS_speed);
     break;
   case MSP_COMP_GPS:
     headSerialReply(c,5);
     serialize16(GPS_distanceToHome);
     serialize16(GPS_directionToHome);
     serialize8(GPS_update);
     break;
   case MSP_ATTITUDE:
     headSerialReply(c,6);
     for(uint8_t i=0;i<2;i++) serialize16(angle[i]);
     serialize16(heading);
     break;
   case MSP_ALTITUDE:
     headSerialReply(c,4);
     serialize32(EstAlt);
     break;
   case MSP_BAT:
     headSerialReply(c,3);
     serialize8(vbat);
     serialize16(intPowerMeterSum);
     break;
   case MSP_RC_TUNING:
     headSerialReply(c,7);
     serialize8(conf.rcRate8);
     serialize8(conf.rcExpo8);
     serialize8(conf.rollPitchRate);
     serialize8(conf.yawRate);
     serialize8(conf.dynThrPID);
     serialize8(conf.thrMid8);
     serialize8(conf.thrExpo8);
     break;
   case MSP_PID:
     headSerialReply(c,3*PIDITEMS);
     for(uint8_t i=0;i<PIDITEMS;i++) {
       serialize8(conf.P8[i]);
       serialize8(conf.I8[i]);
       serialize8(conf.D8[i]);
     }
     break;
   case MSP_BOX:
     headSerialReply(c,2*CHECKBOXITEMS);
     for(uint8_t i=0;i<CHECKBOXITEMS;i++) {
       serialize16(conf.activate[i]);
     }
     break;
   case MSP_BOXNAMES:
     headSerialReply(c,getNameLength(boxnames));
     serializeNames(boxnames);
     break;
   case MSP_PIDNAMES:
     headSerialReply(c,getNameLength(pidnames));
     serializeNames(pidnames);
     break;
   case MSP_MISC:
     headSerialReply(c,2);
     serialize16(intPowerTrigger1);
     break;
   case MSP_MOTOR_PINS:
     headSerialReply(c,8);
     for(uint8_t i=0;i<8;i++) {
       serialize8(PWM_PIN[i]);
     }
     break;
   case MSP_RESET_CONF:
     conf.checkNewConf++;
     checkFirstTime();
     headSerialReply(c,0);
     break;
   case MSP_ACC_CALIBRATION:
     calibratingA=400;
     headSerialReply(c,0);
     break;
   case MSP_MAG_CALIBRATION:
     set_flag(FLAG_CALIBRATE_MAG, 1);
     headSerialReply(c,0);
     break;
   case MSP_EEPROM_WRITE:
     writeParams(0);
     headSerialReply(c,0);
     break;
   case MSP_DEBUG:
     headSerialReply(c,8);
     serialize16(debug1); // 4 variables are here for general monitoring purpose
     serialize16(debug2);
     serialize16(debug3);
     serialize16(debug4);
     break;
   default:
     /* we do not know how to handle the (valid) message, indicate error */
     headSerialError(c,0);
     break;
  }
  tailSerialReply();
  #if !defined(PROMICRO)
    UCSR0B |= (1<<UDRIE0); // enable transmitter UDRE interrupt
  #endif
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
static volatile uint8_t headTX,tailTX;
#define TX_BUFFER_SIZE 128
static uint8_t bufTX[TX_BUFFER_SIZE];

void serialize32(uint32_t a) {
  serialize8((a    ) & 0xFF);
  serialize8((a>> 8) & 0xFF);
  serialize8((a>>16) & 0xFF);
  serialize8((a>>24) & 0xFF);
}

void serialize16(int16_t a) {
  /* serialize low byte */
  serialize8(a & 0xFF);
  /* serialize high byte */
  serialize8((a>>8) & 0xFF);
}

void serialize8(uint8_t a)  {
  headTX++;
  if (headTX == TX_BUFFER_SIZE) {
    headTX = 0;
  }
  bufTX[headTX] = a;
  checksum ^= a;
}

#if !defined(PROMICRO)
  ISR_UART {
    if (headTX != tailTX) {
      tailTX++;
      if (tailTX == TX_BUFFER_SIZE) {
        tailTX = 0;
      }
      UDR0 = bufTX[tailTX];  // Transmit next byte in the ring
    }
    if (tailTX == headTX) UCSR0B &= ~(1<<UDRIE0); // Check if all data is transmitted . if yes disable transmitter UDRE interrupt
  }
#endif

void UartSendData() {
  #if defined(PROMICRO)
    #if !defined(TEENSY20)
      USB_Send(USB_CDC_TX,bufTX,headTX);
    #else
      Serial.write(bufTX, headTX);
    #endif
    headTX = 0;
  #endif
}

void SerialOpen(uint8_t port, uint32_t baud) {
  uint8_t h = ((F_CPU  / 4 / baud -1) / 2) >> 8;
  uint8_t l = ((F_CPU  / 4 / baud -1) / 2);
  switch (port) {
    #if !defined(PROMICRO)
    case 0: UCSR0A  = (1<<U2X0); UBRR0H = h; UBRR0L = l; UCSR0B |= (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0); break;
    #else
      #if (ARDUINO > 100) && !defined(TEENSY20)
        case 0: UDIEN &= ~(1<<SOFE); // disable the USB frame interrupt of arduino (it causes strong jitter and we dont need it)
      #endif
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

static void inline store_uart_in_buf(uint8_t data, uint8_t portnum) {
  /* the received data byte */
  uint8_t d = data;
  uint8_t i = serialHeadRX[portnum];
  i++;
  if (i == RX_BUFFER_SIZE) {
    i = 0;
  }
  /* we did not bite our own tail? */
  if (i != serialTailRX[portnum]) {
   serialBufferRX[serialHeadRX[portnum]][portnum] = d;
   serialHeadRX[portnum] = i;
  } else {
    /* sorry, this byte is lost :-/ */
  }
}

#if defined(PROMINI) && !(defined(SPEKTRUM))
ISR(USART_RX_vect){
  /* gcc hopefully inlines this */
  store_uart_in_buf(UDR0, 0);
}
#endif

#if (defined(MEGA) || defined(PROMICRO)) && !defined(SPEKTRUM)
  ISR(USART1_RX_vect){
    store_uart_in_buf(UDR1, 1);
  }
#endif
#if defined(MEGA)
  ISR(USART0_RX_vect){
    store_uart_in_buf(UDR0, 0);
  }
  ISR(USART2_RX_vect){
    store_uart_in_buf(UDR2, 2);
  }
  ISR(USART3_RX_vect){
    store_uart_in_buf(UDR3, 3);
  }
#endif

uint8_t SerialRead(uint8_t port) {
  #if defined(PROMICRO)
     #if defined(TEENSY20)
      if(port == 0) return Serial.read();
    #else
      #if (ARDUINO > 100)
        USB_Flush(USB_CDC_TX);
      #endif
      if(port == 0) return USB_Recv(USB_CDC_RX);      
    #endif
    port = 0;
  #endif
  uint8_t c = serialBufferRX[serialTailRX[port]][port];
  if ((serialHeadRX[port] != serialTailRX[port])) {
    serialTailRX[port]++;
    if (serialTailRX[port] == RX_BUFFER_SIZE) {
      serialTailRX[port] = 0;
    }
  }
  return c;
}

uint8_t SerialAvailable(uint8_t port) {
  #if defined(PROMICRO)
    #if !defined(TEENSY20)
      if(port == 0) return USB_Available(USB_CDC_RX);
    #else
      if(port == 0) return T_USB_Available(USB_CDC_RX);
    #endif
    port = 0;
  #endif
  return (serialHeadRX[port] != serialTailRX[port]);
}

void SerialWrite(uint8_t port,uint8_t c){
 switch (port) {
    case 0: serialize8(c);UartSendData(); break;                 // Serial0 TX is driven via a buffer and a background intterupt
    #if defined(MEGA) || defined(PROMICRO)
    case 1: while (!(UCSR1A & (1 << UDRE1))) ; UDR1 = c; break;  // Serial1 Serial2 and Serial3 TX are not driven via interrupts
    #endif
    #if defined(MEGA)
    case 2: while (!(UCSR2A & (1 << UDRE2))) ; UDR2 = c; break;
    case 3: while (!(UCSR3A & (1 << UDRE3))) ; UDR3 = c; break;
    #endif
  }
}
