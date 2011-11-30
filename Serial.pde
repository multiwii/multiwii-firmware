static uint8_t point;
static uint8_t s[200];
void serialize16(int16_t a) {s[point++]  = a; s[point++]  = a>>8&0xff;}
void serialize8(uint8_t a)  {s[point++]  = a;}

// ***********************************
// Interrupt driven UART transmitter for MIS_OSD
// ***********************************
static uint8_t tx_ptr;
static uint8_t tx_busy = 0;

ISR_UART {
  UDR0 = s[tx_ptr++];           /* Transmit next byte */
  if ( tx_ptr == point ) {      /* Check if all data is transmitted */
    UCSR0B &= ~(1<<UDRIE0);     /* Disable transmitter UDRE interrupt */
    tx_busy = 0;
  }
}

void UartSendData() {          // start of the data block transmission
  cli();
  tx_ptr = 0;
  UCSR0A |= (1<<UDRE0);        /* Clear UDRE interrupt flag */
  UCSR0B |= (1<<UDRIE0);       /* Enable transmitter UDRE interrupt */
  UDR0 = s[tx_ptr++];          /* Start transmission */
  tx_busy = 1;
  sei();
}

void serialCom() {
  int16_t a;
  uint8_t i;

  uint16_t intPowerMeterSum, intPowerTrigger1;   

  if ((!tx_busy) && Serial.available()) {
    switch (Serial.read()) {
    #ifdef BTSERIAL
    case 'K': //receive RC data from Bluetooth Serial adapter as a remote
      rcData[THROTTLE] = (Serial.read() * 4) + 1000;
      rcData[ROLL]     = (Serial.read() * 4) + 1000;
      rcData[PITCH]    = (Serial.read() * 4) + 1000;
      rcData[YAW]      = (Serial.read() * 4) + 1000;
      rcData[AUX1]     = (Serial.read() * 4) + 1000;
      break;
    #endif
    #ifdef LCD_TELEMETRY
    case 'A': // button A press
      if (telemetry=='A') telemetry = 0; else { telemetry = 'A'; LCDprint(12); /* clear screen */ }
      break;    
    case 'B': // button B press
      if (telemetry=='B') telemetry = 0; else { telemetry = 'B'; LCDprint(12); /* clear screen */ }
      break;    
    case 'C': // button C press
      if (telemetry=='C') telemetry = 0; else { telemetry = 'C'; LCDprint(12); /* clear screen */ }
      break;    
    case 'D': // button D press
      if (telemetry=='D') telemetry = 0; else { telemetry = 'D'; LCDprint(12); /* clear screen */ }
      break;
    case 'a': // button A release
    case 'b': // button B release
    case 'c': // button C release
    case 'd': // button D release
      break;      
    #endif
    case 'M': // Multiwii @ arduino to GUI all data
      point=0;
      serialize8('M');
      serialize8(VERSION);  // MultiWii Firmware version
      for(i=0;i<3;i++) serialize16(accSmooth[i]);
      for(i=0;i<3;i++) serialize16(gyroData[i]/8);
      for(i=0;i<3;i++) serialize16(magADC[i]/3);
      serialize16(EstAlt/10);
      serialize16(heading); // compass
      for(i=0;i<4;i++) serialize16(servo[i]);
      for(i=0;i<8;i++) serialize16(motor[i]);
      for(i=0;i<8;i++) serialize16(rcData[i]);
      serialize8(nunchuk|ACC<<1|BARO<<2|MAG<<3|GPSPRESENT<<4);
      serialize8(accMode|baroMode<<1|magMode<<2|(GPSModeHome|GPSModeHold)<<3);
      serialize16(cycleTime);
      for(i=0;i<2;i++) serialize16(angle[i]/10);
      serialize8(MULTITYPE);
      for(i=0;i<5;i++) {serialize8(P8[i]);serialize8(I8[i]);serialize8(D8[i]);}
      serialize8(P8[PIDLEVEL]);
      serialize8(I8[PIDLEVEL]);
      serialize8(P8[PIDMAG]);
      serialize8(rcRate8);
      serialize8(rcExpo8);
      serialize8(rollPitchRate);
      serialize8(yawRate);
      serialize8(dynThrPID);
      for(i=0;i<CHECKBOXITEMS;i++) {serialize8(activate1[i]);serialize8(activate2[i]);}
      serialize16(GPS_distanceToHome);
      serialize16(GPS_directionToHome);
      serialize8(GPS_numSat);
      serialize8(GPS_fix);
      serialize8(GPS_update);
      #if defined(POWERMETER)
        intPowerMeterSum = (pMeter[PMOTOR_SUM]/PLEVELDIV);
        intPowerTrigger1 = powerTrigger1 * PLEVELSCALE; 
        serialize16(intPowerMeterSum);
        serialize16(intPowerTrigger1);
      #else
        serialize16(0);serialize16(0);
      #endif
      serialize8(vbat);
      serialize16(BaroAlt/10);        // 4 variables are here for general monitoring purpose
      serialize16(i2c_errors_count);  // debug2
      serialize16(0);                 // debug3
      serialize16(0);                 // debug4
      serialize8('M');
      UartSendData(); // Serial.write(s,point);
      break;
    case 'O':  // arduino to OSD data - contribution from MIS
      point=0;
      serialize8('O');
      for(i=0;i<3;i++) serialize16(accSmooth[i]);
      for(i=0;i<3;i++) serialize16(gyroData[i]);
      serialize16(EstAlt*10.0f);
      serialize16(heading); // compass - 16 bytes
      for(i=0;i<2;i++) serialize16(angle[i]); //20
      for(i=0;i<6;i++) serialize16(motor[i]); //32
      for(i=0;i<6;i++) {serialize16(rcData[i]);} //44
      serialize8(nunchuk|ACC<<1|BARO<<2|MAG<<3);
      serialize8(accMode|baroMode<<1|magMode<<2);
      serialize8(vbat);     // Vbatt 47
      serialize8(VERSION);  // MultiWii Firmware version
      serialize8('O'); //49
      UartSendData();
      break;
    case 'W': //GUI write params to eeprom @ arduino
      while (Serial.available()<(25+2*CHECKBOXITEMS)) {}
      for(i=0;i<5;i++) {P8[i]= Serial.read(); I8[i]= Serial.read(); D8[i]= Serial.read();} //15
      P8[PIDLEVEL] = Serial.read(); I8[PIDLEVEL] = Serial.read(); //17
      P8[PIDMAG] = Serial.read(); //18
      rcRate8 = Serial.read(); rcExpo8 = Serial.read(); //20
      rollPitchRate = Serial.read(); yawRate = Serial.read(); //22
      dynThrPID = Serial.read(); //23
      for(i=0;i<CHECKBOXITEMS;i++) {activate1[i] = Serial.read();activate2[i] = Serial.read();}
     #if defined(POWERMETER)
      powerTrigger1 = (Serial.read() + 256* Serial.read() ) / PLEVELSCALE; // we rely on writeParams() to compute corresponding pAlarm value
     #else
      Serial.read();Serial.read(); // so we unload the two bytes
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

#if defined(SPEKTRUM)
  // ************************
  // HardwareSerial.cpp copied from Aurduino-022. 
  // There are certain serial interrupts that core takes and won't normally give to a sketch. We need these for Spektrum support. 
  // Therefore, this copy has #ifdef structures changed to supress core code based on MultiWii config.h settings. 
  // Change philosophy: ALL CHANGES are to #ifdefs. NO CHANGES to actual serial code. Goal is to make integrating Arduino-023+ core libraries easier. 
  // ************************
  
  /*
    HardwareSerial.cpp - Hardware serial library for Wiring
    Copyright (c) 2006 Nicholas Zambetti.  All right reserved.
  
    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.
  
    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.
  
    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
    
    Modified 23 November 2006 by David A. Mellis
    Modified 28 September 2010 by Mark Sproul
  */
  
  #include <stdlib.h>
  #include <stdio.h>
  #include <string.h>
  #include <inttypes.h>
  #include "wiring.h"
  #include "wiring_private.h"
  
  // this next line disables the entire HardwareSerial.cpp, 
  // this is so I can support Attiny series and any other chip without a uart
  #if defined(UBRRH) || defined(UBRR0H) || defined(UBRR1H) || defined(UBRR2H) || defined(UBRR3H)
  
  #include "HardwareSerial.h"
  
  // Define constants and variables for buffering incoming serial data.  We're
  // using a ring buffer (I think), in which rx_buffer_head is the index of the
  // location to which to write the next incoming character and rx_buffer_tail
  // is the index of the location from which to read.
  #if (RAMEND < 1000)
    #define RX_BUFFER_SIZE 32
  #else
    #define RX_BUFFER_SIZE 128
  #endif
  
  struct ring_buffer
  {
    unsigned char buffer[RX_BUFFER_SIZE];
    int head;
    int tail;
  };
  
  #if defined(UBRRH) || defined(UBRR0H)
    ring_buffer rx_buffer  =  { { 0 }, 0, 0 };
  #endif
  #if defined(UBRR1H)
    ring_buffer rx_buffer1  =  { { 0 }, 0, 0 };
  #endif
  #if defined(UBRR2H)
    ring_buffer rx_buffer2  =  { { 0 }, 0, 0 };
  #endif
  #if defined(UBRR3H)
    ring_buffer rx_buffer3  =  { { 0 }, 0, 0 };
  #endif
  
  inline void store_char(unsigned char c, ring_buffer *rx_buffer)
  {
    int i = (unsigned int)(rx_buffer->head + 1) % RX_BUFFER_SIZE;
  
    // if we should be storing the received character into the location
    // just before the tail (meaning that the head would advance to the
    // current location of the tail), we're about to overflow the buffer
    // and so we don't write the character or advance the head.
    if (i != rx_buffer->tail) {
      rx_buffer->buffer[rx_buffer->head] = c;
      rx_buffer->head = i;
    }
  }
  
  #if defined(USART_RX_vect) && !(defined(SPEKTRUM) && defined(PROMINI))
    SIGNAL(USART_RX_vect)
    {
    #if defined(UDR0)
      unsigned char c  =  UDR0;
    #elif defined(UDR)
      unsigned char c  =  UDR;  //  atmega8535
    #else
      #error UDR not defined
    #endif
      store_char(c, &rx_buffer);
    }
  #elif defined(SIG_USART0_RECV) && defined(UDR0) 
    SIGNAL(SIG_USART0_RECV)
    {
      unsigned char c  =  UDR0;
      store_char(c, &rx_buffer);
    }
  #elif defined(SIG_UART0_RECV) && defined(UDR0)
    SIGNAL(SIG_UART0_RECV)
    {
      unsigned char c  =  UDR0;
      store_char(c, &rx_buffer);
    }
  //#elif defined(SIG_USART_RECV)
  #elif defined(USART0_RX_vect)
    // fixed by Mark Sproul this is on the 644/644p
    //SIGNAL(SIG_USART_RECV)
    SIGNAL(USART0_RX_vect)
    {
    #if defined(UDR0)
      unsigned char c  =  UDR0;
    #elif defined(UDR)
      unsigned char c  =  UDR;  //  atmega8, atmega32
    #else
      #error UDR not defined
    #endif
      store_char(c, &rx_buffer);
    }
  #elif defined(SIG_UART_RECV)
    // this is for atmega8
    SIGNAL(SIG_UART_RECV)
    {
    #if defined(UDR0)
      unsigned char c  =  UDR0;  //  atmega645
    #elif defined(UDR)
      unsigned char c  =  UDR;  //  atmega8
    #endif
      store_char(c, &rx_buffer);
    }
  #elif defined(USBCON)
    #warning No interrupt handler for usart 0
    #warning Serial(0) is on USB interface
  #else
    #if !(defined(SPEKTRUM) && defined(PROMINI))
      #error No interrupt handler for usart 0
    #endif
  #endif
  
  //#if defined(SIG_USART1_RECV)
  #if defined(USART1_RX_vect) && !(defined(SPEKTRUM) && defined(MEGA))
    //SIGNAL(SIG_USART1_RECV)
    SIGNAL(USART1_RX_vect)
    {
      unsigned char c = UDR1;
      store_char(c, &rx_buffer1);
    }
  #elif defined(SIG_USART1_RECV) && !(defined(SPEKTRUM) && defined(MEGA))
    #error SIG_USART1_RECV
  #endif
  
  #if defined(USART2_RX_vect) && defined(UDR2)
    SIGNAL(USART2_RX_vect)
    {
      unsigned char c = UDR2;
      store_char(c, &rx_buffer2);
    }
  #elif defined(SIG_USART2_RECV)
    #error SIG_USART2_RECV
  #endif
  
  #if defined(USART3_RX_vect) && defined(UDR3)
    SIGNAL(USART3_RX_vect)
    {
      unsigned char c = UDR3;
      store_char(c, &rx_buffer3);
    }
  #elif defined(SIG_USART3_RECV)
    #error SIG_USART3_RECV
  #endif
  
  
  
  // Constructors ////////////////////////////////////////////////////////////////
  
  HardwareSerial::HardwareSerial(ring_buffer *rx_buffer,
    volatile uint8_t *ubrrh, volatile uint8_t *ubrrl,
    volatile uint8_t *ucsra, volatile uint8_t *ucsrb,
    volatile uint8_t *udr,
    uint8_t rxen, uint8_t txen, uint8_t rxcie, uint8_t udre, uint8_t u2x)
  {
    _rx_buffer = rx_buffer;
    _ubrrh = ubrrh;
    _ubrrl = ubrrl;
    _ucsra = ucsra;
    _ucsrb = ucsrb;
    _udr = udr;
    _rxen = rxen;
    _txen = txen;
    _rxcie = rxcie;
    _udre = udre;
    _u2x = u2x;
  }
  
  // Public Methods //////////////////////////////////////////////////////////////
  
  void HardwareSerial::begin(long baud)
  {
    uint16_t baud_setting;
    bool use_u2x = true;
  
  #if F_CPU == 16000000UL
    // hardcoded exception for compatibility with the bootloader shipped
    // with the Duemilanove and previous boards and the firmware on the 8U2
    // on the Uno and Mega 2560.
    if (baud == 57600) {
      use_u2x = false;
    }
  #endif
    
    if (use_u2x) {
      *_ucsra = 1 << _u2x;
      baud_setting = (F_CPU / 4 / baud - 1) / 2;
    } else {
      *_ucsra = 0;
      baud_setting = (F_CPU / 8 / baud - 1) / 2;
    }
  
    // assign the baud_setting, a.k.a. ubbr (USART Baud Rate Register)
    *_ubrrh = baud_setting >> 8;
    *_ubrrl = baud_setting;
  
    sbi(*_ucsrb, _rxen);
    sbi(*_ucsrb, _txen);
    sbi(*_ucsrb, _rxcie);
  }
  
  void HardwareSerial::end()
  {
    cbi(*_ucsrb, _rxen);
    cbi(*_ucsrb, _txen);
    cbi(*_ucsrb, _rxcie);  
  }
  
  int HardwareSerial::available(void)
  {
    return (unsigned int)(RX_BUFFER_SIZE + _rx_buffer->head - _rx_buffer->tail) % RX_BUFFER_SIZE;
  }
  
  int HardwareSerial::peek(void)
  {
    if (_rx_buffer->head == _rx_buffer->tail) {
      return -1;
    } else {
      return _rx_buffer->buffer[_rx_buffer->tail];
    }
  }
  
  int HardwareSerial::read(void)
  {
    // if the head isn't ahead of the tail, we don't have any characters
    if (_rx_buffer->head == _rx_buffer->tail) {
      return -1;
    } else {
      unsigned char c = _rx_buffer->buffer[_rx_buffer->tail];
      _rx_buffer->tail = (unsigned int)(_rx_buffer->tail + 1) % RX_BUFFER_SIZE;
      return c;
    }
  }
  
  void HardwareSerial::flush()
  {
    // don't reverse this or there may be problems if the RX interrupt
    // occurs after reading the value of rx_buffer_head but before writing
    // the value to rx_buffer_tail; the previous value of rx_buffer_head
    // may be written to rx_buffer_tail, making it appear as if the buffer
    // don't reverse this or there may be problems if the RX interrupt
    // occurs after reading the value of rx_buffer_head but before writing
    // the value to rx_buffer_tail; the previous value of rx_buffer_head
    // may be written to rx_buffer_tail, making it appear as if the buffer
    // were full, not empty.
    _rx_buffer->head = _rx_buffer->tail;
  }
  
  void HardwareSerial::write(uint8_t c)
  {
    while (!((*_ucsra) & (1 << _udre)))
      ;
  
    *_udr = c;
  }
  
  // Preinstantiate Objects //////////////////////////////////////////////////////
  
  #if defined(UBRRH) && defined(UBRRL)
    HardwareSerial Serial(&rx_buffer, &UBRRH, &UBRRL, &UCSRA, &UCSRB, &UDR, RXEN, TXEN, RXCIE, UDRE, U2X);
  #elif defined(UBRR0H) && defined(UBRR0L)
    HardwareSerial Serial(&rx_buffer, &UBRR0H, &UBRR0L, &UCSR0A, &UCSR0B, &UDR0, RXEN0, TXEN0, RXCIE0, UDRE0, U2X0);
  #elif defined(USBCON)
    #warning no serial port defined  (port 0)
  #else
    #error no serial port defined  (port 0)
  #endif
  
  #if defined(UBRR1H)
    HardwareSerial Serial1(&rx_buffer1, &UBRR1H, &UBRR1L, &UCSR1A, &UCSR1B, &UDR1, RXEN1, TXEN1, RXCIE1, UDRE1, U2X1);
  #endif
  #if defined(UBRR2H)
    HardwareSerial Serial2(&rx_buffer2, &UBRR2H, &UBRR2L, &UCSR2A, &UCSR2B, &UDR2, RXEN2, TXEN2, RXCIE2, UDRE2, U2X2);
  #endif
  #if defined(UBRR3H)
    HardwareSerial Serial3(&rx_buffer3, &UBRR3H, &UBRR3L, &UCSR3A, &UCSR3B, &UDR3, RXEN3, TXEN3, RXCIE3, UDRE3, U2X3);
  #endif
  
  #endif // whole file

#endif
