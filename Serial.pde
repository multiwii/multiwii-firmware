static uint8_t point;
static uint8_t s[128];
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
      for(i=0;i<3;i++) serialize16(gyroData[i]/8); //13
      for(i=0;i<3;i++) serialize16(magADC[i]/3); //19
      serialize16(BaroAlt*100.0f);
      serialize16(heading); // compass
      for(i=0;i<4;i++) serialize16(servo[i]); //31
      for(i=0;i<6;i++) serialize16(motor[i]); //43
      for(i=0;i<8;i++) serialize16(rcData[i]); //49
      serialize8(nunchuk|ACC<<1|BARO<<2|MAG<<3);
      serialize8(accMode|baroMode<<1|magMode<<2);
      serialize16(cycleTime);
      for(i=0;i<2;i++) serialize16(angle[i]/10); //67
      serialize8(MULTITYPE);
      for(i=0;i<5;i++) {serialize8(P8[i]);serialize8(I8[i]);serialize8(D8[i]);} //80
      serialize8(P8[PIDLEVEL]);serialize8(I8[PIDLEVEL]);
      serialize8(P8[PIDMAG]);
      serialize8(rcRate8); serialize8(rcExpo8);
      serialize8(rollPitchRate); serialize8(yawRate);
      serialize8(dynThrPID); //88
      for(i=0;i<6;i++) serialize8(activate[i]); //94
      #if defined(POWERMETER)
        intPowerMeterSum = (pMeter[PMOTOR_SUM]/PLEVELDIV);
        intPowerTrigger1 = powerTrigger1 * PLEVELSCALE;
      #endif    
      serialize16(intPowerMeterSum);
      serialize16(intPowerTrigger1);
      serialize8(vbat); //99
      serialize16(EstAlt*100.0f);
      serialize8('M'); //100
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
      while (Serial.available()<29) {}
      for(i=0;i<5;i++) {P8[i]= Serial.read(); I8[i]= Serial.read(); D8[i]= Serial.read();} //9
      P8[PIDLEVEL] = Serial.read(); I8[PIDLEVEL] = Serial.read(); //11
      P8[PIDMAG] = Serial.read();
      rcRate8 = Serial.read(); rcExpo8 = Serial.read();
      rollPitchRate = Serial.read(); yawRate = Serial.read(); //16
      dynThrPID = Serial.read();
      for(i=0;i<6;i++) activate[i] = Serial.read(); //22
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
