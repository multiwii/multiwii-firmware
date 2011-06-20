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

#ifdef LCD_TELEMETRY
  // LCD_BAR(n,v) : draw a bar graph - n number of chars for width, v value in % to display
  #ifdef LCD_TEXTSTAR
    #define LCD_BAR(n,v) { LCDprint(0xFE);LCDprint('b');LCDprint(n);LCDprint(v); }
  #else 
    #define LCD_BAR(n,v) {} // add your own implementation here
  #endif
  char line1[17],line2[17];
  static uint8_t telecycle = 0;
  
  if (++telecycle > TELEMETRY_CYCLES) {
    telecycle = 0;

    switch (telemetry) { // output telemetry data, if one of four modes is set
      case 'C': // button C on Textstar LCD -> cycle time
        strcpy(line1,"Cycle    _____us"); //uin16_t cycleTime
        /*            0123456789.12345*/
        strcpy(line2,"[_____, _____]us"); //uin16_t cycleTimeMax
        line1[9] = '0' + cycleTime / 10000;
        line1[10] = '0' + cycleTime / 1000 - (cycleTime/10000) * 10;
        line1[11] = '0' + cycleTime / 100  - (cycleTime/1000)  * 10;
        line1[12] = '0' + cycleTime / 10   - (cycleTime/100)   * 10;
        line1[13] = '0' + cycleTime        - (cycleTime/10)    * 10;
      #ifdef LOG_VALUES
        line2[1] = '0' + cycleTimeMin / 10000;
        line2[2] = '0' + cycleTimeMin / 1000 - (cycleTimeMin/10000) * 10;
        line2[3] = '0' + cycleTimeMin / 100  - (cycleTimeMin/1000)  * 10;
        line2[4] = '0' + cycleTimeMin / 10   - (cycleTimeMin/100)   * 10;
        line2[5] = '0' + cycleTimeMin        - (cycleTimeMin/10)    * 10;
        line2[8] = '0' + cycleTimeMax / 10000;
        line2[9] = '0' + cycleTimeMax / 1000 - (cycleTimeMax/10000) * 10;
        line2[10] = '0' + cycleTimeMax / 100  - (cycleTimeMax/1000)  * 10;
        line2[11] = '0' + cycleTimeMax / 10   - (cycleTimeMax/100)   * 10;
        line2[12] = '0' + cycleTimeMax        - (cycleTimeMax/10)    * 10;
        LCDprint(0xFE);LCDprint('L');LCDprint(2);LCDprintChar(line2); //refresh line 2 of LCD
      #endif
        LCDprint(0xFE);LCDprint('L');LCDprint(1);LCDprintChar(line1); //refresh line 1 of LCD
        break;
      case 'B': // button B on Textstar LCD -> Voltage, PowerSum and power alarm trigger value
        strcpy(line1,"__._V   _____mAh"); //uint8_t vbat, intPowerMeterSum
        /*            0123456789.12345*/
        //strcpy(line2,".......  ......."); // intPowerMeterSum, intPowerTrigger1
      #ifdef VBAT
        line1[0] = '0'+vbat/100; line1[1] = '0'+vbat/10-(vbat/100)*10; line1[3] = '0'+vbat-(vbat/10)*10;
      #endif
      #ifdef POWERMETER
        intPowerMeterSum = (pMeter[6]/PLEVELDIV);
        line1[8] = '0' + intPowerMeterSum / 10000;
        line1[9] = '0' + intPowerMeterSum / 1000 - (intPowerMeterSum/10000) * 10;
        line1[10] = '0' + intPowerMeterSum / 100  - (intPowerMeterSum/1000)  * 10;
        line1[11] = '0' + intPowerMeterSum / 10   - (intPowerMeterSum/100)   * 10;
        line1[12] = '0' + intPowerMeterSum        - (intPowerMeterSum/10)    * 10;
        //line2[13] = '0'+powerTrigger1/100; line2[14] = '0'+powerTrigger1/10-(powerTrigger1/100)*10; line2[15] = '0'+powerTrigger1-(powerTrigger1/10)*10;
      #endif
        LCDprint(0xFE);LCDprint('L');LCDprint(1);LCDprintChar(line1); //refresh line 1 of LCD
        LCDprint(0xFE);LCDprint('L');LCDprint(2); //position on line 2 of LCD
      #ifdef VBAT
        LCD_BAR(7, (((vbat-VBATLEVEL1_3S)*100)/VBATREF) );
        LCDprintChar("  ");
      #endif
      #ifdef POWERMETER  
        //     intPowerMeterSum = (pMeter[6]/PLEVELDIV);
        //   pAlarm = (uint32_t) powerTrigger1 * (uint32_t) PLEVELSCALE * (uint32_t) PLEVELDIV; // need to cast before multiplying
        if (powerTrigger1)
          LCD_BAR(7, (intPowerMeterSum/powerTrigger1 *2) ); // bar graph powermeter (scale intPowerMeterSum/powerTrigger1 with *100/PLEVELSCALE)
      #endif
        break;
      case 'A': // button A on Textstar LCD -> angles 
        uint16_t unit;
        strcpy(line1,"Deg ___._  ___._");
        /*            0123456789.12345*/
        strcpy(line2,"__,__A max__,__A"); //uin16_t cycleTimeMax
        if (angle[0] < 0 ) {
          unit = -angle[0];
          line1[3] = '-';
        } else 
          unit = angle[0];
        //line1[5] = '0' + unit / 10000;
        line1[4] = '0' + unit / 1000; //- (unit/10000) * 10;
        line1[5] = '0' + unit / 100  - (unit/1000)  * 10;
        line1[6] = '0' + unit / 10   - (unit/100)   * 10;
        line1[8] = '0' + unit       - (unit/10)    * 10;
        if (angle[1] < 0 ) {
          unit = -angle[1];
          line1[10] = '-';
        } else 
          unit = angle[1];
        //line2[5] = '0' + unit / 10000;
        line1[11] = '0' + unit / 1000; //- (unit/10000) * 10;
        line1[12] = '0' + unit / 100  - (unit/1000)  * 10;
        line1[13] = '0' + unit / 10   - (unit/100)   * 10;
        line1[15] = '0' + unit       - (unit/10)    * 10;
        #ifdef LOG_VALUES
          unit = powerAvg * PINT2mA;
          line2[0] = '0' + unit / 10000;
          line2[1] = '0' + unit / 1000 - (unit/10000) * 10;
          line2[3] = '0' + unit / 100  - (unit/1000)  * 10;
          line2[4] = '0' + unit / 10   - (unit/100)   * 10;
          unit = powerMax * PINT2mA;
          line2[10] = '0' + unit / 10000;
          line2[11] = '0' + unit / 1000 - (unit/10000) * 10;
          line2[13] = '0' + unit / 100  - (unit/1000)  * 10;
          line2[14] = '0' + unit / 10   - (unit/100)   * 10;
          //line2[13] = '0' + unit        - (unit/10)    * 10;
          LCDprint(0xFE);LCDprint('L');LCDprint(2);LCDprintChar(line2); //refresh line 2 of LCD
        #endif
        LCDprint(0xFE);LCDprint('L');LCDprint(1);LCDprintChar(line1); //refresh line 1 of LCD
        break;    
      case 'D': // button D on Textstar LCD -> sensors
      #define GYROLIMIT 30 // threshold: for larger values replace bar with dots
      #define ACCLIMIT 40 // threshold: for larger values replace bar with dots
        LCDprint(0xFE);LCDprint('L');LCDprint(1);LCDprintChar("G "); //refresh line 1 of LCD
        if (abs(gyroData[0]) < GYROLIMIT) { LCD_BAR(4,(abs(gyroData[0])*100/GYROLIMIT)) } else LCDprintChar("...."); LCDprint(' ');
        if (abs(gyroData[1]) < GYROLIMIT) { LCD_BAR(4,(abs(gyroData[1])*100/GYROLIMIT)) } else LCDprintChar("...."); LCDprint(' ');
        if (abs(gyroData[2]) < GYROLIMIT) { LCD_BAR(4,(abs(gyroData[2])*100/GYROLIMIT)) } else LCDprintChar("....");
        LCDprint(0xFE);LCDprint('L');LCDprint(2);LCDprintChar("A "); //refresh line 2 of LCD
        if (abs(accSmooth[0]) < ACCLIMIT) { LCD_BAR(4,(abs(accSmooth[0])*100/ACCLIMIT)) } else LCDprintChar("...."); LCDprint(' ');
        if (abs(accSmooth[1]) < ACCLIMIT) { LCD_BAR(4,(abs(accSmooth[1])*100/ACCLIMIT)) } else LCDprintChar("...."); LCDprint(' ');
        if (abs(accSmooth[2] - acc_1G) < ACCLIMIT) { LCD_BAR(4,(abs(accSmooth[2]-acc_1G)*100/ACCLIMIT)) } else LCDprintChar("....");
        break; 
    } // end switch (telemetry) 
  } // end if (++telecycle > TELEMETRY_CYCLES)
#endif //  LCD_TELEMETRY
  
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
      serialize16(altitudeSmooth);
      serialize16(heading); // compass
      for(i=0;i<4;i++) serialize16(servo[i]); //31
      for(i=0;i<6;i++) serialize16(motor[i]); //43
      for(i=0;i<8;i++) serialize16(rcData[i]); //49
      serialize8(nunchuk|ACC<<1|BARO<<2|MAG<<3);
      serialize8(accMode|baroMode<<1|magMode<<2);
      serialize16(cycleTime);
      for(i=0;i<2;i++) serialize16(angle[i]/10); //67
    #if defined(TRI)
      serialize8(1);
    #elif defined(QUADP)
      serialize8(2);
    #elif defined(QUADX)
      serialize8(3);
    #elif defined(BI)
      serialize8(4);
    #elif defined(GIMBAL)
      serialize8(5);
    #elif defined(Y6)
      serialize8(6);
    #elif defined(HEX6)
      serialize8(7);
    #elif defined(FLYING_WING)
      serialize8(8);
    #elif defined(Y4)
      serialize8(9);
    #elif defined(HEX6X)
      serialize8(10);
    #elif defined(OCTOX8)
      serialize8(11);
    #elif defined(OCTOFLATP)
      serialize8(11);        //the GUI is the same for all 8 motor configs
    #elif defined(OCTOFLATX)
      serialize8(11);        //the GUI is the same for all 8 motor configs
    #endif
      for(i=0;i<4;i++) {serialize8(P8[i]);serialize8(I8[i]);serialize8(D8[i]);} //80
      serialize8(P8[PIDLEVEL]);serialize8(I8[PIDLEVEL]);
      serialize8(P8[PIDMAG]);
      serialize8(rcRate8); serialize8(rcExpo8);
      serialize8(rollPitchRate); serialize8(yawRate);
      serialize8(dynThrPID); //88
      for(i=0;i<6;i++) serialize8(activate[i]); //94
      #if defined(POWERMETER)
        intPowerMeterSum = (pMeter[6]/PLEVELDIV);
        intPowerTrigger1 = powerTrigger1 * PLEVELSCALE;
      #endif    
      serialize16(intPowerMeterSum);  
      serialize16(intPowerTrigger1);
      serialize8(vbat); //99
      serialize8('M'); //100
      UartSendData(); // Serial.write(s,point);
      break;
    case 'O':  // arduino to OSD data - contribution from MIS
      point=0;
      serialize8('O');
      for(i=0;i<3;i++) serialize16(accSmooth[i]);
      for(i=0;i<3;i++) serialize16(gyroData[i]);
      serialize16(altitudeSmooth);
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
      for(i=0;i<4;i++) {P8[i]= Serial.read(); I8[i]= Serial.read(); D8[i]= Serial.read();} //9
      P8[PIDLEVEL] = Serial.read(); I8[PIDLEVEL] = Serial.read(); //11
      P8[PIDMAG] = Serial.read();
      rcRate8 = Serial.read(); rcExpo8 = Serial.read();
      rollPitchRate = Serial.read(); yawRate = Serial.read(); //16
      dynThrPID = Serial.read();
      for(i=0;i<6;i++) activate[i] = Serial.read(); //22
     #if defined(POWERMETER)
      powerTrigger1 = (Serial.read() + 256* Serial.read() ) / PLEVELSCALE; // we rely on writeParams() to compute corresponding pAlarm value
     #else
      Serial.read();
      Serial.read(); // so we unload the two bytes
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
