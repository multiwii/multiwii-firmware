// ************************************************************************************************************
// LCD & display & monitoring
// ************************************************************************************************************
#ifdef LCD_CONF

typedef void (*formatter_func_ptr)(void *, uint8_t, uint8_t);
typedef void (*inc_func_ptr)(void *, int8_t);

typedef struct lcd_type_desc_t{
  formatter_func_ptr fmt;
  inc_func_ptr inc;
};  

static lcd_type_desc_t LTU8  = {&__u8Fmt,  &__u8Inc};
static lcd_type_desc_t LTU16 = {&__u16Fmt, &__u16Inc};
static lcd_type_desc_t LPMM  = {&__upMFmt, &__nullInc};
static lcd_type_desc_t LPMS  = {&__upSFmt, &__nullInc};

typedef struct lcd_param_def_t{
  lcd_type_desc_t * type;
  uint8_t  decimal;
  uint8_t  multiplier;  
  uint8_t  increment;
};

typedef struct lcd_param_t{
  char*  paramText;
  void * var; 
  lcd_param_def_t * def;
};

// ************************************************************************************************************
// LCD Layout definition
// ************************************************************************************************************
// Descriptors
static lcd_param_def_t __P   = {&LTU8,  1, 1, 1};
static lcd_param_def_t __I   = {&LTU8,  3, 1, 2};
static lcd_param_def_t __D   = {&LTU8,  0, 1, 1};
static lcd_param_def_t __RCR = {&LTU8,  2, 2, 2};
static lcd_param_def_t __RC  = {&LTU8,  2, 1, 2};
static lcd_param_def_t __PM  = {&LPMM,  1, 1, 0};
static lcd_param_def_t __PS  = {&LPMS,  1, 1, 0};
static lcd_param_def_t __PT  = {&LTU8,  0, 1, 1};
static lcd_param_def_t __VB  = {&LTU8,  1, 1, 0};
// Parameters
static lcd_param_t lcd_param[] = {
  {"PITCH&ROLL P",    &P8[ROLL],      &__P}
, {"ROLL   P",        &P8[ROLL],      &__P}, {"ROLL   I", &I8[ROLL],   &__I}, {"ROLL   D", &D8[ROLL],   &__D}
, {"PITCH  P",        &P8[PITCH],     &__P}, {"PITCH  I", &I8[PITCH],  &__I}, {"PITCH  D", &D8[PITCH],  &__D}
, {"YAW    P",        &P8[YAW],       &__P}, {"YAW    I", &I8[YAW],    &__I}, {"YAW    D", &D8[YAW],    &__D}
#if BARO
, {"ALT    P",        &P8[PIDALT],    &__P}, {"ALT    I", &I8[PIDALT], &__I}, {"ALT    D", &D8[PIDALT], &__D}
, {"VEL    P",        &P8[PIDVEL],    &__P}, {"VEL    I", &I8[PIDVEL], &__I}, {"VEL    D", &D8[PIDVEL], &__D}
#endif  
, {"LEVEL  P",        &P8[PIDLEVEL],  &__P}, {"LEVEL  I", &I8[PIDLEVEL],&__I}
#if MAG
, {"MAG    P",        &P8[PIDMAG],    &__P}
#endif  
, {"RC RATE",         &rcRate8,       &__RCR}
, {"RC EXPO",         &rcExpo8,       &__RC}
, {"PITCH&ROLL RATE", &rollPitchRate, &__RC}
, {"YAW RATE",        &yawRate,       &__RC}
, {"THROTTLE PID",    &dynThrPID,     &__RC}
#ifdef POWERMETER
, {"pMeter Motor 0",  &pMeter[0],     &__PM}, {"pMeter Motor 1", &pMeter[1], &__PM}, {"pMeter Motor 2", &pMeter[2], &__PM}
   #if (NUMBER_MOTOR > 3)
   , {"pMeter Motor 3",  &pMeter[3],     &__PM}
   #endif
   #if (NUMBER_MOTOR > 4)
   , {"pMeter Motor 4", &pMeter[4], &__PM}, {"pMeter Motor 5", &pMeter[5], &__PM}
   #endif
   #if (NUMBER_MOTOR > 6)
   , {"pMeter Motor 6", &pMeter[6], &__PM}, {"pMeter Motor 7", &pMeter[7], &__PM}
   #endif
, {"pMeter Sum",      &pMeter[PMOTOR_SUM],     &__PS}
, {"pAlarm /50",      &powerTrigger1, &__PT} // change text to represent PLEVELSCALE value
#endif
#ifdef VBAT
, {"Battery Volt",    &vbat,          &__VB} 
#endif
};
#define PARAMMAX (sizeof(lcd_param)/sizeof(lcd_param_t) - 1)
// ************************************************************************************************************

// 1000000 / 9600  = 104 microseconds at 9600 baud.
// we set it below to take some margin with the running interrupts
#define BITDELAY 102
void LCDprint(uint8_t i) {
  #if defined(LCD_TEXTSTAR)
    Serial.print( i , BYTE);
  #else
    LCDPIN_OFF
    delayMicroseconds(BITDELAY);
    for (uint8_t mask = 0x01; mask; mask <<= 1) {
      if (i & mask) LCDPIN_ON else LCDPIN_OFF // choose bit
      delayMicroseconds(BITDELAY);
    }
    LCDPIN_ON //switch ON digital PIN 0
    delayMicroseconds(BITDELAY);
  #endif
}

void LCDprintChar(const char *s) {
  while (*s) LCDprint(*s++);
}

void initLCD() {
  blinkLED(20,30,1);
  #if defined(LCD_TEXTSTAR)
    // Cat's Whisker Technologies 'TextStar' Module CW-LCD-02
    // http://cats-whisker.com/resources/documents/cw-lcd-02_datasheet.pdf
    // Modified by Luca Brizzi aka gtrick90 @ RCG
    LCDprint(0xFE);LCDprint(0x43);LCDprint(0x02); //cursor blink mode
    LCDprint(0x0c); //clear screen
    LCDprintChar("MultiWii Config");
    LCDprint(0x0d); // carriage return
    LCDprintChar("for all params");
    delay(2500);
    LCDprint(0x0c); //clear screen
  #else
    Serial.end();
    //init LCD
    PINMODE_LCD //TX PIN for LCD = Arduino RX PIN (more convenient to connect a servo plug on arduino pro mini)
  #endif
}

static char line1[17],line2[17];

void __u8Inc(void * var, int8_t inc) {*(uint8_t*)var += inc;};
void __u16Inc(void * var, int8_t inc) {*(uint16_t*)var += inc;};
void __nullInc(void * var, int8_t inc) {};

void __u8Fmt(void * var, uint8_t mul, uint8_t dec) {
  uint16_t unit = *(uint8_t*)var;
  unit *= mul; 
  char c1 = '0'+unit/100; char c2 = '0'+unit/10-(unit/100)*10; char c3 = '0'+unit-(unit/10)*10;
  switch (dec) {
    case 0: line2[6] = c1;  line2[7] = c2;   line2[8] = c3; break;
    case 1: line2[5] = c1;  line2[6] = c2;   line2[7] = '.'; line2[8] = c3; break;
    case 2: line2[5] = c1;  line2[6] = '.';  line2[7] = c2;  line2[8] = c3; break;
    case 3: line2[4] = '0'; line2[5] = '.';  line2[6] = c1;  line2[7] = c2; line2[8] = c3; break;
  }
}  

void __u16Fmt(void * var, uint8_t mul, uint8_t dec) {
  uint16_t unit = *(uint16_t*)var;
  unit *= mul; 
  line2[4] = '0' + unit / 10000;
  line2[5] = '0' + unit / 1000 - (unit/10000) * 10;
  line2[6] = '0' + unit / 100  - (unit/1000)  * 10;
  line2[7] = '0' + unit / 10   - (unit/100)   * 10;
  line2[8] = '0' + unit        - (unit/10)    * 10;
}  

void __upMFmt(void * var, uint8_t mul, uint8_t dec) {
  uint32_t unit = *(uint32_t*)var; 
  // pmeter values need special treatment, too many digits to fit standard 8 bit scheme
  unit = unit / PLEVELDIVSOFT; // [0:1000] * 1000/3 samples per second(loop time) * 60 seconds *5 minutes -> [0:10000 e4] per motor
                               // (that is full throttle for 5 minutes sampling with high sampling rate for wmp only)
                               // times 6 for a maximum of 6 motors equals [0:60000 e4] for the sum
                               // we are only interested in the big picture, so divide by 10.000
  __u16Fmt(&unit, mul, dec);
}  

void __upSFmt(void * var, uint8_t mul, uint8_t dec) {
  uint32_t unit = *(uint32_t*)var; 
  #if (POWERMETER == 1)
    unit = unit / PLEVELDIVSOFT; 
  #else
    unit = unit / PLEVELDIV;
  #endif
  __u16Fmt(&unit, mul, dec);
}  

void configurationLoop() {
  uint8_t chan,i;
  uint8_t p,stickActive = 1;
  uint8_t val;
  uint8_t LCD=1;
  uint8_t refreshLCD = 1;
  uint8_t key = 0;
  initLCD();
  p = 0;
  while (LCD == 1) {
    if (refreshLCD == 1) {
      strcpy(line2,"                ");
      strcpy(line1,"                ");
      i=0; char* point = lcd_param[p].paramText; while (*point) line1[i++] = *point++;
      lcd_param[p].def->type->fmt(lcd_param[p].var, lcd_param[p].def->multiplier, lcd_param[p].def->decimal);
      #if defined(LCD_TEXTSTAR)
        LCDprint(0xFE);LCDprint('L');LCDprint(1);LCDprintChar(line1); //refresh line 1 of LCD
        LCDprint(0xFE);LCDprint('L');LCDprint(2);LCDprintChar(line2); //refresh line 2 of LCD
      #else
        LCDprint(0xFE);LCDprint(128);LCDprintChar(line1);
        LCDprint(0xFE);LCDprint(192);LCDprintChar(line2);
      #endif
      refreshLCD=0;
    }
    #if defined(LCD_TEXTSTAR)
      key = ( Serial.available() ?  Serial.read() : 0 ); 
    #endif
    for (chan = ROLL; chan < 4; chan++) rcData[chan] = readRawRC(chan);
    if (rcData[YAW]  < MINCHECK && rcData[PITCH] > MAXCHECK && stickActive == 0) LCD = 0; // save and exit
    if (rcData[YAW]  > MAXCHECK && rcData[PITCH] > MAXCHECK && stickActive == 0) LCD = 2; // exit without save: eeprom has only 100.000 write cycles
    //switch config param with pitch
    if ((key == LCD_MENU_NEXT || (rcData[PITCH] < MINCHECK && stickActive == 0)) && p<= PARAMMAX) {
      stickActive = 1;refreshLCD=1;blinkLED(10,20,1);
      p++;
      if (p>PARAMMAX) p=0;
    }
    if ((key == LCD_MENU_PREV || (rcData[PITCH] > MAXCHECK && stickActive == 0)) && p>=0) {
      stickActive = 1;refreshLCD=1;blinkLED(10,20,1);
      if (p==0) p=PARAMMAX; else p--;
    }
    //+ or - param with low and high roll
    if ((key == LCD_VALUE_DOWN || (rcData[ROLL] < MINCHECK && stickActive == 0))) {
      stickActive = 1;refreshLCD=1;blinkLED(10,20,1);
      lcd_param[p].def->type->inc(lcd_param[p].var, -lcd_param[p].def->increment);
      if (p == 0) memcpy(lcd_param[4].var, lcd_param[0].var, 1);
    }
    if ((key == LCD_VALUE_UP || (rcData[ROLL] > MAXCHECK && stickActive == 0))) {
      stickActive = 1;refreshLCD=1;blinkLED(10,20,1);
      lcd_param[p].def->type->inc(lcd_param[p].var, +lcd_param[p].def->increment);
      if (p == 0) memcpy(lcd_param[4].var, lcd_param[0].var, 1);
    }
    if (rcData[ROLL] < MAXCHECK && rcData[ROLL]  > MINCHECK
        && rcData[PITCH] < MAXCHECK && rcData[PITCH] > MINCHECK) stickActive = 0;
  } // while (LCD == 1)
  blinkLED(20,30,1);
  #if defined(LCD_TEXTSTAR)
    LCDprint(0x0c); //clear screen
    if ( LCD == 0) LCDprintChar("Saving Settings.."); else LCDprintChar("skipping Save.");
  #endif
  if ( LCD == 0) writeParams();
  #if defined(LCD_TEXTSTAR)
    LCDprintChar("..done! Exit.");
  #else
    Serial.begin(SERIAL_COM_SPEED);
  #endif
  #ifdef LCD_TELEMETRY
    delay(1000); // keep exit message visible for one second even if (auto)telemetry continues writing in main loop
  #endif
}
#endif


// -------------------- telemtry output to LCD over serial ----------------------------------

#ifdef LCD_TELEMETRY
void lcd_telemetry() {
  // LCD_BAR(n,v) : draw a bar graph - n number of chars for width, v value in % to display
  #ifdef LCD_TEXTSTAR
    #define LCD_BAR(n,v) { LCDprint(0xFE);LCDprint('b');LCDprint(n);LCDprint(v); }
  #else 
    #define LCD_BAR(n,v) {} // add your own implementation here
  #endif
  uint16_t intPowerMeterSum;   

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
      //    (line2,".......  ......."); // intPowerMeterSum, intPowerTrigger1
    #ifdef VBAT
      line1[0] = '0'+vbat/100; line1[1] = '0'+vbat/10-(vbat/100)*10; line1[3] = '0'+vbat-(vbat/10)*10;
    #endif
    #ifdef POWERMETER
      intPowerMeterSum = (pMeter[PMOTOR_SUM]/PLEVELDIV);
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
      //     intPowerMeterSum = (pMeter[PMOTOR_SUM]/PLEVELDIV);
      //   pAlarm = (uint32_t) powerTrigger1 * (uint32_t) PLEVELSCALE * (uint32_t) PLEVELDIV; // need to cast before multiplying
      if (powerTrigger1)
        LCD_BAR(7, (intPowerMeterSum/powerTrigger1 *2) ); // bar graph powermeter (scale intPowerMeterSum/powerTrigger1 with *100/PLEVELSCALE)
    #endif
      break;
    case 'A': // button A on Textstar LCD -> angles 
      uint16_t unit;
      strcpy(line1,"Deg ___._  ___._");
      /*            0123456789.12345*/
      strcpy(line2,"___,_A max___,_A"); //uin16_t cycleTimeMax
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
      //line1[5] = '0' + unit / 10000;
      line1[11] = '0' + unit / 1000; //- (unit/10000) * 10;
      line1[12] = '0' + unit / 100  - (unit/1000)  * 10;
      line1[13] = '0' + unit / 10   - (unit/100)   * 10;
      line1[15] = '0' + unit       - (unit/10)    * 10;
      #ifdef LOG_VALUES
        unit = powerAvg * PINT2mA;
        line2[0] = '0' + unit / 10000;
        line2[1] = '0' + unit / 1000 - (unit/10000) * 10;
        line2[2] = '0' + unit / 100  - (unit/1000)  * 10;
        line2[4] = '0' + unit / 10   - (unit/100)   * 10;
        unit = powerMax * PINT2mA;
        line2[10] = '0' + unit / 10000;
        line2[11] = '0' + unit / 1000 - (unit/10000) * 10;
        line2[12] = '0' + unit / 100  - (unit/1000)  * 10;
        line2[14] = '0' + unit / 10   - (unit/100)   * 10;
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
} // end function
#endif //  LCD_TELEMETRY
