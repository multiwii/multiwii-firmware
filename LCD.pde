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
, {"pMeter Sum",      &pMeter[6],     &__PS}
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
  uint8_t p,paramActive;
  uint8_t val,valActive;
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
    //switch config param with pitch
     if ((key == LCD_MENU_NEXT || (rcData[PITCH] < MINCHECK && paramActive == 0)) && p<= PARAMMAX) {
      paramActive = 1;refreshLCD=1;blinkLED(10,20,1);
      p++;
      if (p>PARAMMAX) p=0;
    }
    if ((key == LCD_MENU_PREV || (rcData[PITCH] > MAXCHECK && paramActive == 0)) && p>=0) {
      paramActive = 1;refreshLCD=1;blinkLED(10,20,1);
      if (p==0) p=PARAMMAX; else p--;
    }
    //+ or - param with low and high roll
    if ((key == LCD_VALUE_DOWN || (rcData[ROLL] < MINCHECK && valActive == 0))) {
      valActive = 1;refreshLCD=1;blinkLED(10,20,1);
      lcd_param[p].def->type->inc(lcd_param[p].var, -lcd_param[p].def->increment);
      if (p == 0) memcpy(lcd_param[4].var, lcd_param[0].var, 1);
    }
    if ((key == LCD_VALUE_UP || (rcData[ROLL] > MAXCHECK && valActive == 0))) {
      valActive = 1;refreshLCD=1;blinkLED(10,20,1);
      lcd_param[p].def->type->inc(lcd_param[p].var, +lcd_param[p].def->increment);
      if (p == 0) memcpy(lcd_param[4].var, lcd_param[0].var, 1);
    }
    if (rcData[ROLL] < MAXCHECK && rcData[ROLL]  > MINCHECK) valActive = 0;
    if (rcData[PITCH] < MAXCHECK && rcData[PITCH] > MINCHECK)  paramActive = 0;
    if (valActive) {delay(250); valActive = 0;}
    if (paramActive) {delay(250); paramActive = 0;}
    if (rcData[YAW]  < MINCHECK && rcData[PITCH] > MAXCHECK) LCD = 0; // save and exit
    if (rcData[YAW]  > MAXCHECK && rcData[PITCH] > MAXCHECK) LCD = 2; // exit without save: eeprom has only 100.000 write cycles
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
