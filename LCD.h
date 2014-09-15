#ifndef LCD_H_
#define LCD_H_

void configurationLoop();
void LCDprint(uint8_t i);
void lcd_telemetry();
void initLCD();
void i2c_OLED_DIGOLE_init ();
void i2c_OLED_init();
void LCDclear();
void toggle_telemetry(uint8_t t);
void dumpPLog(uint8_t full);

/* helper functions */
void LCDprintInt16(int16_t v);
void LCDcrlf();

void print_uptime(uint16_t sec);
void output_checkboxitems();

/* candidates for telemetry pages outputs */
void output_fails();
void output_annex();

void output_V();
void output_mAh();
void output_AmaxA();
void output_errors_or_armedTime();
void output_WmaxW();
void output_uptime_cset();
void output_altitude();
void output_checkboxstatus();
void output_cycleMinMax();
void output_cycle();
void output_gyroX();
void output_gyroY();
void output_gyroZ();
void output_accX();
void output_accY();
void output_accZ();

void output_debug0() ;
void output_debug1() ;
void output_debug2() ;
void output_debug3() ;

#endif /* LCD_H_ */
