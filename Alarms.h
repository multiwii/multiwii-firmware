#ifndef ALARMS_H_
#define ALARMS_H_

void blinkLED(uint8_t num, uint8_t ontime,uint8_t repeat);
uint8_t isBuzzerON(void);
void alarmHandler(void);
void vario_signaling(void);
void i2CLedRingState(void);
void blinkLedRing(void);
void auto_switch_led_flasher();
void init_led_flasher();
void led_flasher_set_sequence(uint8_t s);
void led_flasher_autoselect_sequence();
void init_landing_lights(void);
void auto_switch_landing_lights(void);
void PilotLamp(uint8_t count);

#endif /* ALARMS_H_ */
