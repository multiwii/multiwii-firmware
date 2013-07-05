#ifndef ALARMS_H_
#define ALARMS_H_

void blinkLED(uint8_t num, uint8_t ontime,uint8_t repeat);
uint8_t isBuzzerON(void);
void alarmHandler(void);
void vario_signaling(void);
void i2CLedRingState(void);
void blinkLedRing(void);

#endif /* ALARMS_H_ */
