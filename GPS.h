/*
 * GPS.h
 *
 *  Created on: 20 juin 2013
 *      Author: WEYEE
 */

#ifndef GPS_H_
#define GPS_H_

extern int32_t wrap_18000(int32_t ang);

void GPS_set_pids(void);
void GPS_SerialInit(void);
uint8_t GPS_NewData(void);
uint8_t GPS_Compute(void);
void GPS_reset_home_position(void);
void GPS_set_next_wp(int32_t* lat, int32_t* lon);
void GPS_reset_nav(void);

#endif /* GPS_H_ */
