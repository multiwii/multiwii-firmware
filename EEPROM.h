#ifndef EEPROM_H_
#define EEPROM_H_

void readGlobalSet();
bool readEEPROM();
void update_constants();
void writeGlobalSet(uint8_t b);
void writeParams(uint8_t b);
void LoadDefaults();
void readPLog(void);
void writePLog(void);
#if defined(GPS)
//EEPROM functions for storing and restoring waypoints 

void storeWP(void);							// Stores the WP data in the wp struct in the EEPROM
bool recallWP(uint8_t);						// Read the given number of WP from the eeprom, supposedly we can use this during flight.
											// Returns true when reading is successfull and returns false if there were some error (for example checksum)
uint8_t getMaxWPNumber(void);				// Returns the maximum WP number that can be stored in the EEPROM, calculated from conf and plog sizes, and the eeprom size

void loadGPSdefaults(void);
void writeGPSconf(void) ;
bool recallGPSconf(void);


#endif

#endif /* EEPROM_H_ */
