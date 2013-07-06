#ifndef SERIAL_H_
#define SERIAL_H_

void serialCom();
void SerialOpen(uint8_t port, uint32_t baud);
uint8_t SerialRead(uint8_t port);
void SerialWrite(uint8_t port,uint8_t c);
uint8_t SerialAvailable(uint8_t port);
void debugmsg_append_str(const char *str);
void SerialEnd(uint8_t port);
uint8_t SerialPeek(uint8_t port);
#if defined(GPS_SERIAL)
  bool SerialTXfree(uint8_t port);
#endif

#endif /* SERIAL_H_ */
