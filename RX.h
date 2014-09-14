#ifndef RX_H_
#define RX_H_

void configureReceiver();
void computeRC();
uint16_t readRawRC(uint8_t chan);
void readSpektrum(void);
void readSBus(void);
void readSumD(void);
#if defined(OPENLRSv2MULTI)
  void initOpenLRS(void);
  void Read_OpenLRS_RC(void);
#endif
#if defined(SPEK_BIND)  // Bind Support
  void spekBind(void);
#endif

#endif /* RX_H_ */
