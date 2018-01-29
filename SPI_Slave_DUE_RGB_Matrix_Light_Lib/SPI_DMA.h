#include <Arduino.h>
void setupSlave();

uint8_t spiSendReceive(uint8_t* outBuf, uint8_t* inBuf, size_t len);
