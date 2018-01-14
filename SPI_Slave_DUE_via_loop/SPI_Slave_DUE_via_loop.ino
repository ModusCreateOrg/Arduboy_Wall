// SPI slave for DUE, hack
//  master will lower CS and manage CLK
// MISO to MISO  MOSI to MOSI   CLK to CLK   CS to CS, common ground
//   http://forum.arduino.cc/index.php/topic,157203.0.html
// core hardware/arduino/sam/system/libsam/source/spi.c
// TODO:  C class, interrupt, FIFO/16-bit, DMA?

#include <SPI.h>

// assumes MSB
static BitOrder bitOrder = MSBFIRST;

void slaveBegin(uint8_t _pin) {
  SPI.begin(_pin);
  SPI.setClockDivider(4, SPI_CLOCK_DIV2);

  REG_SPI0_CR = SPI_CR_SWRST;     // reset SPI
  REG_SPI0_CR = SPI_CR_SPIEN;     // enable SPI
  REG_SPI0_MR = SPI_MR_MODFDIS;     // slave and no modefault
  REG_SPI0_CSR = SPI_MODE0;    // DLYBCT=0, DLYBS=0, SCBR=0, 8 bit transfer
}


/*
   REG_SPI0_SR 10000010000001001   Meaning
                            ||||
                            |||└-> RDRF (Receive Data Register Full)   - 1 = Data has been received and the received data has been transferred from the serializer to SPI_RDR since the last read
                            ||└--> TDRE (Transmit Data Register Empty) - 1 = The last data written in the Transmit Data Register has been transferred to the serializer.
                            |└---> MODF (Mode Fault Error)             - 1 = A Mode Fault occurred since the last read of the SPI_SR.
                            └----> OVRES(Overrun Error Status)         - 1 = An overrun has occurred since the last read of SPI_SR.

*/
#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))


byte transferz(uint8_t _pin, uint8_t _data) {
  // Reverse bit order
  //  if (bitOrder == LSBFIRST)
  //    _data = __REV(__RBIT(_data));
  uint32_t d = _data;

  Serial.println("Waiting till clear to send");
  // Transmit Data Register Empty mask is 10
  while ((REG_SPI0_SR & SPI_SR_TDRE) == 0) {
    //    Serial.print("REG_SPI0_SR :"); Serial.println(REG_SPI0_SR, BIN);
    //    Serial.print("SPI_SR_TDRE :"); Serial.println(SPI_SR_TDRE, BIN);

  }

  //  Serial.println("Setting transmit data");
  //  REG_SPI0_TDR = d;
  //
  //  Serial.println("Waiting till recieve full");
  //  Serial.print("1. REG_SPI0_SR: ");
  //  Serial.println(REG_SPI0_SR, BIN);

  while ((REG_SPI0_SR & SPI_SR_RDRF) == 0)
    d = REG_SPI0_RDR;

  // While Receive Data Read Full is 01
  //  while ((REG_SPI0_SR & SPI_SR_RDRF) == 0) {
  //    uint8_t val = REG_SPI0_RDR;
  //    Serial.print("REG_SPI0_RDR: ");
  //    Serial.println(val, BIN);
  //    Serial.print("--- REG_SPI0_SR: ");
  //    Serial.println(REG_SPI0_SR, BIN);
  //    if ((REG_SPI0_SR & SPI_SR_TDRE) == 0) {
  //        REG_SPI0_TDR = val;
  //    }
  ////    REG_SPI0_TDR = 0;
  //  }

  Serial.print("2. REG_SPI0_SR: ");
  Serial.println(REG_SPI0_SR, BIN);

  Serial.print("Recieved: ");
  Serial.println(d);

  // Reverse bit order
  //  if (bitOrder == LSBFIRST)
  //    d = __REV(__RBIT(d));
  return d & 0xFF;
}



byte transfer(uint8_t _pin, uint8_t _data) {
  uint32_t d = _data;

  Serial.println("Waiting till clear to send");

  // Transmit data 
  while ((REG_SPI0_SR & SPI_SR_TDRE) == 0) ;

  Serial.println("Setting transmit data");
  REG_SPI0_TDR = 1;

  Serial.println("Waiting till recieve full");
  Serial.print("1. REG_SPI0_SR: ");
  Serial.println(REG_SPI0_SR, BIN);

  while ((REG_SPI0_SR & SPI_SR_RDRF) == 0) {
    //{
    //  Serial.print("REG_SPI0_SR: ");
    //  Serial.println(REG_SPI0_SR);
    //};
    d = REG_SPI0_RDR;
//    Serial.println((uint8_t)d, BIN);
  }
  Serial.print("2. REG_SPI0_SR: ");
  Serial.println(REG_SPI0_SR, BIN);

  Serial.print("Recieved: ");
  Serial.println(d & 0xFF);

  // Reverse bit order

  return d & 0xFF;
}





#define PRREG(x) Serial.print(#x" "); Serial.println(x,BIN)

void prregs() {
  Serial.begin(115200);
  while (!Serial);
  PRREG(REG_SPI0_MR);
  PRREG(REG_SPI0_CSR);
  PRREG(REG_SPI0_SR);
}


#define SS 10
void setup() {
  slaveBegin(SS);
  prregs();  // debug
}

unsigned long cycle = 0;
void loop() {
  byte in;
  static byte out = 0x83;

  in = transfer(SS, out);
  Serial.print("\n ---- cycle ");
  Serial.print(cycle++);
  Serial.println(" -----");

  //out = in;
  //Serial.print("Out: ");
  //Serial.println(out);
}
