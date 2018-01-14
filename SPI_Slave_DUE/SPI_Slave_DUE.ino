#include <SPI.h>

#define bufferLen 8
volatile uint8_t sBuffer[bufferLen];

static BitOrder bitOrder = MSBFIRST;
/*
  Now we need a LedControl to work with.
 ***** These pin numbers will probably not work with your hardware *****
  pin 12 is connected to the DataIn
  pin 11 is connected to the CLK
  pin 10 is connected to LOAD
  We have only a single MAX72XX.
*/
#include "LedControl.h"
#define NUM_DEVICES 2
//LedControl(12,11,10,1);
//LedControll(DataIn, CLK, LOAD)
LedControl ledControl = LedControl(52, 48, 50 , NUM_DEVICES);

volatile bool haveData = false;

unsigned int bufferIndex;


unsigned int SPI_readScreenBuffer() {
  //  byte * p = sBuffer;
  unsigned int i;
  //
  //
  //  sBuffer[0] = (uint8_t)SPDR;  // get first byte
  //  for (i = 1; i < bufferLen; i++) {
  //    sBuffer[i] = SPI.transfer(0);
  //  }
  //  haveData = true;
  //  Serial.println();
  return i;
}  // e


// Vaguely based off of:
// https://github.com/NeuroRoboticTech/JetduinoDrivers/blob/master/arduino/Due_SPI_Slave_Min/Due_SPI_Slave_Min.ino
/** (SPI0)
     https://searchcode.com/file/115994556/hardware/arduino/sam/system/CMSIS/Device/ATMEL/sam3xa/include/instance/instance_spi0.h
     REG_SPI0_CR    Control Register
     REG_SPI0_MR    Mode Register
     REG_SPI0_RDR   Receive Data Register
     REG_SPI0_TDR   Transmit Data Register
     REG_SPI0_SR    Status Register
     REG_SPI0_IER   Interrupt Enable Register
     REG_SPI0_IDR   Interrupt Disable Register
     REG_SPI0_IMR   Interrupt Mask Register
     REG_SPI0_CSR   Chip Select Register
     REG_SPI0_WPMR  Write Protection Control Register
     REG_SPI0_WPSR  Write Protection Status Register
*/
void SPI0_Handler() {
  //  NVIC_DisableIRQ(SPI0_IRQn);
  haveData = true;

  Serial.println("Interrupt triggered!");
  Serial.print("REG_SPI0_SR: ");
  Serial.print(REG_SPI0_SR, BIN);
  Serial.print("     ");
  Serial.println(REG_SPI0_SR, DEC);


  uint32_t registerStatus = REG_SPI0_SR;
  Serial.print("Status : ");
  Serial.println(registerStatus & SPI_SR_RDRF);

  int _count = bufferLen;
  // Send the first byte
  while ((REG_SPI0_SR & SPI_SR_TDRE) == 0);
  REG_SPI0_TDR = sBuffer[_count];
  
//  while ((REG_SPI0_SR & SPI_SR_RDRF) == 0);
  sBuffer[_count] |= REG_SPI0_RDR;

  while (_count > 1) {
    // Prepare next byte
//    if (_count == 2 && _mode == SPI_LAST)

    // Read transferred byte and send next one straight away
    while ((REG_SPI0_SR & SPI_SR_TDRE) == 0)
      ;
    uint8_t r = REG_SPI0_RDR;
    REG_SPI0_TDR = 1;

    // Save read byte
    sBuffer[_count] = r;
    _count--;
  }

  // Receive the last transferred byte
//  while ((REG_SPI0_SR & SPI_SR_RDRF) == 0)
//    ;
  uint8_t r = REG_SPI0_RDR;
  sBuffer[_count] = r;


 


  ////  Serial.print
  ////  if (registerStatus & SPI_SR_RDRF) {
  ////    uint32_t d = REG_SPI0_RDR;
  //
  ////    sBuffer[0] = (uint8_t)REG_SPI0_RDR;  // get first byte
  //    sBuffer[0] = (uint8_t)REG_SPI0_RDR;  // get first byte
  //    REG_SPI0_TDR = (uint32_t)0;
  //
  //    for (int i = 1; i < bufferLen; i++) {
  //      sBuffer[i] = (uint8_t)REG_SPI0_RDR;
  //      REG_SPI0_TDR = (uint32_t)0;
  ////      sBuffer[i] = (uint8_t)REG_SPI0_RDR;  // get first byte
  ////      REG_SPI0_TDR = (uint32_t)0;
  //
  //    }
  //
  //    Serial.print("Read: ");
  //    Serial.println(sBuffer[0], DEC);
  ////    Serial.print(" Read: 0x");
  ////    Serial.println(iInVal, HEX);
  ////  }
  //
  ////  if (registerStatus & SPI_SR_TDRE) {
  //    REG_SPI0_TDR = (uint32_t)0;
  ////  }
  //
  ////  NVIC_EnableIRQ(SPI0_IRQn);
  //  SPI.transfer(0);
}



//void SPI0_Handler() {¨
//  NVIC_DisableIRQ(SPI0_IRQn);
//
//  Serial.println("Interrupt triggered!");
//  Serial.print("REG_SPI0_SR: ");
//  Serial.println(REG_SPI0_SR);
//
//  uint32_t iStatus = REG_SPI0_SR;
//
//  if (iStatus & SPI_SR_RDRF) {
//    uint32_t d = REG_SPI0_RDR;
//    iInVal = (byte) d;
//
//
//
//    Serial.print(" Read: ");
//    Serial.println(iInVal, DEC);
////    Serial.print(" Read: 0x");
////    Serial.println(iInVal, HEX);
//  }
//
//  if (iStatus & SPI_SR_TDRE) {
//    REG_SPI0_TDR = (uint32_t) 0;
//  }
//
//  NVIC_EnableIRQ(SPI0_IRQn);
//  haveData = true;
//}

void slaveBegin(uint8_t _pin) {
  SPI.begin(_pin);
  REG_SPI0_CR = SPI_CR_SWRST;     // reset SPI
  REG_SPI0_CR = SPI_CR_SPIEN;     // enable SPI
  REG_SPI0_MR = SPI_MR_MODFDIS;     // slave and no modefault
  REG_SPI0_CSR = 0x82;    // DLYBCT=0, DLYBS=0, SCBR=0, 8 bit transfer

  REG_SPI0_IDR = ~SPI_IDR_RDRF;
  REG_SPI0_IER = SPI_IDR_RDRF;

  NVIC_DisableIRQ(SPI0_IRQn);
  NVIC_ClearPendingIRQ(SPI0_IRQn);
  NVIC_SetPriority(SPI0_IRQn, 0);
  NVIC_EnableIRQ(SPI0_IRQn);

  // Set data register to 0
  REG_SPI0_TDR = 0;
}


#define PRREG(x) Serial.print(#x" 0x"); Serial.println(x,BIN)

void prregs() {
  PRREG(REG_SPI0_MR); // Mode Register
  PRREG(REG_SPI0_CSR); // Chip Select Register
  PRREG(REG_SPI0_SR); // Status Register
}


//
//void SPI0_Handler(void) {
//  // Reverse bit order
////  if (bitOrder == LSBFIRST)
////    _data = __REV(__RBIT(_data));
//
//  uint32_t d = 0;
//
//  while ((REG_SPI0_SR & SPI_SR_TDRE) == 0) ;
//  REG_SPI0_TDR = d;
//
//  while ((REG_SPI0_SR & SPI_SR_RDRF) == 0) ;
//  d = REG_SPI0_RDR;
//
//  // Reverse bit order
////  if (bitOrder == LSBFIRST)
////    d = __REV(__RBIT(d));
//
//  Serial.println(d & 0xFF);
////  return d & 0xFF;
//}

byte x = 0;
unsigned long ctr = 0;
void setup () {
  while (!Serial);
  Serial.begin (115200);   // debugging
  //  Serial.begin(500000);
  slaveBegin(SS);
  prregs();

  Serial.println(F("\nSetup done"));

  //we have to init all devices in a loop
  for (uint8_t address = 0; address < NUM_DEVICES; address++) {
    /*The MAX72XX is in power-saving mode on startup*/
    ledControl.shutdown(address, false);
    /* Set the brightness to a medium values */
    ledControl.setIntensity(address, 8);
    /* and clear the display */
    ledControl.clearDisplay(address);
  }

}  // end of setup

unsigned long ledStripCtr = 0;


void loop() {
//  REG_SPI0_SR
//  while ((REG_SPI0_SR & SPI_SR_TDRE) == 0);
  Serial.println(REG_SPI0_SR, BIN);
}

void loopz () {
  //
  //    return;
  if (haveData) {
    ledControl.clearDisplay(1);

    Serial.print("--- ");
    Serial.print(++ctr);
    Serial.println(" ---");
    for (byte i = 0; i < bufferLen; i++) {
      ledControl.setRow(0, i, sBuffer[i]);
      //
      Serial.print("sBuffer[");
      Serial.print(i);
      Serial.print("] :: ");
      Serial.println(sBuffer[i]);
    }
    Serial.flush();
    haveData = false;

    x++;
    if (x > 7) {
      x = 0;
    }
    ledControl.setRow(1, x, 255);

  }



  //  delay(1);
  // ledControl.clearDisplay(0);

}  // end of loop

// SPI interrupt routine
//ISR (SPI_STC_vect) {
//  SPI_readScreenBuffer();
//  haveData = true;
//}  // end of interrupt routine SPI_STC_vect
