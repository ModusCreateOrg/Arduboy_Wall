#include "SPI.h"

#define bufferLen 8
volatile uint8_t sBuffer[bufferLen];
volatile uint8_t sBufferShadow[bufferLen];

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


int idx = 0;

/*
 * // SPI receive multiple bytes
uint8_t SdSpiAltDriver::receive(uint8_t* buf, size_t n) {
  Spi* pSpi = SPI0;
  int rtn = 0;
#if USE_SAM3X_DMAC
  // clear overrun error
  uint32_t s = pSpi->SPI_SR;

  spiDmaRX(buf, n);
  spiDmaTX(0, n);

  uint32_t m = millis();
  while (!dmac_channel_transfer_done(SPI_DMAC_RX_CH)) {
    if ((millis() - m) > SAM3X_DMA_TIMEOUT)  {
      dmac_channel_disable(SPI_DMAC_RX_CH);
      dmac_channel_disable(SPI_DMAC_TX_CH);
      rtn = 2;
      break;
    }
  }
  if (pSpi->SPI_SR & SPI_SR_OVRES) {
    rtn |= 1;
  }
#else  // USE_SAM3X_DMAC
  for (size_t i = 0; i < n; i++) {
    pSpi->SPI_TDR = 0XFF;
    while ((pSpi->SPI_SR & SPI_SR_RDRF) == 0) {}
    buf[i] = pSpi->SPI_RDR;
  }
#endif  // USE_SAM3X_DMAC
  return rtn;
}
 * 
 */

void SPI0_Handler(void) {
  Serial.println("Interrupt triggered!");
  Serial.print("REG_SPI0_SR: ");
  Serial.println(REG_SPI0_SR, BIN);


  uint32_t statusRegister = REG_SPI0_SR;
  Serial.print("SPI_SR_RDRF : ");
  Serial.println(SPI_SR_RDRF, BIN);

 
  int rtn = 0;

  for (int i = 0; i < bufferLen; i++) {
    REG_SPI0_TDR = 0XFF;
//    while ((REG_SPI0_SR & SPI_SR_RDRF) == 0) {}
    sBuffer[i] = REG_SPI0_RDR;
  }

//  int i = 1;
//  
//  sBuffer[0] = (uint8_t)REG_SPI0_RDR;  // get first byte
//  REG_SPI0_TDR = (uint32_t)0; 
//  for (; i < bufferLen; i++) {
////    sBuffer[i] = SPI.transfer(0); // This causes a lockup of the MCU
//    sBuffer[i] = (uint8_t)REG_SPI0_RDR;  // get first byte
//    REG_SPI0_TDR = (uint32_t)0;
//  }
//  sBuffer[idx] =  SPI.transfer(0);



//  sBuffer[idx++] = (uint8_t)REG_SPI0_RDR;  // get first byte
//  REG_SPI0_TDR = (uint32_t)0; 
//  if (idx >= bufferLen) {
//    idx = 0;
//  }

//
////  Serial.print
//  if (statusRegister & SPI_SR_RDRF) {
//    uint32_t d = REG_SPI0_RDR;
//
////    sBuffer[0] = (uint8_t)REG_SPI0_RDR;  // get first byte
//    sBuffer[0] = (uint8_t)REG_SPI0_RDR;  // get first byte
//    REG_SPI0_TDR = (uint32_t)0;
//
////    for (int i = 1; i < bufferLen; i++) {
////      sBuffer[i] = (uint8_t)REG_SPI0_RDR;
////      REG_SPI0_TDR = (uint32_t)0;
//////      sBuffer[i] = (uint8_t)REG_SPI0_RDR;  // get first byte
//////      REG_SPI0_TDR = (uint32_t)0;
////
////    }
//
//    Serial.print("Read: ");
//    Serial.println(sBuffer[0], DEC);
////    Serial.print(" Read: 0x");
////    Serial.println(iInVal, HEX);
//  }
//
//  if (statusRegister & SPI_SR_TDRE) {
//    REG_SPI0_TDR = (uint32_t)0;
//  }



  haveData = true;
}


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

}  

void loop () {

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
  delay(250);
}  // end of loop

