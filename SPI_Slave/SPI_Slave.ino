#include <SPI.h>

#define bufferLen 32
volatile uint8_t sBuffer[bufferLen];


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
LedControl ledControl = LedControl(35, 37, 36 , NUM_DEVICES);

volatile bool haveData = false;

unsigned int bufferIndex;


unsigned int SPI_readScreenBuffer() {
  //  byte * p = sBuffer;
  unsigned int i;

  // if (bufferIndex > bufferLen) {
  //   bufferIndex = 0;
  // }
  // sBuffer[bufferIndex] = (uint8_t)SPDR;  // get first byte
  // bufferIndex++;

  sBuffer[0] = (uint8_t)SPDR;  // get first byte
  for (i = 1; i < bufferLen; i++) {
    sBuffer[i] = SPI.transfer(0);
  }
  haveData = true;
  //  Serial.println();
  return i;
}  // e



byte x = 0;
unsigned long ctr = 0;
void setup () {
  while (!Serial);
//  Serial.begin (115200);   // debugging
  Serial.begin(500000);
  // have to send on master in, *slave out*
//  pinMode(MISO, OUTPUT);
  // turn on SPI in slave mode
  SPCR |= _BV(SPE);
//  SPSR = 0;



  // now turn on interrupts
  SPI.attachInterrupt();
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


void loop () {
  //  return;
  if (haveData) {
    Serial.print(++ctr);
    Serial.println("--------------------------------------");
//    for (byte i = 0; i < bufferLen; i++) {
//      //      ledControl.setRow(0, i, sBuffer[i]);
//
//      Serial.print("sBuffer[");
//      Serial.print(i);
//      Serial.print("] :: ");
//      Serial.println(sBuffer[i]);
//
//    }
//    Serial.flush();
    haveData = false;

  }

  x++;
  if (x > 7) {
    x = 0;
  }
  ledControl.setRow(1, x, 255);

  //  delay(150);
  // ledControl.clearDisplay(0);
  ledControl.clearDisplay(1);

}  // end of loop

// SPI interrupt routine
ISR (SPI_STC_vect) {
  SPI_readScreenBuffer();
  haveData = true;
}  // end of interrupt routine SPI_STC_vect
