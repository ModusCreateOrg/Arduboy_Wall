//#include "src/Arduboy2/Arduboy2.h"
//#include "Arduboy2.h"


#include "Ard.h"
#include <SPI.h>



template <typename T> unsigned int SPI_writeAnything (const T& value) {
  const byte * p = (const byte*) &value;
  unsigned int i;
  for (i = 0; i < sizeof value; i++)
    SPI.transfer(*p);
  return i;
}  // end of SPI_writeAnything

template <typename T> unsigned int SPI_readAnything(T& value) {
  byte * p = (byte*) &value;
  unsigned int i;
  for (i = 0; i < sizeof(value); i++) {
    Serial.println(i);
    *p++ = SPI.transfer(0);
  }
  return i;
}  // end of SPI_readAnything


template <typename T> unsigned int SPI_readAnything_ISR(T& value) {
  byte * p = (byte*) &value;
  unsigned int i;
  *p++ = SPDR;  // get first byte
  for (i = 1; i < sizeof(value); i++)
    *p++ = SPI.transfer(0);
  return i;
}  // end of SPI_readAnything_ISR



// create a structure to store the different data values:
typedef struct myStruct {
  byte a;
  int b;
  long c;
};

myStruct foo;

void setup () {
  SPI.begin ();
  // Slow down the master a bit
  SPI.setClockDivider(SPI_CLOCK_DIV8);

  foo.a = 42;
  foo.b = 32000;
  foo.c = 100000;
  ard_setup();
}  // end of setup

void loop () {
  ard_step();
  digitalWrite(SS, LOW);    // SS is pin 10
  SPI_writeAnything(foo);
  digitalWrite(SS, HIGH);
  delay(250);  // for testing

  foo.c++;
}  // end of loop

