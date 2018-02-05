#include "t3spi.h"

#define HEIGHT 64
#define WIDTH 128

volatile uint8_t sBuffer[1024];
//uint8_t sBuffer[1];


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


//Initialize T3SPI class as SPI_SLAVE
T3SPI SPI_SLAVE;

//The number of integers per data packet
//MUST be the same as defined on the MASTER device
#define dataLength  1024

//Initialize the arrays for incoming data
volatile uint8_t data[dataLength] = {};

//Initialize the arrays for outgoing data
 
byte x = 0;
void setup () {
//  while (!Serial);

  pinMode(13, OUTPUT);

  Serial.begin(115200);

  //Begin SPI in SLAVE (SCK pin, MOSI pin, MISO pin, CS pin)
//    SPI_SLAVE.begin_SLAVE(SCK, MOSI, MISO, CS0);
  static const byte
//    _SCK  = SCK,
//    _MOSI = MOSI,
//    _MISO = MISO,
//    _CS0  = CS0;
    _SCK  = 14,
    _MOSI = MOSI,
    _MISO = MISO,
    _CS0  = CS0;    
    
  SPI_SLAVE.begin_SLAVE(_SCK, _MOSI, _MISO, _CS0);

  //Set the CTAR0_SLAVE0 (Frame Size, SPI Mode)
  SPI_SLAVE.setCTAR_SLAVE(8, SPI_MODE0);

  //Enable the SPI0 Interrupt
  NVIC_ENABLE_IRQ(IRQ_SPI0);

  // now turn on interrupts
  //  SPI.attachInterrupt();
  Serial.println(F("Ready for input!"));
  Serial.print("SCK : "); Serial.println(_SCK);
  Serial.print("MOSI: "); Serial.println(_MOSI);
  Serial.print("MISO: "); Serial.println(_MISO);
  Serial.print("CS0 : "); Serial.println(_CS0);

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
  if (haveData) {
    haveData = false;
//    Serial.println(data[0]);

//    for (byte i = 0; i < 8; i++) {
//      ledControl.setRow(0, i, data[i]);
//    }
  }

  x++;
  if (x > 7) {
    x = 0;
  }
  ledControl.setRow(1, x, 255);

  digitalWrite(13, LOW);
  delay(150);
  digitalWrite(13, HIGH);
  ledControl.clearDisplay(0);
  ledControl.clearDisplay(1);


//  ledControl.setRow(0, 0, data[0]);
  if (SPI_SLAVE.dataPointer == 0 && SPI_SLAVE.packetCT == 0) {
    SPI_SLAVE.timeStamp1 = micros();
  }
//  Serial.println(data[0]);
//  Serial.flush();

  //Capture the time when transfer is done
  if (SPI_SLAVE.packetCT == 1) {
    SPI_SLAVE.timeStamp2 = micros();

    //Print data received & data sent
    for (int i = 0; i < dataLength; i++) {
//      Serial.print("data[");
//      Serial.print(i);
//      Serial.print("]: ");
//      Serial.println(data[i]);
////      Serial.print("   returnData[");
////      Serial.print(i);
////      Serial.print("]: ");
////      Serial.println(returnData[i]);
//      Serial.flush();
    }

    //Print statistics for the previous transfer
//    SPI_SLAVE.printStatistics(dataLength);
    //Reset the packet count
    SPI_SLAVE.packetCT = 0;
  }



}  // end of loop


//Interrupt Service Routine to handle incoming data
void spi0_isr(void){
  
  //Function to handle data
//  Serial.println(SPI0_POPR);
  SPI_SLAVE.rx8(data, dataLength);
  haveData = true;
//  SPI_SLAVE.rxtx8(data, returnData, dataLength);
//  SPI_SLAVE.rxtx16(data, returnData, dataLength);
}

