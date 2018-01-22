#include <stdlib.h>
#include "RGBmatrixPanelDue.h"
#define SPI_BUFF_SIZE 1024
uint8_t receive_buffer[SPI_BUFF_SIZE];
uint8_t transmit_buffer[SPI_BUFF_SIZE];

#include "SPI_DMA.h"

// xpanels, ypanels, nplanes (tested w/3)
uint8_t nXpanels = 2;
uint8_t nYpanels = 2;
RGBmatrixPanelDue matrix(nXpanels, nYpanels, 3);
//RGBmatrixPanelDue matrix(nXpanels, nYpanels);


// wrapper for the redrawing code, this gets called by the interrupt
//TIMER: TC1 ch 0
void TC3_Handler() {
  TC_GetStatus(TC1, 0);
  matrix.updateDisplay();
}

void setup() {

  //nYpanels = 1;
  // while (!Serial);
  // Serial.begin(115200);

  setupSlave();

  // matrix.begin(3500); // works for 2 x panels
  matrix.begin(2250); 
  matrix.fill(matrix.Color333(7, 7, 7));


  for (int iIdx = 0; iIdx < SPI_BUFF_SIZE; iIdx++) {
    transmit_buffer[iIdx] = 0;
  }
  // Serial.println("Setup Done!");
  // Serial.print("NUM_BYTES: "); Serial.println(matrix.NUMBYTES);

}


#define BIT_SET(var, pos) ((var) & (1<<(pos)))


void doSPI_gfx() {
  // Serial.println("spiSendReceive();...");
  // Serial.flush();
  spiSendReceive(transmit_buffer, receive_buffer, SPI_BUFF_SIZE);

  uint8_t byteRow = 0;

  for (uint16_t receive_bufferIndex = 0; receive_bufferIndex < 1024; receive_bufferIndex++) {

    if (receive_bufferIndex >= 128 && receive_bufferIndex % 128 == 0) { // 128 for full screen
      byteRow += 8;
    }

    uint16_t color = 0;


    for (uint8_t bitNum = 0; bitNum < 8; bitNum++) {
      color = 0;

      if (BIT_SET(receive_buffer[receive_bufferIndex], bitNum)) {
        color = 65535; 
      }



      matrix.drawPixel(receive_bufferIndex % 128, byteRow + bitNum, color);

    }
  }  
}



void loop() {
  doSPI_gfx();
  // delay(2000);
  // matrix.fill(matrix.Color333(0, 0, 0));
  // delay(500);
  // Serial.println("loop();"); Serial.flush();
  // String strIn;
  
  // for (byte i = 0; i < 8; i++) {
  //   strIn += (String(receive_buffer[i], DEC) + ", ");
  // }

  // Serial.println("Received data:");
  // Serial.println(strIn);
  // delay(200);

  // matrix.fill(matrix.Color333(0, 0, 0));
  // delay(150);

 
 // matrix.drawPixel(128, 32, matrix.Color333(255, 255, 255));
//
//
//  // print each letter with a rainbow color



  // matrix.drawPixel(0,0, matrix.Color333(7, 0, 0));
  // matrix.drawPixel(0,32,matrix.Color333(0, 7, 0));
  // matrix.drawPixel(64,0,matrix.Color333(0, 0, 7));
  // matrix.drawPixel(64,32,matrix.Color333(7, 0, 7));
 

  // matrix.fillRect(0,0,64,32, matrix.Color444(7,0,0));
  // matrix.fillRect(0,65,64,32, matrix.Color444(0,7,0));
  // matrix.fillRect(32,0,64,32, matrix.Color444(0,0,1));
  // matrix.fillRect(0,0,64,32, matrix.Color444(1,0,0));
  // matrix.fillRect(0,0,64,32, matrix.Color444(1,0,0));

}

