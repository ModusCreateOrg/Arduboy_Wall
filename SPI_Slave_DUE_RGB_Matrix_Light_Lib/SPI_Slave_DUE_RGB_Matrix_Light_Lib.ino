#include <stdlib.h>
#include "RGBmatrixPanelDue.h"
#define SPI_BUFF_SIZE 1024
uint8_t receive_buffer[SPI_BUFF_SIZE];
uint8_t transmit_buffer[SPI_BUFF_SIZE];

#include "SPI_DMA.h"

// xpanels, ypanels, nplanes (tested w/3)
// uint8_t nXpanels = 2;
// uint8_t nYpanels = 2;

uint8_t nXpanels = 1;
uint8_t nYpanels = 1;
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
  Serial.begin(115200);

  // setupSlave();

  matrix.begin(1); // works for 2 x panels
  // matrix.begin(2250); // <4 panels 
  // matrix.fill(matrix.Color333(7, 7, 7));


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

// White
//[0, 1]1092 0000010001000100, [0, 2]1092 0000010001000100, [0, 3]1092 0000010001000100, [0, 4]1092 0000010001000100, [0, 5]1092 0000010001000100, [0, 6]1092 0000010001000100, [0, 7]1092 0000010001000100, [0, 8]1092 0000010001000100, [0, 9]1092 0000010001000100, [0, 10]1092 0000010001000100, [0, 11]1092 0000010001000100, [0, 12]1092 0000010001000100, [0, 13]1092 0000010001000100, [0, 14]1092 0000010001000100, [0, 15]1092 0000010001000100, [0, 16]1092 0000010001000100, [0, 17]1092 0000010001000100, [0, 18]1092 0000010001000100, [0, 19]1092 0000010001000100, [0, 20]1092 0000010001000100, [0, 21]1092 0000010001000100, [0, 22]1092 0000010001000100, [0, 23]1092 0000010001000100, [0, 24]1092 0000010001000100, [0, 25]1092 0000010001000100, [0, 26]1092 0000010001000100, [0, 27]1092 0000010001000100, [0, 28]1092 0000010001000100, [0, 29]1092 0000010001000100, [0, 30]1092 0000010001000100, [0, 31]1092 0000010001000100, [14, 0]1092 0000010001000100, [14, 1]1092 0000010001000100, [14, 2]1092 0000010001000100, [14, 3]1092 0000010001000100, [14, 4]1092 0000010001000100, [14, 5]1092 0000010001000100, [14, 6]1092 0000010001000100, [14, 7]1092 0000010001000100, [14, 8]1092 0000010001000100, [14, 9]1092 0000010001000100, [14, 10]1092 0000010001000100, [14, 11]1092 0000010001000100, [14, 12]1092 0000010001000100, [14, 13]1092 0000010001000100, [14, 14]1092 0000010001000100, [14, 15]1092 0000010001000100, [14, 16]1092 0000010001000100, [14, 17]1092 0000010001000100, [14, 18]1092 0000010001000100, [14, 19]1092 0000010001000100, [14, 20]1092 0000010001000100, [14, 21]1092 0000010001000100, [14, 22]1092 0000010001000100, [14, 23]1092 0000010001000100, [14, 24]1092 0000010001000100, [14, 25]1092 0000010001000100, [14, 26]1092 0000010001000100, [14, 27]1092 0000010001000100, [14, 28]1092 0000010001000100, [14, 29]1092 0000010001000100, [14, 30]1092 0000010001000100, [14, 31]1092 0000010001000100, [14, 0]1092 0000010001000100, 
//[0, 1] 522 0000001000001010, [0, 2]522 0000001000001010, [0, 3]522 0000001000001010, [0, 4]522 0000001000001010, [0, 5]522 0000001000001010, [0, 6]522 0000001000001010, [0, 7]522 0000001000001010, [0, 8]522 0000001000001010, [0, 9]522 0000001000001010, [0, 10]522 0000001000001010, [0, 11]522 0000001000001010, [0, 12]522 0000001000001010, [0, 13]522 0000001000001010, [0, 14]522 0000001000001010, [0, 15]522 0000001000001010, [0, 16]522 0000001000001010, [0, 17]522 0000001000001010, [0, 18]522 0000001000001010, [0, 19]522 0000001000001010, [0, 20]522 0000001000001010, [0, 21]522 0000001000001010, [0, 22]522 0000001000001010, [0, 23]522 0000001000001010, [0, 24]522 0000001000001010, [0, 25]522 0000001000001010, [0, 26]522 0000001000001010, [0, 27]522 0000001000001010, [0, 28]522 0000001000001010, [0, 29]522 0000001000001010, [0, 30]522 0000001000001010, [0, 31]522 0000001000001010, [14, 0]522 0000001000001010, [14, 1]522 0000001000001010, [14, 2]522 0000001000001010, [14, 3]522 0000001000001010, [14, 4]522 0000001000001010, [14, 5]522 0000001000001010, [14, 6]522 0000001000001010, [14, 7]522 0000001000001010, [14, 8]522 0000001000001010, [14, 9]522 0000001000001010, [14, 10]522 0000001000001010, [14, 11]522 0000001000001010, [14, 12]522 0000001000001010, [14, 13]522 0000001000001010, [14, 14]522 0000001000001010, [14, 15]522 0000001000001010, [14, 16]522 0000001000001010, [14, 17]522 0000001000001010, [14, 18]522 0000001000001010, [14, 19]522 0000001000001010, [14, 20]522 0000001000001010, [14, 21]522 0000001000001010, [14, 22]522 0000001000001010, [14, 23]522 0000001000001010, [14, 24]522 0000001000001010, [14, 25]522 0000001000001010, [14, 26]522 0000001000001010, [14, 27]522 0000001000001010, [14, 28]522 0000001000001010, [14, 29]522 0000001000001010, [14, 30]522 0000001000001010, [14, 31]522 0000001000001010, [14, 0]522 0000001000001010, 
//[0, 1]1614 0000011001001110, [0, 2]1614 0000011001001110, [0, 3]1614 0000011001001110, [0, 4]1614 0000011001001110, [0, 5]1614 0000011001001110, [0, 6]1614 0000011001001110, [0, 7]1614 0000011001001110, [0, 8]1614 0000011001001110, [0, 9]1614 0000011001001110, [0, 10]1614 0000011001001110, [0, 11]1614 0000011001001110, [0, 12]1614 0000011001001110, [0, 13]1614 0000011001001110, [0, 14]1614 0000011001001110, [0, 15]1614 0000011001001110, [0, 16]1614 0000011001001110, [0, 17]1614 0000011001001110, [0, 18]1614 0000011001001110, [0, 19]1614 0000011001001110, [0, 20]1614 0000011001001110, [0, 21]1614 0000011001001110, [0, 22]1614 0000011001001110, [0, 23]1614 0000011001001110, [0, 24]1614 0000011001001110, [0, 25]1614 0000011001001110, [0, 26]1614 0000011001001110, [0, 27]1614 0000011001001110, [0, 28]1614 0000011001001110, [0, 29]1614 0000011001001110, [0, 30]1614 0000011001001110, [0, 31]1614 0000011001001110, [14, 0]1614 0000011001001110, [14, 1]1614 0000011001001110, [14, 2]1614 0000011001001110, [14, 3]1614 0000011001001110, [14, 4]1614 0000011001001110, [14, 5]1614 0000011001001110, [14, 6]1614 0000011001001110, [14, 7]1614 0000011001001110, [14, 8]1614 0000011001001110, [14, 9]1614 0000011001001110, [14, 10]1614 0000011001001110, [14, 11]1614 0000011001001110, [14, 12]1614 0000011001001110, [14, 13]1614 0000011001001110, [14, 14]1614 0000011001001110, [14, 15]1614 0000011001001110, [14, 16]1614 0000011001001110, [14, 17]1614 0000011001001110, [14, 18]1614 0000011001001110, [14, 19]1614 0000011001001110, [14, 20]1614 0000011001001110, [14, 21]1614 0000011001001110, [14, 22]1614 0000011001001110, [14, 23]1614 0000011001001110, [14, 24]1614 0000011001001110, [14, 25]1614 0000011001001110, [14, 26]1614 0000011001001110, [14, 27]1614 0000011001001110, [14, 28]1614 0000011001001110, [14, 29]1614 0000011001001110, [14, 30]1614 0000011001001110, [14, 31]1614 0000011001001110, [14, 0]1614 0000011001001110, 

//      RR  G  GBB
//      UL  U  LUL
// 0000011001001110

// RED                     RR
// (upper) [0, 1]1024 0000010000000000, [0, 2]1024 0000010000000000, [0, 3]1024 0000010000000000, [0, 4]1024 0000010000000000, [0, 5]1024 0000010000000000, [0, 6]1024 0000010000000000, [0, 7]1024 0000010000000000, [0, 8]1024 0000010000000000, [0, 9]1024 0000010000000000, [0, 10]1024 0000010000000000, [0, 11]1024 0000010000000000, [0, 12]1024 0000010000000000, [0, 13]1024 0000010000000000, [0, 14]1024 0000010000000000, [0, 15]1024 0000010000000000, [0, 16]1024 0000010000000000, [0, 17]1024 0000010000000000, [0, 18]1024 0000010000000000, [0, 19]1024 0000010000000000, [0, 20]1024 0000010000000000, [0, 21]1024 0000010000000000, [0, 22]1024 0000010000000000, [0, 23]1024 0000010000000000, [0, 24]1024 0000010000000000, [0, 25]1024 0000010000000000, [0, 26]1024 0000010000000000, [0, 27]1024 0000010000000000, [0, 28]1024 0000010000000000, [0, 29]1024 0000010000000000, [0, 30]1024 0000010000000000, [0, 31]1024 0000010000000000, [14, 0]1024 0000010000000000, [14, 1]1024 0000010000000000, [14, 2]1024 0000010000000000, [14, 3]1024 0000010000000000, [14, 4]1024 0000010000000000, [14, 5]1024 0000010000000000, [14, 6]1024 0000010000000000, [14, 7]1024 0000010000000000, [14, 8]1024 0000010000000000, [14, 9]1024 0000010000000000, [14, 10]1024 0000010000000000, [14, 11]1024 0000010000000000, [14, 12]1024 0000010000000000, [14, 13]1024 0000010000000000, [14, 14]1024 0000010000000000, [14, 15]1024 0000010000000000, [14, 16]1024 0000010000000000, [14, 17]1024 0000010000000000, [14, 18]1024 0000010000000000, [14, 19]1024 0000010000000000, [14, 20]1024 0000010000000000, [14, 21]1024 0000010000000000, [14, 22]1024 0000010000000000, [14, 23]1024 0000010000000000, [14, 24]1024 0000010000000000, [14, 25]1024 0000010000000000, [14, 26]1024 0000010000000000, [14, 27]1024 0000010000000000, [14, 28]1024 0000010000000000, [14, 29]1024 0000010000000000, [14, 30]1024 0000010000000000, [14, 31]1024 0000010000000000, [14, 0]1024 0000010000000000, 
// (lower) [0, 1] 512 0000001000000000, [0, 2]512 0000001000000000, [0, 3]512 0000001000000000, [0, 4]512 0000001000000000, [0, 5]512 0000001000000000, [0, 6]512 0000001000000000, [0, 7]512 0000001000000000, [0, 8]512 0000001000000000, [0, 9]512 0000001000000000, [0, 10]512 0000001000000000, [0, 11]512 0000001000000000, [0, 12]512 0000001000000000, [0, 13]512 0000001000000000, [0, 14]512 0000001000000000, [0, 15]512 0000001000000000, [0, 16]512 0000001000000000, [0, 17]512 0000001000000000, [0, 18]512 0000001000000000, [0, 19]512 0000001000000000, [0, 20]512 0000001000000000, [0, 21]512 0000001000000000, [0, 22]512 0000001000000000, [0, 23]512 0000001000000000, [0, 24]512 0000001000000000, [0, 25]512 0000001000000000, [0, 26]512 0000001000000000, [0, 27]512 0000001000000000, [0, 28]512 0000001000000000, [0, 29]512 0000001000000000, [0, 30]512 0000001000000000, [0, 31]512 0000001000000000, [14, 0]512 0000001000000000, [14, 1]512 0000001000000000, [14, 2]512 0000001000000000, [14, 3]512 0000001000000000, [14, 4]512 0000001000000000, [14, 5]512 0000001000000000, [14, 6]512 0000001000000000, [14, 7]512 0000001000000000, [14, 8]512 0000001000000000, [14, 9]512 0000001000000000, [14, 10]512 0000001000000000, [14, 11]512 0000001000000000, [14, 12]512 0000001000000000, [14, 13]512 0000001000000000, [14, 14]512 0000001000000000, [14, 15]512 0000001000000000, [14, 16]512 0000001000000000, [14, 17]512 0000001000000000, [14, 18]512 0000001000000000, [14, 19]512 0000001000000000, [14, 20]512 0000001000000000, [14, 21]512 0000001000000000, [14, 22]512 0000001000000000, [14, 23]512 0000001000000000, [14, 24]512 0000001000000000, [14, 25]512 0000001000000000, [14, 26]512 0000001000000000, [14, 27]512 0000001000000000, [14, 28]512 0000001000000000, [14, 29]512 0000001000000000, [14, 30]512 0000001000000000, [14, 31]512 0000001000000000, [14, 0]512 0000001000000000, 

// GREEN
// (upper) [0, 1]64 0000000001000000, [0, 2]64 0000000001000000, [0, 3]64 0000000001000000, [0, 4]64 0000000001000000, [0, 5]64 0000000001000000, [0, 6]64 0000000001000000, [0, 7]64 0000000001000000, [0, 8]64 0000000001000000, [0, 9]64 0000000001000000, [0, 10]64 0000000001000000, [0, 11]64 0000000001000000, [0, 12]64 0000000001000000, [0, 13]64 0000000001000000, [0, 14]64 0000000001000000, [0, 15]64 0000000001000000, [0, 16]64 0000000001000000, [0, 17]64 0000000001000000, [0, 18]64 0000000001000000, [0, 19]64 0000000001000000, [0, 20]64 0000000001000000, [0, 21]64 0000000001000000, [0, 22]64 0000000001000000, [0, 23]64 0000000001000000, [0, 24]64 0000000001000000, [0, 25]64 0000000001000000, [0, 26]64 0000000001000000, [0, 27]64 0000000001000000, [0, 28]64 0000000001000000, [0, 29]64 0000000001000000, [0, 30]64 0000000001000000, [0, 31]64 0000000001000000, [14, 0]64 0000000001000000, [14, 1]64 0000000001000000, [14, 2]64 0000000001000000, [14, 3]64 0000000001000000, [14, 4]64 0000000001000000, [14, 5]64 0000000001000000, [14, 6]64 0000000001000000, [14, 7]64 0000000001000000, [14, 8]64 0000000001000000, [14, 9]64 0000000001000000, [14, 10]64 0000000001000000, [14, 11]64 0000000001000000, [14, 12]64 0000000001000000, [14, 13]64 0000000001000000, [14, 14]64 0000000001000000, [14, 15]64 0000000001000000, [14, 16]64 0000000001000000, [14, 17]64 0000000001000000, [14, 18]64 0000000001000000, [14, 19]64 0000000001000000, [14, 20]64 0000000001000000, [14, 21]64 0000000001000000, [14, 22]64 0000000001000000, [14, 23]64 0000000001000000, [14, 24]64 0000000001000000, [14, 25]64 0000000001000000, [14, 26]64 0000000001000000, [14, 27]64 0000000001000000, [14, 28]64 0000000001000000, [14, 29]64 0000000001000000, [14, 30]64 0000000001000000, [14, 31]64 0000000001000000, [14, 0]64 0000000001000000, 
// (lower) [0, 1] 8 0000000000001000, [0, 2]8 0000000000001000, [0, 3]8 0000000000001000, [0, 4]8 0000000000001000, [0, 5]8 0000000000001000, [0, 6]8 0000000000001000, [0, 7]8 0000000000001000, [0, 8]8 0000000000001000, [0, 9]8 0000000000001000, [0, 10]8 0000000000001000, [0, 11]8 0000000000001000, [0, 12]8 0000000000001000, [0, 13]8 0000000000001000, [0, 14]8 0000000000001000, [0, 15]8 0000000000001000, [0, 16]8 0000000000001000, [0, 17]8 0000000000001000, [0, 18]8 0000000000001000, [0, 19]8 0000000000001000, [0, 20]8 0000000000001000, [0, 21]8 0000000000001000, [0, 22]8 0000000000001000, [0, 23]8 0000000000001000, [0, 24]8 0000000000001000, [0, 25]8 0000000000001000, [0, 26]8 0000000000001000, [0, 27]8 0000000000001000, [0, 28]8 0000000000001000, [0, 29]8 0000000000001000, [0, 30]8 0000000000001000, [0, 31]8 0000000000001000, [14, 0]8 0000000000001000, [14, 1]8 0000000000001000, [14, 2]8 0000000000001000, [14, 3]8 0000000000001000, [14, 4]8 0000000000001000, [14, 5]8 0000000000001000, [14, 6]8 0000000000001000, [14, 7]8 0000000000001000, [14, 8]8 0000000000001000, [14, 9]8 0000000000001000, [14, 10]8 0000000000001000, [14, 11]8 0000000000001000, [14, 12]8 0000000000001000, [14, 13]8 0000000000001000, [14, 14]8 0000000000001000, [14, 15]8 0000000000001000, [14, 16]8 0000000000001000, [14, 17]8 0000000000001000, [14, 18]8 0000000000001000, [14, 19]8 0000000000001000, [14, 20]8 0000000000001000, [14, 21]8 0000000000001000, [14, 22]8 0000000000001000, [14, 23]8 0000000000001000, [14, 24]8 0000000000001000, [14, 25]8 0000000000001000, [14, 26]8 0000000000001000, [14, 27]8 0000000000001000, [14, 28]8 0000000000001000, [14, 29]8 0000000000001000, [14, 30]8 0000000000001000, [14, 31]8 0000000000001000, [14, 0]8 0000000000001000, 

// BLUE
// (upper) [0, 1]4 0000000000000100, [0, 2]4 0000000000000100, [0, 3]4 0000000000000100, [0, 4]4 0000000000000100, [0, 5]4 0000000000000100, [0, 6]4 0000000000000100, [0, 7]4 0000000000000100, [0, 8]4 0000000000000100, [0, 9]4 0000000000000100, [0, 10]4 0000000000000100, [0, 11]4 0000000000000100, [0, 12]4 0000000000000100, [0, 13]4 0000000000000100, [0, 14]4 0000000000000100, [0, 15]4 0000000000000100, [0, 16]4 0000000000000100, [0, 17]4 0000000000000100, [0, 18]4 0000000000000100, [0, 19]4 0000000000000100, [0, 20]4 0000000000000100, [0, 21]4 0000000000000100, [0, 22]4 0000000000000100, [0, 23]4 0000000000000100, [0, 24]4 0000000000000100, [0, 25]4 0000000000000100, [0, 26]4 0000000000000100, [0, 27]4 0000000000000100, [0, 28]4 0000000000000100, [0, 29]4 0000000000000100, [0, 30]4 0000000000000100, [0, 31]4 0000000000000100, [14, 0]4 0000000000000100, [14, 1]4 0000000000000100, [14, 2]4 0000000000000100, [14, 3]4 0000000000000100, [14, 4]4 0000000000000100, [14, 5]4 0000000000000100, [14, 6]4 0000000000000100, [14, 7]4 0000000000000100, [14, 8]4 0000000000000100, [14, 9]4 0000000000000100, [14, 10]4 0000000000000100, [14, 11]4 0000000000000100, [14, 12]4 0000000000000100, [14, 13]4 0000000000000100, [14, 14]4 0000000000000100, [14, 15]4 0000000000000100, [14, 16]4 0000000000000100, [14, 17]4 0000000000000100, [14, 18]4 0000000000000100, [14, 19]4 0000000000000100, [14, 20]4 0000000000000100, [14, 21]4 0000000000000100, [14, 22]4 0000000000000100, [14, 23]4 0000000000000100, [14, 24]4 0000000000000100, [14, 25]4 0000000000000100, [14, 26]4 0000000000000100, [14, 27]4 0000000000000100, [14, 28]4 0000000000000100, [14, 29]4 0000000000000100, [14, 30]4 0000000000000100, [14, 31]4 0000000000000100, [14, 0]4 0000000000000100, 
// (lower) [0, 1]2 0000000000000010, [0, 2]2 0000000000000010, [0, 3]2 0000000000000010, [0, 4]2 0000000000000010, [0, 5]2 0000000000000010, [0, 6]2 0000000000000010, [0, 7]2 0000000000000010, [0, 8]2 0000000000000010, [0, 9]2 0000000000000010, [0, 10]2 0000000000000010, [0, 11]2 0000000000000010, [0, 12]2 0000000000000010, [0, 13]2 0000000000000010, [0, 14]2 0000000000000010, [0, 15]2 0000000000000010, [0, 16]2 0000000000000010, [0, 17]2 0000000000000010, [0, 18]2 0000000000000010, [0, 19]2 0000000000000010, [0, 20]2 0000000000000010, [0, 21]2 0000000000000010, [0, 22]2 0000000000000010, [0, 23]2 0000000000000010, [0, 24]2 0000000000000010, [0, 25]2 0000000000000010, [0, 26]2 0000000000000010, [0, 27]2 0000000000000010, [0, 28]2 0000000000000010, [0, 29]2 0000000000000010, [0, 30]2 0000000000000010, [0, 31]2 0000000000000010, [14, 0]2 0000000000000010, [14, 1]2 0000000000000010, [14, 2]2 0000000000000010, [14, 3]2 0000000000000010, [14, 4]2 0000000000000010, [14, 5]2 0000000000000010, [14, 6]2 0000000000000010, [14, 7]2 0000000000000010, [14, 8]2 0000000000000010, [14, 9]2 0000000000000010, [14, 10]2 0000000000000010, [14, 11]2 0000000000000010, [14, 12]2 0000000000000010, [14, 13]2 0000000000000010, [14, 14]2 0000000000000010, [14, 15]2 0000000000000010, [14, 16]2 0000000000000010, [14, 17]2 0000000000000010, [14, 18]2 0000000000000010, [14, 19]2 0000000000000010, [14, 20]2 0000000000000010, [14, 21]2 0000000000000010, [14, 22]2 0000000000000010, [14, 23]2 0000000000000010, [14, 24]2 0000000000000010, [14, 25]2 0000000000000010, [14, 26]2 0000000000000010, [14, 27]2 0000000000000010, [14, 28]2 0000000000000010, [14, 29]2 0000000000000010, [14, 30]2 0000000000000010, [14, 31]2 0000000000000010, [14, 0]2 0000000000000010, 

unsigned int color = 0;
void loop() {
  // doSPI_gfx();
  // Serial.println("loop()");
  // Serial.flush();
  uint16_t color =  matrix.Color444(7,7,2);

  matrix.drawLine(0, 0, 128, 0, color);
  matrix.drawLine(0, 16, 128, 16, color);
  // matrix.drawLine(0, 16, 63, 16, 65535);
  // matrix.drawLine(0, 1,  63, 1, 65535);
  // matrix.drawLine(0, 17, 63, 17, 65535);

  // byte startRow = 5,
  //      altRow  = startRow + 16;
  // matrix.drawLine(0, startRow, 63, startRow, 65535);
  // matrix.drawLine(0, altRow, 63, altRow, 65535);

  // matrix.drawPixel(0, 0, 65535); 
 
  // matrix.drawPixel(1,2, matrix.Color444(255,255,255));

  // delay(1000);
  matrix.dumpMatrix();

  // delay(2000);
  // matrix.fill(matrix.Color333(0, 0, 0));

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
