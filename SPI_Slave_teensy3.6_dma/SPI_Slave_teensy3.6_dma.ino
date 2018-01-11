#include "DMAChannel.h"

// Display size
#define FRAME_COUNT 4
#define ROW_COUNT 32
#define COL_COUNT 128
#define dataLength  COL_COUNT / 16

// Dot data
uint16_t data[dataLength];
volatile uint8_t dataPointer;
uint16_t frames[FRAME_COUNT][ROW_COUNT][dataLength];
volatile uint8_t frame = 0;
volatile uint8_t row = 0;
volatile bool gotFrame = false;

// Input pins
const int COL_LATCH = 3;
const int ROW_DATA = 2;
const int ROW_CLK = 4;

// Output pins
const int SS_TEST = 1;

// DMA channel
DMAChannel* spi0RxDMA;

void setup() {
  pinMode(ROW_DATA, INPUT_PULLUP);
  pinMode(ROW_CLK, INPUT);
  pinMode(COL_LATCH, INPUT_PULLUP);
  pinMode(SS_TEST, OUTPUT);

  Serial.begin(115200);

  // Configure pins for use by SPI0
  CORE_PIN10_CONFIG = PORT_PCR_MUX(2) | PORT_PCR_PE | PORT_PCR_PS;
  CORE_PIN11_CONFIG = PORT_PCR_DSE | PORT_PCR_MUX(2);
  CORE_PIN12_CONFIG = PORT_PCR_MUX(2) | PORT_PCR_PE;
  CORE_PIN13_CONFIG = PORT_PCR_MUX(2);

  // Enable clock to SPI0
  SIM_SCGC6 |= SIM_SCGC6_SPI0;
  SPI0_MCR = SPI_MCR_HALT | SPI_MCR_MDIS | SPI_MCR_PCSIS(1 << 0);

  SPI0_CTAR0_SLAVE = SPI_CTAR_FMSZ(15); // 16 bit frames, on rising edge

  // Enable FIFO Drain Request Interrupt
  /*
    SPI0_RSER = SPI_RSER_RFDF_RE;
    NVIC_ENABLE_IRQ(IRQ_SPI0);
  */

  // Enable FIFO Drain Request DMA
  SPI0_RSER = SPI_RSER_RFDF_RE | SPI_RSER_RFDF_DIRS;
  spi0RxDMA = new DMAChannel();
  spi0RxDMA->source((volatile uint16_t&) SPI0_POPR);
  spi0RxDMA->destinationBuffer(data, dataLength * sizeof(uint16_t));
  spi0RxDMA->disableOnCompletion();
  spi0RxDMA->triggerAtHardwareEvent(DMAMUX_SOURCE_SPI0_RX);

  //Enable Row Data Interrupt
  attachInterrupt(digitalPinToInterrupt(ROW_CLK), row_clk_isr, FALLING);

  //Enable Dot Latch Interrupt
  attachInterrupt(digitalPinToInterrupt(COL_LATCH), col_latch_isr, RISING);

  // Wait for frame alignment
  do {
    SPI0_MCR |= SPI_MCR_CLR_RXF;
    digitalWriteFast(SS_TEST, HIGH);
  } while (digitalReadFast(ROW_CLK) || !digitalReadFast(COL_LATCH));
  digitalWriteFast(SS_TEST, LOW);

  // Turn on the SPI
  SPI0_MCR &= ~SPI_MCR_HALT & ~SPI_MCR_MDIS;
}

int counter = 0;

void loop() {

  if (gotFrame) {
    uint8_t currentFrame = frame;
//    Serial.println(data);
    uint8_t currentRow = row;

    gotFrame = false;

    memcpy(frames[currentFrame][currentRow], data, sizeof(data));
    Serial.println(frames[currentFrame][currentRow]);
    // REMOVED DEBUG CODE DUMPING FRAME TO USB
  }

}


//Interrupt Service Routine to handle incoming data
/*
  void spi0_isr(void) {
  while (SPI0_SR & 0xF0) {
    data[dataPointer++] = SPI0_POPR;
  }

  SPI0_SR |= SPI_SR_RFDF;
  }

  void col_latch_isr(void) {
  if (dataPointer < 8) {
    while (SPI0_SR & 0xF0) {
      data[dataPointer++] = SPI0_POPR;
    }
    SPI0_SR |= SPI_SR_RFDF;
  }

  dataPointer = 0;

  frame++;
  gotFrame = true;
  }
*/

// DMA to handle incoming data
void col_latch_isr(void) {
  spi0RxDMA->disable();
  spi0RxDMA->clearComplete();
  spi0RxDMA->enable();

  frame++;
  gotFrame = true;
}

void row_clk_isr(void) {
  if (digitalReadFast(ROW_DATA) == HIGH) {
    row = 0;
  }
  else {
    row++;
  }


}
