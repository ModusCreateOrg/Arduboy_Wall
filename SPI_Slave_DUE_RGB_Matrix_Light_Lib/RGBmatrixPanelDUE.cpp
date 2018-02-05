#include "RGBmatrixPanelDue.h"
#include "glcdfontDue.c"
#include <Arduino.h>


uint16_t RGBmatrixPanelDue::width() {return WIDTH; }

uint16_t RGBmatrixPanelDue::height() {return HEIGHT; }

RGBmatrixPanelDue::RGBmatrixPanelDue(uint8_t matrix_type, uint8_t xpanels, uint8_t ypanels, uint8_t planes) {
  single_matrix_width = 64;
  single_matrix_height = 32;
  initNew(xpanels, ypanels, planes);
}

uint8_t *sBuffer;
uint16_t numSecBufferBytes;

RGBmatrixPanelDue::RGBmatrixPanelDue(uint8_t xpanels, uint8_t ypanels, uint8_t planes) {
  // Keep old constructor for backwards compability, assuming 16x32 matrix
  single_matrix_width = 64;
  single_matrix_height = 32;
  initNew(xpanels, ypanels, planes);
}

void RGBmatrixPanelDue::initNew(uint8_t xpanels, uint8_t ypanels, uint8_t planes) {
  NX = xpanels;
  NY = ypanels;

  PWMBITS = planes;
  PWMMAX = ((1 << PWMBITS) - 1);

  // Save number of sections once because dividing by two is oh so expensive
  sections = single_matrix_height / 2;

  WIDTH = single_matrix_width * xpanels;
  HEIGHT = single_matrix_height * ypanels;

  NUMBYTES = (WIDTH * HEIGHT / 2) * planes;
  matrixbuff = (uint8_t *)malloc(NUMBYTES);
  // set all of matrix buff to 0 to begin with
  memset(matrixbuff, 0, NUMBYTES);

  sBuffer = (uint8_t *)malloc(numSecBufferBytes = (single_matrix_width * NX) * (single_matrix_height * NY) / 8);
  memset(sBuffer, 0, numSecBufferBytes);
  // sBuffer[0] = 0b01010101;
  // sBuffer[2] = 255;
  // sBuffer[0] = 255;

  pwmcounter = 0;
  scansection = 0;

  cursor_x = cursor_y = 0;
}


void RGBmatrixPanelDue::begin(uint32_t frequency) {
  pinMode(APIN, OUTPUT);
  digitalWrite(APIN, LOW); 
  pinMode(BPIN, OUTPUT);
  digitalWrite(BPIN, LOW); 
  pinMode(CPIN, OUTPUT);
  digitalWrite(CPIN, LOW); 
  pinMode(DPIN, OUTPUT); // Only applies to 32x32 matrix
  digitalWrite(DPIN, LOW); 
  pinMode(LAT, OUTPUT);
  digitalWrite(LAT, LOW); 
  pinMode(CLK, OUTPUT);
  digitalWrite(CLK, HIGH); 

  pinMode(OE, OUTPUT);
  digitalWrite(OE, LOW); 
  
  pinMode(R1,OUTPUT);
  pinMode(R2,OUTPUT);
  pinMode(G1,OUTPUT);
  pinMode(G2,OUTPUT);
  pinMode(B1,OUTPUT);
  pinMode(B2,OUTPUT);
  digitalWrite(R1,LOW);
  digitalWrite(R2,LOW);
  digitalWrite(G1,LOW);
  digitalWrite(G2,LOW);
  digitalWrite(B1,LOW);
  digitalWrite(B2,LOW);

  // Serial.print("W ");
  // Serial.println(WIDTH);
  // Serial.print("H ");
  // Serial.println(HEIGHT);
  // Serial.print("NUMBYTES ");
  // Serial.println(NUMBYTES);
  // Serial.println("end of matrix.begin");
  // Serial.flush();

  startTimer(TC1, 0, TC3_IRQn, frequency); //TC1 channel 0, the IRQ for that channel and the desired frequency
}





#define ZERO_PADDING 16
void SPrintZeroPadBin(uint16_t number) {
  char binstr[]="0000000000000000";
  uint8_t i=0;
  uint16_t n=number;

  while(n>0 && i<ZERO_PADDING){
    binstr[ZERO_PADDING-1-i]=n%2+'0';
    ++i;
    n/=2;
  }

 Serial.print(binstr);
}

volatile byte readyToDump = 0;


#define IS_BIT_SET(var, pos) ((var) & (1<<(pos)))


void RGBmatrixPanelDue::writeSectionNew(uint8_t secn, uint8_t *buffptr) {

  // Serial.print("---- writeSection : ");
  // Serial.print(secn);
  // Serial.print(" :: ");
  // SPrintZeroPadBin(secn);
  // Serial.println();

  // Serial.print("pwmcounter : ");
  // Serial.println(pwmcounter);

  uint16_t portCstatus_nonclk = 0x0010; // CLK = low
  uint16_t portCstatus = 0x0010; // OE = HIGH
  uint16_t oeLow = 0x0020;
  portCstatus |= 0x0020; // clk is high here too
  REG_PIOC_ODSR = portCstatus; // set OE, CLK to high
    
  // set A, B, C pins
  if (secn & 0x1){  // Apin
    // Serial.println("A PIN");
    portCstatus |= 0x0002;
    portCstatus_nonclk |= 0x0002;
    oeLow |= 0x0002;
  }

  if (secn & 0x2){ // Bpin
    // Serial.println("B PIN");
    portCstatus |= 0x0004;
    portCstatus_nonclk |= 0x0004;
    oeLow |= 0x0004;
  } 
  
  if (secn & 0x4){ // Cpin
    // Serial.println("C PIN");
    portCstatus |= 0x0008;
    portCstatus_nonclk |= 0x0008;
    oeLow |= 0x0008;
  }
  
  if (secn & 0x8){ // Dpin
    // Serial.println("D PIN");
    portCstatus |= 0x0080;
    portCstatus_nonclk |= 0x0080;
    oeLow |= 0x0080;
  } 

  // Serial.print("portCstatus ");
  // Serial.println(portCstatus, BIN);
  
  REG_PIOC_ODSR = portCstatus; // set A, B, C pins
 
  uint8_t  low, high;
  uint16_t out;
  
  uint16_t rowWidth = single_matrix_width * NX * NY; // 256 with 4 panels

  uint16_t sBufferOffset = (secn < 8) ? 0 : (rowWidth);
  // Which bit to look for?
  uint8_t bit = (secn > 7) ? secn - 8 : secn;


  uint8_t sBufferVal;
  // Serial.print("Bit : ");
  // Serial.println(bit);
  // Serial.print("sBufferOffset : ");
  // Serial.println(sBufferOffset);
  // Serial.print("single_matrix_width * NX * NY = ");
  // Serial.println(single_matrix_width * NX * NY); 
  // single_matrix_width * NX * NY == 256

  // uint16_t = 
  for (uint16_t i = 0; i < rowWidth; i++) {

    out = 0x0000;

    if (secn < 8) {

      if (i < 128) {
        sBufferVal = sBuffer[i];
        if (IS_BIT_SET(sBufferVal, bit)) {

          out |= 0b0000010001000100; //0b0000010001000100; // Upper White
        }

        sBufferVal = sBuffer[i + rowWidth];
        if (IS_BIT_SET(sBufferVal, bit)) {
          out |= 0b0000001000001010;//0b0000001000001010;// Lower White
        } 
      }
      else {
        sBufferVal = sBuffer[i + 512 - 128];
        if (IS_BIT_SET(sBufferVal, bit)) {
          out |= 0b0000010001000100;//0b0000010001000100;// Upper White
        } 

        sBufferVal = sBuffer[i + 768 - 128];
        if (IS_BIT_SET(sBufferVal, bit)) {
          out |= 0b0000001000001010;//0b0000001000001010;// Lower White
        }       

      }
    }
    else {

      if (i < 128) {
        sBufferVal = sBuffer[i + 128];
        if (IS_BIT_SET(sBufferVal, bit)) {

          out |= 0b0000010001000100; //0b0000010001000100; // Upper White
        }

        sBufferVal = sBuffer[i + rowWidth + 128];
        if (IS_BIT_SET(sBufferVal, bit)) {
          out |= 0b0000001000001010;//0b0000001000001010;// Lower White
        } 
      }
      else {
        sBufferVal = sBuffer[i + 512];
        if (IS_BIT_SET(sBufferVal, bit)) {
          out |= 0b0000010001000100;//0b0000010001000100;// Upper White
        } 

        sBufferVal = sBuffer[i + 768];
        if (IS_BIT_SET(sBufferVal, bit)) {
          out |= 0b0000001000001010;//0b0000001000001010;// Lower White
        }       

      }
    }
   
    REG_PIOC_ODSR = portCstatus_nonclk; // set clock to low, OE, A, B, C stay the same
    REG_PIOD_ODSR = out;


    //digitalWrite(CLK, HIGH);
    REG_PIOC_ODSR = portCstatus; // set clock to high, OE, A, B, C stay the same
   
  } 

  // latch it!
  
  //digitalWrite(LAT, HIGH);
  REG_PIOC_ODSR = (portCstatus |= 0x0040);

  //digitalWrite(LAT, LOW);  
  REG_PIOC_ODSR = portCstatus;
  
  //digitalWrite(OE, LOW);
  REG_PIOC_ODSR = oeLow; //portCstatus; //<< portCstatus;

  readyToDump = 1;
}



void RGBmatrixPanelDue::dumpMatrix(void) {
  if (readyToDump == 0) {
    return;
  }
  Serial.print("DUMP section : ");
  byte scanSec = scansection - 1 == -1 ? 0 : scansection - 1;
  Serial.println(scanSec);

  byte row = scansection,
       col = 0;
  Serial.print("numSecBufferBytes = "); Serial.println( numSecBufferBytes);
  for (int i = 0; i < numSecBufferBytes * .1; i++) {
    

    // if (sBuffer[i] > 0) {
      Serial.print("[");
      Serial.print(i);

      Serial.print("]");
      Serial.print(sBuffer[i]);
      // Serial.print(" ");
      // SPrintZeroPadBin(sBuffer[i]);
      // Serial.print(sBuffer[i], BIN);
      Serial.print(", ");
    // }

  }
  Serial.println();
  Serial.println("--------------------");
  Serial.flush();

  readyToDump = 0;
}

uint8_t * RGBmatrixPanelDue::getBuffer() {
  return sBuffer;
}

void  RGBmatrixPanelDue::updateDisplay(void) {
  // Serial.print("updateDisplay(); pwmcounter ");
  // Serial.println(pwmcounter);
  // Serial.println(PWMBITS * single_matrix_width * NX * NY * scansection);

  writeSectionNew(scansection, matrixbuff + (PWMBITS * single_matrix_width * NX * NY * scansection));  
  scansection++;
  // Serial.print("updateDisplay() ");
  // Serial.println(scansection);
  if (scansection == sections) { 
    scansection = 0;
    pwmcounter++;
    
    if (pwmcounter == PWMMAX) { 
      pwmcounter = 0; 
    }
  }
  // Serial.print("scansection: ");
  // Serial.println(scansection);
}

void  RGBmatrixPanelDue::startTimer(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t frequency) {
  pmc_set_writeprotect(false);
  pmc_enable_periph_clk((uint32_t)irq);
  TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK1);
  uint32_t rc = VARIANT_MCK/2/frequency; //2 because we selected TIMER_CLOCK1 above
  //TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK4);
  //// VARIANT_MCK = 84000000, I guess timer4 is every 1/4 or something?
  //uint32_t rc = VARIANT_MCK/128/frequency; //128 because we selected TIMER_CLOCK4 above
  TC_SetRA(tc, channel, rc/2); //50% high, 50% low
  TC_SetRC(tc, channel, rc);
  TC_Start(tc, channel);
  tc->TC_CHANNEL[channel].TC_IER=TC_IER_CPCS;
  tc->TC_CHANNEL[channel].TC_IDR=~TC_IER_CPCS;
  NVIC_EnableIRQ(irq);
}
