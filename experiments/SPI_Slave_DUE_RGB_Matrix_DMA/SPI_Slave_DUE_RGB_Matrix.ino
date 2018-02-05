
#include "RGBmatrixPanelDue.h"

// xpanels, ypanels, nplanes (tested w/3)
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



#define numBalls 10

float x[numBalls] = {},
      y[numBalls] = {},
      xDirection[numBalls] = {},
      yDirection[numBalls] = {};


int8_t iteration = numBalls;

float xTail = 0,
      yTail = 0,
      xTailDirection = 1,
      yTailDirection = 1;

float tailX[numBalls],
      tailY[numBalls],
      prevX,
      prevY;
unsigned long counter = 0;
byte ctr = 1;
void setup() {

  //nYpanels = 1;
  Serial.begin(9600);

  matrix.begin(500);

  // draw a pixel in solid white
  //               x, y
  //matrix.drawPixel(65, 1, matrix.Color333(7, 0, 0));

  for (byte i = 0; i < numBalls - 1; i++) {
    x[i] = random(1, 7);
    y[i] = random(1, 7);
    xDirection[i] = random(1, 2);
    yDirection[i] = random(1, 2);
  }

  // whew!
}


void loop() {
//  matrix.drawPixel(x, y, matrix.Color333(7, 0, 0));
   matrix.fill(matrix.Color333(0, 0, 0));
  

  iteration--;

  if (iteration < 0) {
    iteration = numBalls;

  }

  tailX[iteration] = prevX;
  tailY[iteration] = prevY;

  xTail += xTailDirection;
  yTail += yTailDirection;

  static const byte
  randStart = 1,
  randEnd = 10,
  xBoundary = matrix.width() - 1,
  yBoundary = matrix.height() - 1;

  static const float
  negVelocity = -.5,
  posVelocity = .5;

  //  delay(100);
  if (xTail > xBoundary) {
    xTailDirection = negVelocity + ((random(randStart, randEnd) * .1) * -1);
//   sound.tone(250, toneDuration);
    xTail = xBoundary;
  }
  else if (xTail < 0) {
    xTailDirection =  posVelocity + (random(randStart, randEnd) * .1);
//   sound.tone(300, toneDuration);
    xTail = 0;
  }

  if (yTail > yBoundary) {
    yTailDirection = negVelocity + ((random(randStart, randEnd) * .1) * -1);
//   sound.tone(150, toneDuration);
    yTail = yBoundary;
  }
  else if (yTail < 0) {
    yTailDirection =  posVelocity + (random(randStart, randEnd) * .1);
//   sound.tone(200, toneDuration);
    yTail = 0;
  }

  prevX = (byte)xTail;
  prevY = (byte)yTail;


 for (byte i = 1; i < numBalls; i++) {
   matrix.drawPixel(tailX[i], tailY[i], matrix.Color333(255, 255, 255));
//   arduboy.drawPixel(tailX[i], tailY[i], 1);
//   arduboy.drawPixel(0,ctr++,1);
 }
  ctr++;

  // draw some text!
  matrix.setCursor(1, 0);   // start at top left, with one pixel of spacing
  matrix.setTextSize(1);    // size 1 == 8 pixels high
  
  // print each letter with a rainbow color
  matrix.setTextColor(matrix.Color333(7,0,0));
  matrix.writeAChar('1');
  matrix.setTextColor(matrix.Color333(7,4,0)); 
  matrix.writeAChar('6');
  matrix.setTextColor(matrix.Color333(7,7,0));
  matrix.writeAChar('x');
  matrix.setTextColor(matrix.Color333(4,7,0)); 
  matrix.writeAChar('3');
  matrix.setTextColor(matrix.Color333(0,7,0));  
  matrix.writeAChar('2');
  
  matrix.setCursor(1, 9);   // next line
  matrix.setTextColor(matrix.Color333(0,7,7)); 
  matrix.writeAChar('*');
  matrix.setTextColor(matrix.Color333(0,4,7)); 
  matrix.writeAChar('R');
  matrix.setTextColor(matrix.Color333(0,0,7));
  matrix.writeAChar('G');
  matrix.setTextColor(matrix.Color333(4,0,7)); 
  matrix.writeAChar('B');
  matrix.setTextColor(matrix.Color333(7,0,4));   

//  delay(3);
}

