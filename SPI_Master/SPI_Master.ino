#include <Arduboy2.h>
#include <ArduboyTones.h>
// make an instance of arduboy used for many functions
Arduboy2 arduboy;
ArduboyTones sound(arduboy.audio.enabled);

#include "Types.h"

#define numBalls 5

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

// This function runs once in your game.
// use it for anything that needs to be set only once in your game.
void setup() {
  // initiate arduboy instance
  arduboy.boot();
  arduboy.audio.on();
  arduboy.flashlight();
  arduboy.setFrameRate(1);
  // pinMode(LED_BUILTIN, OUTPUT);

  sound.volumeMode(VOLUME_ALWAYS_NORMAL);
  for (byte i = 0; i < numBalls - 1; i++) {
    x[i] = random(1, 7);
    y[i] = random(1, 7);
    xDirection[i] = random(1, 2);
    yDirection[i] = random(1, 2);
  }
  sound.tone(1000, 100); 
}


uint16_t val = 0;

#define bufferLen 8 

void printBuffer() {
  // digitalWrite(SS, LOW);    // SS is pin 10
   // String strOut;
   // for (byte i = 0; i < bufferLen; i++) {
   //   strOut += (String(arduboy.sBuffer[i], DEC) + ", ");
   // }

   // Serial.println("Sent data:");
   // Serial.println(strOut);

  // for (int i = 0; i < bufferLen; i++) {
  //   arduboy.SPItransfer(arduboy.sBuffer[i]);
  //   arduboy.delayShort(1);
  // }

  // digitalWrite(SS, HIGH);

  // Serial.println("--------------------------------------");
  // Serial.print("Wrote "); Serial.print(bufferLen); Serial.println(" bytes");
    // delay(250);  // for testing
}

unsigned long counter = 0;
byte ctr = 1;

void loop() {
  if (! (arduboy.nextFrame()) )
    return;

  counter++;

  if (counter % 5 != 0){
    return;
  }

  arduboy.clear();

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
  xBoundary = 7,
  yBoundary = 7,
  toneDuration = 10;

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


 for (byte i = 0; i < numBalls; i++) {
//   arduboy.drawPixel(tailX[i], tailY[i], 1);
//   arduboy.drawPixel(0,ctr++,1);
 }
 arduboy.drawPixel(0,ctr++,1);
 
//  ctr++;
  if (ctr > 7) {
    ctr = 0;
//    sound.tone(1000, toneDuration);
//    delay(50);
  }

//  arduboy.drawPixel(0,ctr, 1);
//  arduboy.drawPixel(1,ctr+1, 1);
//  arduboy.drawPixel(2,ctr+2, 1);
//  arduboy.drawPixel(3,ctr+3, 1);
//  arduboy.drawPixel(4,ctr+4, 1);
//  sound.tone(200, toneDuration);

  Serial.flush();
  
//  arduboy.setCursor(30,30);
//  arduboy.print(counter++);
//  arduboy.setCursor(30,40);
//  arduboy.print(ctr);
//  arduboy.setCursor(30,50);
//  arduboy.print(arduboy.sBuffer[0]);
   arduboy.display();
   sound.tone(1000, toneDuration);

   printBuffer();

// Serial.print("Frame "); Serial.print(counter); Serial.println(" --  ");

  delay(100);
}
