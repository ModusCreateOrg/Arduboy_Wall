#include "Arduboy2.h"
#include "ArduboyTones.h"
#include "Ard.h"

// make an instance of arduboy used for many functions
extern Arduboy2 arduboy;
extern ArduboyTones sound(arduboy.audio.enabled);


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
void ard_setup() {
  // initiate arduboy instance
  arduboy.boot();
  arduboy.audio.on();

  arduboy.setFrameRate(1);
  
  sound.volumeMode(VOLUME_ALWAYS_NORMAL);
  for (byte i = 0; i < numBalls - 1; i++) {
    x[i] = random(1, 14);
    y[i] = random(1, 7);
    xDirection[i] = random(1,2);
    yDirection[i] = random(1,2);
  }
}


void ard_step() {
  if (!(arduboy.nextFrame()))
    return;

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
    xBoundary = 15,
    yBoundary = 7,
    toneDuration = 50;

  static const float 
    negVelocity = -.5,
    posVelocity = .5;               
               
//  delay(100);
  if (xTail > xBoundary) {
    xTailDirection = negVelocity + ((random(randStart, randEnd) * .1) * -1);
    sound.tone(250, toneDuration); 
    xTail = xBoundary;
  }
  else if (xTail < 0) {
    xTailDirection =  posVelocity + (random(randStart, randEnd) * .1);
    sound.tone(300, toneDuration); 
    xTail = 0;
  }

  if (yTail > yBoundary) {
    yTailDirection = negVelocity + ((random(randStart, randEnd) * .1) * -1);
    sound.tone(150, toneDuration); 
    yTail = yBoundary;
  }
  else if (yTail < 0) {
    yTailDirection =  posVelocity + (random(randStart, randEnd) * .1);
    sound.tone(200, toneDuration); 
    yTail = 0;
  }
  
  prevX = (byte)xTail;
  prevY = (byte)yTail;

  Serial.println("------");

  for (byte i = 0; i < numBalls; i++) {
    arduboy.drawPixel(tailX[i], tailY[i], 1);
  }
  
  arduboy.display();
  delay(250);
}
