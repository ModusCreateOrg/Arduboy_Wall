#include "src/Arduboy2/Arduboy2.h"
#include <ArduboyTones.h>
// make an instance of arduboy used for many functions
Arduboy2 arduboy;
ArduboyTones sound(arduboy.audio.enabled);

#define LEDCONTROL
#undef LEDCONTROL

#ifdef LEDCONTROL

#include "LedControl.h"
#define NUM_DEVICES 2
LedControl ledControl=LedControl(12,11,10,NUM_DEVICES);

#endif

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

  arduboy.setFrameRate(1);
  
  sound.volumeMode(VOLUME_ALWAYS_NORMAL);
  for (byte i = 0; i < numBalls - 1; i++) {
    x[i] = random(1, 14);
    y[i] = random(1, 7);
    xDirection[i] = random(1,2);
    yDirection[i] = random(1,2);
  }

   
#ifdef LEDCONTROL
    //we have to init all devices in a loop
  for(uint8_t address = 0; address < NUM_DEVICES; address++) {
    /*The MAX72XX is in power-saving mode on startup*/
    ledControl.shutdown(address,false);
    /* Set the brightness to a medium values */
    ledControl.setIntensity(address, 8);
    /* and clear the display */
    ledControl.clearDisplay(address);
  }
#endif
}


void loop() {
  if (!(arduboy.nextFrame()))
    return;

  arduboy.clear();
#ifdef LEDCONTROL
  ledControl.clearDisplay(0);
  ledControl.clearDisplay(1);
#endif

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
  
    Serial.print("iteration "); 
    Serial.print(i); 
    Serial.print(" "); 
    Serial.print(tailX[i]); 
    Serial.print(" "); 
    Serial.println(tailY[i]);

#ifdef LEDCONTROL
    if (tailX[i] > 8) {
      ledControl.setLed(1, tailX[i] - 8, tailY[i], 1);
    }
    else {
      ledControl.setLed(0, tailX[i], tailY[i], 1);
    }
#endif
  }
  
  arduboy.display();
  delay(250);
}
