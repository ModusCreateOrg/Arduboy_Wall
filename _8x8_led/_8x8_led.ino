//We always have to include the library
#include "LedControl.h"
#include <Arduino.h>

#include <SPI.h>

/*
 Now we need a LedControl to work with.
 ***** These pin numbers will probably not work with your hardware *****
 pin 12 is connected to the DataIn 
 pin 11 is connected to the CLK 
 pin 10 is connected to LOAD 
 We have only a single MAX72XX.
 */

#define LED_PIN 13

#define NUM_DEVICES 2
//LedControl(12,11,10,1);
//LedControll(DataIn, CLK, LOAD)
LedControl ledControl=LedControl(35, 37,36 ,NUM_DEVICES);

/* we always wait a bit between updates of the display */
unsigned long delaytime=100;

#define numBalls 10

float x[numBalls] = {},
      y[numBalls] = {},
      xDirection[numBalls] = {},
      yDirection[numBalls] = {};

int8_t iteration = numBalls;

byte ledOnOff = 1;
void blinkLED() {
  ledOnOff = ledOnOff == 1 ? 0 : 1;
  digitalWrite(LED_PIN, ledOnOff);   // set the LED on
//  delay(100);                  // wait for a second
//  digitalWrite(LED_PIN, LOW);    // set the LED off
//  delay(100);                  // wait for a second

}

void setup() {
//  while(!Serial);
//  Serial.begin(9600);      // open the serial port at 9600 bps:    

  for (byte i = 0; i < numBalls - 1; i++) {
    x[i] = random(1, 14);
    y[i] = random(1, 7);
    xDirection[i] = random(1,2);
    yDirection[i] = random(1,2);
  }
    
  //we have already set the number of devices when we created the LedControl
  // int devices=ledControl.getDeviceCount();

  //we have to init all devices in a loop
  for(uint8_t address = 0; address < NUM_DEVICES; address++) {
    /*The MAX72XX is in power-saving mode on startup*/
    ledControl.shutdown(address,false);
    /* Set the brightness to a medium values */
    ledControl.setIntensity(address, 8);
    /* and clear the display */
    ledControl.clearDisplay(address);
  }

  pinMode(LED_PIN, OUTPUT);

  blinkLED();
}

float xTail = 0,
      yTail = 0,
      xTailDirection = 1,
      yTailDirection = 1;
      
float tailX[numBalls],
      tailY[numBalls],
      prevX,
      prevY;

void tailBalls() {


  iteration--;

  if (iteration < 0) {
    iteration = numBalls;
  }

  tailX[iteration] = prevX;
  tailY[iteration] = prevY;

  delay(100);
  ledControl.clearDisplay(0);
  ledControl.clearDisplay(1);  

 //  Serial.println("------");

 //  for (byte i = numBalls; i > 0; i--) {
 //    delay(250);
 //    Serial.println(i, DEC);
 //    Serial.print("x "); Serial.println(tailX[i]);
 //    Serial.print("y "); Serial.println(tailY[i]);
 //    if (i > 0) {
 //      tailX[i - 1] = tailX[i];
 //      tailY[i - 1] = tailY[i];
 //    }
 // }

  
  xTail += xTailDirection;
  yTail += yTailDirection;

  static const byte randStart = 1, randEnd = 10;
  
  if (xTail > 15) {
    xTailDirection = -.25 + ((random(randStart, randEnd) * .1) * -1);
    xTail = 15;
  }
  else if (xTail < 0) {
    xTailDirection =  .25 + (random(randStart, randEnd) * .1);
    xTail = 0;
  }

  if (yTail > 8) {
    yTailDirection = -.25 + ((random(randStart, randEnd) * .1) * -1);
    yTail = 7;
  }
  else if (yTail < 0) {
    yTailDirection =  .25 + (random(randStart, randEnd) * .1);
    yTail = 0;
  }
//  ledControl.setLed(0, 0, 7, 1);

  // if (xTail > 7) {
  //   ledControl.setLed(1, xTail - 7, yTail, 1);
  // }
  // else {
  //   ledControl.setLed(0, xTail, yTail, 1);
  // }

  prevX = xTail;
  prevY = yTail;


  for (byte i = 0; i < numBalls; i++) {
    // delay(10);
    if (tailX[i] > 8) {
      ledControl.setLed(1, tailX[i] - 8, tailY[i], 1);
    }
    else {
      ledControl.setLed(0, tailX[i], tailY[i], 1);
    }
  }

}



void loop() { 
//  randBalls();
  tailBalls();
  blinkLED();
}
