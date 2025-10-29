/*
 * Simple Pin Toggle Example
 * This code will turn a pin on and off every 2 seconds.
 * We are using pin P8_5, which is connected to the 
 * front-right yellow LED on the RSLK chassis.
 */

// This header is for the Energia environment and RSLK
#include "SimpleRSLK.h"

// Define the pin you want to control.
// P8_5 is on the J5 connector and controls the Front Right Yellow LED.
const int RELAY_PIN = P8_5;

void setup() {
  // Set the pin as an output
  pinMode(RELAY_PIN, OUTPUT);
}

void loop() {
  // Turn the pin on (set to HIGH)
  digitalWrite(RELAY_PIN, HIGH);
  
  // Wait for 2 seconds (2000 milliseconds)
  delay(2000);

  // Turn the pin off (set to LOW)
  digitalWrite(RELAY_PIN, LOW);

  // Wait for 2 seconds
  delay(2000);
}
