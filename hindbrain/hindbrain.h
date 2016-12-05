#ifndef HINDBRAIN_H
#define HINDBRAIN_H

#include <Adafruit_TiCoServo.h>
#include <Adafruit_NeoPixel.h>
#include "pins.h"

// Set up Arduino GPIO to support robot

// LEDs and Indicator Lights
Adafruit_NeoPixel ringFrontLeft  = Adafruit_NeoPixel(12, LED_FRONT_LEFT,  NEO_GRBW + NEO_KHZ800);
Adafruit_NeoPixel ringFrontRight = Adafruit_NeoPixel(12, LED_FRONT_RIGHT, NEO_GRBW + NEO_KHZ800);
Adafruit_NeoPixel stripBackLeft  = Adafruit_NeoPixel(8,  LED_BACK_LEFT,   NEO_GRBW + NEO_KHZ800);
Adafruit_NeoPixel stripBackRight = Adafruit_NeoPixel(8,  LED_BACK_RIGHT,  NEO_GRBW + NEO_KHZ800);

uint32_t red   = ringFrontLeft.Color(10, 0, 0, 0);
uint32_t white = ringFrontLeft.Color(0, 0, 0, 10);

// Pan-Tilt Servos
int pan  = 90;
int tilt = 90;
Adafruit_TiCoServo panServo;
Adafruit_TiCoServo tiltServo;

// Drive Motors
Adafruit_TiCoServo rightMotors;
Adafruit_TiCoServo leftMotors;

// Initialize Hindbrain State
char command = 'g'; // Command from midbrain
                    // 'g' = run
                    // 's' = software stop
                    // 'p' = rc passthrough
char state   = 'r'; // Hindbrain state
                    // 'r' = run
                    // 's' = software stop
                    // 'e' = e-stop
                    // 'p' = rc passthrough

#endif

boolean readEstop();
