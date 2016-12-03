#ifndef _HINDBRAIN_H
#define _HINDBRAIN_H
#endif

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

// E-Stop
const int eStopPin = A4;

// Sharp IR Distance Sensors
const int sharpFrontLeft  = A0;
const int sharpFrontRight = A1;
const int sharpBackLeft   = A2;
const int sharpBackRight  = A3;

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
char serialReadIn;  // 
String readString;  // String to store midbrain commands from serial buffer (for testing of FSM switching without rosserial)
