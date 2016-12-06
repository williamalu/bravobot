#include "e_stop.h"

void eStop() {
    leftMotorSpeed = 90;
    rightMotorSpeed = 90;
    runMotors(leftMotorSpeed, rightMotorSpeed);
    eStopLEDs();
}

// Reads relay attached to robot e-stop switch
boolean readEstop() {
  boolean eStopTriggered = digitalRead(ESTOP);
  return eStopTriggered;
}
