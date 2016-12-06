#include "soft_stop.h"

void softStop() {
    leftMotorSpeed = 90;
    rightMotorSpeed = 90;   
    runMotors(leftMotorSpeed, rightMotorSpeed);
    softStopLEDs();
}
