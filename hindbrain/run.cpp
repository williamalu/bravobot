#include "run.h"

void run() {
    Serial.print("Motor Speed in Run: ");
    Serial.println(leftMotorSpeed);
    runMotors(leftMotorSpeed, rightMotorSpeed);   
    runLEDs();
}
