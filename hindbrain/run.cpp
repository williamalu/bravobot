#include "run.h"

void run() {
    if (leftMotorSpeed != prevLeftMotorSpeed && leftMotorSpeed > prevLeftMotorSpeed) {
      prevLeftMotorSpeed++;
    }
    if (leftMotorSpeed != prevLeftMotorSpeed && leftMotorSpeed < prevLeftMotorSpeed) {
      prevLeftMotorSpeed--;
    }
    if (rightMotorSpeed != prevRightMotorSpeed && rightMotorSpeed > prevRightMotorSpeed) {
      prevRightMotorSpeed++;
    }
    if (rightMotorSpeed != prevRightMotorSpeed && rightMotorSpeed < prevRightMotorSpeed) {
      prevRightMotorSpeed--;
    }

    Serial.println(prevLeftMotorSpeed);
    runMotors(prevLeftMotorSpeed, prevRightMotorSpeed);
    runLEDs();

    delay(10);
}
