//BravoBot Hindbrain
//Version 2.0 12/01/2016
//Major Revisions: Changed architecture to FSM

#include "hindbrain.h"

void setup() {
  Serial.begin(9600);

  pinMode(LED_PIN, OUTPUT);
  pinMode(eStopPin, INPUT);

  panServo.attach(7);
  tiltServo.attach(8);
  panServo.write(90);
  tiltServo.write(90);

  rightMotors.attach(9);
  leftMotors.attach(10);
  rightMotors.write(90);
  leftMotors.write(90);

  ringFrontLeft.begin();
  ringFrontRight.begin();
  stripBackLeft.begin();
  stripBackRight.begin();
  ringFrontLeft.show();
  ringFrontRight.show();
  stripBackLeft.show();
  stripBackRight.show();
}

void loop() {

  // Temporary serial buffer read code to test state switching for FSM
  // "Reading" commmands from midbrain
  while (Serial.available()) {
    serialReadIn = Serial.read();
    readString += serialReadIn;
  }
}

