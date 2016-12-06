//Servo_blink: cycles the motors on (forward) and off every 2 seconds.

#include <Servo.h>

#define PORT_MOTORS 12
#define STARBOARD_MOTORS 13

Servo myservo1;  // create servo object to control a servo
Servo myservo2;  // create servo object to control a servo

int startLevel = 83;
int stopLevel = 90;

int pos = 0;    // variable to store the servo position

void setup() {
  myservo1.attach(PORT_MOTORS);  // attaches the servo on pin 9 to the servo object
  myservo2.attach(STARBOARD_MOTORS);
  pinMode(13, OUTPUT);
}

void loop() {
  myservo1.write(startLevel);
  myservo2.write(startLevel);
  digitalWrite(13, HIGH);
  delay(2000);
  myservo1.write(stopLevel);
  myservo2.write(stopLevel);
  digitalWrite(13, LOW);
  delay(2000);
}

