//Basic Hind Brain RoadKill Test Code
//Version 1.0 11/01/16

#include <Servo.h>
#include <Adafruit_NeoPixel.h>
#include <ros.h>
#include <std_msgs/Empty.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif
#include <std_msgs/UInt16.h>

ros::NodeHandle  nh;
sensor_msgs::Range range_msg;
ros::Publisher pub_range( "range_data", &range_msg);


// Set up Arduino Ports and Pins to support Robot

// LEDs and Indicator Lights
int ledPin               = 13; // Robot alive indicator light
int neoPixel1            = 10;
int neoPixel2            = 11;
Adafruit_NeoPixel strip1 = Adafruit_NeoPixel(8, neoPixel1, NEO_GRB + NEO_KHZ800);

// Binary Output Port
int actOutPin = 4;  // create binary output port on pin 4

// E-Stop
int eStopPin = 5;

// Sharp IR Distance Sensors
int sharpDistance1 = 0; //create name for sharp ir 1 analog input pin 0
int sharpDistance2 = 1;
int sharpDistance3 = 2;
int sharpDistance4 = 3;
unsigned long range_timer; //create a timer  variable

// Pan-Tilt Servos
//int pan = 90;      // variable to store pan servo position
//int tilt = 90;     // variable to store tilt servo position
Servo panServo;    // create pan servo object
Servo tiltServo;   // create tilt servo object

// Drive Motors
float motorSpeedL = 0; // range from -1 to 1 
float motorSpeedR = 0; // range from -1 to 1 
// William will convert the above 2 floats to : Full forward 24 - stop 90 - full reverse 156


// Midbrain Commands and Hindbrain Status
char command      = 'g'; // 'g' is go command from midbrain, 's' is stop
char hBrainStatus = 'r'; // hindbrain status 'r' running, 's' stopped, 'e' E-Stopped
String readString;       // string to store midbrain commands from serial buffer

//int delayPeriod = 100; // Hindbrain loop delay

void panServo_cb( const std_msgs::UInt16& cmd_msg){
  panServo.write(cmd_msg.data); //set panServo angle, should be from 0-180  
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
}

void tiltServo_cb( const std_msgs::UInt16& cmd_msg){
  tiltServo.write(cmd_msg.data); //set tiltServo angle, should be from 0-180  
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
}

void cmdvel_cb( const geometry_msgs::Twist& msg){
  float linearSpeed = msg.linear.x;
  float angularDir = msg.angular.z;

  //If angular direction = 0, then we send linear speed * r to both motors.
  //If angular dirction = -1 then we send linear speed * r to the right motors, and 85 to the left.
  //If angular dirction =  1 then we send linear speed * r to the left motors, and 85 to the right.
  //If angluar speed = ang_speed, then se send linear speed*r to the left motors, and linear speed *r * (1-ang_speed) to the right.
  //If angluar speed = -ang_speed, then se send linear speed*r to the right motors, and linear speed *r * (1-ang_speed) to the right.
  if (linearSpeed > = 0){
    if (angularDir == 0) {
        motorSpeedL = linearSpeed;
        motorSpeedR = linearSpeed;
    }
    else if (angularDir>0){
        motorSpeedL = linearSpeed;
        motorSpeedR = linearSpeed*(1-angularDir)+0.05;
    }
    else if (angularDir<0){
        motorSpeedR = linearSpeed;
        motorSpeedL = linearSpeed*(1-angularDir)+0.05;
    }
  }
  else {
      motorSpeedR = linearSpeed;
      motorSpeedL = linearSpeed;
  }
}


ros::Subscriber<std_msgs::UInt16> sub("tiltServo", tiltServo_cb);
ros::Subscriber<geometry_msgs cmd("cmd_vel", cmdvel_cb);

void setup(){
  pinMode(13, OUTPUT);

  nh.initNode();
  nh.subscribe(sub);

  panServo.attach(9); //attach it to pin 9
  tiltServo.attach(8); //attach it to pin 8
}

void loop(){
  nh.spinOnce();
  delay(1);
}

//void setup() {
//  Serial.begin(9600); //send and recieve at 9600 baud
  
//  pinMode (ledPin, OUTPUT);    // sets up robot alive indicator light
  
//  pinMode (actOutPin, OUTPUT); // sets up binary output port on pin 4
//  Serial.println("Have Midbrain send 1 or 0 to Hinbrain pin 4");
  
//  pinMode (eStopPin, INPUT);
  
//  panServo.writeMicroseconds(1500);  // set initial servo position to 90 deg
//  tiltServo.writeMicroseconds(1500); // set initial servo position to 90 deg
//  panServo.attach(9);                // attach the pan servo to pin 9
//  tiltServo.attach(8);               // attach the tilt servo to pin 8
  
//  strip1.begin();
//  strip1.show(); //Inialize all pixels to 'off'
//}


// Run Hindbrain loop until commanded to stop by Midbrain
void loop() {

  // Read Midbrain Commands
  while (Serial.available()) { // checks to see if a command is in the serial buffer
    command = Serial.read();   // reads characters from serial buffer
    readString += command;     // stores command in string readString
    delay (2);                 // slows loop to allow buffer to fill with next character of command
  }

  Serial.print("Last Midbrain command sent: ");
  Serial.println(command);

  if (readString.length() > 0) {
    Serial.println(readString);
    if (hBrainStatus != 'e') {    // during e-stop, ignore command to move pan or tilt servos
      pan = readString.toInt();   // convert readString into a pan angle
      tilt = pan;                 // use same angle for tilt
    }
    readString = "";              // empty readString for next input
  }

  // Sense: Read Robot Sensors

  if (readEstop() == 1) {
    hBrainStatus = 'e';     // if E-Stop switch triggered, set hindbrain status to e-stopped
  } 
  else hBrainStatus = 'r';  // if E-Stop not triggered, set hindbrain status to running

  // Serial.println(sharpRange(sharpDistance1)); // Print range from Sharp1
  //float SharpRange1 = sharpRange(sharpDistance1); // Read Sharp1
  //float SharpRange2 = sharpRange(sharpDistance2); // Read Sharp2
  //float SharpRange3 = sharpRange(sharpDistance3); // Read Sharp3
  //float SharpRange4 = sharpRange(sharpDistance4); // Read Sharp4
  float getRange(int sharpDistance1){
    int sample;
    sample = analogRead(sharpDistance1)/4;
    if(sample < 10) //if reading too low, we are far from anything
        return 254;    
    sample= 1309/(sample-3); //get cm
    return (sample - 1)/100; //convert to meters
}

  char frameid[] = "/ir_ranger"; // creates a global variable for the sensors frame id string

  void setup() // initializes the node handle and then fills in the descriptor fields for range_msg
{
  nh.initNode();
  nh.advertise(pub_range);
  
  range_msg.radiation_type = sensor_msgs::Range::INFRARED;
  range_msg.header.frame_id =  frameid;
  range_msg.field_of_view = 0.01;
  range_msg.min_range = 0.03;
  range_msg.max_range = 0.4;
  
}

  void loop() //samples the ranger once every 50 milliseconds and publishes the range data
{
  if ( (millis()-range_timer) > 50){ //publish the range value every 50 ms
    range_msg.range = getRange(sharpDistance1);
    range_msg.header.stamp = nh.now();
    pub_range.publish(&range_msg);
    range_timer =  millis();
  }
    nh.spinOnce();
}

  // Think: Run Low-Level Cognition and Safety Code
  if (command == 's') {
    blink();  // blink robot alive indicator
  } else {
    delay (delayPeriod);  // delay so midbrain can see serial text
 2;
int sharpDistance4 =    delay (delayPeriod);
  }

  //If obstacle too close, e-stop
  if (SharpRange1 < 50.0 || SharpRange2 < 50.0 || SharpRange3 < 50.0 || SharpRange4 < 50.0) {
    hBrainStatus = 'e';
  }

  // Act: Run Actuators and Behavior Lights
  toggleActPin4(command); // toggle high/low Act pin 4 with commands from midbrain

  if (hBrainStatus == 'e') {  // if hindbrain is in e-stop mode, all motors stop
    // this requires leaving pan and tilt where they are and setting Roboclaw
    // to a zero speed on both channels
  } else {                                // hindbrain not estopped
    Serial.println("Hindbrain running.");
    Serial.print("Commanding pan/tilt angle: ");
    Serial.println(pan);
    panServo.write(pan);
    tiltServo.write(pan);
  }


  // Write Status Data to Midbrain
  if (command == 's') {
    Serial.println("Hindbrain stopped.");
  }



// ===============================================================  

//  //Act: Run actuators and behavior lights
//  toggleActPin4(command);
//  if (hBrainStatus == 'e') {
//
//  } else {
//    Serial.print("Hindbrain running.");
//    Serial.print("Commanding pan/tilt angle: ");
//    panServo.write(pan);
//    tiltServo.write(tilt);
//
//    strip1.setPixelColor(1, 100, 0, 0);
//    strip1.setPixelColor(2, 100, 0, 0);
//    strip1.setPixelColor(3, 100, 0, 0);
//    strip1.show();
//    //      delay(delayPeriod);
//    //      strip1.setPixelColor(0,0,0,0);
//    //      strip1.show();
//  }
//
//  //Write: status data up to MidBrain
//  if (hBrainStatus == 'e') {
//    Serial.println(" hind brain E-stopped");
//  }
}

// Blinks robot alive indicator light
void blink() {
  digitalWrite (ledPin, HIGH);
  delay(delayPeriod);
  digitalWrite (ledPin, LOW);
  delay(delayPeriod);
}

// Sets Act pin 4 either high or low based on pinState
void toggleActPin4 (char pinState) {
  if (pinState == '1') {
    digitalWrite(actOutPin, HIGH);
  }
  else if (command != '1') {
    digitalWrite(actOutPin, LOW);
  }
}

// Reads relay attached to robot e-stop switch
boolean readEstop() {
  boolean eStopTriggered = digitalRead(eStopPin);
  return eStopTriggered;
}

// Reads range from Sharp IR distance sensors, returns range in cm
float sharpRange(int sensornum) {
  int rawData = analogRead(sensornum);  // V is 0-1023
  float volts = rawData * 0.0048828125; // convert to volts
  float range = 65 * pow(volts, -1.10); // approximate exp data graph function
  return range;
}
