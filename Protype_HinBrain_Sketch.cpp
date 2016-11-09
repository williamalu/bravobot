//Basic Hind Brain RoadKill Test Code
//Version 1.0 10/27/16
#include <Servo.h>
#include <Adafruit_NeoPixel.h>
//Set up Arduino Ports and Pins to support Robot
int ledPin = 13;
int actOutPin = 4;
int eStopPin = 5;
int neoPixel1 = 10;
int neoPixel2 = 11;
int sharpDistance1 = 0; //create name for sharp ir 1 analog input pin 0
int sharpDistance2 = 1;
int sharpDistance3 = 2;
int sharpDistance4 = 3;
int pan = 90;
int tilt = 90;
Servo panServo;
Servo tiltServo;
int delayPeriod = 100;
char command = 'g'; // 'g' is go command from midbrain, 's' is stop
char hBrainStatus = 'r'; // hinbrain status 'r' running, 's' stopped, 'e' E-Stopped
String readString; 
Adafruit_NeoPixel strip1 = Adafruit_NeoPixel(8, neoPixel1, NEO_GRB +NEO_KHZ800);

void setup() {
  // put your setup code here, to run once:
  pinMode (ledPin, OUTPUT); // sets up Blinky "alive" light
  pinMode (actOutPin, OUTPUT); //sets up Act binary output port
  pinMode (eStopPin, INPUT);
  Serial.begin(9600); //send and recieve at 9600 baud
  Serial.println("Have Midbrain send 1 or 0 to Hinbrain pin 4");
  panServo.writeMicroseconds(1500);
  tiltServo.writeMicroseconds(1500); //set initial servo position to 90 deg
  panServo.attach(9);
  tiltServo.attach(8);
  strip1.begin();
  strip1.show(); //Inialize all pixels to 'off'
}


// Run Hindbrain loop until commaned to stop by Midbrain
void loop() {

//Read Midbrain commands
  while (Serial.available()){
   command = Serial.read();
   readString+= command;
   delay (2);
  }

  Serial.println("Last Midbrain command sent: ");
  Serial.println(readString);
  Serial.println(hBrainStatus);
  if (readString.length()>0){
    Serial.println(readString);
    if (hBrainStatus!='e'){
      pan = readString.toInt();
      tilt = pan;
      }
     readString = "";
    }
    
  if (command == 'g'){
    hBrainStatus = 'r';
    }

  if (command == 'e'){
    hBrainStatus = 'e';
    }
//Sense: Read Robot Sensors
  if (readEstop() == 1){
    hBrainStatus = 'e';
    }else hBrainStatus = 'r';

  float SharpRange1 = sharpRange(sharpDistance1);
//  Serial.println(SharpRange1);
  float SharpRange2 = sharpRange(sharpDistance2);
  float SharpRange3 = sharpRange(sharpDistance3);
  float SharpRange4 = sharpRange(sharpDistance4);
    
//Think: Run low level cognition and safety code
  if (command != 's'){
  blink();
  }else{
    delay (delayPeriod);
    delay (delayPeriod);
    }
  if (SharpRange1 < 50.0 || SharpRange2 < 50.0 || SharpRange3 < 50.0 || SharpRange4 < 50.0){ //If obstacle too close on Sharp1 E-Stop
    hBrainStatus = 'e';
    }

//Act: Run actuators and behavior lights 
  toggleActPin4(command);
  if (hBrainStatus =='e'){

    }else{
      Serial.print("Hindbrain running.");
      Serial.print("Commanding pan/tilt angle: ");
      panServo.write(pan);
      tiltServo.write(tilt);

      strip1.setPixelColor(1,100,0,0);
      strip1.setPixelColor(2,100,0,0);
      strip1.setPixelColor(3,100,0,0);
      strip1.show();
//      delay(delayPeriod);
//      strip1.setPixelColor(0,0,0,0);
//      strip1.show();
      }
      
//Write: status data up to MidBrain
  if (command == 's'){
    Serial.println(" hind brain stopped");
   }
  if (hBrainStatus == 'e'){
    Serial.println(" hind brain E-stopped");
    }
}

float sharpRange(int sensornum){
  int rawData = analogRead(sensornum);
  float volts = rawData*0.0048828125;
  float range = 65*pow(volts,-1.10);
  return range;
  }


void blink(){
    digitalWrite (ledPin, HIGH);
    delay(delayPeriod);
    digitalWrite (ledPin, LOW);
    delay(delayPeriod);
  }

void toggleActPin4 (char pinState){
  if (pinState =='1'){
    digitalWrite(actOutPin,HIGH);
    }
  else if (command != '1'){
    digitalWrite(actOutPin,LOW);
    }
  }

boolean readEstop(){
  boolean eStopTriggered = digitalRead(eStopPin);
  return eStopTriggered;
}
