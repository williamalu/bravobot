int IR_pin = A0;

void setup() {
  // put your setup code here, to run once:
  pinMode(IR_pin, INPUT);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(is_IR_ok());
}

float getRange(int pin){
    int sample;
    sample = analogRead(pin)/4;
    if(sample < 10) //if reading too low, we are far from anything
        return 254;    
    sample= 1309/(sample-3); //get cm
    return (sample - 1)/100; //convert to meters
}
