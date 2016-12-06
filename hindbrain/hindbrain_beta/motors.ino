void setupMotors() {
  starboard_motors.attach(STARBOARD_MOTORS);
  port_motors.attach(PORT_MOTORS);
}

void runMotors() {
  port_motors.write(port_speed);
  starboard_motors.write(starboard_speed);
}

void stopMotors() {
  port_motors.write(MOTOR_STOP);
  starboard_motors.write(MOTOR_STOP);
}

//setMotorSpeed: Converts linar and angular speeds between -1 and 1 to Motor servo values from 24-90-156. These values are assigned to
//global variables that handle motor speed (allowing speed to be changed in the loop()).
void setMotorSpeed(float linearSpeed, float angularDir) {
  //If angular direction = 0, then we send linear speed * r to both motors.
  //If angular dirction = -1 then we send linear speed * r to the right motors, and 85 to the left.
  //If angular dirction =  1 then we send linear speed * r to the left motors, and 85 to the right.
  //If angluar speed = ang_speed, then se send linear speed*r to the left motors, and linear speed *r * (1-ang_speed) to the right.
  //If angluar speed = -ang_speed, then se send linear speed*r to the right motors, and linear speed *r * (1-ang_speed) to the right.

  linearSpeed = constrain(linearSpeed,-1.0,1.0); //failsafe to prevent wackiness if values are unconstrained
  angularDir = constrain(angularDir,-1.0,1.0);   //failsafe to prevent wackiness if values are unconstrained
  
  //Helper variables that designate motor speeds, but at the scale (i.e. -1 to 1) of the input values
  float port_speed_rel; // range from -1 to 1
  float starboard_speed_rel; // range from -1 to 1

  //Calculate motors' differential speeds assigned to a scale of -1 to 1.
  if (linearSpeed >= 0){ //we gon go forward!
    if (angularDir == 0) { //straight
        port_speed_rel = linearSpeed;
        starboard_speed_rel = linearSpeed;
    }
    else if (angularDir>0){//right, right motor should go faster
        port_speed_rel = linearSpeed;
        starboard_speed_rel = linearSpeed*(1-abs(angularDir))+.005; //added the abs()
        
    }
    else if (angularDir<0){//left
        starboard_speed_rel = linearSpeed;
        port_speed_rel = linearSpeed*(1-abs(angularDir))+0.005; //added the abs() 
    }
  }
  else {//we gon go straight backwards
      starboard_speed_rel = linearSpeed;
      port_speed_rel = linearSpeed;
  }

  //Map relative speeds to actual motor values and update the global variables.
  port_speed = round(port_speed_rel*MOTOR_MULTIPLIER + MOTOR_OFFSET);
  starboard_speed = round(starboard_speed_rel*MOTOR_MULTIPLIER + MOTOR_OFFSET);

//  //Output debug values as Rosinfo
//  char pRelMsg[8]; //Empty char array required for dtostrf
//  char sRelMsg[8];
//  char pAbsMsg[8];
//  char sAbsMsg[8];
//  
//  dtostrf(port_speed_rel, 6, 2, pRelMsg); //Converts floats (and ints) to char arrays, which are required by loginfo.
//  dtostrf(starboard_speed_rel, 6, 2, sRelMsg);  
//  dtostrf(port_speed, 6, 2, pAbsMsg);  
//  dtostrf(starboard_speed, 6, 2, sAbsMsg);  
//  
//  nh.loginfo("Relative Motor Speed (port, starboard)"); //Equivalent to ROSINFO() in cpp
//  nh.loginfo(pRelMsg);
//  nh.loginfo(sRelMsg);
//  nh.loginfo("Absolute Motor Speed (port, starboard)");
//  nh.loginfo(pAbsMsg);
//  nh.loginfo(sAbsMsg);
  
}

