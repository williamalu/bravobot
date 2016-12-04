/* 
 * cmd_vel_unit_test
 * Subscrubes to cmd_vel. When cmd_val is published to, converts linear and angular velocities in the Twist message to
 * port and starboard motor speeds, which are controlled by Servo values (0-90-180). Sends motor signals on loop().
 */

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <Servo.h>

#define PORT_MOTORS 10
#define STARBOARD_MOTORS 11

ros::NodeHandle  nh;
Servo port_motors;
Servo starboard_motors;

int port_speed = 0; // range from 24 to 156
int starboard_speed = 0; // range from 24 to 156

//Since the cmd_vel speeds range between -1 and 1, the following variables simplify
//the math needed to convert these speeds to the 24-90-156 speeds. Feel free to hard-code these in. 
float motor_offset = 90.0;
float motor_multiplier = (24.0-156.0)/2;

//cmdvel_cb: The callback function when a Twist msg is received on cmd_vel.
//The callback deconstructs the twist and sends it to the setMotorSpeed function.
void cmdvel_cb( const geometry_msgs::Twist& msg){
  float linearSpeed = msg.linear.x;
  float angularDir = msg.angular.z;
  setMotorSpeed(linearSpeed, angularDir);
}

ros::Subscriber<geometry_msgs::Twist> cmd("cmd_vel", &cmdvel_cb);

void setup()
{ 
  //Initialize Hardware
  starboard_motors.attach(STARBOARD_MOTORS);
  port_motors.attach(PORT_MOTORS);

  //Initialize ROS and subscribe
  nh.initNode();
  nh.subscribe(cmd);
}

void loop()
{  
  //Power the motors from the values of the global variables.
  port_motors.write(port_speed);
  starboard_motors.write(starboard_speed);


  //ROS
  nh.spinOnce();
  
}

//setMotorSpeed: Converts linar and angular speeds between -1 and 1 to Motor servo values from 24-90-156. These values are assigned to
//global variables that handle motor speed (allowing speed to be changed in the loop()).
void setMotorSpeed(float linearSpeed, float angularDir) {
  //If angular direction = 0, then we send linear speed * r to both motors.
  //If angular dirction = -1 then we send linear speed * r to the right motors, and 85 to the left.
  //If angular dirction =  1 then we send linear speed * r to the left motors, and 85 to the right.
  //If angluar speed = ang_speed, then se send linear speed*r to the left motors, and linear speed *r * (1-ang_speed) to the right.
  //If angluar speed = -ang_speed, then se send linear speed*r to the right motors, and linear speed *r * (1-ang_speed) to the right.
  
  //Helper variables that designate motor speeds, but at the scale (i.e. -1 to 1) of the input values
  float port_speed_rel; // range from -1 to 1
  float starboard_speed_rel; // range from -1 to 1

  //Calculate motors' differential speeds assigned to a scale of -1 to 1.
  if (linearSpeed >= 0){ //we gon go forward!
    if (angularDir == 0) { //straight
        port_speed_rel = linearSpeed;
        starboard_speed_rel = linearSpeed;
    }
    else if (angularDir>0){//right
        port_speed_rel = linearSpeed;
        starboard_speed_rel = linearSpeed*(1-angularDir)+.005;
    }
    else if (angularDir<0){//left
        starboard_speed_rel = linearSpeed;
        port_speed_rel = linearSpeed*(1-angularDir)+0.005;
    }
  }
  else {//we gon go straight backwards
      starboard_speed_rel = linearSpeed;
      port_speed_rel = linearSpeed;
  }

  //Map relative speeds to actual motor values and update the global variables.
  port_speed = round(port_speed_rel*motor_multiplier + motor_offset);
  starboard_speed = round(starboard_speed_rel*motor_multiplier + motor_offset);

  //Output debug values as Rosinfo
  char pRelMsg[8]; //Empty char array required for dtostrf
  char sRelMsg[8];
  char pAbsMsg[8];
  char sAbsMsg[8];
  
  dtostrf(port_speed_rel, 6, 2, pRelMsg); //Converts floats (and ints) to char arrays, which are required by loginfo.
  dtostrf(starboard_speed_rel, 6, 2, sRelMsg);  
  dtostrf(port_speed, 6, 2, pAbsMsg);  
  dtostrf(starboard_speed, 6, 2, sAbsMsg);  
  
  nh.loginfo("Relative Motor Speed (port, starboard)"); //Equivalent to ROSINFO() in cpp
  nh.loginfo(pRelMsg);
  nh.loginfo(sRelMsg);
  nh.loginfo("Absolute Motor Speed (port, starboard)");
  nh.loginfo(pAbsMsg);
  nh.loginfo(sAbsMsg);
  
}
