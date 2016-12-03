/* 
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <Servo.h>

ros::NodeHandle  nh;
Servo port_motors;
Servo starboard_motors;

float port_speed = 0; // range from -1 to 1 
float starboard_speed = 0; // range from -1 to 1 

void cmdvel_cb( const geometry_msgs::Twist& msg){
  float linearSpeed = msg.linear.x;
  float angularDir = msg.angular.z;

  //If angular direction = 0, then we send linear speed * r to both motors.
  //If angular dirction = -1 then we send linear speed * r to the right motors, and 85 to the left.
  //If angular dirction =  1 then we send linear speed * r to the left motors, and 85 to the right.
  //If angluar speed = ang_speed, then se send linear speed*r to the left motors, and linear speed *r * (1-ang_speed) to the right.
  //If angluar speed = -ang_speed, then se send linear speed*r to the right motors, and linear speed *r * (1-ang_speed) to the right.
  
  if (linearSpeed >= 0){ //we gon go forward!
    if (angularDir == 0) { //straight
        port_speed = linearSpeed;
        starboard_speed = linearSpeed;
    }
    else if (angularDir>0){//right
        port_speed = linearSpeed;
        starboard_speed = linearSpeed*(1-angularDir)+0.05;
    }
    else if (angularDir<0){//left
        starboard_speed = linearSpeed;
        port_speed = linearSpeed*(1-angularDir)+0.05;
    }
  }
  else {//we gon go straight backwards
      starboard_speed = linearSpeed;
      port_speed = linearSpeed;
  }
}

ros::Subscriber<geometry_msgs::Twist> cmd("cmd_vel", &cmdvel_cb);

void setup()
{ 
  pinMode(13, OUTPUT);
  starboard_motors.attach(11);
  port_motors.attach(10);
  nh.initNode();
  nh.subscribe(cmd);
}

//mapFloat is a direct copy of the float function from Arduino.
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void loop()
{  
  
  port_motors.write(mapFloat(port_speed, -1.0,1.0, 24.0,156.0));
  starboard_motors.write(mapFloat(starboard_speed, -1.0,1.0, 24.0,156.0));
  nh.spinOnce();
  
}
