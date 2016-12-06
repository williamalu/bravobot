void setupROS() {
  nh.initNode();

  //Subscribes to /cmd/vel
  nh.subscribe(cmd);
  nh.advertise(pub_portspeed);
}

//cmdvel_cb: The callback function when a Twist msg is received on cmd_vel.
//The callback deconstructs the twist and sends it to the setMotorSpeed function.
void cmdvel_cb( const geometry_msgs::Twist& msg){
  setMotorSpeed(msg.linear.x, msg.angular.z);
}

void publishAtInterval() {
    if(millis() - lastPublishTime > ROS_PUBLISH_INTERVAL) {
    portspeed_msg.data=port_speed;
    starboardspeed_msg.data=starboard_speed;
    pub_portspeed.publish( &portspeed_msg);
    pub_starboardspeed.publish( &starboardspeed_msg);
    lastPublishTime = millis();
    nh.loginfo("Published motor speeds.");
  }
}

