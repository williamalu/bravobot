void setupROS() {
  nh.initNode();
  //Subscribes to /cmd/vel
  nh.advertise(pub_portspeed);
  nh.advertise(pub_starboardspeed);
  nh.advertise(pub_state);
  nh.advertise(pub_millis);

  nh.subscribe(cmd);
  nh.subscribe(enable);
}

//cmdvel_cb: The callback function when a Twist msg is received on cmd_vel.
//The callback deconstructs the twist and sends it to the setMotorSpeed function.
void cmdvel_cb( const geometry_msgs::Twist& msg){
  nh.loginfo("CmdVel callback.");
  setMotorSpeed(msg.linear.x, msg.angular.z);
}
void enable_cb( const std_msgs::Bool& msg){
  nh.loginfo("Enable callback.");
  if(msg.data == true) {
    ROSReady = true;
    //nh.loginfo("Received: ROS READY");
  }
  else{
    ROSReady = false;
    //nh.loginfo("Received: ROS NOT READY");
  }
}

boolean is_ROS_ok() {
  if(ROSReady) return true;
  else return false;
}

void publishAtInterval()
{
    portspeed_msg.data=port_speed_actual;
    starboardspeed_msg.data=starboard_speed_actual;
    millis_msg.data = millis();
    boolean rS = is_ROS_ok();
    boolean irS = is_IR_ok();
    switch (mode) {
      case OK_MODE:
//        state_msg.data="OK";
        state_msg.data=0;
        break;
      case PHYS_ESTOP_MODE:
        //state_msg.data="ESTOP";
        state_msg.data=1;
        break;
      case SOFT_STOP_MODE:
//        if(!rS && !irS) state_msg.data="SSTOP.BOTH";
//        else if(!rS) state_msg.data="SSTOP.ROS";
//        else if(!irS) state_msg.data="SSTOP.IR";
//        else state_msg.data="SSTOP.UNK";
        if(!rS && !irS) state_msg.data=2;
        else if(!rS) state_msg.data=3;
        else if(!irS) state_msg.data=4;
        else state_msg.data=5;
        break;
      default:
        state_msg.data=6;
        break;
    }
    pub_portspeed.publish( &portspeed_msg);
    pub_starboardspeed.publish( &starboardspeed_msg);
    pub_state.publish( &state_msg);
    pub_millis.publish( &millis_msg);
    //lastPublishTime = millis();
//    nh.loginfo("Published messages.");
}

