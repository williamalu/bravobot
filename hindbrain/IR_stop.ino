#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;

std_msgs::String state_msg;
ros::Publisher pub_state("state", &state_msg);

char readhBrainStatus() {
  return 's';
}

void setup() {
  nh.initNode();
  nh.advertise(pub_state);

}

void loop() {
  if (  if (SharpRange1 < 50.0 || SharpRange2 < 50.0 || SharpRange3 < 50.0 || SharpRange4 < 50.0) {
    state_msg.data= readhBrainStatus() ;
    pub_state.publish(&state_msg);
    delay(500);
  }
}

