//
// Created by xiaozheng on 12/6/16.
//

#include <std_msgs/Int16MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <string>
#include <vector>
#include <ros/ros.h>

ros::Publisher pub;


void messageCallback(const std_msgs::Int16MultiArray msg){

    geometry_msgs::Twist msg_twist;

    pub.publish(msg_twist);


}


int main(int argc, char **argv) {
    ros::init(argc, argv, "camera_cmd");

    ros::NodeHandle nh;

    ros::Subscriber sub = n.subscribe("obstacle_positions", 10, messageCallback);

    pub = nh.advertise<geometry_msgs::Twist>("cmr/cmd_vel", 1000);
    ros::spin();
}