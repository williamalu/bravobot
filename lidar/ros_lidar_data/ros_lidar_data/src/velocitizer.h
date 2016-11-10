//
// Created by isaac on 11/8/16.
//

#ifndef ROS_LIDAR_DATA_VELOCITIZER_H
#define ROS_LIDAR_DATA_VELOCITIZER_H

#endif //ROS_LIDAR_DATA_VELOCITIZER_H

#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include "std_msgs/String.h"
#include <vector>

class velocitizer {

public:
    void init(int argc, char *argv[]);
    std::vector<int> prime_factorize(int n);
    void VelocityCallback(const std_msgs::Int16ConstPtr& msg);
public:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher pub;


};