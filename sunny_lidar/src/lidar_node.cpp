//
// Created by sunny on 11/8/16.

#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include <iostream>
#include "ros/ros.h"
#include <vector>
#include "sensor_msgs/LaserScan.h"
#include <string>
#include <stdlib.h>
#include <stdio.h>

template<typename T>
std::ostream& operator<<(std::ostream& os, const std::vector<T>& v){
    for(int i=0; i<v.size(); ++i){
        os << v[i] << ", ";
    }
    return os << std::endl;
}

void laserData(const sensor_msgs::LaserScan msg){
    std::cout << msg.ranges;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "lidar_node");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("scan", 10, laserData);

    ros::spin();

    return 0;
}