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

ros::Publisher vel_pub;


void laserData(const sensor_msgs::LaserScan msg){
    std::cout << msg.ranges;
    float min = 6.0;
    for (int i = 0;i<msg.ranges.size();i++){
        if (msg.ranges[i]<min){
            min = msg.ranges[i];
        }
    }
    std_msgs::Int16MultiArray msgp;
    std::vector<short int> v(11,0);
    if (min<0.2){
        v.[2] = 1;
    }
    else if (min<1.0){
        v[5] = 1;
    }
    else if (min<2.0){
        v[7] = 1;
    }else if (min>=2.0){
        v[9] = 1;
    }
    msgp.data = v;
    vel_pub.publish(msgp);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "lidar_node");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("scan", 10, laserData);
    vel_pub = n.advertise<std_msgs::Int16MultiArray>("obst/cmd_vel",1000);
    ros::spin();

    return 0;
}
