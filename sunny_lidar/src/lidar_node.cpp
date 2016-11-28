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
#include <math.h>

ros::Publisher vel_pub;
const int DEFAULT_SIZE = 11;
const double ANGLE_INCREMENT = 0.00613592332229;
const double TIME_INCREMENT = 9.76562514552e-05;

void laserData(const sensor_msgs::LaserScan msg){
    std::cout << msg.ranges.size()<< "\n";
    std_msgs::Int16MultiArray msgp;
    std::vector<short int> v(DEFAULT_SIZE*2,0); //index 0-11 is for linear velocity and index 12-21 are for turning. Initial values are all zeros

    float min = 6.0;
    double angle = 0;
    short int MAX_turning = 4;
    bool marked = false;

    for (int i = 0;i<msg.ranges.size();i++){
        if (i%(int)(512.0/DEFAULT_SIZE) == 0){
            marked = false;
        }
        if (msg.ranges[i]<2.0 && !marked) {
            v[i / (512.0 / DEFAULT_SIZE) + DEFAULT_SIZE] = -MAX_turning; //Put negative values in the turning array at the obstacle directions
            marked = true;
        }
    }
    //ranges[0] correspond to 0 degrees (to the right), and ranges[512] corresponds to
    //180 degrees (to the left)

    //Check the angle of the obstacle and turn to avoid it
    if (v[11+5]<0 || v[11+4]< 0 || v[11+3]< 0 || v[11+6]<0 || v[11+7]<0){ //if the angle of the obstacle is roughly between 45 and 135 degrees
        v[0] = 1;
        v[1] = 1;
        v[2] = 2; // Support decelleration
        v[3] = 1;
        v[4] = 1;
        v[5] = 0;
        v[6], v[7] = -1; // No acceleration!
        v[8], v[9] = -2;
        v[10] = -3;
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
