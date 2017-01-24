//
// Created by sunny on 11/8/16.

#include "std_msgs/Int8MultiArray.h"
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
#define PI 3.141592

ros::Publisher vel_pub;

const int DEFAULT_SIZE = 11;
const double ANGLE_INCREMENT = 0.00613592332229;
const double TIME_INCREMENT = 9.76562514552e-05;
float min = 6.0;
double angle = 0;
short int MAX_turning = 2;
bool marked = false;

double angleMin;
double angleCoef = 1;
double P = 10;
double D = 5;
double diffE;
double e;
double wallDistance = 1;
double angularSpeed;
int angularIndex;

int map(double angularSpeed){ //maps the desired angular speed to a index in the turning array
    if (angularSpeed>0){  //turn left
        return (5-(angularSpeed/P) +DEFAULT_SIZE);
    }else if (angularSpeed == 0) return DEFAULT_SIZE+5;
    else {  //turn right
        return (5+(-angularSpeed)/P + DEFAULT_SIZE);
    }
}

void laserData(const sensor_msgs::LaserScan msg){
    int size = msg.ranges.size();
    int direction = 1; // 1 means wall is on the left and -1 means wall is on the right
    int minIndex = size*(direction+1)/4;
    int maxIndex = size*(direction+3)/4;

    std_msgs::Int8MultiArray msgp;
    std::vector<signed char> v(DEFAULT_SIZE*2,0); //index 0-11 is for linear velocity and index 12-21 are for turning. Initial values are all zeros
    //ranges[0] correspond to 0 degrees (to the right), and ranges[512] corresponds to
    //180 degrees (to the left)
    for (int i = 0;i<msg.ranges.size();i++){
        if (msg.ranges[i] < min && msg.ranges[i]>0.15){
            min = msg.ranges[i];
            minIndex = i;
        }
        //Mark turning arrays:
        if (i%(int)(512.0/DEFAULT_SIZE) == 0) {
            marked = false;
        }if (msg.ranges[i]<2.0 && msg.ranges[i] >0.15 && !marked) {
            v[i / (512.0 / DEFAULT_SIZE) + DEFAULT_SIZE] = -MAX_turning; //Put negative values in the turning array at the obstacle directions
            marked = true;
        }
    }

    angleMin = (minIndex - size/2)*ANGLE_INCREMENT;
    diffE = (min - wallDistance) -e;
    e = min-wallDistance;
    angularSpeed = direction*(P*e) + angleCoef*(angleMin - PI*(direction+2)/2); //P P controller
    angularIndex = map(angularSpeed, DEFAULT_SIZE);

    v[angularIndex] +=3; //angularIndex is the direction we should turn to in order to wall follow

    //Change linear velocity: Should modify code below:
    if (v[11+5]<0 || v[11+4]< 0 || v[11+3]< 0 || v[11+6]<0 || v[11+7]<0) { //if the angle of the obstacle is roughly between 45 and 135 degrees
        v[0] = 1;
        v[1] = 1;
        v[2] = 1; // Support decelleration
        v[3] = 2;
        v[4] = 2;
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
    vel_pub = n.advertise<std_msgs::Int8MultiArray>("obst/cmd_vel",1000);
    ros::spin();

    return 0;
}
