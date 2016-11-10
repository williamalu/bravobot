//
// Created by isaac on 11/8/16.
//


#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include "std_msgs/String.h"
#include <std_msgs/Int16MultiArray.h>

#include "velocitizer.h"

std::vector<int> velocitizer::prime_factorize(int n) {
    // This function takes an integer as an input and it prints the list of its prime numbers.
    int factor = 2;
    std::vector<int> factors;

    while(true) {
        if (n <= 1) {
            break;
        }
        if (n % factor == 0) {
            factors.push_back(factor);
            n /= factor;
        } else{
            factor = factor + 1;
        }
    }
    return factors;
}

void velocitizer::init(int argc, char* argv[]){

    ros::init(argc, argv, "prime_factorer");
    sub = nh.subscribe<std_msgs::Int16>("scan", 10, &velocitizer::VelocityCallback, this);
    pub = nh.advertise<std_msgs::Int16MultiArray>("/obst/cmd_vel", 1000);

}

void velocitizer::VelocityCallback(const std_msgs::Int16ConstPtr &msg){
    int n = msg->data;
    std::vector<int> factors;
    factors = prime_factorize(n);

    std_msgs::Int16MultiArray pub_msg;
    pub_msg.data = factors;
    pub_msg.data = factors;

    pub.publish(pub_msg);
}