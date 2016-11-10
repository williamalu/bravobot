//
// Created by isaac on 11/8/16.
//
#include <ros/ros.h>
#include "velocitizer.h"

int main(int argc, char* argv[]){
    ros::init(argc,argv, "my_node");

    velocitizer f;
    f.init(argc,argv);
    ros::spin();
    return 0;
}
