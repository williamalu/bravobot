//
// Created by xiaozheng on 12/6/16.
//

#include <std_msgs/Int16MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <string>
#include <vector>
#include <ros/ros.h>

ros::Publisher pub;

//camera screen is 640 * 480
void messageCallback(const std_msgs::Int16MultiArray input){
    std::vector<float> left;
    std::vector<float> right;
    std::vector<float> center;
    geometry_msgs::Twist msg;
    msg.linear.x = 0.3;

    for (int i = 0; i<input.data.size(); i+=4){
        float x = input.data[i];
        float width = input.data[i+2];
        float centerx = x + width/2;

        if (centerx < 100){
            left.push_back(x+width);
        }else if (centerx>=250 && centerx <= 380){
            center.push_back(x); //center always have left most and right most
            center.push_back(x+width);
        }else if (centerx> 440){
            right.push_back(x);
        }

    }
    if (left.size() == 0 && center.size() == 0 && right.size() == 0){// if there's no obstacles
        msg.angular.z = 0;
    }else if (center.size() == 0){ // there might be obstacle on the right or left, but not in the middle, so we can go straight
        msg.angular.z = 0;
        pub.publish(msg);
        std::cout << "I'm going straight, but there might be obstacles beside me" << std::endl;
    }else if (left.size() > 0 && center.size() > 0 && right.size() == 0){
        msg.angular.z = .5;
        pub.publish(msg);
        std::cout << "I'm turning right" << std::endl;
    }else if (right.size()>0 && center.size()> 0 && left.size() == 0){
        msg.angular.z = -.5;
        pub.publish(msg);
        std::cout << "I'm turning left" <<std::endl;
    }else{ //if there's obstacles everywhere, just try going straight, and scream on the inside
        msg.angular.z = 0;
    }

}


int main(int argc, char **argv) {
    ros::init(argc, argv, "camera_cmd");

    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("obstacle_positions", 10, messageCallback);

    pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    ros::spin();
}
