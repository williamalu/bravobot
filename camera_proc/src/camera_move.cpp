//
// Created by isaac on 12/7/16.
//

#include <std_msgs/Int16MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <string>
#include <vector>
#include <ros/ros.h>

ros::Publisher pub;

//camera screen is 640 * 480
void messageCallback(const std_msgs::Int16MultiArray input){
    std::vector<float> far;
    std::vector<float> close;
    std::vector<float> center;
    geometry_msgs::Twist msg;
    msg.angular.z = -0.1;

    for (int i = 0; i<input.data.size(); i+=4){
        // float y = input.data[i+1];
        float height = input.data[i+3];
        float centery = height/2;

        if (centery < 120) {
            far.push_back(height);
        }else if (centery> 120){
            close.push_back(height);
        }

    }

    if (far.size() == 0 && close.size() == 0){// if there's no obstacles
        msg.linear.x = 0;
    }else if (close.size()>0){
        msg.linear.x = .3;
        pub.publish(msg);
        std::cout << "Object is close" << std::endl;
    }else if (far.size() >0){
        msg.linear.x = .5;
        pub.publish(msg);
        std::cout << "Object is far away" <<std::endl;
    }

}


int main(int argc, char **argv) {
    ros::init(argc, argv, "camera_cmd");

    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("obstacle_positions", 10, messageCallback);

    pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    ros::spin();
}

