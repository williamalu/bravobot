//
// Created by xiaozheng on 12/6/16.
//

#include <std_msgs/Int16MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <string>
#include <vector>
#include <ros/ros.h>

ros::Publisher pub;

float heading=-100;
float savedHeading=-100;

float leftLidObst;
float rightLidObst;

//camera screen is 640 * 480
void headTo(float desired){
    float error = desired-heading;
    //if negative, drift left, else drift right
    while (heading!=desired){
        msg.angular.z=0.1*error;
    }
}

//maps an camera screen x value to a turning direction between -0.4 and 0.4
float mapAngular(int avgx){
    return (avgx-320)*0.4/320;
}

void messageCallback(const std_msgs::Int16MultiArray input){
    std::vector<float> left;
    std::vector<float> right;
    std::vector<float> center;
    std::vector<float> far;
    std::vector<float> close;
    std::vector<int16_t> obst_pos;
    std::vector<int16_t> obst_height;
    std::vector<int16_t> max_heights;
    geometry_msgs::Twist msg;
    msg.linear.x = 0.3;
    msg.angular.z = -0.15;

//    obst_pos = input.data;
//
//    for (int i = 0; i < obst_pos.size()-2; i += 4) {
//        obst_height.push_back(input.data[i + 3]);
//    }
//
//    while (obst_height.size()>0){
//        int max_index = *std::max_element(obst_height.begin(), obst_height.end());
//        int max = (int) std::distance(obst_height.begin(), obst_height.end());
//        max_heights.push_back(max);
//    }
//
//    std::cout << "MAX HEIGHTS HERE WE GO BITCHEZ!" << std::endl;
//    for (std::vector<short>::const_iterator i = max_heights.begin(); i != max_heights.end(); ++i)
//        std::cout << *i << ' ';
//    std::cout << "------" << std::endl;


    for (int i = 0; i<input.data.size(); i+=4){
        // float y = input.data[i+1];
        float height = input.data[i+3];
        float centery = height/2;
        float x = input.data[i];
        float width = input.data[i+2];
        float centerx = x + width/2;
        if (centery < 60) {
            far.push_back(height);
        }
        else if (centery >= 60){
            close.push_back(height);
            if (centerx < 250){
                left.push_back(x+width);
            }else if (centerx>=250 && centerx <= 440){
                center.push_back(x); //center always have left most and right most
                center.push_back(x+width);
            }else if (centerx> 440){
                right.push_back(x);
            }
        }
    }

    std::cout << "LEFT" << std::endl;
    for (std::vector<float>::const_iterator i = left.begin(); i != left.end(); ++i)
        std::cout << *i << ' ';
    std::cout << "------" << std::endl;

    std::cout << "CENTER" << std::endl;
    for (std::vector<float>::const_iterator j = center.begin(); j != center.end(); ++j)
        std::cout << *j << ' ';
    std::cout << "------" << std::endl;

    std::cout << "RIGHT" << std::endl;
    for (std::vector<float>::const_iterator k = right.begin(); k != right.end(); ++k)
        std::cout << *k << ' ';
    std::cout << "------" << std::endl;

    if (far.size() == 0 && close.size() == 0){// if there's no obstacles
        msg.linear.x = 0;
        std::cout << "No obstacles at all" << std::endl;
        if (savedHeading != heading && savedHeading != -100){
            headTo(savedHeading);
            std::cout << "Headed back to saved heading";
            savedHeading = -100;
        }
        else if (savedHeading == -100){
            msg.angular.z=0;
            pub.publish(msg);
            std::cout << "Going straight because no obstacle and no saved heading" << std::endl;
        }
    }else if (close.size()>0){
        savedHeading = heading;
        std::cout << "Saved a Heading" << std::cout;
        msg.linear.x = .3;
        if (left.size() == 0){ // If there's no close obstacle on the left, go a little left by default
            if (center.size() == 0){
                msg.angular.z = -0.15;
                std::cout << "I'm turning slight left" <<std::endl;
            }else{
                msg.angular.z = -.5;
                std::cout << "I'm turning left" <<std::endl;
            }
            pub.publish(msg);
        }else if (center.size() == 0){ // there might be obstacle on the right, but not in the middle, so we can go straight
            msg.angular.z = 0;
            pub.publish(msg);
            std::cout << "I'm going straight, but there might be obstacles beside me" << std::endl;
        }else if (right.size() == 0){// There are things on the left and in the center, go right
            msg.angular.z = .5;
            pub.publish(msg);
            std::cout << "I'm turning right" << std::endl;
        }else{ //if there's obstacles everywhere, just try going straight, and scream on the inside
            if (leftLidObst < 2.0){
                msg.angular.z = 0.5;
                std::cout << "There are obstacles in front, a curb on the left, I'm turning right" << std::endl;
            }else if (rightLidObst<2.0){
                msg.angular.z = -0.5;
                std::cout << "There are obstacles in front, an obstacle on the right, I'm turning left" << std::endl;
            }
            pub.publish(msg);
        }
        std::cout << "Object is close" << std::endl;
    }else if (far.size() > 0){
        float avgx = 0;
        for (int i = 0; i<far.size(); i++){
            avgx+= far[i];
        }
        avgx/= far.size();
        msg.linear.x = .5;
        msg.linear.z = mapAngular(avgx);
        pub.publish(msg);
        std::cout << "Object is far away" <<std::endl;
    }
}

void IMUCallback(const geometry_msgs::Vector3Stamped::ConstPtr &msg){
    float headingx = msg->vector.x;
    float headingy = msg->vector.y;
    float headingz = msg->vector.z;

    float compassHeading = (std::atan2(headingy,headingx) * 180.00000) / 3.14159265359;

    // Normalize to 0-360
    if (compassHeading < 0)
    {
        compassHeading = 360.00000 + compassHeading;
    }

    heading = static_cast<double>(compassHeading);
    // std::cout << "Current heading: " << compassHeading << std::endl;
}

void lidCallback(const std_msgs::Int16MultiArray lidPos){
    leftLidObst = lidPos[0];
    rightLidObst = lidPos[2];
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "camera_cmd");

    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("obstacle_positions", 10, messageCallback);
    ros::Subscriber imu = nh.subscribe("imu_data", 10, IMUCallback);
    ros::Subscriber lidsub = nh.subscribe("lidPos",10, lidCallback);
    
    pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    ros::spin();
}


