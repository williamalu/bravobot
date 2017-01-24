#include <math.h>
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float32.h"
#include "cmath"

#define PI 3.141592

#define SUBSCRIBER_BUFFER_SIZE 1  // Size of buffer for subscriber.
#define PUBLISHER_BUFFER_SIZE 1000  // Size of buffer for publisher.
//#define MAX_SPEED 0.5
//#define P 10    // Proportional constant for controller
//#define D 5     // Derivative constant for controller
//#define angleCoef 1    // Proportional constant for angle controller
//#define DIRECTION 1 // 1 for wall on the left side of the robot (-1 for the right side).
// #define PUBLISHER_TOPIC "/syros/base_cmd_vel"
#define PUBLISHER_TOPIC "/cmd_vel"
// #define SUBSCRIBER_TOPIC "/syros/laser_laser"
#define SUBSCRIBER_TOPIC "/scan"

ros::Publisher pubMessage;
double e_left = 0;
double e_right = 0;
float setPt = -0.05;
float wallDist = 1.25;
float P = 1;
float D = 0.5;
float minSpd = 0.30;
float maxSpd = 0.65;
float minObstDist= 0.10;
float angleCoef = 1;

//Publisher
void publishMessage(double diffE_right, double diffE_left, double angleMin_right, double angleMin_left)
{
    //preparing message
    geometry_msgs::Twist msg;

    double rotVel_left = setPt + -(P*e_left + D*diffE_left) + angleCoef * (angleMin_left);    //PD controller

    if(e_left > 2 || !std::isfinite(e_left)){
        rotVel_left = setPt;
    }

        double rotVel_right = 0;

    if(e_right < 0) {
        rotVel_right = setPt + (P * e_right + D * diffE_right) + angleCoef * (angleMin_right);    //PD controller
    }

    double rotVel = rotVel_left + rotVel_right;
    msg.angular.z = rotVel;

    if(rotVel < 0){
        msg.linear.x = maxSpd;
    } else {
        msg.linear.x = maxSpd - (maxSpd - minSpd) * (fabs(rotVel - setPt));
    }

    //publishing message
    pubMessage.publish(msg);

    /*if(!ros::ok()){
        ROS_INFO("P = %f", P);
        ROS_INFO("D = %f", D);
        ROS_INFO("minSpd = %f", minSpd);
        ROS_INFO("maxSpd = %f", maxSpd);
        ROS_INFO("wallDist = %f", wallDist);
        ROS_INFO("minObstDist = %f", minObstDist);
        ROS_INFO("setPt = %f", setPt);
        ROS_INFO("angleCoef = %f", angleCoef);
    }*/
}

//Subscriber
void messageCallback(const sensor_msgs::LaserScan msg)
{
    int size = msg.ranges.size();

    //Variables whith index of highest and lowest value in array.
    int one_third = size/3;
    int two_thirds = 2*one_third;
    int minIndex_right = one_third-1;
    int minIndex_middle = two_thirds-1;
    int minIndex_left = size-1;

    //This cycle goes through array and finds minimum
    for(int i = 0; i < one_third; i++)
    {
        if (msg.ranges[i] < msg.ranges[minIndex_right] && msg.ranges[i] > minObstDist){
            minIndex_right = i;
        }
    }
    for(int i = one_third; i < two_thirds; i++)
    {
        if (msg.ranges[i] < msg.ranges[minIndex_middle] && msg.ranges[i] > minObstDist){
            minIndex_middle = i;
        }
    }
    for(int i = two_thirds; i < size; i++)
    {
        if (msg.ranges[i] < msg.ranges[minIndex_left] && msg.ranges[i] > minObstDist){
            minIndex_left = i;
        }
    }

    //Calculation of angles from indexes and storing data to class variables.
    double angleMin_left = (size-minIndex_left)*msg.angle_increment;
    double angleMin_right = (minIndex_right)*msg.angle_increment;

    double distMin_right = msg.ranges[minIndex_right];
    double distMin_middle = msg.ranges[minIndex_middle];
    double distMin_left = msg.ranges[minIndex_left];
    //double distFront = msg.ranges[size/2];

    double diffE_left = (distMin_left - wallDist) - e_left;
    e_left = distMin_left - wallDist;

    double diffE_right = (distMin_right - wallDist) - e_right;
    e_right = distMin_right - wallDist;

    ROS_INFO("dist_left= %f", distMin_left);
    ROS_INFO( "   diffE_left= %f", diffE_left);
    ROS_INFO( "   angleMin_left= %f", angleMin_left);
    ROS_INFO("dist_right= %f", distMin_right);
    ROS_INFO( "   diffE_right= %f", diffE_right);
    ROS_INFO( "   angleMin_right= %f", angleMin_right);

    //Invoking method for publishing message
    publishMessage(diffE_right, diffE_left, angleMin_right, angleMin_left);
}

void setP(const std_msgs::Float32::ConstPtr& msg){
    P = msg->data;
    ROS_INFO("P CHANGED.");
}

void setD(const std_msgs::Float32::ConstPtr& msg){
    D = msg->data;
    ROS_INFO("D CHANGED.");
}

void setMaxSpd(const std_msgs::Float32::ConstPtr& msg){
    maxSpd = msg->data;
    ROS_INFO("MAXSPD CHANGED.");
}

void setMinSpd(const std_msgs::Float32::ConstPtr& msg){
    minSpd = msg->data;
    ROS_INFO("MINSPD CHANGED.");
}

void setWallDist(const std_msgs::Float32::ConstPtr& msg){
    wallDist = msg->data;
    ROS_INFO("WALL_DIST CHANGED.");
}

void setSetPt(const std_msgs::Float32::ConstPtr& msg){
    setPt = msg->data;
    ROS_INFO("SETPT CHANGED.");
}

void setMinObstDist(const std_msgs::Float32::ConstPtr& msg){
    setPt = msg->data;
    ROS_INFO("MIN_OBST_DIST CHANGED.");
}

void setAngleCoef(const std_msgs::Float32::ConstPtr& msg){
    setPt = msg->data;
    ROS_INFO("ANGLE_COEF CHANGED.");
}

int main(int argc, char **argv)
{
    //Initialization of node
    ros::init(argc, argv, "wall_follow");
    ros::NodeHandle n;

    //Creating publisher
    pubMessage = n.advertise<geometry_msgs::Twist>(PUBLISHER_TOPIC, PUBLISHER_BUFFER_SIZE);

    //Creating subscriber and publisher
    ros::Subscriber sub = n.subscribe(SUBSCRIBER_TOPIC, SUBSCRIBER_BUFFER_SIZE, messageCallback);
    ros::Subscriber Psub = n.subscribe("wall_follow/P", 1, setP);
    ros::Subscriber Dsub = n.subscribe("wall_follow/D", 1, setD);
    ros::Subscriber maxSpdSub = n.subscribe("wall_follow/maxSpd", 1, setMaxSpd);
    ros::Subscriber minSpdSub = n.subscribe("wall_follow/minSpd", 1, setMinSpd);
    ros::Subscriber wallDistSub = n.subscribe("wall_follow/wallDist", 1, setWallDist);
    ros::Subscriber setPtSub = n.subscribe("wall_follow/setPt", 1, setSetPt);
    ros::Subscriber minDistSub = n.subscribe("wall_follow/minObstDist", 1, setMinObstDist);
    ros::Subscriber angleCoeffSub = n.subscribe("wall_follow/angleCoef", 1, setAngleCoef);
    ros::spin();

    return 0;
}
