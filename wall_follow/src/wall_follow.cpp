#include <math.h>
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#define PI 3.141592

#define SUBSCRIBER_BUFFER_SIZE 1  // Size of buffer for subscriber.
#define PUBLISHER_BUFFER_SIZE 1000  // Size of buffer for publisher.
#define WALL_DISTANCE 0.13
#define MAX_SPEED 0.5
#define P 10    // Proportional constant for controller
#define D 5     // Derivative constant for controller
#define ANGLE_COEF 1    // Proportional constant for angle controller
#define DIRECTION 1 // 1 for wall on the left side of the robot (-1 for the right side).
// #define PUBLISHER_TOPIC "/syros/base_cmd_vel"
#define PUBLISHER_TOPIC "/cmd_vel"
// #define SUBSCRIBER_TOPIC "/syros/laser_laser"
#define SUBSCRIBER_TOPIC "/scan"

ros::Publisher pubMessage;
double e = 0;

//Publisher
void publishMessage(double diffE, double angleMin, double distFront)
{
    //preparing message
    geometry_msgs::Twist msg;

    msg.angular.z = DIRECTION*(P*e + D*diffE) + ANGLE_COEF * (angleMin - PI*DIRECTION/2);    //PD controller

    if (distFront < WALL_DISTANCE){
        msg.linear.x = 0;
    }
    else if (distFront < WALL_DISTANCE * 2){
        msg.linear.x = 0.5*MAX_SPEED;
    }
    else if (fabs(angleMin)>1.75){
        msg.linear.x = 0.4*MAX_SPEED;
    }
    else {
        msg.linear.x = MAX_SPEED;
    }

    //publishing message
    pubMessage.publish(msg);
}

//Subscriber
void messageCallback(const sensor_msgs::LaserScan msg)
{
    int size = msg.ranges.size();

    //Variables whith index of highest and lowest value in array.
    int minIndex = size*(DIRECTION+1)/4;
    int maxIndex = size*(DIRECTION+3)/4;

    //This cycle goes through array and finds minimum
    for(int i = minIndex; i < maxIndex; i++)
    {
        if (msg.ranges[i] < msg.ranges[minIndex] && msg.ranges[i] > 0.0){
            minIndex = i;
        }
    }

    //Calculation of angles from indexes and storing data to class variables.
    double angleMin = (minIndex-size/2)*msg.angle_increment;
    double distMin;
    distMin = msg.ranges[minIndex];
    double distFront = msg.ranges[size/2];
    double diffE = (distMin - WALL_DISTANCE) - e;
    e = distMin - WALL_DISTANCE;

    //Invoking method for publishing message
    publishMessage(diffE, angleMin, distFront);
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
    ros::spin();

    return 0;
}