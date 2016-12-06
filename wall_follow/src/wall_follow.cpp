#include <math.h>
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#define PI 3.141592

#define SUBSCRIBER_BUFFER_SIZE 1  // Size of buffer for subscriber.
#define PUBLISHER_BUFFER_SIZE 1000  // Size of buffer for publisher.
#define WALL_DISTANCE 1
#define MAX_SPEED 0.5
//#define P 10    // Proportional constant for controller
//#define D 5     // Derivative constant for controller
#define ANGLE_COEF 1    // Proportional constant for angle controller
#define DIRECTION 1 // 1 for wall on the left side of the robot (-1 for the right side).
// #define PUBLISHER_TOPIC "/syros/base_cmd_vel"
#define PUBLISHER_TOPIC "/cmd_vel"
// #define SUBSCRIBER_TOPIC "/syros/laser_laser"
#define SUBSCRIBER_TOPIC "/scan"

ros::Publisher pubMessage;
double e = 0;
double setPt = 0;
float P = 1;
float D = 0.5;
float minSpd = 0.30;
float maxSpd = 0.65;

//Publisher
void publishMessage(double diffE, double distMin_right, double distMin_middle, double distMin_left, double angleMin)
{
    //preparing message
    geometry_msgs::Twist msg;

    double rotVel = setPt + -(P*e + D*diffE) + ANGLE_COEF * (angleMin);    //PD controller
    msg.angular.z = rotVel;

    if(rotVel < 0){
	msg.linear.x = maxSpd;
    } else {
        msg.linear.x = maxSpd - (maxSpd - minSpd) * (fabs(rotVel) - setPt);
    }
 
    //publishing message
    pubMessage.publish(msg);
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
        if (msg.ranges[i] < msg.ranges[minIndex_right] && msg.ranges[i] > 0.0){
            minIndex_right = i;
        }
    }
    for(int i = one_third; i < two_thirds; i++)
    {
        if (msg.ranges[i] < msg.ranges[minIndex_middle] && msg.ranges[i] > 0.0){
            minIndex_middle = i;
        }
    }
    for(int i = two_thirds; i < size; i++)
    {
        if (msg.ranges[i] < msg.ranges[minIndex_left] && msg.ranges[i] > 0.0){
            minIndex_left = i;
        }
    }

    //Calculation of angles from indexes and storing data to class variables.
    double angleMin = (size-minIndex_left)*msg.angle_increment;
    double distMin_right = msg.ranges[minIndex_right];
    double distMin_middle = msg.ranges[minIndex_middle];
    double distMin_left = msg.ranges[minIndex_left];
    //double distFront = msg.ranges[size/2];
    double diffE = (distMin_left - WALL_DISTANCE) - e;
    e = distMin_left - WALL_DISTANCE;

    ROS_INFO("dist_left= %f", distMin_left);
    ROS_INFO( "   diffE= %f", diffE);
    ROS_INFO( "   angleMin= %f", angleMin);

    //Invoking method for publishing message
    publishMessage(diffE,distMin_right,distMin_middle,distMin_left,angleMin);
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
