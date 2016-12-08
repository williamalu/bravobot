#include <math.h>
#include "cmath"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float32.h"
#include "vector"

//#define PI 3.141592

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

//Define global variables
ros::Publisher pubMessage;
ros::Publisher lidarPub;
double e_left = 0;
double e_right = 0;
float setPt = -0.05;
float wallDist = 1.5;
float P = 0.50;
float D = 0.25;
float minSpd = 0.30;
float maxSpd = 0.65;
float minObstDist= 0.10;
float blockDist = 0.80;
float angleCoef = 1;
float midCoef = 0.05;
float maxLeftSpd = -0.35;
float angleJump = 0.17;
float cornerTrackAngle = 0.02;
//int lookAhead = 50;
bool block = false;
float lastScan[512];
int count = 0;
float prevMinAngles[3] = {0.0, 0.0, 0.0};
float prevWallDist = 0;

//Publisher
void publishMessage(double diffE_right, double diffE_left, double distMin_left, double distMin_middle, double angleMin_right, double angleMin_middle, double angleMin_left)
{
    //preparing message
    geometry_msgs::Twist msg;

    //Determine direction
    double rotVel_left = 0;

    if(block){
        rotVel_left = setPt + -(P * ((e_left) + D * diffE_left));
    } else {
        rotVel_left = setPt + -(P * e_left + D * diffE_left) + angleCoef * (angleMin_left);
    }

    double rotVel_right = 0;
    double rotVel_middle = 0;

    if(e_right < 0) {
        rotVel_right = setPt + (P * e_right + D * diffE_right) + angleCoef * (angleMin_right);    //PD controller
    }

    if(distMin_middle < 1.5){
        rotVel_middle = midCoef*((P*distMin_middle) + angleCoef * (angleMin_middle));
    }

    ROS_INFO("rotVel_left= %f", rotVel_left);
    ROS_INFO("rotVel_right= %f", rotVel_right);
    ROS_INFO("rotVel_middle= %f", rotVel_middle);

    double rotVel = rotVel_left + rotVel_right + rotVel_middle;

    if(rotVel < maxLeftSpd){
        rotVel = maxLeftSpd;
    }
    if(!std::isfinite(e_left) && !std::isfinite(e_right)){
        rotVel = setPt;
    }

    msg.angular.z = rotVel;

    //Map speed based on angular velocity
    if(rotVel < 0){
	    msg.linear.x = maxSpd;
    } else if(rotVel == setPt){
        msg.linear.x = maxSpd;
    } else{
        msg.linear.x = maxSpd - (maxSpd - minSpd) * (fabs(rotVel - setPt));
    }
 
    //publishing message
    pubMessage.publish(msg);
    
    ROS_INFO("PARAMETER VALUES: %f %f %f %f %f %f %f %f %f %f %f %f %f", P, D, minSpd, maxSpd, setPt, maxLeftSpd, wallDist, blockDist, minObstDist, angleJump, angleCoef, midCoef);

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
void messageCallback(sensor_msgs::LaserScan msg)
{
    int size = msg.ranges.size();

    float lidarDists[size];

    /*
    for(int i = 0; i < size+4; ++i){
        //ROS_INFO("%f", msg.ranges[i]);
        if(isinf(msg.ranges[i+48]) || isnan(msg.ranges[i+48])){
            msg.ranges[i+48] = 0;
            //ROS_INFO("%f", msg.ranges[i]);
        }
    }

    //Smooth out lidar data
    for(int i = 0; i < size; ++i){
        int numFinite = 0;
        float smooth = 0;

        for(int n = 48; n < 53; ++n){
            if(msg.ranges[i+n] != 0){
                smooth += msg.ranges[i+n];
                numFinite++;
            }
        }

        if(numFinite < 3){
            lidarDists[i] = 0;
        } else {
            lidarDists[i] = smooth/numFinite;
        }

        //ROS_INFO("%f %f %f %f %f", msg.ranges[i+48], msg.ranges[i+49], msg.ranges[i+50], msg.ranges[i+51], msg.ranges[i+52]);
        //ROS_INFO("%f", lidarDists[i]);
    }*/

    for(int i = 0; i < size; i++){
        float delta = fabs(msg.ranges[i] - lastScan[i]);
        if(delta < 0.1){
            lidarDists[i] = msg.ranges[i];
        } else{
            lidarDists[i] = 0;
        }

        lastScan[i] = msg.ranges[i];
    }

    ROS_INFO("DATA SMOOTHED");

    for(unsigned int i = 0; i < size; ++i){
        ROS_INFO("%f", msg.ranges[i]);
        ROS_INFO("%f", lidarDists[i]);
        msg.ranges[i] = lidarDists[i];
        ROS_INFO("i= %i", i);
    }



    //ROS_INFO("PUBLISHING!");

    lidarPub.publish(msg);

    //Variables whith index of highest and lowest value in array.
    int one_third = size/3;
    int two_thirds = 2*one_third;
    int minIndex_right = one_third-1;
    int minIndex_middle = two_thirds-1;
    int minIndex_left = size;

    //This cycle goes through array and finds minimum
    for(int i = 50; i < one_third; i++)
    {
        if (lidarDists[i] < lidarDists[minIndex_right] && lidarDists[i] > minObstDist){
            minIndex_right = i;
        }
    }
    for(int i = one_third; i < two_thirds; i++)
    {
        if (lidarDists[i] < lidarDists[minIndex_middle] && lidarDists[i] > minObstDist){
            minIndex_middle = i;
        }
    }
    for(int i = two_thirds; i < (size-50); i++)
    {
        if (lidarDists[i] < lidarDists[minIndex_left] && lidarDists[i] > minObstDist){
            minIndex_left = i;
        }
    }

    //Calculation of angles from indexes and storing data to class variables.
    double angleMin_left = (size-minIndex_left)*msg.angle_increment;
    double angleMin_right = (minIndex_right)*msg.angle_increment;
    double angleMin_middle = (size*3/4-minIndex_middle)*msg.angle_increment;

    double distMin_right = lidarDists[minIndex_right];
    double distMin_middle = lidarDists[minIndex_middle];
    double distMin_left = lidarDists[minIndex_left];
    //double distFront = msg.ranges[size/2];



    double diffE_left = 0;
    double diffE_right = 0;

    if(prevMinAngles[0] > angleJump && prevMinAngles[1] > angleJump && prevMinAngles[2] > angleJump){
        block = true;
        prevWallDist = distMin_left;
    } else{

    }
    if (block){
        ROS_INFO("block");
        float curbDist = lidarDists[75];

        if(curbDist > blockDist){
            ROS_INFO("curbDist= %f", curbDist);
            diffE_left = (curbDist - wallDist) - e_left;
            e_left = curbDist - wallDist;

            diffE_right = (distMin_right - wallDist) - e_right;
            e_right = distMin_right - wallDist;
        } else {
            diffE_left = (distMin_left - blockDist) - e_left;
            e_left = distMin_left - blockDist;


            diffE_right = (distMin_right - blockDist) - e_right;
            e_right = distMin_right - blockDist;
        }

        if(distMin_left > blockDist + 0.1){
            block = false;
        }
    } else {
        ROS_INFO("no block");
        diffE_left = (distMin_left - wallDist) - e_left;
        e_left = distMin_left - wallDist;

        diffE_right = (distMin_right - wallDist) - e_right;
        e_right = distMin_right - wallDist;
    }

    ROS_INFO("dist_left= %f", distMin_left);
    ROS_INFO( "   diffE_left= %f", diffE_left);
    ROS_INFO( "   angleMin_left= %f", angleMin_left);
    ROS_INFO("dist_right= %f", distMin_right);
    ROS_INFO( "   diffE_right= %f", diffE_right);
    ROS_INFO( "   angleMin_right= %f", angleMin_right);
    ROS_INFO("dist_middle= %f", distMin_middle);
    ROS_INFO( "   angleMin_middle= %f", angleMin_middle);

    for(int i = 1; i < 2; i++){
        prevMinAngles[i+1] = prevMinAngles[i];
    }

    prevMinAngles[0] = angleMin_left;

    //Invoking method for publishing message
    publishMessage(diffE_right, diffE_left, distMin_left, distMin_middle, angleMin_right, angleMin_middle, angleMin_left);
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
    angleCoef = msg->data;
    ROS_INFO("ANGLE_COEF CHANGED.");
}

void setMidCoef(const std_msgs::Float32::ConstPtr& msg){
    midCoef = msg->data;
    ROS_INFO("MID_COEF CHANGED.");
}

void setMaxLeftSpd(const std_msgs::Float32::ConstPtr& msg){
    maxLeftSpd = msg->data;
    ROS_INFO("MAX_LEFT_SPEED CHANGED.");
}

void setAngleJump(const std_msgs::Float32::ConstPtr& msg){
    angleJump = msg->data;
    ROS_INFO("ANGLE_JUMP CHANGED.");
}

void setBlockDist(const std_msgs::Float32::ConstPtr& msg){
    blockDist = msg->data;
    ROS_INFO("MIN_WALL_DIST CHANGED.");
}

int main(int argc, char **argv)
{
    //Initialization of node
    ros::init(argc, argv, "wall_follow");
    ros::NodeHandle n;

    //Creating publisher
    pubMessage = n.advertise<geometry_msgs::Twist>(PUBLISHER_TOPIC, PUBLISHER_BUFFER_SIZE);
    lidarPub = n.advertise<sensor_msgs::LaserScan>("/smoothScan", 1000);

    //Creating subscriber and publisher
    ros::Subscriber sub = n.subscribe(SUBSCRIBER_TOPIC, SUBSCRIBER_BUFFER_SIZE, messageCallback);
    ros::Subscriber Psub = n.subscribe("wall_follow/P", 1, setP);
    ros::Subscriber Dsub = n.subscribe("wall_follow/D", 1, setD);
    ros::Subscriber maxSpdSub = n.subscribe("wall_follow/maxSpd", 1, setMaxSpd);
    ros::Subscriber minSpdSub = n.subscribe("wall_follow/minSpd", 1, setMinSpd);
    ros::Subscriber wallDistSub = n.subscribe("wall_follow/wallDist", 1, setWallDist);
    ros::Subscriber setPtSub = n.subscribe("wall_follow/setPt", 1, setSetPt);
    ros::Subscriber minObstDistSub = n.subscribe("wall_follow/minObstDist", 1, setMinObstDist);
    ros::Subscriber angleCoefSub = n.subscribe("wall_follow/angleCoef", 1, setAngleCoef);
    ros::Subscriber midCoefSub = n.subscribe("wall_follow/midCoef", 1, setMidCoef);
    ros::Subscriber maxLeftSpdSub = n.subscribe("wall_follow/maxLeftSpd", 1, setMaxLeftSpd);
    ros::Subscriber angleJumpSub = n.subscribe("wall_follow/angleJump", 1, setAngleJump);
    ros::Subscriber blockDistSub = n.subscribe("wall_follow/block", 1, setBlockDist);
    ros::spin();
    
    return 0;
}
