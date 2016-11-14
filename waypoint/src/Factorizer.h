#pragma once

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <vector>

class WaypointNav
{

public:

	void init(int argc, char* argv[]);

	std::vector <int> prime_factorization( int n );

	void WaypointCallback(const std_msgs::Int32ConstPtr &msg);


public:	
	ros::NodeHandle nh;
	ros::Subscriber subGPS;
	ros::Subscriber subIMU;
	ros::Subscriber subInput;

	int currentPosx;
	int currentPosy; 

	etc.
	ros::Publisher pub;

};

