#pragma once
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <vector>

class WaypointFinder
{

public:

	void init(int argc, char* argv[]);

	void FindWaypoint(const std_msgs::Int32ConstPtr &msg);

	void IMUCallback(const geometry_msgs::Vector3Stamped::ConstPtr &msg);
	void GPSCallback(const std_msgs::Int32ConstPtr &msg);
	void InputCallback(const std_msgs::Float64MultiArray::ConstPtr &msg);


public:	
	ros::NodeHandle nh;
	ros::Subscriber subGPS;
	ros::Subscriber subIMU;
	ros::Subscriber subInput;

	bool hasInput;
	int pubInterval;
	
	//current state
	float currentLat;
	float currentLong;

	float headingx;
	float headingy;
	float headingz;

	float waypointx;
	float waypointy;

	ros::Publisher pub;

};

