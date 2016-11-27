#pragma once
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <vector>

class WaypointFinder
{

public:

	void init(int argc, char* argv[]);

	void FindWaypoint(const std_msgs::Float64MultiArray::ConstPtr &msg);

	void IMUCallback(const geometry_msgs::Vector3Stamped::ConstPtr &msg);
	void GPSCallback(const sensor_msgs::NavSatFix &msg);
	void InputCallback(const std_msgs::Float64MultiArray::ConstPtr &msg);


public:	
	ros::NodeHandle nh;
	ros::Subscriber subGPS;
	ros::Subscriber subIMU;
	ros::Subscriber subInput;
	ros::Subscriber subWPfinder;

	bool hasInput;
	int pubInterval;
	int counter;
	
	//current state
	float currentLat;
	float currentLong;

	float headingx;
	float headingy;
	float headingz;

	float waypointx;
	float waypointy;
	float setpointx;
	float setpointy;
	float dremainx;
	float dremainy;

	std_msgs::String pubdirection;
	std::stringstream direction;
	// std::stringstream s;


	int maxturnrad;

	ros::Publisher pub;

};

