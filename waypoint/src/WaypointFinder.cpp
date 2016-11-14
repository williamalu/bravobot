#include "WaypointFinder.h"


void WaypointFinder::FindWaypoint(const std_msgs::Int32ConstPtr &msg){

	std::cout << "yo" << std::endl;

}

void WaypointFinder::InputCallback(const std_msgs::Int32ConstPtr &msg){
	
}

void WaypointFinder::GPSCallback(const std_msgs::Int32ConstPtr &msg){
	
}

void WaypointFinder::IMUCallback(const std_msgs::Int32ConstPtr &msg){
	
}

void WaypointFinder::init(int argc, char* argv[]){

	subInput = nh.subscribe("Input", 10, &WaypointFinder::InputCallback, this);
	subGPS = nh.subscribe("GPS", 10, &WaypointFinder::GPSCallback, this);
	subIMU = nh.subscribe("IMU", 10, &WaypointFinder::IMUCallback, this);

	pub = nh.advertise<std_msgs::Int32MultiArray>("primer", 1000);
}

