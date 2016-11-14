#include "WaypointFinder.h"


void WaypointFinder::FindWaypoint(const std_msgs::Int32ConstPtr &msg){

}

void WaypointFinder::InputCallback(const std_msgs::Float64MultiArray::ConstPtr &msg){
	if(!hasInput){
	 	hasInput = true;
	}
	waypointx = msg->data[0];
	waypointy = msg->data[1];
}

void WaypointFinder::GPSCallback(const std_msgs::Int32ConstPtr &msg){

}

void WaypointFinder::IMUCallback(const geometry_msgs::Vector3Stamped::ConstPtr &msg){
	headingx = msg->vector.x;
	headingy = msg->vector.y;
	headingz = msg->vector.z;
}

void WaypointFinder::init(int argc, char* argv[]){
	//Initialize input flag
	hasInput = false;

	subInput = nh.subscribe("Input", 10, &WaypointFinder::InputCallback, this);
	subGPS = nh.subscribe("GPS", 10, &WaypointFinder::GPSCallback, this);
	subIMU = nh.subscribe("IMU", 10, &WaypointFinder::IMUCallback, this);

	pub = nh.advertise<std_msgs::Int32MultiArray>("primer", 1000);
}

