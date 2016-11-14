#include "WaypointFinder.h"


void WaypointFinder::FindWaypoint(const std_msgs::Int32ConstPtr &msg){

}

void WaypointFinder::InputCallback(const std_msgs::Float64MultiArray::ConstPtr &msg){
	std::cout << "Yo bitch I'm here!!" << std::endl;
	if(hasInput == false){
	 	hasInput = true;
	}

	waypointx = msg->data[0];
	waypointy = msg->data[1];
}

void WaypointFinder::GPSCallback(const sensor_msgs::NavSatFix &msg){
	currentLat = msg.latitude;
	currentLong = msg.longitude;
}

void WaypointFinder::IMUCallback(const geometry_msgs::Vector3Stamped::ConstPtr &msg){
	headingx = msg->vector.x;
	headingy = msg->vector.y;
	headingz = msg->vector.z;
}

void WaypointFinder::init(int argc, char* argv[]){
	//Initialize input flag
	hasInput = false;
	waypointx = 0;
	waypointy = 0;

	subInput = nh.subscribe("/goto", 10, &WaypointFinder::InputCallback, this);
	subGPS = nh.subscribe("/fix", 10, &WaypointFinder::GPSCallback, this);
	subIMU = nh.subscribe("/imu/mag", 10, &WaypointFinder::IMUCallback, this);

	pub = nh.advertise<std_msgs::Float64MultiArray>("velocity", 1000);

	ros::Rate loop_rate(10);

	int count = 0;
	while (ros::ok())
  	{
	    std_msgs::Float64MultiArray now;

	    now.data.push_back(currentLat);
	    now.data.push_back(currentLong);
	    now.data.push_back(headingx);
	    now.data.push_back(headingy);
	    now.data.push_back(headingz);
	    now.data.push_back(waypointx);
	    now.data.push_back(waypointy);
	    

	    /*now[1] = currentLong;
	    now[2] = headingx;
	    now[3] = headingy;
	    now[4] = headingz;
	    now[5] = waypointx;
	    now[6] = waypointy;*/

	    pub.publish(now);

	    ros::spinOnce();

	    loop_rate.sleep();

	    ++count;
	}

}

