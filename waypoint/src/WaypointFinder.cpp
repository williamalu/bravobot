#include "WaypointFinder.h"
#include <cmath.h>


void WaypointFinder::FindWaypoint(const std_msgs::Int32ConstPtr &msg){
	//Untested code
	//Need to test with GPS

	waypointx = msg->data[0];
	waypointy = msg->data[1];

	if(counter == 1){
		//Generating test waypoints
		//Change to reasonable values.
		setpointx = waypointx + 10;
		setpointy = waypointy + 10;
		counter++;
	}

	if(setpointx != None){
		//Find remaining distance to waypoint
		dremainx = waypointx - setpointx;
		dremainy = waypointy - setpointy;

		if(std::atan(std::abs(dremainy/dremainx)) <= std::tan(0.523598776)){ //atan(|y/x|) <= tan(30 deg)
			//If the heading angle is less than 30 deg, hard turn left/right
			if(dremainx == std::abs(dremainx)){ //remaining x is positive, must turn right
				std::cout << "Hard right" << std::endl;
			}
			else{ //remaining x is negative, must turn left
				std::cout << "Hard left" << std::endl;
			}
		}
		else if(std::atan(std::abs(dremainy/dremainx)) <= std::tan(1.04719755)){ //atan(|y/x|) <= tan(60 deg) and atan(|y/x|) >= tan(30 deg)
			//If the heading angle is less than 60 deg but greater than 30 deg, normal turn left/right.
			if(dremainx == std::abs(dremainx)){ //remaining x is positive, must turn right
				std::cout << "Mid right" << std::endl;
			}
			else{ //remaining x is negative, must turn left
				std::cout << "Mid left" << std::endl;
			}
		}
		else if(std::atan(std::abs(dremainy/dremainx)) <= std::tan(1.48352986)){ //atan(|y/x|) <= tan(85 deg) and atan(|y/x|) >= tan(60 deg)
			//If the heading angle is less than 85 deg but greater than 60 deg, slight turn left/right.
			if(dremainx == std::abs(dremainx)){ //remaining x is positive, must turn right
				std::cout << "Slight right" << std::endl;
			}
			else{ //remaining x is negative, must turn left
				std::cout << "Slight left" << std::endl;
			}
		}
		else{ //If the waypoint is within a cone +-10 degrees from straight, the rover will drive straight ahead.
			std::cout << "Straight ahead" << std::endl;
		}

	}

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
	maxturnrad = 3; 
	counter = 1;
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

