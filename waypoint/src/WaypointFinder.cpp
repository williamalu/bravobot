#include "WaypointFinder.h"
#include <cmath>


void WaypointFinder::FindWaypoint(const std_msgs::Float64MultiArray::ConstPtr &msg){
	//Untested code
	//Need to test with GPS

	setpointx = msg->data[0];
	setpointy = msg->data[1];

	if(counter >= 0){
		//Generating test waypoints
		//Change to reasonable values.
		waypointx = setpointx + 10;
		waypointy = setpointy + 10;
		counter++;
	}

	if(setpointx != NULL){
		//Find remaining distance to waypoint
		dremainx = setpointx - waypointx;
		dremainy = setpointy - waypointy;

		if(std::atan(std::abs(dremainy/dremainx)) <= std::tan(0.523598776)){ //atan(|y/x|) <= tan(30 deg)
			//If the heading angle is less than 30 deg, hard turn left/right
			if(dremainx == std::abs(dremainx)){ //remaining x is positive, must turn right
				direction << "Hard right";
				std::cout << direction << std::endl;
			}
			else{ //remaining x is negative, must turn left
				direction << "Hard left";
				std::cout << direction << std::endl;
			}
		}
		else if(std::atan(std::abs(dremainy/dremainx)) <= std::tan(1.04719755)){ //atan(|y/x|) <= tan(60 deg) and atan(|y/x|) >= tan(30 deg)
			//If the heading angle is less than 60 deg but greater than 30 deg, normal turn left/right.
			if(dremainx == std::abs(dremainx)){ //remaining x is positive, must turn right
				direction << "Mid right";
				std::cout << direction << std::endl;
			}
			else{ //remaining x is negative, must turn left
				direction << "Mid left";
				std::cout << direction << std::endl;
			}
		}
		else if(std::atan(std::abs(dremainy/dremainx)) <= std::tan(1.48352986)){ //atan(|y/x|) <= tan(85 deg) and atan(|y/x|) >= tan(60 deg)
			//If the heading angle is less than 85 deg but greater than 60 deg, slight turn left/right.
			if(dremainx == std::abs(dremainx)){ //remaining x is positive, must turn right
				direction << "Slight right";
 				std::cout << direction << std::endl;
			}
			else{ //remaining x is negative, must turn left
				direction << "Slight left";
				std::cout << direction << std::endl;
			}
		}
		else{ //If the waypoint is within a cone +-10 degrees from straight, the rover will drive straight ahead.
			direction << "Straight ahead";
			std::cout << direction << std::endl;
		}

	}

}

void WaypointFinder::InputCallback(const std_msgs::Float64MultiArray::ConstPtr &msg){
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

	subWPfinder = nh.subscribe("/goto", 10, &WaypointFinder::FindWaypoint, this);

	pub = nh.advertise<std_msgs::String>("velocity", 1000);

	ros::Rate loop_rate(10);

	int count = 0;
	while (ros::ok())
  	{
	    //std_msgs::Float64MultiArray now;

	    /*now.data.push_back(currentLat);
	    now.data.push_back(currentLong);
	    now.data.push_back(headingx);
	    now.data.push_back(headingy);
	    now.data.push_back(headingz);
	    now.data.push_back(waypointx);
	    now.data.push_back(waypointy);
	    now.data.push_back(direction);*/

	    /*now[1] = currentLong;
	    now[2] = headingx;
	    now[3] = headingy;
	    now[4] = headingz;
	    now[5] = waypointx;
	    now[6] = waypointy;*/

  		pubdirection.data = direction.str();
	    pub.publish(pubdirection);

	    ros::spinOnce();

	    loop_rate.sleep();

	    ++count;
	    std::cout << count << "   " << direction.str() << "Hoi" << std::endl;
	    // std::cout << count << "   " << pubdirection.data.c_str() << "Hoi" << std::endl;
	}

}

