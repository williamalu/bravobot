// Olin College Fundamentals of Robotics 2016
// Last edited by Lydia Z. 12/7/16

#include "WaypointFinder.h"
#include <cmath>


void WaypointFinder::FindWaypoint(const std_msgs::Float64MultiArray::ConstPtr &msg){
	std::cout << "I'm here!" << std::endl;

	setpointx = msg->data[0];
	setpointy = msg->data[1];

	std::cout << "Setpoint: " << setpointx << std::endl;

	try{

		// if(counter <= 1){
		// 	// //Generating test waypoints
		// 	// //Change to reasonable values.
		// 	// waypointx = setpointx + 10;
		// 	// waypointy = setpointy + 10;

		// 	for ( i=0; i <= 6; i++ ) {
		// 		//If waypoint has been reached, reset waypoint arbiter array to zero in preparation for finding the next waypoint.
		// 		arbArray [i] = 0;
		// 	}
		// }

			// counter++;

		for ( i=0; i < 7; i++ ) {
			arbArray [i] = 0;
		}

		//Find remaining distance to waypoint
		dremainx = setpointx - waypointx;
		dremainy = setpointy - waypointy;

		if(std::atan(std::abs(dremainy/dremainx)) <= std::tan(0.523598776)){ //atan(|y/x|) <= tan(30 deg)
			//If the heading angle to the next waypoint is less than 30 deg, hard turn left/right
			if(dremainx == std::abs(dremainx)){ //remaining x is positive, must turn right
				direction << "Hard right";

				arbArray[6] = 1;
				arbArray[5] = 0.5;
			}
			else{ //remaining x is negative, must turn left
				direction << "Hard left";

				arbArray[0] = 1;
				arbArray[1] = 0.5;
			}
		}
		else if(std::atan(std::abs(dremainy/dremainx)) <= std::tan(1.04719755)){ //atan(|y/x|) <= tan(60 deg) and atan(|y/x|) >= tan(30 deg)
			//If the heading angle is less than 60 deg but greater than 30 deg, normal turn left/right.
			if(dremainx == std::abs(dremainx)){ //remaining x is positive, must turn right
				direction << "Mid right";

				arbArray[4] = 0.5;
				arbArray[5] = 1;
				arbArray[6] = 0.5;
			}
			else{ //remaining x is negative, must turn left
				direction << "Mid left";
				arbArray[0] = 0.5;
				arbArray[1] = 1;
				arbArray[2] = 0.5;
			}
		}
		else if(std::atan(std::abs(dremainy/dremainx)) <= std::tan(1.48352986)){ //atan(|y/x|) <= tan(85 deg) and atan(|y/x|) >= tan(60 deg)
			//If the heading angle is less than 85 deg but greater than 60 deg, slight turn left/right.
			if(dremainx == std::abs(dremainx)){ //remaining x is positive, must turn right
				direction << "Slight right";
				arbArray[3] = 0.5;
				arbArray[4] = 1;
				arbArray[5] = 0.5;
			}
			else{ //remaining x is negative, must turn left
				direction << "Slight left";
				arbArray[1] = 0.5;
				arbArray[2] = 1;
				arbArray[3] = 0.5;
			}
		}
		else{ //If the waypoint is within a cone +-10 degrees from straight, the rover will drive straight ahead.
			direction << "Straight ahead";
			arbArray[2] = 0.5;
			arbArray[3] = 1;
			arbArray[4] = 0.5;
		}
		std::cout << direction.str() << std::endl;
	}
	catch (...){
		std::cout << "No setpoint" << std::endl;
	}

}

void WaypointFinder::InputCallback(const std_msgs::Float64MultiArray::ConstPtr &msg){
	if(hasInput == false){
	 	hasInput = true;
	}

	waypointx = msg->data[0];
	waypointy = msg->data[1];

	std::cout << "Input Callback" << std::endl;
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

void WaypointFinder::WaypointList(const sensor_msgs::NavSatFix &msg){
	//FIRST DRAFT IN PROGRESS

	int waypoints [5] = { (0,0),(0,0),(0,0),(0,0),(0,0) }; // The current queue of waypoints

	R = 6371; //Radius of the earth in km

	currentLat = msg.latitude;
	currentLong = msg.longitude;

	dLong = currentLong - wpLong;
    dLat = currentLat - wpLat;

	//Haversine formula- calculate arc distance between two GPS points
	a = std::pow(std::sin(dLat/2),2) + std::cos(wpLat) * std::cos(currentLat) * std::pow(sin(dLong/2),2);
	calc = 2 * (std::atan2(std::sqrt(a), std::sqrt(1-a)));
	arcDist = R * calc;


	std::cout << "Latitude" << currentLat << std::endl;

	// if (currentLat == )

	if (waypointFound == true){ //If a waypoint was found...
		std::cout << "Next WP" << std::endl;
		waypoints[currwp] = (0,0); //Set the current waypoint to default value, clearing it off the list.
		counter = 1; //Reset previous heading weights in arbiter array (arbArray) to 0
		waypointFound = false;
		direction.str(std::string());
		direction.clear();
	}
	else{
		for ( i=0; i <= 4; i++ ) {
			// For waypoint i in list waypoints, check if waypoint exists (default value is (0,0))
			// Once a valid waypoint is found, set currwp (currentwaypoint) to waypoint i.
			if (waypoints[i] != (0,0)){ //If waypoint i in waypoints exists, make it our current waypoint.
				currwp = i;
			}
		}
	}
}

void WaypointFinder::init(int argc, char* argv[]){
	//Initialize input flag
	hasInput = false;
	waypointFound = false;
	maxturnrad = 3;  // meters
	counter = 1;
	waypointx = 0;
	waypointy = 0;

	int arbArray [7] = { 0,0,0,0,0,0,0 }; 

	// ros::Publisher wp_pub = n.advertise<std_msgs::String>("wplist", 1000);

	// subInput = nh.subscribe("/wplist", 10, &WaypointFinder::InputCallback, this); //You cannot run subInput and subWPfinder at the same time.
	subWPfinder = nh.subscribe("/wplist", 10, &WaypointFinder::FindWaypoint, this);

	// subGPS = nh.subscribe("/fix", 10, &WaypointFinder::GPSCallback, this);
	subWPlist = nh.subscribe("/fix", 10, &WaypointFinder::WaypointList, this);

	subIMU = nh.subscribe("/imu/mag", 10, &WaypointFinder::IMUCallback, this);

	pubvel = nh.advertise<std_msgs::String>("velocity", 1000);
	pubwp = nh.advertise<std_msgs::Float64MultiArray>("wplist", 1000);

	ros::Rate loop_rate(10);

	// int count = 0;
	while (ros::ok())
  	{

	    // std_msgs::Float64MultiArray now;

	    // now.data.push_back(currentLat);
	    // now.data.push_back(currentLong);
	    // now.data.push_back(headingx);
	    // now.data.push_back(headingy);
	    // now.data.push_back(headingz);
	    // now.data.push_back(waypointx);
	    // now.data.push_back(waypointy);

  		// FindWaypoint(const std_msgs::Float64MultiArray::ConstPtr &msg)


	    /*now[1] = currentLong;
	    now[2] = headingx;
	    now[3] = headingy;
	    now[4] = headingz;
	    now[5] = waypointx;
	    now[6] = waypointy;*/
  		// direction << "Slight left";
  		// wp_pub.publish("");
  		pubdirection.data = direction.str();
	    pubvel.publish(pubdirection);

	    ros::spinOnce();

	    loop_rate.sleep();

	    // ++count;
	    std::cout << "   " << direction.str() << std::endl;
	//     std::cout << count << "   " << pubdirection.data.c_str() << "Hoi" << std::endl;
	}

}

