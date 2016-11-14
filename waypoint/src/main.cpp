#include <ros/ros.h>
#include "WaypointFinder.h"

int main(int argc, char* argv[]){
	ros::init(argc, argv, "prime_factorer");	
	WaypointFinder w;
	w.init(argc,argv);

	ros::spin();
	return 0;
}