
#include <ros/ros.h>
#include "Factorizer.h"

int main(int argc, char* argv[]){
	ros::init(argc, argv, "prime_factorer");	
	Factorizer f;
	f.init(argc,argv);

	ros::spin();
	return 0;
}