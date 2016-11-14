#include "Factorizer.h"


std::vector <int> Factorizer::prime_factorization( int n ){
	int divisor = 2;
	std::vector <int> factors;
	
	while (n>1) {
		if ( n%divisor == 0 ) {
			factors.push_back(divisor);
			n /= divisor;
		}
		else {
		divisor += 1;
		}	
	}

	return factors;
}

void Factorizer::FactorCallback(const std_msgs::Int32ConstPtr &msg){
	int n = msg->data;
	std::vector <int> factors = prime_factorization(n);

	std_msgs::Int32MultiArray newMsg;
	
	newMsg.data=factors;
	
	pub.publish(newMsg);

}

void Factorizer::init(int argc, char* argv[]){

	subInput = nh.subscribe("number", 10, &Factorizer::InputCallback, this);
	subGPS = nh.subscribe("number", 10, &Factorizer::GPSCallback, this);
	subIMU = nh.subscribe("number", 10, &Factorizer::FactorCallback, this);

	pub = nh.advertise<std_msgs::Int32MultiArray>("primer", 1000);
}







