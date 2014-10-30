#include "sensor.hpp"

sensor::sensor(int num,int x6,int x5,int x4,int x3,int x2,int x1) : 
	x1(x1),x2(x2),x3(x3),x4(x4),x5(x5),x6(x6){
	number=num;
}

void sensor::calculateDistance(int val){
	value= val;
	double tmp=0;
	tmp=x6 * std::pow(value, 5) +
		x5 * std::pow(value, 4) +
		x4 * std::pow(value, 3) +
		x3 * std::pow(value, 2) +
		x2 * value + x1;
	distance = tmp;
}

int sensor::get_number(){
	return number;
}

int sensor::get_value(){
	return value;
}

double sensor::get_distance(){
	return distance;
}