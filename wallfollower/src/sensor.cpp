#include "sensor.hpp"

sensor::sensor(int num,double x6,double x5,double x4,double x3,double x2,double x1,double n) : 
	x1(x1),x2(x2),x3(x3),x4(x4),x5(x5),x6(x6),n(n){
	number=num;
}

void sensor::calculateDistance(int val){
	value= val;
	double tmp=0;
	tmp=x6 * std::pow(value, 6) +
		x5 * std::pow(value, 5) +
		x4 * std::pow(value, 4) +
		x3 * std::pow(value, 3) +
		x2 * std::pow(value, 2) +
		x1 * value + n;
	distance = tmp;
}

void sensor::calibrate(double xx6,double xx5,double xx4,double xx3,double xx2,double xx1,double nn){
	x1=xx1;
	x2=xx2;
	x3=xx3;
	x4=xx4;
	x5=xx5;
	x6=xx6;
	n=nn;
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
