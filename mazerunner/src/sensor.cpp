#include "sensor.hpp"

// !!!!!COMMENTED CODE FOR POLYNOMIAL SENSOR CALIBRATION!!!!!

/*sensor::sensor(int num,double x6,double x5,double x4,double x3,double x2,double x1,double n, bool lrange) : 
	x1(x1),x2(x2),x3(x3),x4(x4),x5(x5),x6(x6),n(n),lrange(lrange){
	number=num;
}*/

/*void sensor::calculateDistance(int val){
	value= val;
	double tmp=0;
	
	tmp=x6 * std::pow(value, 6) +
		x5 * std::pow(value, 5) +
		x4 * std::pow(value, 4) +
		x3 * std::pow(value, 3) +
		x2 * std::pow(value, 2) +
		x1 * value + n;
	distance = tmp;
	if(distance>30 && !lrange){
		distance=30;
	}
}*/

/*void sensor::calibrate(double xx6,double xx5,double xx4,double xx3,double xx2,double xx1,double nn,bool llrange){
	x1=xx1;
	x2=xx2;
	x3=xx3;
	x4=xx4;
	x5=xx5;
	x6=xx6;
	n=nn;
	lrange=llrange;
}*/

// For exponential calibration curve:
sensor::sensor(int num2, double a, double b, double c, double d, bool lrange) : 
	a(a), b(b), c(c), d(d), lrange(lrange){
	number = num2;
}

void sensor::calculateDistanceExp(int val2){
	value = val2;
	double tmp2 = 0;

	tmp2 = a*std::exp(b*val2) + c*std::exp(d*val2);

	distance = tmp2;
	if (distance > 30 && !lrange){
		distance = 30;
	}

}

void sensor::calibrateExp(double aa, double bb, double cc, double dd, bool llrange){

	a = aa;
	b = bb;
	c = cc;
	d = dd;
	lrange = llrange;

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
