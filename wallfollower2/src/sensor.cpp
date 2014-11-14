#include "sensor.hpp"

sensor::sensor(int num,double x6,double x5,double x4,double x3,double x2,double x1,double n, bool lrange) : 
	x1(x1),x2(x2),x3(x3),x4(x4),x5(x5),x6(x6),n(n),lrange(lrange){
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
	/*if(distance>maxRange){
		distance=-1;
	}
	for(int i=0;i<29;i++){
		record[i]=record[i+1];
		if(n==116.8){
			ROS_INFO("%d: %f",i,record[i]);
		}
	}
	record[29]=distance;
	if(n==116.8){
		ROS_INFO("%d: %f",19,record[19]);
	}*/
	// TEMP SOULUTION REALY UGLY
	if(value<120){
		value=-1;
	}
	for(int i=0;i<29;i++){
		record[i]=record[i+1];
		/*if(n==116.8){
			ROS_INFO("%d: %f",i,record[i]);
		}*/
	}
	record[29]=value;
}

int sensor::hasContact(){
	if(record[29]>0 /*&& record[28]>0 /*&& record[17]>0 */){
		return 1;
	}
	else{
		return 0;
	}
}

int sensor::hasBump(){
	int count=0;
	for(int i=0;i<30;i++){
		if(record[i]>0){
			count++;
		}
	}
	if(count>5){
		return 1;
	}
	else{
		return 0;
	}
}

void sensor::calibrate(double xx6,double xx5,double xx4,double xx3,double xx2,double xx1,double nn,int max,int min){
	x1=xx1;
	x2=xx2;
	x3=xx3;
	x4=xx4;
	x5=xx5;
	x6=xx6;
	n=nn;
	maxRange=max;
	minRange=min;
	value=-1;
	distance=-1;
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
