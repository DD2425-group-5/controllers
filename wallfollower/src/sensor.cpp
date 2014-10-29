#include "sensor.hpp"

sensor::sensor(int num){
	number=num;
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