#ifndef SENSOR_HPP
#define SENSOR_HPP
#include <cmath>

class sensor {
public:
	sensor(){}
	sensor(int num,int x6,int x5,int x4,int x3,int x2,int x1);
	double get_distance();
	int get_value();
	int get_number();
	void calculateDistance(int val);
private:
	double distance;
	int value;
	int number;
	int x6,x5,x4,x3,x2,x1;
};
#endif