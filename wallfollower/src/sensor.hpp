#ifndef SENSOR_HPP
#define SENSOR_HPP
#include <cmath>

class sensor {
public:
	sensor(){}
	sensor(int num,double x6,double x5,double x4,double x3,double x2,double x1,double n);
	double get_distance();
	int get_value();
	int get_number();
	void calculateDistance(int val);
	void calibrate(double xx6,double xx5,double xx4,double xx3,double xx2,double xx1,double nn);
	
private:
	double distance;
	int value;
	int number;
	double x6,x5,x4,x3,x2,x1,n;
};
#endif
