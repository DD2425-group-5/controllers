#ifndef SENSOR_HPP
#define SENSOR_HPP
#include <cmath>

// !!!!!COMMENTED CODE FOR POLYNOMIAL SENSOR CALIBRATION!!!!!

class sensor {
public:
	//sensorExp(){}
	sensor(){}
	//sensor(int num,double x6,double x5,double x4,double x3,double x2,double x1,double n, bool lrange);
	double get_distance();
	int get_value();
	int get_number();
	//void calculateDistance(int val);
	//void calibrate(double xx6,double xx5,double xx4,double xx3,double xx2,double xx1,double nn, bool lrange);
	
	sensor(int num2, double a, double b, double c, double d, bool lrange);
	void calculateDistanceExp(int val2);
	void calibrateExp(double aa, double bb, double cc, double dd, bool llrange);
private:
	double distance;
	int value;
	int number;
	//double x1,x2,x3,x4,x5,x6,n;
	double a, b, c, d;
	bool lrange;
};
#endif
