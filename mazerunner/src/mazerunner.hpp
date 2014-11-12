#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Twist.h"
#include "ras_arduino_msgs/ADConverter.h"
#include "sensor.cpp"
#include <rosutil/rosutil.hpp>
#include <math.h>
#include <sstream>

class mazerunner {
public:
	mazerunner(int argc, char *argv[]);
	
private:
	double GP_left;
	double GI_left;
	double GD_left;
	double Gcontr_left;
	
	double GP_right;
	double GI_right;
	double GD_right;
	double Gcontr_right;

	double setpoint_left;
	double setpoint_right;
	double angvel_left;
	double angvel_right;

	float contr_time;
	float contr_freq;

	double a0;
	double b0;
	double c0;
	double d0;

	double a1;
	double b1;
	double c1;
	double d1;

	double a2;
	double b2;
	double c2;
	double d2;

	double a3;
	double b3;
	double c3;
	double d3;

	double a4;
	double b4;
	double c4;
	double d4;

	double a5;
	double b5;
	double c5;
	double d5;

	sensor sensors[6];
	sensor s1;
	ros::Subscriber sub_sensor;	//sub to get distance values
	ros::Publisher pub_motor;	//for the motor
	
	void runNode();
	void sensorCallback(const ras_arduino_msgs::ADConverter msg);
};
