#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ras_arduino_msgs/ADConverter.h"
#include "sensor.cpp"


class wallfollower {
public:
	wallfollower(int argc, char *argv[]);
	
private:
	sensor s1;
	ros::Subscriber sub_sensor;	//sub to get distance values
	ros::Publisher pub_motor;	//for the motor
	
	void sensorCallback(const ras_arduino_msgs::ADConverter::ConstPtr& msg);
};
