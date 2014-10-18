#include "ros/ros.h"
#include "geometry_msgs/Twist.h"


class blobfollower {
private:
	ros::Subscriber sub_rgbd;	//subscribe for depth and angle
	ros::Publisher pub_motor;	//for the motor
	
	
	//functions
	void rgbdCallback(const geometry_msgs::Twist::ConstPtr& msg);
public:
	blobfollower(int argc, char *argv[]);	//the main loop
	
};
