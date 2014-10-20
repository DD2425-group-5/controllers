#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <blobdetection/depth_point.h>


class blobfollower {
private:
	ros::Subscriber sub_rgbd;	//subscribe for depth and angle
	ros::Publisher pub_motor;	//for the motor

    int x;
    float depth;

    //desired values
    int halfwidth;
    float desired_depth;

    //tolerances
    int width_tolerance;
    float depth_tolerance;

    //speeds to move at
    float linear_speed;
    float angular_speed;
	
	//functions
    void rgbdCallback(const blobdetection::depth_point::ConstPtr& msg);

public:
	blobfollower(int argc, char *argv[]);	//the main loop
    void update();
};
