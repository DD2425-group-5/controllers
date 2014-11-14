#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <blobdetection/depth_point.h>
#include <rosutil/rosutil.hpp>


class BlobFollower {
private:
    ros::Subscriber sub_rgbd;	//subscribe for depth and angle
    ros::Publisher pub_motor;	//for the motor

    int blobCentre;
    float depth;

    //desired values
    int halfWidth; // half the width of the image
    float desiredDepth; // desired distance from the detected blob

    //tolerances
    int widthTolerance;
    float depthTolerance;

    //speeds to move at
    float linearSpeed;
    float angularSpeed;
	
    //functions
    void rgbdCallback(const blobdetection::depth_point::ConstPtr& msg);
public:
    BlobFollower(int argc, char *argv[]);	//the main loop
    void runNode();
    void update();
};
