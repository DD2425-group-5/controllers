#include "blobfollower.hpp"


blobfollower::blobfollower(int argc, char *argv[]) {
    ros::init(argc, argv, "follower");	//name of node
    ros::NodeHandle handle;			//the handle

    pub_motor = handle.advertise<geometry_msgs::Twist>("/motor_controller/Twist", 1000);
    sub_rgbd = handle.subscribe("/vision/closest_blob", 1000, &blobfollower::rgbdCallback, this);

    halfwidth = 640/2;
    desired_depth = 500;

    width_tolerance = 30;
    depth_tolerance = 60;

    linear_speed = 0.1;
    angular_speed = 0.1;
}

void blobfollower::rgbdCallback(const blobdetection::depth_point::ConstPtr& msg) {
    x = msg->x;
    depth = msg->depth;
}

void blobfollower::update() {

    //calculate speeds
    float ls = 0;
    if(depth < desired_depth-depth_tolerance)
        ls = -linear_speed;
    else if(depth > desired_depth+depth_tolerance)
        ls = linear_speed;

    float as = 0;
    if(x < halfwidth-width_tolerance)
        as = angular_speed;
    else if(x > halfwidth+width_tolerance)
        as = -angular_speed;

    //publish
    geometry_msgs::Twist msg;	//for controlling the motor
    msg.linear.x = ls;
    msg.angular.z = as;
    pub_motor.publish(msg);		//pub to motor
};

int main(int argc, char *argv[]) 
{
	blobfollower blob(argc,argv);
    ros::Rate loop_rate(10);
    while(ros::ok()) {
        blob.update();
        ros::spinOnce();
        loop_rate.sleep();
    }

	return 0;
}
