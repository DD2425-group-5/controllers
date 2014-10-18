#include "blobfollower.hpp"

void blobfollower::rgbdCallback(const geometry_msgs::Twist::ConstPtr& msg) 
{
	
};

blobfollower::blobfollower(int argc, char *argv[])
{
	ros::init(argc, argv, "follower");	//name of node
	ros::NodeHandle handle;			//the handle
	
	pub_motor = handle.advertise<geometry_msgs::Twist>("/motor_controller/Twist", 1000);
	sub_rgbd = handle.subscribe("/tmp/rgb", 1000, &blobfollower::rgbdCallback, this);
	
	ros::Rate loop_rate(10);	//10 Hz
	while (ros::ok())			//main loop of this code
	{
		/*Todo add code for following the nearest point according to rgbd (missing)*/
		
		
		geometry_msgs::Twist msg;	//for controlling the motor
		msg.linear.x = 0.0;
		msg.angular.x = 0.0;
		pub_motor.publish(msg);		//pub to motor
		
		ros::spinOnce();
		loop_rate.sleep();
	}
};

int main(int argc, char *argv[]) 
{
	blobfollower blob(argc,argv);
	return 0;
}
