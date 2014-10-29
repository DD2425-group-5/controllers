#include "wallfollower.hpp"

void wallfollower::sensorCallback(const ras_arduino_msgs::ADConverter msg){
	ROS_INFO("sensor: 1: [%d] 2: [%d]",\
	msg.ch1,\
	msg.ch2);
}

wallfollower::wallfollower(int argc, char *argv[]){
	ros::init(argc, argv, "wallfollower");	//name of node
	ros::NodeHandle handle;					//the handle
	
	//s1 = sensor(1);
	//s1.get_value();
	
	pub_motor = handle.advertise<geometry_msgs::Twist>("/motor_controller/Twist", 1000);
	sub_sensor = handle.subscribe("/arduino/adc", 1000, &wallfollower::sensorCallback, this);
	
	
	ros::Rate loop_rate(10);	//10 Hz
	
	while (ros::ok())			//main loop of this code
	{
		/*Todo add code for following the wall*/
		
		
		geometry_msgs::Twist msg;	//for controlling the motor
		msg.linear.x = 0.0;
		msg.angular.z = 0.0;
		pub_motor.publish(msg);		//pub to motor
		
		ros::spinOnce();
		loop_rate.sleep();
	}
}

int main(int argc, char *argv[]) 
{
    wallfollower wallfollower(argc, argv);
}