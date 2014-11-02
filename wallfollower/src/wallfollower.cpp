#include "wallfollower.hpp"

void wallfollower::sensorCallback(const ras_arduino_msgs::ADConverter msg){
	ROS_INFO("sensor: 1: [%d] 2: [%d]",\
	msg.ch1,\
	msg.ch2);
	s1.calculateDistance(msg.ch1);
	ROS_INFO("sensor distance: 1: [%f]",\
	s1.get_distance());
	/*for(int i=0;i<6;i++){
		
	}*/
}

void wallfollower::runNode(){
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

wallfollower::wallfollower(int argc, char *argv[]){
	ros::init(argc, argv, "wallfollower");	//name of node
	ros::NodeHandle handle;					//the handle
	
	/*setup the sensor calibration*/
	s1.calibrate(7.037*std::pow(10, -14),-1.552*std::pow(10, -10),
		1.355*std::pow(10, -7),-6.017*std::pow(10, -5),
		0.0145,-1.864,116.8);
	
	
	pub_motor = handle.advertise<geometry_msgs::Twist>("/motor_controller/Twist", 1000);
	sub_sensor = handle.subscribe("/arduino/adc", 1000, &wallfollower::sensorCallback, this);
	
	runNode();
	/*ros::Rate loop_rate(10);	//10 Hz
	
	while (ros::ok())			//main loop of this code
	{
		//Todo add code for following the wall
		
		
		geometry_msgs::Twist msg;	//for controlling the motor
		msg.linear.x = 0.0;
		msg.angular.z = 0.0;
		pub_motor.publish(msg);		//pub to motor
		
		ros::spinOnce();
		loop_rate.sleep();
	}*/
}

int main(int argc, char *argv[]) 
{
    wallfollower wallfollower(argc, argv);
}
