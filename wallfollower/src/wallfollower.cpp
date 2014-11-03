#include "wallfollower.hpp"

void wallfollower::sensorCallback(const ras_arduino_msgs::ADConverter msg){
	ROS_INFO("sensor: 1: [%d] 2: [%d]",\
	msg.ch1,\
	msg.ch2);
	//s1.calculateDistance(msg.ch1);
	/*ROS_INFO("sensor distance: 1: [%f]",\
	s1.get_distance());*/
	int tmp[] ={msg.ch1,msg.ch2,msg.ch3,msg.ch4,msg.ch7,msg.ch8};
	for(int i=0;i<6;i++){
		sensors[i].calculateDistance(tmp[i]);
	}
	ROS_INFO("sensor distance: 1: [%f] 2: [%f] 3: [%f] 4: [%f] 5: [%f] 6: [%f] ",\
	sensors[0].get_distance(),\
	sensors[1].get_distance(),\
	sensors[2].get_distance(),\
	sensors[3].get_distance(),\
	sensors[4].get_distance(),\
	sensors[5].get_distance());
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
	sensors[0].calibrate(7.037*std::pow(10, -14),-1.552*std::pow(10, -10),
		1.355*std::pow(10, -7),-6.017*std::pow(10, -5),
		0.0145,-1.864,116.8);
	sensors[1].calibrate(0,-7.107*std::pow(10, -12),1.306*std::pow(10, -8),
		-9.541*std::pow(10, -6),0.003542,-0.7071,72.41);
	sensors[2].calibrate(2.878*std::pow(10, -14),-6.602*std::pow(10, -11),
		6.098*std::pow(10, -8),-2.936*std::pow(10, -5),
		0.007948,-1.204,94.02);
	sensors[3].calibrate(0,-1.043*std::pow(10, -11),1.973*std::pow(10, -8),
		-1.461*std::pow(10, -5),0.00536,-1.014,91.89);
	sensors[4].calibrate(0,0,0,0,0,0,0);
	sensors[5].calibrate(0,0,0,0,0,0,0);
	
	pub_motor = handle.advertise<geometry_msgs::Twist>("/motor_controller/Twist", 1000);
	sub_sensor = handle.subscribe("/arduino/adc", 1000, &wallfollower::sensorCallback, this);
	
	runNode();
}

int main(int argc, char *argv[]) 
{
    wallfollower wallfollower(argc, argv);
}
