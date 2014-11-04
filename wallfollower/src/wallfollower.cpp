#include "wallfollower.hpp"

void wallfollower::sensorCallback(const ras_arduino_msgs::ADConverter msg){
	int tmp[] ={msg.ch1,msg.ch2,msg.ch3,msg.ch4,msg.ch7,msg.ch8};
	for(int i=0;i<6;i++){
		sensors[i].calculateDistance(tmp[i]);
	}
	ROS_INFO("sensor distance: 1: [%f] 2: [%f] 3: [%f] 4: [%f] 5: [%f] 6: [%f] \n\n",\
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
		/*work in progress*/
		

		// IMPORTANT NOTICE!
		// msg.angular.z > 0 -> turning LEFT
		// msg.angular.z < 0 -> turning RIGHT


		geometry_msgs::Twist msg;	//for controlling the motor
		
		double angvel_left = sensors[0].get_distance() - sensors[2].get_distance();
		double angvel_right = sensors[1].get_distance() - sensors[3].get_distance();

		// Make angvel_left and angvel_right positive values
		if (angvel_left < 0) {
		angvel_left = -angvel_left;
		}

		if (angvel_right < 0) {
		angvel_right = -angvel_right;
		}
		
		double angvel_diff = angvel_left - angvel_right;
		//double new_sensor0 = sensors[0].get_distance() + angvel_left;
		

		ROS_INFO(" angvel_left= %f", angvel_left);
		ROS_INFO(" angvel_right= %f", angvel_right);
		ROS_INFO(" angvel_diff = %f", angvel_diff);
						

		/*if (angvel_diff > 0) {
		msg.angular.z = -0.3*angvel_diff;
		}
		
		if (angvel_diff < 0) {
		msg.angular.z = -0.3*angvel_diff;
		}*/

		msg.linear.x = 0.4;
		msg.angular.z = -0.03*angvel_diff;		// -0.03 works okay, just dont put it too close to the walls!		
		//msg.angular.z = -0.03*angvel_left;		// following left wall only

		pub_motor.publish(msg);		//pub to motor
		ROS_INFO(" msg.angular.z = %f", msg.angular.z);		

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
		0.0145,-1.864,116.8, false);
	sensors[1].calibrate(0,-7.107*std::pow(10, -12),1.306*std::pow(10, -8),
		-9.541*std::pow(10, -6),0.003542,-0.7071,72.41, false);
	sensors[2].calibrate(2.878*std::pow(10, -14),-6.602*std::pow(10, -11),
		6.098*std::pow(10, -8),-2.936*std::pow(10, -5),
		0.007948,-1.204,94.02, false);
	sensors[3].calibrate(0,-1.043*std::pow(10, -11),1.973*std::pow(10, -8),
		-1.461*std::pow(10, -5),0.00536,-1.014,91.89, false);
	sensors[4].calibrate(0,-4.235*std::pow(10, -11),7.098*std::pow(10, -8),
		-4.627*std::pow(10, -5),0.01486,-2.45,194.6, true);
	sensors[5].calibrate(0,-3.199*std::pow(10, -11),5.346*std::pow(10, -8),
		-3.489*std::pow(10, -5),0.0113,-1.91,161.8, true);
	
	pub_motor = handle.advertise<geometry_msgs::Twist>("/motor2/twist", 1000);
	sub_sensor = handle.subscribe("/arduino/adc", 1000, 			&wallfollower::sensorCallback, this);
	
	runNode();
}

int main(int argc, char *argv[]) 
{
    wallfollower wallfollower(argc, argv);
}
