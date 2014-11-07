#include "wallfollower.hpp"

/*void wallfollower:feedbackCallback(int argc, char *argv[]){

}*/

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

	double err_left = 0.0;
	double err_left_prev = 0.0;
	double err_right = 0.0;
	double err_right_prev = 0.0;

	double Pcontrol_left = 0.0;
	double Icontrol_left = 0.0;
	double Dcontrol_left = 0.0;
	double Pcontrol_right = 0.0;
	double Icontrol_right = 0.0;
	double Dcontrol_right = 0.0;

	//double Pcontrol_left_prev = 0.0;
	double Icontrol_left_prev = 0.0;
	//double Dcontrol_left_prev = 0.0;
	//double Pcontrol_right_prev = 0.0;
	double Icontrol_right_prev = 0.0;
	//double Dcontrol_right_prev = 0.0;

	double PIDcontrol_left = 0.0;
	//double PIDcontrol_left_prev = 0.0;
	double PIDcontrol_right = 0.0;
	//double PIDcontrol_right_prev = 0.0;

	ros::Rate loop_rate(10);	//10 Hz
	while (ros::ok())			//main loop of this code
	{
		/*work in progress*/
		

		// IMPORTANT NOTICE!
		// msg.angular.z > 0 -> turning LEFT
		// msg.angular.z < 0 -> turning RIGHT


		geometry_msgs::Twist msg;	//for controlling the motor
		

		// PID-Controller, regulating msg.angular.z

		angvel_left = sensors[0].get_distance() - sensors[2].get_distance();
		angvel_right = sensors[1].get_distance() - sensors[3].get_distance();


		// Target values of sensors from wall
		setpoint_left = 20.0;
		setpoint_right = 20.0;


		// Error between target value and measured value
		err_left = setpoint_left - angvel_left;
		err_right = setpoint_right - angvel_right;


		// Left sensors controller
		Pcontrol_left = GP_left*err_left;
		Icontrol_left = Icontrol_left_prev + PIDcontrol_time*GI_left*err_left; //+ (Gcontr_left/GP_left)*(
		Dcontrol_left = (GD_left/PIDcontrol_time)*(err_left - err_left_prev);
		PIDcontrol_left = Pcontrol_left + Icontrol_left + Dcontrol_left;


		// Right sensors controller
		Pcontrol_right = GP_right*err_right;
		Icontrol_right = Icontrol_right_prev + PIDcontrol_time*GI_right*err_right; 
		Dcontrol_right = (GD_right/PIDcontrol_time)*(err_right - err_right_prev);
		PIDcontrol_right = Pcontrol_right + Icontrol_right + Dcontrol_right;


		// Define new prev values
		err_left_prev = err_left;
		Icontrol_left_prev = Icontrol_left;
		//PIDcontrol_left_prev = PIDcontrol_left;

		err_right_prev = err_right;
		Icontrol_right_prev = Icontrol_right;
		//PIDcontrol_right_prev = PIDcontrol_right;
		
		
		ROS_INFO(" angvel_left= %f", angvel_left);
		ROS_INFO(" angvel_right= %f", angvel_right);
						
		msg.linear.x = 0.4;
		msg.angular.z = PIDcontrol_left;		

		pub_motor.publish(msg);		//pub to motor
		ROS_INFO(" msg.angular.z = %f", msg.angular.z);		

		ros::spinOnce();
		loop_rate.sleep();
	}
}

wallfollower::wallfollower(int argc, char *argv[]){
	ros::init(argc, argv, "wallfollower");	//name of node
	ros::NodeHandle handle;					//the handle
	angvel_left = 0.0;
	angvel_right = 0.0;

	/*setup the sensor calibration*/
	sensors[0].calibrate(-2.575*std::pow(10, -5), 0.002731, -0.1108,
		2.079, -15.55, -25.63, 871.1, false);
	sensors[1].calibrate(0, -0.0001057, 0.01266, -0.6095, 14.97, 
		-192.1, 1198, false);
	sensors[2].calibrate(-9.161*std::pow(10, -5), 0.008392, -0.3012,
		5.273, -43.75, 98.26, 662, false);
	sensors[3].calibrate(-1.215*std::pow(10, -5), 0.00113, -0.03767,
		0.4372, 3.311, -126.2, 1069, false);
	sensors[4].calibrate(0, -1.741*std::pow(10, -6), 0.0004776, -0.05162,
		2.786, -78.43, 1084, true);
	sensors[5].calibrate(0, -1.859*std::pow(10, -6), 0.0005059, -0.0542,
		2.898, -80.75, 1096, true);
	
	pub_motor = handle.advertise<geometry_msgs::Twist>("/motor2/twist", 1000);
	sub_sensor = handle.subscribe("/arduino/adc", 1000, 			&wallfollower::sensorCallback, this);
	
	ROSUtil::getParam(handle, "/controllerwf/GP_left", GP_left);
	ROSUtil::getParam(handle, "/controllerwf/GI_left", GI_left);
    	ROSUtil::getParam(handle, "/controllerwf/GD_left", GD_left);
	ROSUtil::getParam(handle, "/controllerwf/Gcontr_left", Gcontr_left);
    	ROSUtil::getParam(handle, "/controllerwf/GP_right", GP_right);
    	ROSUtil::getParam(handle, "/controllerwf/GI_right", GI_right);
    	ROSUtil::getParam(handle, "/controllerwf/GD_right", GD_right);
    	ROSUtil::getParam(handle, "/controllerwf/Gcontr_right", Gcontr_right);
    	ROSUtil::getParam(handle, "/controllerwf/PIDcontrol_freq", PIDcontrol_freq);
    	ROSUtil::getParam(handle, "/controllerwf/PIDcontrol_time", PIDcontrol_time);
    	
	runNode();
}

int main(int argc, char *argv[]) 
{
    wallfollower wallfollower(argc, argv);
}
