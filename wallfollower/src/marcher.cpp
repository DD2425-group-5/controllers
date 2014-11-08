#include "marcher.hpp"
#include "rosutil/rosutil.hpp"
#include <math.h>

void marcher::encoderCallback(const ras_arduino_msgs::Encoders encoderValues){
	currentTotalEncoderFeedbackL = encoderValues.encoder1;
	currentTotalEncoderFeedbackR = encoderValues.encoder2;
	//debug info	
	ROS_INFO("totalEncoderFeedbackL: %d, totalEncoderFeedbackR: %d \n\n",\
	currentTotalEncoderFeedbackL,\
	currentTotalEncoderFeedbackR);
}

void marcher::runNode(){
ros::Rate loop_rate(10);		//10 Hz
	float reference=0.0;
	float feedback = 0.0;
	float proportionalGain = 0.5;
	float marchDistance = 150; //mm
	w = 0.0;
	
	geometry_msgs::Twist marchControl;
	initialTotalEncoderFeedbackL = currentTotalEncoderFeedbackL;
	initialTotalEncoderFeedbackR = currentTotalEncoderFeedbackR;
	
while (ros::ok())			//main loop of this code
	{
		/*work in progress*/
		


	/*  P_controller  */
		relativeTotalEncoderFeedbackL = initialTotalEncoderFeedbackL - currentTotalEncoderFeedbackL; //relative to the begining moment;
		relativeTotalEncoderFeedbackR = initialTotalEncoderFeedbackR - currentTotalEncoderFeedbackR;
	
		feedback = (-1)*(relativeTotalEncoderFeedbackL - relativeTotalEncoderFeedbackR)/2; //average
		reference = ( marchDistance* 180/ wheelRadius / M_PI);
		v = proportionalGain * (reference - feedback);
	/* /P_controller   */
		
		marchControl.linear.x = v; 
		marchControl.angular.z = w; //w = 0.0 because marching;

		pub_march_control.publish(marchControl);		//pub to motor
		ROS_INFO(" msg.angular.z = %f", marchControl.angular.z);		

		ros::spinOnce();
		loop_rate.sleep();
	}
}


marcher::marcher(int argc, char *argv[]){                         // CONSTRUCTOR
	ros::init(argc, argv, "marcher");			//name of node
	ros::NodeHandle handle;
					//the handle
	ROSUtil::getParam(handle, "robot_info/wheel_baseline", robotBase);
	ROSUtil::getParam(handle, "robot_info/wheel_radius", wheelRadius);
	ROSUtil::getParam(handle, "robot_info/ticks_per_rev", ticksPerRev);
	v = 0.0;
	w = 0.0;
	turn = 0;
	state=0;
	
	pub_march_control = handle.advertise<geometry_msgs::Twist>("/motor2/twist", 1000);
	sub_encoder = handle.subscribe("/arduino/encoders", 1000, &marcher::encoderCallback, this);
	
	runNode();
}

int main(int argc, char *argv[]) 
{
    marcher marcher(argc, argv);
}
